from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

from .centroid_kalman_filter import CentroidKFConfig, CentroidKalmanFilter
from .controller_math import is_in_roi
from .image_jacobian import (
    JacobianDebugInfo,
    apply_refraction_scale,
    is_valid_twist_covariance,
    predict_pixel_centroid_velocity_debug,
    predict_pixel_velocity_covariance,
)


@dataclass
class DetectionEvent:
    t: float  # seconds, bag time
    u: float
    v: float


@dataclass
class BodyVelocityEvent:
    t: float
    v_body: np.ndarray  # (3,) body-frame linear velocity
    w_body: np.ndarray  # (3,) body-frame angular velocity
    # 6x6 twist covariance ([vx,vy,vz,wx,wy,wz]), e.g. from
    # nav_msgs/Odometry.twist.covariance -- None if the source doesn't
    # publish one (or it's being deliberately ignored).
    covariance: Optional[np.ndarray] = None


@dataclass
class DepthEvent:
    t: float
    z: float  # target range, meters


@dataclass
class ReplayConfig:
    '''Offline replica of the node's centroid-tracking behavior. Field names
    and defaults mirror the corresponding ROS parameters/config so this stays
    a faithful stand-in for what the live node would have done -- pass the
    node's actual runtime parameter values in here, don't just use defaults.
    '''
    tick_hz: float = 60.0
    body_velocity_timeout_s: float = 0.5
    depth_timeout_s: float = 0.5
    roi_width_px: float = 50.0
    debug_ignore_roi_consistency: bool = True
    # None = never clear on a gap, matching the live node's current behavior
    # (its target-loss reset is presently disabled -- see
    # _update_target_tracked_from_tick). Set a value to explore what
    # re-enabling that reset would look like.
    clear_on_loss_after_s: Optional[float] = None
    # get_image_jacobian assumes the standard pinhole convention (u increases
    # rightward, v increases downward). If a detector instead reports u as
    # the vertical axis and v as the horizontal one, the Jacobian's (du, dv)
    # output needs swapping before it's used as this detector's centroid
    # velocity, or the predicted motion lands on the wrong image axis
    # entirely (not just the wrong sign). Doesn't touch the raw detections
    # themselves -- those already are whatever the detector reports.
    swap_uv: bool = False
    # When set, use this fixed range (meters) for every Jacobian prediction
    # instead of whatever the depth source reports -- bypasses depth
    # freshness/validity entirely, so it works even with no depth topic at
    # all. Since du/dv scale as 1/Z, this isolates whether the depth
    # source's value (as opposed to the Jacobian geometry itself) is
    # responsible for a magnitude mismatch against raw detections.
    override_z: Optional[float] = None
    # Multiplies whatever the real depth source reports (ignored when
    # override_z is set, since that's already the intended final value) --
    # e.g. 1.5 to test "target range is 150% of what altitude reports",
    # without flattening out the depth signal's own real variation/timing
    # the way a fixed override_z would.
    z_scale_factor: float = 1.0
    fx: float = 600.0
    fy: float = 600.0
    cx: float = 640.0
    cy: float = 360.0
    focal_refraction_scale: float = 1.0
    r_cam_body: np.ndarray = field(default_factory=lambda: np.zeros(3))
    R_cam_body: np.ndarray = field(default_factory=lambda: np.eye(3))
    kf_config: CentroidKFConfig = field(default_factory=CentroidKFConfig)
    # When a BodyVelocityEvent carries a covariance, propagate it into the
    # KF's process noise instead of relying solely on the flat
    # kf_config.process_noise_px2_per_s floor. Set False to ignore any
    # recorded covariance and match pre-covariance-aware behavior exactly.
    use_velocity_covariance: bool = True


@dataclass
class ReplaySample:
    t: float
    raw_u: Optional[float]
    raw_v: Optional[float]
    has_detection: bool
    filtered_u: Optional[float]
    filtered_v: Optional[float]
    gap_since_last_detection_s: Optional[float]
    # The range (meters) actually fed to the Jacobian this tick -- after
    # override_z/z_scale_factor are applied -- or None if no depth was
    # available at all. Exposed so callers (e.g. annotate_centroid_kf_bag.py)
    # can write it out for inspection alongside the raw depth source.
    z_used: Optional[float] = None
    # Full intermediate breakdown of this tick's Jacobian prediction (v_cam,
    # w_cam, normalized position, pre/post swap_uv output), or None if the
    # Jacobian didn't fire this tick. See JacobianDebugInfo -- use this
    # instead of assuming which interaction-matrix term dominated.
    jacobian_debug: Optional[JacobianDebugInfo] = None


def replay_centroid_kf(
    detections: Sequence[DetectionEvent],
    body_velocities: Sequence[BodyVelocityEvent],
    depths: Sequence[DepthEvent],
    config: ReplayConfig,
    t_start: Optional[float] = None,
    t_end: Optional[float] = None,
) -> List[ReplaySample]:
    '''Re-run the centroid KF's predict/update loop offline over recorded
    events, at a fixed synthetic tick rate, mirroring how _tick() (predict)
    and _detection_cb() (update) interleave in the live node.

    t_start/t_end default to the earliest/latest event across the three
    streams, but pass the bag's actual recording start/end explicitly when
    replaying real data -- inferring the end from "last message on these
    topics" understates a real target-loss gap that runs to the end of the
    bag (exactly the case this tool exists to visualize).
    '''
    all_times = (
        [e.t for e in detections] + [e.t for e in body_velocities] + [e.t for e in depths]
    )
    if t_start is None or t_end is None:
        if not all_times:
            return []
        if t_start is None:
            t_start = min(all_times)
        if t_end is None:
            t_end = max(all_times)
    dt = 1.0 / config.tick_hz
    n_steps = int(round((t_end - t_start) / dt)) + 1

    kf = CentroidKalmanFilter(config.kf_config)
    fx, fy = apply_refraction_scale(config.fx, config.fy, config.focal_refraction_scale)

    det_by_time = sorted(detections, key=lambda e: e.t)
    vel_by_time = sorted(body_velocities, key=lambda e: e.t)
    depth_by_time = sorted(depths, key=lambda e: e.t)

    det_idx = vel_idx = depth_idx = 0
    latest_vel: Optional[BodyVelocityEvent] = None
    latest_depth: Optional[DepthEvent] = None
    last_accepted_centroid: Optional[np.ndarray] = None
    last_detection_time: Optional[float] = None

    samples: List[ReplaySample] = []
    for step in range(n_steps):
        # Computed fresh from step*dt each iteration (not accumulated via
        # repeated t += dt) so float rounding can't drift the last tick
        # under t_end and silently drop an event stamped exactly at t_end.
        t = t_start + step * dt
        while vel_idx < len(vel_by_time) and vel_by_time[vel_idx].t <= t:
            latest_vel = vel_by_time[vel_idx]
            vel_idx += 1
        while depth_idx < len(depth_by_time) and depth_by_time[depth_idx].t <= t:
            latest_depth = depth_by_time[depth_idx]
            depth_idx += 1

        raw_u = raw_v = None
        has_detection = False
        while det_idx < len(det_by_time) and det_by_time[det_idx].t <= t:
            d = det_by_time[det_idx]
            det_idx += 1
            raw_u, raw_v, has_detection = d.u, d.v, True
            centroid = np.array([d.u, d.v], dtype=np.float32)
            accepted = (
                config.debug_ignore_roi_consistency or
                last_accepted_centroid is None or
                is_in_roi(centroid[0], centroid[1], last_accepted_centroid, config.roi_width_px)
            )
            if accepted:
                last_accepted_centroid = centroid
                last_detection_time = d.t
                kf.update(d.u, d.v)

        if (
            config.clear_on_loss_after_s is not None and
            last_detection_time is not None and
            (t - last_detection_time) > config.clear_on_loss_after_s
        ):
            kf.clear()
            last_accepted_centroid = None

        du = dv = 0.0
        Q_dynamic = None
        jacobian_debug: Optional[JacobianDebugInfo] = None
        if config.override_z is not None:
            z_value = config.override_z
            z_fresh = True
        elif (
            latest_depth is not None and
            (t - latest_depth.t) <= config.depth_timeout_s and
            latest_depth.z > 0.0
        ):
            z_value = latest_depth.z * config.z_scale_factor
            z_fresh = True
        else:
            z_value = None
            z_fresh = False

        velocity_source_fresh = (
            kf.is_initialized and
            latest_vel is not None and (t - latest_vel.t) <= config.body_velocity_timeout_s and
            z_fresh
        )
        if velocity_source_fresh:
            u0, v0 = kf.state
            try:
                jacobian_debug = predict_pixel_centroid_velocity_debug(
                    u0, v0, z_value,
                    latest_vel.v_body, latest_vel.w_body,
                    config.r_cam_body, config.R_cam_body,
                    fx, fy, config.cx, config.cy,
                    swap_uv=config.swap_uv,
                )
                du, dv = jacobian_debug.du_final, jacobian_debug.dv_final
                if (
                    config.use_velocity_covariance and
                    latest_vel.covariance is not None and
                    is_valid_twist_covariance(latest_vel.covariance)
                ):
                    Q_dynamic = predict_pixel_velocity_covariance(
                        u0, v0, z_value,
                        latest_vel.covariance,
                        config.r_cam_body, config.R_cam_body,
                        fx, fy, config.cx, config.cy,
                        swap_uv=config.swap_uv,
                    )
            except ValueError:
                du = dv = 0.0
                jacobian_debug = None

        kf.predict(dt, du, dv, Q=Q_dynamic)

        state = kf.state
        samples.append(ReplaySample(
            t=t,
            raw_u=raw_u, raw_v=raw_v, has_detection=has_detection,
            filtered_u=(state[0] if state is not None else None),
            filtered_v=(state[1] if state is not None else None),
            gap_since_last_detection_s=(
                (t - last_detection_time) if last_detection_time is not None else None
            ),
            z_used=z_value,
            jacobian_debug=jacobian_debug,
        ))

    return samples
