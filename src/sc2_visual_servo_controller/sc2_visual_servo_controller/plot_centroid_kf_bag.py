'''Offline tool: replay a recorded ROS bag through the same centroid Kalman
filter logic the live node uses, and plot the filtered centroid trajectory
against the raw detector centroid -- to check the filter holds a valid
target estimate through detector dropouts.

Usage:
    ros2 run sc2_visual_servo_controller plot_centroid_kf_bag \\
        --bag /path/to/bag_dir --out centroid_kf.png

The body->camera extrinsic is read from the bag's own /tf_static (and /tf)
messages, matching whatever was actually running at record time, rather than
hardcoded -- see sc2_base_link -> oak-d-base-frame in perception.launch.py /
the transform_tree package for why that matters.
'''
import argparse
import sys
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D

from .ibvs.centroid_kalman_filter import CentroidKFConfig
from .ibvs.centroid_kf_replay import (
    BodyVelocityEvent,
    DepthEvent,
    DetectionEvent,
    ReplayConfig,
    ReplaySample,
    replay_centroid_kf,
)
from .ibvs.image_jacobian import quaternion_to_rotation_matrix

try:
    from marine_acoustic_msgs.msg import Dvl
    _HAS_DVL_MSG = True
except ImportError:
    Dvl = None
    _HAS_DVL_MSG = False

try:
    # sc2_interfaces/msg/SimpleDistance -- assumed (not independently
    # verifiable from this checkout; the sc2_interfaces submodule isn't
    # checked out here) to mirror state_estimation/msg/SimpleDistance's
    # schema exactly, given every other sc2_interfaces type we've found
    # (e.g. Vector6) is a straight rename of a state_estimation type with
    # the same fields: `uint32 distance` in millimeters, `uint8 confidence`.
    # Verify with `ros2 interface show sc2_interfaces/msg/SimpleDistance`
    # before trusting --depth-source sc2_altitude results.
    from sc2_interfaces.msg import SimpleDistance
    _HAS_SC2_INTERFACES = True
except ImportError:
    SimpleDistance = None
    _HAS_SC2_INTERFACES = False


def _open_bag(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    return reader


def _read_bag_events(
    bag_path: str,
    bbox_topic: str,
    body_velocity_topic: str,
    depth_source: str,
    target_point_topic: str,
    dvl_topic: str,
    mavros_altitude_topic: str,
    sc2_altitude_topic: str,
    body_frame_id: str,
    camera_optical_frame_id: str,
    camera_info_topic: Optional[str] = None,
    extra_static_transforms: Optional[List['TransformStamped']] = None,
    drop_detection_windows: Optional[List[Tuple[float, float]]] = None,
    drop_detections_every_nth: int = 1,
):
    reader = _open_bag(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    depth_topic = {
        'target_point': target_point_topic,
        'dvl': dvl_topic,
        'mavros': mavros_altitude_topic,
        'sc2_altitude': sc2_altitude_topic,
    }[depth_source]

    wanted = {bbox_topic, body_velocity_topic, depth_topic, '/tf', '/tf_static'}
    missing = wanted - set(topic_types.keys())
    if bbox_topic in missing or body_velocity_topic in missing:
        print(
            f'WARNING: expected topic(s) not found in bag: '
            f'{sorted((missing - {"/tf", "/tf_static"}) & wanted)}. '
            f'Available topics: {sorted(topic_types.keys())}',
            file=sys.stderr,
        )

    detections: List[DetectionEvent] = []
    body_velocities: List[BodyVelocityEvent] = []
    depths: List[DepthEvent] = []
    tf_buffer = Buffer()
    # Injected before any bag /tf_static so a bag's own recorded transforms
    # (if any) can still override it -- useful when the bag's own camera
    # frame uses a different axis convention than the target
    # camera_optical_frame_id and you want to bridge the two without
    # re-recording (e.g. a sim rig's camera link vs. the real vehicle's
    # optical-frame convention).
    for transform in (extra_static_transforms or []):
        tf_buffer.set_transform_static(transform, 'extra_static_transform')
    t_first: Optional[float] = None
    t_last: Optional[float] = None
    detection_count = 0
    dropped_count = 0
    # (fx, fy, cx, cy) from the bag's own CameraInfo, if requested and found
    # -- the real calibration for whatever resolution the sim/detector
    # actually ran at, rather than a guessed/hardcoded value.
    camera_info_intrinsics: Optional[Tuple[float, float, float, float]] = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        t_first = t if t_first is None else min(t_first, t)
        t_last = t if t_last is None else max(t_last, t)

        if topic == bbox_topic:
            detection_count += 1
            # t_first is stable from the very first message onward (bag
            # messages are read in overall chronological order), so this is
            # safe to compute in the same single pass rather than needing
            # the final t_first known only after the whole file is read.
            t_relative = t - t_first
            dropped = False
            if drop_detections_every_nth > 1 and (detection_count % drop_detections_every_nth) != 0:
                dropped = True
            if not dropped:
                for start, end in (drop_detection_windows or []):
                    if start <= t_relative < end:
                        dropped = True
                        break
            if dropped:
                dropped_count += 1
            else:
                msg = deserialize_message(data, Detection2D)
                detections.append(DetectionEvent(
                    t=t,
                    u=float(msg.bbox.center.position.x),
                    v=float(msg.bbox.center.position.y),
                ))
        elif topic == body_velocity_topic:
            msg = deserialize_message(data, Odometry)
            twist_with_cov = msg.twist
            twist = twist_with_cov.twist
            body_velocities.append(BodyVelocityEvent(
                t=t,
                v_body=np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
                w_body=np.array([twist.angular.x, twist.angular.y, twist.angular.z]),
                covariance=np.array(twist_with_cov.covariance).reshape(6, 6),
            ))
        elif topic == depth_topic:
            if depth_source == 'target_point':
                msg = deserialize_message(data, PoseStamped)
                depths.append(DepthEvent(t=t, z=float(msg.pose.position.z)))
            elif depth_source == 'dvl' and _HAS_DVL_MSG:
                msg = deserialize_message(data, Dvl)
                depths.append(DepthEvent(t=t, z=float(msg.altitude)))
            elif depth_source == 'mavros':
                msg = deserialize_message(data, Float64)
                depths.append(DepthEvent(t=t, z=abs(float(msg.data))))
            elif depth_source == 'sc2_altitude' and _HAS_SC2_INTERFACES:
                msg = deserialize_message(data, SimpleDistance)
                depths.append(DepthEvent(t=t, z=float(msg.distance) * 1e-3))
        elif camera_info_topic and topic == camera_info_topic and camera_info_intrinsics is None:
            msg = deserialize_message(data, CameraInfo)
            k = msg.k
            camera_info_intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
        elif topic == '/tf_static':
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, 'bag_replay')
        elif topic == '/tf':
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                tf_buffer.set_transform(transform, 'bag_replay')

    r_cam_body = np.zeros(3)
    R_cam_body = np.eye(3)
    try:
        transform = tf_buffer.lookup_transform(
            body_frame_id, camera_optical_frame_id, rclpy.time.Time(),
        )
        t_xyz = transform.transform.translation
        q = transform.transform.rotation
        r_cam_body = np.array([t_xyz.x, t_xyz.y, t_xyz.z])
        R_cam_body = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w).T
        print(f'Resolved extrinsic from bag TF: r_cam_body={r_cam_body}, R_cam_body=\n{R_cam_body}')
    except TransformException as ex:
        print(
            f'WARNING: could not resolve {body_frame_id} -> {camera_optical_frame_id} '
            f'from the bag\'s /tf and /tf_static: {ex}. Falling back to zero lever arm '
            f'and identity rotation -- the Jacobian prediction will be wrong until this '
            f'is fixed (pass --body-frame-id/--camera-optical-frame-id, or check the bag '
            f'actually recorded tf).',
            file=sys.stderr,
        )

    if dropped_count:
        print(
            f'Artificially dropped {dropped_count}/{detection_count} detections '
            f'({100.0 * dropped_count / max(1, detection_count):.1f}%) per '
            f'--drop-detections-* options.'
        )

    if camera_info_topic and camera_info_intrinsics is None:
        print(
            f'WARNING: --camera-info-topic {camera_info_topic!r} requested but no '
            f'message found in the bag -- falling back to --fx/--fy/--cx/--cy.',
            file=sys.stderr,
        )

    return (
        detections, body_velocities, depths, r_cam_body, R_cam_body, t_first, t_last,
        camera_info_intrinsics,
    )


def _parse_extra_static_transform_args(values_list) -> List[TransformStamped]:
    '''values_list: a list of 9-token lists, each
    [parent, child, x, y, z, qx, qy, qz, qw] (as produced by
    argparse(..., nargs=9, action='append')). Lets a bag-reading tool bridge
    a frame the bag doesn't define correctly (e.g. a sim rig's camera link
    using a different axis convention than the target
    camera_optical_frame_id) without re-recording.
    '''
    transforms = []
    for values in (values_list or []):
        parent, child, x, y, z, qx, qy, qz, qw = values
        tr = TransformStamped()
        tr.header.frame_id = parent
        tr.child_frame_id = child
        tr.transform.translation.x = float(x)
        tr.transform.translation.y = float(y)
        tr.transform.translation.z = float(z)
        tr.transform.rotation.x = float(qx)
        tr.transform.rotation.y = float(qy)
        tr.transform.rotation.z = float(qz)
        tr.transform.rotation.w = float(qw)
        transforms.append(tr)
    return transforms


def _add_camera_info_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '--camera-info-topic', default='/oak/rgb/camera_info',
        help=(
            'Read fx/fy/cx/cy from this topic\'s CameraInfo.k in the bag, if '
            'present -- the real calibration for whatever resolution the '
            'sim/detector actually ran at. --fx/--fy/--cx/--cy override this '
            'per-value when explicitly given; pass an empty string to disable '
            'and always use --fx/--fy/--cx/--cy (falling back to 600/600/640/360 '
            'for whichever of those isn\'t given either).'
        ),
    )
    parser.add_argument('--fx', type=float, default=None)
    parser.add_argument('--fy', type=float, default=None)
    parser.add_argument('--cx', type=float, default=None)
    parser.add_argument('--cy', type=float, default=None)


def _resolve_intrinsics(args, camera_info_intrinsics: Optional[Tuple[float, float, float, float]]):
    '''Priority per value: explicit --fx/--fy/--cx/--cy > the bag's own
    CameraInfo (if --camera-info-topic found one) > a hardcoded fallback.
    '''
    fallback = (600.0, 600.0, 640.0, 360.0)
    bag_values = camera_info_intrinsics if camera_info_intrinsics is not None else (None,) * 4
    cli_values = (args.fx, args.fy, args.cx, args.cy)
    names = ('fx', 'fy', 'cx', 'cy')
    resolved = []
    sources = []
    for name, cli_v, bag_v, fallback_v in zip(names, cli_values, bag_values, fallback):
        if cli_v is not None:
            resolved.append(cli_v)
            sources.append('cli')
        elif bag_v is not None:
            resolved.append(bag_v)
            sources.append('bag')
        else:
            print(
                f'WARNING: no --{name} given and no CameraInfo value available -- '
                f'using fallback {name}={fallback_v}.',
                file=sys.stderr,
            )
            resolved.append(fallback_v)
            sources.append('fallback')
    print(
        'Using intrinsics: ' + ', '.join(
            f'{name}={value:.2f} ({source})'
            for name, value, source in zip(names, resolved, sources)
        )
    )
    return tuple(resolved)


def _add_extra_static_transform_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '--extra-static-transform', nargs=9, action='append', metavar=(
            'PARENT', 'CHILD', 'X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW',
        ),
        help=(
            'Inject an additional static transform (translation + quaternion) '
            'not present in the bag, before resolving the extrinsic -- e.g. to '
            'bridge a sim rig\'s differently-oriented camera frame to '
            '--camera-optical-frame-id. May be given multiple times. A bag\'s '
            'own recorded transforms still take precedence if they overlap.'
        ),
    )


def _add_drop_detections_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '--drop-detections-window', nargs=2, type=float, action='append',
        metavar=('START_S', 'END_S'),
        help=(
            'Artificially discard detections in [START_S, END_S) -- seconds '
            'relative to the bag\'s first message -- to carve out a deliberate '
            'gap and see how well the Jacobian/body-velocity prediction holds '
            'up with no measurement update at all. May be given multiple times.'
        ),
    )
    parser.add_argument(
        '--drop-detections-every-nth', type=int, default=1,
        help=(
            'Keep only 1 out of every N detections (N=1 keeps all) -- thins '
            'the detection rate uniformly rather than carving out one gap.'
        ),
    )


def _plot(samples: List[ReplaySample], out_path: str, gap_shade_threshold_s: float) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ts = np.array([s.t for s in samples])
    t0 = ts[0] if len(ts) else 0.0
    ts = ts - t0

    raw_mask = np.array([s.has_detection for s in samples])
    raw_u = np.array([s.raw_u if s.raw_u is not None else np.nan for s in samples])
    raw_v = np.array([s.raw_v if s.raw_v is not None else np.nan for s in samples])
    filt_u = np.array([s.filtered_u if s.filtered_u is not None else np.nan for s in samples])
    filt_v = np.array([s.filtered_v if s.filtered_v is not None else np.nan for s in samples])
    has_target = ~np.isnan(filt_u)
    # Shade only sustained dropouts (gap_since_last_detection_s beyond the
    # threshold), not every individual inter-tick gap between detections
    # arriving slower than tick_hz -- that would shade almost the whole plot.
    in_dropout = np.array([
        (s.gap_since_last_detection_s is None) or (s.gap_since_last_detection_s > gap_shade_threshold_s)
        for s in samples
    ])

    z_used = np.array([s.z_used if s.z_used is not None else np.nan for s in samples])
    has_z = ~np.isnan(z_used)

    fig, (ax_u, ax_v, ax_z) = plt.subplots(3, 1, sharex=True, figsize=(12, 9))

    for ax, filt, raw, label in ((ax_u, filt_u, raw_u, 'u (px)'), (ax_v, filt_v, raw_v, 'v (px)')):
        # Shade spans with a sustained dropout so the gaps the filter is
        # bridging are visually obvious against the continuous filtered line.
        in_gap = False
        gap_start = 0.0
        for i in range(len(ts)):
            if in_dropout[i] and not in_gap:
                in_gap = True
                gap_start = ts[i]
            elif not in_dropout[i] and in_gap:
                in_gap = False
                ax.axvspan(gap_start, ts[i], color='0.9', zorder=0)
        if in_gap:
            ax.axvspan(gap_start, ts[-1], color='0.9', zorder=0)

        ax.plot(ts, filt, '-', color='C1', linewidth=1.5, label='KF filtered', zorder=2)
        ax.plot(ts[raw_mask], raw[raw_mask], '.', color='C0', markersize=4, label='raw detection', zorder=3)
        ax.set_ylabel(label)
        ax.legend(loc='upper right', fontsize='small')
        ax.grid(True, alpha=0.3)

    ax_z.plot(ts[has_z], z_used[has_z], '-', color='C2', linewidth=1.5)
    ax_z.set_ylabel('Z used (m)')
    ax_z.grid(True, alpha=0.3)
    ax_z.set_xlabel('time (s, bag-relative)')
    fig.suptitle(f'Centroid Kalman filter vs. raw detection (shaded = dropout > {gap_shade_threshold_s:.2f}s)')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f'Saved plot to {out_path}')

    n = len(samples)
    n_with_target = int(np.sum(has_target))
    gaps = [s.gap_since_last_detection_s for s in samples if s.gap_since_last_detection_s is not None]
    longest_gap = max(gaps) if gaps else float('nan')
    n_in_dropout = int(np.sum(in_dropout))
    print(f'{n} samples over {ts[-1]:.1f}s, {n_with_target} with a valid KF estimate '
          f'({100.0 * n_with_target / max(1, n):.1f}%), '
          f'{n_in_dropout} samples in a sustained dropout (>{gap_shade_threshold_s:.2f}s since last detection, '
          f'{100.0 * n_in_dropout / max(1, n):.1f}%), longest gap since a detection: {longest_gap:.2f}s.')


def _print_jacobian_debug(samples: List[ReplaySample], n: int) -> None:
    '''Dump the raw intermediate Jacobian computation for up to n ticks that
    actually fired, spread evenly through the run -- for reading off real
    numbers (v_cam/w_cam, normalized position, pre/post swap_uv output)
    instead of assuming which interaction-matrix term dominates.
    '''
    with_debug = [s for s in samples if s.jacobian_debug is not None]
    if not with_debug:
        print('No ticks had a Jacobian prediction (KF never initialized with fresh '
              'velocity+depth) -- nothing to print.', file=sys.stderr)
        return

    t0 = samples[0].t
    stride = max(1, len(with_debug) // n)
    print(f'{len(with_debug)} ticks had a Jacobian prediction; showing every {stride}th:')
    print(
        f'{"t":>7} {"u,v":>15} {"x_norm,y_norm":>17} {"v_cam":>24} {"w_cam":>24} '
        f'{"du_std,dv_std":>19} {"du,dv (final)":>19} {"raw_u,raw_v":>15}'
    )
    for s in with_debug[::stride]:
        info = s.jacobian_debug
        raw = f'{s.raw_u:.1f},{s.raw_v:.1f}' if s.raw_u is not None else '-'
        print(
            f'{s.t - t0:7.2f} '
            f'{s.filtered_u:7.1f},{s.filtered_v:6.1f} '
            f'{info.x_norm:8.4f},{info.y_norm:7.4f} '
            f'{info.v_cam[0]:7.3f},{info.v_cam[1]:7.3f},{info.v_cam[2]:7.3f} '
            f'{info.w_cam[0]:7.3f},{info.w_cam[1]:7.3f},{info.w_cam[2]:7.3f} '
            f'{info.du_std:8.2f},{info.dv_std:8.2f} '
            f'{info.du_final:8.2f},{info.dv_final:8.2f} '
            f'{raw:>15}'
        )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the rosbag2 recording directory')
    parser.add_argument('--out', default='centroid_kf.png', help='Output plot path (PNG)')
    parser.add_argument('--bbox-topic', default='/best_detection')
    parser.add_argument('--body-velocity-topic', default='/sc2/odometry_ekf')
    parser.add_argument(
        '--depth-source', choices=['target_point', 'dvl', 'mavros', 'sc2_altitude'],
        default='target_point',
    )
    parser.add_argument('--target-point-topic', default='/sc2/visual_servo/target_point')
    parser.add_argument('--dvl-topic', default='/waterlinked_dvl_driver/velocity_report')
    parser.add_argument('--mavros-altitude-topic', default='/mavros/global_position/rel_alt')
    parser.add_argument('--sc2-altitude-topic', default='/sc2/altitude')
    parser.add_argument('--body-frame-id', default='sc2_base_link')
    parser.add_argument('--camera-optical-frame-id', default='oak_rgb_camera_optical_frame')
    _add_extra_static_transform_arg(parser)
    _add_drop_detections_args(parser)
    parser.add_argument('--tick-hz', type=float, default=60.0)
    parser.add_argument('--body-velocity-timeout-s', type=float, default=0.5)
    parser.add_argument('--depth-timeout-s', type=float, default=0.5)
    parser.add_argument('--clear-on-loss-after-s', type=float, default=None)
    _add_camera_info_args(parser)
    parser.add_argument('--focal-refraction-scale', type=float, default=1.0)
    parser.add_argument(
        '--swap-uv', action='store_true',
        help=(
            'The detector reports u as the vertical axis and v as the '
            'horizontal one (not the standard u=horizontal/v=vertical '
            'pinhole convention) -- swap u/v, cx/cy, and fx/fy before '
            'evaluating the Jacobian, and swap the (du, dv) result back.'
        ),
    )
    parser.add_argument(
        '--override-z-m', type=float, default=None,
        help=(
            'Use this fixed range (meters) for every Jacobian prediction '
            'instead of whatever the depth source reports -- bypasses depth '
            'entirely (works with no depth topic present at all). Since '
            'du/dv scale as 1/Z, use this to isolate whether the depth '
            'source\'s value, not the geometry, explains a magnitude mismatch '
            'against the raw detections. Takes precedence over '
            '--z-scale-percent if both are given.'
        ),
    )
    parser.add_argument(
        '--z-scale-percent', type=float, default=100.0,
        help=(
            'Scale whatever the depth source reports by this percent (100 = '
            'no change, 150 = use 1.5x the reported range) -- unlike '
            '--override-z-m, this preserves the depth signal\'s own real '
            'variation/timing, just scaled. E.g. test "target range is 150%% '
            'of altitude" without flattening it to a constant.'
        ),
    )
    parser.add_argument('--process-noise-px2-per-s', type=float, default=4.0)
    parser.add_argument('--measurement-noise-px2', type=float, default=25.0)
    parser.add_argument(
        '--no-velocity-covariance', action='store_true',
        help=(
            'Ignore any twist covariance recorded on the body-velocity topic '
            '(e.g. from an EKF) and use only the flat process-noise floor, '
            'matching pre-covariance-aware behavior.'
        ),
    )
    parser.add_argument(
        '--gap-shade-threshold-s', type=float, default=0.5,
        help='Shade plot regions where time since the last accepted detection exceeds this.',
    )
    parser.add_argument(
        '--print-jacobian-debug', type=int, default=0, metavar='N',
        help=(
            'Print the raw intermediate Jacobian computation (v_cam, w_cam, '
            'normalized position, pre/post swap_uv output) for up to N ticks '
            'spread through the run, instead of assuming which term dominates.'
        ),
    )
    args = parser.parse_args(argv)

    detections, body_velocities, depths, r_cam_body, R_cam_body, t_first, t_last, camera_info_intrinsics = _read_bag_events(
        bag_path=args.bag,
        bbox_topic=args.bbox_topic,
        body_velocity_topic=args.body_velocity_topic,
        depth_source=args.depth_source,
        target_point_topic=args.target_point_topic,
        dvl_topic=args.dvl_topic,
        mavros_altitude_topic=args.mavros_altitude_topic,
        sc2_altitude_topic=args.sc2_altitude_topic,
        body_frame_id=args.body_frame_id,
        camera_optical_frame_id=args.camera_optical_frame_id,
        camera_info_topic=(args.camera_info_topic or None),
        extra_static_transforms=_parse_extra_static_transform_args(args.extra_static_transform),
        drop_detection_windows=args.drop_detections_window,
        drop_detections_every_nth=args.drop_detections_every_nth,
    )
    print(
        f'Read {len(detections)} detections, {len(body_velocities)} body-velocity samples, '
        f'{len(depths)} depth samples from {args.bag}'
    )
    if not detections:
        print('No detections found on the bbox topic -- nothing to plot.', file=sys.stderr)
        sys.exit(1)

    fx, fy, cx, cy = _resolve_intrinsics(args, camera_info_intrinsics)
    config = ReplayConfig(
        tick_hz=args.tick_hz,
        body_velocity_timeout_s=args.body_velocity_timeout_s,
        depth_timeout_s=args.depth_timeout_s,
        clear_on_loss_after_s=args.clear_on_loss_after_s,
        fx=fx, fy=fy, cx=cx, cy=cy,
        focal_refraction_scale=args.focal_refraction_scale,
        swap_uv=args.swap_uv,
        override_z=args.override_z_m,
        z_scale_factor=args.z_scale_percent / 100.0,
        r_cam_body=r_cam_body,
        R_cam_body=R_cam_body,
        use_velocity_covariance=not args.no_velocity_covariance,
        kf_config=CentroidKFConfig(
            process_noise_px2_per_s=args.process_noise_px2_per_s,
            measurement_noise_px2=args.measurement_noise_px2,
        ),
    )
    samples = replay_centroid_kf(
        detections, body_velocities, depths, config, t_start=t_first, t_end=t_last,
    )
    if args.print_jacobian_debug > 0:
        _print_jacobian_debug(samples, args.print_jacobian_debug)
    _plot(samples, args.out, args.gap_shade_threshold_s)


if __name__ == '__main__':
    main()
