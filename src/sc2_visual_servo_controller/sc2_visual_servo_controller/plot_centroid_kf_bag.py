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
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
    body_frame_id: str,
    camera_optical_frame_id: str,
):
    reader = _open_bag(bag_path)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    depth_topic = {
        'target_point': target_point_topic,
        'dvl': dvl_topic,
        'mavros': mavros_altitude_topic,
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
    t_first: Optional[float] = None
    t_last: Optional[float] = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        t_first = t if t_first is None else min(t_first, t)
        t_last = t if t_last is None else max(t_last, t)

        if topic == bbox_topic:
            msg = deserialize_message(data, Detection2D)
            detections.append(DetectionEvent(
                t=t,
                u=float(msg.bbox.center.position.x),
                v=float(msg.bbox.center.position.y),
            ))
        elif topic == body_velocity_topic:
            msg = deserialize_message(data, Odometry)
            twist = msg.twist.twist
            body_velocities.append(BodyVelocityEvent(
                t=t,
                v_body=np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
                w_body=np.array([twist.angular.x, twist.angular.y, twist.angular.z]),
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

    return detections, body_velocities, depths, r_cam_body, R_cam_body, t_first, t_last


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

    fig, (ax_u, ax_v) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

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

    ax_v.set_xlabel('time (s, bag-relative)')
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


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the rosbag2 recording directory')
    parser.add_argument('--out', default='centroid_kf.png', help='Output plot path (PNG)')
    parser.add_argument('--bbox-topic', default='/best_detection')
    parser.add_argument('--body-velocity-topic', default='/sc2/odometry_ekf')
    parser.add_argument('--depth-source', choices=['target_point', 'dvl', 'mavros'], default='target_point')
    parser.add_argument('--target-point-topic', default='/sc2/visual_servo/target_point')
    parser.add_argument('--dvl-topic', default='/waterlinked_dvl_driver/velocity_report')
    parser.add_argument('--mavros-altitude-topic', default='/mavros/global_position/rel_alt')
    parser.add_argument('--body-frame-id', default='sc2_base_link')
    parser.add_argument('--camera-optical-frame-id', default='oak_rgb_camera_optical_frame')
    parser.add_argument('--tick-hz', type=float, default=60.0)
    parser.add_argument('--body-velocity-timeout-s', type=float, default=0.5)
    parser.add_argument('--depth-timeout-s', type=float, default=0.5)
    parser.add_argument('--clear-on-loss-after-s', type=float, default=None)
    parser.add_argument('--fx', type=float, default=600.0)
    parser.add_argument('--fy', type=float, default=600.0)
    parser.add_argument('--cx', type=float, default=640.0)
    parser.add_argument('--cy', type=float, default=360.0)
    parser.add_argument('--focal-refraction-scale', type=float, default=1.0)
    parser.add_argument('--process-noise-px2-per-s', type=float, default=4.0)
    parser.add_argument('--measurement-noise-px2', type=float, default=25.0)
    parser.add_argument(
        '--gap-shade-threshold-s', type=float, default=0.5,
        help='Shade plot regions where time since the last accepted detection exceeds this.',
    )
    args = parser.parse_args(argv)

    detections, body_velocities, depths, r_cam_body, R_cam_body, t_first, t_last = _read_bag_events(
        bag_path=args.bag,
        bbox_topic=args.bbox_topic,
        body_velocity_topic=args.body_velocity_topic,
        depth_source=args.depth_source,
        target_point_topic=args.target_point_topic,
        dvl_topic=args.dvl_topic,
        mavros_altitude_topic=args.mavros_altitude_topic,
        body_frame_id=args.body_frame_id,
        camera_optical_frame_id=args.camera_optical_frame_id,
    )
    print(
        f'Read {len(detections)} detections, {len(body_velocities)} body-velocity samples, '
        f'{len(depths)} depth samples from {args.bag}'
    )
    if not detections:
        print('No detections found on the bbox topic -- nothing to plot.', file=sys.stderr)
        sys.exit(1)

    config = ReplayConfig(
        tick_hz=args.tick_hz,
        body_velocity_timeout_s=args.body_velocity_timeout_s,
        depth_timeout_s=args.depth_timeout_s,
        clear_on_loss_after_s=args.clear_on_loss_after_s,
        fx=args.fx, fy=args.fy, cx=args.cx, cy=args.cy,
        focal_refraction_scale=args.focal_refraction_scale,
        r_cam_body=r_cam_body,
        R_cam_body=R_cam_body,
        kf_config=CentroidKFConfig(
            process_noise_px2_per_s=args.process_noise_px2_per_s,
            measurement_noise_px2=args.measurement_noise_px2,
        ),
    )
    samples = replay_centroid_kf(
        detections, body_velocities, depths, config, t_start=t_first, t_end=t_last,
    )
    _plot(samples, args.out, args.gap_shade_threshold_s)


if __name__ == '__main__':
    main()
