'''Offline tool: annotate a recorded bag's mask/image frames with the
centroid Kalman filter's predicted centroid (orange) and, when one lands
close enough in time, the raw detected centroid (cyan) -- so the two can be
compared directly overlaid on the image in Foxglove, rather than as a
separate time-series plot (see plot_centroid_kf_bag.py for that).

Uses visualization_msgs/msg/ImageMarker (a core ROS2 message, not a
foxglove_msgs one) specifically so this doesn't depend on a package whose
exact schema can't be verified from outside the target environment --
Foxglove's Image panel supports ImageMarker overlays natively.

Writes a NEW bag containing every original topic unchanged, plus one new
ImageMarker topic with markers timestamped to match each mask/image frame.
Open the output bag in Foxglove, add an Image panel on the image topic, and
enable the marker topic in that panel's settings.

Usage:
    ros2 run sc2_visual_servo_controller annotate_centroid_kf_bag \\
        --bag /path/to/bag_dir --out /path/to/bag_dir_annotated
'''
import argparse
import bisect
import sys
from typing import List, Optional

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Float64
from visualization_msgs.msg import ImageMarker

from .ibvs.centroid_kalman_filter import CentroidKFConfig
from .ibvs.centroid_kf_replay import DetectionEvent, ReplayConfig, ReplaySample, replay_centroid_kf
from .plot_centroid_kf_bag import (
    _add_camera_info_args,
    _add_drop_detections_args,
    _add_extra_static_transform_arg,
    _open_bag,
    _parse_extra_static_transform_args,
    _read_bag_events,
    _resolve_intrinsics,
)

_PREDICTED_ID = 0
_RAW_ID = 1
_PREDICTED_COLOR = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)  # orange
_RAW_COLOR = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # cyan


def _nearest_sample(samples: List[ReplaySample], t: float) -> Optional[ReplaySample]:
    if not samples:
        return None
    ts = [s.t for s in samples]
    idx = bisect.bisect_left(ts, t)
    candidates = []
    if idx < len(samples):
        candidates.append(samples[idx])
    if idx > 0:
        candidates.append(samples[idx - 1])
    return min(candidates, key=lambda s: abs(s.t - t))


def _nearest_detection(
    detections: List[DetectionEvent], t: float, max_dt: float,
) -> Optional[DetectionEvent]:
    best = None
    best_dt = None
    for d in detections:
        dt = abs(d.t - t)
        if dt > max_dt:
            continue
        if best_dt is None or dt < best_dt:
            best, best_dt = d, dt
    return best


def _marker(
    frame_id: str, stamp, marker_id: int, x: float, y: float,
    color: ColorRGBA, radius: float,
) -> ImageMarker:
    m = ImageMarker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'centroid_kf'
    m.id = marker_id
    m.type = ImageMarker.CIRCLE
    m.action = ImageMarker.ADD
    m.position = Point(x=float(x), y=float(y), z=0.0)
    m.scale = float(radius)
    m.outline_color = color
    m.filled = 0
    m.lifetime = Duration(sec=0, nanosec=0)
    return m


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--out', required=True, help='Output bag directory (must not already exist)')
    parser.add_argument('--image-topic', default='/mask_overlay', help='Image topic to annotate')
    parser.add_argument('--marker-topic', default='/sc2/visual_servo/centroid_kf_markers')
    parser.add_argument(
        '--raw-detection-match-window-s', type=float, default=0.15,
        help='Only draw the raw-detection marker if one landed within this many seconds of the frame.',
    )
    parser.add_argument('--predicted-radius-px', type=float, default=14.0)
    parser.add_argument('--raw-radius-px', type=float, default=9.0)

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
            'instead of whatever the depth source reports. Takes precedence '
            'over --z-scale-percent if both are given.'
        ),
    )
    parser.add_argument(
        '--z-scale-percent', type=float, default=100.0,
        help=(
            'Scale whatever the depth source reports by this percent (100 = '
            'no change, 150 = use 1.5x the reported range), preserving the '
            'depth signal\'s own real variation/timing instead of flattening '
            'it to a constant the way --override-z-m does.'
        ),
    )
    parser.add_argument(
        '--z-used-topic', default='/sc2/visual_servo/z_used_debug',
        help='Topic to write the actual (post override/scale) range used each tick to, in the output bag.',
    )
    parser.add_argument('--process-noise-px2-per-s', type=float, default=4.0)
    parser.add_argument('--measurement-noise-px2', type=float, default=25.0)
    parser.add_argument('--no-velocity-covariance', action='store_true')
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
    if not detections:
        print('No detections found on the bbox topic -- nothing to annotate.', file=sys.stderr)
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
    print(f'Replayed {len(samples)} KF ticks over {(t_last - t_first):.1f}s')

    reader = _open_bag(args.bag)
    topics_info = reader.get_all_topics_and_types()
    image_topic_present = any(t.name == args.image_topic for t in topics_info)
    if not image_topic_present:
        print(
            f'WARNING: --image-topic {args.image_topic!r} not found in the bag. '
            f'Available topics: {sorted(t.name for t in topics_info)}',
            file=sys.stderr,
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    for t in topics_info:
        writer.create_topic(t)
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.marker_topic, type='visualization_msgs/msg/ImageMarker',
        serialization_format='cdr',
    ))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=args.z_used_topic, type='std_msgs/msg/Float64',
        serialization_format='cdr',
    ))

    n_z_written = 0
    for sample in samples:
        if sample.z_used is None:
            continue
        msg = Float64(data=float(sample.z_used))
        writer.write(args.z_used_topic, serialize_message(msg), int(sample.t * 1e9))
        n_z_written += 1
    print(
        f'Wrote {n_z_written} Z-used samples to {args.z_used_topic} '
        f'(override_z={args.override_z_m}, z_scale_percent={args.z_scale_percent}) '
        f'-- compare against the depth source\'s own topic in the copied-through bag.'
    )

    n_frames = 0
    n_predicted = 0
    n_raw = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        writer.write(topic, data, t_ns)

        if topic != args.image_topic:
            continue
        n_frames += 1
        img = deserialize_message(data, Image)
        t = t_ns * 1e-9

        sample = _nearest_sample(samples, t)
        if sample is not None and sample.filtered_u is not None:
            marker = _marker(
                img.header.frame_id, img.header.stamp, _PREDICTED_ID,
                sample.filtered_u, sample.filtered_v,
                _PREDICTED_COLOR, args.predicted_radius_px,
            )
            writer.write(args.marker_topic, serialize_message(marker), t_ns)
            n_predicted += 1

        raw = _nearest_detection(detections, t, args.raw_detection_match_window_s)
        if raw is not None:
            marker = _marker(
                img.header.frame_id, img.header.stamp, _RAW_ID,
                raw.u, raw.v, _RAW_COLOR, args.raw_radius_px,
            )
            writer.write(args.marker_topic, serialize_message(marker), t_ns)
            n_raw += 1

    print(
        f'Wrote {args.out}: {n_frames} frames on {args.image_topic}, '
        f'{n_predicted} predicted-centroid markers (orange), '
        f'{n_raw} raw-detection markers (cyan) on {args.marker_topic}'
    )


if __name__ == '__main__':
    main()
