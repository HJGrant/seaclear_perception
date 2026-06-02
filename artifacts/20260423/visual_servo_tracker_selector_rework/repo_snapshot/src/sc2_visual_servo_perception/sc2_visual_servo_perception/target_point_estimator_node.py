import threading
from typing import Dict, Optional, Tuple

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import math
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .conversion import compute_representative_depth_near_uv_median


class TargetPointEstimatorNode(Node):
    """Estimate a target-relative point from the selected bbox and local depth.

    `/sc2/visual_servo/target_point` is published as a PoseStamped in the camera
    optical frame convention used by the depth image:
      position.x: metric horizontal offset from optical center, meters
      position.y: metric vertical offset from optical center, meters
      position.z: target-relative range/depth, meters

    The staged PID controller consumes position.z as its primary vertical
    target-relative distance source.
    """

    def __init__(self, *, parameter_overrides=None):
        super().__init__('target_point_estimator_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('source_mode', 'bbox_depth_roi')
        self.declare_parameter('target_bounding_box', '/best_detection')
        self.declare_parameter('target_obs_topic', '')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/stereo/camera_info')
        self.declare_parameter('mask_topic', '')
        self.declare_parameter('target_point_topic', '/sc2/visual_servo/target_point')
        self.declare_parameter('grasp_pose_topic', '')
        self.declare_parameter('status_topic', '/sc2/visual_servo/target_point_status')

        self.declare_parameter('max_obs_age_ms', 300.0)
        self.declare_parameter('max_depth_age_ms', 300.0)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 20.0)
        self.declare_parameter('min_valid_points', 20)
        self.declare_parameter('num_nearest_points', 25)
        self.declare_parameter('max_search_radius_px', 40.0)
        self.declare_parameter('use_uv_median_depth', True)
        self.declare_parameter('use_mask_gating', False)
        self.declare_parameter('mask_threshold', 1)

        self.declare_parameter('hold_last_valid', True)
        self.declare_parameter('max_hold_ms', 500.0)
        self.declare_parameter('default_frame_id', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('enable_temporal_filter', True)
        self.declare_parameter('filter_alpha_uv', 0.4)
        self.declare_parameter('filter_alpha_depth', 0.3)
        self.declare_parameter('filter_alpha_size', 0.3)
        self.declare_parameter('max_uv_jump_px', 80.0)
        self.declare_parameter('max_depth_jump_m', 0.5)
        self.declare_parameter('reject_large_jumps', True)

        # Fallback intrinsics when camera_info is unavailable
        self.declare_parameter('fx_px', 600.0)
        self.declare_parameter('fy_px', 600.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)

        self._source_mode = str(self.get_parameter('source_mode').value).strip().lower()
        if self._source_mode != 'bbox_depth_roi':
            self.get_logger().warn(
                f"source_mode={self._source_mode} is not implemented yet. "
                "Falling back to bbox_depth_roi."
            )
            self._source_mode = 'bbox_depth_roi'

        self._target_bbox_topic = str(self.get_parameter('target_bounding_box').value)
        legacy_target_obs_topic = str(self.get_parameter('target_obs_topic').value)
        self._depth_topic = str(self.get_parameter('depth_topic').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._mask_topic = str(self.get_parameter('mask_topic').value)
        self._target_point_topic = str(self.get_parameter('target_point_topic').value)
        legacy_grasp_pose_topic = str(self.get_parameter('grasp_pose_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        if legacy_target_obs_topic:
            self.get_logger().warn(
                'Parameter target_obs_topic is deprecated; use target_bounding_box instead.'
            )
            self._target_bbox_topic = legacy_target_obs_topic
        if legacy_grasp_pose_topic:
            self.get_logger().warn(
                'Parameter grasp_pose_topic is deprecated; use target_point_topic instead.'
            )
            self._target_point_topic = legacy_grasp_pose_topic

        self._max_obs_age_ms = float(self.get_parameter('max_obs_age_ms').value)
        self._max_depth_age_ms = float(self.get_parameter('max_depth_age_ms').value)
        self._depth_min_m = float(self.get_parameter('depth_min_m').value)
        self._depth_max_m = float(self.get_parameter('depth_max_m').value)
        self._min_valid_points = int(self.get_parameter('min_valid_points').value)
        self._num_nearest_points = int(self.get_parameter('num_nearest_points').value)
        self._max_search_radius_px = float(self.get_parameter('max_search_radius_px').value)
        self._use_uv_median_depth = self._as_bool(self.get_parameter('use_uv_median_depth').value)
        self._use_mask_gating = self._as_bool(self.get_parameter('use_mask_gating').value)
        self._mask_threshold = int(self.get_parameter('mask_threshold').value)
        self._hold_last_valid = self._as_bool(self.get_parameter('hold_last_valid').value)
        self._max_hold_ms = float(self.get_parameter('max_hold_ms').value)
        self._default_frame_id = str(self.get_parameter('default_frame_id').value)
        self._enable_temporal_filter = self._as_bool(self.get_parameter('enable_temporal_filter').value)
        self._filter_alpha_uv = float(self.get_parameter('filter_alpha_uv').value)
        self._filter_alpha_depth = float(self.get_parameter('filter_alpha_depth').value)
        self._filter_alpha_size = float(self.get_parameter('filter_alpha_size').value)
        self._max_uv_jump_px = float(self.get_parameter('max_uv_jump_px').value)
        self._max_depth_jump_m = float(self.get_parameter('max_depth_jump_m').value)
        self._reject_large_jumps = self._as_bool(self.get_parameter('reject_large_jumps').value)

        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self._frame_id_from_cam: Optional[str] = None

        self._bridge = CvBridge()
        self._data_lock = threading.Lock()
        self._latest_depth = None
        self._latest_depth_time = None
        self._latest_mask = None

        self._last_valid_pose: Optional[PoseStamped] = None
        self._last_valid_pose_time = self.get_clock().now()
        self._warn_timestamps: Dict[str, float] = {}
        self._filtered_bbox: Optional[Tuple[float, float, float, float]] = None
        self._filtered_rep_uvz: Optional[Tuple[float, float, float]] = None

        self._target_point_pub = self.create_publisher(PoseStamped, self._target_point_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(Detection2D, self._target_bbox_topic, self._target_obs_cb, 20)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, 20)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 20)
        if self._use_mask_gating and self._mask_topic:
            self.create_subscription(Image, self._mask_topic, self._mask_cb, 20)

        self.get_logger().info(
            'target_point_estimator_node: '
            f'mode={self._source_mode}, target_obs={self._target_bbox_topic}, depth={self._depth_topic}, '
            f'camera_info={self._camera_info_topic}, mask={self._mask_topic if self._use_mask_gating else "<disabled>"}, '
            f'out={self._target_point_topic}'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _warn_throttle(self, key: str, msg: str, period_s: float = 2.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1e9)
        if (now_s - last) >= period_s:
            self._warn_timestamps[key] = now_s
            self.get_logger().warn(msg)

    def _publish_status(self, text: str) -> None:
        s = String()
        s.data = text
        self._status_pub.publish(s)

    def _msg_age_ms(self, header) -> float:
        stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        if stamp_ns <= 0:
            return 0.0
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - stamp_ns) * 1e-6)

    @staticmethod
    def _ema(prev: float, value: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, float(alpha)))
        return (alpha * float(value)) + ((1.0 - alpha) * float(prev))

    def _depth_age_ms(self) -> Optional[float]:
        if self._latest_depth_time is None:
            return None
        return (self.get_clock().now() - self._latest_depth_time).nanoseconds * 1e-6

    def _depth_is_fresh(self) -> bool:
        age_ms = self._depth_age_ms()
        return age_ms is not None and age_ms <= self._max_depth_age_ms

    def _reset_filters(self) -> None:
        self._filtered_bbox = None
        self._filtered_rep_uvz = None

    def _hold_last_valid_pose(self, stamp, status: str) -> bool:
        if not (self._hold_last_valid and self._last_valid_pose is not None):
            return False

        hold_age_ms = (self.get_clock().now() - self._last_valid_pose_time).nanoseconds * 1e-6
        if hold_age_ms > self._max_hold_ms:
            return False

        held = PoseStamped()
        held.header.stamp = stamp
        held.header.frame_id = self._last_valid_pose.header.frame_id
        held.pose = self._last_valid_pose.pose
        self._target_point_pub.publish(held)
        self._publish_status(status)
        return True

    def _filter_bbox(self, u: float, v: float, w: float, h: float) -> Tuple[Tuple[float, float, float, float], bool]:
        raw = (float(u), float(v), float(w), float(h))
        if not self._enable_temporal_filter:
            self._filtered_bbox = raw
            return raw, False
        if self._filtered_bbox is None:
            self._filtered_bbox = raw
            return raw, False

        prev = self._filtered_bbox
        if self._reject_large_jumps:
            uv_jump = max(abs(raw[0] - prev[0]), abs(raw[1] - prev[1]))
            size_jump = max(abs(raw[2] - prev[2]), abs(raw[3] - prev[3]))
            if uv_jump > self._max_uv_jump_px or size_jump > self._max_uv_jump_px:
                return prev, True

        filtered = (
            self._ema(prev[0], raw[0], self._filter_alpha_uv),
            self._ema(prev[1], raw[1], self._filter_alpha_uv),
            self._ema(prev[2], raw[2], self._filter_alpha_size),
            self._ema(prev[3], raw[3], self._filter_alpha_size),
        )
        self._filtered_bbox = filtered
        return filtered, False

    def _filter_representative_uvz(
        self, u_rep: float, v_rep: float, z_rep: float
    ) -> Tuple[Tuple[float, float, float], bool]:
        raw = (float(u_rep), float(v_rep), float(z_rep))
        if not self._enable_temporal_filter:
            self._filtered_rep_uvz = raw
            return raw, False
        if self._filtered_rep_uvz is None:
            self._filtered_rep_uvz = raw
            return raw, False

        prev = self._filtered_rep_uvz
        if self._reject_large_jumps:
            uv_jump = max(abs(raw[0] - prev[0]), abs(raw[1] - prev[1]))
            depth_jump = abs(raw[2] - prev[2])
            if uv_jump > self._max_uv_jump_px or depth_jump > self._max_depth_jump_m:
                return prev, True

        filtered = (
            self._ema(prev[0], raw[0], self._filter_alpha_uv),
            self._ema(prev[1], raw[1], self._filter_alpha_uv),
            self._ema(prev[2], raw[2], self._filter_alpha_depth),
        )
        self._filtered_rep_uvz = filtered
        return filtered, False

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttle('depth_convert', f'Failed depth conversion: {exc}')
            return
        with self._data_lock:
            self._latest_depth = depth
            self._latest_depth_time = self.get_clock().now()

    def _mask_cb(self, msg: Image) -> None:
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttle('mask_convert', f'Failed mask conversion: {exc}')
            return
        with self._data_lock:
            self._latest_mask = mask

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            if msg.k[0] > 1e-6:
                self._fx = float(msg.k[0])
            if msg.k[4] > 1e-6:
                self._fy = float(msg.k[4])
            self._cx = float(msg.k[2])
            self._cy = float(msg.k[5])
        if msg.header.frame_id:
            self._frame_id_from_cam = str(msg.header.frame_id)

    def _build_pose(self, stamp, frame_id: str, u_rep: float, v_rep: float, z_rep: float) -> PoseStamped:
        x = (u_rep - self._cx) / max(1e-6, self._fx) * z_rep
        y = (v_rep - self._cy) / max(1e-6, self._fy) * z_rep

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z_rep)
        pose.pose.orientation.w = 1.0
        return pose

    def _target_obs_cb(self, msg: Detection2D) -> None:
        if self._msg_age_ms(msg.header) > self._max_obs_age_ms:
            self._warn_throttle(
                'stale_target_obs',
                f'Dropping stale target_obs older than max_obs_age_ms={self._max_obs_age_ms:.1f}',
            )
            self._publish_status('STALE_TARGET_OBS')
            return

        with self._data_lock:
            depth = self._latest_depth
            depth_fresh = self._depth_is_fresh()
            mask = self._latest_mask if self._use_mask_gating else None

        if depth is None:
            self._reset_filters()
            self._publish_status('NO_DEPTH')
            return
        if not depth_fresh:
            if self._hold_last_valid_pose(msg.header.stamp, 'HOLD_LAST_VALID_DEPTH_STALE'):
                return
            self._reset_filters()
            self._publish_status('STALE_DEPTH')
            return

        target_bbox = msg.bbox
        (u, v, w, h), bbox_rejected = self._filter_bbox(
            target_bbox.center.position.x,
            target_bbox.center.position.y,
            target_bbox.size_x,
            target_bbox.size_y,
        )
        if bbox_rejected:
            self._warn_throttle(
                'target_bbox_jump',
                'Rejected target bbox jump and held the last valid target_point',
            )
            if self._hold_last_valid_pose(msg.header.stamp, 'REJECTED_LARGE_JUMP'):
                return
            self._publish_status('REJECTED_LARGE_JUMP')
            return

        u_rep, v_rep, z_rep, valid_count = compute_representative_depth_near_uv_median(
            depth,
            float(u),
            float(v),
            float(w),
            float(h),
            min_depth_m=self._depth_min_m,
            max_depth_m=self._depth_max_m,
            min_valid_points=self._min_valid_points,
            num_nearest_points=self._num_nearest_points,
            max_search_radius_px=self._max_search_radius_px,
            use_uv_median_depth=self._use_uv_median_depth,
            mask_img=mask,
            mask_threshold=self._mask_threshold,
        )

        if not math.isfinite(z_rep):
            if self._hold_last_valid_pose(msg.header.stamp, 'HOLD_LAST_VALID'):
                return

            self._warn_throttle(
                'target_point_invalid',
                f'No valid representative depth (valid_points={valid_count}, min_required={self._min_valid_points})',
            )
            self._reset_filters()
            self._publish_status('NO_VALID_TARGET_POINT')
            return

        (u_rep, v_rep, z_rep), rep_rejected = self._filter_representative_uvz(u_rep, v_rep, z_rep)
        if rep_rejected:
            self._warn_throttle(
                'target_point_jump',
                'Rejected representative target_point jump and held the last valid target_point',
            )
            if self._hold_last_valid_pose(msg.header.stamp, 'REJECTED_LARGE_JUMP'):
                return
            self._publish_status('REJECTED_LARGE_JUMP')
            return

        frame_id = self._frame_id_from_cam or msg.header.frame_id or self._default_frame_id
        pose = self._build_pose(msg.header.stamp, frame_id, u_rep, v_rep, z_rep)
        self._target_point_pub.publish(pose)
        self._last_valid_pose = pose
        self._last_valid_pose_time = self.get_clock().now()
        self._publish_status('TRACKING_TARGET_POINT')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPointEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
