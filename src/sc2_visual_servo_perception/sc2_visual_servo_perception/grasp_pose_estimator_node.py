import threading
from typing import Dict, Optional

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .conversion import compute_representative_depth_near_uv_median


class GraspPoseEstimatorNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('grasp_pose_estimator_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('source_mode', 'bbox_depth_roi')
        self.declare_parameter('target_obs_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/stereo/camera_info')
        self.declare_parameter('mask_topic', '')
        self.declare_parameter('grasp_pose_topic', '/sc2/visual_servo/grasp_pose')
        self.declare_parameter('status_topic', '/sc2/visual_servo/grasp_pose_status')

        self.declare_parameter('max_obs_age_ms', 300.0)
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

        self._target_obs_topic = str(self.get_parameter('target_obs_topic').value)
        self._depth_topic = str(self.get_parameter('depth_topic').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._mask_topic = str(self.get_parameter('mask_topic').value)
        self._grasp_pose_topic = str(self.get_parameter('grasp_pose_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)

        self._max_obs_age_ms = float(self.get_parameter('max_obs_age_ms').value)
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

        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self._frame_id_from_cam: Optional[str] = None

        self._bridge = CvBridge()
        self._data_lock = threading.Lock()
        self._latest_depth = None
        self._latest_mask = None

        self._last_valid_pose: Optional[PoseStamped] = None
        self._last_valid_pose_time = self.get_clock().now()
        self._warn_timestamps: Dict[str, float] = {}

        self._grasp_pub = self.create_publisher(PoseStamped, self._grasp_pose_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(TargetObservation, self._target_obs_topic, self._target_obs_cb, 20)
        self.create_subscription(Image, self._depth_topic, self._depth_cb, 20)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 20)
        if self._use_mask_gating and self._mask_topic:
            self.create_subscription(Image, self._mask_topic, self._mask_cb, 20)

        self.get_logger().info(
            'grasp_pose_estimator_node: '
            f'mode={self._source_mode}, target_obs={self._target_obs_topic}, depth={self._depth_topic}, '
            f'camera_info={self._camera_info_topic}, mask={self._mask_topic if self._use_mask_gating else "<disabled>"}, '
            f'out={self._grasp_pose_topic}'
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

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttle('depth_convert', f'Failed depth conversion: {exc}')
            return
        with self._data_lock:
            self._latest_depth = depth

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

    def _target_obs_cb(self, msg: TargetObservation) -> None:
        if self._msg_age_ms(msg.header) > self._max_obs_age_ms:
            self._warn_throttle(
                'stale_target_obs',
                f'Dropping stale target_obs older than max_obs_age_ms={self._max_obs_age_ms:.1f}',
            )
            self._publish_status('STALE_TARGET_OBS')
            return

        with self._data_lock:
            depth = self._latest_depth
            mask = self._latest_mask if self._use_mask_gating else None

        u_rep, v_rep, z_rep, valid_count = compute_representative_depth_near_uv_median(
            depth,
            float(msg.u),
            float(msg.v),
            float(msg.w),
            float(msg.h),
            min_depth_m=self._depth_min_m,
            max_depth_m=self._depth_max_m,
            min_valid_points=self._min_valid_points,
            num_nearest_points=self._num_nearest_points,
            max_search_radius_px=self._max_search_radius_px,
            use_uv_median_depth=self._use_uv_median_depth,
            mask_img=mask,
            mask_threshold=self._mask_threshold,
        )

        if not (z_rep == z_rep):  # NaN check
            if self._hold_last_valid and self._last_valid_pose is not None:
                hold_age_ms = (self.get_clock().now() - self._last_valid_pose_time).nanoseconds * 1e-6
                if hold_age_ms <= self._max_hold_ms:
                    held = PoseStamped()
                    held.header.stamp = msg.header.stamp
                    held.header.frame_id = self._last_valid_pose.header.frame_id
                    held.pose = self._last_valid_pose.pose
                    self._grasp_pub.publish(held)
                    self._publish_status('HOLD_LAST_VALID')
                    return

            self._warn_throttle(
                'grasp_invalid',
                f'No valid representative depth (valid_points={valid_count}, min_required={self._min_valid_points})',
            )
            self._publish_status('NO_VALID_GRASP_POINT')
            return

        frame_id = self._frame_id_from_cam or msg.header.frame_id or self._default_frame_id
        pose = self._build_pose(msg.header.stamp, frame_id, u_rep, v_rep, z_rep)
        self._grasp_pub.publish(pose)
        self._last_valid_pose = pose
        self._last_valid_pose_time = self.get_clock().now()
        self._publish_status('TRACKING_GRASP_POSE')


def main(args=None):
    rclpy.init(args=args)
    node = GraspPoseEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
