import math
import threading
from typing import Dict, Optional

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .conversion import compute_depth_median, extract_hypothesis, select_best_detection


class TargetObserverNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('target_observer_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('target_class', '')
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('pick_strategy', 'highest_confidence')
        self.declare_parameter('max_age_ms', 300)
        self.declare_parameter('timeout_ms', 500)

        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 20.0)

        self.declare_parameter('source', 'yolo_detection2d')
        self.declare_parameter('detections_topic', '/detections_output')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('output_topic', '/sc2/visual_servo/target_obs')

        self._target_class = str(self.get_parameter('target_class').value)
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._pick_strategy = str(self.get_parameter('pick_strategy').value).strip().lower()
        self._max_age_ms = float(self.get_parameter('max_age_ms').value)
        self._timeout_ms = float(self.get_parameter('timeout_ms').value)

        self._use_depth = self._as_bool(self.get_parameter('use_depth').value)
        self._depth_min_m = float(self.get_parameter('depth_min_m').value)
        self._depth_max_m = float(self.get_parameter('depth_max_m').value)

        self._source = str(self.get_parameter('source').value)

        detections_topic = str(self.get_parameter('detections_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        if self._pick_strategy not in {'highest_confidence', 'largest_bbox'}:
            self.get_logger().warn(
                f"Invalid pick_strategy='{self._pick_strategy}', using highest_confidence"
            )
            self._pick_strategy = 'highest_confidence'

        self._bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._latest_depth = None

        self._last_pub_time = self.get_clock().now()
        self._warn_timestamps: Dict[str, float] = {}

        self._pub = self.create_publisher(TargetObservation, output_topic, 10)
        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)

        if self._use_depth and depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        else:
            self._use_depth = False

        self.create_timer(0.2, self._health_tick)

        self.get_logger().info(
            'target_observer_node: '
            f'detections={detections_topic}, depth={depth_topic if self._use_depth else "<disabled>"}, '
            f'out={output_topic}, target_class={self._target_class!r}, pick={self._pick_strategy}'
        )

    def _warn_throttle(self, key: str, msg: str, period_s: float = 2.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1e9)
        if (now_s - last) >= period_s:
            self._warn_timestamps[key] = now_s
            self.get_logger().warn(msg)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _depth_cb(self, msg: Image) -> None:
        if not self._use_depth:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttle('depth_convert', f'Failed depth conversion: {exc}')
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _msg_age_ms(self, header) -> float:
        stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        if stamp_ns <= 0:
            return 0.0
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - stamp_ns) * 1e-6)

    def _detections_cb(self, msg: Detection2DArray) -> None:
        age_ms = self._msg_age_ms(msg.header)
        if self._max_age_ms > 0.0 and age_ms > self._max_age_ms:
            self._warn_throttle(
                'stale_detection',
                f'Dropping stale Detection2DArray age={age_ms:.1f}ms > max_age_ms={self._max_age_ms:.1f}',
            )
            return

        det = select_best_detection(msg, self._target_class, self._min_confidence, self._pick_strategy)
        if det is None:
            self._warn_throttle('no_match', 'No detection passed class/confidence filter')
            return

        out = TargetObservation()
        out.header = msg.header
        try:
            out.track_id = int(getattr(det, 'id', -1))
        except (TypeError, ValueError):
            out.track_id = -1
        out.u = float(det.bbox.center.position.x)
        out.v = float(det.bbox.center.position.y)
        out.w = float(det.bbox.size_x)
        out.h = float(det.bbox.size_y)

        cls_id, score = extract_hypothesis(det.results[0])
        out.confidence = float(score)
        out.source = self._source if self._source else cls_id

        depth_m = float('nan')
        if self._use_depth:
            with self._depth_lock:
                depth = self._latest_depth
            depth_m = compute_depth_median(depth, out.u, out.v, out.w, out.h)
            if math.isfinite(depth_m):
                depth_m = max(self._depth_min_m, min(self._depth_max_m, depth_m))
            else:
                self._warn_throttle('depth_nan', 'Depth ROI invalid; publishing NaN depth_m')

        out.depth_m = float(depth_m)
        out.keypoints_uv = []
        self._pub.publish(out)
        self._last_pub_time = self.get_clock().now()

    def _health_tick(self) -> None:
        dt_ms = (self.get_clock().now() - self._last_pub_time).nanoseconds * 1e-6
        if dt_ms > self._timeout_ms:
            self._warn_throttle(
                'obs_timeout',
                f'No TargetObservation published for {dt_ms:.0f}ms (timeout_ms={self._timeout_ms:.0f})',
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
