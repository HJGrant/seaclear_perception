import math
import threading

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .conversion import compute_depth_median, extract_hypothesis, select_best_detection


class TargetObserverNode(Node):
    def __init__(self):
        super().__init__('target_observer_node')

        self.declare_parameter('target_class', '')
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 20.0)
        self.declare_parameter('source', 'yolo_detection2d')
        self.declare_parameter('detections_topic', '/detections_output')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('output_topic', '/sc2/visual_servo/target_obs')

        self._target_class = str(self.get_parameter('target_class').value)
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._use_depth = bool(self.get_parameter('use_depth').value)
        self._depth_min_m = float(self.get_parameter('depth_min_m').value)
        self._depth_max_m = float(self.get_parameter('depth_max_m').value)
        self._source = str(self.get_parameter('source').value)

        detections_topic = str(self.get_parameter('detections_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._latest_depth = None

        self._pub = self.create_publisher(TargetObservation, output_topic, 10)
        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)

        self.get_logger().info(
            f'target_observer_node listening detections={detections_topic}, depth={depth_topic}, out={output_topic}'
        )

    def _depth_cb(self, msg: Image) -> None:
        if not self._use_depth:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed depth conversion: {exc}')
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _detections_cb(self, msg: Detection2DArray) -> None:
        det = select_best_detection(msg, self._target_class, self._min_confidence)
        if det is None:
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

        out.depth_m = float(depth_m)
        out.keypoints_uv = []
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
