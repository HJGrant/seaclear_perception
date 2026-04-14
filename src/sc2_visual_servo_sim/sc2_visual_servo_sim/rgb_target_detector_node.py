from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class RgbTargetDetectorNode(Node):
    def __init__(self):
        super().__init__('rgb_target_detector_node')
        self.declare_parameter('image_topic', '/sim/down/image_raw')
        self.declare_parameter('detections_topic', '/detections_output')
        self.declare_parameter('best_detection_topic', '/best_detection')
        self.declare_parameter('frame_id', 'down_camera_optical_frame')
        self.declare_parameter('class_id', 'sim_target')
        self.declare_parameter('score', 0.95)
        self.declare_parameter('min_area_px', 200.0)

        self._image_topic = str(self.get_parameter('image_topic').value)
        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._best_detection_topic = str(self.get_parameter('best_detection_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._class_id = str(self.get_parameter('class_id').value)
        self._score = float(self.get_parameter('score').value)
        self._min_area = float(self.get_parameter('min_area_px').value)

        self._bridge = CvBridge()
        self._arr_pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)
        self._best_pub = self.create_publisher(Detection2D, self._best_detection_topic, 10)
        self.create_subscription(Image, self._image_topic, self._image_cb, 20)

    def _image_cb(self, msg: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        arr = Detection2DArray()
        arr.header = msg.header
        if not arr.header.frame_id:
            arr.header.frame_id = self._frame_id

        blue = image[:, :, 0].astype(np.int16)
        green = image[:, :, 1].astype(np.int16)
        red = image[:, :, 2].astype(np.int16)
        mask = (red > 180) & (green > 70) & (blue < 80)
        ys, xs = np.nonzero(mask)

        if xs.size == 0 or ys.size == 0:
            self._arr_pub.publish(arr)
            return

        x0 = int(xs.min())
        x1 = int(xs.max()) + 1
        y0 = int(ys.min())
        y1 = int(ys.max()) + 1
        area = float((x1 - x0) * (y1 - y0))
        if area < self._min_area:
            self._arr_pub.publish(arr)
            return

        det = Detection2D()
        det.header = arr.header
        det.bbox.center.position.x = float(x0 + 0.5 * (x1 - x0))
        det.bbox.center.position.y = float(y0 + 0.5 * (y1 - y0))
        det.bbox.size_x = float(x1 - x0)
        det.bbox.size_y = float(y1 - y0)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self._class_id
        hyp.hypothesis.score = self._score
        det.results.append(hyp)
        arr.detections.append(det)
        self._arr_pub.publish(arr)
        self._best_pub.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = RgbTargetDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
