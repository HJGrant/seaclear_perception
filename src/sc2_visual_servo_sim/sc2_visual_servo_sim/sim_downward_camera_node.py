import math

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SimDownwardCameraNode(Node):
    def __init__(self):
        super().__init__('sim_downward_camera_node')
        self.declare_parameter('odom_topic', '/sim/odom')
        self.declare_parameter('image_topic', '/sim/down/image_raw')
        self.declare_parameter('depth_topic', '/sim/down/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/sim/down/camera_info')
        self.declare_parameter('target_point_topic', '/sim/world/target_point')
        self.declare_parameter('camera_optical_frame_id', 'down_camera_optical_frame')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fx_px', 700.0)
        self.declare_parameter('fy_px', 700.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('target_world_x_m', 0.0)
        self.declare_parameter('target_world_y_m', 0.0)
        self.declare_parameter('target_world_z_m', -4.5)
        self.declare_parameter('target_size_m', 0.45)
        self.declare_parameter('lighting_mode', 'bright')
        self.declare_parameter('seabed_depth_m', 4.5)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._depth_topic = str(self.get_parameter('depth_topic').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._target_point_topic = str(self.get_parameter('target_point_topic').value)
        self._camera_frame = str(self.get_parameter('camera_optical_frame_id').value)
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._target_world = np.array([
            float(self.get_parameter('target_world_x_m').value),
            float(self.get_parameter('target_world_y_m').value),
            float(self.get_parameter('target_world_z_m').value),
        ], dtype=np.float32)
        self._target_size = float(self.get_parameter('target_size_m').value)
        self._lighting = str(self.get_parameter('lighting_mode').value).strip().lower()
        self._seabed_depth = float(self.get_parameter('seabed_depth_m').value)

        self._bridge = CvBridge()
        self._odom = None

        self._img_pub = self.create_publisher(Image, self._image_topic, 10)
        self._depth_pub = self.create_publisher(Image, self._depth_topic, 10)
        self._info_pub = self.create_publisher(CameraInfo, self._camera_info_topic, 10)
        self._target_pub = self.create_publisher(PointStamped, self._target_point_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 20)
        self.create_timer(1.0 / max(1e-3, self._rate_hz), self._tick)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _lighting_base(self) -> int:
        if self._lighting == 'dim':
            return 45
        if self._lighting == 'murky':
            return 28
        return 75

    @staticmethod
    def _rotation_world_to_body(yaw: float) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)

    @staticmethod
    def _body_to_optical(point_body: np.ndarray) -> np.ndarray:
        return np.array([
            -point_body[1],
            -point_body[0],
            -point_body[2],
        ], dtype=np.float32)

    def _tick(self) -> None:
        if self._odom is None:
            return

        stamp = self.get_clock().now().to_msg()
        yaw = 2.0 * math.atan2(self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w)
        position = np.array([
            float(self._odom.pose.pose.position.x),
            float(self._odom.pose.pose.position.y),
            float(self._odom.pose.pose.position.z),
        ], dtype=np.float32)

        rel_world = self._target_world - position
        rel_body = self._rotation_world_to_body(yaw) @ rel_world
        rel_opt = self._body_to_optical(rel_body)

        rgb = np.full((self._img_h, self._img_w, 3), self._lighting_base(), dtype=np.uint8)
        rgb[:, :, 1] = np.clip(rgb[:, :, 1] + 18, 0, 255)
        depth = np.full((self._img_h, self._img_w), self._seabed_depth, dtype=np.float32)

        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = self._camera_frame
        point_msg.point.x = float(rel_opt[0])
        point_msg.point.y = float(rel_opt[1])
        point_msg.point.z = float(rel_opt[2])
        self._target_pub.publish(point_msg)

        if rel_opt[2] > 0.15:
            u = self._fx * (rel_opt[0] / rel_opt[2]) + self._cx
            v = self._fy * (rel_opt[1] / rel_opt[2]) + self._cy
            half_w = max(10.0, 0.5 * self._fx * self._target_size / rel_opt[2])
            half_h = max(10.0, 0.5 * self._fy * self._target_size / rel_opt[2])
            x0 = int(max(0, min(self._img_w - 1, round(u - half_w))))
            x1 = int(max(0, min(self._img_w, round(u + half_w))))
            y0 = int(max(0, min(self._img_h - 1, round(v - half_h))))
            y1 = int(max(0, min(self._img_h, round(v + half_h))))
            if x1 > x0 and y1 > y0 and 0 <= u < self._img_w and 0 <= v < self._img_h:
                rgb[y0:y1, x0:x1, :] = np.array([20, 120, 255], dtype=np.uint8)
                rgb[y0:y1, x0:x1, 1] = 90 if self._lighting == 'murky' else 130
                depth[y0:y1, x0:x1] = float(rel_opt[2])

        img_msg = self._bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self._camera_frame
        self._img_pub.publish(img_msg)

        depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self._camera_frame
        self._depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._camera_frame
        info.width = self._img_w
        info.height = self._img_h
        info.k = [self._fx, 0.0, self._cx, 0.0, self._fy, self._cy, 0.0, 0.0, 1.0]
        info.p = [self._fx, 0.0, self._cx, 0.0, 0.0, self._fy, self._cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SimDownwardCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
