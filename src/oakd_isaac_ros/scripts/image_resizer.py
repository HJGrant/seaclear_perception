#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import copy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class ImageResizer(Node):
    def __init__(self):
        super().__init__('image_resizer')

        # Parameters
        self.declare_parameter("resize_width", 640)
        self.declare_parameter("resize_height", 360)
        self.declare_parameter("input_image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("input_camera_info_topic", "/oak/rgb/camera_info")
        self.declare_parameter("output_image_topic", "/oak/rgb/image_resized")
        self.declare_parameter("output_camera_info_topic", "/oak/rgb/camera_resized_info")

        self.width = self.get_parameter("resize_width").get_parameter_value().integer_value
        self.height = self.get_parameter("resize_height").get_parameter_value().integer_value
        self.input_image_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value
        self.input_camera_info_topic = self.get_parameter("input_camera_info_topic").get_parameter_value().string_value
        self.output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value
        self.output_camera_info_topic = self.get_parameter("output_camera_info_topic").get_parameter_value().string_value

        # Publishers
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.output_camera_info_topic, 10)

        # Subscribers
        self.image_sub = self.create_subscription(Image, self.input_image_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.input_camera_info_topic, self.camera_info_callback, 10)

        # Bridge
        self.bridge = CvBridge()

        # Keep latest camera info
        self.latest_camera_info = None

        self.get_logger().info(f"ImageResizer started, resizing {self.input_image_topic} -> {self.output_image_topic} "
                               f"({self.width}x{self.height})")

    def image_callback(self, msg: Image):
        # Convert to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Resize
        resized = cv2.resize(cv_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        print("OpenCV Resized Image: ")
        print(resized)

        # Convert back to ROS Image
        #resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        resized_msg = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")
        resized_msg.header = msg.header
        #resized_msg.encoding = 'rgb8'
        resized_msg.height = self.height
        resized_msg.width = self.width

        # Publish resized image
        self.image_pub.publish(resized_msg)

        # Republish scaled camera_info (if available)
        if self.latest_camera_info:
            new_info = CameraInfo()
            new_info = copy.deepcopy(self.latest_camera_info)
            new_info.width = self.width
            new_info.height = self.height

            # Scale intrinsic parameters fx, fy, cx, cy
            scale_x = self.width / self.latest_camera_info.width
            scale_y = self.height / self.latest_camera_info.height
            new_info.k[0] *= scale_x  # fx
            new_info.k[2] *= scale_x  # cx
            new_info.k[4] *= scale_y  # fy
            new_info.k[5] *= scale_y  # cy
            new_info.p[0] *= scale_x  # fx
            new_info.p[2] *= scale_x  # cx
            new_info.p[5] *= scale_y  # fy
            new_info.p[6] *= scale_y  # cy

            self.camera_info_pub.publish(new_info)

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageResizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
