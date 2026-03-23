#!/usr/bin/env python3

import argparse
import os
import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

class ESSVisualizer(Node):
    def __init__(self, args):
        super().__init__('ess_visualizer')
        self.args = args
        self._bridge = cv_bridge.CvBridge()

        # Input: Disparity from ESS
        self._disp_sub = self.create_subscription(
            DisparityImage, 'disparity', self.ess_callback, 10)

        # Output: Colorized image for the H264 encoder
        self._color_pub = self.create_publisher(Image, 'depth_color', 10)
        
        self.get_logger().info('ESS Visualizer started. Publishing to /depth_color')

    def ess_callback(self, disp_msg):
        try:
            # ESS output is typically 32-bit float disparity
            disp_img = self._bridge.imgmsg_to_cv2(disp_msg.image)
            
            # Normalize based on max disparity (ESS default is often 256)
            # We clip to ensure values stay within 0-255 range
            norm_disp = np.clip(disp_img, self.args.min_disp, self.args.max_disp)
            norm_disp = ((norm_disp - self.args.min_disp) / 
                         (self.args.max_disp - self.args.min_disp) * 255).astype(np.uint8)

            # Apply Viridis (or COLORMAP_JET if you prefer blue-to-red)
            color_map = cv2.applyColorMap(norm_disp, cv2.COLORMAP_VIRIDIS)

            # Convert to ROS Image message and publish
            # We use bgr8 because OpenCV's default is BGR
            color_msg = self._bridge.cv2_to_imgmsg(color_map, encoding='bgr8')
            color_msg.header = disp_msg.header # Keep timestamp alignment
            self._color_pub.publish(color_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to colorize disparity: {str(e)}')

def main():
    parser = argparse.ArgumentParser(description='ESS Disparity Node Visualizer')
    parser.add_argument('--min_disp', type=float, default=0.0)
    parser.add_argument('--max_disp', type=float, default=256.0)
    args, unknown = parser.parse_known_args() # Use parse_known_args for ROS2 compat

    rclpy.init()
    node = ESSVisualizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()