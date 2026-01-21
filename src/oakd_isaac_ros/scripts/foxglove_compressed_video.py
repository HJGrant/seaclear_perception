#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from foxglove_msgs.msg import CompressedVideo

class H264ToFoxglove(Node):
    def __init__(self):
        super().__init__('h264_to_foxglove')
        
        # Subscribe to your H264 compressed topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_raw/h264', # Adjust to your source topic
            self.listener_callback,
            10)
        
        # Publish the Foxglove-specific Video message
        self.publisher = self.create_publisher(
            CompressedVideo,
            '/oak/rgb/video/foxglove',
            10)

    def listener_callback(self, msg):
        video_msg = CompressedVideo()
        
        # Copy header and timestamp
        video_msg.timestamp = msg.header.stamp
        video_msg.frame_id = msg.header.frame_id
        
        # Set format to h264 (Foxglove supports 'h264' or 'h265')
        video_msg.format = 'h264'
        
        # Pass the raw bytes directly (Zero-copy payload)
        video_msg.data = msg.data
        
        self.publisher.publish(video_msg)

def main(args=None):
    rclpy.init(args=args)
    node = H264ToFoxglove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()