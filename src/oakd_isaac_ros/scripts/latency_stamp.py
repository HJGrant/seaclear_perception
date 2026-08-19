#!/usr/bin/env python3
"""Debug-only latency probe.

Burns the current wall-clock time (ms since epoch) into the top-left
corner of /oak/rgb/image_raw as a row of black/white blocks (not text --
robust to H264 compression artifacts that would confuse OCR), and
republishes it on /oak/rgb/image_raw_stamped. Pair with
scripts/latency_measure.py on the receiving end of the RTP stream.

Not part of the normal perception pipeline. To test, temporarily point
whichever RTP video_output node you're measuring at image_raw_stamped
instead of image_raw (e.g. edit the remapping in rtp_composable_nodes'
caller, or run this standalone against debug/rtp.launch.py), then remove
it again afterwards -- the per-frame OpenCV draw is cheap but there's no
reason to pay it outside of a deliberate latency test.

Run directly (no colcon install needed, matches image_resizer.py):
    python3 scripts/latency_stamp.py
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

NUM_BITS = 44  # headroom for ms-since-epoch until year ~2527
BLOCK_SIZE = 16
ORIGIN = (10, 10)  # x, y of the first (MSB) block, top-left corner


class LatencyStamp(Node):
    def __init__(self):
        super().__init__('latency_stamp')
        self.bridge = CvBridge()
        # depthai_ros_driver publishes camera topics as best-effort
        # SensorDataQoS; a default (reliable) subscription silently never
        # connects to that -- no error, just zero messages received.
        self.sub = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, '/oak/rgb/image_raw_stamped', 10)
        self.get_logger().info(
            'Stamping /oak/rgb/image_raw -> /oak/rgb/image_raw_stamped')

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ts_ms = int(time.time() * 1000)
        x0, y0 = ORIGIN
        for bit_index in range(NUM_BITS):
            bit = (ts_ms >> (NUM_BITS - 1 - bit_index)) & 1
            color = (255, 255, 255) if bit else (0, 0, 0)
            x = x0 + bit_index * BLOCK_SIZE
            image[y0:y0 + BLOCK_SIZE, x:x + BLOCK_SIZE] = color

        out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LatencyStamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
