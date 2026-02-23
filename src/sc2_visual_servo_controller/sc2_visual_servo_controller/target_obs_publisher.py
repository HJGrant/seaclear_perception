import math

import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation


class TargetObsPublisher(Node):
    def __init__(self):
        super().__init__('target_obs_publisher')
        self.declare_parameter('topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 720.0)
        self.declare_parameter('amp_u_px', 220.0)
        self.declare_parameter('amp_v_px', 120.0)
        self.declare_parameter('freq_hz', 0.1)
        self.declare_parameter('base_depth_m', 1.4)
        self.declare_parameter('depth_amp_m', 0.25)

        self._topic = str(self.get_parameter('topic').value)
        self._rate = float(self.get_parameter('rate_hz').value)
        self._img_w = float(self.get_parameter('image_width').value)
        self._img_h = float(self.get_parameter('image_height').value)
        self._amp_u = float(self.get_parameter('amp_u_px').value)
        self._amp_v = float(self.get_parameter('amp_v_px').value)
        self._freq = float(self.get_parameter('freq_hz').value)
        self._base_depth = float(self.get_parameter('base_depth_m').value)
        self._depth_amp = float(self.get_parameter('depth_amp_m').value)

        self._t = 0.0
        self._pub = self.create_publisher(TargetObservation, self._topic, 10)
        self.create_timer(1.0 / max(1e-3, self._rate), self._tick)

    def _tick(self):
        self._t += 1.0 / max(1e-3, self._rate)
        msg = TargetObservation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        msg.track_id = 1
        msg.u = self._img_w * 0.5 + self._amp_u * math.sin(2.0 * math.pi * self._freq * self._t)
        msg.v = self._img_h * 0.5 + self._amp_v * math.cos(2.0 * math.pi * self._freq * self._t)
        msg.w = 130.0
        msg.h = 90.0
        msg.depth_m = self._base_depth + self._depth_amp * math.sin(2.0 * math.pi * self._freq * self._t)
        msg.confidence = 0.95
        msg.keypoints_uv = []
        msg.source = 'sim_scripted'
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetObsPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
