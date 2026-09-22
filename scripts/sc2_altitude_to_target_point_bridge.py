#!/usr/bin/env python3
'''Bridge /sc2/altitude (sc2_interfaces/msg/SimpleDistance, mm) into
/sc2/visual_servo/target_point (geometry_msgs/msg/PoseStamped, position.z in
meters) -- visual_servo_controller_node's staged-PID Z source and the
centroid-KF Jacobian's depth input both read target_point/DVL/MAVROS, none of
which match what these sim bags actually recorded (/sc2/altitude). Lets bag
replay drive the live node's depth path without modifying the node itself.

Usage (inside the ROS-sourced environment):
    python3 scripts/sc2_altitude_to_target_point_bridge.py
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sc2_interfaces.msg import SimpleDistance


class AltitudeToTargetPointBridge(Node):
    def __init__(self):
        super().__init__('sc2_altitude_to_target_point_bridge')
        self.declare_parameter('altitude_topic', '/sc2/altitude')
        self.declare_parameter('target_point_topic', '/sc2/visual_servo/target_point')
        self.declare_parameter('frame_id', 'oak_rgb_camera_optical_frame_corrected')
        altitude_topic = self.get_parameter('altitude_topic').value
        target_point_topic = self.get_parameter('target_point_topic').value
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._pub = self.create_publisher(PoseStamped, target_point_topic, 10)
        self.create_subscription(SimpleDistance, altitude_topic, self._on_altitude, 10)
        self.get_logger().info(f'Bridging {altitude_topic} -> {target_point_topic}')

    def _on_altitude(self, msg: SimpleDistance) -> None:
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.pose.position.z = float(msg.distance) * 1e-3
        out.pose.orientation.w = 1.0
        self._pub.publish(out)


def main():
    rclpy.init()
    node = AltitudeToTargetPointBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
