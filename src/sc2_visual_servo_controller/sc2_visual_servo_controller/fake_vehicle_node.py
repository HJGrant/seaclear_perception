import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class FakeVehicleNode(Node):
    def __init__(self):
        super().__init__('fake_vehicle_node')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('pose_topic', '/sc2/visual_servo/fake_pose')
        self.declare_parameter('rate_hz', 30.0)

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._pose_topic = str(self.get_parameter('pose_topic').value)
        self._rate = float(self.get_parameter('rate_hz').value)

        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wyaw = 0.0

        self.create_subscription(TwistStamped, cmd_topic, self._cmd_cb, 10)
        self._pub = self.create_publisher(PoseStamped, self._pose_topic, 10)
        self.create_timer(1.0 / max(1e-3, self._rate), self._tick)

    def _cmd_cb(self, msg: TwistStamped):
        self._vx = float(msg.twist.linear.x)
        self._vy = float(msg.twist.linear.y)
        self._vz = float(msg.twist.linear.z)
        self._wyaw = float(msg.twist.angular.z)

    def _tick(self):
        dt = 1.0 / max(1e-3, self._rate)
        self._yaw += self._wyaw * dt

        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        vwx = self._vx * c - self._vy * s
        vwy = self._vx * s + self._vy * c

        self._x += vwx * dt
        self._y += vwy * dt
        self._z += self._vz * dt

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = self._z
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVehicleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
