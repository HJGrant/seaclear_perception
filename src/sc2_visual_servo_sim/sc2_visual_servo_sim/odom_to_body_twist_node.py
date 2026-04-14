import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class OdomToBodyTwistNode(Node):
    def __init__(self):
        super().__init__('odom_to_body_twist_node')
        self.declare_parameter('odom_topic', '/sim/odom')
        self.declare_parameter('body_twist_topic', '/sim/body_velocity')
        self.declare_parameter('body_frame_id', 'base_link')

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._body_twist_topic = str(self.get_parameter('body_twist_topic').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)

        self._pub = self.create_publisher(TwistStamped, self._body_twist_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_w = float(msg.twist.twist.linear.x)
        vy_w = float(msg.twist.twist.linear.y)

        out = TwistStamped()
        out.header = msg.header
        out.header.frame_id = self._body_frame_id
        out.twist.linear.x = vx_w * c + vy_w * s
        out.twist.linear.y = -vx_w * s + vy_w * c
        out.twist.linear.z = float(msg.twist.twist.linear.z)
        out.twist.angular.z = float(msg.twist.twist.angular.z)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBodyTwistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
