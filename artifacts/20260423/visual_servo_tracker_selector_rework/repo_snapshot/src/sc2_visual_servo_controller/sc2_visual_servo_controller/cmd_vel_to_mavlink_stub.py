from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class CmdVelToMavlinkStub(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavlink_stub')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.create_subscription(TwistStamped, cmd_topic, self._cb, 20)
        self.get_logger().info('MAVLink stub active; logging velocity commands only.')

    def _cb(self, msg: TwistStamped):
        self.get_logger().info(
            'MAVLINK_STUB send body vel: '
            f'vx={msg.twist.linear.x:.3f} vy={msg.twist.linear.y:.3f} '
            f'vz={msg.twist.linear.z:.3f} yaw_rate={msg.twist.angular.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavlinkStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
