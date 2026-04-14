from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from .sim_backend_common import make_odom, world_from_body


class SimDirectVelocityBackendNode(Node):
    def __init__(self):
        super().__init__('sim_direct_velocity_backend_node')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('odom_topic', '/sim/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('initial_x_m', 0.0)
        self.declare_parameter('initial_y_m', 0.0)
        self.declare_parameter('initial_z_m', -1.5)
        self.declare_parameter('initial_yaw_rad', 0.0)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)

        self._x = float(self.get_parameter('initial_x_m').value)
        self._y = float(self.get_parameter('initial_y_m').value)
        self._z = float(self.get_parameter('initial_z_m').value)
        self._yaw = float(self.get_parameter('initial_yaw_rad').value)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wyaw = 0.0

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self.create_subscription(TwistStamped, self._cmd_topic, self._cmd_cb, 20)
        self.create_timer(1.0 / max(1e-3, self._rate_hz), self._tick)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._vx = float(msg.twist.linear.x)
        self._vy = float(msg.twist.linear.y)
        self._vz = float(msg.twist.linear.z)
        self._wyaw = float(msg.twist.angular.z)

    def _tick(self) -> None:
        dt = 1.0 / max(1e-3, self._rate_hz)
        self._yaw += self._wyaw * dt
        vwx, vwy = world_from_body(self._vx, self._vy, self._yaw)
        self._x += vwx * dt
        self._y += vwy * dt
        self._z += self._vz * dt
        stamp = self.get_clock().now().to_msg()
        self._odom_pub.publish(
            make_odom(
                self._odom_frame_id,
                self._body_frame_id,
                stamp,
                self._x,
                self._y,
                self._z,
                self._yaw,
                self._vx,
                self._vy,
                self._vz,
                self._wyaw,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimDirectVelocityBackendNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
