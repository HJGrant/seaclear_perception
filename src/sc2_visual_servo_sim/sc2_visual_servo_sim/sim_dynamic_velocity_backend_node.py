from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from .sim_backend_common import clamp, make_odom, world_from_body


class SimDynamicVelocityBackendNode(Node):
    def __init__(self):
        super().__init__('sim_dynamic_velocity_backend_node')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('odom_topic', '/sim/odom')
        self.declare_parameter('wrench_topic', '/sim/backend/wrench')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('tau_xy_sec', 0.8)
        self.declare_parameter('tau_z_sec', 0.6)
        self.declare_parameter('tau_yaw_sec', 0.5)
        self.declare_parameter('max_accel_xy_mps2', 0.6)
        self.declare_parameter('max_accel_z_mps2', 0.5)
        self.declare_parameter('max_accel_yaw_rps2', 0.8)
        self.declare_parameter('virtual_mass_kg', 10.0)
        self.declare_parameter('virtual_yaw_inertia', 4.0)
        self.declare_parameter('initial_x_m', 0.0)
        self.declare_parameter('initial_y_m', 0.0)
        self.declare_parameter('initial_z_m', -1.5)
        self.declare_parameter('initial_yaw_rad', 0.0)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._wrench_topic = str(self.get_parameter('wrench_topic').value)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._tau_xy = max(1e-3, float(self.get_parameter('tau_xy_sec').value))
        self._tau_z = max(1e-3, float(self.get_parameter('tau_z_sec').value))
        self._tau_yaw = max(1e-3, float(self.get_parameter('tau_yaw_sec').value))
        self._max_accel_xy = float(self.get_parameter('max_accel_xy_mps2').value)
        self._max_accel_z = float(self.get_parameter('max_accel_z_mps2').value)
        self._max_accel_yaw = float(self.get_parameter('max_accel_yaw_rps2').value)
        self._mass = max(1e-3, float(self.get_parameter('virtual_mass_kg').value))
        self._yaw_inertia = max(1e-3, float(self.get_parameter('virtual_yaw_inertia').value))

        self._x = float(self.get_parameter('initial_x_m').value)
        self._y = float(self.get_parameter('initial_y_m').value)
        self._z = float(self.get_parameter('initial_z_m').value)
        self._yaw = float(self.get_parameter('initial_yaw_rad').value)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wyaw = 0.0
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_vz = 0.0
        self._target_wyaw = 0.0

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._wrench_pub = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self.create_subscription(TwistStamped, self._cmd_topic, self._cmd_cb, 20)
        self.create_timer(1.0 / max(1e-3, self._rate_hz), self._tick)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._target_vx = float(msg.twist.linear.x)
        self._target_vy = float(msg.twist.linear.y)
        self._target_vz = float(msg.twist.linear.z)
        self._target_wyaw = float(msg.twist.angular.z)

    @staticmethod
    def _step_axis(current: float, target: float, tau: float, accel_limit: float, dt: float) -> float:
        desired_accel = (target - current) / max(tau, 1e-3)
        accel = clamp(desired_accel, accel_limit)
        return current + accel * dt

    def _tick(self) -> None:
        dt = 1.0 / max(1e-3, self._rate_hz)
        next_vx = self._step_axis(self._vx, self._target_vx, self._tau_xy, self._max_accel_xy, dt)
        next_vy = self._step_axis(self._vy, self._target_vy, self._tau_xy, self._max_accel_xy, dt)
        next_vz = self._step_axis(self._vz, self._target_vz, self._tau_z, self._max_accel_z, dt)
        next_wyaw = self._step_axis(self._wyaw, self._target_wyaw, self._tau_yaw, self._max_accel_yaw, dt)

        ax = (next_vx - self._vx) / dt
        ay = (next_vy - self._vy) / dt
        az = (next_vz - self._vz) / dt
        ayaw = (next_wyaw - self._wyaw) / dt

        self._vx = next_vx
        self._vy = next_vy
        self._vz = next_vz
        self._wyaw = next_wyaw
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

        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = self._body_frame_id
        wrench.wrench.force.x = self._mass * ax
        wrench.wrench.force.y = self._mass * ay
        wrench.wrench.force.z = self._mass * az
        wrench.wrench.torque.z = self._yaw_inertia * ayaw
        self._wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = SimDynamicVelocityBackendNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
