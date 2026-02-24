from typing import Optional

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from .ibvs.controller_math import clamp


class CmdVelToMavrosBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavros_bridge')

        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('backend', 'rc_override')
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('watchdog_hz', 20.0)
        self.declare_parameter('log_hz', 2.0)

        self.declare_parameter('deadman_topic', '/sc2/visual_servo/deadman')
        self.declare_parameter('deadman_required', True)
        self.declare_parameter('deadman_initial', False)

        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vz', 0.3)
        self.declare_parameter('max_yaw_rate', 0.4)

        # Example mapping from reference code in
        # docs/reference/blue_rov_position_control/pos_controller.py:
        # Yaw CH4, Forward CH5, Lateral CH6, neutral 1500.
        self.declare_parameter('rc_topic', '/mavros/rc/override')
        self.declare_parameter('setpoint_vel_topic', '/mavros/setpoint_velocity/cmd_vel_unstamped')
        self.declare_parameter('channel_yaw', 4)
        self.declare_parameter('channel_forward', 5)
        self.declare_parameter('channel_lateral', 6)
        self.declare_parameter('channel_vertical', 3)
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('min_pwm', 1100)
        self.declare_parameter('max_pwm', 1900)
        self.declare_parameter('scale_forward_pwm_per_mps', 220.0)
        self.declare_parameter('scale_lateral_pwm_per_mps', 220.0)
        self.declare_parameter('scale_vertical_pwm_per_mps', 180.0)
        self.declare_parameter('scale_yaw_pwm_per_rps', 260.0)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._backend = str(self.get_parameter('backend').value).strip().lower()
        self._timeout_s = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._log_hz = max(0.1, float(self.get_parameter('log_hz').value))
        self._deadman_topic = str(self.get_parameter('deadman_topic').value)
        self._deadman_required = self._as_bool(self.get_parameter('deadman_required').value)
        self._deadman = self._as_bool(self.get_parameter('deadman_initial').value)

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vz = float(self.get_parameter('max_vz').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)

        self._rc_topic = str(self.get_parameter('rc_topic').value)
        self._setpoint_vel_topic = str(self.get_parameter('setpoint_vel_topic').value)
        self._ch_yaw = int(self.get_parameter('channel_yaw').value)
        self._ch_forward = int(self.get_parameter('channel_forward').value)
        self._ch_lateral = int(self.get_parameter('channel_lateral').value)
        self._ch_vertical = int(self.get_parameter('channel_vertical').value)
        self._neutral_pwm = int(self.get_parameter('neutral_pwm').value)
        self._min_pwm = int(self.get_parameter('min_pwm').value)
        self._max_pwm = int(self.get_parameter('max_pwm').value)
        self._scale_forward = float(self.get_parameter('scale_forward_pwm_per_mps').value)
        self._scale_lateral = float(self.get_parameter('scale_lateral_pwm_per_mps').value)
        self._scale_vertical = float(self.get_parameter('scale_vertical_pwm_per_mps').value)
        self._scale_yaw = float(self.get_parameter('scale_yaw_pwm_per_rps').value)

        self._last_cmd_time = self.get_clock().now()
        self._last_cmd: Optional[TwistStamped] = None
        self._last_log_time = self.get_clock().now()

        self._rc_msg_type = None
        self._rc_pub = None
        self._vel_pub = None

        if self._backend == 'rc_override':
            try:
                from mavros_msgs.msg import OverrideRCIn  # pylint: disable=import-outside-toplevel
                self._rc_msg_type = OverrideRCIn
                self._rc_pub = self.create_publisher(OverrideRCIn, self._rc_topic, 10)
            except Exception as exc:
                self.get_logger().error(f'rc_override backend unavailable (mavros_msgs missing): {exc}')
                self.get_logger().error('Falling back to dry_run backend.')
                self.get_logger().error(
                    'REQUESTED rc_override but running dry_run due to missing mavros_msgs. '
                    'Install ros-humble-mavros-msgs or run ./scripts/ensure_ros_apt_deps.sh'
                )
                self._backend = 'dry_run'
        elif self._backend == 'setpoint_velocity':
            self._vel_pub = self.create_publisher(Twist, self._setpoint_vel_topic, 10)
        elif self._backend != 'dry_run':
            self.get_logger().warn(f'Unknown backend={self._backend}. Falling back to dry_run.')
            self._backend = 'dry_run'

        self.create_subscription(TwistStamped, self._cmd_topic, self._cmd_cb, 20)
        self.create_subscription(Bool, self._deadman_topic, self._deadman_cb, 10)

        watchdog_hz = max(1e-3, float(self.get_parameter('watchdog_hz').value))
        self.create_timer(1.0 / watchdog_hz, self._watchdog)

        self.get_logger().info(
            f'cmd_vel_to_mavros_bridge backend={self._backend} cmd_topic={self._cmd_topic} '
            f'deadman_required={self._deadman_required} deadman_initial={self._deadman}'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _deadman_cb(self, msg: Bool) -> None:
        self._deadman = bool(msg.data)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _safe_cmd(self, msg: TwistStamped):
        return (
            clamp(float(msg.twist.linear.x), self._max_vx),
            clamp(float(msg.twist.linear.y), self._max_vy),
            clamp(float(msg.twist.linear.z), self._max_vz),
            clamp(float(msg.twist.angular.z), self._max_yaw),
        )

    @staticmethod
    def _channel_index(channel_number_1_based: int) -> int:
        return max(0, min(17, int(channel_number_1_based) - 1))

    def _to_pwm(self, neutral: int, value: float, scale: float) -> int:
        pwm = int(round(neutral + (value * scale)))
        return max(self._min_pwm, min(self._max_pwm, pwm))

    def _publish_rc(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        if self._rc_pub is None or self._rc_msg_type is None:
            return

        msg = self._rc_msg_type()
        msg.channels = [self._rc_msg_type.CHAN_NOCHANGE] * len(msg.channels)
        msg.channels[self._channel_index(self._ch_forward)] = self._to_pwm(self._neutral_pwm, vx, self._scale_forward)
        msg.channels[self._channel_index(self._ch_lateral)] = self._to_pwm(self._neutral_pwm, vy, self._scale_lateral)
        msg.channels[self._channel_index(self._ch_vertical)] = self._to_pwm(self._neutral_pwm, vz, self._scale_vertical)
        msg.channels[self._channel_index(self._ch_yaw)] = self._to_pwm(self._neutral_pwm, yaw, self._scale_yaw)
        self._rc_pub.publish(msg)

    def _publish_rc_neutral(self) -> None:
        if self._rc_pub is None or self._rc_msg_type is None:
            return
        msg = self._rc_msg_type()
        msg.channels = [self._rc_msg_type.CHAN_NOCHANGE] * len(msg.channels)
        msg.channels[self._channel_index(self._ch_forward)] = self._neutral_pwm
        msg.channels[self._channel_index(self._ch_lateral)] = self._neutral_pwm
        msg.channels[self._channel_index(self._ch_vertical)] = self._neutral_pwm
        msg.channels[self._channel_index(self._ch_yaw)] = self._neutral_pwm
        self._rc_pub.publish(msg)

    def _publish_setpoint_vel(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        if self._vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw
        self._vel_pub.publish(msg)

    def _throttled_log(self, text: str) -> None:
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 >= (1.0 / self._log_hz):
            self._last_log_time = now
            self.get_logger().info(text)

    def _watchdog(self) -> None:
        now = self.get_clock().now()
        stale = (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout_s
        deadman_ok = (not self._deadman_required) or self._deadman

        active = (self._last_cmd is not None) and (not stale) and deadman_ok
        if active:
            vx, vy, vz, yaw = self._safe_cmd(self._last_cmd)
        else:
            vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0

        if self._backend == 'rc_override':
            if active:
                self._publish_rc(vx, vy, vz, yaw)
            else:
                self._publish_rc_neutral()
        elif self._backend == 'setpoint_velocity':
            self._publish_setpoint_vel(vx, vy, vz, yaw)

        self._throttled_log(
            f'MAVROS_BRIDGE backend={self._backend} active={active} deadman={self._deadman} stale={stale} '
            f'vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} yaw={yaw:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMavrosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
