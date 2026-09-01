'''Drives the vehicle through the fixed calibration maneuver sequence used to
validate/tune the centroid Kalman filter's image-Jacobian prediction:

    settle -> hold_still -> translate_x -> translate_y -> yaw -> stop

Same command interface as velocity_control_test.py (TF-transformed
sc2_base_link -> base_link velocity, published to both
/mavros/setpoint_velocity/cmd_vel and the console_signals Vector6 topic), but
runs the whole sequence automatically instead of one axis/duration per
invocation, and additionally publishes a phase-name marker topic every tick
so the resulting bag's phase windows can be extracted programmatically later
(by plot_centroid_kf_bag.py or a calibration-report script) instead of by
eyeballing timestamps.

Run this under scripts/record_kf_calibration_bag.sh, which wraps it with
`ros2 bag record` for the topics the calibration analysis needs.
'''
from dataclasses import dataclass, field
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # noqa: F401 -- required for do_transform_vector3
from state_estimation.msg import Vector6


@dataclass
class ManeuverPhase:
    name: str
    duration_s: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wz: float = 0.0


def _build_phases(
    settle_s: float,
    hold_s: float,
    translate_x_s: float,
    translate_y_s: float,
    yaw_s: float,
    stop_s: float,
    translate_velocity: float,
    yaw_rate: float,
) -> List[ManeuverPhase]:
    return [
        ManeuverPhase('settle', settle_s),
        ManeuverPhase('hold_still', hold_s),
        ManeuverPhase('translate_x', translate_x_s, vx=translate_velocity),
        ManeuverPhase('translate_y', translate_y_s, vy=translate_velocity),
        ManeuverPhase('yaw', yaw_s, wz=yaw_rate),
        ManeuverPhase('stop', stop_s),
    ]


class CalibrationManeuverNode(Node):
    def __init__(self):
        super().__init__('calibration_maneuver_node')

        self.declare_parameter('source_frame_id', 'sc2_base_link')
        self.declare_parameter('target_frame_id', 'base_link')
        self.declare_parameter('cmd_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('cmd_topic_vec6', '/sc2/velocity_controller_node/outer/console_signals')
        self.declare_parameter('phase_marker_topic', '/sc2/visual_servo/calibration_phase')
        self.declare_parameter('rate_hz', 20.0)

        self.declare_parameter('settle_duration_sec', 2.0)
        self.declare_parameter('hold_duration_sec', 60.0)
        self.declare_parameter('translate_x_duration_sec', 5.0)
        self.declare_parameter('translate_y_duration_sec', 5.0)
        self.declare_parameter('yaw_duration_sec', 30.0)
        self.declare_parameter('stop_duration_sec', 1.0)
        self.declare_parameter('translate_velocity_mps', 0.2)
        self.declare_parameter('yaw_rate_rad_s', 0.3)

        self._source_frame_id = str(self.get_parameter('source_frame_id').value)
        self._target_frame_id = str(self.get_parameter('target_frame_id').value)

        self._phases = _build_phases(
            settle_s=float(self.get_parameter('settle_duration_sec').value),
            hold_s=float(self.get_parameter('hold_duration_sec').value),
            translate_x_s=float(self.get_parameter('translate_x_duration_sec').value),
            translate_y_s=float(self.get_parameter('translate_y_duration_sec').value),
            yaw_s=float(self.get_parameter('yaw_duration_sec').value),
            stop_s=float(self.get_parameter('stop_duration_sec').value),
            translate_velocity=float(self.get_parameter('translate_velocity_mps').value),
            yaw_rate=float(self.get_parameter('yaw_rate_rad_s').value),
        )
        self._phase_idx = 0
        self._phase_start_time = None
        self._done = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        cmd_topic = str(self.get_parameter('cmd_topic').value)
        cmd_topic_vec6 = str(self.get_parameter('cmd_topic_vec6').value)
        phase_marker_topic = str(self.get_parameter('phase_marker_topic').value)
        self._cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self._cmd_vec6_pub = self.create_publisher(Vector6, cmd_topic_vec6, 10)
        self._phase_marker_pub = self.create_publisher(String, phase_marker_topic, 10)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        total_s = sum(p.duration_s for p in self._phases)
        self.get_logger().info(
            f'calibration_maneuver_node: {len(self._phases)} phases, {total_s:.1f}s total -- '
            + ', '.join(f'{p.name}={p.duration_s:.1f}s' for p in self._phases)
        )

    def _tick(self) -> None:
        if self._done:
            return

        now = self.get_clock().now()
        if self._phase_start_time is None:
            self._phase_start_time = now
            self.get_logger().info(f'Starting phase: {self._phases[0].name}')

        elapsed_in_phase = (now - self._phase_start_time).nanoseconds * 1e-9
        phase = self._phases[self._phase_idx]

        if elapsed_in_phase >= phase.duration_s:
            self._phase_idx += 1
            self._phase_start_time = now
            if self._phase_idx >= len(self._phases):
                self.get_logger().info('Calibration maneuver complete.')
                self._publish_command(0.0, 0.0, 0.0, 0.0, now)
                self._done = True
                raise SystemExit
            phase = self._phases[self._phase_idx]
            self.get_logger().info(f'Starting phase: {phase.name} ({phase.duration_s:.1f}s)')

        marker = String()
        marker.data = phase.name
        self._phase_marker_pub.publish(marker)

        self._publish_command(phase.vx, phase.vy, phase.vz, phase.wz, now)

    def _publish_command(self, vx: float, vy: float, vz: float, wz: float, stamp) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame_id, self._source_frame_id, rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF: {ex}', throttle_duration_sec=2.0)
            return

        v_in = Vector3Stamped()
        v_in.header.frame_id = self._source_frame_id
        v_in.header.stamp = stamp.to_msg()
        v_in.vector.x = vx
        v_in.vector.y = vy
        v_in.vector.z = vz

        w_in = Vector3Stamped()
        w_in.header.frame_id = self._source_frame_id
        w_in.header.stamp = stamp.to_msg()
        w_in.vector.z = wz

        v_out = tf2_geometry_msgs.do_transform_vector3(v_in, transform)
        w_out = tf2_geometry_msgs.do_transform_vector3(w_in, transform)

        cmd = TwistStamped()
        cmd.header.stamp = stamp.to_msg()
        cmd.header.frame_id = self._target_frame_id
        cmd.twist.linear = v_out.vector
        cmd.twist.angular = w_out.vector
        self._cmd_pub.publish(cmd)

        vec6 = Vector6()
        vec6.linear = v_out.vector
        vec6.angular = w_out.vector
        self._cmd_vec6_pub.publish(vec6)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationManeuverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
