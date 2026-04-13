import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

"""
usage examples:
`ros2 run sc2_visual_servo_controller axes_test --ros-args -p velocity:=0.3 -p sequence:="x:1.0,y:1.0"`
`ros2 run sc2_visual_servo_controller axes_test --ros-args -p velocity:=0.3 -p sequence:="x:1.0,-x:1.0,y:1.0,-y:1.0"`
"""


class Sc2VelocityPublisher(Node):
    def __init__(self):
        super().__init__('sc2_velocity_commander')
        
        # 1. Parameters
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('velocity', 0.5)
        self.declare_parameter('axis', 'x')
        self.declare_parameter('duration', 1.0)
        self.declare_parameter('sequence', 'x:1.0,x:1.0')

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self.vel_pub = self.create_publisher(
            TwistStamped, 
            self._cmd_topic,
            10
        )
        
        self.timer = self.create_timer(0.05, self.publish_velocity)
        self.start_time = self.get_clock().now()
        self.sequence = self._parse_sequence(str(self.get_parameter('sequence').value))
        
        self.get_logger().info(
            f'Axis test initialized cmd_topic={self._cmd_topic} body_frame_id={self._body_frame_id} sequence={self.sequence}'
        )

    def _parse_sequence(self, raw: str):
        sequence = []
        for item in raw.split(','):
            token = item.strip()
            if not token:
                continue
            try:
                axis, duration_text = token.split(':', 1)
                axis = axis.strip().lower()
                duration = float(duration_text.strip())
            except ValueError as exc:
                raise ValueError(
                    f'Invalid sequence item "{token}". Expected format axis:seconds'
                ) from exc

            if axis not in {'x', 'y', 'z', '-x', '-y', '-z', 'stop'}:
                raise ValueError(
                    f'Invalid axis "{axis}". Use x, y, z, -x, -y, -z, or stop.'
                )
            if duration < 0.0:
                raise ValueError(f'Duration must be non-negative: "{token}"')

            sequence.append((axis, duration))

        if sequence:
            return sequence

        axis = str(self.get_parameter('axis').value).strip().lower()
        duration = float(self.get_parameter('duration').value)
        return [(axis, duration)]

    def _active_step(self, elapsed_time: float):
        accumulated = 0.0
        for axis, duration in self.sequence:
            if elapsed_time < accumulated + duration:
                return axis
            accumulated += duration
        if elapsed_time < accumulated + 1.0:
            return 'stop'
        return None

    def publish_velocity(self):
        curr_time = self.get_clock().now()
        elapsed_time = (curr_time - self.start_time).nanoseconds / 1e9

        axis = self._active_step(elapsed_time)
        vel = float(self.get_parameter('velocity').value)
        msg = TwistStamped()
        msg.header.stamp = curr_time.to_msg()
        msg.header.frame_id = self._body_frame_id

        if axis == 'x':
            msg.twist.linear.x = vel
        elif axis == '-x':
            msg.twist.linear.x = -vel
        elif axis == 'y':
            msg.twist.linear.y = vel
        elif axis == '-y':
            msg.twist.linear.y = -vel
        elif axis == 'z':
            msg.twist.linear.z = vel
        elif axis == '-z':
            msg.twist.linear.z = -vel
        elif axis is None:
            raise SystemExit

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Sc2VelocityPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
