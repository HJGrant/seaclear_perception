import csv
from pathlib import Path

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation


class CsvLoggerNode(Node):
    def __init__(self):
        super().__init__('csv_logger_node')
        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('csv_path', '/tmp/sc2_visual_servo_mvp.csv')

        target_topic = str(self.get_parameter('target_topic').value)
        cmd_topic = str(self.get_parameter('cmd_topic').value)
        csv_path = Path(str(self.get_parameter('csv_path').value))
        csv_path.parent.mkdir(parents=True, exist_ok=True)

        self._f = csv_path.open('w', newline='')
        self._writer = csv.writer(self._f)
        self._writer.writerow(['time_ns', 'type', 'u', 'v', 'w', 'h', 'depth_m', 'vx', 'vy', 'vz', 'yaw_rate'])

        self.create_subscription(TargetObservation, target_topic, self._target_cb, 50)
        self.create_subscription(TwistStamped, cmd_topic, self._cmd_cb, 50)

        self.get_logger().info(f'CSV logger writing to {csv_path}')

    def _stamp_ns(self):
        return self.get_clock().now().nanoseconds

    def _target_cb(self, msg: TargetObservation):
        self._writer.writerow([
            self._stamp_ns(),
            'target',
            msg.u,
            msg.v,
            msg.w,
            msg.h,
            msg.depth_m,
            '',
            '',
            '',
            '',
        ])
        self._f.flush()

    def _cmd_cb(self, msg: TwistStamped):
        self._writer.writerow([
            self._stamp_ns(),
            'cmd',
            '',
            '',
            '',
            '',
            '',
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.z,
        ])
        self._f.flush()

    def destroy_node(self):
        try:
            self._f.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsvLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
