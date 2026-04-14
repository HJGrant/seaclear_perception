import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('sc2_visual_servo_sim'),
        'launch',
        'simulator_phase1_dynamic.launch.py',
    )

    dut = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
    )

    return launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest(),
    ])


class TestPhase1DynamicLaunch(unittest.TestCase):
    def test_topics_flow(self):
        rclpy.init()
        node = rclpy.create_node('phase1_dynamic_sub')

        got = {
            'cmd': False,
            'target_point': False,
            'odom': False,
        }

        def _cmd_cb(msg: TwistStamped):
            if abs(msg.twist.linear.x) > 1e-6 or abs(msg.twist.linear.y) > 1e-6:
                got['cmd'] = True

        def _tp_cb(msg: PoseStamped):
            if abs(msg.pose.position.y) > 1e-6:
                got['target_point'] = True

        def _odom_cb(msg: Odometry):
            if abs(msg.pose.pose.position.z) > 1e-6:
                got['odom'] = True

        node.create_subscription(TwistStamped, '/sc2/visual_servo/cmd_vel_body', _cmd_cb, 10)
        node.create_subscription(PoseStamped, '/sc2/visual_servo/target_point', _tp_cb, 10)
        node.create_subscription(Odometry, '/sim/odom', _odom_cb, 10)

        deadline = time.time() + 10.0
        while time.time() < deadline and not all(got.values()):
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_node()
        rclpy.try_shutdown()

        self.assertTrue(got['cmd'])
        self.assertTrue(got['target_point'])
        self.assertTrue(got['odom'])
