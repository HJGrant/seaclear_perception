import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('sc2_visual_servo_bringup'),
        'launch',
        'mvp_sim.launch.py',
    )

    dut = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={'use_stub_target': 'true', 'controller_mode': 'ibvs'}.items(),
    )

    return launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMvpSimLaunch(unittest.TestCase):
    def test_cmd_topic_publishes(self):
        rclpy.init()
        node = rclpy.create_node('mvp_sim_smoke_test_sub')

        got_msg = {'ok': False}

        def _cb(msg: TwistStamped):
            got_msg['ok'] = True

        node.create_subscription(TwistStamped, '/sc2/visual_servo/cmd_vel_body', _cb, 10)

        deadline = time.time() + 8.0
        while time.time() < deadline and not got_msg['ok']:
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_node()
        rclpy.shutdown()

        self.assertTrue(got_msg['ok'])
