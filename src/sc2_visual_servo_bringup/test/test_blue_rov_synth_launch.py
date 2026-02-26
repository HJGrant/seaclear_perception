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
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('sc2_visual_servo_bringup'),
        'launch',
        'blue_rov_synth_vs.launch.py',
    )

    dut = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'pattern': 'fixed_offsets',
            'publish_target_obs_direct': 'true',
            'controller_mode': 'bbox_approach',
            'mavros_backend': 'dry_run',
            'deadman': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest(),
    ])


class TestBlueRovSynthLaunch(unittest.TestCase):
    def test_cmd_and_status_publish(self):
        rclpy.init()
        node = rclpy.create_node('blue_rov_synth_smoke_test_sub')

        got_cmd = {'ok': False}
        got_status = {'ok': False}

        def _cmd_cb(msg: TwistStamped):
            if abs(msg.twist.angular.z) > 1e-6 or abs(msg.twist.linear.x) > 1e-6:
                got_cmd['ok'] = True

        def _status_cb(msg: String):
            if 'TRACKING_BBOX_APPROACH' in msg.data:
                got_status['ok'] = True

        node.create_subscription(TwistStamped, '/sc2/visual_servo/cmd_vel_body', _cmd_cb, 10)
        node.create_subscription(String, '/sc2/visual_servo/status', _status_cb, 10)

        deadline = time.time() + 8.0
        while time.time() < deadline and not (got_cmd['ok'] and got_status['ok']):
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_node()
        rclpy.try_shutdown()

        self.assertTrue(got_cmd['ok'])
        self.assertTrue(got_status['ok'])
