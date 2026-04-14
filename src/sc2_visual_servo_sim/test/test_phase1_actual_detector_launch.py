import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest
import rclpy
from vision_msgs.msg import Detection2D, Detection2DArray


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('sc2_visual_servo_sim'),
        'launch',
        'simulator_phase1_actual_detector.launch.py',
    )

    dut = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
    )

    return launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest(),
    ])


class TestPhase1ActualDetectorLaunch(unittest.TestCase):
    def test_detector_and_target_point(self):
        rclpy.init()
        node = rclpy.create_node('phase1_actual_detector_sub')

        got = {
            'detections': False,
            'best': False,
            'target_point': False,
        }

        def _detections_cb(msg: Detection2DArray):
            if msg.detections:
                got['detections'] = True

        def _best_cb(msg: Detection2D):
            if msg.bbox.size_x > 1.0:
                got['best'] = True

        def _tp_cb(msg: PoseStamped):
            if abs(msg.pose.position.y) > 1e-6:
                got['target_point'] = True

        node.create_subscription(Detection2DArray, '/detections_output', _detections_cb, 10)
        node.create_subscription(Detection2D, '/best_detection', _best_cb, 10)
        node.create_subscription(PoseStamped, '/sc2/visual_servo/target_point', _tp_cb, 10)

        deadline = time.time() + 10.0
        while time.time() < deadline and not all(got.values()):
            rclpy.spin_once(node, timeout_sec=0.2)

        node.destroy_node()
        rclpy.try_shutdown()

        self.assertTrue(got['detections'])
        self.assertTrue(got['best'])
        self.assertTrue(got['target_point'])
