import time

import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from sc2_visual_servo_perception.grasp_pose_estimator_node import GraspPoseEstimatorNode


class SyntheticInputs(Node):
    def __init__(self):
        super().__init__('synthetic_grasp_pose_inputs')
        self._obs_pub = self.create_publisher(TargetObservation, '/test/grasp/target_obs', 10)
        self._depth_pub = self.create_publisher(Image, '/test/grasp/depth', 10)
        self._cam_pub = self.create_publisher(CameraInfo, '/test/grasp/camera_info', 10)
        self.create_timer(0.05, self._tick)

        self._depth = np.full((120, 160), 4000, dtype=np.uint16)
        self._depth[50:71, 70:91] = 1500

    def _depth_msg(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        msg.height = int(self._depth.shape[0])
        msg.width = int(self._depth.shape[1])
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step = int(self._depth.shape[1] * 2)
        msg.data = self._depth.tobytes()
        return msg

    def _camera_info_msg(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        msg.width = 160
        msg.height = 120
        # fx, 0, cx, 0, fy, cy, 0, 0, 1
        msg.k = [600.0, 0.0, 80.0, 0.0, 600.0, 60.0, 0.0, 0.0, 1.0]
        return msg

    def _target_obs_msg(self):
        msg = TargetObservation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        msg.track_id = 1
        msg.u = 80.0
        msg.v = 60.0
        msg.w = 60.0
        msg.h = 40.0
        msg.depth_m = float('nan')
        msg.confidence = 0.95
        msg.source = 'test'
        return msg

    def _tick(self):
        self._depth_pub.publish(self._depth_msg())
        self._cam_pub.publish(self._camera_info_msg())
        self._obs_pub.publish(self._target_obs_msg())


def test_grasp_pose_estimator_publishes_pose_from_bbox_depth_roi():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)

    params = [
        Parameter('source_mode', value='bbox_depth_roi'),
        Parameter('target_obs_topic', value='/test/grasp/target_obs'),
        Parameter('depth_topic', value='/test/grasp/depth'),
        Parameter('camera_info_topic', value='/test/grasp/camera_info'),
        Parameter('grasp_pose_topic', value='/test/grasp/pose'),
        Parameter('min_valid_points', value=10),
        Parameter('num_nearest_points', value=25),
        Parameter('max_search_radius_px', value=20.0),
        Parameter('use_uv_median_depth', value=True),
        Parameter('hold_last_valid', value=False),
    ]

    estimator = GraspPoseEstimatorNode(parameter_overrides=params)

    inputs = SyntheticInputs()
    sink = Node('grasp_pose_sink')

    received = []

    def _pose_cb(msg):
        received.append(msg)

    sink.create_subscription(PoseStamped, '/test/grasp/pose', _pose_cb, 10)

    executor.add_node(estimator)
    executor.add_node(inputs)
    executor.add_node(sink)

    deadline = time.time() + 5.0
    while time.time() < deadline and len(received) < 3:
        executor.spin_once(timeout_sec=0.1)

    for n in (sink, inputs, estimator):
        executor.remove_node(n)
        n.destroy_node()
    rclpy.try_shutdown()

    assert len(received) >= 1
    assert received[-1].header.frame_id == 'oak_rgb_camera_optical_frame'
    assert 1.3 < received[-1].pose.position.z < 1.7
