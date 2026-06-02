import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose

from sc2_visual_servo_perception.target_observer_node import TargetObserverNode


class SyntheticPerceptionPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_perception_publisher')
        self._det_pub = self.create_publisher(Detection2DArray, '/test/detections', 10)
        self._depth_pub = self.create_publisher(Image, '/test/depth', 10)
        self.create_timer(0.05, self._tick)

        self._depth = np.full((120, 160), 1500, dtype=np.uint16)
        self._depth[40:90, 50:120] = 1200

    def _depth_msg(self):
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        msg.height = int(self._depth.shape[0])
        msg.width = int(self._depth.shape[1])
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step = int(self._depth.shape[1] * 2)
        msg.data = self._depth.tobytes()
        return msg

    def _det_msg(self):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'

        det = Detection2D()
        det.bbox.center.position.x = 80.0
        det.bbox.center.position.y = 60.0
        det.bbox.size_x = 60.0
        det.bbox.size_y = 40.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'bottle'
        hyp.hypothesis.score = 0.95
        det.results.append(hyp)

        msg.detections.append(det)
        return msg

    def _tick(self):
        self._depth_pub.publish(self._depth_msg())
        self._det_pub.publish(self._det_msg())


class SequencedDetectionPublisher(Node):
    def __init__(self):
        super().__init__('sequenced_detection_publisher')
        self._det_pub = self.create_publisher(Detection2DArray, '/test/tracker_detections', 10)
        self._tick_count = 0
        self.create_timer(0.05, self._tick)

    def _make_msg(self, *, score: float, include_detection: bool = True):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_rgb_camera_optical_frame'
        if not include_detection:
            return msg

        det = Detection2D()
        det.bbox.center.position.x = 80.0 + min(self._tick_count, 3)
        det.bbox.center.position.y = 60.0
        det.bbox.size_x = 60.0
        det.bbox.size_y = 40.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = '3'
        hyp.hypothesis.score = score
        det.results.append(hyp)
        msg.detections.append(det)
        return msg

    def _tick(self):
        self._tick_count += 1
        if self._tick_count <= 3:
            self._det_pub.publish(self._make_msg(score=0.92))
        elif self._tick_count <= 6:
            self._det_pub.publish(self._make_msg(score=0.35))
        else:
            self._det_pub.publish(self._make_msg(score=0.0, include_detection=False))


def test_target_observer_publishes_from_synthetic_detection_and_depth():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)

    params = [
        Parameter('detections_topic', value='/test/detections'),
        Parameter('depth_topic', value='/test/depth'),
        Parameter('output_topic', value='/test/target_obs'),
        Parameter('target_class', value='bottle'),
        Parameter('min_confidence', value=0.1),
        Parameter('pick_strategy', value='highest_confidence'),
        Parameter('use_depth', value=True),
        Parameter('max_age_ms', value=500),
    ]

    observer = TargetObserverNode(parameter_overrides=params)
    pub_node = SyntheticPerceptionPublisher()
    sink_node = Node('target_obs_sink')

    received = []

    def _cb(msg: TargetObservation):
        received.append(msg)

    sink_node.create_subscription(TargetObservation, '/test/target_obs', _cb, 10)

    executor.add_node(observer)
    executor.add_node(pub_node)
    executor.add_node(sink_node)

    deadline = time.time() + 5.0
    while time.time() < deadline and len(received) < 3:
        executor.spin_once(timeout_sec=0.1)

    for n in (sink_node, pub_node, observer):
        executor.remove_node(n)
        n.destroy_node()
    rclpy.try_shutdown()

    assert len(received) >= 3
    assert all(m.confidence > 0.1 for m in received)
    assert any(np.isfinite(m.depth_m) and 0.5 < m.depth_m < 2.0 for m in received)


def test_target_observer_confirms_recovers_and_holds_selected_target():
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)

    params = [
        Parameter('detections_topic', value='/test/tracker_detections'),
        Parameter('output_topic', value='/test/tracker_target_obs'),
        Parameter('target_class', value='3'),
        Parameter('min_confidence', value=0.8),
        Parameter('low_confidence', value=0.2),
        Parameter('pick_strategy', value='nearest_track'),
        Parameter('use_depth', value=False),
        Parameter('n_confirm', value=2),
        Parameter('t_stale_sec', value=1.0),
        Parameter('base_gate_px', value=80.0),
        Parameter('enable_motion_aware_gate', value=False),
        Parameter('max_age_ms', value=500),
    ]

    observer = TargetObserverNode(parameter_overrides=params)
    pub_node = SequencedDetectionPublisher()
    sink_node = Node('tracker_target_obs_sink')

    received = []

    def _cb(msg: TargetObservation):
        received.append(msg)

    sink_node.create_subscription(TargetObservation, '/test/tracker_target_obs', _cb, 10)

    executor.add_node(observer)
    executor.add_node(pub_node)
    executor.add_node(sink_node)

    deadline = time.time() + 5.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)
        if (
            any(m.selected_valid and m.track_state == 'CONFIRMED' for m in received)
            and any(m.reason_code == 'low_confidence_recovery' for m in received)
            and any((not m.selected_valid) and m.track_state == 'HOLD' for m in received)
        ):
            break

    for n in (sink_node, pub_node, observer):
        executor.remove_node(n)
        n.destroy_node()
    rclpy.try_shutdown()

    assert any(m.selected_valid and m.track_state == 'CONFIRMED' for m in received)
    assert any(m.reason_code == 'low_confidence_recovery' for m in received)
    assert any((not m.selected_valid) and m.track_state == 'HOLD' for m in received)
