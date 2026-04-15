import time

import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D

from sc2_visual_servo_perception.target_point_estimator_node import TargetPointEstimatorNode


class _PublisherCapture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _camera_info_msg(frame_id='oak_rgb_camera_optical_frame'):
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = 160
    msg.height = 120
    msg.k = [600.0, 0.0, 80.0, 0.0, 600.0, 60.0, 0.0, 0.0, 1.0]
    return msg


def _depth_msg(depth_mm: np.ndarray, frame_id='oak_rgb_camera_optical_frame'):
    msg = Image()
    msg.header.frame_id = frame_id
    msg.height = int(depth_mm.shape[0])
    msg.width = int(depth_mm.shape[1])
    msg.encoding = '16UC1'
    msg.is_bigendian = 0
    msg.step = int(depth_mm.shape[1] * 2)
    msg.data = depth_mm.astype(np.uint16).tobytes()
    return msg


def _detection_msg(u: float, v: float, w: float = 20.0, h: float = 20.0, frame_id='oak_rgb_camera_optical_frame'):
    msg = Detection2D()
    msg.header.frame_id = frame_id
    msg.bbox.center.position.x = float(u)
    msg.bbox.center.position.y = float(v)
    msg.bbox.size_x = float(w)
    msg.bbox.size_y = float(h)
    return msg


def _make_node(params):
    node = TargetPointEstimatorNode(parameter_overrides=params)
    node._target_point_pub = _PublisherCapture()
    node._status_pub = _PublisherCapture()
    return node


def _base_params():
    return [
        Parameter('source_mode', value='bbox_depth_roi'),
        Parameter('target_bounding_box', value='/test/grasp/best_detection'),
        Parameter('depth_topic', value='/test/grasp/depth'),
        Parameter('camera_info_topic', value='/test/grasp/camera_info'),
        Parameter('target_point_topic', value='/test/target_point/pose'),
        Parameter('min_valid_points', value=5),
        Parameter('num_nearest_points', value=9),
        Parameter('max_search_radius_px', value=20.0),
        Parameter('use_uv_median_depth', value=True),
    ]


def _status_texts(node):
    return [msg.data for msg in node._status_pub.messages if isinstance(msg, String)]


def _pose_messages(node):
    return [msg for msg in node._target_point_pub.messages if isinstance(msg, PoseStamped)]


def _shutdown(*nodes):
    for node in nodes:
        node.destroy_node()
    rclpy.try_shutdown()


def test_target_point_pose_fields_use_camera_offsets_and_z_range():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=False),
        Parameter('enable_temporal_filter', value=False),
    ])

    depth = np.full((120, 160), 1500, dtype=np.uint16)
    node._camera_info_cb(_camera_info_msg())
    node._depth_cb(_depth_msg(depth))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 1
    assert poses[-1].header.frame_id == 'oak_rgb_camera_optical_frame'
    assert abs(poses[-1].pose.position.x) < 0.01
    assert abs(poses[-1].pose.position.y) < 0.01
    assert 1.49 < poses[-1].pose.position.z < 1.51
    assert _status_texts(node)[-1] == 'TRACKING_TARGET_POINT'

    _shutdown(node)


def test_temporal_filter_reduces_pixel_jitter_in_metric_target_point():
    rclpy.init()
    common = _base_params() + [
        Parameter('hold_last_valid', value=False),
        Parameter('reject_large_jumps', value=False),
    ]
    off_node = _make_node(common + [Parameter('enable_temporal_filter', value=False)])
    on_node = _make_node(common + [
        Parameter('enable_temporal_filter', value=True),
        Parameter('filter_alpha_uv', value=0.25),
        Parameter('filter_alpha_depth', value=0.2),
        Parameter('filter_alpha_size', value=0.2),
    ])

    depth = np.full((120, 160), 1500, dtype=np.uint16)
    for node in (off_node, on_node):
        node._camera_info_cb(_camera_info_msg())
        node._depth_cb(_depth_msg(depth))

    offsets = [-8.0, 8.0, -6.0, 6.0, -7.0, 7.0, -5.0, 5.0]
    for offset in offsets:
        msg = _detection_msg(80.0 + offset, 60.0 + 0.5 * offset)
        off_node._target_obs_cb(msg)
        on_node._target_obs_cb(msg)

    off_poses = _pose_messages(off_node)
    on_poses = _pose_messages(on_node)
    off_x_var = float(np.var([pose.pose.position.x for pose in off_poses]))
    on_x_var = float(np.var([pose.pose.position.x for pose in on_poses]))

    assert len(off_poses) == len(offsets)
    assert len(on_poses) == len(offsets)
    assert on_x_var < off_x_var

    _shutdown(off_node, on_node)


def test_large_depth_jump_is_rejected_and_holds_last_valid_pose():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=True),
        Parameter('enable_temporal_filter', value=True),
        Parameter('reject_large_jumps', value=True),
        Parameter('max_depth_jump_m', value=0.2),
    ])
    node._camera_info_cb(_camera_info_msg())

    node._depth_cb(_depth_msg(np.full((120, 160), 1500, dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))
    node._depth_cb(_depth_msg(np.full((120, 160), 3000, dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 2
    assert abs(poses[-1].pose.position.z - poses[-2].pose.position.z) < 0.05
    assert _status_texts(node)[-1] == 'REJECTED_LARGE_JUMP'

    _shutdown(node)


def test_large_bbox_jump_is_rejected_and_holds_last_valid_pose():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=True),
        Parameter('enable_temporal_filter', value=True),
        Parameter('reject_large_jumps', value=True),
        Parameter('max_uv_jump_px', value=30.0),
    ])
    node._camera_info_cb(_camera_info_msg())
    node._depth_cb(_depth_msg(np.full((120, 160), 1500, dtype=np.uint16)))

    node._target_obs_cb(_detection_msg(80.0, 60.0))
    node._target_obs_cb(_detection_msg(130.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 2
    assert abs(poses[-1].pose.position.z - poses[-2].pose.position.z) < 0.05
    assert _status_texts(node)[-1] == 'REJECTED_LARGE_JUMP'

    _shutdown(node)


def test_invalid_depth_short_gap_holds_last_valid_pose():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=True),
        Parameter('max_hold_ms', value=500.0),
        Parameter('enable_temporal_filter', value=False),
    ])
    node._camera_info_cb(_camera_info_msg())

    node._depth_cb(_depth_msg(np.full((120, 160), 1500, dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))
    node._depth_cb(_depth_msg(np.zeros((120, 160), dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 2
    assert abs(poses[-1].pose.position.z - poses[-2].pose.position.z) < 0.05
    assert _status_texts(node)[-1] == 'HOLD_LAST_VALID'

    _shutdown(node)


def test_stale_depth_short_gap_holds_last_valid_pose():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=True),
        Parameter('max_hold_ms', value=500.0),
        Parameter('max_depth_age_ms', value=1.0),
        Parameter('enable_temporal_filter', value=False),
    ])
    node._camera_info_cb(_camera_info_msg())

    node._depth_cb(_depth_msg(np.full((120, 160), 1500, dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))
    time.sleep(0.01)
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 2
    assert abs(poses[-1].pose.position.z - poses[-2].pose.position.z) < 0.05
    assert _status_texts(node)[-1] == 'HOLD_LAST_VALID_DEPTH_STALE'

    _shutdown(node)


def test_invalid_depth_after_hold_window_is_not_published():
    rclpy.init()
    node = _make_node(_base_params() + [
        Parameter('hold_last_valid', value=True),
        Parameter('max_hold_ms', value=1.0),
        Parameter('enable_temporal_filter', value=False),
    ])
    node._camera_info_cb(_camera_info_msg())

    node._depth_cb(_depth_msg(np.full((120, 160), 1500, dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))
    time.sleep(0.01)
    node._depth_cb(_depth_msg(np.zeros((120, 160), dtype=np.uint16)))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 1
    assert _status_texts(node)[-1] == 'NO_VALID_TARGET_POINT'

    _shutdown(node)
