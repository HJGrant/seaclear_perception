import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
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


def _detection_msg(u: float, v: float, w: float = 60.0, h: float = 40.0, frame_id='oak_rgb_camera_optical_frame'):
    msg = Detection2D()
    msg.header.frame_id = frame_id
    msg.bbox.center.position.x = float(u)
    msg.bbox.center.position.y = float(v)
    msg.bbox.size_x = float(w)
    msg.bbox.size_y = float(h)
    return msg


def _cloud_msg(
    x_img: np.ndarray,
    y_img: np.ndarray,
    z_img: np.ndarray,
    frame_id='oak_rgb_camera_optical_frame',
):
    height, width = z_img.shape
    stacked = np.zeros((height, width, 3), dtype=np.float32)
    stacked[:, :, 0] = x_img.astype(np.float32)
    stacked[:, :, 1] = y_img.astype(np.float32)
    stacked[:, :, 2] = z_img.astype(np.float32)

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = int(height)
    msg.width = int(width)
    msg.is_bigendian = False
    msg.is_dense = False
    msg.point_step = 12
    msg.row_step = int(width * msg.point_step)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = stacked.tobytes()
    return msg


def _make_node(params):
    node = TargetPointEstimatorNode(parameter_overrides=params)
    node._target_point_pub = _PublisherCapture()
    node._status_pub = _PublisherCapture()
    return node


def _status_texts(node):
    return [msg.data for msg in node._status_pub.messages if isinstance(msg, String)]


def _pose_messages(node):
    return [msg for msg in node._target_point_pub.messages if isinstance(msg, PoseStamped)]


def test_target_point_estimator_publishes_pose_from_bbox_depth_roi():
    rclpy.init()
    params = [
        Parameter('source_mode', value='bbox_depth_roi'),
        Parameter('target_bounding_box', value='/test/grasp/best_detection'),
        Parameter('depth_topic', value='/test/grasp/depth'),
        Parameter('camera_info_topic', value='/test/grasp/camera_info'),
        Parameter('target_point_topic', value='/test/target_point/pose'),
        Parameter('min_valid_points', value=10),
        Parameter('num_nearest_points', value=25),
        Parameter('max_search_radius_px', value=20.0),
        Parameter('use_uv_median_depth', value=True),
        Parameter('hold_last_valid', value=False),
        Parameter('enable_temporal_filter', value=False),
    ]
    node = _make_node(params)

    depth = np.full((120, 160), 4000, dtype=np.uint16)
    depth[50:71, 70:91] = 1500
    node._camera_info_cb(_camera_info_msg())
    node._depth_cb(_depth_msg(depth))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 1
    assert poses[-1].header.frame_id == 'oak_rgb_camera_optical_frame'
    assert 1.3 < poses[-1].pose.position.y < 1.7
    assert _status_texts(node)[-1] == 'TRACKING_TARGET_POINT'

    node.destroy_node()
    rclpy.try_shutdown()


def test_temporal_filter_reduces_jitter():
    rclpy.init()
    common = [
        Parameter('source_mode', value='bbox_depth_roi'),
        Parameter('hold_last_valid', value=False),
        Parameter('min_valid_points', value=5),
        Parameter('num_nearest_points', value=9),
        Parameter('max_search_radius_px', value=20.0),
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
    off_var = float(np.var([pose.pose.position.z for pose in off_poses]))
    on_var = float(np.var([pose.pose.position.z for pose in on_poses]))

    assert len(off_poses) == len(offsets)
    assert len(on_poses) == len(offsets)
    assert on_var < off_var

    for node in (off_node, on_node):
        node.destroy_node()
    rclpy.try_shutdown()


def test_depth_jump_reject_holds_last_valid():
    rclpy.init()
    params = [
        Parameter('source_mode', value='bbox_depth_roi'),
        Parameter('hold_last_valid', value=True),
        Parameter('enable_temporal_filter', value=True),
        Parameter('reject_large_jumps', value=True),
        Parameter('max_depth_jump_m', value=0.2),
        Parameter('min_valid_points', value=5),
    ]
    node = _make_node(params)
    node._camera_info_cb(_camera_info_msg())

    depth_nominal = np.full((120, 160), 1500, dtype=np.uint16)
    depth_jump = np.full((120, 160), 3000, dtype=np.uint16)

    node._depth_cb(_depth_msg(depth_nominal))
    node._target_obs_cb(_detection_msg(80.0, 60.0))
    node._depth_cb(_depth_msg(depth_jump))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 2
    assert abs(poses[-1].pose.position.y - poses[-2].pose.position.y) < 0.05
    assert _status_texts(node)[-1] == 'REJECTED_LARGE_JUMP'

    node.destroy_node()
    rclpy.try_shutdown()


def test_nvblox_back_projected_depth_publishes_pose():
    rclpy.init()
    params = [
        Parameter('source_mode', value='nvblox_back_projected_depth'),
        Parameter('hold_last_valid', value=False),
        Parameter('enable_temporal_filter', value=False),
        Parameter('min_valid_points', value=5),
    ]
    node = _make_node(params)
    node._camera_info_cb(_camera_info_msg())

    height, width = 120, 160
    uu = np.tile(np.arange(width, dtype=np.float32), (height, 1))
    vv = np.tile(np.arange(height, dtype=np.float32).reshape(-1, 1), (1, width))
    z = np.full((height, width), 1.5, dtype=np.float32)
    x = ((uu - 80.0) / 600.0) * z
    y = ((vv - 60.0) / 600.0) * z

    node._nvblox_cloud_cb(_cloud_msg(x, y, z))
    node._target_obs_cb(_detection_msg(80.0, 60.0))

    poses = _pose_messages(node)
    assert len(poses) == 1
    assert poses[-1].header.frame_id == 'oak_rgb_camera_optical_frame'
    assert 1.3 < poses[-1].pose.position.y < 1.7
    assert _status_texts(node)[-1] == 'TRACKING_TARGET_POINT'

    node.destroy_node()
    rclpy.try_shutdown()
