import math
import random

import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class SyntheticBBoxPublisherNode(Node):
    def __init__(self):
        super().__init__('synthetic_bbox_publisher_node')

        self.declare_parameter('detections_topic', '/detections_output')
        self.declare_parameter('target_obs_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('publish_target_obs_direct', False)
        self.declare_parameter('pattern', 'fixed_offsets')
        self.declare_parameter('rate_hz', 15.0)

        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 720.0)
        self.declare_parameter('frame_id', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('class_id', 'bottle')
        self.declare_parameter('score', 0.95)
        self.declare_parameter('track_id', 1)

        self.declare_parameter('base_u_px', 640.0)
        self.declare_parameter('base_v_px', 360.0)
        self.declare_parameter('base_w_px', 120.0)
        self.declare_parameter('base_h_px', 80.0)
        self.declare_parameter('direct_depth_m', float('nan'))

        self.declare_parameter('fixed_offset_u_px', 220.0)
        self.declare_parameter('fixed_offset_v_px', 130.0)
        self.declare_parameter('fixed_hold_s', 2.0)

        self.declare_parameter('size_min_scale', 0.6)
        self.declare_parameter('size_max_scale', 1.8)
        self.declare_parameter('size_period_s', 10.0)

        self.declare_parameter('lissajous_amp_u_px', 260.0)
        self.declare_parameter('lissajous_amp_v_px', 170.0)
        self.declare_parameter('lissajous_freq_u_hz', 0.09)
        self.declare_parameter('lissajous_freq_v_hz', 0.07)
        self.declare_parameter('lissajous_phase_rad', 1.2)

        self.declare_parameter('walk_step_px', 16.0)
        self.declare_parameter('walk_bound_u_px', 300.0)
        self.declare_parameter('walk_bound_v_px', 200.0)

        self.declare_parameter('dropout_period_s', 0.0)
        self.declare_parameter('dropout_duration_s', 0.0)
        self.declare_parameter('loss_duration_s', 3.0)
        self.declare_parameter('reacquire_duration_s', 4.0)

        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._target_obs_topic = str(self.get_parameter('target_obs_topic').value)
        self._publish_direct = self._as_bool(self.get_parameter('publish_target_obs_direct').value)
        self._pattern = str(self.get_parameter('pattern').value).strip().lower()
        self._rate_hz = float(self.get_parameter('rate_hz').value)

        self._img_w = float(self.get_parameter('image_width').value)
        self._img_h = float(self.get_parameter('image_height').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._class_id = str(self.get_parameter('class_id').value)
        self._score = float(self.get_parameter('score').value)
        self._track_id = int(self.get_parameter('track_id').value)

        self._base_u = float(self.get_parameter('base_u_px').value)
        self._base_v = float(self.get_parameter('base_v_px').value)
        self._base_w = float(self.get_parameter('base_w_px').value)
        self._base_h = float(self.get_parameter('base_h_px').value)
        self._direct_depth = float(self.get_parameter('direct_depth_m').value)

        self._fixed_u = float(self.get_parameter('fixed_offset_u_px').value)
        self._fixed_v = float(self.get_parameter('fixed_offset_v_px').value)
        self._fixed_hold_s = max(0.1, float(self.get_parameter('fixed_hold_s').value))

        self._size_min = float(self.get_parameter('size_min_scale').value)
        self._size_max = float(self.get_parameter('size_max_scale').value)
        self._size_period = max(0.1, float(self.get_parameter('size_period_s').value))

        self._lis_amp_u = float(self.get_parameter('lissajous_amp_u_px').value)
        self._lis_amp_v = float(self.get_parameter('lissajous_amp_v_px').value)
        self._lis_fu = float(self.get_parameter('lissajous_freq_u_hz').value)
        self._lis_fv = float(self.get_parameter('lissajous_freq_v_hz').value)
        self._lis_phase = float(self.get_parameter('lissajous_phase_rad').value)

        self._walk_step = max(0.0, float(self.get_parameter('walk_step_px').value))
        self._walk_bound_u = max(1.0, float(self.get_parameter('walk_bound_u_px').value))
        self._walk_bound_v = max(1.0, float(self.get_parameter('walk_bound_v_px').value))

        self._dropout_period = max(0.0, float(self.get_parameter('dropout_period_s').value))
        self._dropout_duration = max(0.0, float(self.get_parameter('dropout_duration_s').value))
        self._loss_duration = max(0.0, float(self.get_parameter('loss_duration_s').value))
        self._reacq_duration = max(0.0, float(self.get_parameter('reacquire_duration_s').value))

        self._det_pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)
        self._obs_pub = self.create_publisher(TargetObservation, self._target_obs_topic, 10)

        self._rng = random.Random(7)
        self._t = 0.0
        self._walk_u = self._base_u
        self._walk_v = self._base_v

        period = 1.0 / max(1e-3, self._rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'synthetic_bbox_publisher_node pattern={self._pattern} '
            f'detections_topic={self._detections_topic} direct_target_obs={self._publish_direct}'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _clip_bbox(self, u: float, v: float, w: float, h: float):
        w = max(4.0, min(self._img_w, w))
        h = max(4.0, min(self._img_h, h))
        u = max(0.5 * w, min(self._img_w - 0.5 * w, u))
        v = max(0.5 * h, min(self._img_h - 0.5 * h, v))
        return u, v, w, h

    def _is_visible(self) -> bool:
        visible = True

        if self._pattern == 'loss_reacquire':
            cycle = self._loss_duration + self._reacq_duration
            if cycle > 0.0:
                phase = self._t % cycle
                visible = phase < self._reacq_duration

        if self._dropout_period > 0.0 and self._dropout_duration > 0.0:
            phase = self._t % self._dropout_period
            if phase < self._dropout_duration:
                visible = False

        return visible

    def _pattern_bbox(self):
        u = self._base_u
        v = self._base_v
        w = self._base_w
        h = self._base_h

        if self._pattern == 'fixed_offsets':
            idx = int(self._t / self._fixed_hold_s) % 4
            offsets = [
                (self._fixed_u, 0.0),
                (-self._fixed_u, 0.0),
                (0.0, self._fixed_v),
                (0.0, -self._fixed_v),
            ]
            du, dv = offsets[idx]
            u = self._base_u + du
            v = self._base_v + dv
        elif self._pattern == 'size_ramp':
            s = 0.5 * (1.0 + math.sin(2.0 * math.pi * (self._t / self._size_period)))
            scale = self._size_min + (self._size_max - self._size_min) * s
            w = self._base_w * scale
            h = self._base_h * scale
        elif self._pattern == 'lissajous':
            u = self._base_u + self._lis_amp_u * math.sin(2.0 * math.pi * self._lis_fu * self._t)
            v = self._base_v + self._lis_amp_v * math.sin(2.0 * math.pi * self._lis_fv * self._t + self._lis_phase)
        elif self._pattern == 'random_walk':
            self._walk_u += self._rng.uniform(-self._walk_step, self._walk_step)
            self._walk_v += self._rng.uniform(-self._walk_step, self._walk_step)
            self._walk_u = max(self._base_u - self._walk_bound_u, min(self._base_u + self._walk_bound_u, self._walk_u))
            self._walk_v = max(self._base_v - self._walk_bound_v, min(self._base_v + self._walk_bound_v, self._walk_v))
            u = self._walk_u
            v = self._walk_v
        elif self._pattern == 'loss_reacquire':
            u = self._base_u + self._lis_amp_u * math.sin(2.0 * math.pi * self._lis_fu * self._t)
            v = self._base_v + self._lis_amp_v * math.cos(2.0 * math.pi * self._lis_fv * self._t)

        return self._clip_bbox(u, v, w, h)

    def _build_detection_array(self, u: float, v: float, w: float, h: float) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        det = Detection2D()
        det.bbox.center.position.x = float(u)
        det.bbox.center.position.y = float(v)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self._class_id
        hyp.hypothesis.score = float(self._score)
        det.results.append(hyp)
        msg.detections.append(det)
        return msg

    def _build_target_obs(self, stamp, u: float, v: float, w: float, h: float) -> TargetObservation:
        msg = TargetObservation()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.track_id = self._track_id
        msg.u = float(u)
        msg.v = float(v)
        msg.w = float(w)
        msg.h = float(h)
        msg.depth_m = float(self._direct_depth)
        msg.confidence = float(self._score)
        msg.keypoints_uv = []
        msg.source = 'synthetic_bbox'
        return msg

    def _tick(self):
        dt = 1.0 / max(1e-3, self._rate_hz)
        self._t += dt

        visible = self._is_visible()
        if not visible:
            empty = Detection2DArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self._frame_id
            self._det_pub.publish(empty)
            return

        u, v, w, h = self._pattern_bbox()
        det_msg = self._build_detection_array(u, v, w, h)
        self._det_pub.publish(det_msg)

        if self._publish_direct:
            self._obs_pub.publish(self._build_target_obs(det_msg.header.stamp, u, v, w, h))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticBBoxPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
