import math
import threading
from typing import Dict, Optional

from cv_bridge import CvBridge
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from sensor_msgs.msg import Image, Imu
from vision_msgs.msg import Detection2D, Detection2DArray

from .conversion import compute_depth_median, extract_hypothesis
from .tracker_selector_core import (
    DetectionCandidate,
    TrackState,
    TRACK_CONFIRMED,
    TRACK_ESCALATE,
    TRACK_LOST,
    associate_two_stage,
    class_matches,
    compute_freshness,
    create_track,
    motion_gate_scale,
    select_track,
    update_track_with_match,
    update_track_with_miss,
)


class TargetObserverNode(Node):
    """Track-table based selector for `/detections_output`.

    This node intentionally uses image-plane association and motion-aware gate
    expansion only. It does not perform precise camera/TF reprojection.
    """

    def __init__(self, *, parameter_overrides=None):
        super().__init__('target_observer_node', parameter_overrides=parameter_overrides)

        self.declare_parameter('target_class', '')
        # Initial thresholds are provisional and must be tuned with field bags.
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('low_confidence', 0.15)
        self.declare_parameter('pick_strategy', 'nearest_track')
        self.declare_parameter('max_age_ms', 300)
        self.declare_parameter('timeout_ms', 500)

        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 20.0)

        self.declare_parameter('source', 'detections_output_tracker_selector')
        self.declare_parameter('detections_topic', '/detections_output')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('output_topic', '/sc2/visual_servo/target_obs')

        # Provisional initial values; require in-water tuning.
        self.declare_parameter('alpha_uv_predict', 0.0)
        self.declare_parameter('t_stale_sec', 1.0)
        self.declare_parameter('n_confirm', 5)
        self.declare_parameter('base_gate_px', 80.0)
        self.declare_parameter('r_gate_max_scale', 1.6)
        self.declare_parameter('confidence_ema_alpha', 0.35)
        self.declare_parameter('hold_max_misses', 12)
        self.declare_parameter('lost_max_misses', 30)
        self.declare_parameter('max_tracks', 8)
        self.declare_parameter('selector_switch_margin', 0.05)
        self.declare_parameter('selector_selected_track_bonus', 0.05)

        self.declare_parameter('enable_motion_aware_gate', True)
        self.declare_parameter('odom_topic', '/sc2/odometry_ekf')
        self.declare_parameter('imu_topic', '/sc2/imu/imu')
        self.declare_parameter('v_ref_mps', 0.5)
        self.declare_parameter('omega_ref_radps', 0.5)
        self.declare_parameter('beta_v', 0.3)
        self.declare_parameter('beta_omega', 0.3)

        self._target_class = str(self.get_parameter('target_class').value)
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._low_confidence = float(self.get_parameter('low_confidence').value)
        self._max_age_ms = float(self.get_parameter('max_age_ms').value)
        self._timeout_ms = float(self.get_parameter('timeout_ms').value)

        self._use_depth = self._as_bool(self.get_parameter('use_depth').value)
        self._depth_min_m = float(self.get_parameter('depth_min_m').value)
        self._depth_max_m = float(self.get_parameter('depth_max_m').value)
        self._source = str(self.get_parameter('source').value)

        self._alpha_uv_predict = max(0.0, min(0.3, float(self.get_parameter('alpha_uv_predict').value)))
        self._t_stale_sec = max(1e-3, float(self.get_parameter('t_stale_sec').value))
        self._n_confirm = max(1, int(self.get_parameter('n_confirm').value))
        self._base_gate_px = max(1.0, float(self.get_parameter('base_gate_px').value))
        self._r_gate_max_scale = max(1.0, float(self.get_parameter('r_gate_max_scale').value))
        self._confidence_ema_alpha = max(0.0, min(1.0, float(self.get_parameter('confidence_ema_alpha').value)))
        self._hold_max_misses = max(1, int(self.get_parameter('hold_max_misses').value))
        self._lost_max_misses = max(self._hold_max_misses + 1, int(self.get_parameter('lost_max_misses').value))
        self._max_tracks = max(1, int(self.get_parameter('max_tracks').value))
        self._selector_switch_margin = max(0.0, float(self.get_parameter('selector_switch_margin').value))
        self._selector_selected_track_bonus = max(
            0.0,
            float(self.get_parameter('selector_selected_track_bonus').value),
        )

        self._enable_motion_gate = self._as_bool(self.get_parameter('enable_motion_aware_gate').value)
        self._v_ref_mps = max(1e-3, float(self.get_parameter('v_ref_mps').value))
        self._omega_ref_radps = max(1e-3, float(self.get_parameter('omega_ref_radps').value))
        self._beta_v = max(0.0, float(self.get_parameter('beta_v').value))
        self._beta_omega = max(0.0, float(self.get_parameter('beta_omega').value))

        detections_topic = str(self.get_parameter('detections_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        self._bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._latest_depth = None
        self._tracks: Dict[int, TrackState] = {}
        self._selected_track_id: Optional[int] = None
        self._last_detection_header = None
        self._next_track_id = 1
        self._latest_linear_speed_mps = 0.0
        self._latest_yaw_rate_radps = 0.0

        self._last_pub_time = self.get_clock().now()
        self._warn_timestamps: Dict[str, float] = {}

        self._pub = self.create_publisher(TargetObservation, output_topic, 10)
        self.create_subscription(Detection2DArray, detections_topic, self._detections_cb, 10)

        if self._use_depth and depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        else:
            self._use_depth = False

        if self._enable_motion_gate:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
            self.create_subscription(Imu, imu_topic, self._imu_cb, 10)

        self.create_timer(0.2, self._health_tick)

        self.get_logger().info(
            'target_observer_node track-table selector: '
            f'detections={detections_topic}, depth={depth_topic if self._use_depth else "<disabled>"}, '
            f'out={output_topic}, target_class={self._target_class!r}, '
            f'high_conf={self._min_confidence:.3f}, low_conf={self._low_confidence:.3f}, '
            f'n_confirm={self._n_confirm}, max_tracks={self._max_tracks}, '
            f't_stale={self._t_stale_sec:.2f}s, motion-aware gate={"on" if self._enable_motion_gate else "off"}'
        )

    def _warn_throttle(self, key: str, msg: str, period_s: float = 2.0) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1e9)
        if (now_s - last) >= period_s:
            self._warn_timestamps[key] = now_s
            self.get_logger().warn(msg)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _depth_cb(self, msg: Image) -> None:
        if not self._use_depth:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._warn_throttle('depth_convert', f'Failed depth conversion: {exc}')
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _odom_cb(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        self._latest_linear_speed_mps = float(
            math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_yaw_rate_radps = abs(float(msg.angular_velocity.z))

    def _msg_age_ms(self, header) -> float:
        stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        if stamp_ns <= 0:
            return 0.0
        now_ns = self.get_clock().now().nanoseconds
        return max(0.0, (now_ns - stamp_ns) * 1e-6)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _gate_px(self) -> float:
        if not self._enable_motion_gate:
            return self._base_gate_px
        scale = motion_gate_scale(
            self._latest_linear_speed_mps,
            self._latest_yaw_rate_radps,
            v_ref_mps=self._v_ref_mps,
            omega_ref_radps=self._omega_ref_radps,
            beta_v=self._beta_v,
            beta_omega=self._beta_omega,
            r_gate_max_scale=self._r_gate_max_scale,
        )
        return self._base_gate_px * scale

    def _candidate_from_detection(self, det: Detection2D) -> Optional[DetectionCandidate]:
        if not det.results:
            return None
        cls_id, score = extract_hypothesis(det.results[0])
        score = float(score)
        if score < self._low_confidence:
            return None
        if not class_matches(cls_id, self._target_class):
            return None
        bbox = det.bbox
        return DetectionCandidate(
            detection=det,
            cls_id=cls_id,
            score=score,
            u=float(bbox.center.position.x),
            v=float(bbox.center.position.y),
            w=float(bbox.size_x),
            h=float(bbox.size_y),
            high_confidence=score >= self._min_confidence,
        )

    def _extract_candidates(self, msg: Detection2DArray) -> list[DetectionCandidate]:
        candidates = []
        for det in msg.detections:
            candidate = self._candidate_from_detection(det)
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _create_track_from_candidate(self, candidate: DetectionCandidate, now_sec: float) -> TrackState:
        track = create_track(self._next_track_id, candidate, now_sec)
        self._tracks[track.track_id] = track
        self._next_track_id += 1
        return track

    def _mark_unmatched_tracks_missed(self, matched_track_ids: set[int], now_sec: float) -> None:
        for track_id, track in list(self._tracks.items()):
            if track_id in matched_track_ids:
                continue
            update_track_with_miss(
                track,
                now_sec,
                t_stale_sec=self._t_stale_sec,
                hold_max_misses=self._hold_max_misses,
                lost_max_misses=self._lost_max_misses,
            )

    def _prune_tracks(self) -> None:
        if len(self._tracks) <= self._max_tracks:
            return
        ranked = sorted(
            self._tracks.values(),
            key=lambda t: (
                t.state not in {TRACK_LOST, TRACK_ESCALATE},
                compute_freshness(self._now_sec(), t.last_match_time_sec, self._t_stale_sec),
                t.confidence_ema,
                t.hit_count,
            ),
        )
        while len(self._tracks) > self._max_tracks and ranked:
            remove = ranked.pop(0)
            self._tracks.pop(remove.track_id, None)

    def _compute_depth(self, track: TrackState) -> float:
        if not self._use_depth:
            return float('nan')
        with self._depth_lock:
            depth = self._latest_depth
        depth_m = compute_depth_median(depth, track.u, track.v, track.w, track.h)
        if math.isfinite(depth_m):
            return float(max(self._depth_min_m, min(self._depth_max_m, depth_m)))
        self._warn_throttle('depth_nan', 'Depth ROI invalid; publishing NaN depth_m')
        return float('nan')

    def _publish_selected(self, header, track: Optional[TrackState], *, fallback_reason: str = 'no_track') -> None:
        out = TargetObservation()
        out.header = header
        out.source = self._source
        out.keypoints_uv = []

        if track is None:
            out.track_id = -1
            out.selected_track_id = -1
            out.track_state = TRACK_LOST
            out.reason_code = fallback_reason
            out.selected_valid = False
            out.freshness = 0.0
            self._selected_track_id = None
            self._pub.publish(out)
            self._last_pub_time = self.get_clock().now()
            return

        now_sec = self._now_sec()
        freshness = compute_freshness(now_sec, track.last_match_time_sec, self._t_stale_sec)
        track.freshness = freshness
        selected_valid = bool(track.state == TRACK_CONFIRMED and freshness > 0.0)
        self._selected_track_id = int(track.track_id)

        out.track_id = int(track.track_id)
        out.selected_track_id = int(track.track_id)
        out.u = float(track.u)
        out.v = float(track.v)
        out.w = float(track.w)
        out.h = float(track.h)
        out.depth_m = self._compute_depth(track)
        out.confidence = float(track.last_match_confidence)
        out.selected_valid = selected_valid
        out.track_state = str(track.state)
        out.reason_code = str(track.reason_code)
        out.age_frames = int(track.age_frames)
        out.miss_count = int(track.miss_count)
        out.freshness = float(freshness)
        self._pub.publish(out)
        self._last_pub_time = self.get_clock().now()

    def _detections_cb(self, msg: Detection2DArray) -> None:
        self._last_detection_header = msg.header
        age_ms = self._msg_age_ms(msg.header)
        if self._max_age_ms > 0.0 and age_ms > self._max_age_ms:
            self._warn_throttle(
                'stale_detection',
                f'Dropping stale Detection2DArray age={age_ms:.1f}ms > max_age_ms={self._max_age_ms:.1f}',
            )
            return

        now_sec = self._now_sec()
        candidates = self._extract_candidates(msg)
        active_tracks = [
            t for t in self._tracks.values()
            if t.state not in {TRACK_LOST, TRACK_ESCALATE}
        ]
        assignments, unmatched_candidate_ids = associate_two_stage(
            active_tracks,
            candidates,
            gate_px=self._gate_px(),
            alpha_uv_predict=self._alpha_uv_predict,
        )

        matched_track_ids = set(assignments.keys())
        for track_id, (candidate, reason) in assignments.items():
            update_track_with_match(
                self._tracks[track_id],
                candidate,
                now_sec,
                n_confirm=self._n_confirm,
                confidence_ema_alpha=self._confidence_ema_alpha,
                reason_code=reason,
            )

        self._mark_unmatched_tracks_missed(matched_track_ids, now_sec)
        self._prune_tracks()

        for cand_idx in unmatched_candidate_ids:
            candidate = candidates[cand_idx]
            if not candidate.high_confidence:
                continue
            if len(self._tracks) >= self._max_tracks:
                break
            self._create_track_from_candidate(candidate, now_sec)

        self._prune_tracks()
        selected = select_track(
            self._tracks.values(),
            now_sec,
            self._t_stale_sec,
            previous_selected_track_id=self._selected_track_id,
            selector_switch_margin=self._selector_switch_margin,
            selector_selected_track_bonus=self._selector_selected_track_bonus,
        )
        if not candidates:
            self._warn_throttle('no_candidates', 'No detection passed class/low-confidence filter')
        self._publish_selected(msg.header, selected)

    def _health_tick(self) -> None:
        now = self.get_clock().now()
        dt_ms = (now - self._last_pub_time).nanoseconds * 1e-6
        if dt_ms <= self._timeout_ms:
            return
        self._warn_throttle(
            'obs_timeout',
            f'No TargetObservation published for {dt_ms:.0f}ms (timeout_ms={self._timeout_ms:.0f})',
        )
        if not self._tracks or self._last_detection_header is None:
            return
        now_sec = self._now_sec()
        self._mark_unmatched_tracks_missed(set(), now_sec)
        selected = select_track(
            self._tracks.values(),
            now_sec,
            self._t_stale_sec,
            previous_selected_track_id=self._selected_track_id,
            selector_switch_margin=self._selector_switch_margin,
            selector_selected_track_bonus=self._selector_selected_track_bonus,
        )
        self._publish_selected(self._last_detection_header, selected, fallback_reason='timeout_hold')


def main(args=None):
    rclpy.init(args=args)
    node = TargetObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
