import numpy as np

from sc2_visual_servo_controller.ibvs.centroid_kalman_filter import CentroidKFConfig
from sc2_visual_servo_controller.ibvs.centroid_kf_replay import (
    BodyVelocityEvent,
    DepthEvent,
    DetectionEvent,
    ReplayConfig,
    replay_centroid_kf,
)

FX = 600.0
FY = 600.0
CX = 320.0
CY = 320.0


def test_filtered_state_is_none_before_first_detection():
    detections = [DetectionEvent(t=1.0, u=CX, v=CY)]
    samples = replay_centroid_kf(
        detections, [], [], ReplayConfig(tick_hz=10.0, fx=FX, fy=FY, cx=CX, cy=CY),
        t_start=0.0, t_end=2.0,
    )

    before = [s for s in samples if s.t < 1.0]
    assert before
    assert all(s.filtered_u is None for s in before)
    after = [s for s in samples if s.t >= 1.0]
    assert after
    assert all(s.filtered_u is not None for s in after)


def test_predicts_through_a_detection_gap_using_body_velocity():
    # One detection at the image center, then nothing else -- but a
    # continuous body velocity feed the whole time. The Jacobian should keep
    # moving the filtered centroid even with no further raw detections.
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=t, v_body=np.array([1.0, 0.0, 0.0]), w_body=np.zeros(3))
        for t in np.arange(0.0, 5.0, 0.05)
    ]
    depths = [DepthEvent(t=t, z=2.0) for t in np.arange(0.0, 5.0, 0.05)]

    config = ReplayConfig(
        tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY,
        kf_config=CentroidKFConfig(process_noise_px2_per_s=0.0),
    )
    samples = replay_centroid_kf(detections, body_velocities, depths, config)

    gap_samples = [s for s in samples if s.t > 1.0 and not s.has_detection]
    assert len(gap_samples) > 10

    # vx > 0 at the image center should decrease u (matches the interaction
    # matrix: du = -fx/Z * vx), and it should be a steady drift, not frozen.
    us = [s.filtered_u for s in gap_samples]
    assert us[-1] < us[0]
    assert len({round(u, 6) for u in us}) > 1


def test_falls_back_to_hold_when_body_velocity_goes_stale():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=0.0, v_body=np.array([1.0, 0.0, 0.0]), w_body=np.zeros(3)),
    ]
    depths = [DepthEvent(t=0.0, z=2.0)]

    config = ReplayConfig(
        tick_hz=10.0, fx=FX, fy=FY, cx=CX, cy=CY,
        body_velocity_timeout_s=0.5,
    )
    samples = replay_centroid_kf(detections, body_velocities, depths, config, t_start=0.0, t_end=3.0)

    late_samples = [s for s in samples if s.t > 2.0]
    assert late_samples
    values = {round(s.filtered_u, 9) for s in late_samples}
    assert len(values) == 1  # held constant once the velocity feed goes stale


def test_clear_on_loss_resets_filter_after_configured_gap():
    detections = [
        DetectionEvent(t=0.0, u=CX, v=CY),
        DetectionEvent(t=5.0, u=CX + 40.0, v=CY + 40.0),
    ]
    config = ReplayConfig(tick_hz=10.0, fx=FX, fy=FY, cx=CX, cy=CY, clear_on_loss_after_s=1.0)
    samples = replay_centroid_kf(detections, [], [], config)

    mid_gap = [s for s in samples if 2.0 < s.t < 4.9]
    assert mid_gap
    assert all(s.filtered_u is None for s in mid_gap)

    after_reacquire = [s for s in samples if s.t >= 5.0]
    assert after_reacquire
    assert after_reacquire[0].filtered_u == CX + 40.0


def test_updates_track_frequent_detections_with_zero_velocity():
    detections = [DetectionEvent(t=t, u=100.0, v=200.0) for t in np.arange(0.0, 2.0, 0.1)]
    config = ReplayConfig(tick_hz=10.0, fx=FX, fy=FY, cx=CX, cy=CY)
    samples = replay_centroid_kf(detections, [], [], config)

    late = [s for s in samples if s.t > 1.5]
    assert late
    for s in late:
        assert abs(s.filtered_u - 100.0) < 1.0
        assert abs(s.filtered_v - 200.0) < 1.0
