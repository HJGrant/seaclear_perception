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


def test_velocity_covariance_changes_filtered_trajectory_vs_flat_floor_only():
    # Same everything except one run carries a (large) body-velocity
    # covariance and the other doesn't -- if the covariance path is actually
    # wired in, the resulting filtered trajectories must diverge once a
    # detection gap lets process noise actually matter for the update gain.
    detections = [
        DetectionEvent(t=0.0, u=CX, v=CY),
        DetectionEvent(t=3.0, u=CX + 30.0, v=CY),
    ]
    body_velocities = [
        BodyVelocityEvent(
            t=t, v_body=np.array([0.05, 0.0, 0.0]), w_body=np.zeros(3),
            covariance=np.diag([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        )
        for t in np.arange(0.0, 3.0, 0.05)
    ]
    depths = [DepthEvent(t=t, z=2.0) for t in np.arange(0.0, 3.0, 0.05)]

    config_with_cov = ReplayConfig(
        tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, use_velocity_covariance=True,
    )
    config_without_cov = ReplayConfig(
        tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, use_velocity_covariance=False,
    )

    samples_with = replay_centroid_kf(detections, body_velocities, depths, config_with_cov)
    samples_without = replay_centroid_kf(detections, body_velocities, depths, config_without_cov)

    # Just before the second detection lands, the two runs should have
    # accumulated different uncertainty (and therefore different update
    # behavior at t=3.0) if the covariance is actually influencing Q.
    just_before = [s for s in samples_with if 2.9 < s.t < 3.0][-1]
    just_before_no_cov = [s for s in samples_without if 2.9 < s.t < 3.0][-1]
    assert just_before.filtered_u is not None
    assert just_before_no_cov.filtered_u is not None
    # Means should match (Q doesn't affect the predicted mean)...
    assert abs(just_before.filtered_u - just_before_no_cov.filtered_u) < 1e-6

    after_with = [s for s in samples_with if s.t >= 3.0][0]
    after_without = [s for s in samples_without if s.t >= 3.0][0]
    # ...but the update at reacquisition should snap closer to the new
    # detection in the higher-Q (covariance-driven) run, since its Kalman
    # gain is larger.
    target = CX + 30.0
    assert abs(after_with.filtered_u - target) < abs(after_without.filtered_u - target)


def test_override_z_predicts_with_no_depth_events_at_all():
    # No DepthEvent stream provided at all -- without override_z this would
    # fall back to (0.0, 0.0) every tick (stale_depth-equivalent), so any
    # observed drift proves override_z alone is driving the prediction.
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=t, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))
        for t in np.arange(0.0, 2.0, 0.05)
    ]
    config = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, override_z=2.0)
    samples = replay_centroid_kf(detections, body_velocities, [], config)

    late = [s for s in samples if s.t > 1.5]
    assert late
    assert late[-1].filtered_u is not None
    assert late[-1].filtered_u < CX  # -fx/Z * vx < 0, so u should have drifted down


def test_override_z_scales_prediction_inversely_with_z():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=t, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))
        for t in np.arange(0.0, 1.0, 0.05)
    ]
    config_near = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, override_z=1.0)
    config_far = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, override_z=4.0)

    samples_near = replay_centroid_kf(detections, body_velocities, [], config_near)
    samples_far = replay_centroid_kf(detections, body_velocities, [], config_far)

    drift_near = CX - samples_near[-1].filtered_u
    drift_far = CX - samples_far[-1].filtered_u
    # Halving Z (4.0 -> 1.0 is 4x closer) should ~4x the drift magnitude.
    assert drift_near > 3.0 * drift_far


def test_override_z_ignored_when_none_falls_back_to_depth_stream():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=t, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))
        for t in np.arange(0.0, 1.0, 0.05)
    ]
    config = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, override_z=None)
    samples = replay_centroid_kf(detections, body_velocities, [], config)
    # No depth stream and no override -> Jacobian never fires -> holds at seed value.
    assert samples[-1].filtered_u == CX


def test_z_scale_factor_scales_the_real_depth_signal():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [
        BodyVelocityEvent(t=t, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))
        for t in np.arange(0.0, 1.0, 0.05)
    ]
    depths = [DepthEvent(t=t, z=2.0) for t in np.arange(0.0, 1.0, 0.05)]

    config_1x = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, z_scale_factor=1.0)
    config_2x = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY, z_scale_factor=2.0)

    samples_1x = replay_centroid_kf(detections, body_velocities, depths, config_1x)
    samples_2x = replay_centroid_kf(detections, body_velocities, depths, config_2x)

    # z_used should reflect the scaled value, not the raw depth reading.
    assert abs(samples_1x[-1].z_used - 2.0) < 1e-9
    assert abs(samples_2x[-1].z_used - 4.0) < 1e-9

    # Doubling Z halves the drift magnitude (du = -fx/Z * vx).
    drift_1x = CX - samples_1x[-1].filtered_u
    drift_2x = CX - samples_2x[-1].filtered_u
    assert drift_1x > 1.8 * drift_2x  # ~2x, loose bound for tick discretization


def test_override_z_takes_precedence_over_z_scale_factor():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [BodyVelocityEvent(t=0.0, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))]
    depths = [DepthEvent(t=0.0, z=2.0)]

    config = ReplayConfig(
        tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY,
        override_z=5.0, z_scale_factor=3.0,  # scale should be ignored entirely
    )
    samples = replay_centroid_kf(detections, body_velocities, depths, config, t_start=0.0, t_end=0.5)
    used = [s.z_used for s in samples if s.z_used is not None]
    assert used and all(abs(z - 5.0) < 1e-9 for z in used)


def test_z_used_is_none_when_no_depth_available():
    detections = [DetectionEvent(t=0.0, u=CX, v=CY)]
    body_velocities = [BodyVelocityEvent(t=0.0, v_body=np.array([0.2, 0.0, 0.0]), w_body=np.zeros(3))]
    config = ReplayConfig(tick_hz=20.0, fx=FX, fy=FY, cx=CX, cy=CY)
    samples = replay_centroid_kf(detections, body_velocities, [], config, t_start=0.0, t_end=0.5)
    assert all(s.z_used is None for s in samples)


def test_updates_track_frequent_detections_with_zero_velocity():
    detections = [DetectionEvent(t=t, u=100.0, v=200.0) for t in np.arange(0.0, 2.0, 0.1)]
    config = ReplayConfig(tick_hz=10.0, fx=FX, fy=FY, cx=CX, cy=CY)
    samples = replay_centroid_kf(detections, [], [], config)

    late = [s for s in samples if s.t > 1.5]
    assert late
    for s in late:
        assert abs(s.filtered_u - 100.0) < 1.0
        assert abs(s.filtered_v - 200.0) < 1.0
