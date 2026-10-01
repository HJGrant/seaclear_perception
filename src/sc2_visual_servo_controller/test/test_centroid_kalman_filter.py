import numpy as np

from sc2_visual_servo_controller.ibvs.centroid_kalman_filter import (
    CentroidKFConfig,
    CentroidKalmanFilter,
)


def _kf(**overrides):
    config = CentroidKFConfig(**overrides)
    return CentroidKalmanFilter(config)


def test_uninitialized_state_is_none():
    kf = _kf()
    assert kf.state is None
    assert not kf.is_initialized


def test_predict_before_init_is_noop():
    kf = _kf()
    kf.predict(dt=1.0, du=10.0, dv=10.0)
    assert kf.state is None


def test_update_initializes_on_first_measurement():
    kf = _kf()
    kf.update(100.0, 200.0)
    assert kf.is_initialized
    assert kf.state == (100.0, 200.0)


def test_predict_moves_state_by_velocity_times_dt():
    kf = _kf(process_noise_px2_per_s=0.0)
    kf.reset(0.0, 0.0)
    kf.predict(dt=0.5, du=10.0, dv=-4.0)
    assert kf.state == (5.0, -2.0)


def test_predict_grows_variance():
    kf = _kf(process_noise_px2_per_s=4.0, initial_variance_px2=100.0)
    kf.reset(0.0, 0.0)
    kf.predict(dt=1.0, du=0.0, dv=0.0)
    var_u, var_v = kf.variance
    assert var_u == 104.0
    assert var_v == 104.0


def test_update_pulls_state_toward_measurement_without_overshoot():
    kf = _kf(measurement_noise_px2=25.0, initial_variance_px2=100.0)
    kf.reset(0.0, 0.0)
    kf.update(10.0, -10.0)
    u, v = kf.state
    assert 0.0 < u < 10.0
    assert -10.0 < v < 0.0


def test_repeated_updates_reduce_variance_and_converge():
    kf = _kf(measurement_noise_px2=25.0, initial_variance_px2=400.0)
    kf.reset(0.0, 0.0)
    for _ in range(50):
        kf.update(100.0, 100.0)
    u, v = kf.state
    assert abs(u - 100.0) < 1.0
    assert abs(v - 100.0) < 1.0
    var_u, _ = kf.variance
    assert var_u < 5.0


def test_clear_returns_to_uninitialized():
    kf = _kf()
    kf.reset(1.0, 2.0)
    kf.clear()
    assert kf.state is None
    assert not kf.is_initialized
    kf.predict(dt=1.0, du=5.0, dv=5.0)
    assert kf.state is None


def test_reset_clears_to_new_value_and_variance():
    kf = _kf(initial_variance_px2=64.0)
    kf.reset(0.0, 0.0)
    kf.update(100.0, 100.0)
    kf.reset(5.0, 6.0)
    assert kf.state == (5.0, 6.0)
    assert kf.variance == (64.0, 64.0)


def test_predict_with_dynamic_q_adds_floor_plus_dynamic_rate():
    kf = _kf(process_noise_px2_per_s=2.0, initial_variance_px2=0.0)
    kf.reset(0.0, 0.0)
    Q = np.array([[10.0, 0.0], [0.0, 20.0]])
    kf.predict(dt=1.0, du=0.0, dv=0.0, Q=Q)
    cov = kf.covariance
    # (floor=2*I + Q) * dt
    assert np.allclose(cov, np.array([[12.0, 0.0], [0.0, 22.0]]))


def test_predict_with_off_diagonal_q_produces_correlated_covariance():
    kf = _kf(process_noise_px2_per_s=0.0, initial_variance_px2=0.0)
    kf.reset(0.0, 0.0)
    Q = np.array([[4.0, 3.0], [3.0, 4.0]])
    kf.predict(dt=1.0, du=0.0, dv=0.0, Q=Q)
    cov = kf.covariance
    assert cov[0, 1] != 0.0
    assert cov[0, 1] == cov[1, 0]  # stays symmetric


def test_zero_dynamic_q_behaves_like_floor_only():
    kf_a = _kf(process_noise_px2_per_s=3.0, initial_variance_px2=10.0)
    kf_a.reset(1.0, 2.0)
    kf_a.predict(dt=0.5, du=1.0, dv=-1.0, Q=np.zeros((2, 2)))

    kf_b = _kf(process_noise_px2_per_s=3.0, initial_variance_px2=10.0)
    kf_b.reset(1.0, 2.0)
    kf_b.predict(dt=0.5, du=1.0, dv=-1.0)

    assert kf_a.state == kf_b.state
    assert np.allclose(kf_a.covariance, kf_b.covariance)


def test_update_accepts_custom_measurement_covariance():
    kf = _kf(measurement_noise_px2=25.0, initial_variance_px2=100.0)
    kf.reset(0.0, 0.0)
    # A much tighter R than the default should pull the state closer to the
    # measurement than the default-R update would.
    tight_R = np.eye(2) * 1.0
    kf.update(10.0, 10.0, R=tight_R)
    u, v = kf.state
    assert u > 9.0 and v > 9.0  # near-full trust in the measurement


def test_covariance_property_is_a_copy_not_a_live_reference():
    kf = _kf()
    kf.reset(0.0, 0.0)
    cov = kf.covariance
    cov[0, 0] = 999.0
    assert kf.covariance[0, 0] != 999.0
