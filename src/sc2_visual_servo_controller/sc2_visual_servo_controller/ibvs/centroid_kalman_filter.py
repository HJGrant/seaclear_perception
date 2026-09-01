from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CentroidKFConfig:
    process_noise_px2_per_s: float = 4.0
    measurement_noise_px2: float = 25.0
    initial_variance_px2: float = 400.0


class _ScalarKF:
    '''1D Kalman filter for a single pixel axis.'''

    def __init__(self, config: CentroidKFConfig) -> None:
        self._config = config
        self.value: Optional[float] = None
        self.variance: float = config.initial_variance_px2

    def reset(self, value: float) -> None:
        self.value = value
        self.variance = self._config.initial_variance_px2

    def predict(self, dt: float, velocity: float) -> None:
        if self.value is None:
            return
        self.value += velocity * dt
        self.variance += self._config.process_noise_px2_per_s * dt

    def update(self, measurement: float) -> None:
        if self.value is None:
            self.reset(measurement)
            return
        r = self._config.measurement_noise_px2
        k = self.variance / (self.variance + r)
        self.value += k * (measurement - self.value)
        self.variance *= (1.0 - k)


class CentroidKalmanFilter:
    '''Tracks a bounding-box centroid (u, v) in pixels.

    predict() takes an externally supplied velocity (du, dv) -- typically the
    image-Jacobian body-velocity projection -- rather than estimating velocity
    as part of the state. There is no u/v cross-coupling anywhere in this model
    (predict is a plain additive shift, H is identity, Q and R are diagonal),
    so the two axes are exactly independent and each reduces to a standalone
    1D filter.
    '''

    def __init__(self, config: CentroidKFConfig) -> None:
        self._u = _ScalarKF(config)
        self._v = _ScalarKF(config)

    @property
    def is_initialized(self) -> bool:
        return self._u.value is not None

    @property
    def state(self) -> Optional[Tuple[float, float]]:
        if not self.is_initialized:
            return None
        return self._u.value, self._v.value

    @property
    def variance(self) -> Tuple[float, float]:
        return self._u.variance, self._v.variance

    def reset(self, u: float, v: float) -> None:
        self._u.reset(u)
        self._v.reset(v)

    def clear(self) -> None:
        '''Return to the uninitialized state (e.g. on target loss), so the next
        accepted detection re-seeds position rather than predicting onward from
        a stale estimate.
        '''
        self._u.value = None
        self._v.value = None

    def predict(self, dt: float, du: float = 0.0, dv: float = 0.0) -> None:
        self._u.predict(dt, du)
        self._v.predict(dt, dv)

    def update(self, u_meas: float, v_meas: float) -> None:
        self._u.update(u_meas)
        self._v.update(v_meas)
