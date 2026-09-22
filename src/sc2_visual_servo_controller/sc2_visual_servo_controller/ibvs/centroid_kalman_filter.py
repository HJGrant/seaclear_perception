from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CentroidKFConfig:
    process_noise_px2_per_s: float = 4.0
    measurement_noise_px2: float = 25.0
    initial_variance_px2: float = 400.0


class CentroidKalmanFilter:
    '''Tracks a bounding-box centroid (u, v) in pixels with a full 2x2
    covariance.

    predict() takes an externally supplied velocity (du, dv) -- typically the
    image-Jacobian body-velocity projection -- rather than estimating
    velocity as part of the state. A real 2x2 matrix (not two independent
    scalars) is needed here specifically so a process-noise covariance
    derived from a real velocity-source covariance (e.g. an EKF's twist
    covariance, propagated through the image Jacobian via
    image_jacobian.predict_pixel_velocity_covariance) can be used directly --
    that propagation generally produces a non-diagonal result, correlating
    du and dv, which two decoupled 1D filters cannot represent.

    `CentroidKFConfig.process_noise_px2_per_s` is always applied as an
    additive diagonal floor on top of any dynamic Q passed to predict(),
    since a covariance source that comes back unpopulated/zero (a common
    default for a not-fully-configured EKF) would otherwise silently make
    the filter overconfident rather than falling back to a sane default.
    '''

    def __init__(self, config: CentroidKFConfig) -> None:
        self._config = config
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None

    @property
    def is_initialized(self) -> bool:
        return self._x is not None

    @property
    def state(self) -> Optional[Tuple[float, float]]:
        if self._x is None:
            return None
        return float(self._x[0]), float(self._x[1])

    @property
    def covariance(self) -> Optional[np.ndarray]:
        return None if self._P is None else self._P.copy()

    @property
    def variance(self) -> Tuple[float, float]:
        '''Marginal (diagonal) variances -- P is not necessarily diagonal
        once a dynamic Q has been used, so this is P[0,0]/P[1,1], not the
        whole picture.
        '''
        if self._P is None:
            return (self._config.initial_variance_px2, self._config.initial_variance_px2)
        return float(self._P[0, 0]), float(self._P[1, 1])

    def reset(self, u: float, v: float) -> None:
        self._x = np.array([u, v], dtype=float)
        self._P = np.eye(2) * self._config.initial_variance_px2

    def clear(self) -> None:
        '''Return to the uninitialized state (e.g. on target loss), so the next
        accepted detection re-seeds position rather than predicting onward from
        a stale estimate.
        '''
        self._x = None
        self._P = None

    def predict(
        self,
        dt: float,
        du: float = 0.0,
        dv: float = 0.0,
        Q: Optional[np.ndarray] = None,
    ) -> None:
        '''Q, if given, is a 2x2 process-noise *rate* (same per-second
        convention as process_noise_px2_per_s, e.g. from
        image_jacobian.predict_pixel_velocity_covariance) -- it is added to
        the diagonal floor rate and the sum is scaled by dt, matching how the
        floor alone was always applied.
        '''
        if self._x is None:
            return
        self._x = self._x + dt * np.array([du, dv])
        floor_rate = np.eye(2) * self._config.process_noise_px2_per_s
        total_rate = floor_rate if Q is None else (floor_rate + Q)
        self._P = self._P + total_rate * dt

    def update(self, u_meas: float, v_meas: float, R: Optional[np.ndarray] = None) -> None:
        if self._x is None:
            self.reset(u_meas, v_meas)
            return
        R_eff = np.eye(2) * self._config.measurement_noise_px2 if R is None else R
        z = np.array([u_meas, v_meas])
        S = self._P + R_eff
        K = self._P @ np.linalg.inv(S)
        self._x = self._x + K @ (z - self._x)
        self._P = (np.eye(2) - K) @ self._P
