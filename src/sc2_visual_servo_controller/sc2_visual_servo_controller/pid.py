from dataclasses import dataclass
from typing import Optional


def clamp_abs(value: float, limit: Optional[float]) -> float:
    if limit is None or limit <= 0.0:
        return value
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    integral_limit: Optional[float] = None
    output_limit: Optional[float] = None


class PIDController:
    def __init__(self, gains: PIDGains) -> None:
        self.gains = gains
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt: float) -> float:
        dt = max(float(dt), 1e-6)

        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt

        self._integral += error * dt
        self._integral = clamp_abs(self._integral, self.gains.integral_limit)

        output = (
            (self.gains.kp * error)
            + (self.gains.ki * self._integral)
            + (self.gains.kd * derivative)
        )
        output = clamp_abs(output, self.gains.output_limit)

        self._prev_error = error
        return output
