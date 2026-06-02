from dataclasses import dataclass


@dataclass(frozen=True)
class HoldVelocityDampingConfig:
    xy_damping: float = 0.4
    yaw_damping: float = 0.3
    max_vx: float = 0.4
    max_vy: float = 0.3
    max_yaw: float = 0.4


@dataclass(frozen=True)
class HoldVelocityMeasurement:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


def clamp_symmetric(value: float, limit: float) -> float:
    lim = max(0.0, float(limit))
    return max(-lim, min(lim, float(value)))


def should_enter_hold_maintain(
    *,
    use_target_observation: bool,
    observation_received: bool,
    observation_fresh: bool,
    selected_valid: bool,
) -> bool:
    """Return true when visual correction must stop and HOLD maintain should run."""
    if not use_target_observation:
        return False
    return (not observation_received) or (not observation_fresh) or (not selected_valid)


def compute_hold_velocity_damping(
    measurement: HoldVelocityMeasurement,
    config: HoldVelocityDampingConfig,
) -> tuple[float, float, float]:
    """Body-frame velocity damping used during visual-target HOLD.

    This is not a full position hold. It commands against measured body-frame
    drift so HOLD is not merely a zero-command fallback.
    """
    vx_cmd = clamp_symmetric(-float(config.xy_damping) * float(measurement.vx), config.max_vx)
    vy_cmd = clamp_symmetric(-float(config.xy_damping) * float(measurement.vy), config.max_vy)
    yaw_cmd = clamp_symmetric(-float(config.yaw_damping) * float(measurement.yaw_rate), config.max_yaw)
    return vx_cmd, vy_cmd, yaw_cmd
