from sc2_visual_servo_controller.hold_maintain_core import (
    HoldVelocityDampingConfig,
    HoldVelocityMeasurement,
    compute_hold_velocity_damping,
    should_enter_hold_maintain,
)


def test_should_enter_hold_maintain_when_selected_invalid():
    assert should_enter_hold_maintain(
        use_target_observation=True,
        observation_received=True,
        observation_fresh=True,
        selected_valid=False,
    )


def test_should_not_enter_hold_for_legacy_detection_path():
    assert not should_enter_hold_maintain(
        use_target_observation=False,
        observation_received=False,
        observation_fresh=False,
        selected_valid=False,
    )


def test_hold_velocity_damping_commands_against_measured_drift():
    vx, vy, yaw = compute_hold_velocity_damping(
        HoldVelocityMeasurement(vx=0.2, vy=-0.1, yaw_rate=0.5),
        HoldVelocityDampingConfig(
            xy_damping=0.5,
            yaw_damping=0.4,
            max_vx=0.4,
            max_vy=0.3,
            max_yaw=0.4,
        ),
    )

    assert vx < 0.0
    assert vy > 0.0
    assert yaw < 0.0


def test_hold_velocity_damping_clamps_output():
    vx, vy, yaw = compute_hold_velocity_damping(
        HoldVelocityMeasurement(vx=10.0, vy=-10.0, yaw_rate=10.0),
        HoldVelocityDampingConfig(
            xy_damping=1.0,
            yaw_damping=1.0,
            max_vx=0.4,
            max_vy=0.3,
            max_yaw=0.2,
        ),
    )

    assert vx == -0.4
    assert vy == 0.3
    assert yaw == -0.2
