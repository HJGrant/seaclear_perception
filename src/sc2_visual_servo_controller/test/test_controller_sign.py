from sc2_visual_servo_controller.ibvs.controller_math import (
    BBoxApproachConfig,
    IBVSConfig,
    Observation,
    compute_bbox_approach_command,
    compute_ibvs_command,
)


def test_horizontal_error_right_drives_negative_body_y_and_negative_yaw():
    cfg = IBVSConfig(image_width=1000.0, image_height=500.0, target_u=500.0, target_v=250.0)
    obs = Observation(u=700.0, v=250.0, w=100.0, h=100.0, depth_m=1.0)
    vx, vy, vz, yaw = compute_ibvs_command(obs, cfg)
    assert vy < 0.0
    assert yaw < 0.0
    assert abs(vz) < 1e-9


def test_vertical_error_down_drives_negative_body_z():
    cfg = IBVSConfig(image_width=1000.0, image_height=500.0, target_u=500.0, target_v=250.0)
    obs = Observation(u=500.0, v=350.0, w=100.0, h=100.0, depth_m=1.0)
    _, _, vz, _ = compute_ibvs_command(obs, cfg)
    assert vz < 0.0


def test_depth_far_drives_positive_forward_velocity():
    cfg = IBVSConfig(desired_depth_m=1.0)
    obs = Observation(u=640.0, v=360.0, w=100.0, h=100.0, depth_m=2.0)
    vx, _, _, _ = compute_ibvs_command(obs, cfg)
    assert vx > 0.0


def test_bbox_approach_horizontal_error_right_drives_negative_yaw():
    cfg = BBoxApproachConfig(image_width=1000.0, image_height=500.0, target_u=500.0, target_v=250.0)
    obs = Observation(u=700.0, v=250.0, w=80.0, h=60.0, depth_m=float('nan'))
    _, _, _, yaw = compute_bbox_approach_command(obs, cfg)
    assert yaw < 0.0


def test_bbox_approach_small_area_drives_forward_without_depth():
    cfg = BBoxApproachConfig(stop_area_px=20000.0, k_area=0.4)
    obs = Observation(u=640.0, v=360.0, w=60.0, h=50.0, depth_m=float('nan'))
    vx, _, _, _ = compute_bbox_approach_command(obs, cfg)
    assert vx > 0.0


def test_bbox_approach_large_area_stops_without_depth():
    cfg = BBoxApproachConfig(stop_area_px=4000.0, k_area=0.5)
    obs = Observation(u=640.0, v=360.0, w=100.0, h=80.0, depth_m=float('nan'))
    vx, _, _, _ = compute_bbox_approach_command(obs, cfg)
    assert abs(vx) < 1e-9
