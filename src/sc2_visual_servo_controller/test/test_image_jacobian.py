import math

import numpy as np

from sc2_visual_servo_controller.ibvs.controller_math import get_image_jacobian
from sc2_visual_servo_controller.ibvs.image_jacobian import (
    apply_refraction_scale,
    body_twist_to_camera_twist,
    predict_pixel_centroid_velocity,
    quaternion_to_rotation_matrix,
)

FX = 600.0
FY = 600.0
CX = 320.0
CY = 320.0
Z = 2.0


def _jacobian_at_center():
    return get_image_jacobian(np.array([[CX, CY]]), np.array([Z]), FX, FY, CX, CY)


def test_jacobian_pure_lateral_translation_at_center():
    L = _jacobian_at_center()
    du, dv = L @ np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert du == -FX / Z
    assert dv == 0.0


def test_jacobian_forward_translation_off_center():
    u = CX + 100.0  # x = (u - cx) / fx = 100/600 = 1/6
    L = get_image_jacobian(np.array([[u, CY]]), np.array([Z]), FX, FY, CX, CY)
    du, dv = L @ np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    x = 100.0 / FX
    assert abs(du - FX * x / Z) < 1e-9
    assert dv == 0.0


def test_jacobian_pitch_rate_at_center():
    L = _jacobian_at_center()
    du, dv = L @ np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert du == -FX
    assert dv == 0.0


def test_jacobian_roll_rate_at_center():
    L = _jacobian_at_center()
    du, dv = L @ np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    assert du == 0.0
    assert dv == FY


def test_jacobian_yaw_rate_off_center():
    u = CX + 100.0
    L = get_image_jacobian(np.array([[u, CY]]), np.array([Z]), FX, FY, CX, CY)
    du, dv = L @ np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    x = 100.0 / FX
    assert du == 0.0
    assert abs(dv - (-FY * x)) < 1e-9


def test_body_twist_to_camera_twist_identity_no_lever_arm():
    v_body = np.array([1.0, 2.0, 3.0])
    w_body = np.array([0.1, 0.2, 0.3])
    v_cam, w_cam = body_twist_to_camera_twist(
        v_body, w_body, np.zeros(3), np.eye(3),
    )
    assert np.allclose(v_cam, v_body)
    assert np.allclose(w_cam, w_body)


def test_body_twist_to_camera_twist_lever_arm_cross_term():
    # Pure yaw about body z with the camera offset along body x: the
    # rotation sweeps the camera sideways at wz * rx.
    v_body = np.zeros(3)
    w_body = np.array([0.0, 0.0, 0.4])
    r_cam_body = np.array([0.115, 0.0, 0.0])
    v_cam, w_cam = body_twist_to_camera_twist(
        v_body, w_body, r_cam_body, np.eye(3),
    )
    assert np.allclose(v_cam, [0.0, 0.4 * 0.115, 0.0])
    assert np.allclose(w_cam, w_body)


def test_body_twist_to_camera_twist_rotation_reorients_axes():
    # R_cam_body maps body x onto camera y (a 90 degree rotation about z).
    R_cam_body = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    v_body = np.array([1.0, 0.0, 0.0])
    v_cam, _ = body_twist_to_camera_twist(
        v_body, np.zeros(3), np.zeros(3), R_cam_body,
    )
    assert np.allclose(v_cam, [0.0, 1.0, 0.0])


def test_predict_pixel_centroid_velocity_matches_manual_jacobian_composition():
    v_body = np.array([0.2, 0.0, 0.0])
    w_body = np.array([0.0, 0.0, 0.4])
    r_cam_body = np.array([0.115, 0.0, 0.0])
    R_cam_body = np.eye(3)

    du, dv = predict_pixel_centroid_velocity(
        CX, CY, Z, v_body, w_body, r_cam_body, R_cam_body, FX, FY, CX, CY,
    )

    v_cam, w_cam = body_twist_to_camera_twist(v_body, w_body, r_cam_body, R_cam_body)
    L = _jacobian_at_center()
    expected_du, expected_dv = L @ np.concatenate([v_cam, w_cam])

    assert abs(du - expected_du) < 1e-9
    assert abs(dv - expected_dv) < 1e-9


def test_quaternion_to_rotation_matrix_identity():
    R = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(R, np.eye(3))


def test_quaternion_to_rotation_matrix_90deg_about_z():
    # 90 degree rotation about z: qz = sin(45deg), qw = cos(45deg)
    s = math.sqrt(2.0) / 2.0
    R = quaternion_to_rotation_matrix(0.0, 0.0, s, s)
    v = R @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(v, [0.0, 1.0, 0.0], atol=1e-9)


def test_quaternion_to_rotation_matrix_is_orthonormal():
    s = math.sqrt(2.0) / 2.0
    R = quaternion_to_rotation_matrix(0.1, 0.2, 0.3, math.sqrt(1 - 0.1**2 - 0.2**2 - 0.3**2))
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)


def test_apply_refraction_scale_is_a_noop_at_one():
    assert apply_refraction_scale(FX, FY, 1.0) == (FX, FY)


def test_apply_refraction_scale_scales_both_axes():
    assert apply_refraction_scale(FX, FY, 1.33) == (FX * 1.33, FY * 1.33)


def test_predict_pixel_centroid_velocity_rejects_nonpositive_depth():
    try:
        predict_pixel_centroid_velocity(
            CX, CY, 0.0, np.zeros(3), np.zeros(3), np.zeros(3), np.eye(3),
            FX, FY, CX, CY,
        )
        assert False, 'expected ValueError for non-positive Z'
    except ValueError:
        pass
