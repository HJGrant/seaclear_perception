import math

import numpy as np

from sc2_visual_servo_controller.ibvs.controller_math import get_image_jacobian
from sc2_visual_servo_controller.ibvs.image_jacobian import (
    apply_refraction_scale,
    body_twist_covariance_to_camera_twist_covariance,
    body_twist_to_camera_twist,
    is_valid_twist_covariance,
    predict_pixel_centroid_velocity,
    predict_pixel_centroid_velocity_debug,
    predict_pixel_velocity_covariance,
    quaternion_to_rotation_matrix,
    skew_matrix,
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


def test_skew_matrix_matches_cross_product():
    r = np.array([1.0, 2.0, 3.0])
    w = np.array([4.0, -5.0, 6.0])
    assert np.allclose(skew_matrix(r) @ w, np.cross(r, w))


def test_skew_matrix_is_antisymmetric():
    r = np.array([1.0, -2.0, 0.5])
    S = skew_matrix(r)
    assert np.allclose(S, -S.T)


def test_body_twist_covariance_identity_passthrough():
    cov = np.diag([0.1, 0.2, 0.3, 0.01, 0.02, 0.03])
    out = body_twist_covariance_to_camera_twist_covariance(cov, np.zeros(3), np.eye(3))
    assert np.allclose(out, cov)


def test_body_twist_covariance_rotation_reorients_variance():
    # 90 degree rotation about z maps body-x variance onto camera-y.
    R_cam_body = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    cov = np.zeros((6, 6))
    cov[0, 0] = 5.0  # var(vx) = 5
    out = body_twist_covariance_to_camera_twist_covariance(cov, np.zeros(3), R_cam_body)
    assert abs(out[1, 1] - 5.0) < 1e-9
    assert abs(out[0, 0]) < 1e-9


def test_body_twist_covariance_lever_arm_introduces_linear_from_angular():
    # Angular-velocity uncertainty about z, with a lever arm along x, must
    # show up as linear-velocity (y-axis) uncertainty at the camera -- the
    # covariance analogue of the mean lever-arm cross term.
    cov = np.zeros((6, 6))
    cov[5, 5] = 0.16  # var(wz) = 0.16 -> std 0.4 rad/s
    r_cam_body = np.array([0.115, 0.0, 0.0])
    out = body_twist_covariance_to_camera_twist_covariance(cov, r_cam_body, np.eye(3))
    expected_vy_var = (0.115 ** 2) * 0.16
    assert abs(out[1, 1] - expected_vy_var) < 1e-12


def test_predict_pixel_velocity_covariance_is_symmetric_psd():
    cov = np.diag([0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
    out = predict_pixel_velocity_covariance(
        CX, CY, Z, cov, np.zeros(3), np.eye(3), FX, FY, CX, CY,
    )
    assert out.shape == (2, 2)
    assert np.allclose(out, out.T)
    eigvals = np.linalg.eigvalsh(out)
    assert np.all(eigvals >= -1e-9)


def test_predict_pixel_velocity_covariance_zero_input_gives_zero_output():
    out = predict_pixel_velocity_covariance(
        CX, CY, Z, np.zeros((6, 6)), np.zeros(3), np.eye(3), FX, FY, CX, CY,
    )
    assert np.allclose(out, np.zeros((2, 2)))


def test_predict_pixel_velocity_covariance_matches_manual_composition():
    cov = np.diag([0.05, 0.04, 0.03, 0.02, 0.01, 0.005])
    r_cam_body = np.array([0.115, 0.0, 0.0])
    R_cam_body = np.eye(3)
    out = predict_pixel_velocity_covariance(
        CX, CY, Z, cov, r_cam_body, R_cam_body, FX, FY, CX, CY,
    )
    twist_cam_cov = body_twist_covariance_to_camera_twist_covariance(cov, r_cam_body, R_cam_body)
    L = _jacobian_at_center()
    expected = L @ twist_cam_cov @ L.T
    assert np.allclose(out, expected)


def test_predict_pixel_velocity_covariance_rejects_nonpositive_depth():
    try:
        predict_pixel_velocity_covariance(
            CX, CY, 0.0, np.eye(6), np.zeros(3), np.eye(3), FX, FY, CX, CY,
        )
        assert False, 'expected ValueError for non-positive Z'
    except ValueError:
        pass


def test_is_valid_twist_covariance_accepts_reasonable_diagonal():
    assert is_valid_twist_covariance(np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]))


def test_is_valid_twist_covariance_rejects_all_zero():
    assert not is_valid_twist_covariance(np.zeros((6, 6)))


def test_is_valid_twist_covariance_rejects_negative_sentinel_diagonal():
    cov = np.eye(6)
    cov[0, 0] = -1.0
    assert not is_valid_twist_covariance(cov)


def test_is_valid_twist_covariance_rejects_wrong_shape():
    assert not is_valid_twist_covariance(np.eye(3))


def test_swap_uv_matches_manually_swapped_call():
    # swap_uv=True evaluates the standard Jacobian at the substituted
    # position (v, u, fx, -fy, cx, cy) -- same substitution used by every
    # prior iteration of this fix -- but combines the raw (du_std, dv_std)
    # output asymmetrically: du_final = dv_std (unchanged), dv_final =
    # -du_std (negated). This exact combination was derived empirically from
    # real bag data (see predict_pixel_centroid_velocity's docstring): two
    # earlier, more "principled"-looking guesses (a plain reflection, then a
    # naive symmetric 90-degree swap) were both wrong once tested against
    # the vehicle's real non-identity extrinsic.
    u, v = CX + 40.0, CY - 15.0
    fx_, fy_ = 600.0, 550.0  # deliberately unequal to exercise fx vs fy distinctly
    v_body = np.array([0.2, -0.1, 0.05])
    w_body = np.array([0.01, -0.02, 0.3])
    r_cam_body = np.array([0.115, 0.0, 0.0])
    R_cam_body = np.eye(3)

    du_swapped, dv_swapped = predict_pixel_centroid_velocity(
        u, v, Z, v_body, w_body, r_cam_body, R_cam_body,
        fx_, fy_, CX, CY, swap_uv=True,
    )
    du_std, dv_std = predict_pixel_centroid_velocity(
        v, u, Z, v_body, w_body, r_cam_body, R_cam_body,
        fx_, -fy_, CX, CY, swap_uv=False,
    )
    assert abs(du_swapped - dv_std) < 1e-12
    assert abs(dv_swapped - (-du_std)) < 1e-12


def test_swap_uv_covariance_matches_manually_swapped_call():
    u, v = CX + 40.0, CY - 15.0
    fx_, fy_ = 600.0, 550.0
    cov = np.diag([0.05, 0.04, 0.03, 0.02, 0.01, 0.005])
    r_cam_body = np.array([0.115, 0.0, 0.0])
    R_cam_body = np.eye(3)

    swapped = predict_pixel_velocity_covariance(
        u, v, Z, cov, r_cam_body, R_cam_body, fx_, fy_, CX, CY, swap_uv=True,
    )
    cov_std = predict_pixel_velocity_covariance(
        v, u, Z, cov, r_cam_body, R_cam_body, fx_, -fy_, CX, CY, swap_uv=False,
    )
    # Same asymmetric rotation as the mean transform, not a plain
    # row/column reversal.
    rotation = np.array([[0.0, 1.0], [-1.0, 0.0]])
    assert np.allclose(swapped, rotation @ cov_std @ rotation.T)


def test_swap_uv_pure_vx_lands_on_v_not_u_when_detector_axes_are_rotated():
    # Concrete regression matching real bag data: with swap_uv, a pure vx
    # (forward) body velocity at the image center must show up as a change
    # in v (not u) -- the opposite of the standard-convention result -- and
    # specifically dv = +fx/Z * vx. (At image center, dv_std = 0 and
    # du_std = -fx/Z*vx, so du_final = dv_std = 0 and
    # dv_final = -du_std = +fx/Z*vx.)
    v_body = np.array([0.2, 0.0, 0.0])
    w_body = np.zeros(3)
    du, dv = predict_pixel_centroid_velocity(
        CX, CY, Z, v_body, w_body, np.zeros(3), np.eye(3), FX, FY, CX, CY,
        swap_uv=True,
    )
    assert abs(du) < 1e-9
    assert abs(dv - (FX / Z * 0.2)) < 1e-9


def test_swap_uv_pure_vy_lands_on_u_with_opposite_sign_to_reflection():
    # Pure vy must land on u with a POSITIVE coefficient (+fy/Z), not
    # negative -- confirmed against real bag data where the y-direction test
    # moved the predicted centroid in exactly the opposite direction from
    # the raw detections under the old reflection-based swap. This value is
    # unaffected by the du_final/dv_final sign fix below: at image center
    # pure vy gives du_std = 0, and negating a zero changes nothing.
    v_body = np.array([0.0, 0.2, 0.0])
    w_body = np.zeros(3)
    du, dv = predict_pixel_centroid_velocity(
        CX, CY, Z, v_body, w_body, np.zeros(3), np.eye(3), FX, FY, CX, CY,
        swap_uv=True,
    )
    assert abs(dv) < 1e-9
    assert abs(du - (FY / Z * 0.2)) < 1e-9  # positive, not -fy/Z*vy


def test_debug_info_matches_plain_call_exactly():
    v_body = np.array([0.1, -0.05, 0.02])
    w_body = np.array([0.0, 0.01, -0.2])
    r_cam_body = np.array([0.0, -0.115, 0.0])
    R_cam_body = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

    du, dv = predict_pixel_centroid_velocity(
        CX + 20, CY - 10, Z, v_body, w_body, r_cam_body, R_cam_body, FX, FY, CX, CY,
        swap_uv=True,
    )
    info = predict_pixel_centroid_velocity_debug(
        CX + 20, CY - 10, Z, v_body, w_body, r_cam_body, R_cam_body, FX, FY, CX, CY,
        swap_uv=True,
    )
    assert info.du_final == du
    assert info.dv_final == dv


def test_debug_info_exposes_v_cam_and_normalized_position():
    v_body = np.array([0.2, 0.0, 0.0])
    w_body = np.zeros(3)
    r_cam_body = np.array([0.0, -0.115, 0.0])
    R_cam_body = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])

    info = predict_pixel_centroid_velocity_debug(
        CX, CY, Z, v_body, w_body, r_cam_body, R_cam_body, FX, FY, CX, CY,
        swap_uv=False,
    )
    # pure body-vx maps through this extrinsic onto pure camera-Z (boresight)
    assert np.allclose(info.v_cam, [0.0, 0.0, 0.2], atol=1e-9)
    assert info.x_norm == 0.0
    assert info.y_norm == 0.0
