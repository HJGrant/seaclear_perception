import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .controller_math import get_image_jacobian

# swap_uv's output combination: (du_std, dv_std) -> (dv_std, -du_std).
# Empirically derived from real bag data (see predict_pixel_centroid_velocity's
# docstring) -- deliberately not a symmetric swap, so this is expressed as an
# explicit matrix rather than array reversal, and reused for both the mean
# and the covariance propagation so they can't drift apart.
_SWAP_UV_OUTPUT_ROTATION = np.array([
    [0.0, 1.0],
    [-1.0, 0.0],
])


@dataclass
class JacobianDebugInfo:
    '''Intermediate values from a predict_pixel_centroid_velocity_debug call,
    for inspecting what actually drove a prediction instead of assuming a
    simplified (identity-extrinsic, centered-target) mental model -- e.g. a
    non-identity R_cam_body can make the *dominant* term for a given motion a
    position-dependent cross-term rather than the "obvious" direct
    translation term, which a fixed axis-relabeling correction can't capture
    correctly without knowing which term actually mattered.
    '''
    v_cam: np.ndarray       # (3,) linear velocity, camera-frame
    w_cam: np.ndarray       # (3,) angular velocity, camera-frame
    x_norm: float           # normalized coordinate actually evaluated (post swap_uv substitution)
    y_norm: float
    du_std: float           # raw get_image_jacobian output, before any swap_uv output relabeling
    dv_std: float
    du_final: float         # final (du, dv) after swap_uv relabeling if enabled -- matches
    dv_final: float         # predict_pixel_centroid_velocity's return exactly


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    '''Standard unit-quaternion (x, y, z, w) to 3x3 rotation matrix, matching
    ROS/geometry_msgs.msg.Quaternion field order and tf2's convention: if the
    quaternion represents "orientation of frame B in frame A", this returns
    R such that v_A = R @ v_B.
    '''
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def body_twist_to_camera_twist(
    v_body: np.ndarray,
    w_body: np.ndarray,
    r_cam_body: np.ndarray,
    R_cam_body: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    '''Rotate a rigid-body twist from the body frame into the camera frame.

    Args:
        v_body: linear velocity of the body origin, expressed in body-frame axes.
        w_body: angular velocity of the body, expressed in body-frame axes.
        r_cam_body: position of the camera origin relative to the body origin,
            expressed in body-frame axes (the lever arm).
        R_cam_body: rotation matrix mapping body-frame vectors into camera-frame
            axes, i.e. v_camera_frame = R_cam_body @ v_body_frame.

    Returns:
        (v_cam, w_cam): linear/angular velocity of the camera origin, expressed
        in camera-frame axes.
    '''
    v_at_camera_in_body_axes = v_body + np.cross(w_body, r_cam_body)
    v_cam = R_cam_body @ v_at_camera_in_body_axes
    w_cam = R_cam_body @ w_body
    return v_cam, w_cam


def skew_matrix(r: np.ndarray) -> np.ndarray:
    '''3x3 skew-symmetric matrix of r, such that skew_matrix(r) @ v == cross(r, v).'''
    rx, ry, rz = r
    return np.array([
        [0.0, -rz, ry],
        [rz, 0.0, -rx],
        [-ry, rx, 0.0],
    ])


def body_twist_covariance_to_camera_twist_covariance(
    twist_body_cov: np.ndarray,
    r_cam_body: np.ndarray,
    R_cam_body: np.ndarray,
) -> np.ndarray:
    '''Propagate a 6x6 body-frame twist covariance (order [vx,vy,vz,wx,wy,wz],
    e.g. from a nav_msgs/Odometry.twist.covariance) into the camera frame,
    through the same linear lever-arm + rotation map body_twist_to_camera_twist
    applies to the mean. The map is linear in [v_body, w_body] given fixed
    extrinsics, so this is exact (first-order-exact, i.e. not a linearization
    approximation) for this step -- the interaction matrix step downstream
    that maps camera twist to pixel velocity is the only place a local
    linearization is actually involved.
    '''
    skew_r = skew_matrix(r_cam_body)
    T = np.zeros((6, 6))
    T[0:3, 0:3] = R_cam_body
    T[0:3, 3:6] = -R_cam_body @ skew_r
    T[3:6, 3:6] = R_cam_body
    return T @ twist_body_cov @ T.T


def is_valid_twist_covariance(cov: np.ndarray) -> bool:
    '''Reject an unpopulated/sentinel covariance before propagating it --
    some publishers leave covariance all-zero when not configured, and the
    common ROS convention for "this field is not provided" is a negative
    value (e.g. -1) on the diagonal. Either would silently corrupt a
    covariance propagation if used as-is, so callers should fall back to a
    default (e.g. no dynamic Q) rather than trust this.
    '''
    if cov.shape != (6, 6):
        return False
    diag = np.diag(cov)
    if np.any(diag < 0.0):
        return False
    if np.all(diag == 0.0):
        return False
    return True


def predict_pixel_velocity_covariance(
    u: float,
    v: float,
    Z: float,
    twist_body_cov: np.ndarray,
    r_cam_body: np.ndarray,
    R_cam_body: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    swap_uv: bool = False,
) -> np.ndarray:
    '''Propagate a body-twist covariance into a 2x2 covariance on the
    predicted (du, dv) pixel velocity, via the same interaction matrix used
    for the mean prediction (predict_pixel_centroid_velocity). Intended to
    drive a Kalman filter's process noise from a real velocity-source
    covariance (e.g. an EKF's twist.covariance) instead of a hand-tuned flat
    constant -- the caller is responsible for the dt scaling convention (this
    returns a (px/s)^2-ish rate-like quantity, not something already
    integrated over a time step).

    swap_uv: see predict_pixel_centroid_velocity -- applies the same
    u/v-swap-plus-fy-negation input substitution, then applies the same
    _SWAP_UV_OUTPUT_ROTATION to the resulting 2x2 covariance (not a plain
    row/column swap, since the mean transform isn't a plain swap either).
    '''
    if not math.isfinite(Z) or Z <= 0.0:
        raise ValueError(f'Z must be finite and positive, got {Z!r}')

    if swap_uv:
        u, v = v, u
        fy = -fy

    twist_cam_cov = body_twist_covariance_to_camera_twist_covariance(
        twist_body_cov, r_cam_body, R_cam_body,
    )
    L = get_image_jacobian(np.array([[u, v]]), np.array([Z]), fx, fy, cx, cy)
    cov = L @ twist_cam_cov @ L.T
    if swap_uv:
        cov = _SWAP_UV_OUTPUT_ROTATION @ cov @ _SWAP_UV_OUTPUT_ROTATION.T
    return cov


def apply_refraction_scale(fx: float, fy: float, scale: float) -> Tuple[float, float]:
    '''Placeholder for the underwater refraction correction on focal length.

    scale=1.0 is a no-op (air-calibrated intrinsics, uncorrected). The real
    correction factor -- and whether a flat scalar is even an adequate model
    of the refraction effect -- is TBD empirically from ROS bag data; until
    then this just documents where that correction plugs in.
    '''
    return fx * scale, fy * scale


def predict_pixel_centroid_velocity(
    u: float,
    v: float,
    Z: float,
    v_body: np.ndarray,
    w_body: np.ndarray,
    r_cam_body: np.ndarray,
    R_cam_body: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    swap_uv: bool = False,
) -> Tuple[float, float]:
    '''Predict a target centroid's pixel velocity from body-frame vehicle
    velocity via the image Jacobian, assuming the target is stationary in the
    world (i.e. the apparent motion is entirely the vehicle's own ego-motion).

    fx/fy are taken as given -- any refraction correction is the caller's
    responsibility to fold in before calling this.

    get_image_jacobian assumes the standard pinhole convention (u increases
    rightward, v increases downward). If a detector's axes don't follow that,
    set swap_uv=True. The exact transform here (u/v swapped as position
    inputs with fy negated; du_final=dv_std, dv_final=-du_std on the way out)
    was derived directly from real bag data rather than an a-priori model of
    the detector's convention: with this vehicle's actual (non-identity)
    extrinsic, the dominant term for a given body motion is often a
    position-dependent cross-term rather than a "clean" axis-aligned
    translation term, which made two earlier, more principled-looking
    derivations (a plain reflection, then a naive 90-degree rotation) both
    wrong in a way that only showed up against real data, not the
    identity-extrinsic/centered-target cases exercised by unit tests. Trust
    this against further real bag data (e.g. via
    predict_pixel_centroid_velocity_debug / --print-jacobian-debug) before
    assuming it generalizes to a different vehicle/mount.

    This is a thin wrapper around predict_pixel_centroid_velocity_debug
    (returning just its final (du, dv)) so the two can never disagree --
    see that function to inspect the intermediate values (v_cam/w_cam,
    normalized position, pre-relabeling output) instead of assuming which
    term is dominant.
    '''
    info = predict_pixel_centroid_velocity_debug(
        u, v, Z, v_body, w_body, r_cam_body, R_cam_body, fx, fy, cx, cy, swap_uv,
    )
    return info.du_final, info.dv_final


def predict_pixel_centroid_velocity_debug(
    u: float,
    v: float,
    Z: float,
    v_body: np.ndarray,
    w_body: np.ndarray,
    r_cam_body: np.ndarray,
    R_cam_body: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    swap_uv: bool = False,
) -> JacobianDebugInfo:
    '''Same computation as predict_pixel_centroid_velocity, but returns every
    intermediate quantity instead of just the final (du, dv) -- use this to
    see exactly what drove a prediction (which term actually dominated, what
    the camera-frame twist and normalized position were) rather than
    reasoning from a simplified/assumed model of the extrinsic.
    '''
    if not math.isfinite(Z) or Z <= 0.0:
        raise ValueError(f'Z must be finite and positive, got {Z!r}')

    u_arg, v_arg, fx_arg, fy_arg = u, v, fx, fy
    if swap_uv:
        u_arg, v_arg = v, u
        fy_arg = -fy

    v_cam, w_cam = body_twist_to_camera_twist(v_body, w_body, r_cam_body, R_cam_body)
    twist_cam = np.concatenate([v_cam, w_cam])

    x_norm = (u_arg - cx) / fx_arg
    y_norm = (v_arg - cy) / fy_arg

    L = get_image_jacobian(np.array([[u_arg, v_arg]]), np.array([Z]), fx_arg, fy_arg, cx, cy)
    du_std, dv_std = L @ twist_cam

    if swap_uv:
        du_final, dv_final = _SWAP_UV_OUTPUT_ROTATION @ np.array([du_std, dv_std])
    else:
        du_final, dv_final = du_std, dv_std

    return JacobianDebugInfo(
        v_cam=v_cam, w_cam=w_cam,
        x_norm=float(x_norm), y_norm=float(y_norm),
        du_std=float(du_std), dv_std=float(dv_std),
        du_final=float(du_final), dv_final=float(dv_final),
    )
