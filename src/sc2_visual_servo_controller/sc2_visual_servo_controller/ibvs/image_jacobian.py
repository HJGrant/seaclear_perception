import math
from typing import Tuple

import numpy as np

from .controller_math import get_image_jacobian


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
) -> Tuple[float, float]:
    '''Predict a target centroid's pixel velocity from body-frame vehicle
    velocity via the image Jacobian, assuming the target is stationary in the
    world (i.e. the apparent motion is entirely the vehicle's own ego-motion).

    fx/fy are taken as given -- any refraction correction is the caller's
    responsibility to fold in before calling this.
    '''
    if not math.isfinite(Z) or Z <= 0.0:
        raise ValueError(f'Z must be finite and positive, got {Z!r}')

    v_cam, w_cam = body_twist_to_camera_twist(v_body, w_body, r_cam_body, R_cam_body)
    twist_cam = np.concatenate([v_cam, w_cam])

    L = get_image_jacobian(np.array([[u, v]]), np.array([Z]), fx, fy, cx, cy)
    du, dv = L @ twist_cam
    return float(du), float(dv)
