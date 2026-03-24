import math
from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class Observation:
    u: float
    v: float
    w: float
    h: float
    depth_m: float


@dataclass
class IBVSConfig:
    image_width: float = 1280.0
    image_height: float = 720.0
    target_u: float = 640.0
    target_v: float = 360.0
    desired_depth_m: float = 1.0
    desired_bbox_area_px: float = 18000.0
    k_u: float = 0.6
    k_v: float = 0.6
    k_depth: float = 0.5
    k_area: float = 0.3
    k_yaw: float = 0.8


@dataclass
class BBoxApproachConfig:
    image_width: float = 1280.0
    image_height: float = 720.0
    target_u: float = 640.0
    target_v: float = 360.0
    desired_depth_m: float = 1.0
    stop_area_px: float = 18000.0
    k_u: float = 0.6
    k_v: float = 0.6
    k_depth: float = 0.5
    k_area: float = 0.3
    k_yaw: float = 0.8
    use_vy: bool = True
    use_vz: bool = True


def is_in_roi(x, y, center, size):
    '''Checks whether a 2D point is within a sqaure region of interest with side length 'size'
    Args:
        x (int) : the x coordinate of the pixel 
        y (int) : the y coordinate of the pixel
        cx (int) : x coordinate of ROI center
        cy (int) : y coordinate of ROI center
        size (int) : side length of the square ROI

    Returns:
        True OR False : depending on whether or not the point (x, y) is in the ROI

    '''
    return (center[0] - size <= x <= center[0] + size and 
            center[1] - size <= y <= center[1] + size)

def get_image_jacobian(pts_2d, depths, fx, fy, cx, cy):
    """
    Computes the interaction matrix (Image Jacobian) for multiple points.
    
    :param pts_2d: np.array of shape (N, 2) -> [[u, v], ...]
    :param depths: np.array of shape (N,) -> [Z1, Z2, ...]
    :param fx, fy, cx, cy: Camera intrinsic parameters
    :return: Combined Jacobian matrix of shape (2*N, 6)
    """
    N = pts_2d.shape[0]
    J_full = np.zeros((2 * N, 6))

    for i in range(N):
        u, v = pts_2d[i]
        Z = depths[i]
        
        # Normalized coordinates (x, y)
        x = (u - cx) / fx
        y = (v - cy) / fy

        # Standard Interaction Matrix formula for a single point
        # mapping [vx, vy, vz, wx, wy, wz] -> [du, dv]
        Ji = np.array([
            [-fx/Z,  0,     fx*x/Z,  fx*x*y,      -fx*(1 + x**2),  fx*y],
            [ 0,    -fy/Z,  fy*y/Z,  fy*(1 + y**2), -fy*x*y,      -fy*x]
        ])

        J_full[2*i:2*i+2, :] = Ji

    return J_full

def get_uncalibrated_jacobian(e, prev_e, nu, J_hat, Pk, lmbda_rls, dt):
    h_vec = nu.reshape(-1, 1) * dt
    delta_e = (e - prev_e).reshape(-1, 1)

    # 1. Magnitude Check: Don't update if we aren't moving much
    if np.linalg.norm(h_vec) < 1e-3:
        print(f"Movement Too Small, np.linalg.norm(h_vec) = {np.linalg.norm(h_vec)}")
        return J_hat, Pk, (e - prev_e)

    # 2. RLS Math
    denominator = lmbda_rls + (h_vec.T @ Pk @ h_vec)
    innovation = delta_e - (J_hat @ h_vec)
    
    # Update Jacobian
    J_update = (innovation @ h_vec.T @ Pk) / denominator
    
    # 3. Projection: Zero out updates for columns with near-zero velocity
    # This prevents the "inactive" columns from drifting due to noise
    mask = (np.abs(nu) > 1e-4).astype(float)
    print(f"J_hat mask: {mask}. Mask not applied...")
    J_hat += J_update #* mask # Only update active DOFs
    
    # Update Covariance
    Pk_prev = Pk
    Pk = (1 / lmbda_rls) * (Pk - (Pk @ h_vec @ h_vec.T @ Pk / denominator))
    print(f"Difference in the Covariance {np.linalg.norm(Pk - Pk_prev, ord='fro')} ")

    return J_hat, Pk, (e - prev_e)


def clamp(val: float, lim: float) -> float:
    return max(-lim, min(lim, val))


def compute_ibvs_command(obs: Observation, cfg: IBVSConfig) -> Tuple[float, float, float, float]:
    ex = (obs.u - cfg.target_u) / max(1.0, cfg.image_width)
    ey = (obs.v - cfg.target_v) / max(1.0, cfg.image_height)

    vy = -cfg.k_u * ex
    vz = -cfg.k_v * ey
    yaw_rate = -cfg.k_yaw * ex

    if math.isfinite(obs.depth_m) and obs.depth_m > 0.0:
        vx = cfg.k_depth * (obs.depth_m - cfg.desired_depth_m)
    else:
        area = max(1.0, obs.w * obs.h)
        area_err = (cfg.desired_bbox_area_px - area) / max(1.0, cfg.desired_bbox_area_px)
        vx = cfg.k_area * area_err

    return vx, vy, vz, yaw_rate


def compute_bbox_approach_command(
    obs: Observation,
    cfg: BBoxApproachConfig,
) -> Tuple[float, float, float, float]:
    ex = (obs.u - cfg.target_u) / max(1.0, cfg.image_width)
    ey = (obs.v - cfg.target_v) / max(1.0, cfg.image_height)

    yaw_rate = -cfg.k_yaw * ex
    vy = -cfg.k_u * ex if cfg.use_vy else 0.0
    vz = -cfg.k_v * ey if cfg.use_vz else 0.0

    if math.isfinite(obs.depth_m) and obs.depth_m > 0.0:
        vx = cfg.k_depth * (obs.depth_m - cfg.desired_depth_m)
    else:
        area = max(1.0, obs.w * obs.h)
        area_err = (cfg.stop_area_px - area) / max(1.0, cfg.stop_area_px)
        vx = cfg.k_area * max(0.0, area_err)

    return vx, vy, vz, yaw_rate
