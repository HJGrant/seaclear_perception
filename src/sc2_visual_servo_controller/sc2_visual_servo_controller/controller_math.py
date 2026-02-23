import math
from dataclasses import dataclass
from typing import Tuple


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
