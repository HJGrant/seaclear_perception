#!/usr/bin/env python3
"""Torch MPPI core for camera-frame visual-servo control.

Extracted from SC1 legacy package:
SC1/visual-servoing/sc_sim_visualservo/src/sc_sim_visualservo_mppi/core.py
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Tuple

import numpy as np
import torch

_AXIS_TO_INDEX = {"x": 0, "y": 1, "z": 2}


def axis_to_index(axis: str) -> int:
    try:
        return _AXIS_TO_INDEX[axis.lower()]
    except KeyError as exc:
        raise ValueError(f"Invalid axis '{axis}', expected one of x/y/z") from exc


def build_linear_map_from_axes(
    forward_axis: str,
    forward_sign: float,
    right_axis: str,
    right_sign: float,
    down_axis: str,
    down_sign: float,
) -> np.ndarray:
    """Build body->camera linear velocity map from axis/sign mapping params."""
    mapping = np.zeros((3, 3), dtype=np.float32)
    f_idx = axis_to_index(forward_axis)
    r_idx = axis_to_index(right_axis)
    d_idx = axis_to_index(down_axis)
    if len({f_idx, r_idx, d_idx}) != 3:
        raise ValueError("cam_forward_axis/cam_right_axis/cam_down_axis must be distinct")
    mapping[0, f_idx] = float(forward_sign)
    mapping[1, r_idx] = float(right_sign)
    mapping[2, d_idx] = float(down_sign)
    return mapping


def _signed_axis_tensor(p: torch.Tensor, axis_idx: int, sign: float) -> torch.Tensor:
    return float(sign) * p[..., axis_idx]


def extract_fov_coordinates(
    p: torch.Tensor,
    forward_axis_idx: int,
    forward_sign: float,
    right_axis_idx: int,
    right_sign: float,
    down_axis_idx: int,
    down_sign: float,
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Extract depth/right/down camera coordinates from raw p[...,3]."""
    d = _signed_axis_tensor(p, forward_axis_idx, forward_sign)
    u = _signed_axis_tensor(p, right_axis_idx, right_sign)
    v = _signed_axis_tensor(p, down_axis_idx, down_sign)
    return d, u, v


def fov_margin_from_positions(
    p: torch.Tensor,
    a_h: float,
    a_v: float,
    x_t: float,
    y_t: float,
    forward_axis_idx: int,
    forward_sign: float,
    right_axis_idx: int,
    right_sign: float,
    down_axis_idx: int,
    down_sign: float,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Compute FOV margin m and depth d from raw p[...,3]."""
    d, u, v = extract_fov_coordinates(
        p,
        forward_axis_idx=forward_axis_idx,
        forward_sign=forward_sign,
        right_axis_idx=right_axis_idx,
        right_sign=right_sign,
        down_axis_idx=down_axis_idx,
        down_sign=down_sign,
    )
    half_h = d * math.tan(float(a_h) * 0.5)
    half_v = d * math.tan(float(a_v) * 0.5)
    m_h = half_h - 0.5 * float(x_t) - torch.abs(u)
    m_v = half_v - 0.5 * float(y_t) - torch.abs(v)
    return torch.minimum(m_h, m_v), d


def rollout_risk_cost(
    margins: torch.Tensor,
    depths: torch.Tensor,
    *,
    mode: str,
    alpha: float,
    agg: str,
    depth_min: float = 1e-3,
    soft_violation_weight: float = 0.0,
) -> Tuple[torch.Tensor, Dict[str, float]]:
    """Return per-rollout risk cost vector and summary stats.

    margins: (K,H)
    depths: (K,H)
    """
    if margins.ndim != 2 or depths.ndim != 2:
        raise ValueError("margins and depths must be rank-2 tensors shaped (K,H)")

    margin_viol = torch.relu(-margins)
    depth_viol = torch.relu(float(depth_min) - depths)
    viol = margin_viol + depth_viol

    if agg == "max":
        v_i = torch.max(viol, dim=1).values
    elif agg == "sum":
        v_i = torch.sum(viol, dim=1)
    else:
        raise ValueError("risk_agg must be 'sum' or 'max'")

    mode = mode.lower()
    if mode == "none":
        costs = float(soft_violation_weight) * v_i
        return costs, {"viol_mean": float(v_i.mean().item())}

    if mode == "chance":
        any_violation = (viol > 0.0).any(dim=1).to(v_i.dtype)
        return any_violation, {"chance": float(any_violation.mean().item())}

    if mode == "cvar":
        k = int(math.ceil((1.0 - float(alpha)) * float(v_i.shape[0])))
        k = max(1, min(v_i.shape[0], k))
        top_vals, top_idx = torch.topk(v_i, k=k, largest=True, sorted=False)
        tail_mask = torch.zeros_like(v_i)
        tail_mask[top_idx] = 1.0
        costs = tail_mask * v_i * (float(v_i.shape[0]) / float(k))
        return costs, {"cvar": float(top_vals.mean().item()), "tail_k": float(k)}

    raise ValueError("risk_mode must be one of: none, chance, cvar")


class MPPIController:
    """Vectorized MPPI controller over relative marker position p=[px,py,pz]."""

    def __init__(self, cfg: Dict):
        use_cuda = bool(cfg.get("use_cuda", True))
        self.device = torch.device("cuda" if use_cuda and torch.cuda.is_available() else "cpu")
        self.dtype = torch.float32

        self.horizon = int(cfg["horizon_steps"])
        self.num_samples = int(cfg["num_samples"])
        self.dt = float(cfg["dt"])
        self.lam = float(cfg["lambda"])
        if self.lam <= 1e-8:
            raise ValueError("lambda must be > 0")

        self.marker_update_sign = float(cfg.get("marker_update_sign", 1.0))
        self.yaw_coupling_coeff = float(cfg.get("yaw_coupling_coeff", 0.0))

        self.w_pos = float(cfg.get("w_pos", 10.0))
        self.w_u_lin = float(cfg.get("w_u_lin", cfg.get("w_u", 0.1)))
        self.w_u_ang = float(cfg.get("w_u_ang", cfg.get("w_u", 0.1)))
        self.w_terminal = float(cfg.get("w_terminal", 20.0))
        self.beta_risk = float(cfg.get("beta_risk", 0.0))
        self.soft_violation_weight = float(cfg.get("soft_violation_weight", 0.0))

        self.risk_mode = str(cfg.get("risk_mode", "none")).lower()
        self.risk_alpha = float(cfg.get("risk_alpha", 0.9))
        self.risk_agg = str(cfg.get("risk_agg", "sum")).lower()

        self.noise_std = torch.tensor(cfg["noise_std"], dtype=self.dtype, device=self.device)
        self.control_min = torch.tensor(cfg["control_min"], dtype=self.dtype, device=self.device)
        self.control_max = torch.tensor(cfg["control_max"], dtype=self.dtype, device=self.device)
        self.lin_map = torch.tensor(cfg["lin_map"], dtype=self.dtype, device=self.device)
        self.p_des = torch.tensor(cfg.get("p_des_raw", [0.0, 0.0, 0.7]), dtype=self.dtype, device=self.device)

        self.forward_axis_idx = int(cfg["forward_axis_idx"])
        self.right_axis_idx = int(cfg["right_axis_idx"])
        self.down_axis_idx = int(cfg["down_axis_idx"])
        self.forward_sign = float(cfg["forward_sign"])
        self.right_sign = float(cfg["right_sign"])
        self.down_sign = float(cfg["down_sign"])

        self.a_h = float(cfg["a_h"])
        self.a_v = float(cfg["a_v"])
        self.x_t = float(cfg["X_T"])
        self.y_t = float(cfg["Y_T"])
        self.depth_min = float(cfg.get("depth_min", 1e-3))

        self.u_nominal = torch.zeros((self.horizon, 4), dtype=self.dtype, device=self.device)
        self.last_risk_stats: Dict[str, float] = {}

    def _clamp_controls(self, u: torch.Tensor) -> torch.Tensor:
        return torch.max(torch.min(u, self.control_max.view(1, 1, 4)), self.control_min.view(1, 1, 4))

    def _rollout(self, p0: torch.Tensor, u_seq: torch.Tensor) -> torch.Tensor:
        k = u_seq.shape[0]
        p_traj = torch.zeros((k, self.horizon + 1, 3), dtype=self.dtype, device=self.device)
        p_traj[:, 0, :] = p0.view(1, 3)

        for t in range(self.horizon):
            p_t = p_traj[:, t, :]
            u_t = u_seq[:, t, :]
            lin = u_t[:, :3]
            c_v = torch.matmul(lin, self.lin_map.t())
            delta = self.dt * self.marker_update_sign * (-c_v)

            if abs(self.yaw_coupling_coeff) > 1e-9:
                wz = u_t[:, 3]
                delta[:, self.forward_axis_idx] += (
                    self.dt * self.yaw_coupling_coeff * wz * p_t[:, self.right_axis_idx]
                )
                delta[:, self.right_axis_idx] -= (
                    self.dt * self.yaw_coupling_coeff * wz * p_t[:, self.forward_axis_idx]
                )

            p_traj[:, t + 1, :] = p_t + delta

        return p_traj

    def _trajectory_cost(self, p_traj: torch.Tensor, u_seq: torch.Tensor) -> torch.Tensor:
        diff = p_traj[:, 1:, :] - self.p_des.view(1, 1, 3)
        task = self.w_pos * torch.sum(diff * diff, dim=(1, 2))

        u_lin = u_seq[:, :, :3]
        u_ang = u_seq[:, :, 3]
        control = self.w_u_lin * torch.sum(u_lin * u_lin, dim=(1, 2))
        control = control + self.w_u_ang * torch.sum(u_ang * u_ang, dim=1)

        terminal_diff = p_traj[:, -1, :] - self.p_des.view(1, 3)
        terminal = self.w_terminal * torch.sum(terminal_diff * terminal_diff, dim=1)

        risk_cost = torch.zeros_like(task)
        if self.beta_risk > 0.0:
            margins, depths = fov_margin_from_positions(
                p_traj[:, 1:, :],
                a_h=self.a_h,
                a_v=self.a_v,
                x_t=self.x_t,
                y_t=self.y_t,
                forward_axis_idx=self.forward_axis_idx,
                forward_sign=self.forward_sign,
                right_axis_idx=self.right_axis_idx,
                right_sign=self.right_sign,
                down_axis_idx=self.down_axis_idx,
                down_sign=self.down_sign,
            )
            risk_cost, self.last_risk_stats = rollout_risk_cost(
                margins,
                depths,
                mode=self.risk_mode,
                alpha=self.risk_alpha,
                agg=self.risk_agg,
                depth_min=self.depth_min,
                soft_violation_weight=self.soft_violation_weight,
            )

        return task + control + terminal + self.beta_risk * risk_cost

    @torch.no_grad()
    def step(self, p0_raw: Iterable[float]) -> np.ndarray:
        p0 = torch.tensor(list(p0_raw), dtype=self.dtype, device=self.device)
        noise = torch.randn(
            (self.num_samples, self.horizon, 4), dtype=self.dtype, device=self.device
        ) * self.noise_std.view(1, 1, 4)

        base = self.u_nominal.view(1, self.horizon, 4)
        u_noisy = self._clamp_controls(base + noise)

        p_traj = self._rollout(p0, u_noisy)
        costs = self._trajectory_cost(p_traj, u_noisy)

        shifted = costs - torch.min(costs)
        weights = torch.exp((-1.0 / self.lam) * shifted)
        weights = weights / (torch.sum(weights) + 1e-12)

        self.u_nominal = torch.sum(weights.view(-1, 1, 1) * u_noisy, dim=0)
        self.u_nominal = torch.max(
            torch.min(self.u_nominal, self.control_max.view(1, 4)),
            self.control_min.view(1, 4),
        )

        u0 = self.u_nominal[0].clone()

        self.u_nominal[:-1, :] = self.u_nominal[1:, :].clone()
        self.u_nominal[-1, :] = self.u_nominal[-2, :].clone()

        return u0.detach().cpu().numpy()
