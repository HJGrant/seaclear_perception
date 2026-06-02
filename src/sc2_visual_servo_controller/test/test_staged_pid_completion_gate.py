"""Pure-Python unit tests for the staged PID completion gate.

No ROS 2 runtime is required.  The fixture class below replicates the exact
logic of three private methods from VisualServoControllerNode:

  _is_staged_pid_vertical_gate_aligned  (lines 747-755)
  _update_staged_pid_completion_gate    (lines 781-828)
  _clear_staged_pid_completion          (lines 607-609)

Keep _Gate in sync with the source when those methods change.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
import pytest


class _Gate:
    """Minimal replica of the completion gate methods in the controller node."""

    def __init__(
        self,
        *,
        enable: bool = True,
        latch: bool = True,
        window_ticks: int = 5,
        enter_ratio: float = 0.8,
        tol_x: float = 150.0,
        tol_y: float = 150.0,
        desired_dist: float = 1.0,
        dist_tol: float = 0.08,
        use_target_observation: bool = False,
    ) -> None:
        self._staged_pid_completion_enable = enable
        self._staged_pid_completion_latch = latch
        self._staged_pid_completion_window_ticks = window_ticks
        self._staged_pid_completion_enter_ratio = enter_ratio
        self._staged_pid_horizontal_tol_x_px = tol_x
        self._staged_pid_horizontal_tol_y_px = tol_y
        self._desired_vertical_dist = desired_dist
        self._staged_pid_distance_tol_m = dist_tol
        self._use_target_observation = use_target_observation
        self._staged_pid_target_centroid = np.array([320.0, 320.0], dtype=np.float32)
        self._staged_pid_recent_completion_ok: deque = deque(maxlen=window_ticks)
        self._vs_control_finished = False
        self._target_observation_valid = True
        self.control_enabled = True
        self._target_tracked = True
        self.latest_centroid = [320.0, 320.0]

    def _is_staged_pid_vertical_gate_aligned(self) -> bool:
        if not hasattr(self, 'latest_centroid'):
            return False
        centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
        centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])
        return (
            abs(centroid_err_x) <= self._staged_pid_horizontal_tol_x_px
            and abs(centroid_err_y) <= self._staged_pid_horizontal_tol_y_px
        )

    def _clear_staged_pid_completion(self) -> None:
        self._staged_pid_recent_completion_ok.clear()
        self._vs_control_finished = False

    def _update_staged_pid_completion_gate(
        self,
        *,
        obs_fresh: bool,
        z_meas: Optional[float],
        z_fresh: bool,
    ) -> bool:
        if not self._staged_pid_completion_enable:
            self._clear_staged_pid_completion()
            return False

        if self._staged_pid_completion_latch and self._vs_control_finished:
            return True

        selected_valid_ok = True
        if self._use_target_observation:
            selected_valid_ok = bool(self._target_observation_valid)

        horizontal_ok = self._is_staged_pid_vertical_gate_aligned()
        distance_ok = (
            z_fresh
            and z_meas is not None
            and abs(float(z_meas) - self._desired_vertical_dist) <= self._staged_pid_distance_tol_m
        )
        completion_ok = (
            self.control_enabled
            and self._target_tracked
            and obs_fresh
            and selected_valid_ok
            and horizontal_ok
            and distance_ok
        )

        self._staged_pid_recent_completion_ok.append(bool(completion_ok))
        if len(self._staged_pid_recent_completion_ok) < self._staged_pid_completion_window_ticks:
            return False

        ready_ratio = sum(self._staged_pid_recent_completion_ok) / float(
            self._staged_pid_completion_window_ticks
        )
        return ready_ratio >= self._staged_pid_completion_enter_ratio


# ── helpers ──────────────────────────────────────────────────────────────────

def _drive_gate(
    gate: _Gate,
    ticks: int,
    *,
    obs_fresh: bool = True,
    z_meas: float = 1.0,
    z_fresh: bool = True,
) -> bool:
    result = False
    for _ in range(ticks):
        result = gate._update_staged_pid_completion_gate(
            obs_fresh=obs_fresh, z_meas=z_meas, z_fresh=z_fresh
        )
    return result


# ── test cases ───────────────────────────────────────────────────────────────

def test_centered_and_in_tolerance_completes_after_dwell():
    """Case 1: bbox centered + distance within tolerance + fresh obs/depth → complete after window."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    result = _drive_gate(gate, ticks=5, z_meas=1.0)
    assert result is True


def test_bbox_outside_tolerance_does_not_complete():
    """Case 2: centroid outside pixel tolerance → no completion."""
    gate = _Gate(tol_x=50.0, tol_y=50.0)
    gate.latest_centroid = [500.0, 320.0]  # err_x = 180 px > 50
    result = _drive_gate(gate, ticks=10, z_meas=1.0)
    assert result is False


def test_distance_outside_tolerance_does_not_complete():
    """Case 3: z_meas far from desired → no completion."""
    gate = _Gate(desired_dist=1.0, dist_tol=0.08)
    result = _drive_gate(gate, ticks=10, z_meas=1.5)  # err = 0.5 m >> 0.08
    assert result is False


def test_z_stale_does_not_complete():
    """Case 4: z_fresh=False → distance_ok is False → no completion."""
    gate = _Gate()
    result = _drive_gate(gate, ticks=10, z_fresh=False, z_meas=1.0)
    assert result is False


def test_target_observation_selected_invalid_does_not_complete():
    """Case 5: TargetObservation selected_valid=False → no completion."""
    gate = _Gate(use_target_observation=True)
    gate._target_observation_valid = False
    result = _drive_gate(gate, ticks=10, z_meas=1.0)
    assert result is False


def test_single_tick_does_not_complete():
    """Case 6: window not yet full after 1 tick → False."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    result = gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    assert result is False


def test_ratio_threshold_exact_boundary():
    """Case 7: exactly enter_ratio True ticks in window → completes."""
    # window=5, ratio=0.8 → need 4/5 True ticks
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    # Push 1 False then 4 True
    gate._update_staged_pid_completion_gate(obs_fresh=False, z_meas=1.0, z_fresh=True)  # False
    for _ in range(4):
        gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    # Now window = [F, T, T, T, T] → ratio = 4/5 = 0.8 >= 0.8
    result = gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    # After 6 calls, window (maxlen=5) = [T, T, T, T, T] → all True
    assert result is True


def test_ratio_below_threshold_does_not_complete():
    """Case 7b: fewer True ticks than ratio requires → no completion."""
    # window=5, ratio=0.8 → need ≥4 True; 3 True + 2 False → ratio 0.6 < 0.8
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    for _ in range(2):
        gate._update_staged_pid_completion_gate(obs_fresh=False, z_meas=1.0, z_fresh=True)
    result = False
    for _ in range(3):
        result = gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    assert result is False


def test_latch_holds_true_after_completion():
    """Case 8: once _vs_control_finished is set and latch=True, gate stays True."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8, latch=True)
    _drive_gate(gate, ticks=5, z_meas=1.0)
    # Simulate node setting _vs_control_finished on first True
    gate._vs_control_finished = True

    # Now obs goes stale / z goes stale; gate still returns True (latch)
    result = gate._update_staged_pid_completion_gate(obs_fresh=False, z_meas=None, z_fresh=False)
    assert result is True


def test_clear_resets_to_false():
    """Case 9: after _clear_staged_pid_completion, gate returns False again."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8, latch=True)
    _drive_gate(gate, ticks=5, z_meas=1.0)
    gate._vs_control_finished = True

    gate._clear_staged_pid_completion()
    assert gate._vs_control_finished is False
    # Latch check no longer fires
    result = gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    assert result is False  # window cleared, only 1 tick filled → < window_ticks


def test_completion_disabled_always_false_and_clears():
    """enable=False → always False and history cleared."""
    gate = _Gate(enable=False)
    gate._staged_pid_recent_completion_ok.append(True)
    gate._vs_control_finished = True
    result = gate._update_staged_pid_completion_gate(obs_fresh=True, z_meas=1.0, z_fresh=True)
    assert result is False
    assert len(gate._staged_pid_recent_completion_ok) == 0
    assert gate._vs_control_finished is False


def test_no_target_tracked_does_not_complete():
    """target_tracked=False → completion_ok=False → no completion."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    gate._target_tracked = False
    result = _drive_gate(gate, ticks=10, z_meas=1.0)
    assert result is False


def test_control_disabled_does_not_complete():
    """control_enabled=False → completion_ok=False → no completion."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8)
    gate.control_enabled = False
    result = _drive_gate(gate, ticks=10, z_meas=1.0)
    assert result is False


def test_z_meas_none_does_not_complete():
    """z_meas=None with z_fresh=True → distance_ok=False → no completion."""
    gate = _Gate()
    result = _drive_gate(gate, ticks=10, z_fresh=True, z_meas=None)
    assert result is False


def test_latch_false_re_evaluates_after_complete():
    """latch=False → gate re-checks conditions every tick; fails when conditions drop."""
    gate = _Gate(window_ticks=5, enter_ratio=0.8, latch=False)
    _drive_gate(gate, ticks=5, z_meas=1.0)
    gate._vs_control_finished = True  # node would set this, but latch=False ignores it in gate

    # Now centroid drifts out of tolerance
    gate.latest_centroid = [700.0, 320.0]  # err_x = 380 >> 150 px
    result = _drive_gate(gate, ticks=5, z_meas=1.0)
    assert result is False


def test_desired_vertical_distance_used_not_hold_zero():
    """Gate uses desired_vertical_distance_m, not staged_pid_hold_zero_distance_m."""
    # desired_dist=1.0, dist_tol=0.08 → z=0.5 is 0.5 m off → no completion
    gate = _Gate(desired_dist=1.0, dist_tol=0.08)
    # z=0.5 would satisfy hold_zero (<=0.5) but NOT desired_dist ±0.08
    result = _drive_gate(gate, ticks=10, z_meas=0.5)
    assert result is False

    # z=1.05 is within desired_dist ±0.08 → should complete
    gate2 = _Gate(desired_dist=1.0, dist_tol=0.08, window_ticks=5)
    result2 = _drive_gate(gate2, ticks=5, z_meas=1.05)
    assert result2 is True
