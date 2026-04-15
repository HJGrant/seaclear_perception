# Controller Tolerance Update

Date: 2026-04-15

Branch: `grapple_integration`

## 25 px Audit

25 px assumptions were found in these active controller paths:

| Location | Previous value/use | Action |
|---|---|---|
| `visual_servo_controller_node_implementation.py` staged PID declarations | `staged_pid_horizontal_tolerance_px = 25.0` | changed default to `150.0` |
| same | `staged_pid_horizontal_tolerance_x_px = 25.0` | changed default to `150.0`; this is now the u-axis gate source |
| same | `staged_pid_horizontal_tolerance_y_px = 25.0` | changed default to `150.0`; this is now the v-axis gate source |
| same | `staged_pid_vertical_tolerance_px = 25.0` | changed default to `150.0` and treated as deprecated y-axis alias |
| controller tolerance compatibility branch | compared x/y defaults against literal `25.0` | replaced with `default_vertical_gate_tol_px = 150.0` |
| `_update_staged_pid_vertical_gate()` | already used x/y tolerances | retained, now using 150 px defaults |
| `_compute_staged_pid_command()` aligned status | used x tolerance plus separate `vertical_tolerance_px` for y | changed to use the same u/v gate helper as vertical on/off |
| `object_detection_ibvs.launch.py` | overrides all staged PID tolerances to `25.0` | changed launch-equivalent defaults to `150.0` |
| Phase 2 compare scripts | fixed launch-equivalent tolerances at `25.0` | changed to `150.0` and added axis-over-threshold cases |

Historical artifacts and old branch snapshots still contain 25 px because they
are frozen reference material; they were not edited.

## Tolerance Design

Vertical z output is now gated by the same condition everywhere:

```text
abs(u - u*) <= staged_pid_horizontal_tolerance_x_px
and
abs(v - v*) <= staged_pid_horizontal_tolerance_y_px
```

Default:

```text
staged_pid_horizontal_tolerance_x_px = 150.0
staged_pid_horizontal_tolerance_y_px = 150.0
```

`staged_pid_horizontal_tolerance_px` remains as a shared legacy alias when the
x/y values are left at default. `staged_pid_vertical_tolerance_px` remains as a
deprecated y-axis alias for older configs, but vertical on/off and aligned status
now use the same u/v helper.

## Physical Approximation

The controller source now documents:

- HFOV ~= 66 deg
- bbox/controller image width = 640 px
- at 0.5 m range, 1 px ~= 1 mm laterally
- 25 px ~= 2.5 cm
- 150 px ~= 15 cm
- 200 px ~= 20 cm

The adopted 150 px gate is slightly stricter than a 0.2 m lateral-offset
allowance at 0.5 m, without requiring near-perfect centering.

## Changed Files

- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py`
- `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py`
- `tools/compare_visual_servo_controller_nodes_phase2.py`
- `tools/compare_visual_servo_controller_nodes_phase2_bluerov.py`
- `artifacts/20260415/controller_tolerance_update_01/...`

## Verification

| Command | Result |
|---|---|
| `uv run python tools/compare_visual_servo_controller_nodes_phase1.py` | PASS |
| `uv run python tools/compare_visual_servo_controller_nodes_phase2.py --out-dir artifacts/20260415/controller_tolerance_update_01/phase2_baseline_compare` | PASS |
| `uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py --out-dir artifacts/20260415/controller_tolerance_update_01/phase2_bluerov_compare` | PASS with 3 expected differences |
| `uv run python tools/compare_staged_pid_horizontal_behavior.py` | PASS |
| `PYTHONPATH=src/sc2_visual_servo_controller uv run pytest src/sc2_visual_servo_controller/test/test_controller_sign.py` | 6 passed |
| `python3 -m py_compile ...` | PASS |

## Added Compare Coverage

`tools/compare_visual_servo_controller_nodes_phase2.py` now includes:

- `case_d_v_axis_over_150_vertical_off`
- `case_e_u_axis_over_150_vertical_off`

`tools/compare_visual_servo_controller_nodes_phase2_bluerov.py` now includes:

- `case_i_v_axis_over_150_vertical_off`
- `case_j_u_axis_over_150_vertical_off`

These cases first enter vertical control, then move only one axis beyond 150 px.
Expected result:

```text
horizontal_aligned_now = False
allow_z = False
vz = 0.0
staged_state = HORIZONTAL
```

## Perception Next-Step Notes

Do not proceed with search/recovery before cleaning up perception z semantics.
The next perception work should address:

1. Decide whether controller vertical measurement is target range or frame-z offset.
2. Fix/document `target_point` field semantics accordingly.
3. Port temporal EMA and jump rejection from `dev/aoki_perception_module`.
4. Add depth-image freshness gating in `target_point_estimator_node`.

