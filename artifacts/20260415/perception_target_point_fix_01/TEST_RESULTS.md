# Test Results

Date: 2026-04-15

## Commands Run

```bash
python3 -m py_compile \
  src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_point_estimator_node.py \
  src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py \
  src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py
```

Result: PASS

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase1.py
```

Result: PASS, `real controller horizontal-scope outputs match`

```bash
uv run python tools/compare_staged_pid_horizontal_behavior.py
```

Result: PASS, `horizontal-only staged PID outputs match`

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase2.py \
  --out-dir artifacts/20260415/perception_target_point_fix_01/phase2_baseline_compare
```

Result: PASS

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py \
  --out-dir artifacts/20260415/perception_target_point_fix_01/phase2_bluerov_compare
```

Result: PASS with 3 expected differences:

- terminal hold-zero is still only a grapple status hook
- DVL fallback is current grapple behavior, not BlueROV behavior
- target_point recovery after fallback is current grapple behavior

## Commands Blocked By Environment

```bash
pytest -q src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py
python3 -m pytest -q src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py
uv run pytest -q src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py
```

Result: blocked on this host.

- `pytest` is not installed for system Python.
- `uv run pytest` works but cannot import ROS message packages: `ModuleNotFoundError: No module named 'geometry_msgs'`.
- `source install/setup.bash` also fails because `/opt/ros/humble/local_setup.bash` is missing on this host.

The new ROS tests should be run inside the configured ROS/Isaac container.
