# Test Report

## 実行した確認

### Python compile

```bash
python3 -m py_compile \
  src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py \
  src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py \
  src/sc2_visual_servo_controller/sc2_visual_servo_controller/hold_maintain_core.py \
  src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py \
  src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py \
  src/sc2_visual_servo_perception/test/test_tracker_selector_core.py \
  src/sc2_visual_servo_perception/test/test_target_observer_integration.py \
  src/sc2_visual_servo_controller/test/test_hold_maintain_core.py
```

Result: success

### Pure Python unit test

```bash
PYTHONPATH=src/sc2_visual_servo_perception \
uv run python -m unittest src/sc2_visual_servo_perception/test/test_tracker_selector_core.py
```

Result:

```text
Ran 5 tests in 0.000s
OK
```

Covered:

- freshness clamp
- motion-aware gate scale cap
- high-confidence stage plus low-confidence second association
- CONFIRMED -> HOLD and selection over multiple tracks
- stale CONFIRMED track remains HOLD until miss budget is exceeded

```bash
PYTHONPATH=src/sc2_visual_servo_controller \
uv run python - <<'PY'
import importlib.util
from pathlib import Path

for path in [
    Path('src/sc2_visual_servo_controller/test/test_hold_maintain_core.py'),
    Path('src/sc2_visual_servo_controller/test/test_controller_sign.py'),
]:
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in sorted(dir(module)):
        if name.startswith('test_'):
            getattr(module, name)()
            print(f'PASS {path}:{name}')
PY
```

Result: 10 direct-call pure Python tests passed.

Covered:

- HOLD entry condition when `selected_valid=false`
- legacy detection path not forced into TargetObservation HOLD
- body velocity damping sign and clamping
- existing controller sign math regression tests

## 追加したが未実行の ROS integration test

`test_target_observer_integration.py` に tracker confirm / low-confidence recovery / HOLD の ROS-node test を残した。

未実行理由:

- system Python に `pytest` がない
- system Python に `numpy` がないため controller sign tests は `uv run python` で実行した
- host に `ros2` がない
- host に `colcon` がない

## 実行不能確認

Previous environment checks:

```text
pytest: command not found
python3 -m pytest: No module named pytest
system python direct controller test: No module named numpy
which ros2: no output
which colcon: no output
```

## 次に実機 / ROS integration で確認すべきこと

- `/detections_output` replay で複数 track が同時保持されること
- class `3` 以外の detections が混じるときの association stability
- low-confidence recovery が false positive を拾いすぎないこと
- `selected_valid=false` 時に visual correction が止まり、DVL vertical maintain と body velocity damping が出ること
- DVL stale 時に `HOLD_MAINTAIN_*_NO_DEPTH` になり、body velocity が fresh なら XY/yaw damping は残ること
- body velocity stale 時に `HOLD_MAINTAIN_NO_VELOCITY_*` になること
- `/sc2/visual_servo/status` の `HOLD_SELECTED_INVALID_*_HOLD_MAINTAIN_*` が Foxglove で見えること
