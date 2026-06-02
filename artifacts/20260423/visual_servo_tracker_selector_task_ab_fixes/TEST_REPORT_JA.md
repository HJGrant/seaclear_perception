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
  src/sc2_visual_servo_controller/test/test_hold_maintain_core.py
```

Result: success.

### Perception pure Python unit test

```bash
PYTHONPATH=src/sc2_visual_servo_perception \
uv run python -m unittest src/sc2_visual_servo_perception/test/test_tracker_selector_core.py
```

Result:

```text
Ran 7 tests in 0.001s
OK
```

追加 coverage:

- previous selected confirmed track が微差では維持されること
- challenger が `selector_switch_margin` を超えた場合に切り替わること

### Controller pure Python direct-call tests

```bash
PYTHONPATH=src/sc2_visual_servo_controller uv run python <direct-call harness>
```

Result:

```text
PASS 10 direct-call tests
```

Coverage:

- HOLD entry condition
- HOLD velocity damping sign / clamp
- existing controller sign math

## 未実行 / 実行不能

```bash
python3 -m pytest -q ...
```

Result:

```text
/usr/bin/python3: No module named pytest
```

`ros2` と `colcon` は host で見つからないため、ROS integration / clean colcon build は未実行。

## 次に Task C で確認すること

- clean ROS environment で `colcon build` が dependency error なく通ること。
- `/detections_output` replay / live run で selected track id が微差 confidence で揺れないこと。
- `HOLD` 中の low-confidence recovery が実際に同一 track id を維持して `CONFIRMED` に戻ること。
