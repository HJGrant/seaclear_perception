# Implementation Report

## 実装概要

P1-P4 rework として、前回の single active track 実装を複数 track table に置き換え、controller HOLD を zero-only から DVL/altitude vertical maintain と body velocity damping へ進めた。

主経路:

```text
/detections_output
  -> target_observer_node track table / selector
  -> /sc2/visual_servo/target_obs
  -> visual_servo_controller_node staged PID HOLD gate
```

## 主な変更

- `tracker_selector_core.py` を追加し、association / freshness / motion gate / state transition を pure Python 化。
- `target_observer_node.py` は `self._tracks` に複数 track を保持するよう変更。
- two-stage association を追加。
- controller に `HOLD_MAINTAIN_*` path を追加。
- `hold_maintain_core.py` を追加し、HOLD entry 判定と body velocity damping を pure Python 化。
- launch に `max_tracks`, `hold_maintain_enable`, `hold_maintain_z_kp_scale`, `hold_maintain_*_damping` を追加。
- pure Python unit test `test_tracker_selector_core.py` と `test_hold_maintain_core.py` を追加。

## P1

複数 track table を実装した。association と selection は分離している。

## P2

motion-aware gate expansion を pure helper と node に実装した。精密 reprojection は実装していない。

## P3

high-confidence first association と low-confidence second recovery を実装した。low-confidence detection は新規 track を開始しない。

## P4

`selected_valid=false` 時に visual correction を止める。さらに、DVL/altitude が fresh なら existing z PID を用いて vertical maintain command を出し、body velocity measurement が fresh なら XY/yaw drift に対して damping command を出す。

完全な pose hold ではない。実装できたのは depth/distance maintain と body velocity damping であり、localization setpoint による horizontal station keeping は未実装。

## 変更ファイル

- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py`
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
- `src/sc2_visual_servo_perception/test/test_tracker_selector_core.py`
- `src/sc2_visual_servo_perception/test/test_target_observer_integration.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/hold_maintain_core.py`
- `src/sc2_visual_servo_controller/test/test_hold_maintain_core.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py`
- `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py`
- previous interface/dependency changes remain: `TargetObservation.msg`, `sc2_visual_servo_perception/package.xml`
