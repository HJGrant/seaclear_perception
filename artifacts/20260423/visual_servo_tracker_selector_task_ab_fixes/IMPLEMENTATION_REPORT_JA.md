# Implementation Report

## 目的

前回の P1-P4 tracker/selector rework に対し、Task A/B の追加未達を修正した。

- Task A: `sc2_visual_servo_controller/package.xml` の依存不足を補正。
- Task B: selector continuity bias / switch margin を追加し、`REACQUIRE` は現実装では `HOLD` が兼ねる方針として明文化。

## Source Changes

- `src/sc2_visual_servo_controller/package.xml`
  - controller 実装の import / message / service / TF 使用に合わせて依存を追加。
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py`
  - `selection_score()` を追加。
  - `select_track()` に `previous_selected_track_id`, `selector_switch_margin`, `selector_selected_track_bonus` を追加。
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
  - selector continuity parameters を宣言し、前回 selected track id とともに core selector へ渡す。
- `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py`
  - `selector_switch_margin` と `selector_selected_track_bonus` の初期値を追加。
- `src/sc2_visual_servo_perception/test/test_tracker_selector_core.py`
  - 微差では previous selected track を維持し、margin 超過で切り替わる pure Python tests を追加。

## Selector Continuity

採用方式は bonus + switch margin 方式。

前回 selected だった track が `CONFIRMED` かつ fresh の場合、その track の score に `selector_selected_track_bonus` を加える。他候補へ切り替えるのは、他候補の base score が `previous_score + bonus + selector_switch_margin` を超えた場合だけである。

## REACQUIRE Policy

明示的な `REACQUIRE` state は追加していない。現実装では `HOLD` が passive reacquire を兼ねる。

- `HOLD` track は active track として association 対象に残る。
- `HOLD` track は high-confidence match または low-confidence recovery で `CONFIRMED` に戻る。
- `LOST` / `ESCALATE` 後は active association 対象から外れ、新規 high-confidence detection が new track を作る。

## Scope Boundary

今回、Hungarian assignment、full 6-DoF hold、localization setpoint station keeping、precise reprojection、P5 target_point 再接続、実機 live integration は実装していない。
