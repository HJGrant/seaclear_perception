# Change Summary

## Task A

`src/sc2_visual_servo_controller/package.xml` に不足依存を追加した。

追加:

- `std_srvs`
- `sensor_msgs`
- `vision_msgs`
- `marine_acoustic_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `state_estimation`

`nav_msgs` は controller source で直接 import されていないため追加していない。perception package は `nav_msgs` を既に宣言している。

## Task B

Selector continuity を追加した。

- `selection_score()` を追加。
- `select_track()` に previous selected track の continuity bias を追加。
- `selector_switch_margin` と `selector_selected_track_bonus` を parameter 化。
- target observer node と launch に初期値を追加。
- stability test を追加。

## Documentation

新規成果物ディレクトリ:

`artifacts/20260423/visual_servo_tracker_selector_task_ab_fixes/`

必須文書に加えて、以下を追加した。

- `SELECTOR_CONTINUITY_DESIGN_JA.md`
- `REACQUIRE_POLICY_NOTE_JA.md`
