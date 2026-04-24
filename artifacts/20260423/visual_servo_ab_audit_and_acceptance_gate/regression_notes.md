# regression_notes

## DVL message fallback

- controller は `marine_acoustic_msgs/Dvl` を優先し、clean env では `state_estimation/DVL` を使う。
- 実機で actual publisher が `marine_acoustic_msgs/Dvl` の場合、従来挙動は維持される。
- 実機で topic type が別の場合は launch/runtime 再検証が必要。

## bringup launch remains externally blocked

- `object_detection_ibvs.launch.py` は依然として `oakd_isaac_ros` install を前提にしている。
- clean env では `oakd_isaac_ros` build が `foxglove_msgs` 欠落で止まるため、production launch acceptance は未達。

## artifact snapshots and colcon

- `code_snapshot/` は `COLCON_IGNORE` を付けた。
- 新しい snapshot を追加する場合も同様の対策が必要。

