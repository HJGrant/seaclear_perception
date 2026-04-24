# Control Response Summary

controller command は actual `VisualServoControllerNode` を in-process harness で駆動して取得した。

経路:
- synthetic / bag 由来の `TargetObservation` 相当入力を `_target_observation_cb` に投入
- depth は `_dvl_altitude_cb` へ投入
- body velocity measurement は `_vel_measurement_cb` へ投入
- command は `_tick_staged_pid` から `_publish_cmd_direct_body_with_slew` 経由で取得

| case | max | mean abs vx | mean abs vy | mean abs vz | mean abs yaw | first hold status |
|---|---|---:|---:|---:|---:|---|
| `scenario_a` | 0.200 | 0.023 | 0.163 | 0.019 | 0.016 | `HOLD_SELECTED_INVALID_NONE_not_selected_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL` |
| `scenario_b_legacy_synth` | 0.350 | 0.101 | 0.045 | 0.008 | 0.218 | `HOLD_SELECTED_INVALID_NONE_not_selected_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL` |
| `scenario_b_production_like` | 0.060 | 0.023 | 0.018 | 0.016 | 0.039 | `HOLD_SELECTED_INVALID_NONE_not_selected_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL` |
| `variable_bbox_b_production_like` | 0.350 | 0.123 | 0.063 | 0.005 | 0.239 | `HOLD_SELECTED_INVALID_NONE_not_selected_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL` |
| `bag_multi_object_window` | 0.100 | 0.000 | 0.000 | 0.100 | 0.000 | `HOLD_SELECTED_INVALID_NONE_not_selected_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL` |
