# Phase 2 Real Controller Compare Result

Status: PASS

This is a fake-ROS comparison of the real `VisualServoControllerNode` implementations.
It imports the actual controller class from `bluerov_pid` and the current `grapple_integration` worktree.

## Compared Revisions

- old: `bluerov_pid` `38535876ac1d74eb893ffa954856aff5415209f2`
- new: `grapple_integration` `237c0ac420e21854e9c4f39ce92e7715df2c004f`
- new worktree dirty: `True`

## Source Of Truth

Default experiment command:

```bash
ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=false
```

Current grapple Phase 2 default vertical source behavior is `target_point` primary plus DVL fallback.
The controller uses `/sc2/visual_servo/target_point` while it is fresh, uses DVL altitude only while target_point is stale/unavailable, and returns to target_point when it recovers.
The latest `bluerov_pid` reference does not implement this combined primary/fallback selector; it supports target_pose/target_point or MAVROS-depth source modes. Cases G/H therefore record the new fallback behavior as expected differences.

## Compare Semantics

- `status_raw` is logged exactly as published.
- `status_normalized` strips transport/source suffixes such as `_TARGET_POSE` and `_MAVROS_Z`.
- Cases A-E require exact numeric/controller-state/source match with normalized status.
- Cases I/J additionally verify that either axis exceeding 150 px gates z off immediately.
- Case F is an expected design difference: BlueROV enters `HOLD_ZERO_VELOCITY`, while current grapple Phase 2 exposes only a status hook.
- Cases G/H are expected differences because current grapple adds DVL fallback and target_point recovery on top of the BlueROV target_point-only mode.

## Launch Parameter Alignment

The fixed harness parameters mirror `object_detection_ibvs.launch.py` for Phase 2 staged PID. Extra `bluerov_pid` compatibility parameters are included where needed. `depth_estimation_mode=DVL` selects the fallback subscriber for current grapple staged PID; it does not override target_point primary.

| parameter | value |
| --- | --- |
| `target_topic` | `/sc2/visual_servo/target_obs` |
| `target_point_topic` | `/sc2/visual_servo/target_point` |
| `controller_mode` | `staged_pid` |
| `cmd_topic` | `/mavros/setpoint_velocity/cmd_vel` |
| `cmd_topic_vec_6` | `/sc2/velocity_controller_node/outer/console_signals` |
| `status_topic` | `/sc2/visual_servo/status` |
| `dvl_altitude_topic` | `/waterlinked_dvl_driver/velocity_report` |
| `mavros_depth_topic` | `/mavros/global_position/rel_alt` |
| `body_frame_id` | `base_link` |
| `rate_hz` | `10.0` |
| `timeout_ms` | `500` |
| `target_point_timeout_ms` | `500` |
| `max_vx` | `0.2` |
| `max_vy` | `0.2` |
| `max_vz` | `0.2` |
| `max_yaw_rate` | `0.35` |
| `image_width` | `640.0` |
| `image_height` | `640.0` |
| `target_u` | `320.0` |
| `target_v` | `320.0` |
| `desired_target_distance_m` | `1.0` |
| `desired_vertical_distance_m` | `1.0` |
| `staged_pid_z_source` | `target_point_primary_dvl_fallback` |
| `staged_pid_z_kp` | `0.8` |
| `staged_pid_z_ki` | `0.0` |
| `staged_pid_z_kd` | `0.1` |
| `staged_pid_z_integral_limit` | `1.0` |
| `staged_pid_enable_yaw` | `True` |
| `staged_pid_horizontal_tolerance_px` | `150.0` |
| `staged_pid_horizontal_tolerance_x_px` | `150.0` |
| `staged_pid_horizontal_tolerance_y_px` | `150.0` |
| `staged_pid_vertical_tolerance_px` | `150.0` |
| `staged_pid_distance_tolerance_m` | `0.08` |
| `staged_pid_vertical_gate_window_ticks` | `10` |
| `staged_pid_vertical_enter_ratio` | `0.8` |
| `staged_pid_vertical_exit_ratio` | `0.4` |
| `staged_pid_vertical_enter_hold_ticks` | `2` |
| `staged_pid_vertical_exit_hold_ticks` | `2` |
| `staged_pid_hold_zero_distance_m` | `0.5` |
| `tracked_enter_window_obs` | `10` |
| `tracked_enter_min_obs` | `2` |
| `tracked_exit_no_obs_sec` | `0.6` |
| `ibvs_lambda` | `0.1` |
| `alpha_1` | `0.5` |
| `decay_rate` | `0.1` |
| `depth_estimation_mode` | `DVL` |
| `mavros_depth` | `False` |

## Case Results

| case | description | validation | compared ticks | vertical_enabled ticks | allow_z ticks | nonzero_vz ticks | primary source ticks | fallback source ticks | result |
| --- | --- | --- | ---: | --- | --- | --- | --- | --- | --- |
| case_a_horizontal_only_regression | Fresh target_point is present, but the target never enters the horizontal alignment gate. | exact | 4 | old [] / new [] | old [] / new [] | old [] / new [] | old [0, 1, 2, 3] / new [0, 1, 2, 3] | old [] / new [] | PASS |
| case_b_vertical_entry | Stable centered target enables vertical gate and enters HORIZONTAL_AND_VERTICAL. | exact | 13 | old [9, 10, 11, 12, 13] / new [9, 10, 11, 12, 13] | old [9, 10, 11, 12, 13] / new [9, 10, 11, 12, 13] | old [9, 10, 11, 12, 13] / new [9, 10, 11, 12, 13] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13] | old [] / new [] | PASS |
| case_c_z_stale | target_point becomes stale after vertical is enabled; horizontal remains fresh. | exact | 11 | old [9, 10, 11] / new [9, 10, 11] | old [9, 10] / new [9, 10] | old [9, 10] / new [9, 10] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] | old [] / new [] | PASS |
| case_d_vertical_stay | Target remains aligned with fresh target_point for several ticks after vertical entry. | exact | 19 | old [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] / new [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] | old [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] / new [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] | old [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] / new [9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] | old [] / new [] | PASS |
| case_e_vertical_exit_to_horizontal | After vertical entry, horizontal alignment is broken but tracking continuity remains valid. | exact | 16 | old [9, 10, 11, 12, 13, 14, 15, 16] / new [9, 10, 11, 12, 13, 14, 15, 16] | old [9, 10, 11] / new [9, 10, 11] | old [9, 10, 11] / new [9, 10, 11] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21] | old [] / new [] | PASS |
| case_f_target_distance_threshold_hook | z_meas crosses the hold-zero distance threshold. | expected_difference | 12 | old [9, 10, 11, 12] / new [9, 10, 11, 12] | old [9, 10, 11, 12] / new [9, 10, 11, 12] | old [9, 10, 11] / new [9, 10, 11] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] | old [] / new [] | EXPECTED_DIFFERENCE |
| case_g_target_point_stale_dvl_fallback | target_point becomes stale but fresh DVL altitude keeps vertical allowed. | expected_difference | 11 | old [9, 10, 11] / new [9, 10, 11] | old [9, 10] / new [9, 10, 11] | old [9, 10] / new [9, 10, 11] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] | old [] / new [11] | EXPECTED_DIFFERENCE |
| case_h_target_point_recovers_after_fallback | After one DVL fallback tick, fresh target_point returns and becomes primary again. | expected_difference | 12 | old [9, 10, 11, 12] / new [9, 10, 11, 12] | old [9, 10, 12] / new [9, 10, 11, 12] | old [9, 10, 12] / new [9, 10, 11, 12] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12] | old [] / new [11] | EXPECTED_DIFFERENCE |
| case_i_v_axis_over_150_vertical_off | After vertical entry, v alone exceeds 150 px and immediately gates off z. | exact | 12 | old [9, 10, 11, 12] / new [9, 10, 11, 12] | old [9, 10, 11] / new [9, 10, 11] | old [9, 10, 11] / new [9, 10, 11] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] | old [] / new [] | PASS |
| case_j_u_axis_over_150_vertical_off | After vertical entry, u alone exceeds 150 px and immediately gates off z. | exact | 12 | old [9, 10, 11, 12] / new [9, 10, 11, 12] | old [9, 10, 11] / new [9, 10, 11] | old [9, 10, 11] / new [9, 10, 11] | old [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] / new [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] | old [] / new [] | PASS |

## Findings

- expected_difference: case=case_f_target_distance_threshold_hook, tick=12, field=terminal_behavior, old={'staged_state': 'HOLD_ZERO_VELOCITY', 'status_normalized': 'HOLD_ZERO_VELOCITY', 'cmd': [0.0, 0.0, 0.0, 0.0]}, new={'staged_state': 'HORIZONTAL_AND_VERTICAL', 'status_normalized': 'TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED', 'cmd': [0.005, 0.005, 0.0, 0.005000000000000001]}, input={'t': 1.2, 'u': 322.0, 'v': 318.0, 'pose_z': 0.4, 'dvl_altitude': None}, old_status_raw=HOLD_ZERO_VELOCITY, new_status_raw=TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED_TARGET_POINT
- expected_difference: case=case_g_target_point_stale_dvl_fallback, tick=11, field=fallback_behavior, old={'z_source': 'none', 'allow_z': False, 'vz': 0.0, 'status_normalized': 'TRACKING_STAGED_PID_HORIZONTAL_Z_STALE'}, new={'z_source': 'fallback', 'z_source_raw': 'dvl_fallback', 'allow_z': True, 'vz': 0.2, 'status_normalized': 'TRACKING_STAGED_PID_VERTICAL'}, input={'t': 1.6, 'u': 322.0, 'v': 318.0, 'pose_z': None, 'dvl_altitude': 1.35}, old_status_raw=TRACKING_STAGED_PID_HORIZONTAL_Z_STALE_TARGET_POSE, new_status_raw=TRACKING_STAGED_PID_VERTICAL_DVL_FALLBACK
- expected_difference: case=case_h_target_point_recovers_after_fallback, tick=12, field=primary_recovery_after_fallback, old={'fallback_ticks': [], 'last_z_source': 'primary', 'last_allow_z': True}, new={'fallback_ticks': [11], 'last_z_source': 'primary', 'last_z_source_raw': 'target_point', 'last_allow_z': True, 'last_vz': 0.09999999999999992}, input={'t': 1.7, 'u': 322.0, 'v': 318.0, 'pose_z': 1.25, 'dvl_altitude': 1.35}, old_status_raw=TRACKING_STAGED_PID_VERTICAL_TARGET_POSE, new_status_raw=TRACKING_STAGED_PID_VERTICAL_TARGET_POINT

## Result Interpretation

Cases A-E pass when `vx/vy/vz/yaw`, `target_tracked`, normalized status, staged state, `horizontal_aligned_now`, `vertical_enabled`, `allow_z`, `z_fresh`, normalized `z_source`, and `z_meas` match on compared ticks.
Raw status suffix differences are non-semantic and are preserved in JSON.

Case F documents the intentionally incomplete terminal behavior in current grapple Phase 2. BlueROV has an absorbing hold-zero state; grapple Phase 2 only reports `TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED` and keeps terminal command policy out of scope.

Case G documents DVL fallback: when target_point is stale and DVL is fresh, current grapple uses `z_source=fallback`, keeps `allow_z=True`, and continues vertical command.

Case H documents recovery: after a DVL fallback tick, a fresh target_point switches current grapple back to `z_source=primary`.

Cases I/J document the updated vertical gate tolerance: `u` and `v` are both checked, and an error greater than 150 px on either axis sets `horizontal_aligned_now=False`, `allow_z=False`, `vz=0`, and `HORIZONTAL` for that tick.

## Residual Risks

- Perception now defines `target_point.pose.position.z` as target-relative range; verify the vehicle positive-z command convention before hardware tests.
- Define DVL fallback/safety policy separately.
- Search/recovery and final hold-zero behavior remain Phase 3+ work.
