# Phase 2 Vertical Compare Result

Status: PASS

Comparison type: fake ROS import/drive of real `VisualServoControllerNode` implementations.

Baseline implementation: saved Phase 1 snapshot in `tools/baselines/phase1_sc2_visual_servo_controller`.
New implementation: current Phase 2 worktree.

Phase 2 vertical source behavior: `target_point_topic` / `PoseStamped.pose.position.z` is primary.
DVL altitude is used only as fallback while target_point is stale or unavailable.

## Launch Alignment

The harness uses the Phase 2 launch-equivalent parameters below.

| parameter | value |
| --- | --- |
| `target_topic` | `/sc2/visual_servo/target_obs` |
| `target_point_topic` | `/sc2/visual_servo/target_point` |
| `controller_mode` | `staged_pid` |
| `cmd_topic` | `/mavros/setpoint_velocity/cmd_vel` |
| `cmd_topic_vec_6` | `/sc2/velocity_controller_node/outer/console_signals` |
| `status_topic` | `/sc2/visual_servo/status` |
| `dvl_altitude_topic` | `/waterlinked_dvl_driver/velocity_report` |
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

## Case Results

| case | description | validation | horizontal-scope ticks | result |
| --- | --- | --- | ---: | --- |
| case_a_horizontal_only_exact | Never enters the horizontal alignment gate; includes a short detection stale tick. | horizontal_exact | 0 | PASS |
| case_b_vertical_entry | Aligned observations plus fresh target_pose enable z control. | vertical_entry | 13 | PASS |
| case_c_z_stale | Vertical gate is enabled, then target_pose becomes stale while detections stay fresh. | z_stale | 11 | PASS |
| case_d_v_axis_over_150_vertical_off | After vertical entry, v alone exceeds 150 px and immediately forces allow_z false. | axis_gate_off | 12 | PASS |
| case_e_u_axis_over_150_vertical_off | After vertical entry, u alone exceeds 150 px and immediately forces allow_z false. | axis_gate_off | 12 | PASS |

## Result

Case A passed exact horizontal-only regression for `vx`, `vy`, `vz`, `yaw`, `target_tracked`, and status on horizontal-scope ticks.

Case B passed: `allow_z=True`, `HORIZONTAL_AND_VERTICAL`, and non-zero `vz` were observed with fresh target_pose after the horizontal gate.

Case C passed: stale target_pose forced `allow_z=False` and `vz=0` while horizontal `vx/vy/yaw` matched the Phase 1 baseline on the stale tick.

Cases D/E passed: a single-axis error greater than 150 px forced `horizontal_aligned_now=False`, `allow_z=False`, `vz=0`, and `HORIZONTAL`.
