# topic_wiring_table

| topic | type | producer | consumer | 検証方法 | 結果 |
|---|---|---|---|---|---|
| `/detections_output` | `vision_msgs/Detection2DArray` | object detector or harness | `target_observer_node` | harness で publish | OK |
| `/sc2/visual_servo/target_obs` | `sc2_visual_servo_interfaces/TargetObservation` | `target_observer_node` | `visual_servo_controller_node` | harness subscribe | OK |
| `/waterlinked_dvl_driver/velocity_report` | clean env では `state_estimation/DVL` fallback | DVL publisher or harness | controller | harness publish | OK (fallback) |
| `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | MAVROS or harness | controller | harness publish | OK |
| `/sc2/visual_servo/status` | `std_msgs/String` | controller | debug sink | harness subscribe | OK |
| `/sc2/velocity_controller_node/outer/console_signals` | `state_estimation/Vector6` | controller | vehicle adapter/debug sink | harness subscribe | OK |
| `/best_detection` | `vision_msgs/Detection2D` | legacy path | controller compat path | code audit only | not used in staged PID observer path |
| `/sc2/visual_servo/target_point` | `geometry_msgs/PoseStamped` | `target_point_estimator_node` | controller | code audit only | compat path, default disabled in launch |

