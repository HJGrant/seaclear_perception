# Topic Wiring Table

| Topic | Type | Producer | Consumer | Validation |
|---|---|---|---|---|
| `/detections_output` | `vision_msgs/msg/Detection2DArray` | rosbag2 replay | `target_observer_node` | sim time replay で `target_obs` に反映 |
| `/sc2/imu/imu` | `sensor_msgs/msg/Imu` | rosbag2 replay | `target_observer_node` | motion-aware gate input として使用 |
| `/sc2/odometry_ekf` | `nav_msgs/msg/Odometry` | rosbag2 replay | `target_observer_node` | motion-aware gate input として使用 |
| `/sc2/visual_servo/target_obs` | `sc2_visual_servo_interfaces/msg/TargetObservation` | `target_observer_node` | controller, probe | `obs_count=414` |
| `/sc2/visual_servo/status` | `std_msgs/msg/String` | controller | probe | HOLD / tracking status 取得 |
| `/sc2/visual_servo/target_locked` | `std_msgs/msg/String` | controller | probe | `target_locked_count=160` |
| `/sc2/velocity_controller_node/outer/console_signals` | `state_estimation/msg/Vector6` | controller | probe | `cmd_count=160`, nonzero HOLD cmd 確認 |
| `/sc2/dvl/dvl` | `nav_msgs/msg/Odometry` | rosbag2 replay | probe velocity bridge | HOLD velocity damping 検証用 |
| `/mavros/local_position/velocity_body` | `geometry_msgs/msg/TwistStamped` | probe velocity bridge | controller | HOLD velocity damping 用入力 |
| `/waterlinked_dvl_driver/velocity_report` | `marine_acoustic_msgs/msg/Dvl` | rosbag2 replay | controller intended canonical input | clean env では official package 不在のため replay ignore |

