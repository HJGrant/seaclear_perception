# ROS Topic / Bag Check Status

| Check | Expected command | Status | Note |
| --- | --- | --- | --- |
| topic list | `ros2 topic list` | not run | ROS runtime unavailable |
| odom echo | `ros2 topic echo /sim/odom --once` | not run | ROS runtime unavailable |
| target point echo | `ros2 topic echo /sc2/visual_servo/target_point --once` | not run | ROS runtime unavailable |
| cmd echo | `ros2 topic echo /sc2/visual_servo/cmd_vel_body --once` | not run | ROS runtime unavailable |
| image hz | `ros2 topic hz /sim/down/image_raw` | not run | ROS runtime unavailable |
| detection hz | `ros2 topic hz /best_detection` | not run | ROS runtime unavailable |
| target point hz | `ros2 topic hz /sc2/visual_servo/target_point` | not run | ROS runtime unavailable |
| rosbag record | `ros2 bag record ...` | not run | ROS runtime unavailable |

## Required follow-up once ROS environment is present
1. Run Phase 1A with `dynamic_velocity_backend`.
2. Confirm all required topics publish finite values.
3. Record rosbag for at least one bright and one dim condition.
4. Append measured `hz`, finite-value checks, and convergence notes to this table.
