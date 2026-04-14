# SIMULATOR RUNTIME VALIDATION PLAN 2026-04-13

## Purpose
Headless で simulator + visual servo を動かし、GUI 前に必要な runtime 証跡を残す。

## Preconditions
- Docker Desktop / Orin 上で container 実行可能
- ROS 2 Humble + Gazebo Fortress が container に入っている
- `ros_gz_sim` と `ros_gz_bridge` がインストール済み
- FastDDS Discovery Server が起動済み

## Container Startup
- Orin: `my_ws/scripts/launch_container.sh`
- x86_64: `my_ws/scripts/launch_container_x86_64.sh`

## Local-First Split (Mainline)
- Host: Gazebo server (WSL or native Ubuntu)
- Container: ROS 2 nodes (visual servo, perception, backend)
- DDS: FastDDS discovery profile is shared via `FASTRTPS_DEFAULT_PROFILES_FILE`

## Headless Launch Options
1. `ros2 launch ros_gz_sim gz_server.launch.py ...`
2. or `gz sim -s <world.sdf>` server-only equivalent

## Visual Servo Launch Options
- Simulator-native Phase 1 (mainline):
  - `ros2 launch sc2_visual_servo_sim simulator_phase1_dynamic.launch.py`
- IBVS reference (secondary):
  - `ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=false`

## Required Runtime Evidence
- `ros2 node list`
- `ros2 topic list`
- `ros2 topic echo /sc2/visual_servo/cmd_vel_body --once`
- `ros2 topic hz /sim/down/image_raw` (or sensor image topic)
- `ros2 topic hz /best_detection`
- `ros2 topic hz /sc2/visual_servo/target_point`
- `ros2 topic echo /sim/odom --once`
- rosbag (short): `ros2 bag record ...`

## FastDDS Notes
- `config/fastdds_client_profile.xml` sets domain 8 and Discovery Server `10.1.1.200:11815`.
- Orin discovery server must match or XML needs update.

## Known Risks
- `object_detection_ibvs.launch.py` outputs to `/mavros/setpoint_velocity/cmd_vel`.
- Phase 1 simulator expects `/sc2/visual_servo/cmd_vel_body`.

## Exit Criteria
- Headless launch stays alive for > 10s
- Topics publish at expected rates
- `/sc2/visual_servo/cmd_vel_body` or `/mavros/setpoint_velocity/cmd_vel` seen
- Odom/velocity feedback changes when control enabled
