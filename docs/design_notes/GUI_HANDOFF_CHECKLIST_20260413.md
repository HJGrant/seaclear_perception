# GUI HANDOFF CHECKLIST 2026-04-13

## Purpose
GUI 確認はユーザー担当。Codex は GUI 前の headless 証跡までを用意済み。

## GUI Startup (example)
1. Foxglove:
   - `cd deployment_env/docker && docker compose up`
2. FastDDS Discovery Server:
   - `ssh itr@10.1.1.200 fastdds discovery -i 0 -p 11815 -s`
3. Container on Orin:
   - `ssh itr@10.1.1.200 oakd_isaac_ros/my_ws/scripts/launch_container.sh`
4. Build workspace if needed:
   - `colcon build --merge-install --packages-skip ros_deep_learning --cmake-args -DBUILD_TESTING=OFF`
5. Launch IBVS:
   - `ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=true`
6. Enable control:
   - `ros2 service call /enable_ibvs_control std_srvs/srv/SetBool "{data: true}"`

## Local-First Mainline Reminder
- Phase 1 simulator の mainline は `sc2_visual_servo_sim` 経路。
- IBVS/MAVROS 経路は参考用であり、local-first runtime の主対象ではない。

## GUI Checklist
- Foxglove connects to ROS 2 graph.
- Camera image appears.
- `/best_detection` shows bounding box.
- `/sc2/visual_servo/target_point` is finite.
- `/sc2/visual_servo/cmd_vel_body` or `/mavros/setpoint_velocity/cmd_vel` publishes.
- Behavior matches sign expectations (left/right/forward/back/up/down).

## Troubleshooting
- Discovery mismatch:
  - Confirm `config/fastdds_client_profile.xml` matches Discovery Server IP/port.
- No topics:
  - Ensure `ROS_DOMAIN_ID=8` in container.
  - Confirm `FASTRTPS_DEFAULT_PROFILES_FILE` is exported.
- IBVS not moving:
  - Call `/enable_ibvs_control` service.
  - Check `/sc2/visual_servo/status`.

## Data Capture
- Record rosbag for GUI session:
  - `ros2 bag record /sc2/visual_servo/cmd_vel_body /sc2/visual_servo/target_point /best_detection /oak/rgb/image_raw`
