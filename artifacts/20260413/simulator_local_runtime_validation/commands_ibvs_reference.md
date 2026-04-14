# Commands (IBVS / MAVROS Reference)

## [CONTAINER] IBVS launch
```bash
colcon build --merge-install --packages-skip ros_deep_learning --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=true
ros2 service call /enable_ibvs_control std_srvs/srv/SetBool "{data: true}"
```
Expected: `/mavros/setpoint_velocity/cmd_vel` publishes.

## Note
- This is a reference path only. Mainline local-first runtime uses `sc2_visual_servo_sim`.
