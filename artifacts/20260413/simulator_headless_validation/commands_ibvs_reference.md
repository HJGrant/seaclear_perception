# Commands (IBVS / MAVROS Reference Path)

## Container-side (expected)
```bash
my_ws/scripts/launch_container.sh
colcon build --merge-install --packages-skip ros_deep_learning --cmake-args -DBUILD_TESTING=OFF
ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_ibvs:=true launch_foxglove:=true
ros2 service call /enable_ibvs_control std_srvs/srv/SetBool "{data: true}"
```

## Notes
- This path publishes to `/mavros/setpoint_velocity/cmd_vel`.
- It is not the same as the Phase 1 simulator `TwistStamped` path.
