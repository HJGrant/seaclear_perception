# Topic / Node / TF Evidence

## Status
- Not captured in this environment.

## Plan
Run inside container and attach outputs:
```bash
ros2 node list
ros2 topic list
ros2 topic info -v /sc2/visual_servo/cmd_vel_body
ros2 topic info -v /sc2/visual_servo/target_point
ros2 run tf2_ros tf2_echo base_link down_camera_optical_frame
```
