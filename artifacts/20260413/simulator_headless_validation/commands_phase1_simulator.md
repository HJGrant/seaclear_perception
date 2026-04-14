# Commands (Phase 1 Simulator Path)

## Host-side audit commands executed
```bash
docker --version
docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}'
command -v ros2
command -v colcon
command -v gz
```

## Container-side (expected)
```bash
my_ws/scripts/launch_container.sh
colcon build --merge-install --packages-select sc2_visual_servo_sim sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_synth --cmake-args -DBUILD_TESTING=OFF
ros2 launch sc2_visual_servo_sim simulator_phase1_dynamic.launch.py
ros2 launch sc2_visual_servo_sim simulator_phase1_direct.launch.py
ros2 launch sc2_visual_servo_sim simulator_phase1_actual_detector.launch.py
```

## Headless runtime checks
```bash
ros2 node list
ros2 topic list
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
ros2 topic echo /sc2/visual_servo/target_point --once
ros2 topic hz /sim/down/image_raw
ros2 topic hz /best_detection
ros2 topic hz /sc2/visual_servo/target_point
ros2 topic echo /sim/odom --once
ros2 bag record /sc2/visual_servo/cmd_vel_body /sc2/visual_servo/target_point /best_detection /sim/odom /sim/down/image_raw /sim/down/depth/image_raw
```
