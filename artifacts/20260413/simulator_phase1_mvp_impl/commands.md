# Commands

## Static checks executed
```bash
python3 -m compileall src/sc2_visual_servo_sim src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py src/sc2_visual_servo_synth/sc2_visual_servo_synth/synthetic_bbox_publisher_node.py
python3 -m compileall src/sc2_visual_servo_sim/launch src/sc2_visual_servo_bringup/launch
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
```

## Runtime commands to execute in a ROS 2 Humble environment
```bash
colcon build --packages-select sc2_visual_servo_sim sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_synth
source install/setup.bash
ros2 launch sc2_visual_servo_sim simulator_phase1_dynamic.launch.py
ros2 launch sc2_visual_servo_sim simulator_phase1_actual_detector.launch.py
ros2 launch sc2_visual_servo_sim simulator_phase1_direct.launch.py
```

## Topic probes to run later
```bash
ros2 topic list
ros2 topic echo /sim/odom --once
ros2 topic echo /sc2/visual_servo/target_point --once
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
ros2 topic hz /sim/down/image_raw
ros2 topic hz /best_detection
ros2 topic hz /sc2/visual_servo/target_point
ros2 bag record /sim/odom /sim/down/image_raw /sim/down/depth/image_raw /best_detection /sc2/visual_servo/target_point /sc2/visual_servo/cmd_vel_body
```
