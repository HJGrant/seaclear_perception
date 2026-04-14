# Commands (Phase 1 Simulator Local-First)

## [HOST-WSL] Gazebo server (headless)
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim gz_server.launch.py world:=/path/to/world.sdf
```
Expected: `gzserver` stays alive, no GUI required.

## [HOST-WSL] Gazebo GUI (optional)
```bash
gz sim -g /path/to/world.sdf
```
Expected: world loads and camera view is available.

## [CONTAINER] Build and launch Phase 1 simulator path
```bash
colcon build --merge-install --packages-select sc2_visual_servo_sim sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_synth --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch sc2_visual_servo_sim simulator_phase1_dynamic.launch.py
```
Expected: `/sc2/visual_servo/cmd_vel_body` and `/sim/odom` publish.

## [CONTAINER] runtime checks
```bash
ros2 node list
ros2 topic list
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
ros2 topic echo /sc2/visual_servo/target_point --once
ros2 topic hz /best_detection
ros2 topic hz /sim/down/image_raw
ros2 topic echo /sim/odom --once
ros2 run tf2_ros tf2_echo base_link down_camera_optical_frame
```
Expected: topics publish at non-zero rate, tf is available.
