#!/bin/bash

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=8

colcon build --packages-select oakd_isaac_ros --merge-install

source install/setup.bash

ros2 launch oakd_isaac_ros visualization.launch.py use_foxglove_whitelist:='True' &

cd /workspaces/visualize_urdf/src/visualize_urdf/assets/sc2_urdf/meshes/ || exit 1

pwd

python3 -m http.server 8000 &

cd /workspaces/visualize_urdf/ || exit 1

pwd

source install/setup.bash
export ROS_DOMAIN_ID=8

ros2 launch visualize_urdf visualize_urdf.launch.py

#ros2 run visualize_urdf pub_system_stats