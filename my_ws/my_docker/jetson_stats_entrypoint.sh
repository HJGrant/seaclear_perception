#!/bin/bash

export ROS_DOMAIN_ID=8

source install/setup.bash

ros2 launch isaac_ros_jetson_stats jtop.launch.py