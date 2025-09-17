#! /bin/bash

/mnt/orin_ssd/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/run_dev.sh --isaac_ros_dev_dir /mnt/orin_ssd/workspaces/oakd_isaac_ros/ -b 1 -a '--env=ROS_DOMAIN_ID=8' -a "-v /mnt/orin_ssd/ros_bags_marseille_25/:/mnt/orin_ssd/ros_bags_marseille_25/"