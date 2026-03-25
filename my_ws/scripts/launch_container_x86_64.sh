#! /bin/bash

# Use ${HOME} instead of ~ inside quotes to ensure the full path is captured
ISAAC_ROS_WS=${ISAAC_ROS_WS:-"${HOME}/workspaces/oakd_isaac_ros"}

# This evaluates the internal container path to exactly:
# /workspaces/oakd_isaac_ros/config/fastdds_client_profile.xml
CONTAINER_CONF="/workspaces/$(basename ${ISAAC_ROS_WS})/config/fastdds_client_profile.xml"

${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh --skip_lfs_check\
    --isaac_ros_dev_dir "${ISAAC_ROS_WS}" \
    -a '--env=ROS_DOMAIN_ID=8' \
    -a '--env=ROS_LOCALHOST_ONLY=0' \
    -a "--env=FASTRTPS_DEFAULT_PROFILES_FILE=${CONTAINER_CONF}"