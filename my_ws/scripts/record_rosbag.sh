#! /bin/bash

# A simple script to record the correct topics in a ros bag for the SC2 tests in Marseille

source /opt/ros/humble/setup.bash
source /workspaces/oakd_isaac_ros/install/setup.bash

export ROS_DOMAIN_ID=8

# we have set the ros bag in compression mode, and set it to split the bag files every 100Mb
# we also publish stats on lost packages from topics once every 2 seconds
#first move into the right directory on the orins ssd
# add the following for compression --compression-mode file --compression-format zstd
pushd /mnt/orin_ssd/ros_bags_marseille_25/ && \
ros2 bag record  \
        -b 4000000000 \
        /tf \
        /tf_static \
        /transform \
        /rosout \
        /robot_description \
        /diagnostics \
        /diagnostics_agg \
        /oak/rgb/image_raw/compressed \
        /mask_overlay \
        /mask_overlay_dual \
        /nvblox_node/color_layer_marker \
        /nvblox_node/color_layer \
        /nvblox_node/dynamic_depth_frame_overlay \
        /nvblox_node/dynamic_occupancy_layer \
        /nvblox_node/dynamic_points \
        /nvblox_node/dynamic_occupancy_layer_marker \
        /oak/stereo/image_raw/compressed \
        /sc2/dvl/dvl \
        /seabed_grid \
        /sc2/state_estimation_node/detected_objects/target \
        /sc2/altitude \
        /sc2/aruco_pose/pose \
        /sc2/dvl/odometry \
        /sc2/imu/data_ned \
        /sc2/odometry_ekf \
        /sc2/position_controller_node/console_desireds  \
        /sc2/position_controller_node/control_errors \
        /sc2/position_controller_node/control_signals \
        /sc2/position_controller_node/outer/console_signals  \
        /sc2/position_controller_node/outer/control_signals \
        /sc2/position_controller_node/outer/gamepad_signals \
        /sc2/pwm_out \
        /sc2/state_estimation_node/depth \
        /sc2/state_estimation_node/detected_objects/target \
        /sc2/state_estimation_node/dvl \
        /sc2/state_estimation_node/estimated_position \
        /sc2/state_estimation_node/estimated_position_exploring \
        /sc2/state_estimation_node/estimated_position_visual \
        /sc2/state_estimation_node/estimated_state \
        /sc2/state_estimation_node/estimated_velocity \
        /sc2/thruster_manager/input \
        /sc2/velocity_controller_node/console_desireds \
        /sc2/velocity_controller_node/control_errors \
        /sc2/velocity_controller_node/control_signals \
        /sc2/velocity_controller_node/estimated_values_exploring \
        /sc2/velocity_controller_node/estimated_values_visual \
        /sc2/velocity_controller_node/outer/console_signals \
        /sc2/velocity_controller_node/outer/gamepad_signals \
        /sc2/wrench_order_enable \
        /sc2/wrench_rov \
        /sc2/wrench_rov_console \
        /sc2/wrench_tortuga_interface \
        /senso_input_hr_data \
        /senso_joint/input_data \
        /senso_joint/raw_input_data \
        /senso_on_off \
        /socket_can_receiver/transition_event \
        /socket_can_sender/transition_event \
        /system_stats \
        /abs_torque_nm \
        /abs_torque_nm_per_joint \
        /decoupled_position_rad \
        /grapple_percentage \
        /grapple_percentage_SpecifiedLevel \
        /joint_states \
        /position_rad \
        /target_position \
        /velocity_rpm \
        



                


