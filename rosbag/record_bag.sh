#! /bin/bash

ros2 bag record \
  /best_detection \
  /detections_output \
  /oak/rgb/camera_info \
  /oak/rgb/image_raw \
  /mask_overlay \
  /sc2/imu/imu \
  /sc2/imu/data_ned \
  /sc2/dvl/dvl \
  /sc2/dvl/odometry \
  /sc2/odometry_ekf \
  /sc2/state_estimation_node/estimated_velocity \
  /waterlinked_dvl_driver/velocity_report \
  /sc2/altitude \
  /tf \
  /tf_static \
  /sc2/visual_servo/status \
  /sc2/visual_servo/target_locked \
  /sc2/visual_servo/centroid_error \
  /sc2/visual_servo/target_point \
  /sc2/visual_servo/velocity_camera_image_plane_frame \
  /sc2/visual_servo/velocity_sc2_base_link \
  /sc2/velocity_controller_node/outer/visual_servoing_signals \
  /sc2/visual_servo/jacobian_predicted_velocity \
  /sc2/visual_servo/calibration_phase \
  /sc2/visual_servo/vs_control_finish \
  /sc2/visual_servo/centroid_kf_covariance \
  /sc2/visual_servo/filtered_centroid_error \
  /sc2/visual_servo/centroid_kf_marker \
  #/sc2/wrench_rov \ 
	#/sc2/wrench_rov_console \
  #/wrench_rov