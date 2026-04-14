# Launch Structure

## Main launch
- `sc2_visual_servo_sim/launch/simulator_phase1_mvp.launch.py`

## Arguments
- `backend`
  - `dynamic_velocity`
  - `direct_velocity`
- `detector_mode`
  - `synthetic`
  - `actual`
- `lighting`
  - `bright`
  - `dim`
  - `murky`
- `target_world_x_m`
- `target_world_y_m`
- `target_world_z_m`
- `initial_z_m`
- `initial_yaw_rad`

## Node composition
```text
sim_static_frames_node
  publishes: /tf_static

backend=direct_velocity:
  sim_direct_velocity_backend_node
    /sc2/visual_servo/cmd_vel_body -> /sim/odom

backend=dynamic_velocity:
  sim_dynamic_velocity_backend_node
    /sc2/visual_servo/cmd_vel_body -> /sim/odom
    /sc2/visual_servo/cmd_vel_body -> /sim/backend/wrench

odom_to_body_twist_node
  /sim/odom -> /sim/body_velocity

sim_downward_camera_node
  /sim/odom -> /sim/down/image_raw
  /sim/odom -> /sim/down/depth/image_raw
  /sim/odom -> /sim/down/camera_info

detector_mode=synthetic:
  synthetic_bbox_publisher_node
    -> /detections_output
    -> /best_detection

detector_mode=actual:
  rgb_target_detector_node
    /sim/down/image_raw -> /detections_output
    /sim/down/image_raw -> /best_detection

target_point_estimator_node
  /best_detection + depth + camera_info -> /sc2/visual_servo/target_point

visual_servo_controller_node
  /best_detection + /sc2/visual_servo/target_point + /sim/body_velocity -> /sc2/visual_servo/cmd_vel_body
```
