# Topic / Node / Frame Graph

## Topic flow
```text
/sim/odom
  -> sim_downward_camera_node
  -> odom_to_body_twist_node

/sim/down/image_raw
  -> rgb_target_detector_node

/detections_output
  from synthetic_bbox_publisher_node or rgb_target_detector_node

/best_detection
  -> target_point_estimator_node
  -> visual_servo_controller_node

/sim/down/depth/image_raw
  -> target_point_estimator_node

/sim/down/camera_info
  -> target_point_estimator_node
  -> visual_servo_controller_node

/sc2/visual_servo/target_point
  -> visual_servo_controller_node

/sim/body_velocity
  -> visual_servo_controller_node

/sc2/visual_servo/cmd_vel_body
  -> sim_direct_velocity_backend_node or sim_dynamic_velocity_backend_node
```

## Frame chain
```text
odom
└── base_link
    └── down_camera_link
        └── down_camera_optical_frame
```

## Optical mapping
```text
x_opt = -y_body
y_opt = -x_body
z_opt = -z_body
```

## Phase split
```text
Phase 1A: synthetic_bbox_publisher_node active
Phase 1B: rgb_target_detector_node active
```
