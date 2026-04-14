# SIMULATOR_IF_SPEC_BLUEROV2_PHASE1_20260412

## Stable Interface Boundary

上位 visual-servoing stack の出力は常に `geometry_msgs/msg/TwistStamped` とする。

- topic: `/sc2/visual_servo/cmd_vel_body`
- frame: `base_link`
- semantics: body velocity setpoint

## Backend Split

### direct_velocity_backend

- approximate vehicle motion update
- debugging only

### dynamic_velocity_backend

- consume `cmd_vel`, `odom`, `tf`
- convert to thrust via control layer
- preferred implementation path: `auv_control` + `thruster_manager`

## Simulator Sensor Contract

- `/sim/down/image_raw`
- `/sim/down/camera_info`
- `/sim/down/depth/image_raw`
- `/sim/odom`
- `/sim/body_velocity`

## Frame Contract

- `odom`: ENU
- `base_link`: FLU
- `down_camera_link`: rigidly attached under vehicle
- `down_camera_optical_frame`: optical frame, right/down/forward

## Testing

- left/right sign test
- forward/back sign test
- altitude sign test
- synthetic bbox loop test
- actual detector loop test
