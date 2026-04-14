# SIMULATOR_INTERFACE_PLAN_BLUEROV2_HUMBLE_20260412

## Boundary

controller / perception と simulator の境界は `TwistStamped` body velocity command に固定する。

- upstream command contract: `/sc2/visual_servo/cmd_vel_body`
- message: `geometry_msgs/TwistStamped`
- frame: `base_link`

## Simulator Inputs

- `/sim/down/image_raw`
- `/sim/down/camera_info`
- `/sim/odom`
- optional `/sim/depth/image_raw`

## Adapter Responsibilities

### phase-1

- `cmd_vel_body_to_sim_thrusters`
- `sim_odom_to_body_velocity`
- static TF publisher for `base_link -> down_camera_optical_frame`

### future

- reuse `cmd_vel_to_mavros_bridge` style backend switching
- add `backend=rc_override`
- add `backend=setpoint_velocity`

## Non-goals

- 初手での ArduSub full-stack 統合
- controller 本体の大幅改造
- `orca4` 互換の manager / Nav2 / ORB_SLAM2 一式移植
