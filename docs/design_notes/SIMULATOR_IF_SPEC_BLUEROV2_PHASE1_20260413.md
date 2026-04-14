# SIMULATOR IF SPEC BLUEROV2 PHASE1 2026-04-13

## Scope
- Phase 1 MVP の interface 契約を規定する。
- source of truth は本書と `docs/codex/CODEX_TASK_SIMULATOR_PHASE1_MVP_IMPLEMENT_JA.md`。
- artifact 配下の copy は参考用であり、差異があれば docs 配下を優先する。

## Top-Level Contract

### Upstream output
- publisher: `visual_servo_controller_node`
- topic: `/sc2/visual_servo/cmd_vel_body`
- type: `geometry_msgs/msg/TwistStamped`
- frame: `base_link`
- semantics:
  - `linear.x`: body surge, `+` forward
  - `linear.y`: body sway, `+` left
  - `linear.z`: body heave, `+` up
  - `angular.z`: yaw rate, `+` CCW viewed from above

### Upstream prohibition
- visual servoing 側は以下を publish しない。
  - raw thruster
  - RC override
  - MAVROS 固有 setpoint

## Sensor Contract
- RGB topic: `/sim/down/image_raw`
- depth topic: `/sim/down/depth/image_raw`
- camera info topic: `/sim/down/camera_info`
- detection topics:
  - `/detections_output`
  - `/best_detection`
- target point topic: `/sc2/visual_servo/target_point`
- odom topic: `/sim/odom`
- body velocity topic: `/sim/body_velocity`

## Node Ownership
- `sim_downward_camera_node`
  - downward RGB + depth + `CameraInfo` を生成する。
- `target_point_estimator_node`
  - `/best_detection` と depth/camera_info から target point を推定する。
- `visual_servo_controller_node`
  - detection / target point / body velocity を受け、`TwistStamped` を出す。
- `direct_velocity_backend`
  - debug 用に `cmd_vel_body` を直接積分して odom 化する。
- `dynamic_velocity_backend`
  - 評価用に `cmd_vel_body` を内部追従器へ通し、`odom` と近似 `wrench` を生成する。

## Backend Split

### direct_velocity_backend
- input
  - `/sc2/visual_servo/cmd_vel_body`
- output
  - `/sim/odom`
- purpose
  - frame/sign/debug

### dynamic_velocity_backend
- input
  - `/sc2/visual_servo/cmd_vel_body`
- output
  - `/sim/odom`
  - `/sim/backend/wrench`
- internal model
  - axis-wise first-order tracking
  - acceleration saturation
- future replacement point
  - `auv_control` / `thruster_manager` or MAVROS backend

## Phase Split

### Phase 1A
- detector source: synthetic
- objective: frame, sign, topic flow, backend wiring
- launch selector: `detector_mode:=synthetic`

### Phase 1B
- detector source: rendered RGB
- objective: actual detector path and bbox jitter readiness
- launch selector: `detector_mode:=actual`

## Frame Contract
- `odom`
  - world-fixed ENU-like local frame
- `base_link`
  - `+x forward`, `+y left`, `+z up`
- `down_camera_link`
  - colocated helper frame
- `down_camera_optical_frame`
  - down-looking optical frame
  - mapping from body:
    - `x_opt = -y_body`
    - `y_opt = -x_body`
    - `z_opt = -z_body`

## Image Error To Body Command Sign
- `u_error = u_measured - u_target`
- `v_error = v_measured - v_target`
- current staged PID mapping:
  - `cmd_vx_body = f(v_error)`
  - `cmd_vy_body = -f(u_error)`
  - `cmd_vz_body = -f(z_error)`
- practical sign expectations:
  - target appears right of center: command should move body right, so `cmd_vy_body < 0`
  - target appears below center: command should move body forward, so `cmd_vx_body > 0`
  - target farther below vehicle than desired: command should move vehicle down, so `cmd_vz_body < 0`

## Launch Interface
- `backend`
  - `dynamic_velocity` or `direct_velocity`
- `detector_mode`
  - `synthetic` or `actual`
- `lighting`
  - `bright`, `dim`, `murky`
- `target_world_x_m`, `target_world_y_m`, `target_world_z_m`
- `initial_z_m`, `initial_yaw_rad`

## Verification Expectations
- minimum static checks
  - Python compile
  - launch file parse
- minimum runtime checks
  - all key topics present
  - finite `target_point`
  - three initial offset cases with decreasing center error
  - left/right, front/back, up/down sign tests

## Current Status
- static implementation: complete
- runtime validation in this shell: blocked by missing ROS 2 runtime
