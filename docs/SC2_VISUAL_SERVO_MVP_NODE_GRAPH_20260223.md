# SeaClear2 Smart Grapple — Visual Servo MVP (ROS2) Node Graph & I/F Map

> Purpose: make it **one-glance obvious** how the MVP pipeline is wired end-to-end (sensor/detection → controller → command),
> and what to replace when moving from **stub sim** → **live perception** → **BlueROV**.

## 1) Launch modes (entry points)
Launch files:
- `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
- `src/sc2_visual_servo_bringup/launch/blue_rov_synth_vs.launch.py`

- **Mode A (default): stub target → controller → cmd**
  - `use_stub_target:=true` (default)
- **Mode B: YOLO+Depth input → controller → cmd**
  - `use_stub_target:=false`
- **Controller mode**
  - `controller_mode:=ibvs` (default) or `controller_mode:=mppi`
- **BlueROV synthetic actuation check (detector-free)**
  - `ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py`
  - defaults: `controller_mode:=bbox_approach`, `mavros_backend:=rc_override`

## 2) Node graph (topics and message types)

### 2.1 Default (stub) mode
```mermaid
flowchart LR
  A["target_obs_publisher<br/>(sc2_visual_servo_controller)"] -->|/sc2/visual_servo/target_obs<br/>TargetObservation| B["visual_servo_controller_node<br/>(sc2_visual_servo_controller)"]
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| C["fake_vehicle_node<br/>(sc2_visual_servo_controller)"]
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| D["csv_logger_node<br/>(sc2_visual_servo_controller)"]
  A -->|/sc2/visual_servo/target_obs<br/>TargetObservation| D
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| E["cmd_vel_to_mavlink_stub<br/>(sc2_visual_servo_controller)"]
  B -->|/sc2/visual_servo/status<br/>String| F["/sc2/visual_servo/status"]
  C -->|/sc2/visual_servo/fake_pose<br/>PoseStamped| G["/sc2/visual_servo/fake_pose"]
```

### 2.2 Perception-input mode (YOLO+Depth)
```mermaid
flowchart LR
  Y["/detections_output<br/>vision_msgs/Detection2DArray"] --> P["target_observer_node<br/>(sc2_visual_servo_perception)"]
  Z["/oak/stereo/image_raw<br/>sensor_msgs/Image"] --> P
  P -->|/sc2/visual_servo/target_obs<br/>TargetObservation| B["visual_servo_controller_node<br/>(sc2_visual_servo_controller)"]
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| E["cmd_vel_to_mavlink_stub"]
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| D["csv_logger_node"]
  P -->|/sc2/visual_servo/target_obs<br/>TargetObservation| D
  B -->|/sc2/visual_servo/status<br/>String| F["/sc2/visual_servo/status"]
```

### 2.3 BlueROV synthetic mode (detector-free)
```mermaid
flowchart LR
  S["synthetic_bbox_publisher_node<br/>(sc2_visual_servo_synth)"] -->|/detections_output<br/>Detection2DArray| P["target_observer_node<br/>(optional when direct=false)"]
  S -->|/sc2/visual_servo/target_obs<br/>TargetObservation (direct mode)| B["visual_servo_controller_node<br/>(bbox_approach)"]
  P -->|/sc2/visual_servo/target_obs<br/>TargetObservation| B
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| M["cmd_vel_to_mavros_bridge<br/>(dry_run / rc_override / setpoint_velocity)"]
  B -->|/sc2/visual_servo/status<br/>String| F["/sc2/visual_servo/status"]
  B -->|/sc2/visual_servo/cmd_vel_body<br/>TwistStamped| D["csv_logger_node"]
```

## 3) Data contracts (what flows between nodes)

### 3.1 `/sc2/visual_servo/target_obs` — `TargetObservation`
Message: `sc2_visual_servo_interfaces/msg/TargetObservation.msg`

| Field | Meaning (MVP) |
|---|---|
| `header.stamp` | detection timestamp |
| `header.frame_id` | camera optical frame id (e.g., `oak_rgb_camera_optical_frame`) |
| `track_id` | tracker id if available, else -1 |
| `u,v` | bbox center pixel |
| `w,h` | bbox size in pixels |
| `depth_m` | robust depth estimate inside bbox ROI (median). NaN allowed. |
| `confidence` | detection confidence |
| `keypoints_uv` | reserved (future) |
| `source` | provenance string (e.g., `sim_scripted`, `yolo_detection2d`) |

### 3.2 `/sc2/visual_servo/cmd_vel_body` — `geometry_msgs/TwistStamped`
- `header.frame_id` = body frame (default parameter `base_link`)
- `twist.linear.{x,y,z}` = body-frame linear velocity command
- `twist.angular.z` = yaw rate

### 3.3 `/sc2/visual_servo/status` — `std_msgs/String`
- `NO_TARGET_TIMEOUT` when target is missing beyond timeout
- `TRACKING_IBVS`, `TRACKING_BBOX_APPROACH`, or `TRACKING_MPPI`

## 4) What to replace when moving toward hardware

### 4.1 Replace stub target with real perception
**Current**: `target_obs_publisher` produces scripted TargetObservation.

**Replace with**:
- `target_observer_node` (already implemented)
- Provide correct topic names:
  - detections: `/detections_output` (or remap)
  - depth: `/oak/stereo/image_raw` (or remap)

### 4.2 BlueROV bridge status
**Available now**:
- `cmd_vel_to_mavros_bridge` supports:
  - `backend:=dry_run` (log only)
  - `backend:=rc_override` (publish to `/mavros/rc/override`)
  - `backend:=setpoint_velocity` (publish to `/mavros/setpoint_velocity/cmd_vel_unstamped`)
- deadman + timeout + command saturation are implemented.

**Still pending**:
- closed-loop validation with vehicle state feedback/pose integration.

### 4.3 Replace placeholder IBVS with MPPI (same I/F)
**Switch** using launch arg:
- `controller_mode:=mppi`

Note: MPPI requires `torch` at runtime; if unavailable, it falls back to `ibvs`.

## 5) Verification protocol (what you must run to claim “works”)

### 5.1 Build + run (MVP stub)
1) `source /opt/ros/humble/setup.bash`
2) `./scripts/uv_bootstrap.sh`
3) `./scripts/colcon_build_venv.sh --packages-up-to sc2_visual_servo_bringup sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_interfaces sc2_visual_servo_synth`
4) `source install/setup.bash`
5) `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py`
6) Confirm topic publishing:
- `ros2 topic echo /sc2/visual_servo/cmd_vel_body --once`
- check `/tmp/sc2_visual_servo_mvp.csv` is created and growing

### 5.2 Perception-input run (YOLO+Depth)
1) run your YOLO/depth pipeline
2) `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false`
3) Confirm:
- `/detections_output` exists and messages arrive
- depth topic is truly a depth image (not rectified left)

### 5.3 Tests (required to claim PR is clean)
- Run:
  - `./scripts/colcon_test_venv.sh --packages-select sc2_visual_servo_perception sc2_visual_servo_controller sc2_visual_servo_bringup`

## 6) Known MVP caveats (intentional)
- No SLAM dependency (by design).
- Depth is bbox-ROI median (no mask yet).
- BlueROV path is currently open-loop command validation via MAVROS bridge; closed-loop vehicle-state validation is not integrated yet.
