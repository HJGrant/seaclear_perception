# SeaClear2.0 — Smart Grapple Visual Servoing
## Status & Gap Analysis (as of 2026-02-22, prepared for 2026-02-23 sync)

**Scope of this document**: consolidate what is implemented vs. what is described/planned, across:
- **D5.2 “Report on Smart Grapple Software Framework”** (`20251225_D5_2_Control_Overhaul_SC2.pdf`)
- **Winter school slides** (`winter school_v3.pdf`)
- **ROS2 perception/VSLAM workspace** (this repository: `oakd_isaac_ros`)
- **Legacy ROS1 sim visual-servo package** (external snapshot: `sc_sim_visualservo.zip`)

Primary objective for the next **~2 months**: make **visual servoing easily runnable for quick experiments**, and be able to run a **pilot deployment on BlueROV** (controller + perception loop, even if other subsystems remain stubbed).

---

## 1. “What the project expects” (from D5.2)

D5.2 describes the Smart Grapple control stack as a state machine with phases **deploying → planning → tracking → visual servoing → grasping → recovering**.

### 1.1 Visual servoing role (expected behavior)
- Visual servoing is the **camera-driven, precise positioning phase** after tracking reaches the last planned waypoint.
- Inputs:
  - object detection output (bounding box)
  - (optionally) grasp point estimation output (desired grasp pose)
  - altitude feedback (DVL) for vertical positioning
- Control objective (minimum): **center the detection** in the camera image and regulate **altitude / approach distance** until reaching a pre-grasp pose, then transition to grasping.

### 1.2 Perception pipeline role (expected behavior)
- Stereo RGB-D camera → YOLOv8 detection → depth masking → voxel mapping (static + dynamic) → downstream usage for grasp planning & servoing.

### 1.3 Digital twin
- D5.2 indicates a MARUS-based SIL pipeline exists for navigation/perception/planning/tracking integration, used between sea trials.

---

## 2. What is in the repos today

This section lists concrete code assets and their maturity.

### 2.1 `oakd_isaac_ros` (ROS2 perception + VSLAM workspace)

**Purpose of this workspace (as captured in the included docs):**
- Run **OAK-D PoE** with ROS2
- Run **YOLOv8 detection** and **nvBlox** (static + dynamic layers)
- Run **Isaac ROS Visual SLAM (cuVSLAM)** on Jetson Orin
- Provide a diagnosis/triage workflow for OAK-D reliability problems

#### 2.1.1 What is implemented and usable
1) **YOLOv8 inference pipeline (Isaac ROS)**
- Launch: `src/oakd_isaac_ros/launch/tools/detection.launch.py`
- Image preprocessing: `isaac_ros_dnn_image_encoder` pipeline
- Inference: `isaac_ros_tensor_rt`
- Decode: `isaac_ros_yolov8`

2) **Mask generation + timestamp alignment for depth**
- Component: `oakd_isaac_ros/src/publish_mask.cpp`
- Builds a `mono8` mask image from `vision_msgs/Detection2DArray` (`/detections_output`) and republishes depth image + camera_info with RGB timestamps.
- This is consistent with the D5.2 “depth masking” idea, though it currently publishes **mask + depth re-timestamping**, not an explicitly masked depth image.

3) **nvBlox pipeline integration (dynamic mapping mode)**
- Launch: `src/oakd_isaac_ros/launch/oakd_nvblox.launch.py`
- Modes include “people_detection” which (per comments) is used to run SeaClear YOLO inference and do selective/dynamic mapping.

4) **OAK-D + cuVSLAM integration scaffolding**
- Launch: `src/oakd_isaac_ros/launch/oakd_visual_slam.launch.py`
- Includes static TF publishers and image_transport republish nodes.

5) **Systematic debug + test matrix**
- `docs/status/PROJECT_STATUS_OAKD_VSLAM.md`
- `docs/status/TEST_MATRIX.md`
- `docs/worklogs/WORKLOG_20260223.md`
- `docs/OAKD_VSLAM_DEBUG_NOTES.md` with a parameter-integrity audit and staged YAMLs.

#### 2.1.2 What is *not* implemented here yet (relative to visual servoing)
- **No ROS2 visual-servo controller node** (no node that outputs `cmd_vel` / `Twist` or equivalent from detections/depth).
- **No grasp-point estimation / “gripping points optimization” service** (D5.2 Sec. 5.4). The workspace currently ends at detection + mapping.
- **cuVSLAM is not yet “production stable”** in this repo snapshot; there are documented issues:
  - input sync/transport/TF mismatches causing odometry not moving or jumping
  - intermittent DepthAI disconnects / non-recovering stream stop


### 2.2 `sc_sim_visualservo.zip` (legacy ROS1 simulation package)

**Purpose:** a ROS1 (catkin) package to run and iterate on a visual-servo controller in simulation. The README documents an end-to-end sim flow:
- bring up simulated collection ROV
- run ArUco-based perception
- (optional) run a complementary-filter pose estimator
- run either MPC (`VS_mpc`) or Python MPPI controller (`mppi_vs_node.py`)

#### 2.2.1 What is implemented and usable
1) **C++ MPC visual servo controller**
- `src/VS_mpc.cpp`
- Implements a constrained MPC loop driven by TF transforms (camera↔marker, base↔camera), with explicit FOV constraints.

2) **Python+Torch MPPI controller with FOV risk term**
- `scripts/mppi_vs_node.py` (ROS node) + `src/sc_sim_visualservo_mppi/core.py`
- Implements MPPI in torch (CUDA optional) on a **relative pose state p=[px,py,pz]** and outputs `geometry_msgs/Twist` on `/colrov/cmd_vel`.
- Includes a “risk_mode” (none/chance/cvar) and a FOV margin penalty; this is closely aligned with the winter school idea of FOV-aware control.

3) **Launch files & config**
- `launch/mppi_vs.launch`
- `config/mppi_vs.yaml`

#### 2.2.2 What is missing / mismatched vs. SeaClear2.0 target
- This package is **ROS1**, while the OAK-D + Isaac pipeline is **ROS2**.
- Perception input is primarily **ArUco marker TF**, not YOLO bounding boxes + depth / point cloud.
- The “IBVS (2½-D) minimal-feature controller” described in the winter school slides is **not implemented** as-is here; the controllers are essentially **pose-based camera-centric** (PBVS-like), relying on a 3D relative position.
- The controller output is `cmd_vel` in the sim robot’s convention; for BlueROV and Smart Grapple the command interface and frame conventions must be re-validated.


---

## 3. Mapping: documentation ↔ code

### 3.1 D5.2 “Visual servoing” (state machine) ↔ code
- D5.2 describes visual servoing as a distinct state, triggered after tracking reaches final waypoint.
- In the provided code snapshots:
  - **State machine implementation is not present** (likely exists in a different control repo).
  - Visual servoing exists only as **standalone controllers in ROS1 simulation** (`sc_sim_visualservo`).

### 3.2 D5.2 perception pipeline ↔ `oakd_isaac_ros`
- D5.2 pipeline components present in ROS2 snapshot:
  - YOLO-based detection (Isaac ROS nodes)
  - mask publication node (`publish_mask.cpp`)
  - nvBlox integration
- Missing vs D5.2:
  - TSDF “static+dynamic” separation is implemented by nvBlox, but **control over static-only vs dynamic-only** is still a TODO in `oakd_isaac_ros` README.
  - grasp-point estimation service (Sec. 5.4) not present.

### 3.3 Winter school plan ↔ existing controller code
- Slides propose:
  - **IBVS-first** (robust without global pose)
  - switch to **PBVS** when SLAM map converges / conditions satisfied
  - include FOV constraints
- In code:
  - FOV constraint concept exists (C++ MPC + MPPI risk term)
  - switching logic IBVS↔PBVS does **not** exist yet
  - IBVS with image features (u,v,ln(Z/Z*)) does **not** exist yet

---

## 4. Current readiness for “easy visual servo experiment”

### 4.1 What can be demoed quickly (today)
- **Simulation-only** demo using `sc_sim_visualservo`:
  - ArUco TF → MPPI/MPC → `/cmd_vel`.

### 4.2 What is *blocked* for ROS2 / hardware-facing experiments
1) **No ROS2 visual servo node** consuming:
   - `/detections_output` (YOLO decoder)
   - depth image aligned with RGB (from OAK-D)
   - producing velocity commands in a defined frame

2) **Relative pose extraction (minimum)** is missing:
   - need to turn (bbox center + depth) into an estimate of target relative coordinates in camera frame
   - or compute a minimal IBVS feature vector and use an interaction-matrix-based control

3) **Localization choice is undecided for MVP**
- PBVS requires stable vSLAM or equivalent; currently cuVSLAM integration is still under debug.
- IBVS can work without vSLAM, which is attractive for a 2-month MVP.

4) **BlueROV integration layer** is missing
- Need a ROS2 interface to send velocity/thruster commands to BlueROV (e.g., MAVROS2 or direct Blue Robotics stack).
- Need a consistent frame convention (camera frame vs body frame vs ENU/NED conversions).

---

## 5. “Minimum viable” path for the next ~2 months

This is a pragmatic MVP proposal aimed at **fast experimental iteration**, while keeping the winter school long-term vision intact.

### 5.1 MVP definition (acceptance criteria)
A run is considered “easy to experiment” if:
- With a single command (docker/launch), the system starts:
  - OAK-D camera
  - YOLO detection
  - a visual servo node
- And the servo node publishes a bounded velocity command that:
  - drives the target toward the image center
  - regulates approach distance to a desired depth/altitude
  - handles detection loss safely (hold/stop)

### 5.2 Recommended MVP architecture
**IBVS-first** (no SLAM dependency), camera-centric:
1) subscribe to:
   - detections (bbox)
   - depth image
   - camera intrinsics
2) compute:
   - (u,v) = bbox center
   - Z = robust depth estimate (median depth inside mask/bbox)
   - s = [u,v,ln(Z/Z*)]  (winter school minimal formulation)
3) output:
   - v_cam (camera-frame translational velocity)
   - optional yaw rate (align object long axis later; not required for MVP)
4) transform to robot command frame and publish

**PBVS + switching** can be Phase-2 once cuVSLAM is reliable.

### 5.3 Work packages (backlog) with clear “done” criteria

**WP0 — Interfaces & logging (1–3 days)**
- Define ROS2 topics for: detection, depth, command, debug viz
- Add a rosbag record profile for reproducible experiments

**WP1 — ROS2 Visual Servo Node (IBVS/PID) (1–2 weeks)**
- Implement `visual_servo_node` (rclpy or rclcpp)
- Depth extraction from mask/bbox
- Safety: command saturation, detection timeout

**WP2 — Integrate with `oakd_isaac_ros` launch (≤1 week)**
- Add launch option: run VS node alongside detection
- Ensure TF tree and camera_info availability

**WP3 — Port MPPI controller to ROS2 (optional, 1–2 weeks)**
- Reuse `sc_sim_visualservo_mppi/core.py` (torch) with rclpy node
- Validate on recorded bags (no sim dependency)

**WP4 — BlueROV pilot integration (1–2 weeks)**
- Choose comms stack: MAVROS2 or direct
- Create a velocity-command bridge (ROS2 Twist → BlueROV control)
- Bench test: command injection + kill-switch

**WP5 — (Parallel) Stabilize cuVSLAM on OAK-D (ongoing)**
- Keep working through the existing test matrix.
- Only required for PBVS / switching phase.

---

## 6. Key technical risks (and how to de-risk early)

1) **OAK-D stream instability / driver crashes**
- Mitigation: keep a known-good baseline config (400p stereo-only) and run VS on recorded bags until stability improves.

2) **Depth quality & masking**
- Mitigation: depth estimate must be robust (median + validity checks) and degrade gracefully.

3) **Frame convention mismatch (camera vs body vs ENU/NED)**
- Mitigation: document and unit-test axis mappings explicitly; avoid “silent sign errors”.

4) **BlueROV safety**
- Mitigation: enforce command limits, timeouts, and have an external hardware kill switch.

---

## 7. Immediate next actions (suggested ordering)

1) Implement ROS2 IBVS/PID node (WP1) using recorded bags (no SLAM required)
2) Integrate it into `oakd_isaac_ros` launches (WP2)
3) Only then push to live OAK-D
4) In parallel, keep cuVSLAM stabilization work (WP5)
5) Start BlueROV comms integration after WP1+WP2 show stable behavior

---

## Appendix A — Where things live (paths)

### A.1 ROS2 workspace (`oakd_isaac_ros`)
- Workspace root: `oakd_isaac_ros/`
- Main package: `oakd_isaac_ros/src/oakd_isaac_ros/`
  - Launch: `launch/oakd_nvblox.launch.py`, `launch/oakd_visual_slam.launch.py`, `launch/tools/detection.launch.py`
  - Nodes: `src/publish_mask.cpp`
  - Config: `config/oak_d/*.yaml`, `config/nvblox*.yaml`
  - Diagnostics: `scripts/diag_oakd_run_and_monitor.sh`
  - Status docs: `docs/status/*`, `docs/worklogs/*`

### A.2 ROS1 sim package (`sc_sim_visualservo.zip`)
- Package root: `sc_sim_visualservo/`
  - MPPI node: `scripts/mppi_vs_node.py`
  - MPPI core: `src/sc_sim_visualservo_mppi/core.py`
  - MPC: `src/VS_mpc.cpp`
  - Launch: `launch/mppi_vs.launch`
  - Config: `config/mppi_vs.yaml`

---

## 8. Implementation Snapshot (2026-02-23)

### 8.1 Implemented
- `sc2_visual_servo_interfaces` (custom message contract)
  - `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg`
- `sc2_visual_servo_perception` (YOLO `Detection2DArray` -> `TargetObservation`)
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/conversion.py`
- `sc2_visual_servo_controller` (Phase-1 IBVS + Phase-2 MPPI-ready mode)
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py`
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/controller_math.py`
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/mppi_core.py`
- Simulation harness and stubs
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/target_obs_publisher.py`
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/fake_vehicle_node.py`
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/csv_logger_node.py`
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavlink_stub.py`
- Bringup launch (one-command MVP)
  - `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
- Quickstart
  - `README_quickstart.md`

### 8.2 Verified (code + test assets)
- Unit tests
  - perception conversion: `src/sc2_visual_servo_perception/test/test_conversion.py`
  - controller sign correctness: `src/sc2_visual_servo_controller/test/test_controller_sign.py`
- Integration smoke test (launch + topic alive)
  - `src/sc2_visual_servo_bringup/test/test_mvp_sim_launch.py`

### 8.3 Pending
- Direct adapter to SC1 simulator (`sc_sim_visualservo`) as an alternative to ROS2-only harness.
- Hardware BlueROV command transport (stub exists only).
- Full MPPI tuning against real detections/depth noise profile.

### 8.4 uv Dependency Management (2026-02-23)

Status: **Implemented / Partially verified in current host**

Implemented files:
- `pyproject.toml`
- `uv.lock`
- `scripts/uv_bootstrap.sh`
- `scripts/colcon_build_venv.sh`
- `scripts/colcon_test_venv.sh`
- `.gitignore` updates for `.venv/`, `**/__pycache__/`, `**/*.pyc`, `.pytest_cache/`, `.mypy_cache/`

Canonical command flow:
1. `source /opt/ros/humble/setup.bash`
2. `./scripts/uv_bootstrap.sh`
3. `./scripts/colcon_build_venv.sh --packages-up-to sc2_visual_servo_bringup sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_interfaces`
4. `source install/setup.bash`
5. `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py`
6. `./scripts/colcon_test_venv.sh --packages-select sc2_visual_servo_perception sc2_visual_servo_controller sc2_visual_servo_bringup`

Pending full verification item:
- End-to-end script execution in a host that actually provides `/opt/ros/humble/setup.bash` and Humble `colcon` runtime.

---

## 9. Deferred: Detection topic mapping & inference pipeline

### 9.1 Current observed facts

Verified:
- uv + venv + colcon workflow was validated in a Humble container.
- MVP sim pipeline (stub target) runs end-to-end:
  - `target_obs_publisher -> visual_servo_controller_node -> /sc2/visual_servo/cmd_vel_body -> csv_logger_node -> cmd_vel_to_mavlink_stub`
- Jetson depth topic observation:
  - `/oak/stereo/image_raw` encoding is `16UC1`
  - median observed value was around `1609` (interpreted as about `1.6 m` if mm scale)
  - max often reaches `65535` (sentinel-like value)

Suspected:
- Detection pipeline instability is tied to ONNX/TensorRT engine build/parse failures seen in prior logs.
- In the observed session, `/detections_output` existed as `vision_msgs/msg/Detection2DArray`, but active publisher count was `0` while subscriber side was present.

Deferred:
- Finalizing `detections_topic` mapping/remaps and stabilizing the TensorRT/ONNX detection path are deferred pending colleague input on existing launch/topic conventions.

### 9.2 Why this is deferred

- Existing launch chains include multiple containers/remaps, and the observed topic visibility did not guarantee an active detector publisher.
- Resolving this efficiently needs insider knowledge of the current team-maintained topic naming/remap policy and which launch file is authoritative for production detection.

### 9.3 Inputs needed from colleague

- Exact detection output topic name(s) expected in the active Jetson launch profile.
- Expected detection message type per topic (`vision_msgs/msg/Detection2DArray` or nitros variant).
- Where remaps are defined (launch file paths and container composition points).
- Whether current deployment is nitros-only on critical edges and if bridge topics are intentionally disabled.
- Which ONNX/model + TensorRT engine workflow is considered stable baseline on current Jetson image.

### 9.4 Closure criteria for this deferred item

- `ros2 topic info -v <detections_topic>` shows publisher count `> 0` consistently during runtime.
- Detection topic rate is stable enough for controller input (target: steady non-zero `ros2 topic hz`).
- `target_observer_node` publishes `/sc2/visual_servo/target_obs` from real detections (not stub), and controller emits `/sc2/visual_servo/cmd_vel_body`.
- Reproducible by documented command sequence (live and/or bag replay).

### 9.5 Recent investigation timeline (2026-02-23, approximate)

- `2026-02-23 09:00-10:30` Verified:
  - uv bootstrap/build/test flow validated in Humble container.
  - libexec layout issue fixed via post-build shims under `install/lib/<pkg>`.
  - `set -u` env sourcing hardened (`COLCON_TRACE` / `AMENT_*` defaults + safe source script).
- `2026-02-23 10:30-11:30` Verified:
  - Ctrl-C shutdown RCLError removed by switching nodes to `rclpy.try_shutdown()`.
  - MVP stub launch runs and publishes expected command/log outputs.
- `2026-02-23 13:00-15:00` Verified:
  - Perception-side integration landed:
    - bringup `use_stub_target` switch
    - configurable `target_observer_node`
    - depth ROI median extraction, stale detection drop, throttled warnings
    - rosbag record/play scripts
    - synthetic integration pytest
- `2026-02-23 15:00-16:00` Verified:
  - Jetson depth topic checks confirmed `16UC1` depth behavior and plausible mm-scale values.
- `2026-02-23 16:00-18:00` Suspected/Deferred:
  - `/detections_output` had no active publisher in observed session.
  - ONNX/TensorRT parse/engine-plan failures were observed in related runs.
  - Topic naming + stable publisher path deferred pending colleague input.

### 9.6 Concrete next steps

1. Run colleague sync focused on detection launch contract:
   - confirm authoritative launch file and exact detection topic output names/types.
2. Re-run live pipeline with confirmed remaps and record evidence:
   - `ros2 topic info -v <detections_topic>`
   - `ros2 topic hz <detections_topic>`
3. Validate observer-controller chain without stub:
   - `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py use_stub_target:=false ...`
   - verify `/sc2/visual_servo/target_obs` and `/sc2/visual_servo/cmd_vel_body`.
4. If TensorRT instability persists:
   - pin to team-approved ONNX/model/engine workflow and document exact environment versions.
