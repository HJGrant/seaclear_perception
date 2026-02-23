# SeaClear2.0 — Smart Grapple Visual Servoing
## At-a-glance Implementation Checklist (as of 2026-02-22, prepared for 2026-02-23 sync)

This is a **quick status checklist** (✅/⚠️/❌) intended for weekly progress tracking and "what's blocking MVP?" decisions.
It complements the deeper status memo (`SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_STATUS_20260223.md`).

### Status legend
- ✅ **Done** (implemented + runnable in a repeatable way)
- ⚠️ **Partial / unstable** (exists but fragile, not yet “easy experiment”)
- 🧪 **Prototype / legacy** (works in old stack or toy setting; needs port/adaptation)
- 🔁 **Needs integration** (components exist but are not wired end-to-end)
- 🟡 **Optional for MVP** (valuable but not required for the minimum loop)
- ❌ **Not started / missing**

---

## 0) MVP definition (2-month minimum)
**Goal:** After tracking, run **visual servoing** to generate velocity commands that align the grapple to the target for grasping using perception.

**Expected inputs/logic (from D5.2 + winter school):**
- D5.2: visual servoing is triggered after the final tracking waypoint; uses **object detection bbox** to center target and uses **altitude (DVL)** for vertical alignment.
- D5.2: perception pipeline is stereo RGBD → YOLOv8 bbox → depth mask → (optional) voxel/TSDF mapping + monitoring.
- Winter school: start with **IBVS** (camera-centric, robust) and optionally switch to **PBVS** when SLAM is reliable; MPC can include **FOV constraints**.

---

## 1) MVP readiness gates (one-glance)
| Gate | What “DONE” means | Current status | Primary blocker / next action |
|---|---|---:|---|
| G1. Perception output for VS | ROS2 publishes: bbox (target) + depth-to-target (or proxy) at ≥ ~10 Hz | ⚠️ | YOLO bbox is present in ROS2 stack; VS-consumable message/API not standardized yet (define one topic/message + minimal test publisher). |
| G2. Servo controller runnable | A node consumes (bbox, depth/altitude) and outputs body-frame `Twist` at control rate | 🧪🔁 | Controller exists in legacy sim (MPPI/VS); needs “ROS2 wrapper” and input adaptation (bbox-based features). |
| G3. Plant interface | Commands actually move the vehicle in sim and (later) BlueROV | 🧪 | Sim path exists; BlueROV interface & safety gating still to do (rate limits, deadman, mode switching). |
| G4. Frame conventions | Camera optical frame ↔ body frame mapping is defined & tested (TF sanity) | 🔁 | Need a single “authoritative TF contract” and unit tests (sign conventions kill VS silently). |
| G5. “One-command demo” | One launch command that brings up **perception + VS + visualization** | ❌ | Compose minimal launch + parameter file + README steps. |
| G6. Regression harness | Record/replay (rosbag) + plots to verify convergence | ❌ | Add bag + offline evaluation script (bbox trajectory, command saturation, time-to-center). |

---

## 2) Checklist by subsystem

### 2.1 Perception pipeline (what D5.2 expects)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| P1. Stereo RGBD source | Stereo camera provides RGB + depth (aligned or alignable) | ⚠️ | OAK-D pipeline exists; “stable mode” is constrained (stereo-only baseline is stable; richer modes trigger failures) — treat as *integration risk*. |
| P2. YOLO detection | YOLOv8 outputs bbox for litter objects | ⚠️ | Implemented in ROS2 workspace (needs confirmation on underwater dataset + runtime). D5.2 expects bbox as the main VS cue. |
| P3. Depth masking | Use bbox to mask depth map into target vs non-target | ⚠️ | Implemented/partially present; must confirm topic contract and that depth is aligned with RGB. |
| P4. Voxel/TSDF map (optional for MVP) | TSDF-based voxel reconstruction around target ROI | 🟡 | Valuable later; **not required** for MVP VS loop. D5.2 describes it for geometry/obstacles. |
| P5. Monitoring | Foxglove/Gstreamer monitoring of RGB+bbox+map | ⚠️ | Present in concept; ensure a "demo panel" exists for debugging. |

### 2.2 Visual servo controller (what winter school proposes)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| C1. IBVS core | Feature error from image → velocity command (camera-centric) | 🧪🔁 | Planned starting point. |
| C2. IBVS 2½D variant | Use augmented feature incl. depth log-term for robustness | ❌ | In slides; not yet integrated. |
| C3. MPC/MPPI with FOV constraint | Constrain bbox center within image bounds (probabilistic margin) | 🧪 | Legacy sim controller is MPPI-style; needs "FOV constraint port" and tests. |
| C4. PBVS option | Use vSLAM pose (camera frame) when reliable | ⚠️ | Depends on stable vSLAM integration; planned switching policy exists conceptually. |
| C5. IBVS↔PBVS switching | Switch based on map convergence / tracking loss / uncertainty | ❌ | Not yet implemented as a state/guard. |
| C6. Command shaping | Saturation, smoothing, deadband, integrator windup guards | 🧪 | Exists in legacy; must be made explicit in ROS2 stack for safety. |

### 2.3 State machine / transitions (D5.2 integration expectation)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| S1. Trigger condition | Enter VS after tracking reaches final path point and visibility is sufficient | ❌ | D5.2 states this trigger. |
| S2. VS termination | Exit VS when desired camera-frame pose achieved → grasping | ❌ | Needs clear termination metric (bbox centered + altitude + (optionally) yaw). |
| S3. Manual fallback | Manual grasp / joystick override | 🧪 | Present conceptually in system; confirm in current ROS2 stack. |
| S4. Failure fallback | If visibility bad, position based on last known trash location | ❌ | Must define behavior and operator UI hook. |

---

## 3) “Shortest path” work plan (organized as checkboxes)

### Phase A (Week 1–2): **Make VS loop runnable in simulation**
- [ ] A1. Define VS I/O contract (ROS2 messages + topic names): `bbox`, `depth/altitude`, `Twist` output.
- [ ] A2. Make a ROS2 “shim” node that publishes synthetic bbox/depth (for controller bring-up).
- [ ] A3. Port legacy controller into ROS2 (minimal wrapper) OR re-implement simple IBVS baseline first.
- [ ] A4. Add a one-command sim demo launch + README (this is the “easy experiment” threshold).

### Phase B (Week 3–5): **Wire perception → controller (end-to-end in sim)**
- [ ] B1. Connect YOLO bbox topic to VS input.
- [ ] B2. Produce depth-to-target (from stereo depth or DVL proxy) in consistent units.
- [ ] B3. Add FOV constraint (hard clamp first; probabilistic later).
- [ ] B4. Add evaluation harness: rosbag + offline plots (error vs time, command saturation, time-to-center).

### Phase C (Week 6–8): **Pilot on BlueROV (minimum safe integration)**
- [ ] C1. Vehicle interface: `Twist` → BlueROV command (MAVROS/bridge) + rate limits + deadman.
- [ ] C2. TF contract validated on hardware: camera extrinsics, body axes, optical frame.
- [ ] C3. Safety: abort conditions (lost target N frames, depth invalid, saturation too long).
- [ ] C4. Operator workflow: start/stop VS, manual override, logging.

---

## 4) Known integration risks (track weekly)
- **VSLAM stability risk:** PBVS path depends on reliable vSLAM; keep IBVS MVP independent.
- **Frame/sign convention risk:** camera optical frame vs body frame mismatch will “look like bad tuning” — must test with unit vectors early.
- **Depth quality risk:** underwater depth noise/outliers will break 2½D IBVS unless filtered; plan for robust depth (median in masked ROI, etc.).

---

## 5) How to use this checklist weekly
- Update **Gate statuses (G1–G6)** first.
- Then update **Phase A/B/C** checkboxes.
- Keep any new blockers in “Known integration risks”.

## 6) Sources used for this checklist
- `docs/SC2/20251225_D5_2_Control_Overhaul_SC2.pdf`
- `docs/implementation_goal/winter school_v3.pdf`
- `docs/SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_STATUS_20260223.md`

---

## 7) 2026-02-23 Progress Update (Implemented / Verified / Pending)

### Implemented
- [x] `A1` VS I/O contract and custom message
  - `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg`
- [x] `A2` ROS2 synthetic observation publisher
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/target_obs_publisher.py`
- [x] `A3` ROS2 controller node (IBVS baseline + MPPI mode hook)
  - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py`
  - MPPI core extracted from SC1:
    - `src/sc2_visual_servo_controller/sc2_visual_servo_controller/mppi_core.py`
- [x] `A4` one-command launch demo
  - `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
  - command: `ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py`
- [x] Perception wrapper for existing YOLO topic
  - `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
  - input: `/detections_output` (`vision_msgs/Detection2DArray`)
  - output: `/sc2/visual_servo/target_obs`
- [x] Logging + vehicle command stub
  - CSV logger: `src/sc2_visual_servo_controller/sc2_visual_servo_controller/csv_logger_node.py`
  - MAVLink stub: `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavlink_stub.py`

### Verified
- [x] Unit test: conversion
  - `src/sc2_visual_servo_perception/test/test_conversion.py`
- [x] Unit test: controller sign
  - `src/sc2_visual_servo_controller/test/test_controller_sign.py`
- [x] Integration smoke test: launch + cmd topic alive
  - `src/sc2_visual_servo_bringup/test/test_mvp_sim_launch.py`

### Pending
- [ ] SC1 simulator adapter path (alternate to ROS2-only harness)
- [ ] Hardware command bridge beyond logging stub
- [ ] MPPI mode validation against bag replay and real detections

### uv dependency management status (2026-02-23)
- [x] Implemented: `pyproject.toml` with `requires-python = ">=3.10,<3.11"` and `numpy<2`
- [x] Implemented: `uv.lock` committed
- [x] Implemented: bootstrap/build/test scripts
  - `scripts/uv_bootstrap.sh`
  - `scripts/colcon_build_venv.sh`
- `scripts/colcon_test_venv.sh`
- [x] Implemented: repository hygiene for `.venv/`, `__pycache__/`, `*.pyc`
- [ ] Verified in this host: full Humble-based build+test via `/opt/ros/humble/setup.bash`

---

## 8) 2026-02-23 Jetson/Perception checkpoint

- [x] Depth topic validated on Jetson:
  - `/oak/stereo/image_raw` encoding is `16UC1` (mm scale interpretation in observed run)
  - observed median around `1609` (about `1.6 m`)
  - observed max often `65535` (sentinel-like)
- [i] Use container for ROS2/perception commands; host Python may have NumPy/cv_bridge mismatch (informational):
  - supported workflow is to run ROS2 and cv_bridge-dependent nodes in the Humble container
  - host-side Python environment alignment is out of scope for this project setup
- [DEFERRED] Detection topic mapping and inference pipeline stabilization:
  - observed `/detections_output` topic type `vision_msgs/msg/Detection2DArray`, but publisher count was `0` in the problematic session
  - ONNX/TensorRT parse/engine-plan failures were observed in related runs
  - deferred pending colleague input on authoritative launch/remap/topic conventions

### Deferred closure criteria

- [ ] `ros2 topic info -v <detections_topic>` shows publisher count `> 0` while inference is running
- [ ] `ros2 topic hz <detections_topic>` is stable (non-zero, sustained)
- [ ] `target_observer_node` publishes `/sc2/visual_servo/target_obs` from detections (not stub path)
- [ ] controller publishes `/sc2/visual_servo/cmd_vel_body` from real observer output
