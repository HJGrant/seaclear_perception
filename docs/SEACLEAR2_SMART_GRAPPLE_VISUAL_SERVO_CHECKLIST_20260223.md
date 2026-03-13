# SeaClear2.0 — Smart Grapple Visual Servoing
## At-a-glance Implementation Checklist (as of 2026-02-22, prepared for 2026-02-23 sync)

Scope: active status tracker for the visual-servo MVP and controller integration work.
Source of truth: use this file for weekly progress and blocker tracking.
Last updated: 2026-03-13
Supersedes: routine updates previously added to `SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_STATUS_20260223.md`, now archived under `docs/archive/2026Q1/`.

This is the primary status checklist (`DONE`/`PARTIAL`/`NOT_STARTED`) for weekly progress tracking and "what's blocking MVP?" decisions.

### Status legend
- `DONE`: implemented and runnable in a repeatable way.
- `PARTIAL`: exists but fragile, not yet an "easy experiment."
- `PROTOTYPE`: works in legacy stack or toy setting; needs port/adaptation.
- `INTEGRATION_NEEDED`: components exist but are not wired end-to-end.
- `OPTIONAL`: valuable but not required for the MVP loop.
- `NOT_STARTED`: not started or missing.

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
| G1. Perception output for VS | ROS2 publishes: bbox (target) + depth-to-target (or proxy) at ≥ ~10 Hz | IN_PROGRESS | `TargetObservation` is standardized and observer wiring exists; remaining blocker is stable real detection-topic/inference bringup. |
| G2. Servo controller runnable | A node consumes (bbox, depth/altitude) and outputs body-frame `Twist` at control rate | VERIFIED | ROS2 controller node supports `ibvs`, `bbox_approach`, `bbox_target_point`, and optional `mppi` fallback. |
| G3. Plant interface | Commands actually move the vehicle in sim and (later) BlueROV | IN_PROGRESS | Sim and MAVROS bridge paths exist; remaining work is hardware validation and operator workflow hardening. |
| G4. Frame conventions | Camera optical frame ↔ body frame mapping is defined & tested (TF sanity) | IN_PROGRESS | Topic/frame contract is documented, but hardware-validated TF/sign checks are still pending. |
| G5. “One-command demo” | One launch command that brings up **perception + VS + visualization** | VERIFIED | `mvp_sim.launch.py` and `blue_rov_synth_vs.launch.py` bring up end-to-end graphs with optional debug markers. |
| G6. Regression harness | Record/replay (rosbag) + plots to verify convergence | IN_PROGRESS | Bag record/play and explicit pytest integration checks exist; offline convergence plotting is still pending. |

---

## 2) Checklist by subsystem

### 2.1 Perception pipeline (what D5.2 expects)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| P1. Stereo RGBD source | Stereo camera provides RGB + depth (aligned or alignable) | PARTIAL | OAK-D pipeline exists; “stable mode” is constrained (stereo-only baseline is stable; richer modes trigger failures) — treat as *integration risk*. |
| P2. YOLO detection | YOLOv8 outputs bbox for litter objects | PARTIAL | Implemented in ROS2 workspace (needs confirmation on underwater dataset + runtime). D5.2 expects bbox as the main VS cue. |
| P3. Depth masking | Use bbox to mask depth map into target vs non-target | PARTIAL | VS path currently uses bbox-ROI depth statistics; mask-based gating exists for target-point estimation but is not the default observer path. |
| P4. Voxel/TSDF map (optional for MVP) | TSDF-based voxel reconstruction around target ROI | OPTIONAL | Valuable later; **not required** for MVP VS loop. D5.2 describes it for geometry/obstacles. |
| P5. Monitoring | Foxglove/Gstreamer monitoring of RGB+bbox+map | PARTIAL | Present in concept; ensure a "demo panel" exists for debugging. |

### 2.2 Visual servo controller (what winter school proposes)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| C1. IBVS core | Feature error from image → velocity command (camera-centric) | VERIFIED | Implemented in ROS2 controller node and used as the default MVP mode. |
| C2. IBVS 2½D variant | Use augmented feature incl. depth log-term for robustness | NOT_STARTED | In slides; not yet integrated. |
| C3. MPC/MPPI with FOV constraint | Constrain bbox center within image bounds (probabilistic margin) | PROTOTYPE | MPPI core contains FOV/risk-cost utilities, but the shipped ROS2 configuration uses `risk_mode=none` and does not enable probabilistic constraints by default. |
| C4. PBVS option | Use vSLAM pose (camera frame) when reliable | PARTIAL | Depends on stable vSLAM integration; planned switching policy exists conceptually. |
| C5. IBVS↔PBVS switching | Switch based on map convergence / tracking loss / uncertainty | NOT_STARTED | Not yet implemented as a state/guard. |
| C6. Command shaping | Saturation, smoothing, deadband, integrator windup guards | VERIFIED | Saturation, timeout-to-zero, and slew-rate limiting are implemented in the ROS2 stack; integral control is not used in the current MVP controller. |

### 2.3 State machine / transitions (D5.2 integration expectation)
| Item | Requirement | Status | Notes |
|---|---|---:|---|
| S1. Trigger condition | Enter VS after tracking reaches final path point and visibility is sufficient | NOT_STARTED | D5.2 states this trigger. |
| S2. VS termination | Exit VS when desired camera-frame pose achieved → grasping | NOT_STARTED | Needs clear termination metric (bbox centered + altitude + (optionally) yaw). |
| S3. Manual fallback | Manual grasp / joystick override | PROTOTYPE | Present conceptually in system; confirm in current ROS2 stack. |
| S4. Failure fallback | If visibility bad, position based on last known trash location | NOT_STARTED | Must define behavior and operator UI hook. |

---

## 3) Shortest path work plan (integrated status matrix)

This section is the current baseline for implementation status and replaces standalone updates in later sections.

### Phase A (Week 1-2): Make VS loop runnable in simulation
| Item | Current status | Implementation path(s) | Notes |
|---|---|---|---|
| A1. Define VS I/O contract (`bbox`, `depth/altitude`, `Twist`) | Verified | `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg`, `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py` | `TargetObservation` and `/sc2/visual_servo/cmd_vel_body` are standardized. |
| A2. ROS2 shim publisher for controller bring-up | Verified | `src/sc2_visual_servo_controller/sc2_visual_servo_controller/target_obs_publisher.py`, `src/sc2_visual_servo_synth/sc2_visual_servo_synth/synthetic_bbox_publisher_node.py` | Stub and synthetic detection publishers exist. |
| A3. ROS2 controller baseline (IBVS first) | Verified | `src/sc2_visual_servo_controller/sc2_visual_servo_controller/ibvs/controller_math.py`, `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py` | `ibvs`, `bbox_approach`, `bbox_target_point` are available; MPPI is optional. |
| A4. One-command simulation demo + docs | Verified | `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`, `README_quickstart.md` | End-to-end MVP launch path is documented. |

### Phase B (Week 3-4): Wire perception to controller (end-to-end in sim/live)
| Item | Current status | Implementation path(s) | Notes |
|---|---|---|---|
| B1. Connect YOLO bbox topic to VS input | In Progress | `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`, `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py` | Wiring exists via `use_stub_target:=false`; detection topic stabilization remains deferred. |
| B2. Depth-to-target estimation in consistent units | Verified | `src/sc2_visual_servo_perception/sc2_visual_servo_perception/conversion.py`, `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_point_estimator_node.py` | ROI median and UV-median-neighborhood representative depth are implemented. |
| B3. FOV constraint (hard/probabilistic) | In Progress | `src/sc2_visual_servo_controller/sc2_visual_servo_controller/pbvs/mppi_core.py` | FOV/risk cost exists in MPPI core; not default path in MVP IBVS mode. |
| B4. Reproducible evaluation harness | In Progress | `scripts/record_perception_bag.sh`, `scripts/play_perception_bag.sh`, `src/sc2_visual_servo_perception/test/test_target_observer_integration.py`, `src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py` | Record/replay and perception-side integration tests exist; offline convergence plotting is still pending. |

### Phase C (Week 5-6): Pilot on BlueROV (minimum safe integration)
| Item | Current status | Implementation path(s) | Notes |
|---|---|---|---|
| C1. Vehicle interface (`Twist` to BlueROV command) | Verified | `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavros_bridge.py`, `src/sc2_visual_servo_bringup/launch/blue_rov_synth_vs.launch.py` | `rc_override` / `setpoint_velocity` / `dry_run` backends are implemented. |
| C2. TF contract validated on hardware | Deferred | `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py` | Hardware-validated TF contract is still pending. |
| C3. Safety guards (timeout/deadman/limits) | Verified | `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node.py`, `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavros_bridge.py` | Saturation, slew limit, timeout-to-zero, deadman timeout/neutral are implemented. |
| C4. Operator workflow (start/stop/manual override/logging) | In Progress | `docs/BLUEROV_MAVROS_QUICKCHECK.md`, `src/sc2_visual_servo_controller/sc2_visual_servo_controller/csv_logger_node.py` | Basic runbook and logging exist; full mission-level manual override workflow remains pending. |

---

## 4) Known integration risks (track weekly)
- **VSLAM stability risk:** PBVS path depends on reliable vSLAM; keep IBVS MVP independent.
- **Frame/sign convention risk:** camera optical frame vs body frame mismatch will “look like bad tuning” — must test with unit vectors early.
- **Depth quality risk:** underwater depth noise/outliers will break 2½D IBVS unless filtered; plan for robust depth (median in masked ROI, etc.).

---

## 5) How to use this checklist weekly
- Update **Gate statuses (G1–G6)** first.
- Then update **Phase A/B/C** status rows in Section 3.
- Keep any new blockers in “Known integration risks”.

## 6) Sources used for this checklist
- `docs/SC2/20251225_D5_2_Control_Overhaul_SC2.pdf`
- `docs/reference/implementation_goal/winter school_v3.pdf`
- `docs/archive/2026Q1/SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_STATUS_20260223.md`

---

## 7) 2026-02-23 Frozen Progress Register

### Item: Visual Servo MVP stub pipeline in Humble container
Status: Verified
Evidence:
- `README_quickstart.md` one-command launch flow is documented.
- Observed runtime chain: `target_obs_publisher -> visual_servo_controller_node -> /sc2/visual_servo/cmd_vel_body -> csv_logger_node -> cmd_vel_to_mavlink_stub`.
- Output artifact path: `/tmp/sc2_visual_servo_mvp.csv`.
Owner: SC2 visual-servo team

### Item: ROS2 perception observer path implemented (topic names provisional)
Status: In Progress
Evidence:
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
- `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py` with `use_stub_target` switch.
- `docs/PERCEPTION_TOPIC_MAP.md` marks detections mapping as provisional.
Owner: SC2 visual-servo team

### Item: Tests and reproducibility harness
Status: In Progress
Evidence:
- Unit tests: `src/sc2_visual_servo_perception/test/test_conversion.py`, `src/sc2_visual_servo_controller/test/test_controller_sign.py`
- Integration tests: `src/sc2_visual_servo_perception/test/test_target_observer_integration.py`, `src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py`
- Rosbag scripts: `scripts/record_perception_bag.sh`, `scripts/play_perception_bag.sh`
Owner: SC2 visual-servo team

### Item: uv dependency management workflow
Status: Verified
Evidence:
- `pyproject.toml`, `uv.lock`, `scripts/uv_bootstrap.sh`, `scripts/colcon_build_venv.sh`, `scripts/colcon_test_venv.sh`
- Canonical command sequence is documented in `README_quickstart.md`.
Owner: SC2 visual-servo team

---

## 8) 2026-02-23 Jetson and Pipeline Checkpoint

### Item: Depth topic validation on Jetson session (2026-02-23)
Status: Verified
Evidence:
- Topic: `/oak/stereo/image_raw` (`sensor_msgs/msg/Image`)
- Encoding: `16UC1`
- Observed depth statistics (approx): min `633`, median `1609`, max `65535`
- Interpretation: mm-scale depth; `65535` acts as invalid/sentinel and is excluded in ROI statistics.
Owner: SC2 visual-servo team

### Item: Host vs container execution boundary
Status: Verified
Evidence:
- Supported workflow: run ROS2 commands and cv_bridge-dependent nodes inside Humble container.
- Host-side Python/ROS commands may show NumPy/cv_bridge mismatch and are outside supported workflow.
Owner: SC2 visual-servo team

### Item: Detection topic mapping (pending colleague confirmation)
Status: Deferred
Evidence:
- Candidate topic observed: `/detections_output` (`vision_msgs/msg/Detection2DArray`).
- In problematic session: `ros2 topic info -v /detections_output` showed publisher count `0` with subscriber present (`tensor_rt_container`).
- Mapping/remap authority is not yet confirmed; topic names remain provisional.
Owner: Colleague + SC2 visual-servo team

### Item: Exposure control validated
Status: TBD
Evidence:
- Whiteout/overexposure impact is currently documented as a hypothesis requiring dedicated diagnostics.
Owner: Camera pipeline owner

### Item: OAK-D stability validated
Status: TBD
Evidence:
- Intermittent bringup/drop behavior requires dedicated health checks before using runs as VSLAM/perception validation evidence.
Owner: Camera pipeline owner

### Deferred closure criteria for detections mapping

### Item: Detection publisher availability
Status: Deferred
Evidence:
- `ros2 topic info -v <detections_topic>` must show publisher count `> 0` during active inference.

### Item: Detection topic rate stability
Status: Deferred
Evidence:
- `ros2 topic hz <detections_topic>` must show sustained non-zero publication rate.

### Item: Observer output from real detections
Status: Deferred
Evidence:
- `target_observer_node` must publish `/sc2/visual_servo/target_obs` without stub source.

### Item: Controller output from observer path
Status: Deferred
Evidence:
- `/sc2/visual_servo/cmd_vel_body` must be driven from real observer output with `use_stub_target:=false`.

---
