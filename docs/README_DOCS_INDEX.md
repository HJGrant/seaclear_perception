# Docs Index

Scope: current documentation map for the SC2 visual-servo MVP repository.
Source of truth: this index classifies which documents are active, archival, or lower-priority domain notes.
Last updated: 2026-03-13
Supersedes: ad hoc document placement at `docs/`, `docs/status/`, and `docs/worklogs/`.

## Active documents

- `../README_quickstart.md` — Primary run/build/test entry point for the MVP workflow.
- `SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_CHECKLIST_20260223.md` — Primary status tracker and weekly implementation checklist.
- `SC2_VISUAL_SERVO_MVP_NODE_GRAPH_20260223.md` — Current ROS 2 node graph and topic contracts for the MVP pipeline.
- `PERCEPTION_TOPIC_MAP.md` — Current perception topic contract and live/bag inspection notes.
- `VS_BBOX_TARGET_POINT_INTEGRATION_20260225.md` — Current bbox+depth ROI and target-point integration design notes.
- `VISUAL_SERVO_SYSTEM_INTEGRATION_PLAN_20260313.md` — Mapping and design plan for integrating `oakd_nvblox.launch.py` with the visual-servo stack under a new `oakd_system.launch.py`.
- `BLUEROV_MAVROS_QUICKCHECK.md` — Current BlueROV synthetic actuation test runbook.

## Reference documents

- `reference/blue_rov_position_control/` — External BlueROV MAVROS reference scripts (`pos_controller.py`, `pos_setpoint.py`) for topic/channel defaults.
- `reference/implementation_goal/winter school_v3.pdf` — Winter school planning slides referenced by controller design notes.
- `SC2/20251225_D5_2_Control_Overhaul_SC2.pdf` — D5.2 PDF requirement source.
- `SC2/D5.2_Smart_Grapple_Report_SC2.tex` — D5.2 LaTeX source.
- `../scripts/ensure_ros_apt_deps.sh` — Preflight installer/check for `ros-humble-mavros-msgs` required by rc_override workflow.

## OAK-D and VSLAM documents

- `oakd-vslam/PROJECT_STATUS_OAKD_VSLAM.md` — Lower-priority OAK-D/VSLAM status and priorities.
- `oakd-vslam/TEST_MATRIX.md` — OAK-D/VSLAM config test matrix and runbook.
- `oakd-vslam/WORKLOG_20260223.md` — Chronological OAK-D/VSLAM work log (2026-02-23).
- `oakd-vslam/PIPELINE_HEALTHCHECK.md` — Runnable diagnostics for exposure and OAK-D stability gates.
- `oakd-vslam/OAKD_VSLAM_DEBUG_NOTES.md` — Parameter audit and staged VSLAM debugging notes.
- `oakd-vslam/upstream_issue_template_depthai_ros_driver.md` — Crash report template for DepthAI/OAK-D upstream issues.
- `oakd-vslam/attachments/` — Zipped OAK-D/VSLAM diagnosis logs and reference snapshots.

## Archive

- `archive/2026Q1/SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_STATUS_20260223.md` — Frozen status memo retained for historical context only.

## Maintenance rules

- Update progress in `SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_CHECKLIST_20260223.md`; do not create a new status memo for routine updates.
- Keep runnable command sequences in `README_quickstart.md` or `BLUEROV_MAVROS_QUICKCHECK.md`; other docs should link instead of duplicating commands.
- Put new dated investigation notes under `archive/` unless they remain part of the current operating workflow.
