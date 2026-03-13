# BlueROV MAVROS Quickcheck (Synthetic Visual Servo)

Scope: active runbook for synthetic visual-servo actuation checks through MAVROS/MAVLink.
Source of truth: use this file for BlueROV rc_override and dry-run validation steps.
Last updated: 2026-03-13
Supersedes: scattered MAVROS quick-test notes in status and README drafts.

## Purpose

Validate the end-to-end actuation path before stabilizing the real detector:

`synthetic bbox -> visual servo (bbox_approach) -> MAVROS/MAVLink -> vehicle command output`

This flow is detector-free and does not depend on SLAM or `/BlueROV2/pose` feedback.
The launch also starts `visual_servo_debug_viz_node` by default, which publishes `MarkerArray` on `/sc2/visual_servo/debug_markers` for quick visual inspection in RViz/Foxglove.

Update note (what changed / why):
- Added MAVROS dependency preflight via `scripts/ensure_ros_apt_deps.sh` to prevent silent rc_override fallback when `mavros_msgs` is missing.
- Preflight runs from `scripts/uv_bootstrap.sh` and `scripts/colcon_build_venv.sh`; set `SC2_SKIP_APT=1` to opt out of automatic apt installation.
- `ros2 topic echo` examples are standardized to `--once` for ROS2 Humble compatibility.

## Reference-only scripts (external, not executed in this workspace build)

- `docs/reference/blue_rov_position_control/pos_controller.py`
- `docs/reference/blue_rov_position_control/pos_setpoint.py`
- These files are preserved as upstream behavior references only; they are not launched or tested by `colcon` in this repository.

## Canonical facts extracted from reference scripts

- RC override topic is `/mavros/rc/override` in `docs/reference/blue_rov_position_control/pos_controller.py:148`.
- Pose feedback topic in the reference closed-loop controller is `/BlueROV2/pose` in `docs/reference/blue_rov_position_control/pos_controller.py:159`.
- Reference RC defaults are `YAW_CH=4`, `FORWARD_CH=5`, `LATERAL_CH=6`, `PWM_MID=1500` in `docs/reference/blue_rov_position_control/pos_controller.py:13`, `docs/reference/blue_rov_position_control/pos_controller.py:14`, `docs/reference/blue_rov_position_control/pos_controller.py:15`, `docs/reference/blue_rov_position_control/pos_controller.py:16`.
- Depth setpoint topic in reference depth mode is `/mavros/setpoint_position/global` in `docs/reference/blue_rov_position_control/pos_setpoint.py:52`.

## Prerequisites

- Run ROS2 commands inside the Humble container.
- Use a container user with `sudo` privileges (default workflow uses `admin`).
- Install MAVROS message dependency for rc_override:

```bash
sudo apt-get install -y ros-humble-mavros-msgs
```

- Verify dependency resolution:

```bash
ros2 interface show mavros_msgs/msg/OverrideRCIn
python3 -c "import mavros_msgs"
```

- Or run the workspace preflight:

```bash
./scripts/ensure_ros_apt_deps.sh
```

- To opt out of automatic apt installation in scripts, set `SC2_SKIP_APT=1` (preflight will fail fast with manual install instructions if deps are missing).

- MAVROS is running and connected to the vehicle/autopilot.
- Tethered safety setup is in place.
- Confirm arming/disarming procedure and manual override before testing.

## Launch command

Dry-run backend (no MAVROS output, command direction check only):

```bash
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  pattern:=fixed_offsets \
  controller_mode:=bbox_approach \
  mavros_backend:=dry_run \
  deadman:=true
```

RC override backend (default, MAVROS `/mavros/rc/override` output):

```bash
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  pattern:=size_ramp \
  controller_mode:=bbox_approach \
  deadman:=true
```

Direct TargetObservation fallback (bypass observer):

```bash
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  publish_target_obs_direct:=true \
  pattern:=lissajous \
  mavros_backend:=dry_run \
  deadman:=true
```

## Synthetic patterns

Configured in `sc2_visual_servo_synth/synthetic_bbox_publisher_node.py` via `pattern` parameter:

- `fixed_offsets`: right/left/up/down step offsets.
- `size_ramp`: bbox size increases/decreases over time.
- `random_walk`: slow random-walk drift around center.
- `lissajous`: smooth periodic drift.
- `loss_reacquire`: alternating visible/lost phases.

Optional dropout can be injected for any pattern using:

- `dropout_period_s`
- `dropout_duration_s`

## Safety controls (implemented)

- Deadman gating (`deadman_topic`, `deadman_required`, `deadman_initial`).
- Command timeout (`timeout_ms`): stale commands produce neutral/zero output.
- Command limits (`max_vx`, `max_vy`, `max_vz`, `max_yaw_rate`).
- Slew-rate limiting in controller (`slew_vx`, `slew_vy`, `slew_vz`, `slew_yaw_rate`).

## Verification commands

Check visual-servo output command stream:

```bash
ros2 topic hz /sc2/visual_servo/cmd_vel_body
ros2 topic echo /sc2/visual_servo/cmd_vel_body --once
ros2 topic echo /sc2/visual_servo/status --once
```

Check visualization markers (bbox + command vector):

```bash
ros2 topic hz /sc2/visual_servo/debug_markers
ros2 topic echo /sc2/visual_servo/debug_markers --once
```

The marker topic is designed for both RViz2 and Foxglove.

Check MAVROS state/output:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/rc/override --once
```

Check synthetic detections:

```bash
ros2 topic hz /detections_output
ros2 topic echo /detections_output --once
```

Check deadman timeout behavior:

```bash
ros2 topic pub /sc2/visual_servo/deadman std_msgs/msg/Bool "{data: false}" --once
ros2 topic echo /mavros/rc/override --once
```

Then re-enable:

```bash
ros2 topic pub /sc2/visual_servo/deadman std_msgs/msg/Bool "{data: true}" --once
```

Dynamic toggle example (start with deadman disabled, then enable at 10 Hz):

```bash
# Terminal A: launch with deadman false
ros2 launch sc2_visual_servo_bringup blue_rov_synth_vs.launch.py \
  pattern:=fixed_offsets \
  controller_mode:=bbox_approach \
  deadman:=false

# Terminal B: publish deadman true for a few seconds
ros2 topic pub -r 10 /sc2/visual_servo/deadman std_msgs/msg/Bool "{data: true}"
```

Observe bridge logs switching to `deadman=True active=True`.

## Troubleshooting: deadman published but bridge stays false

Symptom:
- `ros2 topic list` / `ros2 topic info /sc2/visual_servo/deadman` shows the topic, but bridge logs remain `deadman=False`.

Fix:
- When publishing from another shell via Docker, use the same user as the running nodes (typically `admin`):

```bash
docker exec -it -u admin <container> bash
```

- Publishing as `root` can show the topic but messages may not be delivered in FastDDS shared-memory setups due to permission/IPC mismatch.
- If you must publish as another user, align permissions or disable SHM transport for DDS.

## Common pitfalls

- Symptom: startup log shows `rc_override backend unavailable (mavros_msgs missing)`.
- Fix: run `./scripts/ensure_ros_apt_deps.sh`, then relaunch.
- Note for ROS2 Humble CLI: use `ros2 topic echo --once`; `-n` is not supported.

## Deadman and RC override semantics

- `deadman=false` does not necessarily mean "no `/mavros/rc/override` traffic".
- Current bridge behavior is:
  - mapped control channels (forward/lateral/vertical/yaw) are driven to neutral PWM on timeout or `deadman=false`
  - non-mapped channels are published as `CHAN_NOCHANGE` (not overridden)
- This means a neutral override message can still be visible on `/mavros/rc/override` while actuation commands are effectively inactive.

## Expected behavior checks

- BBox right of image center: yaw command sign should be right-turn for configured frame/sign convention.
- BBox left of center: yaw command sign should invert.
- BBox area grows large: forward command should reduce and stop near threshold.
- BBox lost (loss/reacquire or dropout): command should go neutral/zero after timeout.

## Example RC mapping (configurable)

The default mapping in `cmd_vel_to_mavros_bridge.py` follows the same channel assumptions used in `docs/reference/blue_rov_position_control/pos_controller.py:13`, `docs/reference/blue_rov_position_control/pos_controller.py:14`, `docs/reference/blue_rov_position_control/pos_controller.py:15`, `docs/reference/blue_rov_position_control/pos_controller.py:16`:

- Yaw: channel 4
- Forward: channel 5
- Lateral: channel 6
- Vertical: channel 3
- Neutral PWM: 1500

All mapping parameters are configurable:

- `channel_yaw`, `channel_forward`, `channel_lateral`, `channel_vertical`
- `neutral_pwm`, `min_pwm`, `max_pwm`
- `scale_forward_pwm_per_mps`, `scale_lateral_pwm_per_mps`, `scale_vertical_pwm_per_mps`, `scale_yaw_pwm_per_rps`

Reference URLs provided by colleague:

- https://github.com/tum-erl/position_control/blob/master/pos_control/pos_controller.py
- https://github.com/tum-erl/position_control/blob/master/pos_control/pos_setpoint.py

## Notes

- If detections path is unstable, keep using synthetic detections or direct `TargetObservation` mode to validate controller-to-MAVROS behavior.
- For first milestone, open-loop command verification is sufficient; no vehicle pose feedback is required.

## Future TODO

- Closed-loop position control integration (reference parity) should consume `/BlueROV2/pose` (`PoseStamped`) as shown in `docs/reference/blue_rov_position_control/pos_controller.py:159`.

## Done definition (rc_override dependency + CLI compatibility)

- For `mavros_backend:=rc_override`, missing `mavros_msgs` is detected before launch/build in preflight.
  - If `SC2_SKIP_APT!=1`: install `ros-humble-mavros-msgs` and re-check.
  - If `SC2_SKIP_APT=1`: fail fast with explicit error and manual install command.
- In a ready environment, `/mavros/rc/override` publishing is observable (for example via `ros2 topic hz /mavros/rc/override`).
- QuickCheck commands are ROS2 Humble compatible (`ros2 topic echo --once` only, no `-n` usage).
