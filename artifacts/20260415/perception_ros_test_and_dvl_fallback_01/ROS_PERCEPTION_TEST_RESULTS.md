# ROS Perception Test Results

Date: 2026-04-15

## Environment

Host workspace: `/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz`

ROS container image used:

```text
isaac_ros_dev-x86_64:latest
```

The image provides ROS 2 Humble Python packages. It was run with the repository bind-mounted at:

```text
/workspaces/oakd_isaac_ros_lrz
```

## Primary Perception Test

Command:

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/workspaces/oakd_isaac_ros_lrz \
  -w /workspaces/oakd_isaac_ros_lrz \
  isaac_ros_dev-x86_64:latest \
  bash -lc 'source /opt/ros/humble/setup.bash &&
    export PYTHONPATH=/workspaces/oakd_isaac_ros_lrz/src/sc2_visual_servo_perception:$PYTHONPATH &&
    python3 -m pytest -q src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py'
```

Result:

```text
.......                                                                  [100%]
7 passed in 1.00s
```

## Covered Behavior

| behavior | test coverage |
| --- | --- |
| `target_point.pose.position.z` is target-relative range | `test_target_point_pose_fields_use_camera_offsets_and_z_range` |
| metric x/y offsets are derived from representative UV and depth | `test_target_point_pose_fields_use_camera_offsets_and_z_range` |
| temporal filter reduces small pixel jitter | `test_temporal_filter_reduces_pixel_jitter_in_metric_target_point` |
| depth jump rejection holds last valid pose | `test_large_depth_jump_is_rejected_and_holds_last_valid_pose` |
| bbox jump rejection holds last valid pose | `test_large_bbox_jump_is_rejected_and_holds_last_valid_pose` |
| short invalid depth gap holds last valid pose | `test_invalid_depth_short_gap_holds_last_valid_pose` |
| short stale depth gap holds last valid pose | `test_stale_depth_short_gap_holds_last_valid_pose` |
| long invalid depth gap suppresses new pose | `test_invalid_depth_after_hold_window_is_not_published` |

## Related Test Attempt

Command:

```bash
docker run --rm ... python3 -m pytest -q \
  src/sc2_visual_servo_perception/test/test_conversion.py \
  src/sc2_visual_servo_perception/test/test_target_observer_integration.py \
  src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py \
  src/sc2_visual_servo_controller/test/test_controller_sign.py
```

Result: blocked before running tests because the bind-mounted workspace did not have generated Python message modules for `sc2_visual_servo_interfaces`.

Failure class: dependency/build artifact missing in this direct container run, not an implementation failure in `target_point_estimator_node`.

Key error:

```text
ModuleNotFoundError: No module named 'sc2_visual_servo_interfaces'
```

## Related Tests Without Generated Interface Package

Command:

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/workspaces/oakd_isaac_ros_lrz \
  -w /workspaces/oakd_isaac_ros_lrz \
  isaac_ros_dev-x86_64:latest \
  bash -lc 'source /opt/ros/humble/setup.bash &&
    export PYTHONPATH=/workspaces/oakd_isaac_ros_lrz/src/sc2_visual_servo_perception:/workspaces/oakd_isaac_ros_lrz/src/sc2_visual_servo_controller:$PYTHONPATH &&
    python3 -m pytest -q \
      src/sc2_visual_servo_perception/test/test_conversion.py \
      src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py \
      src/sc2_visual_servo_controller/test/test_controller_sign.py'
```

Result:

```text
..................                                                       [100%]
18 passed in 1.23s
```

## Controller Compare Regression

Commands:

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase1.py
uv run python tools/compare_staged_pid_horizontal_behavior.py
uv run python tools/compare_visual_servo_controller_nodes_phase2.py \
  --out-dir artifacts/20260415/perception_ros_test_and_dvl_fallback_01/phase2_baseline_compare
uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py \
  --out-dir artifacts/20260415/perception_ros_test_and_dvl_fallback_01/phase2_bluerov_compare
```

Results:

```text
PASS: real controller horizontal-scope outputs match
PASS: horizontal-only staged PID outputs match
PASS: Phase 2 vertical behavior and horizontal regression checks passed
PASS: Phase 2 real controller comparison passed with 3 expected difference(s)
```
