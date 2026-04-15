# Test Commands

Date: 2026-04-15

## Container Availability

```bash
docker ps --format '{{.Names}} {{.Image}} {{.Status}}'
docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}'
docker images --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.Size}}'
```

Observed usable image:

```text
isaac_ros_dev-x86_64:latest
```

## ROS Import Check

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/workspaces/oakd_isaac_ros_lrz \
  -w /workspaces/oakd_isaac_ros_lrz \
  isaac_ros_dev-x86_64:latest \
  bash -lc 'source /opt/ros/humble/setup.bash && python3 - <<'"'"'PY'"'"'
import geometry_msgs, rclpy, cv_bridge
print("ROS imports OK")
PY'
```

## Target Point Estimator Tests

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/workspaces/oakd_isaac_ros_lrz \
  -w /workspaces/oakd_isaac_ros_lrz \
  isaac_ros_dev-x86_64:latest \
  bash -lc 'source /opt/ros/humble/setup.bash &&
    export PYTHONPATH=/workspaces/oakd_isaac_ros_lrz/src/sc2_visual_servo_perception:$PYTHONPATH &&
    python3 -m pytest -q src/sc2_visual_servo_perception/test/test_target_point_estimator_integration.py'
```

## Related Tests Without Generated Interfaces

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

## Controller Compare

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase1.py
uv run python tools/compare_staged_pid_horizontal_behavior.py
uv run python tools/compare_visual_servo_controller_nodes_phase2.py \
  --out-dir artifacts/20260415/perception_ros_test_and_dvl_fallback_01/phase2_baseline_compare
uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py \
  --out-dir artifacts/20260415/perception_ros_test_and_dvl_fallback_01/phase2_bluerov_compare
```
