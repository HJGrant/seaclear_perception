# depthai_ros_driver crash report template (OAK-D PoE)

## Summary
- Symptom:
- Frequency:
- First known good config:
- First known bad config:

## Environment
- Device: OAK-D-PoE model:
- Host: Jetson Orin (JetPack / L4T):
- ROS 2 distro:
- `depthai_ros_driver` commit:
- `depthai-core` / firmware version (if available):
- Run mode: native / container

## Minimal reproduction

### 1) Config file
- Attach YAML (minimal diff only)

### 2) Launch command
```bash
ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py \
  params_file:=<yaml> \
  enable_vslam:=false
```

### 3) Trigger condition
- Time to failure:
- Required load:
- Whether it reproduces with `enable_vslam:=false`:

## Observed logs
- Driver log excerpt (`X_LINK_ERROR`, `ping was missed`, `No Data`, `double free`, `bad_alloc`)
- `dmesg` excerpt around failure
- `ip -s link` / `ethtool -S` counters around failure

## Expected behavior
- Stable stream at target FPS (e.g. 30 Hz) without process crash.

## Artifacts
- Attach `logs/oakd_diag/<run_id>/launch.log`
- Attach `logs/oakd_diag/<run_id>/events.log`
- Attach `logs/oakd_diag/<run_id>/snapshots/*.log`
