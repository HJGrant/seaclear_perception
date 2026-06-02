# DVL Decode Commands

このタスクで実際に使った主要コマンドを残す。すべて WSL + Docker 範囲で閉じている。

## 1. official package availability

```bash
docker run --rm acb2bd610943 /bin/bash -lc \
  'apt-cache policy ros-humble-marine-acoustic-msgs || true'
```

## 2. install official package

```bash
docker run --rm acb2bd610943 /bin/bash -lc \
  'apt-get update -o Acquire::AllowReleaseInfoChange::Origin=true -o Acquire::AllowReleaseInfoChange::Label=true && \
   apt-get install -y ros-humble-marine-acoustic-msgs'
```

## 3. inspect official interface

```bash
docker run --rm acb2bd610943 /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && \
   ros2 interface show marine_acoustic_msgs/msg/Dvl'
```

## 4. direct DB decode + replay subscriber probe

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws \
  -w /ws \
  acb2bd610943 \
  /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && \
   apt-get update -o Acquire::AllowReleaseInfoChange::Origin=true -o Acquire::AllowReleaseInfoChange::Label=true >/dev/null && \
   apt-get install -y ros-humble-marine-acoustic-msgs >/dev/null && \
   python3 /ws/artifacts/20260423/wsl_only_dvl_and_tracker_validation/dvl_decode_probe.py \
   > /ws/artifacts/20260423/wsl_only_dvl_and_tracker_validation/dvl_decode_logs/dvl_decode_probe.log'
```

## 5. controller replay with official DVL package

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws \
  -w /ws \
  acb2bd610943 \
  /bin/bash -lc \
  'set -e
   apt-get update -o Acquire::AllowReleaseInfoChange::Origin=true -o Acquire::AllowReleaseInfoChange::Label=true >/dev/null
   apt-get install -y ros-humble-marine-acoustic-msgs >/dev/null
   source /opt/ros/humble/setup.bash
   colcon build --packages-select \
     state_estimation \
     sc2_visual_servo_interfaces \
     sc2_visual_servo_perception \
     sc2_visual_servo_controller \
     sc2_visual_servo_synth \
     sc2_visual_servo_bringup
   source /ws/install/setup.bash
   python3 /ws/artifacts/20260423/visual_servo_production_bringup_gate/production_bringup_probe.py \
   > /ws/artifacts/20260423/wsl_only_dvl_and_tracker_validation/dvl_decode_logs/controller_with_official_dvl.log'
```

## 6. bag inventory

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws \
  -w /ws \
  acb2bd610943 \
  /bin/bash -lc \
  'source /opt/ros/humble/setup.bash && \
   apt-get update -o Acquire::AllowReleaseInfoChange::Origin=true -o Acquire::AllowReleaseInfoChange::Label=true >/dev/null && \
   apt-get install -y ros-humble-marine-acoustic-msgs python3-matplotlib python3-yaml >/dev/null && \
   python3 /ws/artifacts/20260423/wsl_only_dvl_and_tracker_validation/analyze_bags.py'
```

## 7. synthetic selector validation

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws \
  -w /ws \
  acb2bd610943 \
  /bin/bash -lc \
  'apt-get update -o Acquire::AllowReleaseInfoChange::Origin=true -o Acquire::AllowReleaseInfoChange::Label=true >/dev/null && \
   apt-get install -y python3-matplotlib >/dev/null && \
   PYTHONPATH=/ws/src/sc2_visual_servo_perception \
   python3 /ws/artifacts/20260423/wsl_only_dvl_and_tracker_validation/synthetic_selector_validation.py'
```

## 8. tracker core unit tests

```bash
docker run --rm \
  -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws \
  -w /ws \
  acb2bd610943 \
  /bin/bash -lc \
  'PYTHONPATH=/ws/src/sc2_visual_servo_perception \
   python3 -m unittest /ws/src/sc2_visual_servo_perception/test/test_tracker_selector_core.py'
```
