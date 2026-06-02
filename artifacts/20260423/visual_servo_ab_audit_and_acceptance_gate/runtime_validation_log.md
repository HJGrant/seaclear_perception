# runtime_validation_log

## 環境

- host: `ros2`, `colcon` なし
- clean env: docker image `acb2bd610943` (`nvcr.io/nvidia/isaac/ros:*`)
- ROS underlay: `/opt/ros/humble`

## build / install

### 1. repo root の `colcon build --packages-select ...` は最初に duplicate package で失敗

- 原因: `artifacts/**/repo_snapshot` が package discovery に入った
- 対応: snapshot/tree に `COLCON_IGNORE` を追加

### 2. `state_estimation` は Ninja generator で build 成功

- command: `colcon build --base-paths src --packages-select state_estimation --cmake-args -G Ninja`
- result: success

### 3. core packages は build 成功

- `sc2_visual_servo_interfaces`: success
- `sc2_visual_servo_perception`: success
- `sc2_visual_servo_controller`: success
- `sc2_visual_servo_bringup`: success (`--packages-select sc2_visual_servo_bringup`)

### 4. `packages-up-to sc2_visual_servo_bringup` は `oakd_isaac_ros` で失敗

- failure:
  - `foxglove_msgsConfig.cmake` not found
  - `oakd_isaac_ros` package build 不能

## production launch

- command: `ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false`
- result: fail
- direct cause: `PackageNotFoundError: package 'oakd_isaac_ros' not found`
- meaning: bringup package 自体は install 済みでも、required launch dependency が clean env に無く acceptance 未達

## live runtime harness

実施内容:

- `target_observer_node`
- `visual_servo_controller_node`
- synthetic multi-detection publisher
- synthetic DVL publisher
- synthetic body velocity publisher
- `enable_ibvs_control=true`

観測結果:

- `obs_count=5940`
- `valid_obs_count=5934`
- `hold_obs_count=4`
- continuity:
  - `phase1_ids=[1]`
  - `phase2_ids=[1]`
  - `phase3_ids=[2]`
- first valid observation:
  - `selected_valid=True`
  - `selected_track_id=1`
  - `track_state='CONFIRMED'`
- first hold observation:
  - `selected_valid=False`
  - `selected_track_id=2`
  - `track_state='HOLD'`
  - `reason_code='hold_no_match'`
- first runtime hold status:
  - `HOLD_SELECTED_INVALID_HOLD_hold_no_match_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL`
- first non-zero hold command:
  - `vx=-0.08`
  - `vy=0.04`
  - `vz=0.12`
  - `yaw=-0.045`

結論:

- `/sc2/visual_servo/target_obs` の live publish は確認
- `selected_valid=false` / stale 後の HOLD enter は確認
- HOLD maintain が zero command ではないことを確認
- continuity bias により small score change では `track_id=1` を維持し、大きい差分で `track_id=2` に切替わることを確認

