# SIMULATOR_PHASE1_MVP_IMPLEMENT

## 目的
- `CentraleNantesROV/bluerov2` 系 simulator へ接続できる Phase 1 MVP の上位 stack を、この repository 内で先行実装する。
- 上位 visual servoing の最終出力 I/F は `geometry_msgs/msg/TwistStamped` に固定する。
- Phase 1 は MAVROS 非依存とし、backend 側で vehicle 速度追従を担う。

## 今回実装した範囲
- `sc2_visual_servo_sim` package を追加し、Phase 1 用の simulator-native node 群を実装した。
- downward RGB + depth を供給する `sim_downward_camera_node` を追加した。
- `direct_velocity_backend` と `dynamic_velocity_backend` を分離実装した。
- `odom -> base_link -> down_camera_link -> down_camera_optical_frame` を出す `sim_static_frames_node` を追加した。
- `odom` から body velocity を再構成する `odom_to_body_twist_node` を追加した。
- Phase 1A 用の synthetic detector 経路と、Phase 1B 用の rendered RGB detector 経路を launch 引数で切り替えられるようにした。
- `visual_servo_controller_node` に `start_enabled` parameter を追加し、launch から即時閉ループ起動できるようにした。
- staged PID の vertical phase 実装欠落を補完し、`cmd_vel_body` を継続 publish できる状態へ修正した。
- `synthetic_bbox_publisher_node` に `/best_detection` publish を追加し、`target_point_estimator_node` を既存のまま通せるようにした。

## 実装要点

### 1. 上位 I/F 固定
- controller 出力 topic は `/sc2/visual_servo/cmd_vel_body` に固定した。
- 上位側は raw thruster topic, RC override, MAVROS 固有 topic を publish しない。
- backend 側が `TwistStamped` を受け、vehicle motion へ反映する。

### 2. backend 分離
- `direct_velocity_backend`
  - `TwistStamped` をそのまま近似積分して `/sim/odom` を生成する。
  - frame/sign 確認用の debug backend。
- `dynamic_velocity_backend`
  - `TwistStamped` を 1 次遅れと加速度制限つきで追従させる。
  - `/sim/odom` に加えて `/sim/backend/wrench` も publish する。
  - 正式評価用 backend の位置づけ。

### 3. センサ構成
- 正式案は downward RGB + aligned depth。
- `target_point_estimator_node` は `/best_detection`, `/sim/down/depth/image_raw`, `/sim/down/camera_info` を受ける既存構成を維持した。
- Phase 1B 用に rendered RGB から bbox を作る `rgb_target_detector_node` を追加した。

### 4. frame 契約
- `base_link`: REP-103 body frame を前提に `+x forward`, `+y left`, `+z up`。
- `down_camera_optical_frame` は下向き camera 用に `x_opt=-y_body`, `y_opt=-x_body`, `z_opt=-z_body` とした。
- image 誤差から body 速度への簡易写像は current staged PID 実装に合わせて
  - `u` 右ずれ増加 -> `cmd_vy_body < 0`
  - `v` 下ずれ増加 -> `cmd_vx_body > 0`
  - target depth 増加 -> `cmd_vz_body < 0`
  としている。

## launch
- 主 launch: `sc2_visual_servo_sim/launch/simulator_phase1_mvp.launch.py`
- ラッパ:
  - `simulator_phase1_dynamic.launch.py`
  - `simulator_phase1_direct.launch.py`
  - `simulator_phase1_actual_detector.launch.py`

## Phase 分離
- Phase 1A
  - `detector_mode:=synthetic`
  - synthetic bbox で loop 接続確認
- Phase 1B
  - `detector_mode:=actual`
  - rendered RGB から bbox を生成して loop 接続確認

## 既知の制約
- この shell 環境には `colcon`, `rclpy`, ROS message packages がないため、`ros2 launch` 実行確認までは未実施。
- よって今回は静的実装、syntax compile、launch/test ファイル整備、再現手順、artifact 生成までを完了範囲とした。
- `dynamic_velocity_backend` は simulator-native 近似 backend であり、`auv_control` / `thruster_manager` 実装の直接組み込みまでは今回未着手。

## 次段候補
- ROS 2 環境上で `launch_testing` 実行。
- `dynamic_velocity_backend` を `auv_control` / `thruster_manager` topic 契約へ差し替え。
- Gazebo world / seabed target / lighting sweep の実ランタイム検証。
- Phase 1B detector を実 detector launch に接続し、bbox jitter 評価へ進める。
