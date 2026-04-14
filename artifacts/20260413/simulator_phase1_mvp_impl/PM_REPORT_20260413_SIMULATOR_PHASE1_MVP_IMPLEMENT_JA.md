# PM Report: SIMULATOR_PHASE1_MVP_IMPLEMENT

## 概要
- branch: `simulator/bluerov2_humble_eval`
- head commit at report time: `814d21306251b60426695a6474a3cf502a4e1c1a`
- 今回から stale artifact 誤認防止のため、artifact を新規 `artifacts/20260413/simulator_phase1_mvp_impl/` に分離した。
- 上位 visual servoing I/F は `geometry_msgs/msg/TwistStamped` に固定した。
- `direct_velocity_backend` は debug 用、`dynamic_velocity_backend` を正式評価用 backend として分離した。

## 実装結果
- `src/sc2_visual_servo_sim/` を新規追加し、Phase 1 用 simulator-native backend, camera, frame, detector, launch を実装した。
- `src/sc2_visual_servo_controller/.../visual_servo_controller_node_implementation.py`
  - `start_enabled` parameter を追加した。
  - staged PID の vertical command 欠落を補完した。
- `src/sc2_visual_servo_synth/.../synthetic_bbox_publisher_node.py`
  - `/best_detection` publish を追加した。
- launch 一式:
  - `simulator_phase1_mvp.launch.py`
  - `simulator_phase1_dynamic.launch.py`
  - `simulator_phase1_direct.launch.py`
  - `simulator_phase1_actual_detector.launch.py`
- launch test skeleton:
  - `test_phase1_dynamic_launch.py`
  - `test_phase1_actual_detector_launch.py`

## backend 設計
- visual servoing/controller は `/sc2/visual_servo/cmd_vel_body` のみ publish。
- `direct_velocity_backend`
  - `TwistStamped` を直接積分して `/sim/odom` を生成。
  - frame/sign 切り分け用。
- `dynamic_velocity_backend`
  - `TwistStamped` を内部追従器で平滑化し `/sim/odom` を生成。
  - 同時に `/sim/backend/wrench` を publish。
  - まだ Gazebo thruster plugin 直結ではなく、Phase 1 の simulator-native 近似 backend。

## `auv_control` / `thruster_manager` 流用状況
- 事前調査で `auv_control` / `thruster_manager` は将来 backend 候補として妥当と判断済み。
- ただし今回は ROS runtime 不在のため、実接続・remap 実証までは未了。
- 今回の `dynamic_velocity_backend` は topic 契約を残した近似実装であり、次段で controller 内側を `auv_control` 系に差し替える方針。

## 再現手順
- build 想定:
  - `colcon build --packages-select sc2_visual_servo_sim sc2_visual_servo_controller sc2_visual_servo_perception sc2_visual_servo_synth`
- Phase 1A:
  - `ros2 launch sc2_visual_servo_sim simulator_phase1_dynamic.launch.py`
- Phase 1B:
  - `ros2 launch sc2_visual_servo_sim simulator_phase1_actual_detector.launch.py`
- debug backend:
  - `ros2 launch sc2_visual_servo_sim simulator_phase1_direct.launch.py`

## 検証結果
- 実施済み
  - `python3 -m compileall` による Python syntax check
  - launch ファイル syntax check
- 未実施
  - `ros2 launch` 実行
  - `ros2 topic echo/hz`
  - rosbag 採取
  - target center 誤差減少の runtime 実測
- 未実施理由
  - 現 shell 環境に `colcon`, `rclpy`, ROS 2 message/runtime が存在しない

## 既知の制約
- world は Gazebo 実 world ではなく node 内生成の軽量 simulator 表現。
- screenshot は artifact 用に構成概念図を格納したが、実ランタイム capture ではない。
- acceptance criteria の runtime 系は未完了で、今回の status は「実装完了、環境起因で動作実証待ち」。

## 次タスク候補
- ROS 2 Humble 環境で launch test と topic/hz/rosbag 検証を実施。
- `dynamic_velocity_backend` を `auv_control` / `thruster_manager` topic 契約へ置換。
- Gazebo seabed world と lighting sweep を追加。
- detector の bbox jitter を含む Phase 1B 評価表を更新。
