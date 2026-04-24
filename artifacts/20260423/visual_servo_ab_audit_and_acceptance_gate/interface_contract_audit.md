# interface_contract_audit

## `/sc2/visual_servo/target_obs`

- 型: `sc2_visual_servo_interfaces/msg/TargetObservation`
- 定義: `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg:1-20`
- producer: `target_observer_node.py:287-325`
- consumer: `visual_servo_controller_node_implementation.py:884-910`
- 契約状態:
  - geometry: `u/v/w/h/depth_m/confidence/source`
  - selector state: `selected_valid/selected_track_id/track_state/reason_code/age_frames/miss_count/freshness`
  - controller 側は `selected_valid`, `track_state`, `reason_code` を受けて HOLD gate を掛ける
- 判定: 一致

## `/detections_output`

- 型: `vision_msgs/msg/Detection2DArray`
- consumer: `target_observer_node.py:131-143,328-385`
- 処理:
  - class filter
  - low/high confidence split
  - two-stage association
  - unmatched high-confidence detections から new track 生成
- 判定: 一致

## `/waterlinked_dvl_driver/velocity_report`

- launch 上の想定 topic: `object_detection_ibvs.launch.py:145`
- controller subscriber: `visual_servo_controller_node_implementation.py:252-259,851-854`
- clean env 実測:
  - `marine_acoustic_msgs` package 不在
  - fallback で `state_estimation.msg.DVL` を使用して harness runtime を通した
- 判定: clean env では fallback により動作、実機型整合は要再確認

## `/mavros/local_position/velocity_body`

- 型: `geometry_msgs/msg/TwistStamped`
- consumer: `visual_servo_controller_node_implementation.py:875-882`
- HOLD maintain で XY/yaw damping に使用
- 判定: 一致

## `/sc2/visual_servo/status`

- 型: `std_msgs/msg/String`
- producer: controller
- live 実測:
  - `HOLD_SELECTED_INVALID_HOLD_hold_no_match_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL`
- 判定: 一致

## `/sc2/velocity_controller_node/outer/console_signals`

- 型: `state_estimation/msg/Vector6`
- producer: controller command adapter
- live 実測:
  - HOLD 中に非ゼロ `vx/vy/vz/yaw` を観測
- 判定: 一致

