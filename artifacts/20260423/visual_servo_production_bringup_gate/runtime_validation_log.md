# Runtime Validation Log

## Build / install

clean env docker で以下 6 package を build/install:

- `state_estimation`
- `sc2_visual_servo_interfaces`
- `sc2_visual_servo_perception`
- `sc2_visual_servo_controller`
- `sc2_visual_servo_synth`
- `sc2_visual_servo_bringup`

結果: 成功

## Launch smoke test

コマンド:

```bash
timeout 8s ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false
```

結果:

- detection stack unavailable warning を出しつつ起動
- `target_observer_node` started
- `visual_servo_controller_node` started
- controller は `marine_acoustic_msgs.msg.Dvl is unavailable` warning を出して degraded mode 継続

## DVL negative reproduction

local shim `marine_acoustic_msgs` を使った状態では、`/waterlinked_dvl_driver/velocity_report` replay で subscriber が kill された。これをもって shim を撤回した。

## Production probe

probe 条件:

- launch: `launch_foxglove:=false use_sim_time:=true`
- bag topics:
  - `/detections_output`
  - `/sc2/imu/imu`
  - `/sc2/odometry_ekf`
  - `/sc2/dvl/dvl`
  - `/waterlinked_dvl_driver/velocity_report`
- bag player は `--clock`
- `/sc2/dvl/dvl` を probe 内で `/mavros/local_position/velocity_body` へ bridge

結果:

- `obs_count=414`
- `valid_obs_count=145`
- `invalid_obs_count=269`
- `hold_obs_count=74`
- `valid_track_ids=[1, 2, 3, 4, 6]`
- `transition_count=4`
- `first_hold={'selected_valid': False, 'track_state': 'HOLD', 'reason_code': 'hold_no_match', ...}`
- `first_hold_status=HOLD_SELECTED_INVALID_HOLD_hold_no_match_HOLD_MAINTAIN_VELOCITY_DAMPING_NO_DEPTH`
- `first_nonzero_hold_cmd={'vx': -0.005156..., 'vy': 0.000110..., 'vz': 0.0, 'yaw': -0.149889...}`

補足:

- bag player は `/waterlinked_dvl_driver/velocity_report` を `package 'marine_acoustic_msgs' not found` で ignore
- それでも observer/controller production path 自体は成立
- HOLD maintain の非ゼロ command は `/sc2/dvl/dvl` 由来速度 bridge により確認

