# Commands Executed

主要コマンドのみ記録する。探索用の `sed` / `rg` / `nl` は省略せず必要最小限で残した。

1. clean env build

```bash
docker run --rm -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws -w /ws acb2bd610943 bash -lc 'source /opt/ros/humble/setup.bash && rm -rf build install log && colcon build --base-paths src --packages-select state_estimation sc2_visual_servo_interfaces sc2_visual_servo_perception sc2_visual_servo_controller sc2_visual_servo_synth sc2_visual_servo_bringup --cmake-args -G Ninja --event-handlers console_direct+'
```

結果:

- `state_estimation`
- `sc2_visual_servo_interfaces`
- `sc2_visual_servo_perception`
- `sc2_visual_servo_controller`
- `sc2_visual_servo_synth`
- `sc2_visual_servo_bringup`

の 6 package build/install 成功

2. production bringup launch smoke test

```bash
docker run --rm -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws -w /ws acb2bd610943 bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 8s ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false'
```

結果:

- exit code `124` は `timeout 8s` による想定終了
- observer/controller 起動を確認
- detection stack unavailable を明示ログ

3. DVL-only negative reproduction before shim removal

```bash
docker run --rm -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws -w /ws acb2bd610943 /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && python3 - <<\"PY\" ... subscribe marine_acoustic_msgs.msg.Dvl and ros2 bag play /waterlinked_dvl_driver/velocity_report ... PY'
```

結果:

- Python subscriber process が `exit 137` で kill
- local compatibility `marine_acoustic_msgs` が bag 実 message schema と不一致であることを確認

4. production bringup probe

```bash
docker run --rm -v /home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz:/ws -w /ws acb2bd610943 bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && python3 /ws/artifacts/20260423/visual_servo_production_bringup_gate/production_bringup_probe.py'
```

結果要約:

- `obs_count=414`
- `valid_obs_count=145`
- `hold_obs_count=74`
- `transition_count=4`
- `first_hold_status=HOLD_SELECTED_INVALID_HOLD_hold_no_match_HOLD_MAINTAIN_VELOCITY_DAMPING_NO_DEPTH`
- `first_nonzero_hold_cmd={'vx': -0.005156..., 'vy': 0.000110..., 'vz': 0.0, 'yaw': -0.149889...}`

