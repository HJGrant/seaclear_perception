# DVL Replay Decode Validation

## Summary

結論は `controller まで含めて DVL あり replay 検証が成立` である。前提は WSL + Docker の clean ROS 環境へ official `ros-humble-marine-acoustic-msgs` を導入すること。

## What Was Checked

1. official package が apt で取得可能か
2. `marine_acoustic_msgs/msg/Dvl` の official schema が読めるか
3. bag DB 内の CDR blob を official schema で deserialize できるか
4. `ros2 bag play` + subscriber で structured message として受け取れるか
5. controller が DVL subscribe を使い、`NO_DEPTH` 以外の HOLD status に入るか

## Evidence

### 1. official message package

Docker 内 `apt-cache policy ros-humble-marine-acoustic-msgs` で candidate が確認でき、install 後に `ros2 interface show marine_acoustic_msgs/msg/Dvl` が成功した。

### 2. direct decode from bag DB

[dvl_decode_logs/dvl_decode_probe.log](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/artifacts/20260423/wsl_only_dvl_and_tracker_validation/dvl_decode_logs/dvl_decode_probe.log:1) より:

- `rosbag2_2026_04_22-15_22_39`
  - `altitude=4.705516`
  - `speed_gnd=0.000016`
  - `course_gnd=1.671716`
  - `num_good_beams=4`
- `rosbag2_2026_04_22-16_47_57-diving-noimage`
  - `altitude=6.719603`
  - `speed_gnd=0.013746`
  - `course_gnd=-0.830999`
  - `num_good_beams=4`

これは「bag に topic はあるが message 定義不足で decode できない」状態ではなく、official schema で structured decode できていることを示す。

### 3. replay + subscriber

同ログで `subscriber_count=3` と `subscriber_sample ... altitude=...` が両 bag で確認できた。つまり `ros2 bag play` 経由でも subscriber が structured field を読める。

### 4. controller impact

[dvl_decode_logs/controller_with_official_dvl.log](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/artifacts/20260423/wsl_only_dvl_and_tracker_validation/dvl_decode_logs/controller_with_official_dvl.log:1) より:

- `obs_count=414`
- `valid_obs_count=145`
- `hold_obs_count=74`
- `first_hold_status=HOLD_SELECTED_INVALID_HOLD_hold_no_match_HOLD_MAINTAIN_VELOCITY_DAMPING_DEPTH_DVL`
- `first_nonzero_hold_cmd={'t': 1776971829.363389, 'vx': -0.2, 'vy': -0.2, 'vz': 0.0, 'yaw': -0.35}`

以前の `NO_DEPTH` degraded mode と違い、official DVL package ありの replay では HOLD status が `DEPTH_DVL` に変わっている。したがって controller 側 wiring 不足ではなく、主因は clean env に official message 定義が無かったことだったと切り分けられる。

## Cut Result

今回の切り分けは以下で確定した。

- `bag に topic が無い`
  - 一部 bag はこれに該当
- `bag には topic があるが、WSL+Docker に message 定義が無く読めない`
  - 過去の clean env 状態はこれに該当
- `message は読めるが、controller 側 wiring が不十分`
  - 今回の replay では該当しない
- `controller まで含めて DVL あり replay 検証が成立`
  - `rosbag2_2026_04_22-16_47_57-diving-noimage` で成立

## Limits

- 今回の成立は replay ベースであり、live 実機 sensor path の健全性そのものは検証対象外
- `first_nonzero_hold_cmd` の大きさ妥当性は別問題で、ここでは DVL decode / wiring 成立のみを結論化した
