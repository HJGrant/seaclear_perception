# Bringup Blocker Analysis

## 1. detection stack hard dependency

`object_detection_ibvs.launch.py` は当初 `oakd_isaac_ros` を前提にしていた。clean env では `oakd_isaac_ros` とその依存 package 群が無いため、bringup 全体が launch 前に停止していた。

今回の判断:

- production launch file 自体は残す
- detection stack だけ optional include にする
- observer/controller は degraded mode で起動可能にする

実装:

- `launch_detection_stack:=auto|true|false`
- required package 存在チェック
- missing package を launch log に明示

これにより clean env でも `object_detection_ibvs.launch.py` は起動可能になった。

## 2. bag replay と stale detection

`target_observer_node` は `Detection2DArray.header.stamp` と現在時刻との差を `max_age_ms` で評価する。recorded bag を wall-clock のまま再生すると、約 1 日分の時刻差が乗り stale drop になる。

対処:

- launch に `use_sim_time` を追加
- bag replay probe を `use_sim_time:=true` + `ros2 bag play --clock` に変更

これで production path 上の `/detections_output` をそのまま replay して `target_obs` publish を確認できた。

## 3. `/waterlinked_dvl_driver/velocity_report` の false compatibility

clean env で `marine_acoustic_msgs` が無いため、一度 `src/marine_acoustic_msgs` を local 追加した。しかしこの message 定義は bag 実 schema と一致せず、subscriber process が kill された。

確証:

- bag metadata は `marine_acoustic_msgs/msg/Dvl` を示す
- repo 内既存 subscriber は `msg.speed_gnd` を参照しており、local shim にはその field が無い
- minimal subscriber 再現で `exit 137`

最終対処:

- local shim package は削除
- controller は official `marine_acoustic_msgs.msg.Dvl` が import できる環境だけ DVL subscription を作る
- clean env では warning を出して no-DVL-depth degraded mode で継続する

## 4. launch_foxglove:=false でも残る blocker

`launch_foxglove:=false` は foxglove bridge node を止めるだけで、clean env 不足 package 問題そのものは解消しない。したがって detection stack optional 化は必要だった。

