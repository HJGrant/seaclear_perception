# Code Review Findings

## 1. High: local `marine_acoustic_msgs` shim was ABI-incompatible with recorded bag

症状:

- `/waterlinked_dvl_driver/velocity_report` replay で Python subscriber / controller が kill

原因:

- local shim は UUV-style `Dvl.msg` だった
- recorded bag の real schema は少なくとも `speed_gnd` を含む別 schema

対応:

- shim package 削除
- controller を official package availability guard に変更

## 2. Medium: bag replay では `use_sim_time` が無いと `target_obs` が出ない

症状:

- `Dropping stale Detection2DArray age=...ms > max_age_ms=300`

原因:

- wall-clock と bag timestamp の差

対応:

- `object_detection_ibvs.launch.py` に `use_sim_time` 引数を追加
- probe は `use_sim_time:=true` + `--clock`

## 3. Medium: clean env の production bringup は依然 degraded mode

事実:

- detection stack required package 群が clean env に無い
- 今回の acceptance は observer/controller production path を degraded mode で成立させる形

判断:

- blocker 解消としては十分
- full OAK-D detection stack bringup は別タスク

