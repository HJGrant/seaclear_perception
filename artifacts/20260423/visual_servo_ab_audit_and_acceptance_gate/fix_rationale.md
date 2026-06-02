# fix_rationale

このタスク中に acceptance gate を通すために入れた追加修正は以下のみ。

## 1. `sc2_visual_servo_bringup/package.xml` に runtime 依存を追加

- 変更:
  - `ament_index_python`
  - `oakd_isaac_ros`
- 理由:
  - launch file は `ament_index_python.packages.get_package_share_directory` を import し、`oakd_isaac_ros` の share を参照する。
  - metadata が無いと launch contract と package 宣言が不整合。

## 2. controller の DVL import に clean env fallback を追加

- 変更:
  - `marine_acoustic_msgs.msg.Dvl` を優先
  - import 不可時は `state_estimation.msg.DVL` に fallback
- 理由:
  - clean container には `marine_acoustic_msgs` が無く、build は通っても Python node が即死した。
  - tracker/selector/HOLD gate の runtime acceptance を取るための最小修正。

## 3. artifact snapshot subtree に `COLCON_IGNORE` を追加

- 対象:
  - 既存 `artifacts/**/repo_snapshot`
  - 今回の `code_snapshot/`
- 理由:
  - repo root `colcon build` が artifact 中の duplicate package を拾って失敗したため。
  - source behavior ではなく build hygiene の修正。

