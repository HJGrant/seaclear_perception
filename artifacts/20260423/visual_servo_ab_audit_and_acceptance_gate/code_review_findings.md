# code_review_findings

## 1. Blocker: production bringup launch は clean env で未達

- `object_detection_ibvs.launch.py` は import 時に `get_package_share_directory('oakd_isaac_ros')` を必須にしている (`src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py:35-47`)。
- clean container では `oakd_isaac_ros` が install されておらず、`ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false` は `PackageNotFoundError` で失敗した。
- さらに `packages-up-to sc2_visual_servo_bringup` build は `oakd_isaac_ros` の `foxglove_msgs` 欠落で止まる。これは tracker/selector/HOLD 本体ではなく detection pipeline 側の blocker。

影響:

- acceptance 条件 3, 4 は production bringup では未達。
- mainline merge は保留が妥当。

## 2. Runtime blocker was real: controller の DVL import は build では見えず、clean env で実行時に落ちた

- `visual_servo_controller_node_implementation.py` は元々 `marine_acoustic_msgs.msg.Dvl` を必須 import していた。
- clean container には `marine_acoustic_msgs` が無く、`ros2 run sc2_visual_servo_controller visual_servo_controller_node ...` は import error で即死した。
- このタスクで `marine_acoustic_msgs` 優先、無ければ `state_estimation.msg.DVL` に fallback する最小修正を追加した (`src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:46-50`)。

影響:

- core runtime acceptance を取るための blocker は解消した。
- 実機環境で実際に publish される DVL topic type は別途再確認が必要。

## 3. 受け入れ観点でのコード一致

- selector continuity は実装済みで、small score change では previous track を維持し、margin 超過時のみ切替 (`tracker_selector_core.py:279-330`)。
- `REACQUIRE` を独立 state にしない設計はコードと一致。`HOLD` track が再度 match すると `CONFIRMED` に戻る (`tracker_selector_core.py:115-148`)。
- observer は `selected_valid`, `selected_track_id`, `track_state`, `reason_code`, `freshness` を publish している (`target_observer_node.py:287-325`)。
- controller は invalid/stale 時に HOLD path へ入り、visual correction を継続しない (`visual_servo_controller_node_implementation.py:1167-1182`)。
- HOLD path は depth maintain + body velocity damping を使い zero command only ではない (`hold_maintain_core.py:38-49`, `visual_servo_controller_node_implementation.py:781-830`)。

## Open Questions

- 実運用の `/waterlinked_dvl_driver/velocity_report` が `marine_acoustic_msgs/Dvl` か `state_estimation/DVL` かは clean env では確認不能。
- launch に必要と指定された一部 PM/forensic 文書は repo 内に無かったため、ドキュメント trace は部分欠損。

