# Intended vs Implemented Diff

## Task A

Intended:

- clean environment build で import / message dependency 不足により破綻しないこと。

Implemented:

- controller `package.xml` に不足していた ROS package dependencies を追加した。
- perception / interfaces package は source と宣言の整合を確認し、追加修正不要と判断した。

Remaining:

- この host では `ros2` / `colcon` が無いため、実際の clean colcon build は未確認。

## Task B Selector Continuity

Intended:

- 複数 confirmed track が存在しても、confidence/freshness の微差で selected track が揺れないこと。
- 前回 selected track が有効なら継続し、明確な差がある場合だけ切り替えること。

Implemented:

- `selection_score()` による base score を導入。
- `previous_selected_track_id` に一致する confirmed/fresh track へ `selector_selected_track_bonus` を加える。
- 他 track へ切り替える条件を `base_score(other) > base_score(previous) + bonus + selector_switch_margin` とした。
- pure Python tests で「微差維持」と「margin 超過切替」を確認した。

Remaining:

- Hungarian / global assignment は未実装。
- appearance feature や semantic priority は未実装。

## REACQUIRE

Intended:

- 設計議論で出ていた `REACQUIRE` の扱いを曖昧にしないこと。

Implemented:

- Option 1 を採用し、明示的 `REACQUIRE` state は導入しない。
- 現実装では `HOLD` が passive reacquire 相当を兼ねると定義した。
- `HOLD` 中は low-confidence recovery / miss-budget 内再マッチで `CONFIRMED` に戻る。

Remaining:

- 明示的な `REACQUIRE` state, timeout, status code は未導入。
- `LOST` / `ESCALATE` 後の再取得は new track になり、track id continuity は失われる。
