# Feedback Adoption Note

## Adopted

- `switches=29` を軽く扱わない
  - 120 frame / 12 s で 29 switch は約 0.41 s ごとの切替であり、追従 continuity リスクとして Red 扱いに上げる
- synthetic / production parameter 差分を明示する
  - legacy synthetic と production-like を並列比較する
- reacquire 指標を分ける
  - `hold_rematch_success`
  - `true_reacquire_success`

## Partially Adopted

- Scenario B が実海中 clutter をどこまで代表するか
  - 代表性は未証明
  - ただし stress test としては十分に有用なので gate 判断には使う
- tuning だけでよいか
  - まず tuning gate を置く
  - Red の場合は score 設計や空間一貫性追加も候補に上げる

## Held / Not Claimed

- Scenario B の結果だけで production rejection を即断すること
  - そこまでは言わない
  - ただし実機前の暫定 gate では Red 扱いにする

## Updated Decision Flow

1. production-like Scenario B を再評価する
2. `valid_switches` と `clutter_false_selects` を優先指標にする
3. Yellow までは tuning 再試行
4. Red は設計変更 table を先に開く
5. 最終判断は bag / 実機で確認する
