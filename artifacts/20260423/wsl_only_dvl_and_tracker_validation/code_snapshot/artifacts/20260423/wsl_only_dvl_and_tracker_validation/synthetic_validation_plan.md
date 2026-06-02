# Synthetic Validation Plan

- seed 固定の deterministic generator を使う
- Scenario A: 近接・再出現 target + 低頻度 competing candidate
- Scenario B: random clutter + target 近傍 clutter
- 評価指標:
  - `selected_track_id` switch count
  - `selected_valid=false` spans / frames
  - reacquire success rate
  - clutter false select count
  - HOLD / LOST / ESCALATE accumulation
