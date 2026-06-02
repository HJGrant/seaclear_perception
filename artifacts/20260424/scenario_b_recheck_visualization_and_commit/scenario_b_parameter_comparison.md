# Scenario B Parameter Comparison

Scenario B を legacy synthetic 条件と production 寄り条件で比較した。

## Parameter Sets

- legacy synthetic
  - `n_confirm=3`
  - `hold_max_misses=6`
  - `lost_max_misses=12`
  - `gate_px=70`
  - `selector_switch_margin=0.05`
  - `selector_selected_track_bonus=0.05`
- production-like
  - `n_confirm=5`
  - `hold_max_misses=12`
  - `lost_max_misses=30`
  - `gate_px=80`
  - `selector_switch_margin=0.05`
  - `selector_selected_track_bonus=0.05`

差分が残る点:
- synthetic generator は bag/実機 detector の score 分布を再現していない
- low-confidence 閾値は candidate score 生成で間接的に与えており、observer node の全 filtering path をそのまま通してはいない

| scenario | config | switches | valid switches | false spans | false frames | clutter false selects | HOLD | LOST | ESCALATE | hold rematch | true reacquire |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `scenario_b` | `legacy_synth` | 6 | 6 | 41 | 58 | 12 | 49 | 85 | 81 | 39/40 | 0/0 |
| `scenario_b` | `production_like` | 0 | 0 | 1 | 120 | 0 | 0 | 39 | 27 | 0/0 | 0/0 |
| `variable_bbox_b` | `production_like` | 33 | 29 | 22 | 39 | 45 | 69 | 31 | 27 | 16/21 | 0/0 |
