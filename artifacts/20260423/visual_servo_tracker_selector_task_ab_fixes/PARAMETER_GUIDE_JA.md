# Parameter Guide

## 新規 / 修正パラメータ

| component | parameter | default | tuning | effect |
|---|---:|---:|---|---|
| `target_observer_node` | `selector_switch_margin` | `0.05` | 要 tuning | previous selected track から他 track へ切り替えるために必要な score 差 |
| `target_observer_node` | `selector_selected_track_bonus` | `0.05` | 要 tuning | previous selected confirmed/fresh track に加える continuity bonus |

## 既存だが関連するパラメータ

| component | parameter | default | relation |
|---|---:|---:|---|
| `target_observer_node` | `confidence_ema_alpha` | `0.35` | selection score の `confidence_ema` を滑らかにする |
| `target_observer_node` | `t_stale_sec` | `1.0` | freshness と confirmed/fresh 判定に効く |
| `target_observer_node` | `hold_max_misses` | `12` | `HOLD` が passive reacquire として残る miss budget |
| `target_observer_node` | `lost_max_misses` | `30` | `ESCALATE` へ進む miss budget |

## Tuning Notes

- `selector_switch_margin` を大きくすると selected track は切り替わりにくい。
- `selector_selected_track_bonus` を大きくすると previous selected track の継続性が強くなる。
- 値を大きくしすぎると、実際に良い track への切替が遅れる。
- 値を小さくしすぎると、confidence/freshness の微差で ID switch が起きやすくなる。
