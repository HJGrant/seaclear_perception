# Design Change Gate

## Temporary Gate

| gate | condition | action |
|---|---|---|
| Green | `switches <= 3`, `clutter_false_selects = 0`, no repeated `valid_switches` burst | keep tuning only |
| Yellow | `4 <= switches <= 9` or `clutter_false_selects = 1` | retune margin / bonus / gate and rerun |
| Red | `switches >= 10` or `clutter_false_selects >= 2` or repeated `valid_switches` | open design-change options table before real-world test |

## Current Placement

- `scenario_b` legacy: Red
- `scenario_b` production-like: Red if the rerun still exceeds the threshold
- `variable_bbox_b` production-like: treat as Red when it exceeds the same threshold
