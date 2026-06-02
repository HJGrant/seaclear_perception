# Selector Continuity Design

## Problem

複数の `CONFIRMED` track が存在すると、confidence EMA や freshness の微差で selected track が frame 間で揺れる可能性がある。これは controller にとって target jump として見える。

## Adopted Method

bonus + switch margin 方式を採用した。

## Score

```text
base_score =
  10 * state_score
  + freshness
  + confidence_ema
  + min(hit_count, 100) * 0.001
```

`state_score`:

```text
CONFIRMED and fresh -> 3
HOLD and fresh      -> 2
TENTATIVE           -> 1
otherwise           -> 0
```

## Continuity Logic

```text
if previous_selected_track is not found:
  select highest base_score

if previous_selected_track is not CONFIRMED/fresh:
  select highest base_score

previous_effective =
  base_score(previous_selected_track)
  + selector_selected_track_bonus

switch_threshold =
  previous_effective
  + selector_switch_margin

best_other = highest base_score candidate except previous

if base_score(best_other) > switch_threshold:
  select best_other
else:
  keep previous_selected_track
```

## Parameters

- `selector_selected_track_bonus`: previous selected confirmed/fresh track に加える bonus。
- `selector_switch_margin`: 他 track が切り替えに必要な追加 margin。

## Expected Behavior

- 微差の score 変動では selected track id を維持する。
- 明確に良い track が出たときは切り替える。
- previous selected track が stale / non-confirmed / lost / escalated の場合は bias しない。
