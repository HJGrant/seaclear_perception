# Algorithm As Implemented

## 対象

この文書は Task A/B 後の selector continuity と `REACQUIRE` policy を含めた実装アルゴリズムを記述する。

## Association

Association は前回 rework と同じ。

```text
stage 1:
  high-confidence detections を active tracks に greedy assign

stage 2:
  stage 1 で unmatched の active tracks に low-confidence detections を greedy assign

unmatched high-confidence detections:
  new TENTATIVE track を作る
```

Active tracks は `LOST` / `ESCALATE` 以外である。したがって `HOLD` は association 対象に残る。

## Selection Score

各 candidate track の base score:

```text
freshness = clamp(1 - (now - last_match_time) / t_stale, 0, 1)

state_score =
  3 if CONFIRMED and freshness > 0
  2 if HOLD and freshness > 0
  1 if TENTATIVE
  0 otherwise

hit_score = min(hit_count, 100) * 0.001

base_score = 10 * state_score + freshness + confidence_ema + hit_score
```

状態 rank が支配的になるよう `state_score` に 10 を掛けている。confidence/freshness の微差だけで state rank を逆転させないためである。

## Selector Continuity

採用方式: bonus + switch margin.

Pseudo-code:

```text
best = argmax(base_score over candidates)

previous = candidate with track_id == previous_selected_track_id

if previous is not CONFIRMED or freshness(previous) <= 0:
  return best

previous_effective = base_score(previous) + selector_selected_track_bonus
switch_threshold = previous_effective + selector_switch_margin
best_other = highest base_score candidate excluding previous

if best_other exists and base_score(best_other) > switch_threshold:
  return best_other

return previous
```

これにより、前回 selected だった confirmed/fresh track は微差では維持される。他候補が明確に良い場合だけ切り替わる。

## REACQUIRE

明示的な `REACQUIRE` state は実装していない。現実装では `HOLD` が passive `REACQUIRE` を兼ねる。

```text
CONFIRMED + miss -> HOLD
HOLD + high-confidence match -> CONFIRMED
HOLD + low-confidence recovery match -> CONFIRMED
HOLD + miss_count > hold_max_misses -> LOST
LOST / ESCALATE -> active association から除外
new high-confidence detection after LOST/ESCALATE -> new track_id
```

したがって、miss budget 内の再取得では track id continuity が残る。`LOST` / `ESCALATE` 後は new track 扱いとなり、track id continuity は失われる。
