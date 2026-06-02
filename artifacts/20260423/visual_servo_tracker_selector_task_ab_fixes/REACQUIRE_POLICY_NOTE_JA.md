# REACQUIRE Policy Note

## Decision

今回、明示的な `REACQUIRE` state は追加しない。現実装では `HOLD` が `REACQUIRE` を兼ねる。

これは Task B の Option 1 である。

## Meaning

`HOLD` は passive reacquire state として扱う。

- controller には `selected_valid=false` が届き、visual correction は止まる。
- observer 側では track table に track が残る。
- `HOLD` track は association 対象に残る。
- high-confidence detection または low-confidence recovery が同じ track に割り当たると `CONFIRMED` に戻る。

## State Flow

```text
CONFIRMED
  miss
  -> HOLD

HOLD
  high-confidence match
  -> CONFIRMED

HOLD
  low-confidence recovery match
  -> CONFIRMED

HOLD
  miss_count > hold_max_misses
  -> LOST

active track
  miss_count >= lost_max_misses
  -> ESCALATE
```

## Boundary

`LOST` / `ESCALATE` は active association 対象から外れる。その後に target が見つかった場合、新しい `TENTATIVE` track として生成される。

このため:

- `HOLD` 内再取得では track id continuity が維持される。
- `LOST` / `ESCALATE` 後の再取得では track id continuity は失われる。

## Not Implemented

- explicit `REACQUIRE` state
- dedicated reacquire timeout
- dedicated reacquire reason code beyond existing `low_confidence_recovery` / `hold_no_match`
- reacquire-specific controller behavior
