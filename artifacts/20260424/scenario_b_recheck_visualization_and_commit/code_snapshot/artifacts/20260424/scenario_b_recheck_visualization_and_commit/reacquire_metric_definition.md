# Reacquire Metric Definition

## hold_rematch_success

- invalid / HOLD span に入る直前の selected track を continuity 候補とする
- continuity が切れる前に、同じ `selected_track_id` へ valid 状態で戻ったら success

## true_reacquire_success

- invalid span 中に previous selected track が `LOST` または `ESCALATE` へ移行したら continuity cut とみなす
- その後、true target へ valid selection が戻ったら success
- synthetic では tag=`target` を ground truth として判定した

## Notes

- hold rematch は track table continuity が残っている戻り
- true reacquire は実質的に new track 再取得を含む戻り
- latency は invalid span 開始から success までの frame 数平均
