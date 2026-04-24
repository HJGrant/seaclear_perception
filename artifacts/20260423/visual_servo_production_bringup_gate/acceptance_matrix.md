# Acceptance Matrix

| 条件 | 判定 | 根拠 | 環境 |
|---|---|---|---|
| 3. clean env で build/install が通る | 達成 | 6 package build/install 成功 | docker `acb2bd610943` |
| 4. launch が通る | 達成 | `timeout 8s ros2 launch ...` で observer/controller 起動 | docker `acb2bd610943` |
| 5. `/sc2/visual_servo/target_obs` が live で publish される | 達成 | production probe `obs_count=414` | docker + rosbag2 replay |
| 6. `selected_valid=false` 時に visual correction が止まり、HOLD maintain が動く | 条件付き達成 | `first_hold_status=...HOLD_MAINTAIN_VELOCITY_DAMPING_NO_DEPTH` と `first_nonzero_hold_cmd` を確認。depth は clean env で canonical DVL package 不在 | docker + rosbag2 replay + probe velocity bridge |
| 7. 複数 detection 下で selected track が不自然に飛びすぎない | 条件付き達成 | `valid_track_ids=[1,2,3,4,6]`, `transition_count=4` / `valid_obs_count=145`。continuity は観測できるが bag 1 本のみ | docker + rosbag2 replay |
| 8. status / target_locked / command topics が説明可能 | 達成 | `status_count=179`, `target_locked_count=160`, `cmd_count=160`, first hold status / nonzero hold cmd を採取 | docker + rosbag2 replay |

## 条件付き達成の理由

- 条件 6: HOLD path 自体は production path で確認できたが、`/waterlinked_dvl_driver/velocity_report` の canonical DVL depth は clean env で decode 不可
- 条件 7: one-bag validation であり full field acceptance ではない

