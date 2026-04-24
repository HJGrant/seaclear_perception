# Bag Recommendation

結論: multi-object 検証に使える bag は **yes**。

最有力は `rosbag2_2026_04_22-15_22_39`。

- 理由:
  - `two_plus_frames=423`
  - `same_class_two_plus_frames=423`
  - `longest_two_plus_run_sec=0.429`
  - `longest_run_window_sec=22.087->22.515`
  - `near_center_competitor_frames=2`
  - `near_size_competitor_frames=0`

候補区間: `22.087s` から `22.515s`。

補足: bag だけで stress を十分に掛けるには弱いので、synthetic validation を併用する。
