# DVL Bag Presence Audit

## Scope

WSL + Docker だけで、手元 rosbag に `/waterlinked_dvl_driver/velocity_report` が入っているかを metadata 根拠つきで棚卸しした。

## Result Table

| bag | DVL topic present | recorded type | message_count | note |
|---|---|---|---:|---|
| `rosbag2_2026_04_22-15_22_39` | yes | `marine_acoustic_msgs/msg/Dvl` | 144 | `/detections_output` もあり |
| `rosbag2_2026_04_22-16_47_57-diving-noimage` | yes | `marine_acoustic_msgs/msg/Dvl` | 78 | `/detections_output` もあり |
| `rosbag2_2026_04_10-17_38_57` | no | - | 0 | `/best_detection` のみ |
| `src/deployment_env/assets/rosbag2_2026_04_13-17_56_18` | no | - | 0 | `/best_detection` のみ |

## Metadata Evidence

- [rosbag2_2026_04_22-15_22_39/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/rosbag2_2026_04_22-15_22_39/metadata.yaml:82)
  - `/waterlinked_dvl_driver/velocity_report`
  - `marine_acoustic_msgs/msg/Dvl`
  - `message_count: 144`
- [rosbag2_2026_04_22-16_47_57-diving-noimage/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/rosbag2_2026_04_22-16_47_57-diving-noimage/metadata.yaml:64)
  - `/waterlinked_dvl_driver/velocity_report`
  - `marine_acoustic_msgs/msg/Dvl`
  - `message_count: 78`
- [rosbag2_2026_04_10-17_38_57/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/rosbag2_2026_04_10-17_38_57/metadata.yaml:9)
  - `topics_with_message_count` 冒頭に `/best_detection` はあるが、DVL topic は無い
- [src/deployment_env/assets/rosbag2_2026_04_13-17_56_18/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/src/deployment_env/assets/rosbag2_2026_04_13-17_56_18/metadata.yaml:28)
  - `/best_detection` はあるが、DVL topic は無い

## Cut Classification

このタスクで区別すべき 4 分類に当てると、今回の bag 群は以下になる。

- `bag に topic が無い`
  - `rosbag2_2026_04_10-17_38_57`
  - `src/deployment_env/assets/rosbag2_2026_04_13-17_56_18`
- `bag には topic がある`
  - `rosbag2_2026_04_22-15_22_39`
  - `rosbag2_2026_04_22-16_47_57-diving-noimage`

この段階では bag 内容の有無だけを判定した。decode 可否と controller 影響は別紙に分離した。
