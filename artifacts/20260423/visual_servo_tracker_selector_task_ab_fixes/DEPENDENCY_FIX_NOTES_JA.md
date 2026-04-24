# Dependency Fix Notes

## Controller Package

対象: `src/sc2_visual_servo_controller/package.xml`

追加した依存と理由:

| dependency | 対応する import / API | 防ぐ build failure |
|---|---|---|
| `std_srvs` | `from std_srvs.srv import SetBool` | service type 未宣言による package resolution failure |
| `sensor_msgs` | `from sensor_msgs.msg import CameraInfo` | CameraInfo message dependency missing |
| `vision_msgs` | `from vision_msgs.msg import Detection2D` | Detection2D message dependency missing |
| `marine_acoustic_msgs` | `from marine_acoustic_msgs.msg import Dvl` | DVL message dependency missing |
| `tf2_ros` | `import tf2_ros`, `TransformException`, `Buffer`, `TransformListener` | TF runtime package dependency missing |
| `tf2_geometry_msgs` | `import tf2_geometry_msgs` | geometry transform helper dependency missing |
| `state_estimation` | `from state_estimation.msg import Vector6` | custom Vector6 message package missing |

既存依存で妥当なもの:

| dependency | 対応 |
|---|---|
| `rclpy` | ROS Python node API |
| `std_msgs` | `String`, `Float64`, `Float64MultiArray`, `Bool` |
| `geometry_msgs` | `TwistStamped`, `PoseStamped`, `PointStamped`, `Vector3`, `Vector3Stamped`, `Twist` |
| `visualization_msgs` | debug visualization node |
| `sc2_visual_servo_interfaces` | `TargetObservation` |

`nav_msgs` は今回 controller package では追加していない。現在の controller source は `nav_msgs` を直接 import していないため。

## Perception Package

`src/sc2_visual_servo_perception/package.xml` は現 source と整合している。

| dependency | 対応 |
|---|---|
| `rclpy` | ROS Python node API |
| `std_msgs` | `Header`, `String` |
| `sensor_msgs` | `Image`, `Imu`, `CameraInfo` |
| `geometry_msgs` | `PoseStamped` |
| `nav_msgs` | `Odometry` |
| `vision_msgs` | `Detection2D`, `Detection2DArray`, hypothesis messages |
| `cv_bridge` | depth image conversion |
| `sc2_visual_servo_interfaces` | `TargetObservation` |

## Interfaces Package

`src/sc2_visual_servo_interfaces/package.xml` と `CMakeLists.txt` は `TargetObservation.msg` の `std_msgs/Header` に対して `std_msgs` を宣言済みであり、追加修正は不要。
