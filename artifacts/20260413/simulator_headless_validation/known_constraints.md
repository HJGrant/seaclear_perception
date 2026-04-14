# Known Constraints

- WSL environment lacks Docker and ROS 2 runtime.
- `deployment_env/docker` is not present in this repository.
- `object_detection_ibvs.launch.py` publishes to MAVROS velocity topic, not `/sc2/visual_servo/cmd_vel_body`.
- `ros_gz_sim` / `ros_gz_bridge` availability not verified in this environment.
