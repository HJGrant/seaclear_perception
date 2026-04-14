# Environment Audit

## Host (WSL) check
- `docker`: not available
- `ros2`, `colcon`, `gz`: not available
- `/opt/ros/humble`: not present

## Impact
- Local-first runtime validation could not be executed in this shell.
- Must run on Docker-enabled host or Orin container.
