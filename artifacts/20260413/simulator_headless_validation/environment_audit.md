# Environment Audit

## Host (WSL) facts
- `docker` command: not available in current WSL distro.
- `/opt/ros/humble`: not present.
- `ros2`, `colcon`, `gz`: not available on host.

## Repo facts
- `deployment_env/docker` directory not found in this repo.
- Container launch scripts:
  - `my_ws/scripts/launch_container.sh` (Orin)
  - `my_ws/scripts/launch_container_x86_64.sh` (x86)
- FastDDS profile:
  - `config/fastdds_client_profile.xml` -> `10.1.1.200:11815`, domain 8.

## Impact
- Headless runtime checks must run inside the Orin container or a Docker-enabled host.
