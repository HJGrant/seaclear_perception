# oakd_isaac_ros

This is a package for interfacing the OAK-D PoE with the NVIDIA Isaac ROS packages. 

## Current branch status

This repository contains multiple parallel workflows. For the current
`dev/aoki_detector_integration` line, use these entry points as the operational
source of truth:

- integrated detector + VS entry: `src/oakd_isaac_ros/launch/oakd_system.launch.py`
- detector-only wrapper: `src/oakd_isaac_ros/launch/debug/object_detection.launch.py`
- detector core pipeline: `src/oakd_isaac_ros/launch/tools/detection.launch.py`
- current detector model default: `isaac_ros_assets/models/seaclear/v12s.onnx`
- current detector engine output default: `isaac_ros_assets/models/seaclear/v12s.plan`
  `v12s.plan` is not expected to be versioned in the repo; on a new Jetson / GPU
  runtime it may be generated on first launch from the ONNX model
- integrated `oakd_system.launch.py` reuses `debug/object_detection.launch.py` for
  camera + detector wiring, while overriding the camera params file to
  `config/oak_d/oakd_rgbd.yaml`

For runnable commands and current caveats, prefer `README_quickstart.md`.
The legacy nvBlox / VSLAM sections below are still useful reference material,
but they do not fully describe the current detector + visual-servo integration
workflow.

## Repository structure

This repository is organized as a ROS2 workspace with upstream submodules and SC2-specific packages.
The main locations you will edit are `src/` (ROS2 packages), `docs/`, and `scripts/`.

```text
oakd_isaac_ros/
|-- README.md
|-- README_quickstart.md
|-- pyproject.toml / uv.lock             # Python dependency management (uv)
|-- scripts/                              # Workspace helper scripts (build/test/bag)
|-- docs/                                 # Specifications, status, and investigation logs
|   |-- SEACLEAR2_SMART_GRAPPLE_VISUAL_SERVO_CHECKLIST_20260223.md
|   |-- SC2_VISUAL_SERVO_MVP_NODE_GRAPH_20260223.md
|   |-- PERCEPTION_TOPIC_MAP.md
|   |-- VS_BBOX_TARGET_POINT_INTEGRATION_20260225.md
|   |-- archive/2026Q1/                   # Frozen historical notes no longer used for routine updates
|   |-- oakd-vslam/                       # Lower-priority OAK-D/VSLAM debug and diagnostics
|   |   |-- PROJECT_STATUS_OAKD_VSLAM.md
|   |   |-- PIPELINE_HEALTHCHECK.md
|   |   L-- OAKD_VSLAM_DEBUG_NOTES.md
|   L-- reference/
|-- src/                                  # ROS2 packages (main implementation area)
|   |-- oakd_isaac_ros/                   # OAK-D + Isaac ROS integration (launch/config/C++ nodes)
|   |   |-- launch/                       # Main launch files (nvblox, visual_slam, detection)
|   |   |-- config/                       # Camera and nvblox configuration
|   |   |-- scripts/                      # Diagnostics and utility scripts
|   |   L-- src/                          # C++ node implementations (e.g., publish_mask.cpp)
|   |-- sc2_visual_servo_interfaces/      # SC2 custom interface messages
|   |   L-- msg/TargetObservation.msg
|   |-- sc2_visual_servo_perception/      # Detection2DArray -> TargetObservation conversion
|   |   |-- sc2_visual_servo_perception/
|   |   |   |-- target_observer_node.py
|   |   |   L-- conversion.py
|   |   L-- test/
|   |-- sc2_visual_servo_controller/      # Visual servo controller nodes
|   |   |-- sc2_visual_servo_controller/
|   |   |   |-- visual_servo_controller_node.py
|   |   |   |-- ibvs/controller_math.py   # IBVS core
|   |   |   |-- pbvs/mppi_core.py         # MPPI core (PBVS-oriented)
|   |   |   |-- csv_logger_node.py
|   |   |   |-- cmd_vel_to_mavros_bridge.py
|   |   |   |-- cmd_vel_to_mavlink_stub.py
|   |   |   L-- target_obs_publisher.py
|   |   L-- test/
|   |-- sc2_visual_servo_bringup/         # MVP bringup entrypoint
|   |   L-- launch/{mvp_sim,blue_rov_synth_vs}.launch.py
|   |-- sc2_visual_servo_synth/           # Synthetic Detection2D/TargetObservation generator
|   |   L-- sc2_visual_servo_synth/synthetic_bbox_publisher_node.py
|   |-- depthai-ros/                      # Luxonis DepthAI ROS (submodule)
|   |-- isaac_ros_common/                 # Isaac ROS common stack (submodule)
|   |-- isaac_ros_dnn_inference/          # TensorRT inference stack (submodule)
|   |-- isaac_ros_nvblox/                 # nvblox stack (submodule)
|   |-- isaac_ros_visual_slam/            # Visual SLAM stack (submodule)
|   L-- webrtc_ros/                       # WebRTC stack
|-- build/                                # colcon build artifacts (do not edit)
|-- install/                              # colcon install artifacts (do not edit)
L-- log/                                  # build/test logs (do not edit)
```

Notes:
- `src/sc2_visual_servo_*` contains the SC2 Smart Grapple Visual Servo MVP implementation.
- `src/oakd_isaac_ros` is the OAK-D + Isaac ROS integration layer (detection, mapping, VSLAM integration).
- `depthai-ros` and `isaac_ros_*` are upstream dependency implementations; most local work here is configuration and launch integration.
- `build/`, `install/`, and `log/` are auto-generated directories and are typically not edited directly.

## TODO
- [ ] Find a way to turn off the static TSDF layer and only compute the dynamic layer
- [ ] Interface the vision pipeline with the grasp detection pipeline


## Requirements
 - For x86_64: Ubuntu 22.04+ and CUDA 12.6+ 
 - For Jetson: Jetpack 6.1 or 6.2
 - DepthAI
 - Foxglove for visualization

Install DepthAI: 
```bash
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash && \
python3 -m pip install depthai
```

## Launching nvBlox with the OAK-D PoE
## Step 1 
Make sure you have the OAK-D connected, and give your machine a static IP in the same subnet as the camera (for example, `192.168.2.50` with netmask `255.255.255.0`). Verify that the OAK-D is detected by running:

```bash
ping 192.168.2.97
```

If you are having trouble with the OAK-D then go to troubleshooting. 

## Step 2
Clone the repo and its submodules (DepthAI ROS Driver, Isaac ROS Packages, ...) with the following command:

```bash
mkdir -p ~/workspaces/ && cd ~/workspaces/ && \
git clone --recurse-submodules https://gitlab.lrz.de/itr-seaclear/tum/sc2/development/oakd_isaac_ros.git
```
Copy the repository config file into the home-directory filename expected by `run_dev.sh`:
```bash
cd ~/workspaces/oakd_isaac_ros && \
cp .isaac_ros_common_config ~/.isaac_ros_common-config
```

## Step 3
Install Git LFS to pull large files
```bash
sudo apt-get install git-lfs && \
git lfs install --skip-repo
```

Run the following script to build and launch the Docker container:

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/run_dev.sh --isaac_ros_dev_dir ~/workspaces/oakd_isaac_ros/
```

## Step 4
Inside the container, set the ISAAC_ROS_WS variable and build the workspace:

```bash
export ISAAC_ROS_WS=/workspaces/oakd_isaac_ros/ && \
colcon build --merge-install
```

Note: the option `--merge-install` is important for deployment. 

## Step 5
After the build has completed, source the ROS2 environment and run the launch file to run nvBlox with the OAK-D PoE:

```bash
source install/setup.bash && \
ros2 launch oakd_isaac_ros oakd_nvblox.launch.py mode:='people_detection' use_rosbag:='false' voxel_size:='0.05'
```

Specifying the 'people_detection' mode will enable selective 3D reconstruction of objects in bounding box regions provided by the SeaClear YOLO model. Other mode options include 'static' and 'dynamic'. You can choose to run the pipeline on a pre-recorded rosbag file, or run the pipeline live with the OAK-D camera, by setting 'use_rosbag' to false/true respectively. Set the voxel size of the map by changing the value 'voxel_size'. The default value is 0.05, and reducing this value will increase the resolution of the map, with a decrease in rendering speed. 

## Step 6
Visualize the output of the algorithm in Foxglove. For this, display the static map and the dynamic map of the detected objects by subscribing to `/nvblox_node/color_layer` and `/nvblox_node/dynamic_occupancy_layer`, respectively. Display these topics in a 3D panel in Foxglove.


## Running OAK-D with Visual SLAM

This launch file starts the OAK-D camera, runs NVBlox (optional), and Isaac ROS Visual SLAM using the same camera topics for proper communication.

### 0. launch container
```bash
cd /mnt/nova_ssd/workspaces/oakd_isaac_ros
src/isaac_ros_common/scripts/run_dev.sh \
--isaac_ros_dev_dir /mnt/nova_ssd/workspaces/oakd_isaac_ros
# or
./src/isaac_ros_common/scripts/run_dev.sh --isaac_ros_dev_dir $(pwd)
```

**If you use WSL**
you need to run
`cp .isaac_ros_common_config ~/.isaac_ros_common-config` and launch Docker Desktop beforehand


### 1. Build the workspace
install uv
```
source /opt/ros/humble/setup.bash
cd /workspaces/oakd_isaac_ros

curl -LsSf https://astral.sh/uv/install.sh | sh
# or
# wget -qO- https://astral.sh/uv/install.sh | sh

export PATH="$HOME/.local/bin:$PATH"
hash -r

uv --version
```


From the root of your workspace:

```bash
cd /workspaces/oakd_isaac_ros
colcon build --merge-install
```

If this is your first build or you updated packages, make sure you build the workspace fully.

2. Source the workspace
```bash
source /workspaces/oakd_isaac_ros/install/setup.bash
```

3. Launch OAK-D + Visual SLAM
```bash
ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py
```

4. Optional arguments

`use_rosbag:=true` – Use a rosbag instead of the live OAK-D camera.

`container_name:=<name>` – Name of the composable container to use.

`log_level:=info` – Set ROS log level.

Example with rosbag playback:
```bash
ros2 launch oakd_isaac_ros oakd_visual_slam.launch.py use_rosbag:=true
```

### Debug/diagnostic workflow (OAK-D + VSLAM)

Use the staged camera YAMLs in `src/oakd_isaac_ros/config/oak_d/`:

- `oakd_stereo_only_30hz.yaml`
- `oakd_stereo_rect_30hz.yaml`
- `oakd_stereo_rect_synced_30hz.yaml`
- `oakd_rgb_only_30hz.yaml`
- `oakd_rgb_plus_stereo_30hz.yaml`

Run one-shot diagnostics and log collection:

```bash
src/oakd_isaac_ros/scripts/diag_oakd_run_and_monitor.sh \
  --params-file src/oakd_isaac_ros/config/oak_d/oakd_stereo_only_30hz.yaml \
  --enable-vslam false \
  --iface eno1 \
  --duration-sec 600
```

Detailed notes and troubleshooting playbook:

- `docs/oakd-vslam/OAKD_VSLAM_DEBUG_NOTES.md`
- `docs/oakd-vslam/upstream_issue_template_depthai_ros_driver.md`

5. Notes

Ensure the camera topics in the launch file match the OAK-D configuration (RGB and Depth topics).

Both NVBlox and Visual SLAM subscribe to the same camera topics, ensuring proper data flow.

## Building a Deployment Docker
If you would like to deploy this pipeline as a 1-click solution that launches a Docker container and automatically launches the correct ROS2 launch file, run the following command OUTSIDE of the Docker container:

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/docker_deploy.sh --base_image_key "x86_64.ros2_humble.oakd" --ros_ws ~/workspaces/oakd_isaac_ros --launch_package "oakd_isaac_ros" --launch_file "oakd_nvblox.launch.py" -n "<my_username>/nvblox_dynamics_oakd" 
```

For Jetson Devices run (not tested):

```bash
~/workspaces/oakd_isaac_ros/src/isaac_ros_common/scripts/docker_deploy.sh --base_image_key "aarch64.ros2_humble.oakd" --ros_ws ~/workspaces/isaac_ros-dev --launch_package "oakd_isaac_ros" --launch_file "oakd_nvblox.launch.py" -n "<my_username>/nvblox_dynamics_oakd_jetson_aarch64" 
```
Now you should see the image when you run `docker images` and you can then launch the pipeline by running:

```bash
docker run --rm -it --gpus all --network host <my_username>/nvblox_dynamics_oakd 
```
To run the deployment docker on Jetson use the following command: 
```bash
docker run --rm -it --runtime=nvidia --network host <my_username>/nvblox_dynamics_oakd_jetson_aarch64 
```

## Result 

[Watch the demo video](my_ws/assets/demo_vid.mp4)


## Troubleshooting

### Cannot Receive Data from OAK-D
Check basic functionality of the OAK-D by running ` python -m depthai_viewer `.

If you can't see the OAK-D, follow this [troubleshooting guide.](https://docs.luxonis.com/hardware/platform/deploy/poe-deployment-guide#connected-to-the-same-lan-via-2-interfaces-wifi-ethernet)

## SeaClear2 Visual Servo MVP (ROS2)

For the one-command visual-servo simulation MVP (no SLAM dependency), see:
- `README_quickstart.md`

Main launch command:
```bash
ros2 launch sc2_visual_servo_bringup mvp_sim.launch.py
```
