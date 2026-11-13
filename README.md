# oakd_isaac_ros
## Introduction

This package was developed by Hamish Grant and Nobuaki Aoki 



## Launching nvBlox with the OAK-D PoE
## Step 1 
Make sure you have the OAK-D connected, and give your machine a static ip such as ` 169.254.1.97` with netmask ` 255.255.0.0 `. Verfiy that the OAK-D is detected by running: 

```bash
ping 192.168.2.109
```

If you are having trouble with the OAK-D then go to troubleshooting. 

## Step 2
Clone the repo and it's submodules (DepthAI ROS Driver, Isaac ROS Packages, ...) with the following command: 

```bash
mkdir -p ~/workspaces/ && cd ~/workspaces/ && \
git clone --recurse-submodules https://gitlab.lrz.de/itr-seaclear/tum/sc2/development/oakd_isaac_ros.git
```
Copy the .isaac_ros_common-config file into your home directory:
```bash
cd ~/workspaces/oakd_isaac_ros && \
cp .isaac_ros_common-config $HOME
```

## Step 3
Install Git LFS to pull large files
```bash
sudo apt-get install git-lfs && \
git lfs install --skip-repo
```

Run the following script to build and launch the Docker container:

Jetson: 
```bash
oakd_isaac_ros/my_ws/scripts/launch_container.sh
```

Laptop (x86_64): 
```bash
oakd_isaac_ros/my_ws/scripts/launch_container_x86_64.sh
```

## Step 4
Inside the container, set the ISAAC_ROS_WS variable and build the workspace:

```bash
export ISAAC_ROS_WS=/workspaces/oakd_isaac_ros/ && \
colcon build --merge-install --packages-skip ros_deep_learning --cmake-args -DBUILD_TESTING=OFF
```

## Step 5
After the build has completed, source the ROS2 environment and run the launch file to run nvBlox with the OAK-D PoE:


Object detection only: 
```bash
source install/setup.bash && \
ros2 launch oakd_isaac_ros object_detection.launch.py
```


```bash
source install/setup.bash && \
ros2 launch oakd_isaac_ros oakd_nvblox.launch.py mode:='people_detection' use_rosbag:='false' voxel_size:='0.05'
```

Specifying the 'people_detection' mode will enable selective 3D reconstruction of objects in bounding box regions provided by the SeaClear YOLO model. Other mode options include 'static' and 'dynamic'. You can choose to run the pipeline on a pre-recorded rosbag file, or run the pipeline live with the OAK-D camera, by setting 'use_rosbag' to false/true respectively. Set the voxel size of the map by changing the value 'voxel_size'. The default value is 0.05, and reducing this value will increase the resolution of the map, with a decrease in rendering speed. 

## Step 6
Visualize the output of the algorithm in Foxglove. For this, display the static map and the dynamic map of the detect objects by subscribing to `/nvblox_node/color_layer` and '/nvblox_node/dynamic_occupancy_layer', respectively. Diplay these topics in a 3D panel in Foxglove. 

## Result 

[Watch the demo video](my_ws/assets/demo_vid.mp4)


## Troubleshooting

### Cannot Recieve Data from OAK-D
Check basic functionality of the OAK-D by running ` python -m depthai_viewer `.

If you can't see the OAK-D, follow this [troubleshooting guide.](https://docs.luxonis.com/hardware/platform/deploy/poe-deployment-guide#connected-to-the-same-lan-via-2-interfaces-wifi-ethernet)
