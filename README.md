# UAV Simulation with PX4, ROS2 & Micro XRCE-DDS

Environment for running UAV simulations with **PX4 1.16.0**, **ROS2**, **Gazebo**, and **Micro XRCE-DDS**, including the **PX4-ROS2 bridge**.  
---

## Features

- Fully containerized PX4 SITL environment for Ubuntu 22.04.
- RGB and Depth sensor enabled for multirotor X500.
- GPU based YOLO v8 object detection.


## Directory Structure

```
PX4_1.16_ROS2_XRCE_DDS/
├── docker/
    └── Dockerfile
    └── entrypoint.sh
    └── run.sh
    └── build.sh
    └── scripts
        └── ..
└── launch_files/
    └── single_x500_depth_camera.sh
    └── single_vtol_camera.sh
└── resources/
    └── gz_models/
    └── gz_worlds/
    └── patches/
    └── ros_packages/
        └── camera_subscriber/
            └── ..
└── README.md

```

---

## Installation 

### Clone and build

```bash
git clone https://github.com/rrajnidhi/px4_1.16_ros2_xrce_dds.git
cd px4_1.16_ros2_xrce_dds/docker
./build.sh
```

---

## Launch container

To start and enter the docker container 

```bash
./run.sh
```
---

## Launch simulation

### A. Object detection on x500_depth Quadcopter

Once inside to launch PX4 SITL + ROS2 Bridge + Micro XRCE-DDS with YOLO v8 object detection in custom world :

```bash
cd /launch_files
./single_x500_object_detection.sh
```
Caution: This currently detects NVIDIA GPU and runs on it if availbale. Fallback is CPU, but this can saturate the processor.

#### Demonstration Video : 

Click on the thumbnail below to watch the SITL simulation video.


[![Watch the video](https://img.youtube.com/vi/d1SfdvVCJaI/0.jpg)](https://youtu.be/d1SfdvVCJaI)

---

### A. Lidar 2D Scan on x500_lidar
    To launch PX4 SITL + ROS2 Bridge + Micro XRCE-DDS with lidar scan 2D in custom world

```bash
cd /launch_files
./single_x500_lidar_icp.sh
```
Note: How visualise use pre-installed rviz2. 
    Either load the existing rviz config file or manually do the following.
    a. In Rviz Displays -> Global Options -> Fixed Frame , enter "x500_lidar_2d_0/link/lidar_2d_v2". And in topic section "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan". 
    b. Apply frame.
        To find the frame : ros2 topic echo /world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan --once 
        and  look for header: frame_id: "xxxxx"
        Use that exact frame_id as the Fixed Frame in RViz.


#### Demonstration Video : 

TBA


---

### List PX4 topics

```bash
ros2 topic list
```

---

### List gz topics

```bash
gz topic -l
```

---

## Maintainer

**Nidhi Raj**
[nidhirajr@gmail.com](mailto:nidhirajr@gmail.com)

---

## Acknowledgements

* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
* [ROS 2 Humble](https://docs.ros.org/en/humble/)
* [Gazebo Sim](https://gazebosim.org/)
* [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

---

> Enjoy! 