# UAV Simulation with PX4, ROS2 & Micro XRCE-DDS

Environment for running UAV simulations with **PX4 1.16.0**, **ROS2**, **Gazebo**, and **Micro XRCE-DDS**, including the **PX4-ROS2 bridge**.  
---

## Features

- Fully containerized PX4 SITL environment for Ubuntu 22.04
- RGB and Depth sensor enabled
- Camera drone with object detection


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

Once inside to launch PX4 SITL + ROS2 Bridge + Micro XRCE-DDS:

```bash
cd /launch_files
./single_x500_depth_camera.sh
```

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