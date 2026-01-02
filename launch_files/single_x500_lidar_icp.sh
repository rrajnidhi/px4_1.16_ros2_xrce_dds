#!/bin/bash

SESSION_NAME="tmuxinator_session"

# CONFIGURATION (CHANGE MODEL HERE)
PX4_SYS_AUTOSTART=4013  #x500_lidar_2d
GZ_WORLD="default" #  use custom_indoor_lab
PX4_UXRCE_DDS_PORT=8888

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session
tmux new-session -d -s $SESSION_NAME

# ----------------------------
# Pane 0: Gazebo Simulation
# ----------------------------
tmux send-keys -t $SESSION_NAME:0.0 " bash -c ' python3 /home/ubuntu/PX4-gazebo-models/simulation-gazebo --model_store /home/ubuntu/PX4-gazebo-models --world $GZ_WORLD ' " C-m

# ----------------------------
# Pane 1: PX4 SITL
# ----------------------------
tmux split-window -h -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 "bash -c ' PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_PARAM_UXRCE_DDS_SYNCT=0 \
/home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs ' " C-m

# ----------------------------
# Pane 2: Gazebo → ROS 2 Bridge for 2Dlidar+PointCloud transport
# ----------------------------
gz_ros_lidar_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $gz_ros_lidar_pane "ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked " C-m

# ----------------------------
# Pane 3: Micro XRCE-DDS Agent
# ----------------------------

dds_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $dds_pane "sleep 5; MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT" C-m

# ----------------------------
# Pane 4: icp :TODO
# ----------------------------

icp_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $icp_pane "sleep 8; " C-m

# ----------------------------
# Equalize all panes in the first window
# ----------------------------
tmux select-layout -t $SESSION_NAME:0 tiled

# ROS2 tools window
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 " bash -c ' sleep 8; rviz2 ' " C-m

# Focus and attach
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

