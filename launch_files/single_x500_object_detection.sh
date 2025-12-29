#!/bin/bash

SESSION_NAME="tmuxinator_session"

# CONFIGURATION (CHANGE MODEL HERE)
PX4_SYS_AUTOSTART=4002
PX4_GZ_MODEL=x500_depth
GZ_WORLD="custom"
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
PX4_GZ_MODEL=$PX4_GZ_MODEL /home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs ' " C-m

# ----------------------------
# Pane 2: Gazebo → ROS 2 Bridge for Image transport
# ----------------------------
gz_ros_image_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $gz_ros_image_pane "ros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" C-m

# ----------------------------
# Pane 3: Gazebo → ROS 2 Bridge for Depth transport
# ----------------------------
gz_ros_depth_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $gz_ros_depth_pane "ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image" C-m


# ----------------------------
# Pane 4: Micro XRCE-DDS Agent
# ----------------------------

dds_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $dds_pane "sleep 5; MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT" C-m

# ----------------------------
# Pane 5: Object detection
# ----------------------------

detection_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $detection_pane "sleep 8; CUDA_VISIBLE_DEVICES=0 ros2 run camera_subscriber camera_subscriber_node" C-m


# ----------------------------
# Equalize all panes in the first window
# ----------------------------
tmux select-layout -t $SESSION_NAME:0 tiled

# ROS2 tools window
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 " bash -c ' sleep 8; ros2 run rqt_image_view rqt_image_view ' " C-m

# Focus and attach
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

