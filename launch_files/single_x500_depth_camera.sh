#!/bin/bash

SESSION_NAME="tmuxinator_session"

# CONFIGURATION (CHANGE MODEL HERE)
PX4_SYS_AUTOSTART=4002
GZ_WORLD="default"
UXRCE_DDS_PORT=8888

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session
tmux new-session -d -s $SESSION_NAME

# ----------------------------
# Pane 0: Gazebo Simulation
# ----------------------------
tmux send-keys -t $SESSION_NAME:0.0 \
"exec python3 /home/ubuntu/PX4-gazebo-models/simulation-gazebo \
  --model_store /home/ubuntu/PX4-gazebo-models \
  --world $GZ_WORLD" C-m

# ----------------------------
# Pane 1: PX4 SITL
# ----------------------------
tmux split-window -h -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 \
"PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_PARAM_UXRCE_DDS_SYNCT=0 \
PX4_GZ_MODEL=x500_depth exec /home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs" C-m

# ----------------------------
# Pane 2: Gazebo → ROS 2 Bridge for Image transport
# ----------------------------
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 \
"exec ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" C-m

# ----------------------------
# Pane 3: Micro XRCE-DDS Agent
# ----------------------------
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.3 \
"exec MicroXRCEAgent udp4 -p $UXRCE_DDS_PORT" C-m

# ----------------------------
# Pane 4: Inspect PX4 topics
# ----------------------------
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.4 \
"exec ros2 topic echo /fmu/out/vehicle_status_v1" C-m

# ----------------------------
# Extra window: ROS 2 tools
# ----------------------------
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 \
"exec ros2 topic list" C-m

# Focus and attach
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

