#!/bin/bash

SESSION_NAME="tmuxinator_session"

# CONFIGURATION (CHANGE MODEL HERE)
PX4_SYS_AUTOSTART=4013  #x500_lidar_2d
GZ_WORLD="custom_indoor_lab" #  use custom_indoor_lab, default, walls
PX4_UXRCE_DDS_PORT=8888

# PX4 LOCAL STARTING LOCATION Local ENU coordinates (meters, radians)
PX4_START_X=8.0     # East
PX4_START_Y=0.0     # North
PX4_START_Z=0.3     # Up (spawn slightly above ground)
PX4_START_ROLL=0.0
PX4_START_PITCH=0.0
PX4_START_YAW=0.0   # radians (0 = facing East)

PX4_GZ_MODEL_POSE="${PX4_START_X},${PX4_START_Y},${PX4_START_Z},${PX4_START_ROLL},${PX4_START_PITCH},${PX4_START_YAW}"


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
tmux send-keys -t $SESSION_NAME:0.1 "bash -c ' PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_GZ_MODEL_POSE=\"$PX4_GZ_MODEL_POSE\" \
/home/ubuntu/px4_sitl/bin/px4 -w /home/ubuntu/px4_sitl/romfs ' " C-m

# ----------------------------
# Pane 2: Gazebo → ROS 2 Bridge for 2Dlidar+PointCloud transport
# ----------------------------
gz_ros_lidar_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $gz_ros_lidar_pane "ros2 run ros_gz_bridge parameter_bridge \
/world/$GZ_WORLD/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/world/$GZ_WORLD/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked" C-m

# ----------------------------
# Pane 3: Micro XRCE-DDS Agent
# ----------------------------

dds_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $dds_pane "sleep 5; MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT" C-m

# ----------------------------
# Pane 4: Fixed TF convertion for lidar visualization : base_link → x500_lidar_2d_0/link/lidar_2d_v2
# ----------------------------

tf2_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $tf2_pane "sleep 6; ros2 run tf2_ros static_transform_publisher 0.2 0 0.1 0 0 0 base_link x500_lidar_2d_0/link/lidar_2d_v2" C-m

# ----------------------------
# Pane 5: TF2 convertion for visualization of UAV (map → base_link(UAV))
# ----------------------------

px4_tf_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $px4_tf_pane "sleep 7; ros2 run px4_tf_bridge px4_tf_bridge" C-m

# ----------------------------
# Equalize all panes in the first window
# ----------------------------
tmux select-layout -t $SESSION_NAME:0 tiled

# RVIZ2 window : window 2

# ----------------------------
# pane 0 : Load rviz config
# ----------------------------
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 " bash -c ' sleep 7.5; rviz2 -d /home/ubuntu/rviz/2d_lidar_project/config/rviz_2d_lidar.rviz' " C-m

# ----------------------------
# pane 1 : Load uav model for rviz
# ----------------------------

tmux split-window -h -t $SESSION_NAME:1.0
tmux send-keys -t $SESSION_NAME:1.1 "bash -c 'sleep 8; ros2 run robot_state_publisher robot_state_publisher /home/ubuntu/rviz/2d_lidar_project/uav_model/x500.urdf.xacro '" C-m

# ----------------------------
# Equalize all panes in the second window
# ----------------------------
tmux select-layout -t $SESSION_NAME:1 tiled


# Another window
tmux new-window -t $SESSION_NAME -n "test_window_1"
tmux send-keys -t $SESSION_NAME:1 " bash -c ' sleep 2; ' " C-m

# Focus and attach
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

