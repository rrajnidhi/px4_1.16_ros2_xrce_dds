#!/bin/bash

# test tmux launcher

SESSION_NAME="test"

# Kill any existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session (detached)
tmux new-session -d -s $SESSION_NAME

# Pane 0: PX4 SITL + Gazebo
tmux send-keys -t $SESSION_NAME "sleep 1" C-m

# Pane 1: Micro XRCE-DDS Agent with wait-for-SITL to start logic
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 " sleep 2" C-m

# Pane 2: for future use
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 "sleep 3" C-m

# Select first pane
tmux select-pane -t $SESSION_NAME:0.0

# Attach to tmux session
tmux attach-session -t $SESSION_NAME
