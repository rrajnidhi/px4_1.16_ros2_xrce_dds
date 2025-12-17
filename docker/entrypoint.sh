#!/bin/bash
set -e

printf "Entering px4_ros2_humble container\n"
sleep 1

# ----------------------------
# tmux basic config (if missing)
# ----------------------------
TMUX_CONF="$HOME/.tmux.conf"
if [ ! -f "$TMUX_CONF" ]; then
    echo "bind e kill-session" > "$TMUX_CONF"
fi

# ----------------------------
# ROS 2 environment setup
# ----------------------------
# Source ROS 2 setup if available
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source PX4 ROS 2 workspace if installed
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# ----------------------------
# Execute command or login shell
# ----------------------------
# If no arguments passed, start a login interactive bash shell. This ensures tmux panes and commands run properly
if [ "$#" -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi

