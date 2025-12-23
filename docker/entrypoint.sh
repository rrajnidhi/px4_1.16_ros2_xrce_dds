#!/bin/bash
set -e

printf "Entering px4_ros2_humble container\n"
sleep 1

# tmux basic config
TMUX_CONF="$HOME/.tmux.conf"
if [ ! -f "$TMUX_CONF" ]; then
    echo "bind e kill-session" > "$TMUX_CONF"
fi

grep -qxF 'set-option -g default-shell /bin/bash' "$TMUX_CONF" || \
    echo 'set-option -g default-shell /bin/bash' >> "$TMUX_CONF"

grep -qxF 'set-option -g default-command /bin/bash' "$TMUX_CONF" || \
    echo 'set-option -g default-command /bin/bash' >> "$TMUX_CONF"

# ROS 2 environment setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Execute command or login shell
if [ "$#" -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi

