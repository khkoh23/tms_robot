#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE}")" && pwd)
WS_DIR=$(cd "$SCRIPT_DIR/../../../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/local_setup.bash"
export RCUTILS_COLORIZED_OUTPUT=1
export QT_QPA_PLATFORM=xcb
ros2 launch tms_robot_bringup all.launch.py