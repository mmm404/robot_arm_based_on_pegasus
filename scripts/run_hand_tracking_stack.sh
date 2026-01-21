#!/bin/bash

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Detect workspace root and source overlay
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
else
    echo "Warning: Workspace overlay not found at $WS_ROOT/install/setup.bash"
fi

# Function to launch in new terminal
launch_terminal() {
    local title="$1"
    local command="$2"
    
    echo "Launching $title..."
    gnome-terminal --title="$title" -- bash -c "source /opt/ros/humble/setup.bash; source \"$WS_ROOT/install/setup.bash\"; $command; exec bash"
}

launch_terminal "Kinect Bridge" "ros2 run kinect2_bridge kinect2_bridge_node"
sleep 5

launch_terminal "HandCV" "ros2 run handcv handcv_node"
sleep 2

launch_terminal "CV Pegasus Bridge" "ros2 run cv_pegasus_bridge cv_pegasus_bridge"
sleep 2

launch_terminal "Gripper Control" "ros2 run gripper_control gripper_node"

echo "All nodes launched in separate terminals."
