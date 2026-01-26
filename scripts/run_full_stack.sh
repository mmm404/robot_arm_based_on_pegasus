#!/bin/bash
# Refined Pegasus Arm System Launcher with Hand Tracking Support
# Launches: ROS2 description, Bridge, Listener, HandCV, CV Bridge, GUI

# Detect workspace root (assume script is run from workspace root)
WS_ROOT="$(pwd)"

# Define paths (relative to WS_ROOT)
BRIDGE_SIM_SCRIPT="$WS_ROOT/scripts/arduino_bridge_simulator.sh"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"
MONITOR_PORT="/tmp/virtual_monitor_sim"
VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
REAL_ARDUINO=""  # Will be set by user selection
GUI_SCRIPT="$WS_ROOT/scripts/GUI/GUI_commander.py"
LISTENER_SCRIPT="$WS_ROOT/src/update_pegasus_description/scripts/planned_path_listener.py"
FIFO_PATH="/tmp/pegasus_live_cmd"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE} PEGASUS ARM SYSTEM LAUNCHER (with Hand Tracking)${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ Workspace root detected: $WS_ROOT${NC}"

# PIDs for cleanup
SIM_PID=""
ROS2_PID=""
LISTENER_PID=""
HANDCV_PID=""
CVBRIDGE_PID=""
HEADLESS_PID=""

cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    
    # Stop all background processes
    [ ! -z "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null && echo -e "${GREEN}✓ Bridge stopped${NC}"
    [ ! -z "$ROS2_PID" ] && kill "$ROS2_PID" 2>/dev/null && echo -e "${GREEN}✓ ROS2 description stopped${NC}"
    [ ! -z "$LISTENER_PID" ] && kill "$LISTENER_PID" 2>/dev/null && echo -e "${GREEN}✓ Listener stopped${NC}"
    [ ! -z "$HANDCV_PID" ] && kill "$HANDCV_PID" 2>/dev/null && echo -e "${GREEN}✓ HandCV stopped${NC}"
    [ ! -z "$CVBRIDGE_PID" ] && kill "$CVBRIDGE_PID" 2>/dev/null && echo -e "${GREEN}✓ CV Bridge stopped${NC}"
    [ ! -z "$HEADLESS_PID" ] && kill "$HEADLESS_PID" 2>/dev/null && echo -e "${GREEN}✓ Headless Commander stopped${NC}"
    
    # Cleanup files
    rm -f "$VIRTUAL_ARDUINO" "$MONITOR_PORT" "$LOG_FILE" "$BRIDGE_LOG"
    [ -p "$FIFO_PATH" ] && rm -f "$FIFO_PATH"
    
    echo -e "${GREEN}✓ Cleanup complete${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Wait helper
wait_for_path() {
    local path=$1
    local timeout=${2:-10}
    local i=0
    while [ $i -lt "$timeout" ]; do
        if [ -e "$path" ] || [ -p "$path" ]; then
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    return 1
}

# Set ROS distro (default to humble if not set)
if [ -z "$ROS_DISTRO" ]; then
    export ROS_DISTRO=humble
fi

# Source base ROS2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    echo -e "${GREEN}✓ Base ROS2 ($ROS_DISTRO) environment sourced${NC}"
else
    echo -e "${RED}✗ Base ROS2 setup.bash not found in /opt/ros/$ROS_DISTRO/${NC}"
    echo -e "${YELLOW}Install ROS2 desktop/full: sudo apt install ros-$ROS_DISTRO-desktop${NC}"
    exit 1
fi

# Warn about potential ROS 1 conflict
if [[ ":$PYTHONPATH:" == *":/opt/ros/noetic:"* ]]; then
    echo -e "${YELLOW}⚠  WARNING: ROS 1 (Noetic) detected in PYTHONPATH.${NC}"
    echo -e "${YELLOW}   This may cause issues with the camera feed.${NC}"
    echo -e "${YELLOW}   The GUI will attempt to auto-fix this, but if it fails, remove ROS 1 from .bashrc.${NC}"
fi

# Source ROS2 environment (overlay)
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
    echo -e "${GREEN}✓ ROS2 overlay sourced${NC}"
else
    echo -e "${YELLOW}⚠  ROS2 overlay not found - rebuild workspace with colcon build${NC}"
fi

# Rebuild pegasus_control to ensure latest changes (like the camera fix) are applied
echo -e "\n${YELLOW}>>> [0/6] Updating pegasus_control...${NC}"
source "$WS_ROOT/install/setup.bash"
echo -e "${GREEN}✓ pegasus_control updated${NC}"

# -------------------------
# STEP 1: Launch ROS2 description (background, silent)
# -------------------------
echo -e "\n${YELLOW}>>> [1/6] Launching ROS2 description...${NC}"
export RVIZ_WINDOW_TITLE="Pegasus Arm RViz"
ros2 launch update_pegasus_description demo.launch.py > /tmp/ros2_description.log 2>&1 &
ROS2_PID=$!
echo -e "${GREEN}✓ ROS2 description started (PID: $ROS2_PID)${NC}"
sleep 8

# -------------------------
# STEP 2: Prompt for Arduino Port
# -------------------------
echo -e "\n${YELLOW}>>> [2/6] Arduino Port Selection${NC}"
echo -e "${CYAN}Available serial devices:${NC}"

# List available devices
devices=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)
if [ ! -z "$devices" ]; then
    echo "$devices" | nl
    echo ""
    echo -e "${CYAN}Enter selection:${NC}"
    echo -e "  ${GREEN}1-N${NC} = Use device number from list above"
    echo -e "  ${GREEN}custom${NC} = Enter custom port path"
    echo -e "  ${GREEN}sim${NC} = Use simulator (no physical Arduino)"
    echo -e "  ${GREEN}[Enter]${NC} = Default to simulator"
    echo ""
    read -p "Your choice: " user_choice
    
    if [ -z "$user_choice" ] || [ "$user_choice" = "sim" ]; then
        BRIDGE_MODE="simulation"
        echo -e "${YELLOW}⚠  Using simulator mode${NC}"
    elif [ "$user_choice" = "custom" ]; then
        read -p "Enter full port path (e.g., /dev/ttyUSB1): " custom_port
        if [ -e "$custom_port" ] && [ -r "$custom_port" ] && [ -w "$custom_port" ]; then
            REAL_ARDUINO="$custom_port"
            BRIDGE_MODE="bridge"
            echo -e "${GREEN}✓ Using: $REAL_ARDUINO${NC}"
        else
            echo -e "${RED}✗ Port not accessible: $custom_port${NC}"
            echo -e "${YELLOW}⚠  Falling back to simulator${NC}"
            BRIDGE_MODE="simulation"
        fi
    elif [[ "$user_choice" =~ ^[0-9]+$ ]]; then
        selected_device=$(echo "$devices" | sed -n "${user_choice}p")
        if [ ! -z "$selected_device" ] && [ -e "$selected_device" ]; then
            if [ -r "$selected_device" ] && [ -w "$selected_device" ]; then
                REAL_ARDUINO="$selected_device"
                BRIDGE_MODE="bridge"
                echo -e "${GREEN}✓ Using: $REAL_ARDUINO${NC}"
            else
                echo -e "${RED}✗ Port not accessible: $selected_device${NC}"
                echo -e "${YELLOW}⚠  Falling back to simulator${NC}"
                BRIDGE_MODE="simulation"
            fi
        else
            echo -e "${RED}✗ Invalid selection${NC}"
            echo -e "${YELLOW}⚠  Falling back to simulator${NC}"
            BRIDGE_MODE="simulation"
        fi
    else
        echo -e "${RED}✗ Invalid input${NC}"
        echo -e "${YELLOW}⚠  Falling back to simulator${NC}"
        BRIDGE_MODE="simulation"
    fi
else
    echo -e "${CYAN}(No devices found)${NC}"
    echo ""
    echo -e "${CYAN}Options:${NC}"
    echo -e "  ${GREEN}custom${NC} = Enter custom port path"
    echo -e "  ${GREEN}[Enter]${NC} = Use simulator"
    echo ""
    read -p "Your choice: " user_choice
    
    if [ "$user_choice" = "custom" ]; then
        read -p "Enter full port path: " custom_port
        if [ -e "$custom_port" ] && [ -r "$custom_port" ] && [ -w "$custom_port" ]; then
            REAL_ARDUINO="$custom_port"
            BRIDGE_MODE="bridge"
            echo -e "${GREEN}✓ Using: $REAL_ARDUINO${NC}"
        else
            echo -e "${RED}✗ Port not accessible: $custom_port${NC}"
            echo -e "${YELLOW}⚠  Falling back to simulator${NC}"
            BRIDGE_MODE="simulation"
        fi
    else
        BRIDGE_MODE="simulation"
        echo -e "${YELLOW}⚠  Using simulator mode${NC}"
    fi
fi

echo -e "${BLUE}Mode: $BRIDGE_MODE${NC}"

# Create FIFOs if bridge mode
if [ "$BRIDGE_MODE" = "bridge" ]; then
    [ -p "$MONITOR_PORT" ] || { rm -f "$MONITOR_PORT" 2>/dev/null; mkfifo "$MONITOR_PORT" 2>/dev/null; chmod 666 "$MONITOR_PORT" 2>/dev/null; }
    [ -p "$FIFO_PATH" ] || { rm -f "$FIFO_PATH" 2>/dev/null; mkfifo "$FIFO_PATH" 2>/dev/null; chmod 666 "$FIFO_PATH" 2>/dev/null; }
fi

# -------------------------
# Set Robot Port Env Var for all components
export ROBOT_PORT="$([ "$BRIDGE_MODE" = "bridge" ] && echo "$REAL_ARDUINO" || echo "$VIRTUAL_ARDUINO")"

# STEP 3: Launch Bridge (background, silent)
# -------------------------
echo -e "\n${YELLOW}>>> [3/6] Launching Arduino Bridge...${NC}"
if [ ! -x "$BRIDGE_SIM_SCRIPT" ]; then
    chmod +x "$BRIDGE_SIM_SCRIPT" 2>/dev/null || true
fi

if [ "$BRIDGE_MODE" = "simulation" ]; then
    BRIDGE_MODE="$BRIDGE_MODE" "$BRIDGE_SIM_SCRIPT" > "$LOG_FILE" 2>&1 &
    SIM_PID=$!
    echo -e "${GREEN}✓ Simulator Bridge started (PID: $SIM_PID)${NC}"
else
    echo -e "${GREEN}✓ Real Hardware Mode: Bridge script skipped (Listener manages Port)${NC}"
fi
sleep 4

# Verify virtual ports if simulation
if [ "$BRIDGE_MODE" = "simulation" ]; then
    if wait_for_path "$VIRTUAL_ARDUINO" 8; then
        echo -e "${GREEN}✓ Virtual Arduino: $VIRTUAL_ARDUINO${NC}"
    else
        echo -e "${RED}✗ Virtual Arduino port not found${NC}"
    fi
    if wait_for_path "$MONITOR_PORT" 8; then
        echo -e "${GREEN}✓ Monitor port: $MONITOR_PORT${NC}"
    else
        echo -e "${RED}✗ Monitor port not found${NC}"
    fi
fi

# -------------------------
# STEP 4: Launch Listener (background, silent)
# -------------------------
echo -e "\n${YELLOW}>>> [4/6] Launching planned_path_listener...${NC}"
if [ ! -f "$LISTENER_SCRIPT" ]; then
    echo -e "${RED}✗ Listener script not found: $LISTENER_SCRIPT${NC}"
else
    # python3 "$LISTENER_SCRIPT" > /tmp/listener.log 2>&1 &
    python3 "$LISTENER_SCRIPT" &
    LISTENER_PID=$!
    echo -e "${GREEN}✓ Listener started (PID: $LISTENER_PID)${NC}"
fi

# -------------------------
# STEP 5: Launch HandCV (if Kinect available)
# -------------------------
echo -e "\n${YELLOW}>>> [5/6] Checking for Kinect camera...${NC}"
if ros2 topic list | grep -q "/kinect2/hd/image_color_rect"; then
    echo -e "${GREEN}✓ Kinect topics detected${NC}"
    echo -e "${YELLOW}Launching HandCV node...${NC}"
    ros2 run handcv handcv > /tmp/handcv.log 2>&1 &
    HANDCV_PID=$!
    echo -e "${GREEN}✓ HandCV started (PID: $HANDCV_PID)${NC}"
    sleep 2
    
    # Launch CV Pegasus Bridge
    echo -e "${YELLOW}Launching CV Pegasus Bridge...${NC}"
    ros2 run cv_pegasus_bridge cv_pegasus_bridge > /tmp/cvbridge.log 2>&1 &
    CVBRIDGE_PID=$!
    echo -e "${GREEN}✓ CV Bridge started (PID: $CVBRIDGE_PID)${NC}"
    sleep 2
else
    echo -e "${YELLOW}⚠  No Kinect detected - hand tracking disabled${NC}"
fi

# -------------------------
# STEP 6: Launch Headless Commander (background)
# -------------------------
echo -e "\n${YELLOW}>>> [6/7] Launching Headless Commander...${NC}"
# ros2 run pegasus_control headless > /tmp/headless.log 2>&1 &
# ros2 run pegasus_control headless > /tmp/headless.log 2>&1 &
# ros2 run pegasus_control headless &
python3 "$WS_ROOT/scripts/headless_commander.py" &
HEADLESS_PID=$!
echo -e "${GREEN}✓ Headless Commander started (PID: $HEADLESS_PID)${NC}"
sleep 2

# -------------------------
# STEP 7: Launch GUI (foreground, main process)
# -------------------------
echo -e "\n${BLUE}================================================${NC}"
echo -e "${BLUE} LAUNCHING GUI (All logging in Action Log)${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ Bridge Mode: ${BRIDGE_MODE}${NC}"
echo -e "${GREEN}✓ ROS2 Description: Running (PID: $ROS2_PID)${NC}"
echo -e "${GREEN}✓ Listener: Running (PID: $LISTENER_PID)${NC}"
echo -e "${GREEN}✓ Headless Commander: Running (PID: $HEADLESS_PID)${NC}"
if [ ! -z "$HANDCV_PID" ]; then
    echo -e "${GREEN}✓ Hand Tracking: Running (PIDs: $HANDCV_PID, $CVBRIDGE_PID)${NC}"
else
    echo -e "${YELLOW}⚠ Hand Tracking: Disabled${NC}"
fi
echo -e "${YELLOW}Press Ctrl+C to stop everything${NC}"
echo ""

# Set environment for GUI

export BRIDGE_MODE="$BRIDGE_MODE"

# Run GUI using ros2 run
# Run GUI directly from source (bypassing rebuild requirement)
export PYTHONPATH="$WS_ROOT/src/pegasus_control:$PYTHONPATH"
# python3 "$WS_ROOT/src/pegasus_control/pegasus_control/gui/gui_node.py"
python3 "$GUI_SCRIPT"

