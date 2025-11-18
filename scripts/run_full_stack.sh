#!/bin/bash
# Refined Pegasus Arm System Launcher - Single Terminal
# All logging consolidated to GUI action log

# Define paths
BRIDGE_SIM_SCRIPT="$HOME/ros2-ws/scripts/arduino_bridge_simulator.sh"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"
MONITOR_PORT="/tmp/virtual_monitor_sim"
VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
REAL_ARDUINO="/dev/ttyUSB0"
GUI_SCRIPT="$HOME/ros2-ws/scripts/GUI/GUI_commander.py"
LISTENER_SCRIPT="$HOME/ros2-ws/src/update_pegasus_description/scripts/planned_path_listener.py"
FIFO_PATH="/tmp/pegasus_live_cmd"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE} PEGASUS ARM SYSTEM LAUNCHER${NC}"
echo -e "${BLUE}================================================${NC}"

# PIDs for cleanup
SIM_PID=""
ROS2_PID=""
LISTENER_PID=""

cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    
    # Stop bridge simulator
    if [ ! -z "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Bridge stopped${NC}"
    fi
    
    # Stop ROS2 description
    if [ ! -z "$ROS2_PID" ]; then
        kill "$ROS2_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ ROS2 description stopped${NC}"
    fi
    
    # Stop listener
    if [ ! -z "$LISTENER_PID" ]; then
        kill "$LISTENER_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Listener stopped${NC}"
    fi
    
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

# Source ROS2 environment
if [ -f ~/ros2-ws/install/setup.bash ]; then
    source ~/ros2-ws/install/setup.bash
    echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
    echo -e "${YELLOW}⚠  ROS2 overlay not found${NC}"
fi

# -------------------------
# STEP 1: Launch ROS2 description (background, silent)
# -------------------------
echo -e "\n${YELLOW}>>> [1/4] Launching ROS2 description...${NC}"
export RVIZ_WINDOW_TITLE="Pegasus Arm RViz"
ros2 launch update_pegasus_description demo.launch.py > /tmp/ros2_description.log 2>&1 &
ROS2_PID=$!
echo -e "${GREEN}✓ ROS2 description started (PID: $ROS2_PID)${NC}"
sleep 8

# -------------------------
# STEP 2: Detect Arduino
# -------------------------
echo -e "\n${YELLOW}>>> [2/4] Checking Arduino Connection...${NC}"
if [ -e "$REAL_ARDUINO" ]; then
    if [ -r "$REAL_ARDUINO" ] && [ -w "$REAL_ARDUINO" ]; then
        echo -e "${GREEN}✓ Real Arduino found: $REAL_ARDUINO${NC}"
        BRIDGE_MODE="bridge"
    else
        echo -e "${YELLOW}⚠  Arduino not accessible: $REAL_ARDUINO${NC}"
        BRIDGE_MODE="simulation"
    fi
else
    echo -e "${YELLOW}⚠  Arduino not found: $REAL_ARDUINO${NC}"
    echo -e "${CYAN} Available devices:${NC}"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo -e "${CYAN} (none)${NC}"
    read -p "Enter custom port (or press Enter for simulator): " custom_port
    if [ ! -z "$custom_port" ] && [ -e "$custom_port" ]; then
        REAL_ARDUINO="$custom_port"
        BRIDGE_MODE="bridge"
        echo -e "${GREEN}✓ Using: $REAL_ARDUINO${NC}"
    else
        BRIDGE_MODE="simulation"
        echo -e "${YELLOW}⚠  Using simulator mode${NC}"
    fi
fi

echo -e "${BLUE} Mode: $BRIDGE_MODE${NC}"

# Create FIFOs if bridge mode
if [ "$BRIDGE_MODE" = "bridge" ]; then
    [ -p "$MONITOR_PORT" ] || { rm -f "$MONITOR_PORT" 2>/dev/null; mkfifo "$MONITOR_PORT" 2>/dev/null; chmod 666 "$MONITOR_PORT" 2>/dev/null; }
    [ -p "$FIFO_PATH" ] || { rm -f "$FIFO_PATH" 2>/dev/null; mkfifo "$FIFO_PATH" 2>/dev/null; chmod 666 "$FIFO_PATH" 2>/dev/null; }
fi

# -------------------------
# STEP 3: Launch Bridge (background, silent)
# -------------------------
echo -e "\n${YELLOW}>>> [3/4] Launching Arduino Bridge...${NC}"
if [ ! -x "$BRIDGE_SIM_SCRIPT" ]; then
    chmod +x "$BRIDGE_SIM_SCRIPT" 2>/dev/null || true
fi

BRIDGE_MODE="$BRIDGE_MODE" "$BRIDGE_SIM_SCRIPT" > "$LOG_FILE" 2>&1 &
SIM_PID=$!
echo -e "${GREEN}✓ Bridge started (PID: $SIM_PID, Mode: $BRIDGE_MODE)${NC}"
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
echo -e "\n${YELLOW}>>> [4/4] Launching planned_path_listener...${NC}"
if [ ! -f "$LISTENER_SCRIPT" ]; then
    echo -e "${RED}✗ Listener script not found: $LISTENER_SCRIPT${NC}"
else
    python3 "$LISTENER_SCRIPT" > /tmp/listener.log 2>&1 &
    LISTENER_PID=$!
    echo -e "${GREEN}✓ Listener started (PID: $LISTENER_PID)${NC}"
fi

# -------------------------
# Launch GUI (foreground, main process)
# -------------------------
echo -e "\n${BLUE}================================================${NC}"
echo -e "${BLUE} LAUNCHING GUI (All logging in Action Log)${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ Bridge Mode: ${BRIDGE_MODE}${NC}"
echo -e "${GREEN}✓ ROS2 Description: Running (PID: $ROS2_PID)${NC}"
echo -e "${GREEN}✓ Listener: Running (PID: $LISTENER_PID)${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop everything${NC}"
echo ""

# Set environment for GUI
export ROS_DOMAIN_ID=0
export ROBOT_PORT="$([ "$BRIDGE_MODE" = "bridge" ] && echo "$REAL_ARDUINO" || echo "$VIRTUAL_ARDUINO")"
export BRIDGE_MODE="$BRIDGE_MODE"

cd ~/ros2-ws/scripts/GUI 2>/dev/null || true
if [ -f 'GUI_commander.py' ]; then
    # Run GUI in foreground - when it exits, cleanup runs
    python3 GUI_commander.py
else
    echo -e "${RED}✗ GUI_commander.py not found${NC}"
    sleep 5
fi

# Cleanup happens automatically via trap when GUI exits
