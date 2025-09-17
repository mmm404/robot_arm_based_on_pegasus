#!/bin/bash
# Complete Pegasus Arm System Launcher with Arduino Bridge (non-fatal)
# Only modify this script; bridge script stays as-is (expects BRIDGE_MODE env var)

# Define paths
BRIDGE_SIM_SCRIPT="$HOME/ros2-ws/scripts/arduino_bridge_simulator.sh"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"
MONITOR_PORT="/tmp/virtual_monitor_sim"
VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
REAL_ARDUINO="/dev/ttyUSB0"
TMP_LOG="/tmp/ros2_node_output.log"
GUI_SCRIPT="$HOME/ros2-ws/scripts/GUI/GUI_commander.py"
LISTENER_SCRIPT="$HOME/ros2-ws/src/update_pegasus_description/scripts/planned_path_listener.py"
FIFO_PATH="/tmp/pegasus_live_cmd"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}    PEGASUS ARM BRIDGE SYSTEM LAUNCHER${NC}"
echo -e "${BLUE}================================================${NC}"

# PIDs for cleanup
SIM_PID=""
ROS2_TERMINAL_PID=""
MONITOR_TERMINAL_PID=""
GUI_TERMINAL_PID=""
LISTENER_TERMINAL_PID=""
FIFO_FORWARDER_PID=""

cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"

    # stop bridge simulator (background)
    if [ ! -z "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Bridge stopped${NC}"
    fi

    # kill FIFO forwarder if any
    if [ ! -z "$FIFO_FORWARDER_PID" ]; then
        kill "$FIFO_FORWARDER_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ FIFO forwarder stopped${NC}"
    fi

    # close gnome-terminal windows we started (best-effort)
    if [ ! -z "$MONITOR_TERMINAL_PID" ]; then
        kill "$MONITOR_TERMINAL_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Monitor terminal closed${NC}"
    fi
    if [ ! -z "$ROS2_TERMINAL_PID" ]; then
        kill "$ROS2_TERMINAL_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ ROS2 description terminal closed${NC}"
    fi
    if [ ! -z "$GUI_TERMINAL_PID" ]; then
        kill "$GUI_TERMINAL_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ GUI terminal closed${NC}"
    fi
    if [ ! -z "$LISTENER_TERMINAL_PID" ]; then
        kill "$LISTENER_TERMINAL_PID" 2>/dev/null || true
        echo -e "${GREEN}✓ Listener terminal closed${NC}"
    fi

    # best-effort cleanup of fifos and virtual ports and logs
    rm -f "$VIRTUAL_ARDUINO" "$MONITOR_PORT" "$LOG_FILE" "$BRIDGE_LOG" "$TMP_LOG"
    [ -p "$FIFO_PATH" ] && rm -f "$FIFO_PATH"
    echo -e "${GREEN}✓ Cleanup complete${NC}"

    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# helper: wait for a path with timeout (seconds)
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
    echo -e "${YELLOW}⚠ ROS2 overlay not found at ~/ros2-ws/install/setup.bash - continuing (you may need to source manually)${NC}"
fi

# -------------------------
# STEP 1: ROS2 description
# -------------------------
echo -e "\n${YELLOW}>>> [1/6] Launching ROS2 description in new terminal...${NC}"
gnome-terminal --title="ROS2 Description Launch" -- bash -c "
    echo -e '${BLUE}================================================${NC}'
    echo -e '${BLUE}    ROS2 DESCRIPTION LAUNCH${NC}'
    echo -e '${BLUE}================================================${NC}'
    source ~/ros2-ws/install/setup.bash 2>/dev/null || true
    echo -e '${GREEN}Loading ROS2 description...${NC}'
    ros2 launch update_pegasus_description demo.launch.py
    echo -e '${RED}ROS2 description ended (if it exited)${NC}'
    read -p 'Press Enter to close this terminal...'
" &
# capture the gnome-terminal PID (best-effort)
sleep 2
ROS2_TERMINAL_PID=$(pgrep -f "ROS2 DESCRIPTION LAUNCH" || true)
echo -e "${GREEN}✓ ROS2 description launched in separate terminal${NC}"
sleep 10     # let ROS2 initialize

# -------------------------
# STEP 2: Detect Arduino
# -------------------------
echo -e "\n${YELLOW}>>> [2/6] Checking Arduino Connection...${NC}"
if [ -e "$REAL_ARDUINO" ]; then
    if [ -r "$REAL_ARDUINO" ] && [ -w "$REAL_ARDUINO" ]; then
        echo -e "${GREEN}✓ Real Arduino found and accessible at: $REAL_ARDUINO${NC}"
        BRIDGE_MODE="bridge"
    else
        echo -e "${YELLOW}⚠ Real Arduino present but not accessible: $REAL_ARDUINO${NC}"
        BRIDGE_MODE="simulation"
    fi
else
    echo -e "${YELLOW}⚠ Real Arduino not found at: $REAL_ARDUINO${NC}"
    echo -e "${CYAN}  Available serial devices:${NC}"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo -e "${CYAN}  (none)${NC}"
    read -p "Enter custom Arduino port (or press Enter to use simulator): " custom_port
    if [ ! -z "$custom_port" ] && [ -e "$custom_port" ]; then
        REAL_ARDUINO="$custom_port"
        BRIDGE_MODE="bridge"
        echo -e "${GREEN}✓ Using custom Arduino port: $REAL_ARDUINO${NC}"
    else
        BRIDGE_MODE="simulation"
        echo -e "${YELLOW}⚠ Using simulator mode${NC}"
    fi
fi
echo -e "${BLUE}  Bridge mode: $BRIDGE_MODE${NC}"

# if would-be bridge mode, ensure FIFO and monitor exist
if [ "$BRIDGE_MODE" = "bridge" ]; then
    # create monitor fifo if missing
    [ -p "$MONITOR_PORT" ] || { rm -f "$MONITOR_PORT" 2>/dev/null || true; mkfifo "$MONITOR_PORT" 2>/dev/null || true; chmod 666 "$MONITOR_PORT" 2>/dev/null || true; }
    [ -p "$FIFO_PATH" ] || { rm -f "$FIFO_PATH" 2>/dev/null || true; mkfifo "$FIFO_PATH" 2>/dev/null || true; chmod 666 "$FIFO_PATH" 2>/dev/null || true; }
fi

# -------------------------
# STEP 3: Launch Arduino bridge (simulator or bridge)
# -------------------------
echo -e "\n${YELLOW}>>> [3/6] Launching Arduino Bridge...${NC}"
if [ ! -x "$BRIDGE_SIM_SCRIPT" ]; then
    # try make it executable
    chmod +x "$BRIDGE_SIM_SCRIPT" 2>/dev/null || true
fi

# Launch bridge script once; pass BRIDGE_MODE in env, send logs to LOG_FILE
BRIDGE_MODE="$BRIDGE_MODE" "$BRIDGE_SIM_SCRIPT" > "$LOG_FILE" 2>&1 &
SIM_PID=$!
sleep 2
echo -e "${BLUE}  Bridge PID: $SIM_PID${NC}"
echo -e "${YELLOW}  Waiting briefly for bridge to initialize...${NC}"
sleep 4

# If simulation mode, check for virtual ports (don't abort if missing; only warn)
if [ "$BRIDGE_MODE" = "simulation" ]; then
    if wait_for_path "$VIRTUAL_ARDUINO" 8; then
        echo -e "${GREEN}✓ Virtual Arduino port created: $VIRTUAL_ARDUINO${NC}"
    else
        echo -e "${RED}✗ Virtual Arduino port not found after timeout (bridge may have failed)${NC}"
    fi
    if wait_for_path "$MONITOR_PORT" 8; then
        echo -e "${GREEN}✓ Monitor port created: $MONITOR_PORT${NC}"
    else
        echo -e "${RED}✗ Monitor port not found after timeout${NC}"
    fi
else
    echo -e "${GREEN}✓ Using real Arduino at: $REAL_ARDUINO${NC}"
fi

# -------------------------
# STEP 4: Launch Monitor terminal
# -------------------------
echo -e "\n${YELLOW}>>> [4/6] Launching Enhanced Monitor in new terminal...${NC}"
gnome-terminal --title="Arduino Bridge Monitor" -- bash -c "
    echo -e '${BLUE}================================================${NC}'
    echo -e '${BLUE}    ARDUINO BRIDGE MONITOR${NC}'
    echo -e '${BLUE}================================================${NC}'
    echo -e '${GREEN}Monitoring: ${BRIDGE_MODE} mode${NC}'
    if [ '$BRIDGE_MODE' = 'bridge' ]; then
        echo -e '${GREEN}Real Arduino: $REAL_ARDUINO${NC}'
    else
        echo -e '${GREEN}Monitor port: $MONITOR_PORT${NC}'
    fi
    echo -e '${YELLOW}Press Ctrl+C to close this monitor${NC}'
    echo ''
    if [ -p '$MONITOR_PORT' ]; then
        stdbuf -oL -eL cat '$MONITOR_PORT' 2>/dev/null || while true; do timeout 1s cat '$MONITOR_PORT' 2>/dev/null || sleep 0.2; done
    else
        tail -f '$LOG_FILE'
    fi
    read -p 'Press Enter to close this terminal...'
" &
sleep 1
MONITOR_TERMINAL_PID=$!
echo -e "${GREEN}✓ Enhanced monitor terminal opened${NC}"

# -------------------------
# STEP 5: Launch GUI Commander terminal
# -------------------------
echo -e "\n${YELLOW}>>> [5/6] Launching GUI Commander in new terminal...${NC}"
gnome-terminal --title="Pegasus GUI Commander" -- bash -c "
    echo -e '${BLUE}================================================${NC}'
    echo -e '${BLUE}    PEGASUS GUI COMMANDER${NC}'
    echo -e '${BLUE}================================================${NC}'
    source ~/ros2-ws/install/setup.bash 2>/dev/null || true
    echo -e '${GREEN}Bridge Mode: $BRIDGE_MODE${NC}'
    echo -e '${GREEN}Port: ${BRIDGE_MODE}' > /dev/null 2>&1 || true
    cd ~/ros2-ws/scripts/GUI 2>/dev/null || true
    if [ -f 'GUI_commander.py' ]; then
        python3 GUI_commander.py --port '$([ "$BRIDGE_MODE" = "bridge" ] && echo "$REAL_ARDUINO" || echo "$VIRTUAL_ARDUINO")' 2>&1 | tee '$TMP_LOG'
    else
        echo -e '${RED}✗ GUI_commander.py not found in ~/ros2-ws/scripts/GUI${NC}'
    fi
    read -p 'Press Enter to close this terminal...'
" &
sleep 2
GUI_TERMINAL_PID=$!
echo -e "${GREEN}✓ GUI launched in separate terminal${NC}"

# -------------------------
# STEP 6: Launch planned_path_listener in its own terminal
# -------------------------
echo -e "\n${YELLOW}>>> [6/6] Launching planned_path_listener in new terminal...${NC}"
if [ ! -f "$LISTENER_SCRIPT" ]; then
    echo -e "${RED}✗ planned_path_listener.py not found at: $LISTENER_SCRIPT${NC}"
else
    gnome-terminal --title="Planned Path Listener" -- bash -c "
        source ~/ros2-ws/install/setup.bash 2>/dev/null || true
        echo -e '${BLUE}================================================${NC}'
        echo -e '${BLUE}    PLANNED PATH LISTENER${NC}'
        echo -e '${BLUE}================================================${NC}'
        python3 '$LISTENER_SCRIPT'
        echo -e '${RED}Listener exited${NC}'
        read -p 'Press Enter to close this terminal...'
    " &
    sleep 1
    LISTENER_TERMINAL_PID=$!
    echo -e "${GREEN}✓ Listener started in separate terminal${NC}"
fi

# -------------------------
# Final status + keepalive
# -------------------------
echo -e "\n${BLUE}================================================${NC}"
echo -e "${BLUE}    BRIDGE SYSTEM READY (non-fatal)${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ Arduino Bridge mode: ${BRIDGE_MODE}${NC}"
echo -e "${GREEN}✓ ROS2 Description: Running in separate terminal${NC}"
echo -e "${GREEN}✓ GUI Commander: Running in separate terminal${NC}"
echo -e "${GREEN}✓ Listener: Running in separate terminal${NC}"
echo -e "${GREEN}✓ Monitor: Running in separate terminal${NC}"
echo -e "${YELLOW}Press Ctrl+C here to stop everything and perform cleanup${NC}"
echo ""

# keep the main launcher alive; warn if bridge dies but do not shutdown everything
while true; do
    sleep 5
    if [ ! -z "$SIM_PID" ]; then
        if ! ps -p "$SIM_PID" > /dev/null 2>&1; then
            echo -e "${YELLOW}⚠ Bridge process (PID $SIM_PID) is not running. Check $LOG_FILE for details.${NC}"
            # don't break; keep other terminals alive so you can debug
            SIM_PID=""
        fi
    fi
done
