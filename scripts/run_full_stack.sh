#!/bin/bash
# Complete Pegasus Arm System Launcher with Arduino Bridge
# This script launches bridge, ROS2 node, and monitor

# Define paths
BRIDGE_SIM_SCRIPT="$HOME/ros2-ws/scripts/arduino_bridge_simulator.sh"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"
MONITOR_PORT="/tmp/virtual_monitor_sim"
VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
REAL_ARDUINO="/dev/ttyUSB0"
TMP_LOG="/tmp/ros2_node_output.log"
PYTHON_GUI_SCRIPT="$HOME/ros2-ws/src/pegasus_arm_commander/pegasus_arm_commander/pegasus_commander.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}    PEGASUS ARM BRIDGE SYSTEM LAUNCHER${NC}"
echo -e "${BLUE}================================================${NC}"

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    
    # Kill bridge
    if [ ! -z "$SIM_PID" ]; then
        kill $SIM_PID 2>/dev/null
        echo -e "${GREEN}✓ Bridge stopped${NC}"
    fi
    
    # Kill ROS2 node
    if [ ! -z "$ROS2_PID" ]; then
        kill $ROS2_PID 2>/dev/null
        echo -e "${GREEN}✓ ROS2 node stopped${NC}"
    fi
    
    # Kill Python GUI (fallback)
    if [ ! -z "$GUI_PID" ]; then
        kill $GUI_PID 2>/dev/null
        echo -e "${GREEN}✓ Python GUI stopped${NC}"
    fi
    
    # Clean up virtual ports and logs
    rm -f $VIRTUAL_ARDUINO $MONITOR_PORT $LOG_FILE $BRIDGE_LOG $TMP_LOG
    echo -e "${GREEN}✓ Cleanup complete${NC}"
    
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM EXIT

# Function to check if port exists
wait_for_port() {
    local port=$1
    local timeout=10
    local count=0
    
    while [ $count -lt $timeout ]; do
        if [ -e "$port" ]; then
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    return 1
}

# Function to check Arduino connection
check_arduino() {
    if [ -e "$REAL_ARDUINO" ]; then
        if [ -w "$REAL_ARDUINO" ] && [ -r "$REAL_ARDUINO" ]; then
            return 0  # Arduino available and accessible
        else
            return 1  # Arduino exists but not accessible
        fi
    else
        return 2  # Arduino not found
    fi
}

# Source ROS2 environment
source ~/ros2-ws/install/setup.bash
echo -e "${GREEN}✓ ROS2 environment sourced${NC}"

echo -e "\n${YELLOW}>>> [1/5] Checking Arduino Connection...${NC}"

# Check Arduino status
check_arduino
arduino_status=$?

case $arduino_status in
    0)
        echo -e "${GREEN}✓ Real Arduino found and accessible at: $REAL_ARDUINO${NC}"
        BRIDGE_MODE="bridge"
        ;;
    1)
        echo -e "${YELLOW}⚠ Real Arduino found but not accessible at: $REAL_ARDUINO${NC}"
        echo -e "${CYAN}  Try: sudo chmod 666 $REAL_ARDUINO${NC}"
        echo -e "${CYAN}  Or: sudo usermod -a -G dialout $USER (requires logout/login)${NC}"
        BRIDGE_MODE="simulation"
        ;;
    2)
        echo -e "${YELLOW}⚠ Real Arduino not found at: $REAL_ARDUINO${NC}"
        echo -e "${CYAN}  Available USB devices:${NC}"
        ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo -e "${CYAN}  No USB serial devices found${NC}"
        read -p "Enter custom Arduino port (or press Enter to use simulator): " custom_port
        if [ ! -z "$custom_port" ]; then
            REAL_ARDUINO="$custom_port"
            if [ -e "$REAL_ARDUINO" ] && [ -w "$REAL_ARDUINO" ] && [ -r "$REAL_ARDUINO" ]; then
                echo -e "${GREEN}✓ Custom Arduino port accessible: $REAL_ARDUINO${NC}"
                BRIDGE_MODE="bridge"
            else
                echo -e "${RED}✗ Custom port $REAL_ARDUINO not accessible, using simulator${NC}"
                BRIDGE_MODE="simulation"
            fi
        else
            BRIDGE_MODE="simulation"
        fi
        ;;
esac

echo -e "${BLUE}  Bridge mode: $BRIDGE_MODE${NC}"

echo -e "\n${YELLOW}>>> [2/5] Launching Arduino Bridge...${NC}"

# Check if bridge script exists
if [ ! -f "$BRIDGE_SIM_SCRIPT" ]; then
    echo -e "${RED}✗ Bridge script not found at: $BRIDGE_SIM_SCRIPT${NC}"
    echo -e "${YELLOW}Please ensure the arduino_bridge_simulator.sh script is in the correct location${NC}"
    exit 1
fi

# Launch bridge script in background
$BRIDGE_SIM_SCRIPT > /dev/null 2>&1 &
SIM_PID=$!

echo -e "${BLUE}  Bridge PID: $SIM_PID${NC}"
echo -e "${YELLOW}  Waiting for ports to be created...${NC}"

# Wait for ports (simulator) or Arduino (bridge)
if [ "$BRIDGE_MODE" = "simulation" ]; then
    if wait_for_port "$VIRTUAL_ARDUINO"; then
        echo -e "${GREEN}✓ Virtual Arduino port created: $VIRTUAL_ARDUINO${NC}"
    else
        echo -e "${RED}✗ Failed to create virtual Arduino port${NC}"
        exit 1
    fi

    if wait_for_port "$MONITOR_PORT"; then
        echo -e "${GREEN}✓ Monitor port created: $MONITOR_PORT${NC}"
    else
        echo -e "${RED}✗ Failed to create monitor port${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ Using real Arduino at: $REAL_ARDUINO${NC}"
fi

echo -e "\n${YELLOW}>>> [3/5] Launching Enhanced Monitor in new terminal...${NC}"

# Launch enhanced monitor in new terminal window
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
    echo -e '${BLUE}================================================${NC}'
    echo ''
    echo -e '${CYAN}Waiting for activity...${NC}'
    echo ''
    
    if [ '$BRIDGE_MODE' = 'simulation' ]; then
        if [ -e '$MONITOR_PORT' ]; then
            stdbuf -oL -eL cat '$MONITOR_PORT' 2>/dev/null || {
                echo 'stdbuf not available, using regular cat...'
                while true; do
                    if [ -e '$MONITOR_PORT' ]; then
                        timeout 1s cat '$MONITOR_PORT' 2>/dev/null || sleep 0.1
                    else
                        echo 'Monitor port not available, retrying...'
                        sleep 1
                    fi
                done
            }
        else
            echo 'Monitor port not found. Waiting...'
            while [ ! -e '$MONITOR_PORT' ]; do
                sleep 1
            done
            stdbuf -oL -eL cat '$MONITOR_PORT' 2>/dev/null || cat '$MONITOR_PORT'
        fi
    else
        tail -f '$BRIDGE_LOG'
    fi
" 2>/dev/null &

echo -e "${GREEN}✓ Enhanced monitor terminal opened${NC}"

echo -e "\n${YELLOW}>>> [4/5] Launching ROS2 Node...${NC}"

# Clear previous log
rm -f "$TMP_LOG"

# Try ROS2 run first
if ros2 run pegasus_arm_commander pegasus_commander --port "${BRIDGE_MODE}" = "bridge" && "$REAL_ARDUINO" || "$VIRTUAL_ARDUINO" 2>&1 | tee "$TMP_LOG" & then
    ROS2_PID=$!
    echo -e "${GREEN}✓ ROS2 node started with 'ros2 run' (PID: $ROS2_PID)${NC}"
else
    echo -e "${YELLOW}⚠ Failed to start ROS2 node with 'ros2 run', falling back to direct Python execution${NC}"
    if [ ! -f "$PYTHON_GUI_SCRIPT" ]; then
        echo -e "${RED}✗ Python script not found at: $PYTHON_GUI_SCRIPT${NC}"
        echo -e "${YELLOW}Please ensure the pegasus_commander.py script is in the correct location${NC}"
        exit 1
    fi
    # Fallback to direct Python execution
    cd "$(dirname "$PYTHON_GUI_SCRIPT")"
    python3 "$(basename "$PYTHON_GUI_SCRIPT")" --port "${BRIDGE_MODE}" = "bridge" && "$REAL_ARDUINO" || "$VIRTUAL_ARDUINO" 2>&1 | tee "$TMP_LOG" &
    GUI_PID=$!
    echo -e "${BLUE}  Python GUI PID: $GUI_PID${NC}"
fi

echo -e "\n${YELLOW}>>> [5/5] System Status Check...${NC}"

# Give everything a moment to start
sleep 3

# Check if processes are running
if ps -p $SIM_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Bridge running (PID: $SIM_PID)${NC}"
else
    echo -e "${RED}✗ Bridge not running${NC}"
fi

if [ ! -z "$ROS2_PID" ] && ps -p $ROS2_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ ROS2 node running (PID: $ROS2_PID)${NC}"
elif [ ! -z "$GUI_PID" ] && ps -p $GUI_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Python GUI running (PID: $GUI_PID)${NC}"
else
    echo -e "${RED}✗ ROS2 node/Python GUI not running${NC}"
fi

# Check ports
if [ "$BRIDGE_MODE" = "simulation" ]; then
    if [ -e "$VIRTUAL_ARDUINO" ]; then
        echo -e "${GREEN}✓ Virtual Arduino port active${NC}"
    else
        echo -e "${RED}✗ Virtual Arduino port missing${NC}"
    fi

    if [ -e "$MONITOR_PORT" ]; then
        echo -e "${GREEN}✓ Monitor port active${NC}"
    else
        echo -e "${RED}✗ Monitor port missing${NC}"
    fi
else
    if [ -e "$REAL_ARDUINO" ]; then
        echo -e "${GREEN}✓ Real Arduino port active${NC}"
    else
        echo -e "${RED}✗ Real Arduino port missing${NC}"
    fi
fi

echo -e "\n${BLUE}================================================${NC}"
echo -e "${BLUE}    BRIDGE SYSTEM READY${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ Arduino Bridge: Running in ${BRIDGE_MODE} mode${NC}"
echo -e "${GREEN}✓ Enhanced Monitor Terminal: Opened${NC}"
if [ ! -z "$ROS2_PID" ]; then
    echo -e "${GREEN}✓ ROS2 Node: Running${NC}"
else
    echo -e "${GREEN}✓ Python GUI: Running${NC}"
fi
if [ "$BRIDGE_MODE" = "bridge" ]; then
    echo -e "${GREEN}✓ Real Arduino Bridge: Active at $REAL_ARDUINO${NC}"
    echo -e "${CYAN}  Data flows: ROS2 Node → Real Arduino${NC}"
else
    echo -e "${GREEN}✓ Simulator: Active${NC}"
    echo -e "${CYAN}  Data flows: ROS2 Node → Simulator${NC}"
fi
echo -e ""
echo -e "${YELLOW}USAGE INSTRUCTIONS:${NC}"
echo -e "1. In the ROS2 GUI:"
echo -e "   - Go to ${BLUE}Settings${NC} tab"
echo -e "   - ${RED}UNCHECK${NC} 'Enable MoveIt'"
echo -e "   - Move joint sliders or use directional buttons"
echo -e ""
echo -e "2. Monitor activity in the opened terminal"
echo -e ""
echo -e "3. Press ${RED}Ctrl+C${NC} here to stop all processes"
echo -e ""
echo -e "${BLUE}Logs:${NC}"
echo -e "  Simulation/Bridge: $LOG_FILE"
echo -e "  Bridge: $BRIDGE_LOG"
echo -e "  ROS2: $TMP_LOG"
echo -e ""
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo -e "${YELLOW}TO ENABLE ARDUINO BRIDGE:${NC}"
    echo -e "1. Connect Arduino via USB"
    echo -e "2. Fix permissions: ${CYAN}sudo chmod 666 /dev/ttyUSB0${NC}"
    echo -e "3. Restart this launcher"
    echo -e ""
fi
echo -e "${BLUE}================================================${NC}"

# Wait for user to stop or for processes to finish
echo -e "${YELLOW}Bridge system running... Press Ctrl+C to stop all processes${NC}"
if [ ! -z "$ROS2_PID" ]; then
    wait $ROS2_PID
elif [ ! -z "$GUI_PID" ]; then
    wait $GUI_PID
else
    # Fallback - wait for user interrupt
    while true; do
        sleep 1
    done
fi

