#!/bin/bash
# Integrated Pegasus Bridge System
# Combines Arduino bridge, ROS2 launch, and planned path listener in one terminal

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# Configuration
VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
MONITOR_PORT="/tmp/virtual_monitor_sim"
REAL_ARDUINO="/dev/ttyUSB0"
LOG_FILE="/tmp/pegasus_system.log"
FIFO_PATH="/tmp/pegasus_live_cmd"
PLANNED_PATH_SCRIPT="$HOME/ros2-ws/src/update_pegasus_description/scripts/planned_path_listener.py"

# Process PIDs
SOCAT_PID=""
BRIDGE_PID=""
ROS2_LAUNCH_PID=""
PATH_LISTENER_PID=""
FIFO_FORWARDER_PID=""

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}    INTEGRATED PEGASUS BRIDGE SYSTEM${NC}"
echo -e "${BLUE}================================================${NC}"

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}[CLEANUP] Shutting down all processes...${NC}"
    
    # Kill all background processes
    for pid in $SOCAT_PID $BRIDGE_PID $ROS2_LAUNCH_PID $PATH_LISTENER_PID $FIFO_FORWARDER_PID; do
        if [ ! -z "$pid" ]; then
            kill $pid 2>/dev/null
        fi
    done
    
    # Kill any remaining processes
    pkill -f "pegasus_commander" 2>/dev/null
    pkill -f "planned_path_listener" 2>/dev/null
    pkill -f "demo.launch.py" 2>/dev/null
    
    # Clean up files
    rm -f $VIRTUAL_ARDUINO $MONITOR_PORT $FIFO_PATH $LOG_FILE
    
    echo -e "${GREEN}[CLEANUP] Complete${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Function to log with timestamp and color
log() {
    local color=$1
    local prefix=$2
    local message=$3
    echo -e "${color}[$(date '+%H:%M:%S')] [$prefix] $message${NC}"
}

# Function to check Arduino connection
check_arduino() {
    if [ -e "$REAL_ARDUINO" ]; then
        if [ -w "$REAL_ARDUINO" ] && [ -r "$REAL_ARDUINO" ]; then
            return 0  # Available and accessible
        else
            return 1  # Exists but not accessible
        fi
    else
        return 2  # Not found
    fi
}

# Function to create virtual ports for simulation
create_virtual_ports() {
    log $YELLOW "SETUP" "Creating virtual serial ports..."
    
    rm -f $VIRTUAL_ARDUINO $MONITOR_PORT
    socat -d -d pty,raw,echo=0,link=$VIRTUAL_ARDUINO pty,raw,echo=0,link=$MONITOR_PORT 2>/dev/null &
    SOCAT_PID=$!
    
    sleep 3
    
    if [ ! -e "$VIRTUAL_ARDUINO" ] || [ ! -e "$MONITOR_PORT" ]; then
        log $RED "ERROR" "Failed to create virtual ports"
        exit 1
    fi
    
    log $GREEN "SETUP" "Virtual ports created successfully"
}

# Function to create FIFO for command forwarding
create_fifo() {
    if [ ! -p "$FIFO_PATH" ]; then
        rm -f "$FIFO_PATH" 2>/dev/null
        mkfifo "$FIFO_PATH"
        chmod 666 "$FIFO_PATH"
        log $GREEN "SETUP" "Created command FIFO: $FIFO_PATH"
    fi
}

# Function to start FIFO forwarder
start_fifo_forwarder() {
    local target_port=$1
    
    {
        log $CYAN "FIFO" "Forwarder started"
        while true; do
            if [ -p "$FIFO_PATH" ]; then
                while IFS= read -r line < "$FIFO_PATH"; do
                    line=$(echo "$line" | tr -d '\r\n\0')
                    if [ -z "$line" ]; then
                        continue
                    fi
                    
                    log $MAGENTA "FIFO→ARDUINO" "[$line]"
                    
                    if [ -e "$target_port" ]; then
                        printf '%s\n' "$line" > "$target_port" 2>/dev/null || true
                    fi
                done
            else
                sleep 0.2
            fi
        done
    } &
    FIFO_FORWARDER_PID=$!
}

# Function to parse and forward trajectory data
parse_trajectory() {
    local trajectory_data="$1"
    
    # Extract joint values using grep and awk
    echo "$trajectory_data" | grep -oE "'joint[0-9]+': [0-9.-]+" | while read -r joint_line; do
        joint_name=$(echo "$joint_line" | cut -d"'" -f2)
        joint_value=$(echo "$joint_line" | awk '{print $2}')
        
        # Format as joint_name:value
        formatted_joint="${joint_name}:${joint_value}"
        
        # Send to FIFO if it exists
        if [ -p "$FIFO_PATH" ]; then
            echo "$formatted_joint" > "$FIFO_PATH" 2>/dev/null || true
        fi
        
        log $CYAN "TRAJECTORY" "$formatted_joint"
        sleep 0.1  # Small delay between joints
    done
}

# Function to start planned path listener
start_path_listener() {
    if [ ! -f "$PLANNED_PATH_SCRIPT" ]; then
        log $YELLOW "WARNING" "Planned path listener script not found at: $PLANNED_PATH_SCRIPT"
        return
    fi
    
    log $YELLOW "PATH_LISTENER" "Starting planned path listener..."
    
    # Source ROS2 environment and start listener
    {
        source ~/ros2-ws/install/setup.bash 2>/dev/null
        cd ~/ros2-ws
        python3 "$PLANNED_PATH_SCRIPT" 2>&1 | while IFS= read -r line; do
            if [[ "$line" == *"Trajectory #"* ]]; then
                log $BLUE "PATH_LISTENER" "New trajectory received"
                # Start collecting trajectory data
                trajectory_buffer=""
                collecting=true
            elif [[ "$line" == *"joint"* ]] && [[ "$collecting" == true ]]; then
                # Parse trajectory line and send to Arduino
                parse_trajectory "$line"
            elif [[ "$line" == *"INFO"* ]]; then
                log $CYAN "PATH_LISTENER" "${line#*]: }"
            fi
        done
    } &
    PATH_LISTENER_PID=$!
}

# Function to start Arduino bridge
start_arduino_bridge() {
    local mode=$1
    local port=$2
    
    log $YELLOW "BRIDGE" "Starting Arduino bridge in $mode mode..."
    
    {
        if [ "$mode" = "real" ]; then
            # Configure Arduino port
            stty -F $port 9600 cs8 -cstopb -parenb raw -echo 2>/dev/null
            
            log $GREEN "BRIDGE" "Real Arduino configured at $port"
            
            # Bridge mode: bidirectional communication
            while true; do
                if [ -e "$port" ]; then
                    # Read from Arduino
                    while IFS= read -r -t 0.1 line < "$port"; do
                        line=$(echo "$line" | tr -d '\r\n\0')
                        if [ ! -z "$line" ]; then
                            log $GREEN "ARDUINO→ROS2" "[$line]"
                        fi
                    done
                fi
                sleep 0.05
            done
        else
            # Simulator mode
            log $GREEN "BRIDGE" "Arduino simulator started"
            
            while true; do
                if [ -e "$VIRTUAL_ARDUINO" ]; then
                    while IFS= read -r -t 0.1 line < "$VIRTUAL_ARDUINO"; do
                        line=$(echo "$line" | tr -d '\r\n\0')
                        if [ ! -z "$line" ]; then
                            log $BLUE "ROS2→SIMULATOR" "[$line]"
                            
                            # Simulate Arduino responses
                            if [ "$line" = "PING" ]; then
                                echo "PONG" > $VIRTUAL_ARDUINO
                                log $GREEN "SIMULATOR→ROS2" "[PONG]"
                            elif [[ "$line" =~ ^joint[0-9]+:[0-9.-]+$ ]]; then
                                echo "ACK: Joint command received" > $VIRTUAL_ARDUINO
                                log $GREEN "SIMULATOR→ROS2" "[ACK: Joint command received]"
                            fi
                        fi
                    done
                fi
                sleep 0.05
            done
        fi
    } &
    BRIDGE_PID=$!
}

# Function to start ROS2 launch
start_ros2_launch() {
    log $YELLOW "ROS2" "Starting demo launch..."
    
    {
        source ~/ros2-ws/install/setup.bash 2>/dev/null
        ros2 launch update_pegasus_description demo.launch.py 2>&1 | while IFS= read -r line; do
            if [[ "$line" == *"ERROR"* ]] || [[ "$line" == *"WARN"* ]]; then
                log $RED "ROS2" "$line"
            elif [[ "$line" == *"INFO"* ]]; then
                log $CYAN "ROS2" "${line#*]: }"
            fi
        done
    } &
    ROS2_LAUNCH_PID=$!
}

# Main execution
main() {
    # Source ROS2 environment
    source ~/ros2-ws/install/setup.bash 2>/dev/null
    log $GREEN "SETUP" "ROS2 environment sourced"
    
    # Check Arduino connection
    check_arduino
    arduino_status=$?
    
    case $arduino_status in
        0)
            log $GREEN "SETUP" "Real Arduino found and accessible at: $REAL_ARDUINO"
            BRIDGE_MODE="real"
            ARDUINO_PORT="$REAL_ARDUINO"
            ;;
        1)
            log $YELLOW "SETUP" "Real Arduino found but not accessible - using simulator"
            BRIDGE_MODE="simulation"
            create_virtual_ports
            ARDUINO_PORT="$VIRTUAL_ARDUINO"
            ;;
        2)
            log $YELLOW "SETUP" "Real Arduino not found - using simulator"
            BRIDGE_MODE="simulation"
            create_virtual_ports
            ARDUINO_PORT="$VIRTUAL_ARDUINO"
            ;;
    esac
    
    # Create FIFO for command forwarding
    create_fifo
    
    # Start FIFO forwarder
    start_fifo_forwarder "$ARDUINO_PORT"
    
    # Start ROS2 launch
    start_ros2_launch
    sleep 5  # Wait for ROS2 to initialize
    
    # Start Arduino bridge
    start_arduino_bridge "$BRIDGE_MODE" "$ARDUINO_PORT"
    
    # Start planned path listener
    start_path_listener
    
    # Wait a moment for ROS2 to fully initialize
    sleep 3
    
    # Start GUI
    start_gui "$ARDUINO_PORT"
    
    # System status
    echo -e "${BLUE}================================================${NC}"
    log $GREEN "SYSTEM" "All components started successfully"
    log $BLUE "INFO" "Bridge Mode: $BRIDGE_MODE"
    log $BLUE "INFO" "Arduino Port: $ARDUINO_PORT"
    if [ "$BRIDGE_MODE" = "simulation" ]; then
        log $BLUE "INFO" "Monitor Port: $MONITOR_PORT"
    fi
    echo -e "${BLUE}================================================${NC}"
    
    # Usage instructions
    echo -e "${YELLOW}USAGE:${NC}"
    echo -e "  ${CYAN}• Test connection:${NC} echo 'PING' > $ARDUINO_PORT"
    echo -e "  ${CYAN}• Send joint command:${NC} echo 'joint1:0.5' > $FIFO_PATH"
    echo -e "  ${CYAN}• Monitor activity:${NC} tail -f $LOG_FILE"
    echo -e "  ${CYAN}• Stop system:${NC} Press Ctrl+C"
    echo -e ""
    
    log $GREEN "READY" "System is running - waiting for commands..."
    
    # Keep main process alive
    while true; do
        sleep 5
        
        # Check if critical processes are still running
        if [ ! -z "$BRIDGE_PID" ] && ! ps -p $BRIDGE_PID > /dev/null 2>&1; then
            log $RED "ERROR" "Bridge process died - restarting..."
            start_arduino_bridge "$BRIDGE_MODE" "$ARDUINO_PORT"
        fi
    done
}

# Run main function
main