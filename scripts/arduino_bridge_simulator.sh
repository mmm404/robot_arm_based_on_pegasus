#!/bin/bash
# Enhanced Arduino Bridge (Simulator or Direct)
# This script either bridges to a real Arduino or simulates one

VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
MONITOR_PORT="/tmp/virtual_monitor_sim"
REAL_ARDUINO="/dev/ttyUSB0"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"

echo "================================================"
echo "Enhanced Arduino Bridge"
echo "================================================"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down Arduino bridge..."
    if [ ! -z "$SOCAT_PID" ]; then
        kill $SOCAT_PID 2>/dev/null
    fi
    if [ ! -z "$SIM_PID" ]; then
        kill $SIM_PID 2>/dev/null
    fi
    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null
    fi
    if [ ! -z "$TAIL_PID" ]; then
        kill $TAIL_PID 2>/dev/null
    fi
    rm -f $VIRTUAL_ARDUINO $MONITOR_PORT $LOG_FILE $BRIDGE_LOG
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "Step 1: Select Arduino connection mode..."
echo "1. Use real Arduino"
echo "2. Use simulator"
read -p "Enter choice (1 or 2): " choice

case $choice in
    1)
        # Check for real Arduino
        echo "Checking for real Arduino at $REAL_ARDUINO..."
        if [ ! -e "$REAL_ARDUINO" ]; then
            echo "WARNING: Real Arduino not found at $REAL_ARDUINO"
            echo "Available USB devices:"
            ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"
            read -p "Enter custom Arduino port (or press Enter to use simulator): " custom_port
            if [ ! -z "$custom_port" ]; then
                REAL_ARDUINO="$custom_port"
                if [ ! -e "$REAL_ARDUINO" ]; then
                    echo "ERROR: Custom port $REAL_ARDUINO not found, falling back to simulator"
                    BRIDGE_MODE="simulation"
                else
                    BRIDGE_MODE="bridge"
                fi
            else
                BRIDGE_MODE="simulation"
            fi
        else
            BRIDGE_MODE="bridge"
        fi
        ;;
    2)
        BRIDGE_MODE="simulation"
        ;;
    *)
        echo "Invalid choice, defaulting to simulator"
        BRIDGE_MODE="simulation"
        ;;
esac

# Verify Arduino accessibility if in bridge mode
if [ "$BRIDGE_MODE" = "bridge" ]; then
    if [ -w "$REAL_ARDUINO" ] && [ -r "$REAL_ARDUINO" ]; then
        echo "✓ Real Arduino found and accessible at $REAL_ARDUINO"
        # Configure Arduino serial port
        stty -F $REAL_ARDUINO 9600 cs8 -cstopb -parenb raw -echo
        echo "✓ Arduino serial port configured (9600 8N1)"
    else
        echo "WARNING: Real Arduino found but not accessible (permission issue?)"
        echo "Try: sudo chmod 666 $REAL_ARDUINO"
        echo "Or add user to dialout group: sudo usermod -a -G dialout $USER"
        BRIDGE_MODE="simulation"
    fi
fi

# If in bridge mode, skip virtual ports and use real Arduino directly
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo ""
    echo "Step 2: Creating virtual serial ports..."

    # Remove old ports if they exist
    rm -f $VIRTUAL_ARDUINO $MONITOR_PORT

    # Create virtual port pair
    socat -d -d pty,raw,echo=0,link=$VIRTUAL_ARDUINO pty,raw,echo=0,link=$MONITOR_PORT &
    SOCAT_PID=$!

    echo "Waiting for ports to be created..."
    sleep 3

    # Verify ports exist
    if [ ! -e "$VIRTUAL_ARDUINO" ] || [ ! -e "$MONITOR_PORT" ]; then
        echo "ERROR: Failed to create virtual ports"
        echo "Debug info:"
        ls -la /tmp/virtual_* 2>/dev/null || echo "No virtual ports found"
        exit 1
    fi

    echo "✓ Virtual ports created successfully:"
    echo "  ROS2 node connects to: $VIRTUAL_ARDUINO"
    echo "  Monitor connects to: $MONITOR_PORT"
else
    echo "✓ Using real Arduino at: $REAL_ARDUINO"
fi

echo ""
echo "Step 3: Starting Arduino bridge..."

# Arduino bridge function
start_bridge() {
    echo "$(date '+%H:%M:%S') Arduino bridge started in $BRIDGE_MODE mode" | tee -a $BRIDGE_LOG
    
    if [ "$BRIDGE_MODE" = "bridge" ]; then
        # Direct bridge to real Arduino
        while true; do
            if [ -e "$REAL_ARDUINO" ]; then
                while IFS= read -r -t 0.1 line; do
                    # Clean the line
                    line=$(echo "$line" | tr -d '\r\n\0' | sed 's/[[:space:]]*$//')
                    
                    if [ ! -z "$line" ]; then
                        timestamp=$(date '+%H:%M:%S')
                        echo "$timestamp RX: [$line]" >> $BRIDGE_LOG
                        
                        # Send to real Arduino
                        if echo "$line" > $REAL_ARDUINO 2>/dev/null; then
                            bridge_msg="$timestamp BRIDGE->ARDUINO: [$line]"
                            echo "$bridge_msg" | tee -a $BRIDGE_LOG
                            echo "$bridge_msg" > $MONITOR_PORT
                            sync
                        else
                            error_msg="$timestamp BRIDGE ERROR: Failed to send to Arduino"
                            echo "$error_msg" | tee -a $BRIDGE_LOG
                            echo "$error_msg" > $MONITOR_PORT
                            sync
                        fi
                    fi
                done < $REAL_ARDUINO
            fi
            sleep 0.05
        done
    else
        # Simulator mode
        while true; do
            if [ -e "$VIRTUAL_ARDUINO" ]; then
                while IFS= read -r -t 0.1 line; do
                    # Clean the line
                    line=$(echo "$line" | tr -d '\r\n\0' | sed 's/[[:space:]]*$//')
                    
                    if [ ! -z "$line" ]; then
                        timestamp=$(date '+%H:%M:%S')
                        echo "$timestamp RX: [$line]" | tee -a $LOG_FILE
                        
                        # Send to monitor port
                        if [[ "$line" != *"RX:"* ]] && [[ "$line" != *"BRIDGE"* ]] && [[ "$line" != *"ERROR:"* ]]; then
                            echo "$timestamp RX: [$line]" | tee $MONITOR_PORT
                        fi
                        
                        sync
                        
                        # Process commands for simulation responses
                        if [ "$line" = "PING" ]; then
                            response="PONG"
                            echo "$response" > $VIRTUAL_ARDUINO
                            response_msg="$timestamp TX: $response"
                            echo "$response_msg" | tee -a $LOG_FILE
                            echo "$response_msg" | tee $MONITOR_PORT
                            sync
                            
                        elif [[ "$line" =~ ^joint[0-9]+_[a-z]+:[0-9.-]+(,joint[0-9]+_[a-z]+:[0-9.-]+)*$ ]]; then
                            process_msg="$timestamp Processing joints: $line"
                            echo "$process_msg" | tee -a $LOG_FILE
                            echo "$process_msg" | tee $MONITOR_PORT
                            sync
                            
                            # Parse joint values
                            IFS=',' read -ra JOINTS <<< "$line"
                            valid_joints=true
                            for joint in "${JOINTS[@]}"; do
                                if [[ "$joint" =~ ^joint[0-9]+_[a-z]+:[0-9.-]+$ ]]; then
                                    name=$(echo "$joint" | cut -d':' -f1)
                                    value=$(echo "$joint" | cut -d':' -f2)
                                    joint_info="  $name = $value rad"
                                    joint_msg="$timestamp $joint_info"
                                    echo "$joint_msg" | tee -a $LOG_FILE
                                    echo "$joint_msg" | tee $MONITOR_PORT
                                else
                                    valid_joints=false
                                    error_msg="$timestamp Invalid joint format: $joint"
                                    echo "$error_msg" | tee -a $LOG_FILE
                                    echo "$error_msg" | tee $MONITOR_PORT
                                fi
                            done
                            sync
                            
                            # Send ACK response if all joints are valid
                            if [ "$valid_joints" = true ]; then
                                ack_response="ACK: Joints received"
                                echo "$ack_response" > $VIRTUAL_ARDUINO
                                ack_msg="$timestamp TX: $ack_response"
                                echo "$ack_msg" | tee -a $LOG_FILE
                                echo "$ack_msg" | tee $MONITOR_PORT
                                sync
                            else
                                error_response="ERROR: Invalid joint command format"
                                echo "$error_response" > $VIRTUAL_ARDUINO
                                error_msg="$timestamp TX: $error_response"
                                echo "$error_msg" | tee -a $LOG_FILE
                                echo "$error_msg" | tee $MONITOR_PORT
                                sync
                            fi
                            
                        else
                            error_msg="$timestamp Unknown command: [$line]"
                            echo "$error_msg" | tee -a $LOG_FILE
                            echo "$error_msg" | tee $MONITOR_PORT
                            error_response="ERROR: Unknown command"
                            echo "$error_response" > $VIRTUAL_ARDUINO
                            error_msg="$timestamp TX: $error_response"
                            echo "$error_msg" | tee -a $LOG_FILE
                            echo "$error_msg" | tee $MONITOR_PORT
                            sync
                        fi
                    fi
                done < $VIRTUAL_ARDUINO
            fi
            sleep 0.05
        done
    fi
}

# Start the bridge in background
start_bridge &
SIM_PID=$!

echo "✓ Arduino bridge started (PID: $SIM_PID)"
echo ""

# Start Arduino response listener if in bridge mode
if [ "$BRIDGE_MODE" = "bridge" ]; then
    echo "Step 4: Starting Arduino response listener..."
    
    # Listen for responses from real Arduino
    (
        echo "$(date '+%H:%M:%S') Arduino response listener started" | tee -a $BRIDGE_LOG
        while true; do
            if [ -r "$REAL_ARDUINO" ]; then
                while IFS= read -r -t 1 arduino_response < $REAL_ARDUINO; do
                    if [ ! -z "$arduino_response" ]; then
                        timestamp=$(date '+%H:%M:%S')
                        arduino_response=$(echo "$arduino_response" | tr -d '\r\n\0' | sed 's/[[:space:]]*$//')
                        echo "$timestamp ARDUINO->BRIDGE: [$arduino_response]" | tee -a $BRIDGE_LOG
                        echo "$timestamp ARDUINO->BRIDGE: [$arduino_response]" > $MONITOR_PORT 2>/dev/null
                    fi
                done
            fi
            sleep 0.1
        done
    ) &
    BRIDGE_PID=$!
    echo "✓ Arduino response listener started (PID: $BRIDGE_PID)"
    echo ""
fi

echo "================================================"
echo "DEBUGGING INFORMATION"
echo "================================================"
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "Virtual Arduino port: $VIRTUAL_ARDUINO"
    echo "Monitor port: $MONITOR_PORT"
else
    echo "Real Arduino port: $REAL_ARDUINO"
fi
echo "Mode: $BRIDGE_MODE"
echo "Simulation log: $LOG_FILE"
echo "Bridge log: $BRIDGE_LOG"
echo ""
echo "Port details:"
if [ "$BRIDGE_MODE" = "simulation" ]; then
    ls -la $VIRTUAL_ARDUINO $MONITOR_PORT 2>/dev/null
else
    ls -la $REAL_ARDUINO 2>/dev/null
fi
echo ""

echo "================================================"
echo "CONNECTION TESTING"
echo "================================================"

# Test monitor port or Arduino
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "Testing monitor port..."
    if echo "TEST: Monitor port working" > $MONITOR_PORT 2>/dev/null; then
        echo "✓ Monitor port is accessible"
    else
        echo "✗ Monitor port has issues"
    fi
else
    echo "Testing Arduino connection..."
    if echo "TEST: Bridge working" > $REAL_ARDUINO 2>/dev/null; then
        echo "✓ Arduino connection is working"
    else
        echo "✗ Arduino connection has issues"
    fi
fi

echo ""
echo "================================================"
echo "SETUP INSTRUCTIONS"
echo "================================================"
echo ""
echo "1. ROS2 NODE CONFIGURATION:"
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "   Ensure your ROS2 node uses: portname = '$VIRTUAL_ARDUINO'"
else
    echo "   Ensure your ROS2 node uses: portname = '$REAL_ARDUINO'"
fi
echo ""
echo "2. MONITORING DATA:"
echo ""
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "   METHOD 1 - Screen (most reliable):"
    echo "   screen $MONITOR_PORT 9600"
    echo "   (Press Ctrl+A then K to exit)"
    echo ""
    echo "   METHOD 2 - Simple cat:"
    echo "   cat $MONITOR_PORT"
    echo ""
    echo "   METHOD 3 - PuTTY:"
    echo "   - Connection Type: Serial"
    echo "   - Serial Line: $MONITOR_PORT"
    echo "   - Speed: 9600"
else
    echo "   Monitor Arduino responses in the opened terminal"
fi
echo ""
echo "3. MONITORING BRIDGE ACTIVITY:"
echo "   tail -f $BRIDGE_LOG"
echo ""
echo "4. ROS2 NODE SETUP:"
echo "   - Start your ROS2 node with 'ros2 run pegasus_arm_commander pegasus_commander'"
echo "   - Go to Settings tab in the GUI"
echo "   - UNCHECK 'Enable MoveIt'"
echo "   - Move joint sliders"
echo ""
echo "5. QUICK TEST:"
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "   In another terminal: echo 'PING' > $VIRTUAL_ARDUINO"
else
    echo "   In another terminal: echo 'PING' > $REAL_ARDUINO"
fi
echo "   Data will appear in monitor"
echo ""

if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "================================================"
    echo "ARDUINO CONNECTION HELP"
    echo "================================================"
    echo "To enable Arduino bridge:"
    echo "1. Connect Arduino via USB"
    echo "2. Check connection: ls -la /dev/ttyUSB* /dev/ttyACM*"
    echo "3. Fix permissions: sudo chmod 666 /dev/ttyUSB0"
    echo "4. Or add to group: sudo usermod -a -G dialout $USER"
    echo "5. Restart this script"
    echo ""
fi

echo "================================================"
echo "BRIDGE RUNNING"
echo "================================================"
echo "✓ Ready for connections"
echo "✓ Logging to: $LOG_FILE"
if [ "$BRIDGE_MODE" = "bridge" ]; then
    echo "✓ Bridging to Arduino: $REAL_ARDUINO"
    echo "✓ Bridge logging to: $BRIDGE_LOG"
fi
echo "✓ Press Ctrl+C to stop"
echo ""

# Show real-time log output only in simulation mode
if [ "$BRIDGE_MODE" = "simulation" ]; then
    echo "Real-time activity (waiting for ROS2 node connection):"
    echo "---------------------------------------------------"
    tail -f $LOG_FILE &
    TAIL_PID=$!
fi

# Keep the script running
wait $SIM_PID

