#!/bin/bash
# Enhanced Virtual Arduino Simulator with Real Arduino Bridge
# This script simulates an Arduino, provides monitoring, AND bridges to real Arduino

VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
MONITOR_PORT="/tmp/virtual_monitor_sim"
REAL_ARDUINO="/dev/ttyUSB0"
LOG_FILE="/tmp/arduino_sim.log"
BRIDGE_LOG="/tmp/arduino_bridge.log"

echo "================================================"
echo "Enhanced Virtual Arduino Simulator with Bridge"
echo "================================================"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down virtual Arduino simulator and bridge..."
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

echo "Step 1: Checking real Arduino connection..."

# Check if real Arduino exists and is accessible
if [ ! -e "$REAL_ARDUINO" ]; then
    echo "WARNING: Real Arduino not found at $REAL_ARDUINO"
    echo "Available USB devices:"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"
    echo ""
    echo "Bridge will run in simulation-only mode"
    BRIDGE_MODE="simulation"
else
    # Test if we can access the Arduino
    if [ -w "$REAL_ARDUINO" ] && [ -r "$REAL_ARDUINO" ]; then
        echo "✓ Real Arduino found and accessible at $REAL_ARDUINO"
        BRIDGE_MODE="bridge"
        
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
echo "  Python connects to: $VIRTUAL_ARDUINO"
echo "  Monitor connects to: $MONITOR_PORT"
echo "  Bridge mode: $BRIDGE_MODE"
echo ""

echo "Step 3: Starting Arduino bridge..."

# Arduino bridge function
start_bridge() {
    echo "$(date '+%H:%M:%S') Arduino bridge started in $BRIDGE_MODE mode" | tee -a $BRIDGE_LOG
    
    while true; do
        if [ -e "$VIRTUAL_ARDUINO" ]; then
            # Read from the virtual arduino port with improved buffering
            while IFS= read -r -t 0.1 line; do
                # Clean the line
                line=$(echo "$line" | tr -d '\r\n\0' | sed 's/[[:space:]]*$//')
                
                if [ ! -z "$line" ]; then
                    timestamp=$(date '+%H:%M:%S')
                    echo "$timestamp RX: [$line]" | tee -a $LOG_FILE
                    
                    # Send to monitor port immediately with flush
                    if [[ "$line" != *"RX:"* ]] && [[ "$line" != *"BRIDGE"* ]]; then
                        echo "$timestamp RX: [$line]" | tee $MONITOR_PORT
                    fi
                    
                    # Force flush the monitor port
                    sync
                    
                    # BRIDGE: Send to real Arduino if available
                    if [ "$BRIDGE_MODE" = "bridge" ]; then
                        if echo "$line" > $REAL_ARDUINO 2>/dev/null; then
                            bridge_msg="$timestamp BRIDGE->ARDUINO: [$line]"
                            echo "$bridge_msg" | tee -a $BRIDGE_LOG
                            echo "$bridge_msg" | tee $MONITOR_PORT
                            sync
                        else
                            error_msg="$timestamp BRIDGE ERROR: Failed to send to Arduino"
                            echo "$error_msg" | tee -a $BRIDGE_LOG
                            echo "$error_msg" | tee $MONITOR_PORT
                            sync
                        fi
                    fi
                    
                    # Process commands for simulation responses
                    if [ "$line" = "PING" ]; then
                        response="PONG"
                        echo "$response" > $VIRTUAL_ARDUINO
                        response_msg="$timestamp TX: $response"
                        echo "$response_msg" | tee -a $LOG_FILE
                        echo "$response_msg" | tee $MONITOR_PORT
                        sync
                        
                    elif [[ "$line" == *"joint"* ]]; then
                        # Process joint command
                        process_msg="$timestamp Processing joints: $line"
                        echo "$process_msg" | tee -a $LOG_FILE
                        echo "$process_msg" | tee $MONITOR_PORT
                        sync
                        
                        # Parse joint values
                        IFS=',' read -ra JOINTS <<< "$line"
                        for joint in "${JOINTS[@]}"; do
                            if [[ "$joint" == *":"* ]]; then
                                name=$(echo "$joint" | cut -d':' -f1)
                                value=$(echo "$joint" | cut -d':' -f2)
                                joint_info="  $name = $value rad"
                                joint_msg="$timestamp $joint_info"
                                echo "$joint_msg" | tee -a $LOG_FILE
                                echo "$joint_msg" | tee $MONITOR_PORT
                                
                                # BRIDGE: Also log joint data being sent to Arduino
                                if [ "$BRIDGE_MODE" = "bridge" ]; then
                                    arduino_joint_msg="$timestamp ARDUINO_JOINT: $joint_info"
                                    echo "$arduino_joint_msg" | tee -a $BRIDGE_LOG
                                    echo "$arduino_joint_msg" | tee $MONITOR_PORT
                                fi
                            fi
                        done
                        sync
                        
                        # Send ACK response
                        ack_response="ACK: Joints received"
                        echo "$ack_response" > $VIRTUAL_ARDUINO
                        ack_msg="$timestamp TX: $ack_response"
                        echo "$ack_msg" | tee -a $LOG_FILE
                        echo "$ack_msg" | tee $MONITOR_PORT
                        sync
                        
                    else
                        unknown_msg="$timestamp Unknown command: [$line]"
                        echo "$unknown_msg" | tee -a $LOG_FILE
                        echo "$unknown_msg" | tee $MONITOR_PORT
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
}

# Start the bridge in background
start_bridge &
SIM_PID=$!

echo "✓ Virtual Arduino bridge started (PID: $SIM_PID)"
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
echo "Virtual Arduino port: $VIRTUAL_ARDUINO"
echo "Monitor port: $MONITOR_PORT"
echo "Real Arduino port: $REAL_ARDUINO (mode: $BRIDGE_MODE)"
echo "Simulation log: $LOG_FILE"
echo "Bridge log: $BRIDGE_LOG"
echo ""
echo "Port details:"
ls -la $VIRTUAL_ARDUINO $MONITOR_PORT 2>/dev/null
if [ "$BRIDGE_MODE" = "bridge" ]; then
    ls -la $REAL_ARDUINO 2>/dev/null
fi
echo ""

echo "================================================"
echo "CONNECTION TESTING"
echo "================================================"

# Test monitor port
echo "Testing monitor port..."
if echo "TEST: Monitor port working" > $MONITOR_PORT 2>/dev/null; then
    echo "✓ Monitor port is accessible"
else
    echo "✗ Monitor port has issues"
fi

# Test Arduino bridge if available
if [ "$BRIDGE_MODE" = "bridge" ]; then
    echo "Testing Arduino bridge..."
    if echo "TEST: Bridge working" > $REAL_ARDUINO 2>/dev/null; then
        echo "✓ Arduino bridge is working"
    else
        echo "✗ Arduino bridge has issues"
    fi
fi

echo ""
echo "================================================"
echo "SETUP INSTRUCTIONS"
echo "================================================"
echo ""
echo "1. PYTHON CONFIGURATION:"
echo "   Ensure your Python code uses: portname = '$VIRTUAL_ARDUINO'"
echo ""
echo "2. MONITORING DATA:"
echo ""
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
echo ""
echo "3. MONITORING BRIDGE ACTIVITY:"
echo "   tail -f $BRIDGE_LOG"
echo ""
echo "4. PYTHON GUI SETUP:"
echo "   - Start your Python GUI"
echo "   - Go to Settings tab"
echo "   - UNCHECK 'Enable MoveIt'"
echo "   - Move joint sliders"
echo ""
echo "5. QUICK TEST:"
echo "   In another terminal: echo 'PING' > $VIRTUAL_ARDUINO"
echo "   Data will appear in monitor AND be sent to Arduino"
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
echo "SIMULATOR RUNNING"
echo "================================================"
echo "✓ Ready for connections"
echo "✓ Logging to: $LOG_FILE"
if [ "$BRIDGE_MODE" = "bridge" ]; then
    echo "✓ Bridging to Arduino: $REAL_ARDUINO"
    echo "✓ Bridge logging to: $BRIDGE_LOG"
fi
echo "✓ Press Ctrl+C to stop"
echo ""

# Show real-time log output
echo "Real-time activity (waiting for Python connection):"
echo "---------------------------------------------------"
tail -f $LOG_FILE &
TAIL_PID=$!

# Keep the script running
wait $SIM_PID
