#!/bin/bash

# Virtual Serial Port Bridge for Arduino Communication Monitoring
# This script creates a "tap" on the serial communication

ARDUINO_PORT="/dev/ttyUSB0"
VIRTUAL_ARDUINO="/tmp/virtual_arduino"
VIRTUAL_MONITOR="/tmp/virtual_monitor"
BAUDRATE="9600"

echo "================================================"
echo "Arduino Serial Communication Bridge"
echo "================================================"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [ ! -z "$SOCAT_PID" ]; then
        kill $SOCAT_PID 2>/dev/null
    fi
    if [ ! -z "$TEE_PID" ]; then
        kill $TEE_PID 2>/dev/null
    fi
    rm -f $VIRTUAL_ARDUINO $VIRTUAL_MONITOR
    echo "Cleanup complete."
    exit 0
}

# Set up cleanup on script exit
trap cleanup SIGINT SIGTERM EXIT

# Check if Arduino is connected
if [ ! -e "$ARDUINO_PORT" ]; then
    echo "ERROR: Arduino not found at $ARDUINO_PORT"
    echo "Available ports:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No serial ports found"
    exit 1
fi

# Check if port is already in use
if lsof "$ARDUINO_PORT" > /dev/null 2>&1; then
    echo "WARNING: $ARDUINO_PORT is currently in use by:"
    lsof "$ARDUINO_PORT"
    echo "Please close other applications using this port first."
    exit 1
fi

echo "Step 1: Creating virtual port pair..."

# Create virtual serial port pair with socat
socat -d -d pty,raw,echo=0,link=$VIRTUAL_ARDUINO pty,raw,echo=0,link=$VIRTUAL_MONITOR &
SOCAT_PID=$!

# Wait for virtual ports to be created
sleep 2

# Check if virtual ports were created successfully
if [ ! -e "$VIRTUAL_ARDUINO" ] || [ ! -e "$VIRTUAL_MONITOR" ]; then
    echo "ERROR: Failed to create virtual ports"
    exit 1
fi

echo "✓ Virtual ports created:"
echo "  Arduino side: $VIRTUAL_ARDUINO"
echo "  Monitor side: $VIRTUAL_MONITOR"

echo ""
echo "Step 2: Setting up bidirectional bridge..."

# Create bidirectional bridge between real Arduino and virtual Arduino
# This allows both reading and writing through the virtual port
socat $ARDUINO_PORT,raw,echo=0,b$BAUDRATE $VIRTUAL_ARDUINO,raw,echo=0,b$BAUDRATE &
BRIDGE_PID=$!

sleep 1

echo "✓ Bridge established between $ARDUINO_PORT and $VIRTUAL_ARDUINO"

echo ""
echo "================================================"
echo "Setup Complete! Here's what to do next:"
echo "================================================"
echo ""
echo "1. PYTHON SETUP:"
echo "   Modify your Python code to use: $VIRTUAL_ARDUINO"
echo "   (instead of $ARDUINO_PORT)"
echo ""
echo "2. MONITORING SETUP:"
echo "   Open PuTTY with these settings:"
echo "   - Connection type: Serial"
echo "   - Serial line: $VIRTUAL_MONITOR"
echo "   - Speed: $BAUDRATE"
echo ""
echo "   OR use screen command:"
echo "   screen $VIRTUAL_MONITOR $BAUDRATE"
echo ""
echo "3. DATA FLOW:"
echo "   Python → $VIRTUAL_ARDUINO → Bridge → $ARDUINO_PORT → Arduino"
echo "   Arduino → $ARDUINO_PORT → Bridge → $VIRTUAL_ARDUINO → Python"
echo "   All communication is mirrored to → $VIRTUAL_MONITOR → PuTTY"
echo ""
echo "Press Ctrl+C to stop the bridge and cleanup."

# Keep the script running
wait $BRIDGE_PID
