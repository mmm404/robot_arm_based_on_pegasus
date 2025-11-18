#!/bin/bash
# Test script to verify the virtual Arduino connection

VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
MONITOR_PORT="/tmp/virtual_monitor_sim"

echo "================================================"
echo "Virtual Arduino Connection Test"
echo "================================================"

# Check if ports exist
echo "1. Checking if virtual ports exist..."
if [ -e "$VIRTUAL_ARDUINO" ]; then
    echo "✓ Virtual Arduino port exists: $VIRTUAL_ARDUINO"
    ls -la $VIRTUAL_ARDUINO
else
    echo "✗ Virtual Arduino port NOT found: $VIRTUAL_ARDUINO"
    echo "Make sure the simulator is running first!"
    exit 1
fi

if [ -e "$MONITOR_PORT" ]; then
    echo "✓ Monitor port exists: $MONITOR_PORT"
    ls -la $MONITOR_PORT
else
    echo "✗ Monitor port NOT found: $MONITOR_PORT"
    exit 1
fi

echo ""
echo "2. Testing write access to Arduino port..."
if echo "TEST" > $VIRTUAL_ARDUINO 2>/dev/null; then
    echo "✓ Can write to Arduino port"
else
    echo "✗ Cannot write to Arduino port"
    exit 1
fi

echo ""
echo "3. Testing monitor port..."
echo "Starting monitor in background..."
timeout 5 cat $MONITOR_PORT &
CAT_PID=$!

sleep 1

echo ""
echo "Waiting 3 seconds to let simulator start..."
sleep 3

echo "Sending test commands..."

# Send PING command
echo "PING" > $VIRTUAL_ARDUINO
sleep 1

# Send joint command
echo "joint1_base:1.234,joint2_shoulder:0.567" > $VIRTUAL_ARDUINO
sleep 1

# Wait for cat to finish
wait $CAT_PID

echo ""
echo "4. Test complete!"
echo ""
echo "If you saw responses above, the connection is working."
echo "If not, check:"
echo "- Is the simulator script running?"
echo "- Are there any permission issues?"
echo "- Try running this test as the same user as the simulator"
echo ""
echo "To monitor continuously:"
echo "screen $MONITOR_PORT 9600"
echo "or"
echo "cat $MONITOR_PORT"
