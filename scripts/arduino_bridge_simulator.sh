#!/usr/bin/env bash
#
# arduino_bridge_simulator.sh
# Monitor Arduino (real or simulated) instead of bridging FIFO.
# Python node sends trajectories directly now.

set -e

VIRTUAL_ARDUINO="/tmp/virtual_arduino_sim"
MONITOR_PORT="/tmp/virtual_monitor_sim"
LOGFILE="/tmp/arduino_bridge.log"

MODE=$1   # "sim" or "real"
ARDUINO_PORT=${2:-/dev/ttyUSB0}
BAUD=115200   # match Arduino sketch

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

if [[ "$MODE" == "sim" ]]; then
  echo "[$(timestamp)] 🔄 Starting in SIMULATION mode..." | tee -a "$LOGFILE"
  
  # Clean up old ports
  rm -f "$VIRTUAL_ARDUINO" "$MONITOR_PORT"
  
  # Create virtual serial port pair
  socat -d -d pty,raw,echo=0,link="$VIRTUAL_ARDUINO" \
              pty,raw,echo=0,link="$MONITOR_PORT" 2>&1 | tee -a "$LOGFILE" &
  SOCAT_PID=$!
  
  # Wait for ports to be created
  sleep 2
  
  echo "[$(timestamp)] Created virtual ports: $VIRTUAL_ARDUINO <-> $MONITOR_PORT" | tee -a "$LOGFILE"
  echo "[$(timestamp)] ✅ You can point your Python node to $VIRTUAL_ARDUINO" | tee -a "$LOGFILE"

  # Monitor simulated Arduino responses
  if [ -e "$MONITOR_PORT" ]; then
    cat "$MONITOR_PORT" 2>/dev/null | tee -a "$LOGFILE" &
    CAT_PID=$!
  fi

  # Wait for interrupt
  wait $SOCAT_PID

  # Cleanup
  [ ! -z "$CAT_PID" ] && kill $CAT_PID 2>/dev/null || true
  rm -f "$VIRTUAL_ARDUINO" "$MONITOR_PORT"

elif [[ "$MODE" == "real" ]]; then
  echo "[$(timestamp)] 🔌 Starting in REAL ARDUINO mode..." | tee -a "$LOGFILE"
  echo "[$(timestamp)] Target device: $ARDUINO_PORT @ ${BAUD} baud" | tee -a "$LOGFILE"
  
  # Check if port exists
  if [ ! -e "$ARDUINO_PORT" ]; then
    echo "[$(timestamp)] ❌ ERROR: Arduino port $ARDUINO_PORT not found!" | tee -a "$LOGFILE"
    exit 1
  fi
  
  # Configure serial port
  stty -F "$ARDUINO_PORT" $BAUD raw -echo 2>&1 | tee -a "$LOGFILE"

  # Monitor Arduino serial output
  cat "$ARDUINO_PORT" 2>&1 | tee -a "$LOGFILE"

else
  echo "Usage: $0 {sim|real} [arduino_port]"
  echo "Examples:"
  echo "  $0 sim                    # Start in simulation mode"
  echo "  $0 real /dev/ttyUSB0      # Connect to real Arduino"
  exit 1
fi
