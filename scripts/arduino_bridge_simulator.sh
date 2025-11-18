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
  echo "[$(timestamp)] 🔄 Starting in SIMULATION mode..."
  rm -f "$VIRTUAL_ARDUINO" "$MONITOR_PORT"
  socat -d -d pty,raw,echo=0,link=$VIRTUAL_ARDUINO \
              pty,raw,echo=0,link=$MONITOR_PORT &
  SOCAT_PID=$!
  echo "[$(timestamp)] Created virtual ports: $VIRTUAL_ARDUINO <-> $MONITOR_PORT"
  echo "[$(timestamp)] ✅ You can point your Python node to $VIRTUAL_ARDUINO"

  # Just monitor simulated Arduino responses
  cat $MONITOR_PORT

  kill $SOCAT_PID

elif [[ "$MODE" == "real" ]]; then
  echo "[$(timestamp)] 🔌 Starting in REAL ARDUINO mode..."
  echo "[$(timestamp)] Target device: $ARDUINO_PORT @ ${BAUD} baud"
  stty -F $ARDUINO_PORT $BAUD raw -echo

  # Just monitor Arduino serial output
  cat $ARDUINO_PORT | tee -a $LOGFILE

else
  echo "Usage: $0 {sim|real} [arduino_port]"
  exit 1
fi

