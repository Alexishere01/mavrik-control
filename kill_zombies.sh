#!/bin/bash
# kill_zombies.sh — Kill leftover simulation processes hogging UDP/TCP ports

echo "Looking for zombie simulation processes..."
# List of process patterns to kill
PROCESSES=("pid_vtol.py" "mavrik_ardupilot_bridge.py" "mavrik_web_viewer.py" "mavproxy.py" "arducopter" "mavrik.exe")

for pattern in "${PROCESSES[@]}"; do
    PIDS=$(pgrep -f "$pattern" 2>/dev/null)
    if [ ! -z "$PIDS" ]; then
        echo "  Killing $pattern PIDs: $PIDS"
        kill -9 $PIDS 2>/dev/null
    fi
done

# Also check if commonly used ports are still held
echo ""
echo "Checking simulation ports (5000, 5006, 5007, 5760, 5762, 5763, 9002)..."
lsof -i :5000 -i :5006 -i :5007 -i :5760 -i :5762 -i :5763 -i :9002 2>/dev/null | grep -v "^COMMAND" | while read line; do
    PID=$(echo "$line" | awk '{print $2}')
    CMD=$(echo "$line" | awk '{print $1}')
    echo "  Killing $CMD (PID $PID) on port"
    kill -9 "$PID" 2>/dev/null
done
echo "Ports clear. Ready to run."
