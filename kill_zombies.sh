#!/bin/bash
# kill_zombies.sh — Kill leftover pid_vtol.py processes hogging UDP ports
echo "Looking for zombie pid_vtol processes..."
PIDS=$(pgrep -f "pid_vtol.py" 2>/dev/null)
if [ -z "$PIDS" ]; then
    echo "  No pid_vtol.py processes found."
else
    echo "  Killing PIDs: $PIDS"
    kill -9 $PIDS
    echo "  Done."
fi

# Also check if ports 5006/5007 are still held
echo ""
echo "Checking ports 5006/5007..."
lsof -i :5006 -i :5007 2>/dev/null | grep -v "^COMMAND" | while read line; do
    PID=$(echo "$line" | awk '{print $2}')
    CMD=$(echo "$line" | awk '{print $1}')
    echo "  Killing $CMD (PID $PID) on port"
    kill -9 "$PID" 2>/dev/null
done
echo "Ports clear. Ready to run."
