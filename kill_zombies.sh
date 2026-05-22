#!/bin/bash
# kill_zombies.sh — Kill leftover simulation processes hogging UDP/TCP ports

echo "Looking for zombie simulation processes..."
# List of process patterns to kill
# Note: "wine" and "wineserver" intentionally omitted — killing the whole Wine/Whisky
# session corrupts Wine state for the next MAVRIK launch. Only kill MAVRIK.exe itself.
PROCESSES=("pid_vtol.py" "mavrik_ardupilot_bridge.py" "mavlink_ws_bridge.py" "mavproxy.py" "arducopter" "ArduPlane" "ArduCopter" "mavrik.exe")

for pattern in "${PROCESSES[@]}"; do
    PIDS=$(pgrep -f "$pattern" 2>/dev/null)
    if [ ! -z "$PIDS" ]; then
        echo "  Killing $pattern PIDs: $PIDS"
        kill -9 $PIDS 2>/dev/null
    fi
done

# Also check if commonly used ports are still held
echo ""
echo "Checking simulation ports (5000, 5006, 5007, 5009, 5760, 5762, 5763, 9002, 14550, 14552, 5501)..."
lsof -i :5000 -i :5006 -i :5007 -i :5009 -i :5760 -i :5762 -i :5763 -i :9002 -i :14550 -i :14552 -i :5501 2>/dev/null | grep -v "^COMMAND" | while read line; do
    PID=$(echo "$line" | awk '{print $2}')
    CMD=$(echo "$line" | awk '{print $1}')
    echo "  Killing $CMD (PID $PID) on port"
    kill -9 "$PID" 2>/dev/null
done
echo "Ports clear. Ready to run."

# Close only the Terminal tabs/windows opened by start_terminals.sh
echo "Closing MAVRIK Terminal windows..."
osascript -e 'tell application "Terminal"
    repeat with w in windows
        try
            close (every tab of w whose custom title is "MAVRIK_PROCESS")
        end try
    end repeat
end tell' 2>/dev/null

echo "Deleting eeprom.bin to ensure latest parameters are loaded..."
rm -f eeprom.bin
echo "Done."
