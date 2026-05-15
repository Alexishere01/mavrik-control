#!/bin/bash
# start_terminals.sh - Open 5 new Terminal windows in the workspace with venv activated

DIR="/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment"

# Web Viewer (Starts automatically)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && python3 mavrik_web_viewer.py\""

# Pre-arm PID stabilizer — must start BEFORE the bridge so it owns port 5006 during pre-arm.
# Bridge sends a UDP byte to port 5015 on handover; pid_vtol then exits cleanly.
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && clear && python3 pid_vtol.py\""

# Ardupilot Bridge (Starts automatically)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && clear && python3 mavrik_ardupilot_bridge.py\""

# Ardupilot SITL (Starts automatically)
# --wipe-eeprom: always start fresh from mavrik.parm, never load stale previous-session params.
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && clear && echo 'Starting Ardupilot SITL with JSON backend...' && sim_vehicle.py -v ArduPlane -f quadplane --model JSON:127.0.0.1 --add-param-file=mavrik.parm --out udpin:127.0.0.1:14552 --out udpin:127.0.0.1:14556 --wipe-eeprom --map --console\""

# MAVLink WebSocket Bridge (Connects web GCS to SITL)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && clear && ./venv/bin/python3 mavlink_ws_bridge.py\""

# MAVRIK Physics Engine (Starts automatically after a short delay)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && echo 'Waiting 8 seconds for SITL stack to initialize...' && sleep 8 && clear && export WINEPREFIX='/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401' && export WINEDEBUG='-all' && '/Users/alex/Library/Application Support/com.isaacmarovitz.Whisky/Libraries/Wine/bin/wine64' mavrik.exe input_Team1_hover.json\""
