#!/bin/bash
# start_terminals.sh - Open 5 new Terminal windows in the workspace with venv activated

DIR="/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401/drive_c/zWindowsDeployment"

# Web Viewer (Starts automatically)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && python3 mavrik_web_viewer.py\""

# PID Tester (Pre-typed, ready to run after MAVRIK loads)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && clear && echo 'Press ENTER to start the PID VTOL test (Make sure MAVRIK is running first!)' && python3 pid_vtol.py\""

# Optional: Ardupilot Bridge (Pre-typed, but not needed for PID tests)
osascript -e "tell application \"Terminal\" to do script \"cd '$DIR' && source venv/bin/activate && clear && echo 'NOTE: Do NOT run this while pid_vtol.py is running. They will fight for control!' && python3 mavrik_ardupilot_bridge.py\""
