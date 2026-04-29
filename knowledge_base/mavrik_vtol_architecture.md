# MAVRIK VTOL Controller & Web Viewer Architecture

## Core Components
1. **`pid_vtol.py`**: The main Python-based PID controller.
   - Operates a cascade-through-flaps architecture.
   - Binds to UDP port 5007 to receive MAVRIK State telemetry.
   - Mixes 9 effectors (4 rotors, 4 flaps, 1 pusher).
   - Solved yaw instability in hover by implementing **differential tilt** (forward motors tilt independently for yaw authority).
2. **`mavrik_web_viewer.py`**: A native, zero-dependency browser GCS (Ground Control Station).
   - Replaced QGroundControl due to 3D terrain rendering issues and heavy dependencies.
   - Uses `ThreadingHTTPServer` to host an HTML/Three.js web app.
   - Listens to MAVRIK UDP port 5005 for telemetry.
   - Streams telemetry via Server-Sent Events (SSE) at 30Hz.
   - Includes a Heading-Up Minimap, Pitch/Roll/Yaw live HTML5 Canvas graph, and CSS Altimeter/Speed tapes.
3. **`mavrik_qgc_bridge.py`**: Handles external MAVLink routing (mainly used for 2D map in QGC).

## Key Discoveries & Workarounds
- **Whisky/Wine Networking**: Background processes (`mavrik.exe`, `wineserver`) can easily zombie and hold UDP ports hostage. Always run `killall -9 mavrik.exe wineserver python3.11 Python` if network binding fails.
- **Floating Point Math Crashes**: MAVRIK's physics engine occasionally throws `IEEE_DENORMAL` or spits out `NaN`/`Infinity`. If this hits `json.dumps()` in Python, it crashes the Javascript `JSON.parse` parser on the frontend. We added clamping and math sanitization before sending data over SSE to prevent fatal freezes.
- **Target Altitude**: Changed from 1000 ft to 250 ft to keep the drone closer to ground for visibility and better physics interactions during testing.

## Future Exploration (Web GCS)
As verified, building a fully web-based alternative to Mission Planner / QGC is completely viable. Browsers support the **HTML5 Gamepad API** (to read Radiomaster Pocket inputs directly) and **WebSerial** (to interact with actual radios). The next step in this architecture is routing Radiomaster inputs from the browser back into the Python PID override via WebSocket.
