#!/usr/bin/env python3
import struct
import json
import asyncio
import math
import os
import time
import socket
from aiohttp import web
import hashlib
import glob
import subprocess
import datetime

# MAVRIK's Graphics send port from connections.json
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
HTTP_PORT = 8080

# 14 floats: t, u, v, w, p, q, r, xf, yf, zf, e0, ex, ey, ez
MAVRIK_FMT = "<14f"
MAVRIK_SIZE = struct.calcsize(MAVRIK_FMT)

# Global state holder
latest_frame = {
    "type": "frame",
    "t": 0,
    "state": {
        "t": 0, "u": 0, "v": 0, "w": 0,
        "p": 0, "q": 0, "r": 0,
        "x": 0, "y": 0, "z": 0,
        "e0": 1, "ex": 0, "ey": 0, "ez": 0
    },
    "actuators": {
        "aileron": 0, "elevator": 0, "rudder": 0,
        "thrFR": 0, "thrFL": 0, "thrAR": 0, "thrAL": 0,
        "flapsRight": 0, "flapsLeft": 0, "flaps": 0
    },
    "setpoint": None
}

# List of client queues
client_queues = []

# Recording state
os.makedirs("recordings", exist_ok=True)
last_rec_t = -1
current_recording = None

# Stats tracking
stats = {
    "state_pkts": 0,
    "actuator_pkts": 0,
    "setpoint_pkts": 0
}

active_vehicle_file = "Team1_VTOL.json"
active_vehicle_mtime = 0

def find_vehicle_file():
    global active_vehicle_file, active_vehicle_mtime
    input_files = glob.glob("input*.json")
    if input_files:
        try:
            with open(input_files[0], 'r') as f:
                data = json.load(f)
                vfile = data.get("vehicle", {}).get("properties", {}).get("filepath")
                if vfile and os.path.exists(vfile):
                    active_vehicle_file = vfile
        except Exception as e:
            print(f"[Viewer] Error parsing {input_files[0]}: {e}")
            
    if os.path.exists(active_vehicle_file):
        active_vehicle_mtime = os.path.getmtime(active_vehicle_file)
        with open(active_vehicle_file, 'rb') as f:
            sha256 = hashlib.sha256(f.read()).hexdigest()
        print(f"[Viewer] Airframe source: {active_vehicle_file} (sha256 {sha256[:8]}...)")
    else:
        print(f"[Viewer] Warning: Airframe source {active_vehicle_file} not found.")

def get_git_hash():
    try:
        return subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('utf-8').strip()
    except Exception:
        return "unknown"

def get_file_sha256(filepath):
    if not os.path.exists(filepath): return ""
    with open(filepath, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()

class UdpProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.pkt_count = 0
        super().__init__()

    def connection_made(self, transport):
        self.transport = transport
        print(f"[UDP] Listening for MAVRIK state on port {UDP_PORT}...")

    def datagram_received(self, data, addr):
        global stats
        if len(data) >= MAVRIK_SIZE:
            stats["state_pkts"] += 1
            self.pkt_count += 1
            unpacked = struct.unpack(MAVRIK_FMT, data[:MAVRIK_SIZE])
            global latest_frame
            
            def clean(f):
                return 0.0 if math.isnan(f) or math.isinf(f) else f
                
            latest_frame["t"] = clean(unpacked[0])
            latest_frame["state"] = {
                "t": clean(unpacked[0]), "u": clean(unpacked[1]), "v": clean(unpacked[2]), "w": clean(unpacked[3]),
                "p": clean(unpacked[4]), "q": clean(unpacked[5]), "r": clean(unpacked[6]),
                "x": clean(unpacked[7]), "y": clean(unpacked[8]), "z": clean(unpacked[9]),
                "e0": clean(unpacked[10]), "ex": clean(unpacked[11]), "ey": clean(unpacked[12]), "ez": clean(unpacked[13])
            }
            if self.pkt_count % 35 == 0:
                print(f"[UDP] Receiving Telemetry... t={latest_frame['t']:.2f}")
                
            # Recording logic
            global last_rec_t, current_recording
            t = latest_frame["t"]
            if t < last_rec_t or current_recording is None:
                if current_recording:
                    current_recording.close()
                current_recording = open(f"recordings/run_{int(time.time())}.jsonl", "w")
                
                header = {
                    "type": "recording_header",
                    "started_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat() + "Z",
                    "vehicle_file": active_vehicle_file,
                    "vehicle_file_sha256": get_file_sha256(active_vehicle_file),
                    "gains_file": "pid_vtol_gains.json",
                    "gains_file_sha256": get_file_sha256("pid_vtol_gains.json"),
                    "controller_mode": "mixed",
                    "viewer_git_hash": get_git_hash(),
                    "bridge_git_hash": get_git_hash(),
                    "mavrik_version": "unknown",
                    "notes": ""
                }
                current_recording.write(json.dumps(header) + "\n")
                
            last_rec_t = t
            if current_recording:
                current_recording.write(json.dumps(latest_frame) + "\n")
                
            # Broadcast to all clients via their individual queues
            # This fixes the data aliasing bug
            for q in client_queues:
                if q.qsize() < 5: # prevent infinite buildup if client is slow
                    q.put_nowait(latest_frame)

class UdpActuatorProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport
        print(f"[UDP] Listening for MAVRIK actuators on port 5010...")

    def datagram_received(self, data, addr):
        global latest_frame, stats
        num_floats = len(data) // 4
        if num_floats >= 9:
            stats["actuator_pkts"] += 1
            unpacked = struct.unpack(f"<{num_floats}f", data[:num_floats*4])
            
            if num_floats >= 10:
                vals = unpacked[1:10]
            else:
                vals = unpacked[0:9]
                
            def clean(f):
                return 0.0 if math.isnan(f) or math.isinf(f) else f
                
            latest_frame["actuators"] = {
                "aileron": clean(vals[0]),
                "elevator": clean(vals[1]),
                "rudder": clean(vals[2]),
                "thrFR": clean(vals[3]),
                "thrFL": clean(vals[4]),
                "thrAR": clean(vals[5]),
                "thrAL": clean(vals[6]),
                "flapsRight": clean(vals[7]),
                "flapsLeft": clean(vals[8]),
                "flaps": clean((vals[7] + vals[8]) / 2.0)
            }
            
            for q in client_queues:
                if q.qsize() < 5:
                    q.put_nowait(latest_frame)

class UdpSetpointProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport
        print(f"[UDP] Listening for PID setpoints on port 5011...")

    def datagram_received(self, data, addr):
        global latest_frame, stats
        if len(data) >= 28:
            stats["setpoint_pkts"] += 1
            unpacked = struct.unpack('<7f', data[:28])
            def clean(f):
                return 0.0 if math.isnan(f) or math.isinf(f) else f
            latest_frame["setpoint"] = {
                "t": clean(unpacked[0]),
                "x": clean(unpacked[1]),
                "y": clean(unpacked[2]),
                "z": clean(unpacked[3]),
                "phi": clean(unpacked[4]),
                "theta": clean(unpacked[5]),
                "psi": clean(unpacked[6])
            }

class UdpStatusProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport
        print(f"[UDP] Listening for status messages on port 5013...")

    def datagram_received(self, data, addr):
        try:
            msg = json.loads(data.decode('utf-8'))
            for q in client_queues:
                if q.qsize() < 10:
                    q.put_nowait(msg)
        except Exception as e:
            pass

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    queue = asyncio.Queue()
    client_queues.append(queue)
    
    # UDP socket to send RC overrides
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    async def receive_from_ws():
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                data = json.loads(msg.data)
                if data.get("type") == "rc_override":
                    pkt = struct.pack('<4f', data["roll"], data["pitch"], data["yaw"], data["throttle"])
                    rc_sock.sendto(pkt, ('127.0.0.1', 5012))
                    
                    # Echo back for 6.3
                    echo_msg = {
                        "type": "rc_echo",
                        "axes": [data["roll"], data["pitch"], data["yaw"], data["throttle"]]
                    }
                    for q in client_queues:
                        if q.qsize() < 10:
                            q.put_nowait(echo_msg)
                    
    recv_task = asyncio.create_task(receive_from_ws())
    
    try:
        while True:
            frame = await queue.get()
            await ws.send_json(frame)
    except Exception as e:
        print(f"[HTTP] Client disconnected")
    finally:
        client_queues.remove(queue)
        recv_task.cancel()
        rc_sock.close()
    
    return ws

async def airframe_endpoint(request):
    global active_vehicle_mtime
    try:
        if os.path.exists(active_vehicle_file):
            current_mtime = os.path.getmtime(active_vehicle_file)
            if current_mtime > active_vehicle_mtime:
                print(f"[Viewer] WARNING: {active_vehicle_file} changed mid-session. Viewer may be stale.")
                active_vehicle_mtime = current_mtime
                
        with open(active_vehicle_file, "r") as f:
            vtol_config = json.load(f)
            
        components = []
        for name, comp in vtol_config.get("components", {}).items():
            components.append({
                "name": name,
                "type": comp.get("type"),
                "location": comp.get("location[ft]", [0,0,0]),
                "attitude": comp.get("attitude[deg]", [0,0,0]),
                "lengths": comp.get("lengths[ft]", [0,0,0]),
                "semispan": comp.get("semispan[ft]"),
                "chord": comp.get("chord[ft]"),
                "control_surface": comp.get("control_surface"),
                "control_symmetric": comp.get("control_symmetric")
            })
            
        return web.json_response(components)
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

async def list_recordings(request):
    files = sorted(os.listdir("recordings"), reverse=True)
    return web.json_response([f for f in files if f.endswith('.jsonl')])

async def index(request):
    return web.FileResponse('./static/index.html')

async def stats_loop():
    global stats
    while True:
        await asyncio.sleep(1.0)
        stats_msg = {
            "type": "stats",
            "state_hz": stats["state_pkts"],
            "actuator_hz": stats["actuator_pkts"],
            "setpoint_hz": stats["setpoint_pkts"],
            "state_drop": max(0, (35 - stats["state_pkts"]) / 35.0),
            "actuator_drop": max(0, (50 - stats["actuator_pkts"]) / 50.0),
            "setpoint_drop": max(0, (50 - stats["setpoint_pkts"]) / 50.0)
        }
        for q in client_queues:
            if q.qsize() < 10:
                q.put_nowait(stats_msg)
        stats["state_pkts"] = 0
        stats["actuator_pkts"] = 0
        stats["setpoint_pkts"] = 0

async def main():
    print("==================================================")
    print("  MAVRIK Native Web Viewer  ")
    print("==================================================")
    
    find_vehicle_file()
    
    loop = asyncio.get_running_loop()
    
    asyncio.create_task(stats_loop())
    transport_state, _ = await loop.create_datagram_endpoint(
        lambda: UdpProtocol(),
        local_addr=(UDP_IP, UDP_PORT)
    )
    
    transport_actuators, _ = await loop.create_datagram_endpoint(
        lambda: UdpActuatorProtocol(),
        local_addr=(UDP_IP, 5010)
    )
    
    transport_setpoint, _ = await loop.create_datagram_endpoint(
        lambda: UdpSetpointProtocol(),
        local_addr=(UDP_IP, 5011)
    )
    
    transport_status, _ = await loop.create_datagram_endpoint(
        lambda: UdpStatusProtocol(),
        local_addr=(UDP_IP, 5013)
    )
    
    app = web.Application()
    app.router.add_get('/', index)
    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/airframe', airframe_endpoint)
    app.router.add_get('/recordings', list_recordings)
    app.router.add_static('/recordings/', './recordings/')
    app.router.add_static('/', './static/')
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', HTTP_PORT)
    await site.start()
    print(f"[HTTP] Web Viewer running at http://localhost:{HTTP_PORT}")
    
    try:
        await asyncio.Future() # Run forever
    except asyncio.CancelledError:
        pass
    finally:
        transport_state.close()
        transport_actuators.close()
        transport_setpoint.close()
        transport_status.close()
        if current_recording:
            current_recording.close()
        await runner.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.")
