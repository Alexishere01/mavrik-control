#!/usr/bin/env python3
import socket
import struct
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

# MAVRIK's Graphics send port from connections.json
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
HTTP_PORT = 8080

# 14 floats: t, u, v, w, p, q, r, xf, yf, zf, e0, ex, ey, ez
MAVRIK_FMT = "<14f"
MAVRIK_SIZE = struct.calcsize(MAVRIK_FMT)

# Global state holder
latest_state = {
    "t": 0, "u": 0, "v": 0, "w": 0,
    "p": 0, "q": 0, "r": 0,
    "x": 0, "y": 0, "z": 0,
    "e0": 1, "ex": 0, "ey": 0, "ez": 0
}

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[UDP] Listening for MAVRIK state on port {UDP_PORT}...")
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= MAVRIK_SIZE:
                unpacked = struct.unpack(MAVRIK_FMT, data[:MAVRIK_SIZE])
                # Note: MAVRIK state outputs zf (down), we negate it in the browser
                global latest_state
                latest_state = {
                    "t": unpacked[0], "u": unpacked[1], "v": unpacked[2], "w": unpacked[3],
                    "p": unpacked[4], "q": unpacked[5], "r": unpacked[6],
                    "x": unpacked[7], "y": unpacked[8], "z": unpacked[9],
                    "e0": unpacked[10], "ex": unpacked[11], "ey": unpacked[12], "ez": unpacked[13]
                }
        except Exception as e:
            print(f"[UDP Error] {e}")

HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
    <title>MAVRIK Web Viewer</title>
    <style>
        body { margin: 0; overflow: hidden; background-color: #1a1a1a; font-family: sans-serif;}
        #hud { position: absolute; top: 10px; left: 10px; color: #0f0; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; font-family: monospace;}
        #instructions { position: absolute; bottom: 10px; left: 10px; color: #aaa; font-size: 12px; }
    </style>
</head>
<body>
    <div id="hud">Waiting for MAVRIK telemetry...</div>
    <div id="instructions">Left Click: Rotate | Right Click: Pan | Scroll: Zoom</div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script>
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x222222);
        // Add subtle fog to blend the grid into the background
        scene.fog = new THREE.Fog(0x222222, 50, 300);

        const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.shadowMap.enabled = true;
        document.body.appendChild(renderer.domElement);

        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;

        // Lighting
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.8);
        hemiLight.position.set(0, 200, 0);
        scene.add(hemiLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
        dirLight.position.set(50, 100, 50);
        dirLight.castShadow = true;
        scene.add(dirLight);

        // Ground Grid
        const grid = new THREE.GridHelper(500, 100, 0x444444, 0x333333);
        grid.position.y = -0.1;
        scene.add(grid);

        // Make the ground a dark plane to catch shadows
        const planeGeo = new THREE.PlaneGeometry(500, 500);
        const planeMat = new THREE.MeshStandardMaterial({color: 0x111111, depthWrite: false});
        const plane = new THREE.Mesh(planeGeo, planeMat);
        plane.rotation.x = -Math.PI / 2;
        plane.receiveShadow = true;
        scene.add(plane);

        // Build a minimalist drone
        const drone = new THREE.Group();
        
        // Materials
        const bodyMat = new THREE.MeshStandardMaterial({ color: 0x4444ff });
        const frontArmMat = new THREE.MeshStandardMaterial({ color: 0xff4444 }); // Front is Red
        const backArmMat = new THREE.MeshStandardMaterial({ color: 0x888888 });

        // Fuselage (Length: 4ft, Width: 0.5ft, Height: 0.5ft)
        const fuselage = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.5, 4), bodyMat);
        fuselage.castShadow = true;
        drone.add(fuselage);

        // Front Arm (Width: 4ft across)
        const frontArm = new THREE.Mesh(new THREE.BoxGeometry(4, 0.3, 0.3), frontArmMat);
        frontArm.position.z = -1.5; // Forward in Three.js is -Z
        frontArm.castShadow = true;
        drone.add(frontArm);

        // Back Arm
        const backArm = new THREE.Mesh(new THREE.BoxGeometry(4, 0.3, 0.3), backArmMat);
        backArm.position.z = 1.5; // Backward
        backArm.castShadow = true;
        drone.add(backArm);

        scene.add(drone);

        // Initial camera position (behind and slightly above)
        camera.position.set(10, 10, 15);
        
        // Window resize handler
        window.addEventListener('resize', onWindowResize, false);
        function onWindowResize() {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }

        const hud = document.getElementById('hud');
        let initialSetupDone = false;

        // Server-Sent Events listener
        const source = new EventSource("/stream");
        source.onmessage = function(event) {
            const state = JSON.parse(event.data);
            
            // MAVRIK (NED) -> Three.js (Y-up Right-Handed) Mapping:
            // Three X (Right) = MAVRIK y (East)
            // Three Y (Up)    = -MAVRIK z (Down)
            // Three Z (Back)  = -MAVRIK x (North)
            
            drone.position.set(state.y, -state.z, -state.x);

            // Quaternion Mapping:
            // e0 is scalar (w). Three.js uses (x, y, z, w)
            drone.quaternion.set(state.ey, -state.ez, -state.ex, state.e0);

            // Lock camera to drone dynamically if we haven't manually panned
            if (!initialSetupDone) {
                controls.target.copy(drone.position);
                camera.position.set(drone.position.x + 15, drone.position.y + 10, drone.position.z + 15);
                initialSetupDone = true;
            } else {
                // Keep the orbit target locked on the drone smoothly
                controls.target.lerp(drone.position, 0.1);
            }

            // Update HUD
            hud.innerHTML = `
                TIME:  ${state.t.toFixed(2)} s<br>
                ALT:   ${(-state.z).toFixed(1)} ft<br>
                VEL N: ${state.u.toFixed(1)} ft/s<br>
                VEL E: ${state.v.toFixed(1)} ft/s<br>
                VEL D: ${state.w.toFixed(1)} ft/s
            `;
        };

        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        }
        animate();
    </script>
</body>
</html>
"""

class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass # Suppress standard HTTP logging for clean terminal
        
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode("utf-8"))
            
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header("Content-type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            print(f"[HTTP] Client connected to SSE stream from {self.client_address}")
            try:
                while True:
                    # Send the latest state as a JSON string
                    data_str = json.dumps(latest_state)
                    self.wfile.write(f"data: {data_str}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1/30.0) # 30Hz update rate
            except Exception:
                print(f"[HTTP] Client disconnected from {self.client_address}")
        else:
            self.send_response(404)
            self.end_headers()

def run_server():
    server = HTTPServer(("0.0.0.0", HTTP_PORT), RequestHandler)
    print(f"[HTTP] Web Viewer running at http://localhost:{HTTP_PORT}")
    server.serve_forever()

if __name__ == "__main__":
    print("==================================================")
    print("  MAVRIK Native Web Viewer  ")
    print("==================================================")
    
    # Start UDP listener in background thread
    threading.Thread(target=udp_listener, daemon=True).start()
    
    # Start HTTP server on main thread
    run_server()
