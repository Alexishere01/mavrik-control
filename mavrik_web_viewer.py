#!/usr/bin/env python3
import socket
import struct
import json
import time
import threading
import math
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

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
    
    pkt_count = 0
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= MAVRIK_SIZE:
                pkt_count += 1
                unpacked = struct.unpack(MAVRIK_FMT, data[:MAVRIK_SIZE])
                # Note: MAVRIK state outputs zf (down), we negate it in the browser
                global latest_state
                
                def clean(f):
                    return 0.0 if math.isnan(f) or math.isinf(f) else f
                    
                latest_state = {
                    "t": clean(unpacked[0]), "u": clean(unpacked[1]), "v": clean(unpacked[2]), "w": clean(unpacked[3]),
                    "p": clean(unpacked[4]), "q": clean(unpacked[5]), "r": clean(unpacked[6]),
                    "x": clean(unpacked[7]), "y": clean(unpacked[8]), "z": clean(unpacked[9]),
                    "e0": clean(unpacked[10]), "ex": clean(unpacked[11]), "ey": clean(unpacked[12]), "ez": clean(unpacked[13])
                }
                if pkt_count % 35 == 0:
                    print(f"[UDP] Receiving Telemetry... t={latest_state['t']:.2f}")
        except Exception as e:
            print(f"[UDP Error] {e}")

HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
    <title>MAVRIK Web Viewer</title>
    <style>
        body { margin: 0; overflow: hidden; background-color: #1a1a1a; font-family: sans-serif;}
        #hud { position: absolute; top: 10px; left: 10px; color: #0f0; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px; font-family: monospace;}
        #controls-ui { position: absolute; bottom: 30px; left: 10px; display: flex; flex-direction: column; gap: 5px; }
        .btn-row { display: flex; gap: 5px; justify-content: center; }
        button { background: rgba(50,50,50,0.8); color: white; border: 1px solid #777; padding: 8px 15px; border-radius: 4px; cursor: pointer; font-weight: bold; }
        button:hover { background: #555; }
        #instructions { position: absolute; bottom: 5px; left: 10px; color: #777; font-size: 10px; }
        
        /* HUD Elements */
        #minimap-container { position: absolute; top: 10px; right: 10px; width: 250px; height: 250px; border: 2px solid #555; border-radius: 5px; background: transparent; z-index: 10; pointer-events: none; }
        #horizon-container { position: absolute; bottom: 20px; left: 50%; transform: translateX(-50%); width: 200px; height: 200px; border-radius: 50%; border: 4px solid #333; overflow: hidden; background: #87CEEB; z-index: 10; box-shadow: 0 0 10px black; }
        #horizon-pitch { position: absolute; width: 100%; height: 200%; top: -50%; left: 0; transform-origin: center; transition: transform 0.05s linear; }
        #horizon-sky { position: absolute; top: 0; left: 0; width: 100%; height: 50%; background: #2C75FF; }
        #horizon-ground { position: absolute; bottom: 0; left: 0; width: 100%; height: 50%; background: #654321; border-top: 2px solid white; }
        #horizon-crosshair { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 100px; height: 2px; background: yellow; z-index: 11; }
        #horizon-crosshair::before { content: ''; position: absolute; top: 0; left: 48px; width: 4px; height: 10px; background: yellow; }
        #attitude-readout { position: absolute; bottom: 230px; left: 50%; transform: translateX(-50%); color: #fff; font-family: monospace; font-size: 14px; text-align: center; background: rgba(0,0,0,0.6); padding: 5px 10px; border-radius: 5px; z-index: 10; white-space: nowrap; }

        /* Altimeter Tape */
        #altimeter { position: absolute; top: 10px; right: 280px; width: 60px; height: 250px; background: rgba(0,0,0,0.6); border: 2px solid #555; overflow: hidden; z-index: 10; color: white; font-family: monospace; text-align: right; border-radius: 5px; }
        #alt-tape { position: absolute; width: 100%; bottom: 50%; transition: transform 0.05s linear; }
        .alt-mark { position: absolute; right: 0; width: 100%; border-top: 2px solid #aaa; font-size: 12px; line-height: 12px; padding-right: 5px; box-sizing: border-box; }
        #alt-pointer { position: absolute; top: 50%; right: 0; transform: translateY(-50%); width: 0; height: 0; border-top: 10px solid transparent; border-bottom: 10px solid transparent; border-right: 15px solid red; z-index: 11; }
        #alt-readout { position: absolute; top: 50%; right: 15px; transform: translateY(-50%); background: red; color: white; padding: 2px 5px; font-weight: bold; border-radius: 3px; z-index: 12; font-size: 14px; }
    </style>
</head>
<body>
    <div id="hud">Waiting for MAVRIK telemetry...</div>
    <div id="controls-ui">
        <div class="btn-row">
            <button onclick="pan(0, -10)">▲ Pan Up</button>
        </div>
        <div class="btn-row">
            <button onclick="pan(-10, 0)">◀ Pan L</button>
            <button onclick="zoom(-5)">+ Zoom In</button>
            <button onclick="pan(10, 0)">Pan R ▶</button>
        </div>
        <div class="btn-row">
            <button onclick="pan(0, 10)">▼ Pan Down</button>
            <button onclick="zoom(5)">- Zoom Out</button>
            <button onclick="resetCam()">Chase Cam</button>
        </div>
    </div>
    <div id="instructions">Or use Left Click: Rotate | 2-Finger Click: Pan | 2-Finger Scroll: Zoom</div>
    
    <div id="minimap-container"></div>
    <div id="horizon-container">
        <div id="horizon-pitch">
            <div id="horizon-sky"></div>
            <div id="horizon-ground"></div>
        </div>
        <div id="horizon-crosshair"></div>
    </div>
    <div id="attitude-readout">PITCH: 0.0&deg; | ROLL: 0.0&deg; | HDG: 0.0&deg;</div>
    
    <div id="altimeter">
        <div id="alt-tape"></div>
        <div id="alt-pointer"></div>
        <div id="alt-readout">0</div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <script>
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x222222);
        // Add subtle fog to blend the grid into the background
        scene.fog = new THREE.Fog(0x222222, 50, 300);

        const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
        camera.layers.set(0); // Main camera only sees Layer 0
        
        // Orthographic Minimap Camera
        const mapCamera = new THREE.OrthographicCamera(-50, 50, 50, -50, 1, 1000);
        mapCamera.position.set(0, 200, 0);
        mapCamera.lookAt(0, 0, 0);
        mapCamera.layers.enable(0); // See drone/world
        mapCamera.layers.enable(1); // See minimap arrow

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.shadowMap.enabled = true;
        renderer.autoClear = false; // Required for rendering multiple viewports
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
        const grid = new THREE.GridHelper(5000, 500, 0x444444, 0x222222);
        grid.position.y = -0.1;
        scene.add(grid);

        // Make the ground a dark plane to catch shadows
        const planeGeo = new THREE.PlaneGeometry(5000, 5000);
        const planeMat = new THREE.MeshStandardMaterial({color: 0x111111, depthWrite: false});
        const plane = new THREE.Mesh(planeGeo, planeMat);
        plane.rotation.x = -Math.PI / 2;
        plane.position.y = -0.2;
        plane.receiveShadow = true;
        scene.add(plane);

        // Procedural Cityscape (World Reference)
        const cityGroup = new THREE.Group();
        const bldgMat = new THREE.MeshStandardMaterial({ color: 0x333333, roughness: 0.9 });
        for(let i=0; i<300; i++) {
            const w = 10 + Math.random() * 40;
            const h = 20 + Math.random() * 200;
            const d = 10 + Math.random() * 40;
            const bldg = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), bldgMat);
            bldg.position.x = (Math.random() - 0.5) * 4000;
            bldg.position.z = (Math.random() - 0.5) * 4000;
            bldg.position.y = h / 2;
            bldg.castShadow = true;
            bldg.receiveShadow = true;
            cityGroup.add(bldg);
        }
        scene.add(cityGroup);

        // Minimap Arrow (Layer 1)
        const arrowGeo = new THREE.ConeGeometry(5, 15, 3);
        const arrowMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const arrow = new THREE.Mesh(arrowGeo, arrowMat);
        arrow.rotation.x = -Math.PI / 2; // Point forward
        arrow.layers.set(1);
        scene.add(arrow);

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
        let isChaseMode = true;
        
        controls.addEventListener('start', () => {
            isChaseMode = false; // User took over camera
        });

        // Setup Altimeter Tape
        const altTape = document.getElementById('alt-tape');
        for(let i=-100; i<=2000; i+=10) {
            const mark = document.createElement('div');
            mark.className = 'alt-mark';
            mark.style.bottom = (i * 2) + 'px'; // 2px per ft
            mark.innerText = i % 50 === 0 ? i : '';
            altTape.appendChild(mark);
        }

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

            // Update Minimap Arrow to follow drone position and heading
            arrow.position.copy(drone.position);
            arrow.position.y = 100; // Hover above
            
            // Calculate Euler angles
            const e0 = state.e0, ex = state.ex, ey = state.ey, ez = state.ez;
            
            // Clamp asin argument to [-1, 1] to prevent NaN from floating point errors
            const pitchSin = Math.max(-1, Math.min(1, 2 * (e0 * ey - ez * ex)));
            const pitchRad = Math.asin(pitchSin);
            
            const rollRad = Math.atan2(2 * (e0 * ex + ey * ez), e0 * e0 - ex * ex - ey * ey + ez * ez);
            const yawRad = Math.atan2(2 * (e0 * ez + ex * ey), e0 * e0 + ex * ex - ey * ey - ez * ez);
            
            // Point arrow along yaw
            arrow.rotation.y = yawRad;

            // Camera Tracking
            if (isChaseMode) {
                // Fly behind the drone
                const localOffset = new THREE.Vector3(0, 5, 25); // 25ft back, 5ft up
                const rotatedOffset = localOffset.applyQuaternion(drone.quaternion);
                const targetCamPos = drone.position.clone().add(rotatedOffset);
                camera.position.lerp(targetCamPos, 0.1);
                controls.target.copy(drone.position);
            } else {
                // Free orbit, just drag the camera along with the drone
                const delta = drone.position.clone().sub(controls.target);
                controls.target.copy(drone.position);
                camera.position.add(delta);
            }

            // Update Artificial Horizon (translate 3px per degree of pitch)
            document.getElementById('horizon-pitch').style.transform = `translateY(${pitchDeg * 3}px) rotate(${-rollDeg}deg)`;
            document.getElementById('attitude-readout').innerHTML = `PITCH: ${pitchDeg.toFixed(1)}&deg; | ROLL: ${rollDeg.toFixed(1)}&deg; | HDG: ${yawDeg.toFixed(1)}&deg;`;

            // Update Altimeter
            const altFt = -state.z;
            altTape.style.transform = `translateY(${altFt * 2}px)`;
            document.getElementById('alt-readout').innerText = Math.round(altFt);

            // Update HUD
            hud.innerHTML = `
                <b>TELEMETRY</b><br>
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
            
            // 1. Render Main Fullscreen Camera
            renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);
            renderer.setScissor(0, 0, window.innerWidth, window.innerHeight);
            renderer.setScissorTest(true);
            renderer.clear();
            renderer.render(scene, camera);

            // 2. Render Minimap Camera
            mapCamera.position.x = drone.position.x;
            mapCamera.position.z = drone.position.z;
            
            const minimap = document.getElementById('minimap-container');
            const rect = minimap.getBoundingClientRect();
            // WebGL coordinates are bottom-up, so Y is innerHeight - rect.bottom
            renderer.setViewport(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
            renderer.setScissor(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
            renderer.setScissorTest(true);
            renderer.render(scene, mapCamera);
        }
        animate();

        // UI Button Functions
        function zoom(amount) {
            const dir = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
            camera.position.addScaledVector(dir, amount);
        }
        function pan(dx, dy) {
            // Get camera right and up vectors to pan along the screen plane
            const right = new THREE.Vector3();
            const up = new THREE.Vector3();
            camera.matrixWorld.extractBasis(right, up, new THREE.Vector3());
            
            controls.target.addScaledVector(right, dx);
            controls.target.addScaledVector(up, dy);
            camera.position.addScaledVector(right, dx);
            camera.position.addScaledVector(up, dy);
        }
        function resetCam() {
            isChaseMode = true; // Return to chase view behind drone
        }
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
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), RequestHandler)
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
