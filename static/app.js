const scene = new THREE.Scene();
scene.background = new THREE.Color(0x222222);
scene.fog = new THREE.Fog(0x222222, 50, 300);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
camera.layers.set(0);

const mapCamera = new THREE.OrthographicCamera(-50, 50, 50, -50, 1, 1000);
mapCamera.position.set(0, 200, 0);
mapCamera.lookAt(0, 0, 0);
mapCamera.layers.enable(0);
mapCamera.layers.enable(1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
const viewport3d = document.getElementById('viewport3d');
renderer.setSize(viewport3d.clientWidth, viewport3d.clientHeight);
renderer.shadowMap.enabled = true;
renderer.autoClear = false;
viewport3d.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.8);
hemiLight.position.set(0, 200, 0);
scene.add(hemiLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
dirLight.position.set(50, 100, 50);
dirLight.castShadow = true;
scene.add(dirLight);

const grid = new THREE.GridHelper(5000, 500, 0x444444, 0x222222);
grid.position.y = -0.1;
scene.add(grid);

const planeGeo = new THREE.PlaneGeometry(5000, 5000);
const planeMat = new THREE.MeshStandardMaterial({color: 0x111111, depthWrite: false});
const plane = new THREE.Mesh(planeGeo, planeMat);
plane.rotation.x = -Math.PI / 2;
plane.position.y = -0.2;
plane.receiveShadow = true;
scene.add(plane);

const cityGroup = new THREE.Group();
const bldgMat = new THREE.MeshStandardMaterial({ color: 0x88bbff, roughness: 0.5, metalness: 0.1 });
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

const arrowGeo = new THREE.ConeGeometry(5, 15, 3);
const arrowMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const arrow = new THREE.Mesh(arrowGeo, arrowMat);
arrow.rotation.x = -Math.PI / 2;
arrow.layers.set(1);
scene.add(arrow);

// MAVRIK Origin Reference
const originRingGeo = new THREE.TorusGeometry(10, 0.5, 8, 32);
const originRingMat = new THREE.MeshBasicMaterial({ color: 0xffff00 });
const originRing = new THREE.Mesh(originRingGeo, originRingMat);
originRing.rotation.x = -Math.PI / 2;
originRing.position.y = 0.5;
scene.add(originRing);

const ringGeo = new THREE.TorusGeometry(20, 0.2, 8, 32);
const ringMat = new THREE.MeshBasicMaterial({ color: 0x88bbff, transparent: true, opacity: 0.3 });
for (let h = 100; h <= 2000; h += 100) {
    const r = new THREE.Mesh(ringGeo, ringMat);
    r.rotation.x = -Math.PI / 2;
    r.position.y = h;
    scene.add(r);
}

// Airframe holder
const drone = new THREE.Group();
scene.add(drone);
let airframeComponents = {};
let fwdMotorLeft, fwdMotorRight, aftMotorLeft, aftMotorRight;
let flapsVal = 0;

fetch('/airframe').then(r => r.json()).then(components => {
    const fuseMat = new THREE.MeshStandardMaterial({ color: 0x113388 });
    const wingMat = new THREE.MeshStandardMaterial({ color: 0xcccccc });
    const fwdMotorMat = new THREE.MeshStandardMaterial({ color: 0xff4444 });
    const aftMotorMat = new THREE.MeshStandardMaterial({ color: 0xffaa00 });
    const controlMat = new THREE.MeshStandardMaterial({ color: 0x999999 });

    components.forEach(comp => {
        let mesh;
        let group = new THREE.Group();
        
        // Map MAVRIK location to Three.js Local
        const [cx, cy, cz] = comp.location;
        group.position.set(cy, -cz, -cx);
        
        // Static attitude
        let att = [0, 0, 0];
        if (Array.isArray(comp.attitude)) {
            att = comp.attitude;
            const phi = att[0] * Math.PI / 180;
            const theta = att[1] * Math.PI / 180;
            const psi = att[2] * Math.PI / 180;
            group.rotation.set(theta, psi, -phi, 'YXZ');
        }
        
        if (comp.type === 'cuboid') {
            const [lx, ly, lz] = comp.lengths;
            let w = ly, h = lz, d = lx;
            if (comp.name === 'fuselage') {
                w = ly * 0.1; // make it 0.8ft wide instead of 8.1ft
                h = lz * 0.4; // make it 0.8ft tall instead of 2ft
                d = lx * 0.8; // make it 6.4ft long instead of 8ft
            }
            mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), fuseMat);
        } else if (comp.type === 'cylinder') {
            const r = comp.lengths[2];
            const h = comp.lengths[0];
            const isFwd = comp.name.includes("fwd") || comp.name.includes("tilt");
            mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h), isFwd ? fwdMotorMat : aftMotorMat);
            // Default cylinder is along Y. We want it along Fwd (-Z).
            // Rotate around X by -90 deg.
            mesh.rotation.x = -Math.PI / 2;
        } else if (comp.type === 'wing') {
            const span = comp.semispan * 2;
            const chord = comp.chord[0];
            const thick = comp.chord[0] * 0.09;
            mesh = new THREE.Mesh(new THREE.BoxGeometry(span, thick, chord), wingMat);
        } else {
            return;
        }
        
        mesh.castShadow = true;
        group.add(mesh);
        
        if (comp.type === 'cylinder') {
            const propGroup = new THREE.Group();
            const bladeGeo = new THREE.BoxGeometry(3.0, 0.05, 0.2); // 3ft diameter prop
            const bladeMat = new THREE.MeshBasicMaterial({ color: 0xcccccc });
            const blade1 = new THREE.Mesh(bladeGeo, bladeMat);
            const blade2 = new THREE.Mesh(bladeGeo, bladeMat);
            blade2.rotation.y = Math.PI / 2; // Make an X shape
            propGroup.add(blade1);
            propGroup.add(blade2);
            
            // Put at front of the motor (which is along -Z)
            propGroup.position.z = -comp.lengths[0] / 2;
            // The blades lie in the X-Z plane, spanning X and Z, with thickness Y.
            // The center axis of rotation is Y.
            // We want the center axis to point Fwd (-Z).
            // Rotate around X by -90 deg.
            propGroup.rotation.x = -Math.PI / 2;
            group.add(propGroup);
            comp._disk = propGroup;
            
            const dir = new THREE.Vector3(0, 0, -1); // Fwd
            const origin = new THREE.Vector3(0, 0, -comp.lengths[0]/2);
            const arrow = new THREE.ArrowHelper(dir, origin, 0, 0xff0000);
            group.add(arrow);
            comp._arrow = arrow;
        }

        if (comp.type === 'wing' && comp.control_surface) {
            const span = comp.semispan * 2;
            const chord = comp.chord[0] * 0.3;
            const thick = comp.chord[0] * 0.05;
            const csMesh = new THREE.Mesh(new THREE.BoxGeometry(span, thick, chord), controlMat);
            
            const csGroup = new THREE.Group();
            csGroup.position.z = comp.chord[0] / 2;
            csMesh.position.z = chord / 2;
            csGroup.add(csMesh);
            
            group.add(csGroup);
            comp._csGroup = csGroup;
        }
        
        drone.add(group);
        airframeComponents[comp.name] = { group, comp };
    });
});

// Ghost Drone (Setpoint)
const ghostDrone = new THREE.Group();
const ghostMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true, transparent: true, opacity: 0.3 });
const ghostMesh = new THREE.Mesh(new THREE.BoxGeometry(4, 1, 4), ghostMat);
ghostDrone.add(ghostMesh);
scene.add(ghostDrone);

// EKF Drone (ArduPilot)
const ekfDrone = new THREE.Group();
const ekfMat = new THREE.MeshBasicMaterial({ color: 0xffff00, wireframe: true, transparent: true, opacity: 0.4 });
const ekfMesh = new THREE.Mesh(new THREE.BoxGeometry(4.2, 1.2, 4.2), ekfMat);
ekfDrone.add(ekfMesh);
scene.add(ekfDrone);

// Axis Indicator Scene
const axisScene = new THREE.Scene();
const axisCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
axisCamera.position.set(0, 0, 5);
const axesHelper = new THREE.AxesHelper(2);
axisScene.add(axesHelper);

const maxTrailPoints = 1000;
let trailPoints = [];
const trailGeo = new THREE.BufferGeometry();
const trailMat = new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 2, transparent: true, opacity: 0.8 });
const trailLine = new THREE.Line(trailGeo, trailMat);
scene.add(trailLine);

camera.position.set(10, 10, 15);

window.addEventListener('resize', onWindowResize, false);
function onWindowResize() {
    const vp = document.getElementById('viewport3d');
    if (!vp) return;
    camera.aspect = vp.clientWidth / vp.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(vp.clientWidth, vp.clientHeight);
}

const hud = document.getElementById('hud');
const cameraModes = ['chase', 'top-down', 'side', 'cockpit', 'free'];
let currentCameraMode = 'chase';

window.cycleCamera = function() {
    let idx = cameraModes.indexOf(currentCameraMode);
    currentCameraMode = cameraModes[(idx + 1) % cameraModes.length];
};

window.addEventListener('keydown', (e) => {
    if (e.key === 'c' || e.key === 'C') window.cycleCamera();
});

controls.addEventListener('start', () => {
    currentCameraMode = 'free';
});

const altTape = document.getElementById('alt-tape');
for(let i=-100; i<=2000; i+=10) {
    const mark = document.createElement('div');
    mark.className = 'alt-mark';
    mark.style.bottom = (i * 2) + 'px';
    mark.innerText = i % 50 === 0 ? i : '';
    altTape.appendChild(mark);
}

const speedTape = document.getElementById('speed-tape');
for(let i=0; i<=300; i+=5) {
    const mark = document.createElement('div');
    mark.className = 'speed-mark';
    mark.style.bottom = (i * 2) + 'px';
    mark.innerText = i % 25 === 0 ? i : '';
    speedTape.appendChild(mark);
}

const graphRollCtx = document.getElementById('graph-roll').getContext('2d');
const graphPitchCtx = document.getElementById('graph-pitch').getContext('2d');
const graphYawCtx = document.getElementById('graph-yaw').getContext('2d');
const maxGraphPoints = 150;
let pitchHistory = [], pitchCmdHistory = [];
let rollHistory = [], rollCmdHistory = [];
let yawHistory = [], yawCmdHistory = [];

function setMode(mode) {
    document.body.className = mode + '-mode';
    setTimeout(onWindowResize, 350); // wait for CSS transition
}

let lastT = 0;

const wsProto = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
const ws = new WebSocket(wsProto + window.location.host + '/ws');

ws.onopen = function() { console.log("WebSocket connected."); };

let playbackMode = false;
let playbackData = [];
let playbackIdx = 0;
let playbackTimer = null;

let rcEnabled = false;
let rcRollAxis = 0;
let rcPitchAxis = 1;
let rcThrAxis = 2;
let rcYawAxis = 3;
document.getElementById('rc-enable').addEventListener('change', (e) => {
    rcEnabled = e.target.checked;
});

let calState = 0;
let calAxisMax = 0;
let calActiveAxis = -1;
let stepAxisMin = [];
let stepAxisMax = [];

let claimedAxes = [];

window.startCalibration = function() {
    calState = 1;
    document.getElementById('cal-wizard-overlay').style.display = 'flex';
    document.getElementById('cal-instructions').innerText = 'Center all sticks, then click Next.';
    document.getElementById('cal-progress-bar').style.width = '0%';
    document.getElementById('cal-next-btn').style.display = 'inline-block';
    claimedAxes = [];
};

window.cancelCalibration = function() {
    calState = 0;
    document.getElementById('cal-wizard-overlay').style.display = 'none';
};

window.nextCalibrationStep = function() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) { gp = gamepads[i]; break; }
    }
    
    if (calState === 1) {
        calState = 2;
        document.getElementById('cal-instructions').innerText = 'Push Throttle ALL THE WAY UP, hold, and click Next.';
        document.getElementById('cal-progress-bar').style.width = '0%';
        calAxisMax = 0;
        calActiveAxis = -1;
        if (gp) {
            stepAxisMin = [...gp.axes];
            stepAxisMax = [...gp.axes];
        }
    } else if (calState >= 2 && calState <= 5) {
        if (calActiveAxis === -1) {
            alert("Stick movement not detected! Please push the stick until the bar turns GREEN.");
            return;
        }
        if (claimedAxes.includes(calActiveAxis)) {
            alert("This stick axis was already assigned to another channel! Please push the correct stick.");
            return;
        }
        
        let invert = false;
        let scale = 1.0;
        if (Math.abs(calAxisMax) > 0.1) {
            scale = 1.0 / Math.abs(calAxisMax);
        }
        
        if (calAxisMax < 0) invert = true;
        
        claimedAxes.push(calActiveAxis);
        
        if (calState === 2) {
            document.getElementById('ax-thr').value = calActiveAxis;
            document.getElementById('inv-thr').checked = invert;
            document.getElementById('sc-thr').value = scale.toFixed(2);
            calState = 3;
            document.getElementById('cal-instructions').innerText = 'Push Yaw ALL THE WAY RIGHT, hold, and click Next.';
        } else if (calState === 3) {
            document.getElementById('ax-yaw').value = calActiveAxis;
            document.getElementById('inv-yaw').checked = invert;
            document.getElementById('sc-yaw').value = scale.toFixed(2);
            calState = 4;
            document.getElementById('cal-instructions').innerText = 'Push Pitch ALL THE WAY FORWARD (Nose Down), hold, and click Next.';
        } else if (calState === 4) {
            document.getElementById('ax-pitch').value = calActiveAxis;
            document.getElementById('inv-pitch').checked = invert;
            document.getElementById('sc-pitch').value = scale.toFixed(2);
            calState = 5;
            document.getElementById('cal-instructions').innerText = 'Push Roll ALL THE WAY RIGHT, hold, and click Next.';
        } else if (calState === 5) {
            document.getElementById('ax-roll').value = calActiveAxis;
            document.getElementById('inv-roll').checked = invert;
            document.getElementById('sc-roll').value = scale.toFixed(2);
            cancelCalibration(); // Done!
            return;
        }
        calAxisMax = 0;
        calActiveAxis = -1;
        document.getElementById('cal-progress-bar').style.width = '0%';
        document.getElementById('cal-progress-bar').style.background = '#0055ff';
        if (gp) {
            stepAxisMin = [...gp.axes];
            stepAxisMax = [...gp.axes];
        }
    }
};

function processCalibration(gp) {
    if (calState < 2) return;
    
    const axes = gp.axes;
    const progress = document.getElementById('cal-progress-bar');
    
    for (let i=0; i<axes.length; i++) {
        if (stepAxisMin[i] === undefined) {
            stepAxisMin[i] = axes[i];
            stepAxisMax[i] = axes[i];
        }
        if (axes[i] < stepAxisMin[i]) stepAxisMin[i] = axes[i];
        if (axes[i] > stepAxisMax[i]) stepAxisMax[i] = axes[i];
    }
    
    let maxRange = 0;
    let tempActive = -1;
    let tempMax = 0;
    for (let i=0; i<axes.length; i++) {
        // Skip axes that are already claimed!
        if (claimedAxes.includes(i)) continue;
        
        let range = stepAxisMax[i] - stepAxisMin[i];
        if (range > maxRange) {
            maxRange = range;
            tempActive = i;
            if (Math.abs(stepAxisMax[i]) > Math.abs(stepAxisMin[i])) {
                tempMax = stepAxisMax[i];
            } else {
                tempMax = stepAxisMin[i];
            }
        }
    }
    
    if (maxRange > 0.5) {
        calActiveAxis = tempActive;
        calAxisMax = tempMax;
        progress.style.width = '100%';
        progress.style.background = '#00ff00';
    } else {
        calActiveAxis = -1;
        progress.style.width = '0%';
        progress.style.background = '#0055ff';
    }
}

function pollGamepad() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gp = null;
    for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            gp = gamepads[i];
            break;
        }
    }
    
    if (gp && calState > 0) {
        processCalibration(gp);
        return;
    }
    
    const statusDiv = document.getElementById('rc-status');
    if (!gp) {
        statusDiv.innerText = 'No Gamepad Detected (Wiggle stick/press button to wake)';
        statusDiv.style.color = '#ff4444';
        return;
    }
    
    statusDiv.innerText = 'Gamepad Connected';
    statusDiv.style.color = '#00ff00';
    
    rcRollAxis = parseInt(document.getElementById('ax-roll').value);
    rcPitchAxis = parseInt(document.getElementById('ax-pitch').value);
    rcThrAxis = parseInt(document.getElementById('ax-thr').value);
    rcYawAxis = parseInt(document.getElementById('ax-yaw').value);
    
    const axes = gp.axes;
    if(axes.length > Math.max(rcRollAxis, rcPitchAxis, rcThrAxis, rcYawAxis)) {
        let roll = axes[rcRollAxis];
        let pitch = axes[rcPitchAxis];
        let thr = axes[rcThrAxis];
        let yaw = axes[rcYawAxis];
        
        let scRoll = parseFloat(document.getElementById('sc-roll').value) || 1.0;
        let scPitch = parseFloat(document.getElementById('sc-pitch').value) || 1.0;
        let scThr = parseFloat(document.getElementById('sc-thr').value) || 1.0;
        let scYaw = parseFloat(document.getElementById('sc-yaw').value) || 1.0;
        
        roll = Math.max(-1.0, Math.min(1.0, roll * scRoll));
        pitch = Math.max(-1.0, Math.min(1.0, pitch * scPitch));
        thr = Math.max(-1.0, Math.min(1.0, thr * scThr));
        yaw = Math.max(-1.0, Math.min(1.0, yaw * scYaw));
        
        if (document.getElementById('inv-roll').checked) roll = -roll;
        if (document.getElementById('inv-pitch').checked) pitch = -pitch;
        if (document.getElementById('inv-thr').checked) thr = -thr;
        if (document.getElementById('inv-yaw').checked) yaw = -yaw;
        
        // RC Config mode preview
        if (document.body.className === 'rc_config-mode') {
            document.getElementById('echo-roll').style.width = Math.abs(roll)*50 + '%';
            document.getElementById('echo-roll').style.left = roll < 0 ? (50 + roll*50) + '%' : '50%';
            document.getElementById('echo-pitch').style.width = Math.abs(pitch)*50 + '%';
            document.getElementById('echo-pitch').style.left = pitch < 0 ? (50 + pitch*50) + '%' : '50%';
            document.getElementById('echo-yaw').style.width = Math.abs(yaw)*50 + '%';
            document.getElementById('echo-yaw').style.left = yaw < 0 ? (50 + yaw*50) + '%' : '50%';
            document.getElementById('echo-thr').style.width = Math.abs(thr)*50 + '%';
            document.getElementById('echo-thr').style.left = thr < 0 ? (50 + thr*50) + '%' : '50%';
            
            // Physically rotate the 3D drone to show expected orientation
            // Max 30 deg pitch/roll, yaw is continuous but we'll show up to 45 deg for visualization
            drone.rotation.set(
                pitch * (Math.PI/6),
                yaw * (Math.PI/4),
                -roll * (Math.PI/6),
                'YXZ'
            );
            
            // Apply directly to dummy actuators for visual feedback
            window.latestActuators = {
                aileron: roll * 15,
                elevator: pitch * 15,
                rudder: yaw * 15,
                thrFR: (1 - thr) / 2,
                thrFL: (1 - thr) / 2,
                thrAR: (1 - thr) / 2,
                thrAL: (1 - thr) / 2,
                flapsRight: 0, flapsLeft: 0
            };
        }
        
        if (rcEnabled && ws.readyState === WebSocket.OPEN) {
            const rcMsg = {
                type: 'rc_override',
                roll: roll,
                pitch: pitch,
                throttle: thr,
                yaw: yaw
            };
            ws.send(JSON.stringify(rcMsg));
        }
    }
}

window.loadRecordings = async function() {
    const res = await fetch('/recordings');
    const files = await res.json();
    const sel = document.getElementById('recording-select');
    sel.innerHTML = '';
    files.forEach(f => {
        const opt = document.createElement('option');
        opt.value = f;
        opt.innerText = f;
        sel.appendChild(opt);
    });
};

window.playRecording = async function() {
    const sel = document.getElementById('recording-select');
    if(!sel.value) return;
    const res = await fetch('/recordings/' + sel.value);
    const text = await res.text();
    playbackData = text.trim().split('\n').filter(l => l).map(l => JSON.parse(l));
    playbackMode = true;
    playbackIdx = 0;
    if(playbackTimer) clearInterval(playbackTimer);
    
    playbackTimer = setInterval(() => {
        if(playbackIdx >= playbackData.length) {
            clearInterval(playbackTimer);
            return;
        }
        const frame = playbackData[playbackIdx++];
        processFrame(frame);
        document.getElementById('playback-time').innerText = `Playback: ${frame.t.toFixed(2)}s`;
    }, 20);
};

window.stopRecording = function() {
    playbackMode = false;
    if(playbackTimer) clearInterval(playbackTimer);
    document.getElementById('playback-time').innerText = 'LIVE';
};

function processFrame(msg) {
    const state = msg.type === "state" ? msg.data : (msg.type === "frame" ? msg.state : msg);
    const actuators = msg.type === "frame" ? msg.actuators : {};
    window.latestActuators = actuators;
    
    if (msg.type === "frame" && msg.setpoint) {
        const sp = msg.setpoint;
        ghostDrone.position.set(sp.y, -sp.z, -sp.x);
        const phi = sp.phi * Math.PI / 180;
        const theta = sp.theta * Math.PI / 180;
        const psi = sp.psi * Math.PI / 180;
        ghostDrone.rotation.set(theta, psi, -phi, 'YXZ');
        
        window.latestSetpoint = sp;
    }
    
    if (!state || state.t === undefined) return;
    
    if (state.t < lastT) {
        trailPoints = [];
        trailGeo.setFromPoints(trailPoints);
        pitchHistory = []; rollHistory = []; yawHistory = [];
    }
    lastT = state.t;

    drone.position.set(state.y, -state.z, -state.x);

    const e0 = state.e0, ex = state.ex, ey = state.ey, ez = state.ez;
    const pitchSin = Math.max(-1, Math.min(1, 2 * (e0 * ey - ez * ex)));
    const pitchRad = Math.asin(pitchSin);
    const rollRad = Math.atan2(2 * (e0 * ex + ey * ez), e0 * e0 - ex * ex - ey * ey + ez * ez);
    const yawRad = Math.atan2(2 * (e0 * ez + ex * ey), e0 * e0 + ex * ex - ey * ey - ez * ez);
    
    // Explicit Euler assignment instead of quaternion trickery for guaranteed proper yaw
    drone.rotation.set(pitchRad, yawRad, -rollRad, 'YXZ');

    const pitchDeg = pitchRad * (180 / Math.PI);
    const rollDeg = rollRad * (180 / Math.PI);
    let yawDeg = yawRad * (180 / Math.PI);
    if (yawDeg < 0) yawDeg += 360;
    
    arrow.position.copy(drone.position);
    arrow.position.y = 100;
    arrow.rotation.y = yawRad;
    mapCamera.rotation.z = -yawRad;
    
    if (trailPoints.length === 0 || trailPoints[trailPoints.length - 1].distanceTo(drone.position) > 0.5) {
        trailPoints.push(drone.position.clone());
        if (trailPoints.length > maxTrailPoints) trailPoints.shift();
        trailGeo.setFromPoints(trailPoints);
    }
    
    pitchHistory.push(pitchDeg);
    rollHistory.push(rollDeg);
    yawHistory.push(yawDeg > 180 ? yawDeg - 360 : yawDeg);
    if(pitchHistory.length > maxGraphPoints) pitchHistory.shift();
    if(rollHistory.length > maxGraphPoints) rollHistory.shift();
    if(yawHistory.length > maxGraphPoints) yawHistory.shift();
    
    if (window.latestSetpoint) {
        pitchCmdHistory.push(window.latestSetpoint.theta);
        rollCmdHistory.push(window.latestSetpoint.phi);
        let yCmd = window.latestSetpoint.psi;
        yawCmdHistory.push(yCmd > 180 ? yCmd - 360 : yCmd);
    } else {
        pitchCmdHistory.push(0); rollCmdHistory.push(0); yawCmdHistory.push(0);
    }
    if(pitchCmdHistory.length > maxGraphPoints) pitchCmdHistory.shift();
    if(rollCmdHistory.length > maxGraphPoints) rollCmdHistory.shift();
    if(yawCmdHistory.length > maxGraphPoints) yawCmdHistory.shift();

    if (currentCameraMode === 'chase') {
        const localOffset = new THREE.Vector3(0, 5, 25);
        const rotatedOffset = localOffset.applyEuler(drone.rotation);
        const targetCamPos = drone.position.clone().add(rotatedOffset);
        camera.position.lerp(targetCamPos, 0.1);
        controls.target.copy(drone.position);
    } else if (currentCameraMode === 'top-down') {
        const targetCamPos = drone.position.clone().add(new THREE.Vector3(0, 100, 0));
        camera.position.lerp(targetCamPos, 0.1);
        controls.target.copy(drone.position);
    } else if (currentCameraMode === 'side') {
        const targetCamPos = drone.position.clone().add(new THREE.Vector3(50, 0, 0));
        camera.position.lerp(targetCamPos, 0.1);
        controls.target.copy(drone.position);
    } else if (currentCameraMode === 'cockpit') {
        const localOffset = new THREE.Vector3(0, 0.5, -4);
        const rotatedOffset = localOffset.applyEuler(drone.rotation);
        const targetCamPos = drone.position.clone().add(rotatedOffset);
        camera.position.copy(targetCamPos);
        const lookOffset = new THREE.Vector3(0, 0.5, -100).applyEuler(drone.rotation);
        camera.lookAt(drone.position.clone().add(lookOffset));
    } else {
        const delta = drone.position.clone().sub(controls.target);
        controls.target.copy(drone.position);
        camera.position.add(delta);
    }

    document.getElementById('horizon-pitch').style.transform = `translateY(${pitchDeg * 3}px) rotate(${-rollDeg}deg)`;
    document.getElementById('attitude-readout').innerHTML = `PITCH: ${pitchDeg.toFixed(1)}&deg; | ROLL: ${rollDeg.toFixed(1)}&deg; | HDG: ${yawDeg.toFixed(1)}&deg;`;

    const altFt = -state.z;
    altTape.style.transform = `translateY(${altFt * 2}px)`;
    document.getElementById('alt-readout').innerText = Math.round(altFt);

    const speedFps = Math.sqrt(state.u * state.u + state.v * state.v);
    speedTape.style.transform = `translateY(${speedFps * 2}px)`;
    document.getElementById('speed-readout').innerText = Math.round(speedFps);

    hud.innerHTML = `
        <b>TELEMETRY</b><br>
        TIME:  ${state.t.toFixed(2)} s<br>
        ALT:   ${(-state.z).toFixed(1)} ft<br>
        VEL N: ${state.u.toFixed(1)} ft/s<br>
        VEL E: ${state.v.toFixed(1)} ft/s<br>
        VEL D: ${state.w.toFixed(1)} ft/s<br><br>
        <b>VIEW:</b> ${currentCameraMode.toUpperCase()}
    `;
    
    document.getElementById('recording-status').innerText = window.latestRecording ? 'REC: ON' : 'REC: OFF';
    document.getElementById('recording-status').style.color = window.latestRecording ? '#ff0000' : '#aaa';
}

ws.onmessage = function(event) {
    if(playbackMode) return;
    const msg = JSON.parse(event.data);
    
    if (msg.type === "stats") {
        document.getElementById('obs-state').innerText = `${msg.state_hz} Hz (${(msg.state_drop*100).toFixed(1)}% drop)`;
        document.getElementById('obs-act').innerText = `${msg.actuator_hz} Hz (${(msg.actuator_drop*100).toFixed(1)}% drop)`;
        document.getElementById('obs-set').innerText = `${msg.setpoint_hz} Hz (${(msg.setpoint_drop*100).toFixed(1)}% drop)`;
        document.getElementById('obs-state').style.color = msg.state_drop > 0.01 ? '#ff0000' : '#00ff00';
        document.getElementById('obs-act').style.color = msg.actuator_drop > 0.01 ? '#ff0000' : '#00ff00';
        document.getElementById('obs-set').style.color = msg.setpoint_drop > 0.01 ? '#ff0000' : '#00ff00';
        return;
    }
    if (msg.type === "rc_echo") {
        const roll = msg.axes[0], pitch = msg.axes[1], yaw = msg.axes[2], thr = msg.axes[3];
        document.getElementById('echo-roll').style.width = Math.abs(roll)*50 + '%';
        document.getElementById('echo-roll').style.left = roll < 0 ? (50 + roll*50) + '%' : '50%';
        document.getElementById('echo-pitch').style.width = Math.abs(pitch)*50 + '%';
        document.getElementById('echo-pitch').style.left = pitch < 0 ? (50 + pitch*50) + '%' : '50%';
        document.getElementById('echo-yaw').style.width = Math.abs(yaw)*50 + '%';
        document.getElementById('echo-yaw').style.left = yaw < 0 ? (50 + yaw*50) + '%' : '50%';
        document.getElementById('echo-thr').style.width = Math.abs(thr)*50 + '%';
        document.getElementById('echo-thr').style.left = thr < 0 ? (50 + thr*50) + '%' : '50%';
        return;
    }
    if (msg.type === "status") {
        if (msg.category === "rc_override") {
            const st = document.getElementById('rc-override-status');
            st.innerText = "RC: " + msg.message;
            st.style.color = msg.level === "error" ? "#ff0000" : "#00ff00";
        }
        return;
    }
    if (msg.type === "ekf_state") {
        const ekfEnable = document.getElementById('ekf-enable').checked;
        ekfDrone.visible = ekfEnable;
        if (ekfEnable) {
            ekfDrone.position.set(msg.y, -msg.z, -msg.x);
            const phi = msg.phi * Math.PI / 180;
            const theta = msg.theta * Math.PI / 180;
            const psi = msg.psi * Math.PI / 180;
            ekfDrone.rotation.set(theta, psi, -phi, 'YXZ');
        }
        return;
    }
    
    // Check if recording info was attached to the frame
    if (msg.type === "frame" && msg.t !== undefined) {
        window.latestRecording = true; // simplifying logic, could use actual recording state
    }
    
    processFrame(msg);
};

ws.onclose = function() {
    console.log("WebSocket disconnected.");
    setTimeout(() => { window.location.reload(); }, 2000);
};

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    pollGamepad();
    
    // Actuator Animations
    if (window.latestActuators) {
        const acts = window.latestActuators;
        
        for (const [name, c] of Object.entries(airframeComponents)) {
            // Animate control surfaces
            if (c.comp.control_surface && c.comp._csGroup) {
                let defl = 0;
                if (c.comp.control_surface === 'aileron') {
                    // if left wing, aileron is inverted
                    defl = acts.aileron * (c.comp.location[1] < 0 ? -1 : 1);
                } else if (c.comp.control_surface === 'elevator') {
                    defl = acts.elevator;
                } else if (c.comp.control_surface === 'rudder') {
                    defl = acts.rudder;
                }
                // Hinge is along X axis in Three.js (wing width)
                c.comp._csGroup.rotation.x = defl * Math.PI / 180;
            }
            
            // Animate motor spin and thrust arrows
            if (c.comp.type === 'cylinder') {
                let thr = 0;
                if (name.includes('fwd') || name.includes('tilt')) thr = name.includes('left') || name.includes('Left') ? acts.thrFL : acts.thrFR;
                else thr = name.includes('left') || name.includes('Left') ? acts.thrAL : acts.thrAR;
                
                if (c.comp._disk) {
                    c.comp._disk.rotation.y -= thr * 1.5; // Spin visual around its local Y axis
                }
                if (c.comp._arrow) {
                    c.comp._arrow.setLength(thr * 5 + 0.1); // Thrust arrow
                }
            }
        }
        
        // Animate tilt rotors
        // JSON formulas:
        // elevation = 90 * sin(PI/4 * flaps)
        // xb = 1 + sin(PI/4 * flaps + PI/4)
        // zb = -sin(PI/4 * flaps)
        
        const updateTiltMotor = (name, flaps) => {
            if (airframeComponents[name]) {
                const grp = airframeComponents[name].group;
                // Pitch in Three.js is around X axis. Nose up is positive X.
                const tiltDeg = 90 * Math.sin(Math.PI/4 * flaps);
                grp.rotation.x = tiltDeg * Math.PI / 180;
                
                // Update translation
                const xb = 1 + Math.sin(Math.PI/4 * flaps + Math.PI/4);
                const zb = -Math.sin(Math.PI/4 * flaps);
                // Three.js position: y is right, -z is down, -x is fwd
                // Wait, [cx, cy, cz] = location.
                // group.position.set(cy, -cz, -cx)
                const baseLoc = airframeComponents[name].comp.location;
                // we want to offset X and Z based on formulas relative to the motor mount?
                // The JSON sets motor_tilt_right location directly?
                // The JSON says:
                // "location[ft]": [1.739, 3.88, 0.146]
                // Actually the user's task 3.2 says:
                // xb = 1 + sin((π/4)*flaps + π/4)
                // zb = -sin((π/4)*flaps)
                // We should offset the location from the base. Let's just assume the base is where it is when flaps=0, and offset by differences.
                // At flaps=0: xb = 1 + sin(PI/4) = 1.707
                // zb = 0
                // So delta_x = xb - 1.707, delta_z = zb
                const dx = xb - (1 + Math.sin(Math.PI/4));
                const dz = zb;
                
                // MAVRIK base is baseLoc. new location is baseLoc + delta.
                const cx = baseLoc[0] + dx;
                const cz = baseLoc[2] + dz;
                grp.position.set(baseLoc[1], -cz, -cx);
            }
        };
        
        updateTiltMotor('motor_tilt_right', acts.flapsRight);
        updateTiltMotor('motor_tilt_left', acts.flapsLeft);
    }
    
    const drawGraph = (ctx, actual, cmd, color) => {
        if (!ctx) return;
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        const midY = ctx.canvas.height / 2;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, midY);
        ctx.lineTo(ctx.canvas.width, midY);
        ctx.stroke();
        
        if (actual.length === 0) return;
        const stepX = ctx.canvas.width / maxGraphPoints;
        
        // Draw Cmd (Dashed)
        ctx.strokeStyle = '#aaaaaa';
        ctx.lineWidth = 1;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        for(let i=0; i<cmd.length; i++) {
            const x = i * stepX;
            const y = midY - (cmd[i] * 2);
            if(i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
        
        // Draw Actual (Solid)
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.setLineDash([]);
        ctx.beginPath();
        for(let i=0; i<actual.length; i++) {
            const x = i * stepX;
            const y = midY - (actual[i] * 2);
            if(i===0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
    };
    
    drawGraph(graphRollCtx, rollHistory, rollCmdHistory, '#4444ff');
    drawGraph(graphPitchCtx, pitchHistory, pitchCmdHistory, '#ff4444');
    drawGraph(graphYawCtx, yawHistory, yawCmdHistory, '#00ff00');
    const vp = document.getElementById('viewport3d');
    renderer.setViewport(0, 0, vp.clientWidth, vp.clientHeight);
    renderer.setScissor(0, 0, vp.clientWidth, vp.clientHeight);
    renderer.setScissorTest(true);
    renderer.clear();
    renderer.render(scene, camera);

    mapCamera.position.x = drone.position.x;
    mapCamera.position.z = drone.position.z;
    
    const minimap = document.getElementById('minimap-container');
    if (minimap && window.getComputedStyle(minimap).display !== 'none') {
        const rect = minimap.getBoundingClientRect();
        renderer.setViewport(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
        renderer.setScissor(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
        renderer.setScissorTest(true);
        renderer.render(scene, mapCamera);
    }
    
    // Axis indicator
    const axisDiv = document.getElementById('axis-indicator');
    if (axisDiv && window.getComputedStyle(axisDiv).display !== 'none') {
        const rect = axisDiv.getBoundingClientRect();
        renderer.setViewport(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
        renderer.setScissor(rect.left, window.innerHeight - rect.bottom, rect.width, rect.height);
        renderer.setScissorTest(true);
        axisCamera.position.copy(camera.position);
        axisCamera.position.sub(controls.target);
        axisCamera.position.setLength(5);
        axisCamera.lookAt(0,0,0);
        axisCamera.up.copy(camera.up);
        renderer.render(axisScene, axisCamera);
    }
    
    // Restore full viewport
    renderer.setScissorTest(false);
}
animate();

function zoom(amount) {
    const dir = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
    camera.position.addScaledVector(dir, amount);
}
function pan(dx, dy) {
    const right = new THREE.Vector3();
    const up = new THREE.Vector3();
    camera.matrixWorld.extractBasis(right, up, new THREE.Vector3());
    
    controls.target.addScaledVector(right, dx);
    controls.target.addScaledVector(up, dy);
    camera.position.addScaledVector(right, dx);
    camera.position.addScaledVector(up, dy);
}
function resetCam() {
    isChaseMode = true;
}
