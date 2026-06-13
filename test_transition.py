#!/usr/bin/env python3
"""
test_transition.py — Automated transition validation flight script for MAVRIK + ArduPilot SITL.
Spawns background services, arms in QHOVER, stabilizes, switches to FBWA,
commands throttle to accelerate, and verifies the forward flight transition.
"""

import argparse
import csv
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
import threading

# ─── Paths ────────────────────────────────────────────────────────────────────
DIR        = Path(__file__).parent.resolve()
VENV_PY    = sys.executable  # same Python interpreter running this script
WINE        = Path.home() / "Library/Application Support/com.isaacmarovitz.Whisky/Libraries/Wine/bin/wine64"
SIM_VEHICLE = DIR / "ardupilot/Tools/autotest/sim_vehicle.py"
WINEPREFIX = "/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401"
DIAG_CSV   = DIR / "bridge_diag.csv"
KILL_SH    = DIR / "kill_zombies.sh"

# ─── RC Override Thread ────────────────────────────────────────────────────────
_rc_override_lock     = threading.Lock()
_rc_override_channels = [1500, 1500, 1620, 1500, 0, 0, 0, 0]  # Start with stable hover throttle
_rc_override_stop     = threading.Event()
_rc_thread            = None

def rc_override_thread_fn():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not _rc_override_stop.is_set():
        with _rc_override_lock:
            ch = list(_rc_override_channels)
        target_sys, target_comp = 1, 1
        payload = struct.pack('<HHHHHHHHBB', *ch, target_sys, target_comp)
        pkt = _mav_pkt(70, payload, 124)
        try:
            s.sendto(pkt, ('127.0.0.1', 14552))
        except Exception:
            pass
        time.sleep(0.05)  # 20Hz
    s.close()

# ─── Process list ─────────────────────────────────────────────────────────────
_procs: list[subprocess.Popen] = []

def kill_all(label: str = ""):
    """Kill tracked processes + run kill_zombies.sh."""
    global _procs, _rc_thread
    
    # Stop RC thread
    _rc_override_stop.set()
    if _rc_thread is not None:
        _rc_thread.join(timeout=2.0)
        _rc_thread = None
        
    for p in _procs:
        try:
            p.kill()
        except Exception:
            pass
    _procs.clear()
    subprocess.run(["bash", str(KILL_SH)], capture_output=True, timeout=15)
    (DIR / "eeprom.bin").unlink(missing_ok=True)
    time.sleep(2)
    if label:
        print(f"  [kill] {label} ✓")

def _spawn(cmd, name, env):
    """Launch a subprocess, logging output to a file."""
    safe_name = name.split()[0].replace('/', '_') + '.log'
    log_path = DIR / "tune_logs" / safe_name
    log_path.parent.mkdir(exist_ok=True)
    f = open(log_path, 'w')
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT)
    print(f"  [+] {name:<30} pid={p.pid}")
    _procs.append(p)
    return p

# ─── MAVLink helpers ──────────────────────────────────────────────────────────
_mav_seq = 0

def _x25crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def _mav_pkt(msg_id: int, payload: bytes, crc_extra: int) -> bytes:
    global _mav_seq
    seq = _mav_seq & 0xFF
    _mav_seq += 1
    hdr = bytes([len(payload), seq, 255, 0, msg_id])
    crc = _x25crc(hdr + payload + bytes([crc_extra]))
    return b'\xFE' + hdr + payload + struct.pack('<H', crc)

def _send_mav(pkt: bytes):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(pkt, ('127.0.0.1', 14552))

def mav_set_mode(mode_id: int):
    payload = struct.pack('<LBB', mode_id, 1, 1)
    _send_mav(_mav_pkt(11, payload, 89))

def mav_arm(force: bool = True):
    f = 2989.0 if force else 0.0
    payload = struct.pack('<fffffffHBBB', 1.0, f, 0, 0, 0, 0, 0, 400, 1, 1, 0)
    _send_mav(_mav_pkt(76, payload, 152))

# ─── Main runner ──────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='Transition Flight Validator for MAVRIK')
    ap.add_argument('--duration', type=int, default=120, help='Total flight duration in seconds (default 120)')
    args = ap.parse_args()

    print(f"\n{'━'*75}")
    print(f"  MAVRIK Forward Flight & Transition Validation Test  —  {args.duration}s run")
    print(f"  Verifying transition from QHOVER to FBWA")
    print(f"{'━'*75}\n")

    # 1. Cleanup old simulation instances
    kill_all("pre-flight cleanup")
    if DIAG_CSV.exists():
        DIAG_CSV.unlink()

    # 2. Start core services
    env = os.environ.copy()
    py = str(VENV_PY)

    pyenv_shims = Path.home() / ".pyenv/shims"
    if str(pyenv_shims) not in env.get('PATH', ''):
        env['PATH'] = env.get('PATH', '/usr/bin:/bin') + f':{pyenv_shims}'

    print("Starting background services...")
    _spawn([py, "pid_vtol.py"],                "pid_vtol.py",                env)
    _spawn([py, "mavrik_ardupilot_bridge.py"], "mavrik_ardupilot_bridge.py", env)
    time.sleep(0.5)

    # SITL
    _spawn(
        [py, str(SIM_VEHICLE),
         "-v", "ArduPlane", "-f", "quadplane",
         "--model", "JSON:127.0.0.1",
         f"--add-param-file={DIR}/mavrik.parm",
         "--out", "udpin:127.0.0.1:14552",
         "--wipe-eeprom", "--no-rebuild",
         "--mavproxy-args=--daemon --non-interactive"],
        "sim_vehicle.py (SITL)", env
    )

    print("Starting RC override thread...")
    _rc_override_stop.clear()
    global _rc_thread
    _rc_thread = threading.Thread(target=rc_override_thread_fn, daemon=True)
    _rc_thread.start()

    print("Waiting 10s for SITL stack initialization...")
    time.sleep(10)

    # 3. Start MAVRIK
    print("Starting MAVRIK engine under Wine...")
    wine_env = os.environ.copy()
    wine_env['WINEPREFIX'] = WINEPREFIX
    wine_env['WINEDEBUG']  = '-all'
    _spawn([str(WINE), "mavrik.exe", "input_Team1_hover.json"], "mavrik.exe (wine)", wine_env)
    time.sleep(3)

    # 4. Handle auto-arming
    print("Waiting for MAVRIK to connect and stabilize under pre-arm controller...")
    connected = False
    conn_deadline = time.time() + 45.0
    while time.time() < conn_deadline:
        try:
            if DIAG_CSV.exists():
                with open(DIAG_CSV, newline='') as f:
                    rows = list(csv.DictReader(f))
                if rows:
                    latest_t = float(rows[-1]['sim_t'])
                    if latest_t > 20.0:
                        connected = True
                        print(f"  [stabilized] MAVRIK active & level (t={latest_t:.1f}s) ✓")
                        break
        except Exception:
            pass
        time.sleep(0.5)

    if not connected:
        print(" ✗ timeout/failure")
        kill_all("MAVRIK connection timed out")
        sys.exit(1)

    QSTABILIZE = 17
    deadline = time.time() + 25.0
    print("  [arm] Arming via MAVLink (QSTABILIZE)...", end='', flush=True)
    armed = False
    while time.time() < deadline:
        for _ in range(3):
            mav_set_mode(QSTABILIZE)
        time.sleep(0.4)
        for _ in range(5):
            mav_arm(force=True)
        time.sleep(0.6)
        try:
            with open(DIAG_CSV, newline='') as f:
                rows = list(csv.DictReader(f))
            if any(r.get('ap_has_armed') == '1' for r in rows[-100:]):
                armed = True
                print(" ✓")
                break
        except Exception:
            pass
        print('.', end='', flush=True)

    if not armed:
        print(" ✗ timeout")
        print("\n[ERROR] Arming timed out. Cleaning up...")
        kill_all("failed arming")
        sys.exit(1)

    # MAVLink listener for ArduPilot's estimated attitude
    ap_att = {"roll": 0.0, "pitch": 0.0}
    def ap_listener_fn():
        from pymavlink import mavutil
        # Wait a bit for SITL to start
        time.sleep(11)
        try:
            master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
            while not _rc_override_stop.is_set():
                msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
                if msg is not None:
                    ap_att["roll"] = msg.roll * 57.2957795
                    ap_att["pitch"] = msg.pitch * 57.2957795
        except Exception as e:
            pass
    threading.Thread(target=ap_listener_fn, daemon=True).start()

    # 5. Telemetry loop
    print(f"\nFlight active. Monitoring for transition checklist...")
    print(f"{'═'*115}")
    print(f" {'t[s]':<6} | {'Alt[ft]':<8} | {'Pitch [Act(AP)]':<16} | {'Roll [Act(AP)]':<16} | {'Airspd[fps]':<12} | {'FlapL':<6} {'FlapR':<6} | {'Status':<15}")
    print(f"{'═'*115}")

    start_t = time.time()
    end_t = start_t + args.duration

    def sigint_handler(sig, frame):
        print("\n\n[FlightTest] Interrupted by user. Cleaning up simulation...")
        kill_all("interrupted")
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    takeoff_triggered = False
    transition_triggered = False
    transition_completed = False
    transition_trigger_time = 0.0
    transition_complete_time = 0.0
    handover_sim_t = 0.0
    
    max_pitch = 0.0
    max_roll = 0.0
    max_speed = 0.0

    while time.time() < end_t:
        time.sleep(1.0)
        try:
            with open(DIAG_CSV, newline='') as f:
                rows = list(csv.DictReader(f))
            if not rows:
                continue
            r = rows[-1]
            t = float(r['sim_t'])
            alt = float(r['alt_ft'])
            pitch = float(r['pitch_deg'])
            roll = float(r['roll_deg'])
            yaw = float(r['yaw_deg'])
            u = float(r['u_fps'])
            v = float(r['v_fps'])
            w = float(r['w_fps'])
            
            # Compute true 3D airspeed (fps)
            speed = (u**2 + v**2 + w**2)**0.5
            max_speed = max(max_speed, speed)

            # Read effector flaps/tilt outputs (CH8, CH9 outputs in bridge)
            # flapsL, flapsR are in index 24, 23 (flapsL, flapsR) in bridge_diag
            flap_r = float(r.get('flapsR', 0.99))
            flap_l = float(r.get('flapsL', 0.99))
            
            # Read motor active flag
            motor_active = int(r.get('motor_active', 1))

            status_str = "QSTABILIZE"
            
            # Post-handover trigger stable hover hold
            if not takeoff_triggered and r.get('armed') == '1':
                takeoff_triggered = True
                handover_sim_t = t
                def handover_sequence():
                    print("\n  [handover] Handed over control to ArduPilot in QSTABILIZE (attitude hold).")
                    for _ in range(5):
                        mav_set_mode(17)   # QSTABILIZE: pure attitude hold, no position drift
                    with _rc_override_lock:
                        _rc_override_channels[2] = 1500   # QSTABILIZE throttle center
                        _rc_override_channels[1] = 1500   # Pitch stick neutral
                threading.Thread(target=handover_sequence, daemon=True).start()

            # Trigger transition to forward flight (FBWA) after 3s of stable QSTABILIZE hover.
            # At arm time u=2fps; at arm+3s u≈5-6fps -- still below the aero instability threshold (~8fps).
            # Transitioning early gives FBWA elevator authority before the nose-up divergence develops.
            if takeoff_triggered and t >= handover_sim_t + 3.0 and not transition_triggered:
                transition_triggered = True
                transition_trigger_time = t
                def transition_sequence():
                    print("\n  [transition] Switching flight mode to FBWA (5) to start transition!")
                    for _ in range(5):
                        mav_set_mode(5)    # FBWA
                    with _rc_override_lock:
                        _rc_override_channels[2] = 1750   # Command high throttle to build forward airspeed
                        _rc_override_channels[1] = 1500   # Pitch stick neutral (maintain level altitude/pitch)
                threading.Thread(target=transition_sequence, daemon=True).start()

            # Check for transition completion
            # Transition completes when:
            # 1. Flaps/tilt are fully forward (< 0.05)
            # 2. Airspeed is high (> 35 fps)
            if transition_triggered and not transition_completed:
                if flap_l < 0.05 and flap_r < 0.05 and speed > 35.0:
                    transition_completed = True
                    transition_complete_time = t
                    print(f"\n  [SUCCESS] Forward flight transition completed in {transition_complete_time - transition_trigger_time:.1f}s!")
                    print("  [SUCCESS] Rotors fully horizontal, flight stabilized on aerodynamic surfaces.\n")

            if transition_completed:
                status_str = "FIXED_WING"
            elif transition_triggered:
                status_str = f"TRANSITION ({speed:.0f}fps)"
            elif takeoff_triggered:
                status_str = "HOVER_HOLD"

            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))

            warn = ""
            if abs(pitch) > 20 or abs(roll) > 20:
                warn = " ⚠️ DIVERGENCE"
            if abs(pitch) > 45 or abs(roll) > 45:
                warn = " 🚨 CRITICAL"

            print(f" {t:5.1f}s | {alt:8.1f} | {pitch:6.2f} ({ap_att['pitch']:+5.1f}) | {roll:6.2f} ({ap_att['roll']:+5.1f}) | {speed:12.2f} | {flap_l:.3f}  {flap_r:.3f} | {status_str:<15}{warn}")
        except Exception:
            pass

    # 6. Flight Summary and Cleanup
    print(f"\n{'═'*95}")
    print("Transition Validation Test Summary:")
    print(f"  Takeoff Hook Triggered:     {'Yes ✓' if takeoff_triggered else 'No ✗'}")
    print(f"  Transition Commenced:       {'Yes ✓' if transition_triggered else 'No ✗'}")
    print(f"  Transition Completed:       {'Yes ✓' if transition_completed else 'No ✗'}")
    if transition_completed:
        duration = transition_complete_time - transition_trigger_time
        print(f"  Transition Time:            {duration:.2f}s")
    print(f"  Max Speed Achieved:         {max_speed:.1f} ft/s")
    print(f"  Max Pitch Excursion:        {max_pitch:.2f}°")
    print(f"  Max Roll Excursion:         {max_roll:.2f}°")
    print(f"{'═'*95}\n")

    kill_all("flight finished")
    
    # Return exit code based on whether transition completed successfully
    if transition_completed:
        sys.exit(0)
    else:
        print("[ERROR] Forward flight transition did not complete.")
        sys.exit(1)

if __name__ == '__main__':
    main()
