#!/usr/bin/env python3
"""
headless_flight_test.py — Automated long-duration headless flight test for MAVRIK + ArduPilot SITL.
Starts all simulation stack components, arms the vehicle, hovers headlessly,
and prints a real-time console telemetry HUD.
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
_rc_override_channels = [1500, 1500, 1620, 1500, 0, 0, 0, 0]  # Increased CH3 throttle to 1620 for stable loaded hover
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
    ap = argparse.ArgumentParser(description='Headless Flight Tester for MAVRIK')
    ap.add_argument('--duration', type=int, default=300, help='Flight time in seconds (default 300)')
    args = ap.parse_args()

    print(f"\n{'━'*64}")
    print(f"  MAVRIK Headless Long Flight Test  —  {args.duration}s hover run")
    print(f"  Using tuned parameter set from mavrik.parm")
    print(f"{'━'*64}\n")

    # 1. Cleanup old simulation instances
    kill_all("pre-flight cleanup")
    if DIAG_CSV.exists():
        DIAG_CSV.unlink()

    # Commented out parameter template copy to preserve tuned mavrik.parm gains
    # shutil.copy(str(DIR / "full_payload/mavrik.parm"), str(DIR / "mavrik.parm"))

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
    print("Waiting 3s for MAVRIK to connect to pre-arm stabilizer...")
    time.sleep(3)

    QHOVER = 18
    deadline = time.time() + 25.0
    print("  [arm] Arming via MAVLink...", end='', flush=True)
    armed = False
    while time.time() < deadline:
        for _ in range(3):
            mav_set_mode(QHOVER)
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

    # 5. Telemetry loop
    print(f"\nFlight active. Monitoring for {args.duration}s...")
    print(f"{'═'*85}")
    print(f" {'t[s]':<6} | {'Alt[ft]':<8} | {'Pitch[°]':<8} | {'Roll[°]':<8} | {'Yaw[°]':<8} | {'Speed[fps]':<10} | {'AvgMotor':<8}")
    print(f"{'═'*85}")

    start_t = time.time()
    end_t = start_t + args.duration

    def sigint_handler(sig, frame):
        print("\n\n[FlightTest] Interrupted by user. Cleaning up simulation...")
        kill_all("interrupted")
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    max_pitch = 0.0
    max_roll = 0.0
    max_yaw_err = 0.0
    takeoff_triggered = False

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
            speed = (u**2 + v**2)**0.5

            # Calculate average motor PWM
            pwm_vals = [float(r[k]) for k in ['thrFR', 'thrFL', 'thrAR', 'thrAL']]
            avg_motor = sum(pwm_vals) / len(pwm_vals)

            # Post-handover: QSTABILIZE — attitude hold only, no velocity/altitude braking.
            # QHOVER velocity loop saw 6fps forward and commanded nose-UP (AFT>FWD thrust),
            # creating asymmetric roll via aero coupling. QSTABILIZE is pure attitude hold.
            if not takeoff_triggered and r.get('armed') == '1':
                takeoff_triggered = True
                def handover_sequence():
                    print("\n  [handover] Bridge handover → QHOVER (18): altitude and velocity hold.")
                    for _ in range(5):
                        mav_set_mode(18)   # QHOVER: altitude and velocity hold
                    with _rc_override_lock:
                        _rc_override_channels[2] = 1500   # Standard throttle setpoint for QHOVER
                        # QHOVER sticks neutral
                        _rc_override_channels[1] = 1500   # CH2 pitch stick neutral
                threading.Thread(target=handover_sequence, daemon=True).start()

            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            max_yaw_err = max(max_yaw_err, abs(yaw))

            # Color-coded warnings for high divergence
            warn = ""
            if abs(pitch) > 15 or abs(roll) > 15:
                warn = " ⚠️ DIVERGENCE WARNING!"
            elif abs(pitch) > 45 or abs(roll) > 45:
                warn = " 🚨 CRITICAL FLIGHT EXCURSION!"

            print(f" {t:5.1f}s | {alt:8.1f} | {pitch:8.2f} | {roll:8.2f} | {yaw:8.2f} | {speed:10.2f} | {avg_motor:8.3f}{warn}")
        except Exception:
            pass

    # 6. Flight Summary and Cleanup
    print(f"\n{'═'*85}")
    print("Flight Test Complete. Summary statistics:")
    print(f"  Max Pitch Excursion: {max_pitch:.2f}°")
    print(f"  Max Roll Excursion:  {max_roll:.2f}°")
    print(f"  Max Yaw Deviation:   {max_yaw_err:.2f}°")
    print(f"{'═'*85}\n")

    kill_all("flight finished")

if __name__ == '__main__':
    main()
