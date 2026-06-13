#!/usr/bin/env python3
"""
test_hover.py — QSTABILIZE hover stability validation for MAVRIK + ArduPilot SITL.
Uses IDENTICAL infrastructure to test_transition.py (same SITL cmd, MAVRIK launch, MAVLink helpers).
Pass condition: pitch and roll stay within ±PITCH_LIMIT / ±ROLL_LIMIT for HOVER_DURATION seconds.
No FBWA transition — pure hover tune.
"""

import argparse, csv, os, shutil, signal, socket, struct, subprocess, sys, time, threading
from pathlib import Path

# ── Config ─────────────────────────────────────────────────────────────────────
HOVER_DURATION = 30      # seconds AP must hold stable hover to pass
PITCH_LIMIT    = 10.0    # deg absolute
ROLL_LIMIT     = 10.0    # deg absolute

# ── Paths ──────────────────────────────────────────────────────────────────────
DIR         = Path(__file__).parent.resolve()
VENV_PY     = sys.executable
WINE        = Path.home() / "Library/Application Support/com.isaacmarovitz.Whisky/Libraries/Wine/bin/wine64"
SIM_VEHICLE = DIR / "ardupilot/Tools/autotest/sim_vehicle.py"
WINEPREFIX  = "/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401"
DIAG_CSV    = DIR / "bridge_diag.csv"
KILL_SH     = DIR / "kill_zombies.sh"

# ── RC override (same as test_transition.py) ───────────────────────────────────
_rc_override_lock     = threading.Lock()
_rc_override_channels = [1500, 1540, 1420, 1500, 0, 0, 0, 0]
_rc_override_stop     = threading.Event()
_rc_thread            = None

def rc_override_thread_fn():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not _rc_override_stop.is_set():
        with _rc_override_lock:
            ch = list(_rc_override_channels)
        payload = struct.pack('<HHHHHHHHBB', *ch, 1, 1)
        pkt = _mav_pkt(70, payload, 124)
        try:
            s.sendto(pkt, ('127.0.0.1', 14552))
        except Exception:
            pass
        time.sleep(0.05)
    s.close()

# ── Process tracking ───────────────────────────────────────────────────────────
_procs: list[subprocess.Popen] = []

def kill_all(label=""):
    global _procs, _rc_thread
    _rc_override_stop.set()
    if _rc_thread is not None:
        _rc_thread.join(timeout=2.0)
        _rc_thread = None
    for p in _procs:
        try: p.kill()
        except: pass
    _procs.clear()
    subprocess.run(["bash", str(KILL_SH)], capture_output=True, timeout=15)
    (DIR / "eeprom.bin").unlink(missing_ok=True)
    time.sleep(2)
    if label:
        print(f"  [kill] {label} ✓")

def _spawn(cmd, name, env):
    log = DIR / "tune_logs" / (name.split()[0].replace('/', '_') + '.log')
    log.parent.mkdir(exist_ok=True)
    f = open(log, 'w')
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT,
                         cwd=str(DIR))
    print(f"  [+] {name:<30} pid={p.pid}")
    _procs.append(p)
    return p

# ── MAVLink helpers (identical to test_transition.py) ─────────────────────────
_mav_seq = 0

def _x25crc(data):
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def _mav_pkt(msg_id, payload, crc_extra):
    global _mav_seq
    seq = _mav_seq & 0xFF
    _mav_seq += 1
    hdr = bytes([len(payload), seq, 255, 0, msg_id])
    crc = _x25crc(hdr + payload + bytes([crc_extra]))
    return b'\xFE' + hdr + payload + struct.pack('<H', crc)

def _send_mav(pkt):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(pkt, ('127.0.0.1', 14552))

def mav_set_mode(mode_id):
    payload = struct.pack('<LBB', mode_id, 1, 1)
    _send_mav(_mav_pkt(11, payload, 89))

def mav_arm(force=True):
    f = 2989.0 if force else 0.0
    payload = struct.pack('<fffffffHBBB', 1.0, f, 0, 0, 0, 0, 0, 400, 1, 1, 0)
    _send_mav(_mav_pkt(76, payload, 152))

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    print()
    print("━"*70)
    print("  MAVRIK QSTABILIZE Hover Tune Test")
    print(f"  Pass: pitch/roll < ±{PITCH_LIMIT}° for {HOVER_DURATION}s")
    print("━"*70)
    print()

    signal.signal(signal.SIGINT,  lambda *_: (kill_all("interrupted"), sys.exit(1)))
    signal.signal(signal.SIGTERM, lambda *_: (kill_all("terminated"),  sys.exit(1)))

    # 1. Clean slate
    kill_all("pre-flight cleanup")
    if DIAG_CSV.exists():
        DIAG_CSV.unlink()

    # 2. Environment
    env = os.environ.copy()
    pyenv_shims = Path.home() / ".pyenv/shims"
    if str(pyenv_shims) not in env.get('PATH', ''):
        env['PATH'] = env.get('PATH', '/usr/bin:/bin') + f':{pyenv_shims}'

    print("Starting background services...")
    _spawn([str(VENV_PY), "pid_vtol.py"],                "pid_vtol.py",                env)
    _spawn([str(VENV_PY), "mavrik_ardupilot_bridge.py"], "mavrik_ardupilot_bridge.py", env)
    time.sleep(0.5)

    # SITL — exact command from test_transition.py
    _spawn(
        [str(VENV_PY), str(SIM_VEHICLE),
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

    # 3. Start MAVRIK under Wine — exact from test_transition.py
    print("Starting MAVRIK engine under Wine...")
    wine_env = os.environ.copy()
    wine_env['WINEPREFIX'] = WINEPREFIX
    wine_env['WINEDEBUG']  = '-all'
    _spawn([str(WINE), "mavrik.exe", "input_Team1_hover.json"], "mavrik.exe (wine)", wine_env)
    time.sleep(3)

    # 4. Wait for pre-arm controller to stabilize (sim_t > 20s)
    print("Waiting for MAVRIK to connect and stabilize under pre-arm controller...")
    connected = False
    conn_deadline = time.time() + 60.0
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
        kill_all("MAVRIK connection timed out")
        print("[FAIL] Pre-arm controller never stabilized")
        sys.exit(1)

    # 5. Arm in QSTABILIZE (pure manual attitude control)
    QSTABILIZE = 17
    deadline = time.time() + 25.0
    print("  [arm] Arming in QSTABILIZE...", end='', flush=True)
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
        kill_all("arm timeout")
        sys.exit(1)

    with open(DIAG_CSV, newline='') as f:
        rows = list(csv.DictReader(f))
    arm_sim_t = float(next(r['sim_t'] for r in rows if r.get('ap_has_armed') == '1'))
    print(f"  [arm_sim_t] {arm_sim_t:.2f}s")

    # 6. Monitor hover for HOVER_DURATION seconds
    print()
    print(f"  Monitoring QSTABILIZE hover for {HOVER_DURATION}s...")
    print(f"  {'sim_t':>7}  {'pitch':>8}  {'roll':>8}  {'u_fps':>8}  {'alt':>7}  {'thr':>4}  status")
    print(f"  {'-'*75}")

    hover_start     = None
    diverged        = False
    max_pitch_seen  = 0.0
    max_roll_seen   = 0.0
    last_sim_t      = arm_sim_t
    end_t           = time.time() + 120   # 2-min wall-clock budget

    while time.time() < end_t:
        time.sleep(0.4)
        try:
            with open(DIAG_CSV, newline='') as f:
                rows = list(csv.DictReader(f))
            new_rows = [r for r in rows
                        if r.get('ap_has_armed') == '1'
                        and float(r['sim_t']) > last_sim_t]
            if not new_rows:
                continue

            r     = new_rows[-1]
            last_sim_t = float(r['sim_t'])
            t     = last_sim_t
            pitch = float(r['pitch_deg'])
            roll  = float(r['roll_deg'])
            u     = float(r['u_fps'])
            alt   = float(r['alt_ft'])
            w     = float(r['w_fps'])

            # PD altitude controller to hold altitude near 3000 ft and avoid weathercock pitching moment
            alt_err = 3000.0 - alt
            climb_rate = -w
            new_thr = int(1415 + alt_err * 15.0 - climb_rate * 10.0)
            new_thr = max(1200, min(1480, new_thr))
            with _rc_override_lock:
                _rc_override_channels[2] = new_thr

            max_pitch_seen = max(max_pitch_seen, abs(pitch))
            max_roll_seen  = max(max_roll_seen,  abs(roll))

            if abs(pitch) < PITCH_LIMIT and abs(roll) < ROLL_LIMIT:
                if hover_start is None:
                    hover_start = t
                elapsed = t - hover_start
                status = f"STABLE {elapsed:.1f}/{HOVER_DURATION}s"
            else:
                hover_start = None
                status = f"⚠  pitch={pitch:+.1f} roll={roll:+.1f}"

            if abs(pitch) > 30 or abs(roll) > 30:
                diverged = True
                print(f"  {t:7.1f}  {pitch:+8.2f}  {roll:+8.2f}  {u:8.1f}  {alt:7.1f}  {new_thr:4d}  🚨 DIVERGED — stopping")
                break

            print(f"  {t:7.1f}  {pitch:+8.2f}  {roll:+8.2f}  {u:8.1f}  {alt:7.1f}  {new_thr:4d}  {status}")

            if hover_start is not None and (t - hover_start) >= HOVER_DURATION:
                print(f"\n  ✓ Held stable for {t - hover_start:.1f}s!")
                break

        except Exception:
            pass

    # 7. Summary
    stable_time = (last_sim_t - hover_start) if hover_start is not None else 0.0
    passed = (not diverged) and (stable_time >= HOVER_DURATION)

    print()
    print("━"*70)
    print("  QSTABILIZE Hover Test — Summary")
    print("━"*70)
    print(f"  Max pitch excursion : {max_pitch_seen:.2f}°  (limit ±{PITCH_LIMIT}°)")
    print(f"  Max roll  excursion : {max_roll_seen:.2f}°  (limit ±{ROLL_LIMIT}°)")
    print(f"  Stable time         : {stable_time:.1f}s  (need {HOVER_DURATION}s)")
    print(f"  Result: {'✓ PASS' if passed else '✗ FAIL'}")
    print("━"*70)
    print()

    kill_all("test complete")
    sys.exit(0 if passed else 1)

if __name__ == '__main__':
    main()
