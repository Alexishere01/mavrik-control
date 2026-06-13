#!/usr/bin/env python3
"""
auto_tune.py — Bayesian PID auto-tuner for MAVRIK + ArduPilot SITL.

Startup order (mirrors start_terminals.sh, then adds MAVRIK):
  1. kill_zombies.sh + rm eeprom.bin
  2. web_viewer, pid_vtol, bridge, SITL  ← wait STACK_READY_SECS (10s)
  3. mavrik.exe                           ← wait MAVRIK_RUN_SECS  (30s)
  4. kill_zombies.sh  →  score  →  next trial

Usage:
  source venv/bin/activate
  python3 auto_tune.py [--trials 40] [--flight-secs 30] [--resume]
"""

import argparse, csv, os, shutil, signal, socket, struct
import subprocess, sys, time
from pathlib import Path

# ─── Paths ────────────────────────────────────────────────────────────────────
DIR        = Path(__file__).parent.resolve()
VENV_PY    = sys.executable  # use same Python running this script — not venv/bin/python3 (Python 3.13)
WINE        = Path.home() / "Library/Application Support/com.isaacmarovitz.Whisky/Libraries/Wine/bin/wine64"
SIM_VEHICLE = DIR / "ardupilot/Tools/autotest/sim_vehicle.py"  # absolute path — not in subprocess PATH
WINEPREFIX = "/Users/alex/Library/Containers/com.isaacmarovitz.Whisky/Bottles/37084628-4FFA-403A-BD20-3D881F401401"

DIAG_CSV   = DIR / "bridge_diag.csv"
LOG_DIR    = DIR / "tune_logs"
RESULTS    = DIR / "tune_results.csv"
STUDY_DB   = DIR / "tune_study.db"
KILL_SH    = DIR / "kill_zombies.sh"

# ─── Timing ───────────────────────────────────────────────────────────────────
STACK_READY_SECS = 10   # wait after starting web_viewer/pid_vtol/bridge/SITL
MAVRIK_RUN_SECS  = 30   # how long MAVRIK runs per trial (includes arm + flight)
KILL_GRACE_SECS  = 2    # pause after kill before next trial

# ─── Raw MAVLink helpers (no external deps — copied from mavrik_web_viewer.py) ─
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

# ─── Parameter file ───────────────────────────────────────────────────────────
_PARM_TEMPLATE: str = ""

def load_parm_template():
    global _PARM_TEMPLATE
    _PARM_TEMPLATE = (DIR / "mavrik.parm").read_text()

def write_params(overrides: dict):
    """Stamp overrides into mavrik.parm, preserving all other lines."""
    lines = _PARM_TEMPLATE.splitlines()
    new_lines = []
    replaced = set()
    for line in lines:
        s = line.strip()
        if s.startswith('#') or not s:
            new_lines.append(line)
            continue
        key = s.split()[0]
        if key in overrides:
            new_lines.append(f"{key} {overrides[key]:.6g}")
            replaced.add(key)
        else:
            new_lines.append(line)
    for k, v in overrides.items():
        if k not in replaced:
            new_lines.append(f"{k} {v:.6g}")
    (DIR / "mavrik.parm").write_text('\n'.join(new_lines) + '\n')

# ─── Process management ───────────────────────────────────────────────────────
_procs: list[subprocess.Popen] = []

def kill_all(label: str = ""):
    """Kill tracked processes + run kill_zombies.sh."""
    global _procs
    for p in _procs:
        try:
            p.kill()
        except Exception:
            pass
    _procs.clear()
    subprocess.run(["bash", str(KILL_SH)], capture_output=True, timeout=15)
    (DIR / "eeprom.bin").unlink(missing_ok=True)
    time.sleep(KILL_GRACE_SECS)
    if label:
        print(f"  [kill] {label} ✓")

def _spawn(cmd, name, env):
    """Launch a subprocess detached, logging output to a file."""
    # Convert name like 'mavrik.exe (wine)' to a safe filename like 'mavrik.exe.log'
    safe_name = name.split()[0].replace('/', '_') + '.log'
    log_path = DIR / "tune_logs" / safe_name
    log_path.parent.mkdir(exist_ok=True)
    f = open(log_path, 'w')
    p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT)
    print(f"  [+] {name:<30} pid={p.pid}")
    _procs.append(p)
    return p

def start_stack():
    """Start all services in order. Blocks for STACK_READY_SECS before returning."""
    env = os.environ.copy()  # inherits full terminal PATH (pyenv shims, conda, etc.)
    py  = str(VENV_PY)       # sys.executable — same Python running auto_tune.py

    # Add pyenv shim dir to PATH so mavproxy.py is found by sim_vehicle's child procs.
    # We APPEND (not prepend) so shebang scripts still use their native Python.
    pyenv_shims = Path.home() / ".pyenv/shims"
    if str(pyenv_shims) not in env.get('PATH', ''):
        env['PATH'] = env.get('PATH', '/usr/bin:/bin') + f':{pyenv_shims}'

    # pid_vtol owns port 5006 pre-arm; bridge monitors + handles post-arm
    _spawn([py, "pid_vtol.py"],                "pid_vtol.py",                env)
    _spawn([py, "mavrik_ardupilot_bridge.py"], "mavrik_ardupilot_bridge.py", env)
    time.sleep(0.5)

    # SITL with absolute path so it's found regardless of cwd PATH
    _spawn(
        [py, str(SIM_VEHICLE),
         "-v", "ArduPlane", "-f", "quadplane",
         "--model", "JSON:127.0.0.1",
         f"--add-param-file={DIR}/mavrik.parm",
         "--out", "udpin:127.0.0.1:14552",
         "--wipe-eeprom", "--no-rebuild",
         "--mavproxy-args=--daemon --non-interactive"],
        "sim_vehicle.py (SITL)", env,
    )

    print(f"  [stack] Waiting {STACK_READY_SECS}s for all services to be ready...")
    time.sleep(STACK_READY_SECS)

def start_mavrik():
    """Start MAVRIK.exe via wine. Stack must already be running."""
    wine_env = os.environ.copy()
    wine_env['WINEPREFIX'] = WINEPREFIX
    wine_env['WINEDEBUG']  = '-all'
    _spawn(
        [str(WINE), "mavrik.exe", "input_Team1_hover.json"],
        "mavrik.exe (wine)", wine_env,
    )

def arm_vehicle(timeout: float = 25.0) -> bool:
    """
    Directly send MAVLink SET_MODE(QHOVER=20) + ARM to SITL on port 14552,
    polling bridge_diag.csv until ap_has_armed=1 or timeout.
    Bypasses web_viewer (which crashes headlessly) — auto_tune owns arming.
    """
    QHOVER = 18
    deadline = time.time() + timeout
    print("  [arm] Arming via MAVLink...", end='', flush=True)
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
                print(" ✓")
                return True
        except Exception:
            pass
        print('.', end='', flush=True)
    print(" ✗ timeout")
    return False

# ─── Scoring ─────────────────────────────────────────────────────────────────
def score_run(trial_id: int, flight_secs: float) -> dict:
    if not DIAG_CSV.exists():
        return {'score': 0.0, 'reason': 'no_csv'}

    rows = []
    try:
        with open(DIAG_CSV, newline='') as f:
            rows = list(csv.DictReader(f))
    except Exception as e:
        return {'score': 0.0, 'reason': f'csv_err:{e}'}

    if not rows:
        return {'score': 0.0, 'reason': 'empty'}

    # Find arm moment
    armed_idx = None
    for i, r in enumerate(rows):
        if r.get('ap_has_armed') == '1' and (i == 0 or rows[i-1].get('ap_has_armed') == '0'):
            armed_idx = i
            break

    if armed_idx is None:
        return {'score': 0.0, 'reason': 'never_armed',
                'near5_pct': 0.0, 'max_q': 0.0, 'time_to_crash': 0.0}

    post = rows[armed_idx:]
    if len(post) < 10:
        return {'score': 1.0, 'reason': 'too_short', 'near5_pct': 0.0, 'max_q': 0.0}

    arm_t = float(post[0]['sim_t'])
    total  = len(post)

    # Motor floor check (did MOT_SPIN_ARMED load correctly?)
    early_min = min(
        float(r[k]) for r in post[:20] for k in ['thrFR', 'thrFL', 'thrAR', 'thrAL']
    )
    motor_ok = early_min >= 0.05

    # Attitude metrics
    near5   = sum(1 for r in post if abs(float(r['pitch_deg'])) < 5  and abs(float(r['roll_deg'])) < 5)
    near10  = sum(1 for r in post if abs(float(r['pitch_deg'])) < 10 and abs(float(r['roll_deg'])) < 10)
    near5_pct  = near5  / total
    near10_pct = near10 / total

    # Longest continuous stable streak (rows @ ~50Hz → divide by 50 for seconds)
    max_streak = streak = 0
    for r in post:
        if abs(float(r['pitch_deg'])) < 5 and abs(float(r['roll_deg'])) < 5:
            streak += 1
            max_streak = max(max_streak, streak)
        else:
            streak = 0
    streak_secs = max_streak / 50.0

    # Angular rate extremes (penalise spin)
    max_q = max(abs(float(r['q_rads'])) for r in post)
    max_p = max(abs(float(r['p_rads'])) for r in post)

    # Time until first crash (>60°)
    time_to_crash = float(post[-1]['sim_t']) - arm_t
    for r in post:
        if abs(float(r['pitch_deg'])) > 60 or abs(float(r['roll_deg'])) > 60:
            time_to_crash = float(r['sim_t']) - arm_t
            break

    score = (
        near5_pct  * 60 +
        near10_pct * 20 +
        min(streak_secs / 10.0, 1.0) * 15 +
        min(time_to_crash / flight_secs, 1.0) * 5 +
        (5 if motor_ok else 0)
    )

    # Archive log
    LOG_DIR.mkdir(exist_ok=True)
    try:
        shutil.copy(DIAG_CSV, LOG_DIR / f"trial_{trial_id:04d}_s{score:.1f}.csv")
    except Exception:
        pass

    return {
        'score': score, 'reason': 'ok',
        'near5_pct': near5_pct, 'near10_pct': near10_pct,
        'streak_secs': streak_secs, 'time_to_crash': time_to_crash,
        'max_q': max_q, 'max_p': max_p,
        'motor_ok': motor_ok, 'early_min_motor': early_min,
        'n_post_arm': total,
    }

def _log(trial_id, overrides, result):
    COLS = ['trial', 'rat_p', 'rat_d', 'rat_i', 'ang_p', 'mot_spin',
            'score', 'near5_pct', 'near10_pct', 'streak_secs',
            'time_to_crash', 'max_q', 'max_p', 'motor_ok', 'early_min_motor', 'reason']
    write_hdr = not RESULTS.exists()
    with open(RESULTS, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS, extrasaction='ignore')
        if write_hdr:
            w.writeheader()
        w.writerow({
            'trial':    trial_id,
            'rat_p':    overrides.get('Q_A_RAT_PIT_P', ''),
            'rat_d':    overrides.get('Q_A_RAT_PIT_D', ''),
            'rat_i':    overrides.get('Q_A_RAT_PIT_I', ''),
            'ang_p':    overrides.get('Q_A_ANG_PIT_P', ''),
            'mot_spin': overrides.get('MOT_SPIN_ARMED', ''),
            **result,
        })

# ─── Optuna objective ─────────────────────────────────────────────────────────
def make_objective(flight_secs: float):
    def objective(trial):
        n = trial.number

        rat_p    = trial.suggest_float('rat_p',    0.005, 0.10,  log=True)
        rat_d    = trial.suggest_float('rat_d',    0.001, 0.04,  log=True)
        rat_i    = trial.suggest_float('rat_i',    0.001, 0.015, log=True)
        ang_p    = trial.suggest_float('ang_p',    1.5,   5.0)
        mot_spin = trial.suggest_float('mot_spin', 0.05,  0.11)

        overrides = {
            'Q_A_RAT_PIT_P':    rat_p,
            'Q_A_RAT_PIT_I':    rat_i,
            'Q_A_RAT_PIT_D':    rat_d,
            'Q_A_RAT_PIT_IMAX': min(rat_p * 20, 0.20),
            'Q_A_RAT_RLL_P':    rat_p,
            'Q_A_RAT_RLL_I':    rat_i,
            'Q_A_RAT_RLL_D':    rat_d,
            'Q_A_RAT_RLL_IMAX': min(rat_p * 20, 0.20),
            'Q_A_ANG_PIT_P':    ang_p,
            'Q_A_ANG_RLL_P':    ang_p,
            # QuadPlane motor params use Q_M_ prefix (NOT MOT_ — Copter-only, ignored by AP)
            'Q_M_SPIN_ARM':     mot_spin,
            'Q_M_SPIN_MIN':     0.05,
            'Q_M_SPOOL_TIME':   0.0,
            'Q_M_THST_HOVER':   0.125,  # CRITICAL: prevents drop in QHOVER before I-term builds
        }

        print(f"\n{'═'*64}")
        print(f"  Trial {n:3d}  |  RAT_P={rat_p:.4f}  RAT_D={rat_d:.4f}  "
              f"RAT_I={rat_i:.5f}  ANG_P={ang_p:.2f}  SPIN={mot_spin:.3f}")
        print(f"{'═'*64}")

        # ── 1. Clean slate ──────────────────────────────────────────────────
        kill_all("pre-trial cleanup")
        write_params(overrides)

        # ── 2. Start stack (web_viewer, pid_vtol, bridge, SITL) ─────────────
        print(f"  [stack] Starting services...")
        start_stack()   # blocks for STACK_READY_SECS

        # ── 3. Start MAVRIK ─────────────────────────────────────────────────
        start_mavrik()
        time.sleep(3)  # let MAVRIK connect to pid_vtol before arming
        armed = arm_vehicle(timeout=20.0)
        if not armed:
            print("  [arm] Failed to arm — skipping flight time")
            kill_all("post-trial")
            result = {'score': 0.0, 'reason': 'never_armed',
                      'near5_pct': 0.0, 'max_q': 0.0, 'time_to_crash': 0.0}
            _log(n, overrides, result)
            print(f"  → score=0.00  (never_armed)")
            return 0.0

        print(f"  [mavrik] Running for {flight_secs}s post-arm...")
        time.sleep(flight_secs)

        # ── 4. Kill & score ─────────────────────────────────────────────────
        kill_all("post-trial")
        result = score_run(n, flight_secs)
        score  = result['score']

        _log(n, overrides, result)

        print(f"  → score={score:.2f}  "
              f"near5={result.get('near5_pct', 0)*100:.0f}%  "
              f"streak={result.get('streak_secs', 0):.1f}s  "
              f"crash@{result.get('time_to_crash', 0):.1f}s  "
              f"motor_ok={result.get('motor_ok', '?')}  "
              f"({result.get('reason', '')})")
        return score
    return objective

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='Bayesian PID auto-tuner for MAVRIK')
    ap.add_argument('--trials',      type=int,   default=40)
    ap.add_argument('--flight-secs', type=float, default=30,
                    help='Total MAVRIK run time per trial inc. arm (default 30)')
    ap.add_argument('--fresh', action='store_true',
                    help='Delete existing tune_study.db and start over')
    args = ap.parse_args()

    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    load_parm_template()
    LOG_DIR.mkdir(exist_ok=True)

    if args.fresh and STUDY_DB.exists():
        STUDY_DB.unlink()
        print(f"  [fresh] Deleted {STUDY_DB.name}")

    study = optuna.create_study(
        study_name     = 'mavrik_pid',
        direction      = 'maximize',
        storage        = f'sqlite:///{STUDY_DB}',
        load_if_exists = True,   # always resume if DB exists
        sampler        = optuna.samplers.TPESampler(seed=42),
    )

    # Seed with best params from previous round (trial 19, score=5.44)
    # These are now meaningful since Q_M_SPIN_ARM is fixed — re-evaluate from here.
    if len(study.trials) == 0:
        study.enqueue_trial({
            'rat_p': 0.065, 'rat_d': 0.005, 'rat_i': 0.001,
            'ang_p': 3.82,  'mot_spin': 0.37,
        })
        # Also try a more aggressive D from tuning guide (D first, then P)
        study.enqueue_trial({
            'rat_p': 0.050, 'rat_d': 0.015, 'rat_i': 0.001,
            'ang_p': 3.0,   'mot_spin': 0.37,
        })

    secs_per_trial = STACK_READY_SECS + args.flight_secs + KILL_GRACE_SECS * 2 + 3
    eta_min = args.trials * secs_per_trial / 60
    print(f"\n{'━'*64}")
    print(f"  MAVRIK Auto-Tuner  —  {args.trials} trials × {args.flight_secs}s/trial")
    print(f"  Stack wait: {STACK_READY_SECS}s  |  MAVRIK run: {args.flight_secs}s")
    print(f"  ETA: ~{eta_min:.0f} min  ({secs_per_trial:.0f}s/trial)")
    print(f"  Results:   {RESULTS.name}")
    print(f"  Logs:      {LOG_DIR.name}/")
    print(f"  Study DB:  {STUDY_DB.name}")
    print(f"{'━'*64}")
    print(f"  Tip: watch -n3 'sort -t, -k7 -nr tune_results.csv | head -8'")
    print()

    env = os.environ.copy()
    py = sys.executable
    viewer_proc = subprocess.Popen([py, "mavrik_web_viewer.py"], env=env)
    print(f"  [+] Started persistent Web Viewer on http://localhost:8080 (pid={viewer_proc.pid})")

    def _sigint(sig, frame):
        print("\n[AutoTune] Ctrl-C — killing sim and printing best params...")
        kill_all("interrupted")
        viewer_proc.kill()
        _print_best(study)
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    try:
        study.optimize(make_objective(args.flight_secs), n_trials=args.trials)
    finally:
        kill_all("finished")
        viewer_proc.kill()
        _print_best(study)

def _print_best(study):
    try:
        b = study.best_trial
        p = b.params
        print(f"\n{'━'*64}")
        print(f"  BEST  trial={b.number}  score={b.value:.2f}")
        print(f"{'━'*64}")
        print(f"  Q_A_RAT_PIT_P  {p['rat_p']:.5f}")
        print(f"  Q_A_RAT_PIT_D  {p['rat_d']:.5f}")
        print(f"  Q_A_RAT_PIT_I  {p['rat_i']:.5f}")
        print(f"  Q_A_ANG_PIT_P  {p['ang_p']:.3f}")
        print(f"  MOT_SPIN_ARMED {p['mot_spin']:.3f}")
        print(f"\n  (same values used for RLL axes)")
    except Exception:
        print("  No completed trials yet.")

if __name__ == '__main__':
    main()
