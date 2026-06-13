#!/usr/bin/env python3
"""
validate_mapping.py — Runtime cross-check of motor-channel and tilt-servo assignments.

Reads bridge_diag.csv and runs four diagnostics:

  1. Roll axis sign   — corr(left_diff, p_rads) direction vs closed-loop expectation
  2. Roll decoupling  — |corr(left_diff, p_rads)| vs |corr(left_diff, q_rads)|
  3. Pitch decoupling — |corr(front_diff, q_rads)| vs |corr(front_diff, p_rads)|
  4. Yaw tilt         — tilt activation check + corr(flapsL−flapsR, r_rads)

Sign convention for CLOSED-LOOP correction-dominated data
----------------------------------------------------------
  In stable QHOVER (no manual inputs), ArduPilot OPPOSES errors:
    p > 0 (rolling right) → AP commands left motors DOWN → left_diff < 0
    ⟹ corr(left_diff, p_rads) is NEGATIVE for correct mapping
    ⟹ POSITIVE would mean AP amplifies roll errors → wrong mapping → drone crashes

  Identical logic for pitch:
    q > 0 (nose up) → AP commands front motors DOWN → front_diff < 0
    ⟹ corr(front_diff, q_rads) is NEGATIVE for correct mapping

  For step-input data (deliberate RC inputs), signs REVERSE: both positive.
  The cross-axis DECOUPLING RATIO works regardless of mode — use it as primary check.

Cross-axis decoupling ratios (mode-independent)
------------------------------------------------
  If CH2 and CH4 are swapped, the roll command becomes a DIAGONAL mix (FR↑ + AL↓ vs FL↑ + AR↓).
  A diagonal mix creates nearly equal coupling to roll and pitch.
  Correct assignment: |roll_primary| / |roll_cross| > 2.0

Usage
-----
  python validate_mapping.py [bridge_diag.csv] [--tstart WALL_T] [--tend WALL_T]

Test procedures (run in QHOVER, log to bridge_diag.csv)
--------------------------------------------------------
  1. Hover stably ≥10 s with roll/pitch < 5° (needed for clean correction-mode baselines).
  2. Roll test   : RC roll right ~30 % → hold 3 s → centre → left 3 s → centre.
  3. Pitch test  : RC pitch fwd  ~30 % → hold 3 s → centre → back 3 s → centre.
  4. Yaw test    : RC yaw right  ~30 % → hold 3 s → centre → left 3 s → centre.
  5. Land. Run: python validate_mapping.py bridge_diag.csv
"""

import argparse
import csv
import math
import sys

VTOL_FLAP = 0.99


def load_csv(path: str) -> list:
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                rows.append({k: float(v) for k, v in row.items()})
            except (ValueError, KeyError):
                continue
    return rows


def pearson(x: list, y: list) -> float:
    n = len(x)
    if n < 5:
        return float("nan")
    mx = sum(x) / n
    my = sum(y) / n
    num = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y))
    sx = math.sqrt(sum((xi - mx) ** 2 for xi in x) / n)
    sy = math.sqrt(sum((yi - my) ** 2 for yi in y) / n)
    if sx < 1e-12 or sy < 1e-12:
        return float("nan")
    return (num / n) / (sx * sy)


def _bar(v: float, w: int = 20) -> str:
    if math.isnan(v):
        return " " * w
    filled = int(abs(v) * w)
    bar = "#" * min(filled, w)
    return f"{'':>{w-len(bar)}}{bar}" if v >= 0 else f"{bar}{'':>{w-len(bar)}}"


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", nargs="?", default="bridge_diag.csv")
    parser.add_argument("--tstart", type=float, default=None)
    parser.add_argument("--tend",   type=float, default=None)
    args = parser.parse_args()

    rows = load_csv(args.csv_path)
    print(f"Loaded {len(rows)} rows from '{args.csv_path}'")

    if args.tstart is not None:
        rows = [r for r in rows if r["wall_clock"] >= args.tstart]
    if args.tend is not None:
        rows = [r for r in rows if r["wall_clock"] <= args.tend]

    armed = [r for r in rows if r["armed"] > 0.5 and r["motor_active"] > 0.5]
    print(f"Armed + motor_active rows: {len(armed)}")

    if len(armed) < 10:
        print("\nNot enough armed data. Fly a test session then re-run.")
        _print_procedures()
        return

    # ── signals ───────────────────────────────────────────────────────────────
    # Motor assignment claimed by bridge / CLAUDE.md §6 (empirically verified):
    #   ap_pwm1 = CH1 → thrFR  (Front Right, SERVO1 func 33 = Motor1)
    #   ap_pwm2 = CH2 → thrAL  (Aft Left,   SERVO2 func 34 = Motor2)
    #   ap_pwm3 = CH3 → thrFL  (Front Left,  SERVO3 func 35 = Motor3)
    #   ap_pwm4 = CH4 → thrAR  (Aft Right,   SERVO4 func 36 = Motor4)
    left_diff  = [(r["ap_pwm2"] + r["ap_pwm3"]) - (r["ap_pwm1"] + r["ap_pwm4"])
                  for r in armed]
    front_diff = [(r["ap_pwm1"] + r["ap_pwm3"]) - (r["ap_pwm2"] + r["ap_pwm4"])
                  for r in armed]

    p  = [r["p_rads"]  for r in armed]   # body roll rate   (+ve = roll right)
    q  = [r["q_rads"]  for r in armed]   # body pitch rate  (+ve = nose up)
    r_yaw = [r["r_rads"] for r in armed] # body yaw rate    (+ve = yaw right CW)

    fl = [r["flapsL"] for r in armed]
    fr = [r["flapsR"] for r in armed]
    td = [a - b for a, b in zip(fl, fr)]  # flapsL − flapsR

    # ── correlations ──────────────────────────────────────────────────────────
    c_rp = pearson(left_diff,  p)
    c_rq = pearson(left_diff,  q)
    c_qp = pearson(front_diff, p)
    c_qq = pearson(front_diff, q)
    c_ty = pearson(td,         r_yaw)

    # ── Test 1: Roll axis sign ─────────────────────────────────────────────────
    print(f"\n{'═'*62}")
    print("  Test 1 — Roll axis sign")
    print(f"{'═'*62}")
    print(f"  corr( (ap_pwm2+ap_pwm3)−(ap_pwm1+ap_pwm4),  p_rads ) = {c_rp:+.3f}")
    if not math.isnan(c_rp):
        if c_rp < -0.05:
            print("  ✓ NEGATIVE — AP corrects roll errors correctly (correction mode).")
            print("    CH2=thrAL, CH4=thrAR consistent with stable closed-loop roll control.")
        elif c_rp > 0.05:
            print("  ✗ POSITIVE in correction-mode data — AP appears to amplify roll errors.")
            print("    This is the signature of WRONG mapping. A drone with this behaviour")
            print("    would diverge quickly; if it flew, re-run with fresh step-input data.")
        else:
            print("  ? NEAR ZERO — either insufficient roll excitation or very weak coupling.")
    print()

    # ── Test 2: Roll decoupling ratio ──────────────────────────────────────────
    print(f"{'═'*62}")
    print("  Test 2 — Roll axis decoupling")
    print(f"{'═'*62}")
    print(f"  corr( left_diff, p_rads ) = {c_rp:+.3f}   (primary roll coupling)")
    print(f"  corr( left_diff, q_rads ) = {c_rq:+.3f}   (cross-axis — should be small)")
    EPS = 0.001
    if not (math.isnan(c_rp) or math.isnan(c_rq)):
        ratio = abs(c_rp) / max(abs(c_rq), EPS)
        print(f"  |primary|/|cross| = {ratio:.1f}")
        if ratio >= 2.0:
            print("  ✓ Roll commands couple to roll (not pitch) — decoupling OK.")
        elif ratio >= 1.2:
            print("  ~ Marginal decoupling. Repeat with more roll excitation.")
        else:
            print("  ✗ LOW RATIO — roll commands coupling comparably to pitch and roll.")
            print("    Possible cause: CH2/CH4 swapped in bridge → diagonal motor mix.")
            print("    Fix: swap thr_al/thr_ar assignments in bridge lines 373–374.")
    print()

    # ── Test 3: Pitch decoupling ratio ─────────────────────────────────────────
    print(f"{'═'*62}")
    print("  Test 3 — Pitch axis decoupling")
    print(f"{'═'*62}")
    print(f"  corr( front_diff, q_rads ) = {c_qq:+.3f}   (primary pitch coupling)")
    print(f"  corr( front_diff, p_rads ) = {c_qp:+.3f}   (cross-axis — should be small)")
    if not (math.isnan(c_qq) or math.isnan(c_qp)):
        ratio_p = abs(c_qq) / max(abs(c_qp), EPS)
        print(f"  |primary|/|cross| = {ratio_p:.1f}")
        if ratio_p >= 2.0:
            print("  ✓ Pitch commands couple to pitch — decoupling OK.")
        elif abs(c_qq) < 0.05 and abs(c_qp) < 0.10:
            print("  ? BOTH WEAK — insufficient pitch excitation. Command deliberate pitch step.")
        elif ratio_p >= 1.2:
            print("  ~ Marginal. Repeat with more pitch excitation.")
        else:
            print("  ✗ LOW RATIO — pitch commands coupling comparably to pitch and roll.")
            print("    Possible cause: front/rear swap or large persistent pitch offset corrupting stats.")
            print("    Verify: run a dedicated pitch step test and re-analyse that window.")
    print()

    # ── Test 4: Yaw tilt activation and sign ───────────────────────────────────
    print(f"{'═'*62}")
    print("  Test 4 — Yaw tilt coupling")
    print(f"{'═'*62}")
    tilt_off_snap = sum(
        1 for a, b in zip(fl, fr) if abs(a - VTOL_FLAP) > 0.02 or abs(b - VTOL_FLAP) > 0.02
    )
    pct = int(100 * tilt_off_snap / max(len(armed), 1))
    max_td = max(abs(d) for d in td) if td else 0
    print(f"  Rows with tilt off snap-band: {tilt_off_snap}/{len(armed)} ({pct}%)")
    print(f"  Max |flapsL−flapsR|: {max_td:.4f}  (want >0.05 for yaw test)")
    if pct == 0:
        print("  → Tilt never left snap band [1440–1560]. Run a yaw step test.")
    else:
        print(f"  corr( flapsL−flapsR, r_rads ) = {c_ty:+.3f}")
        if not math.isnan(c_ty):
            if abs(c_ty) < 0.05:
                print("  ? NEAR ZERO — weak yaw excitation in tilt-active rows.")
            elif c_ty > 0:
                print("  Positive: tilt_diff↑ with r↑.")
                print("  In CORRECTION mode this means FL tilts less forward when drone yaws right")
                print("  (AP resisting the yaw) — expected correct sign. ✓")
                print("  In STEP-INPUT mode a positive sign is UNEXPECTED — verify visually.")
            else:
                print("  Negative: tilt_diff↓ with r↑.")
                print("  In STEP-INPUT mode: FL fwd (flapsL↓) → yaw right → expected correct sign. ✓")
                print("  In CORRECTION mode a negative sign is UNEXPECTED — check SERVO8/9 reversed.")
    print()

    # ── Activity summary ───────────────────────────────────────────────────────
    print(f"{'═'*62}")
    print("  Activity summary")
    print(f"{'═'*62}")
    max_roll_exc  = max(abs(d) for d in left_diff)  if left_diff  else 0
    max_pitch_exc = max(abs(d) for d in front_diff) if front_diff else 0

    mean_front = sum(r["ap_pwm1"] + r["ap_pwm3"] for r in armed) / (2 * len(armed))
    mean_rear  = sum(r["ap_pwm2"] + r["ap_pwm4"] for r in armed) / (2 * len(armed))
    std_roll   = math.sqrt(sum((v - sum(p)/len(p))**2 for v in p) / len(p)) if p else 0
    std_pitch  = math.sqrt(sum((v - sum(q)/len(q))**2 for v in q) / len(q)) if q else 0

    def _flag(v, thr, unit): return "✓" if v > thr else "LOW"
    print(f"  Max left−right PWM diff:  {max_roll_exc:6.0f}  (want >30)   {_flag(max_roll_exc,30,'')}")
    print(f"  Max front−rear PWM diff:  {max_pitch_exc:6.0f}  (want >30)   {_flag(max_pitch_exc,30,'')}")
    print(f"  Std p_rads (roll rate):   {std_roll:.4f} rad/s")
    print(f"  Std q_rads (pitch rate):  {std_pitch:.4f} rad/s")
    print(f"  Mean front motor PWM:     {mean_front:7.1f}")
    print(f"  Mean rear  motor PWM:     {mean_rear:7.1f}")
    rear_ok = "✓ (rear heavier = aft CG)" if mean_rear > mean_front else "? (front heavier than rear)"
    print(f"  Rear > front:             {rear_ok}")

    if mean_front < 1200:
        print()
        print("  *** Front motors averaging <1200 PWM — large persistent pitch offset.")
        print("      Statistics may be contaminated by ramp-up or pitch-divergence artefacts.")
        print("      For clean decoupling analysis, filter to a stable hover window with")
        print("      --tstart / --tend to isolate ≥10 s of level flight.")

    print()
    _print_procedures()


def _print_procedures() -> None:
    print("""
────────────────────────────────────────────────────────────────
TEST PROCEDURES  (one flight session in QHOVER)
────────────────────────────────────────────────────────────────
Arm and hover stably for ≥10 s before starting inputs.
This lets the correction-mode baseline settle (roll/pitch < 5°).

1. ROLL TEST
   RC roll right ~30 % → hold 3 s → centre → left 30 % → hold 3 s → centre.
   Expected: ap_pwm2 (thrAL) and ap_pwm3 (thrFL) rise together for right roll.
   Wrong sign → swap thr_al/thr_ar in bridge lines 373–374.

2. PITCH TEST
   RC pitch fwd ~30 % → hold 3 s → centre → back 30 % → hold 3 s → centre.
   Expected: ap_pwm1 (thrFR) and ap_pwm3 (thrFL) rise together for fwd pitch.

3. YAW TEST
   RC yaw right ~30 % → hold 3 s → centre → left 30 % → hold 3 s → centre.
   Expected: flapsL drops below 0.99 (FL tilts forward) when yawing right.
   Wrong sign → set SERVO8_REVERSED 1 in mavrik.parm or swap flaps_l/r in bridge.

Analysing a specific time window:
  python validate_mapping.py bridge_diag.csv --tstart 1778432460 --tend 1778432520

Aero surfaces (CH5/6/7) are near-zero in VTOL mode and cannot be validated here.
Validate in FBWA or QLOITER transition once VTOL mapping is confirmed.
────────────────────────────────────────────────────────────────
""")


if __name__ == "__main__":
    main()
