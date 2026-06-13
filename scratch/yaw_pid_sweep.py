#!/usr/bin/env python3
"""
yaw_pid_sweep.py  — Systematic yaw PID tuning for MAVRIK QuadPlane.

Strategy:
  Step 1: I=0, D=0. Sweep Q_A_RAT_YAW_P from low→high to find P_crit.
  Step 2: Fix P, sweep D to damp overshoot.
  Step 3: Fix P+D, add tiny I with tight IMAX.

Run this once to patch mavrik.parm and full_payload/mavrik.parm with a chosen
set of yaw PID values, then manually run test_hover.py.
"""

import sys, re
from pathlib import Path

DIR = Path(__file__).parent.parent

PARAM_FILES = [
    DIR / "mavrik.parm",
    DIR / "full_payload" / "mavrik.parm",
]

# ── What to set ────────────────────────────────────────────────────────────────
# Edit these values before running:

YAW_RAT_P    = float(sys.argv[1]) if len(sys.argv) > 1 else 0.20
YAW_RAT_I    = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0    # Step 1: start at 0
YAW_RAT_D    = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0    # Step 1: start at 0
YAW_RAT_IMAX = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10   # tight cap
YAW_ANG_P    = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0    # outer loop

print(f"""
┌─────────────────────────────────────────────────────┐
│  Yaw PID Patch                                      │
│  Q_A_RAT_YAW_P    = {YAW_RAT_P:.4f}                     │
│  Q_A_RAT_YAW_I    = {YAW_RAT_I:.4f}                     │
│  Q_A_RAT_YAW_D    = {YAW_RAT_D:.4f}                     │
│  Q_A_RAT_YAW_IMAX = {YAW_RAT_IMAX:.4f}                     │
│  Q_A_ANG_YAW_P    = {YAW_ANG_P:.4f}                     │
└─────────────────────────────────────────────────────┘
""")

REPLACEMENTS = {
    "Q_A_RAT_YAW_P":    f"{YAW_RAT_P}",
    "Q_A_RAT_YAW_I":    f"{YAW_RAT_I}",
    "Q_A_RAT_YAW_D":    f"{YAW_RAT_D}",
    "Q_A_RAT_YAW_IMAX": f"{YAW_RAT_IMAX}",
    "Q_A_ANG_YAW_P":    f"{YAW_ANG_P}",
}

for pfile in PARAM_FILES:
    text = pfile.read_text()
    for param, val in REPLACEMENTS.items():
        text = re.sub(
            rf"^({re.escape(param)})\s+[\d.]+",
            rf"\1 {val}",
            text,
            flags=re.MULTILINE,
        )
    pfile.write_text(text)
    print(f"  ✓ patched {pfile.name}")

print()
print("Now run:  python3 test_hover.py")
print()
