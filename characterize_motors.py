#!/usr/bin/env python3
"""
characterize_motors.py
======================
Systematic motor characterization for MAVRIK VTOL.

Sends pre-arm control packets directly to MAVRIK (bypassing ArduPilot)
with step inputs to individual motors and records the attitude response.
This empirically validates the mixing matrix used in the bridge.

Run with MAVRIK running but BEFORE ArduPilot arms:
    python3 characterize_motors.py

Output: motor_response_matrix.csv with columns:
    motor_step, roll_rate_dps, pitch_rate_dps, yaw_rate_dps
"""

import csv
import socket
import struct
import time
import math

# --- Ports ---
MAVRIK_STATE_PORT   = 5009
MAVRIK_CONTROL_PORT = 5006
MAVRIK_CONTROL_IP   = "127.0.0.1"

MAVRIK_STATE_FMT   = "<14f"
MAVRIK_STATE_SIZE  = struct.calcsize(MAVRIK_STATE_FMT)
MAVRIK_CONTROL_FMT = "<9f"

# Hover trim (from Team1_VTOL.json)
TRIM_FWD = 0.410
TRIM_AFT = 0.342
VTOL_FLAP = 0.99
STEP_DELTA = 0.10   # +10% step input above trim
STEP_HOLD  = 0.5    # hold step for 0.5s and measure

MOTOR_NAMES = ["thrFR", "thrFL", "thrAR", "thrAL"]

# Control packet order: ail, ele, rud, thrFR, thrFL, thrAR, thrAL, flapsR, flapsL
BASE_CTRL = (0.0, 0.0, 0.0, TRIM_FWD, TRIM_FWD, TRIM_AFT, TRIM_AFT, VTOL_FLAP, VTOL_FLAP)


def make_control(motor_idx: int, delta: float) -> tuple:
    """Return a control tuple with one motor stepped up by delta."""
    ctrl = list(BASE_CTRL)
    ctrl[3 + motor_idx] += delta
    ctrl[3 + motor_idx] = min(1.0, max(0.0, ctrl[3 + motor_idx]))
    return tuple(ctrl)


def recv_state(sock) -> dict | None:
    """Drain socket and return latest MAVRIK state as a dict, or None."""
    latest = None
    try:
        while True:
            data, _ = sock.recvfrom(4096)
            if len(data) >= MAVRIK_STATE_SIZE:
                latest = data
    except BlockingIOError:
        pass
    if latest is None:
        return None
    vals = struct.unpack(MAVRIK_STATE_FMT, latest[:MAVRIK_STATE_SIZE])
    keys = ["t", "u", "v", "w", "p", "q", "r", "xf", "yf", "zf", "e0", "ex", "ey", "ez"]
    return dict(zip(keys, vals))


def send_control(ctrl_sock, ctrl):
    pkt = struct.pack(MAVRIK_CONTROL_FMT, *ctrl)
    ctrl_sock.sendto(pkt, (MAVRIK_CONTROL_IP, MAVRIK_CONTROL_PORT))


def measure_rates(state_sock, ctrl_sock, ctrl, duration=0.5) -> tuple:
    """Send control for `duration` seconds and return mean (p, q, r) in deg/s."""
    t_end = time.time() + duration
    p_samples, q_samples, r_samples = [], [], []
    while time.time() < t_end:
        send_control(ctrl_sock, ctrl)
        state = recv_state(state_sock)
        if state:
            p_samples.append(math.degrees(state["p"]))
            q_samples.append(math.degrees(state["q"]))
            r_samples.append(math.degrees(state["r"]))
        time.sleep(0.005)
    if not p_samples:
        return (0.0, 0.0, 0.0)
    n = len(p_samples)
    return (sum(p_samples)/n, sum(q_samples)/n, sum(r_samples)/n)


def main():
    state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    state_sock.bind(("0.0.0.0", MAVRIK_STATE_PORT))
    state_sock.setblocking(False)

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("=" * 65)
    print("  MAVRIK Motor Characterization — Empirical Mixing Matrix")
    print("=" * 65)
    print()
    print("  Waiting for MAVRIK state packets...")
    print("  (Make sure MAVRIK is running and pid_vtol.py is NOT running)")
    print()

    # Wait for MAVRIK to connect
    deadline = time.time() + 30
    while time.time() < deadline:
        state = recv_state(state_sock)
        if state and math.isfinite(state["t"]) and state["t"] > 0:
            print(f"  MAVRIK connected (t={state['t']:.2f}s, alt={-state['zf']:.0f}ft)")
            break
        send_control(ctrl_sock, BASE_CTRL)
        time.sleep(0.1)
    else:
        print("  ERROR: MAVRIK not responding. Exiting.")
        return

    print()
    print("  Stabilizing at hover trim for 3s...")
    t_end = time.time() + 3.0
    while time.time() < t_end:
        send_control(ctrl_sock, BASE_CTRL)
        time.sleep(0.005)

    print()
    print("  Measuring baseline rates (no step input)...")
    p0, q0, r0 = measure_rates(state_sock, ctrl_sock, BASE_CTRL, 1.0)
    print(f"  Baseline: p={p0:+.3f}°/s  q={q0:+.3f}°/s  r={r0:+.3f}°/s")
    print()

    results = []
    print(f"  {'Motor':<10} {'Step':>6} | {'p (roll°/s)':>12} {'q (pitch°/s)':>13} {'r (yaw°/s)':>11} | Roll↑ Pitch↑")
    print("  " + "-" * 75)

    for i, name in enumerate(MOTOR_NAMES):
        for direction, delta in [("+", STEP_DELTA), ("-", -STEP_DELTA)]:
            ctrl = make_control(i, delta)
            actual_delta = ctrl[3 + i] - BASE_CTRL[3 + i]

            # Settle at base first
            settle_end = time.time() + 0.3
            while time.time() < settle_end:
                send_control(ctrl_sock, BASE_CTRL)
                time.sleep(0.005)

            # Apply step and measure
            p, q, r = measure_rates(state_sock, ctrl_sock, ctrl, STEP_HOLD)
            dp = p - p0
            dq = q - q0
            dr = r - r0

            roll_dir  = "LEFT" if dp < -0.5 else ("RIGHT" if dp > 0.5 else "~0")
            pitch_dir = "UP"   if dq > 0.5  else ("DOWN"  if dq < -0.5 else "~0")
            tag = f"{roll_dir}/{pitch_dir}"

            print(f"  {name:<10} {direction}{abs(actual_delta):.2f}  | "
                  f"{dp:+12.3f} {dq:+13.3f} {dr:+11.3f} | {tag}")

            results.append({
                "motor": name,
                "motor_idx": i,
                "direction": direction,
                "delta": actual_delta,
                "dp_dps": dp,
                "dq_dps": dq,
                "dr_dps": dr,
                "roll_dir": roll_dir,
                "pitch_dir": pitch_dir,
            })

    print()

    # Write CSV
    out_path = "motor_response_matrix.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)
    print(f"  Results written to: {out_path}")
    print()

    # Summary: derive mixing matrix
    print("  EMPIRICAL MIXING MATRIX (per unit throttle step):")
    print(f"  {'Motor':<12} {'dRoll/dThr':>12} {'dPitch/dThr':>13} {'dYaw/dThr':>11}")
    print("  " + "-" * 52)
    # Use positive step results for the matrix
    pos_results = [r for r in results if r["direction"] == "+"]
    for r in pos_results:
        roll_gain  = r["dp_dps"] / r["delta"] if r["delta"] != 0 else 0
        pitch_gain = r["dq_dps"] / r["delta"] if r["delta"] != 0 else 0
        yaw_gain   = r["dr_dps"] / r["delta"] if r["delta"] != 0 else 0
        print(f"  {r['motor']:<12} {roll_gain:+12.1f} {pitch_gain:+13.1f} {yaw_gain:+11.1f}")
    print()

    print("  DIAGONAL MIXING VALIDATION:")
    fr = next(r for r in pos_results if r["motor"] == "thrFR")
    fl = next(r for r in pos_results if r["motor"] == "thrFL")
    ar = next(r for r in pos_results if r["motor"] == "thrAR")
    al = next(r for r in pos_results if r["motor"] == "thrAL")
    print(f"  FR roll={fr['dp_dps']:+.1f}°/s, AL roll={al['dp_dps']:+.1f}°/s  → "
          f"{'SAME SIGN ✓ (left-roll diagonal)' if fr['dp_dps'] * al['dp_dps'] > 0 else 'DIFFERENT ✗'}")
    print(f"  FL roll={fl['dp_dps']:+.1f}°/s, AR roll={ar['dp_dps']:+.1f}°/s  → "
          f"{'SAME SIGN ✓ (right-roll diagonal)' if fl['dp_dps'] * ar['dp_dps'] > 0 else 'DIFFERENT ✗'}")
    print(f"  FR/AL vs FL/AR roll signs opposite: "
          f"{'✓' if fr['dp_dps'] * fl['dp_dps'] < 0 else '✗ (unexpected)'}")

    state_sock.close()
    ctrl_sock.close()


if __name__ == "__main__":
    main()
