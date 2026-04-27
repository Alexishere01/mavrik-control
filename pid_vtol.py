#!/usr/bin/env python3
"""
MAVRIK VTOL PID Controller v2
==============================
Cascaded PID controller for the demo_VTOL tilt-rotor MAVRIK simulation.

Key changes from v1:
  - ASYMMETRIC MIXER: fwd motors are tilted (~63°) and ahead of CG, aft are
    vertical and behind CG. Trims are computed separately, and pitch differential
    is scaled to keep total lift constant.
  - SEPARATE TRIMS: trim_fwd ~ 0.69, trim_aft ~ 0.39 derived from W, T0,
    moment arms, and tilt fraction. User can override in JSON.
  - U-DAMPING LOOP: tilted fwd motors give an unavoidable forward push at hover.
    We add a small longitudinal-velocity loop that nudges pitch to balance it.
  - OUTPUT RATE LIMIT: per-tick max throttle change to keep motor solver stable
    (matters when re-enabling battery rotor model later).
  - YAW DEMOTION: at low forward speed the rudder has no authority and our
    custom-thrust motors produce no torque. Yaw loop is disabled by default.
  - CONSERVATIVE STARTING GAINS computed from authority numbers, not guessed.

Architecture
------------
  outer attitude loops -> rate cmds -> rate loops -> mixer -> 4 throttles + rudder

Networking is identical to v1 (state in :5007, controls out :5006).
"""

import socket
import struct
import time
import math
import json
import os
import sys
import signal


# ============================================================================
# UTILITIES
# ============================================================================

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle(a):
    """Wrap radians to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def quat_to_euler(e0, ex, ey, ez):
    """3-2-1 (roll, pitch, yaw) from quaternion."""
    phi = math.atan2(2 * (e0 * ex + ey * ez), 1 - 2 * (ex * ex + ey * ey))
    sinp = 2 * (e0 * ey - ez * ex)
    theta = math.asin(clamp(sinp, -1, 1))
    psi = math.atan2(2 * (e0 * ez + ex * ey), 1 - 2 * (ey * ey + ez * ez))
    return phi, theta, psi


# ============================================================================
# PID CLASS
# ============================================================================

class PID:
    """Plain PID with integrator clamp, output clamp, and derivative on error."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 out_min=-1e9, out_max=1e9,
                 int_min=-1e9, int_max=1e9, name=""):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.int_min, self.int_max = int_min, int_max
        self.name = name
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, t):
        if self.prev_time is None:
            self.prev_time = t
            self.prev_error = error
            return clamp(self.kp * error, self.out_min, self.out_max)

        dt = t - self.prev_time
        if dt <= 0:
            return clamp(self.kp * error, self.out_min, self.out_max)

        p = self.kp * error
        # Anti-windup: only integrate when output isn't already saturated in same direction
        new_integ = self.integrator + error * dt
        new_integ = clamp(new_integ, self.int_min, self.int_max)
        i = self.ki * new_integ
        d = self.kd * (error - self.prev_error) / dt

        out_unsat = p + i + d
        out = clamp(out_unsat, self.out_min, self.out_max)

        # If we saturated AND integrator would push further into saturation, freeze it
        if out != out_unsat and ((out_unsat > self.out_max and error > 0) or
                                 (out_unsat < self.out_min and error < 0)):
            pass  # keep old integrator
        else:
            self.integrator = new_integ

        self.prev_error = error
        self.prev_time = t
        return out


# ============================================================================
# DEFAULT CONFIG
# ============================================================================

DEFAULT_CONFIG = {
    "description": "PID v2 — asymmetric mixer for tilted fwd / vertical aft motors",
    "loop_rate_hz": 50,
    "mode": "hover",
    "flaps_value": 0.99,

    "trim": {
        "description": (
            "Computed for W=160 lbf, T0=80 lbf/motor, fwd tilt 63°, "
            "CG x=-1.10. Override here if MAVRIK reports different mass props."
        ),
        "throttle_fwd": 0.687,
        "throttle_aft": 0.388,
        "aileron_deg": 0.0,
        "elevator_deg": 0.0,
        "rudder_deg": 0.0,
        "lift_balance_factor": 0.892,
        "_note_lift_balance": (
            "= sin(fwd_tilt). When pitch loop adds delta to fwd, "
            "we subtract delta*lift_balance_factor from aft to keep total lift constant."
        )
    },

    "safety": {
        "description": "Per-tick output limits to keep motors in safe regime",
        "max_throttle_step": 0.05,
        "max_surface_step_deg": 5.0,
        "ramp_in_seconds": 2.0,
        "warmup_seconds": 5.0,
        "_note_ramp": (
            "For first ramp_in_seconds, blend controller output with pure trim. "
            "Prevents a startup-transient kick from saturating motors."
        ),
        "_note_warmup": (
            "For first warmup_seconds, hold the vehicle's INITIAL state as the "
            "setpoint (instead of the schedule). Lets all loops start at zero "
            "error rather than fighting a 20+ fps / 3° instant disturbance."
        )
    },

    "altitude": {
        "kp": 0.3,  "ki": 0.05, "kd": 0.1,
        "output_min": -5.0, "output_max": 5.0, "integrator_max": 10.0,
        "_note": "alt error (ft) -> climb rate cmd (ft/s)"
    },
    "climb_rate": {
        "kp": 0.02, "ki": 0.005, "kd": 0.0,
        "output_min": -0.20, "output_max": 0.20, "integrator_max": 5.0,
        "_note": "climb rate error (ft/s) -> collective throttle delta"
    },

    "roll_attitude": {
        "kp": 2.0, "ki": 0.0, "kd": 0.2,
        "output_min": -30.0, "output_max": 30.0,
        "_note": "roll error (deg) -> roll rate cmd (deg/s)"
    },
    "roll_rate": {
        "kp": 0.002, "ki": 0.0005, "kd": 0.0001,
        "output_min": -0.10, "output_max": 0.10, "integrator_max": 3.0,
        "_note": "roll rate error (deg/s) -> differential throttle"
    },

    "pitch_attitude": {
        "kp": 2.0, "ki": 0.0, "kd": 0.2,
        "output_min": -30.0, "output_max": 30.0,
        "_note": "pitch error (deg) -> pitch rate cmd (deg/s)"
    },
    "pitch_rate": {
        "kp": 0.002, "ki": 0.0005, "kd": 0.0001,
        "output_min": -0.10, "output_max": 0.10, "integrator_max": 3.0,
        "_note": "pitch rate error (deg/s) -> differential throttle"
    },

    "u_velocity": {
        "enabled": True,
        "kp": 0.05, "ki": 0.005, "kd": 0.0,
        "output_min": -2.0, "output_max": 2.0, "integrator_max": 5.0,
        "_note": (
            "Forward-velocity damping: tilted fwd motors push forward unavoidably. "
            "This loop nudges PITCH SETPOINT (positive theta = nose up = less fwd push). "
            "Output limited to ±2° pitch nudge so it doesn't fight the pitch loop. "
            "Detuned vs initial guess — initial values caused 5° pitch overshoot."
        )
    },

    "yaw_attitude": {
        "enabled": False,
        "kp": 0.3, "ki": 0.0, "kd": 0.05,
        "output_min": -10.0, "output_max": 10.0,
        "_note": "Yaw control is weak in hover (no rudder authority, no rotor torque)."
    },
    "yaw_rate": {
        "kp": 0.5, "ki": 0.02, "kd": 0.01,
        "output_min": -10.0, "output_max": 10.0, "integrator_max": 5.0
    },

    "setpoints": {
        "description": (
            "Schedule starts when warmup blend completes. The first entry should be "
            "the desired hover state (theta=0, alt=1000, u=0). Disturbances come later."
        ),
        "schedule": [
            {"time":  0.0, "phi_deg": 0.0, "theta_deg": 0.0, "alt_ft": 1000.0, "u_ft_s": 0.0},
            {"time": 30.0, "phi_deg": 0.0, "theta_deg": 0.0, "alt_ft": 1000.0, "u_ft_s": 0.0},
            {"time": 40.0, "phi_deg": 5.0, "theta_deg": 0.0, "alt_ft": 1000.0, "u_ft_s": 0.0},
            {"time": 55.0, "phi_deg": 0.0, "theta_deg": 0.0, "alt_ft": 1000.0, "u_ft_s": 0.0},
            {"time": 70.0, "phi_deg": 0.0, "theta_deg": 0.0, "alt_ft": 1010.0, "u_ft_s": 0.0},
            {"time": 90.0, "phi_deg": 0.0, "theta_deg": 0.0, "alt_ft": 1000.0, "u_ft_s": 0.0}
        ]
    },

    "ports": {
        "state_receive": 5007,
        "control_send": 5006,
        "control_ip": "127.0.0.1"
    },

    "logging": {
        "filename": "pid_log.csv",
        "rate_hz": 30
    }
}


# ============================================================================
# CONTROLLER
# ============================================================================

class VTOLController:
    def __init__(self, config_file="pid_vtol_gains.json"):
        if os.path.exists(config_file):
            print(f"  Loading config from {config_file}")
            with open(config_file, 'r') as f:
                self.cfg = json.load(f)
        else:
            print(f"  Creating default config: {config_file}")
            self.cfg = DEFAULT_CONFIG
            with open(config_file, 'w') as f:
                json.dump(DEFAULT_CONFIG, f, indent=4)

        c = self.cfg

        def make_pid(section, name):
            s = c[section]
            return PID(
                s["kp"], s["ki"], s["kd"],
                s.get("output_min", -1e9), s.get("output_max", 1e9),
                -s.get("integrator_max", 1e9), s.get("integrator_max", 1e9),
                name=name
            )

        # Outer + inner cascades
        self.pid_alt        = make_pid("altitude",       "alt")
        self.pid_climb      = make_pid("climb_rate",     "climb")
        self.pid_roll_att   = make_pid("roll_attitude",  "roll_att")
        self.pid_roll_rate  = make_pid("roll_rate",      "roll_rate")
        self.pid_pitch_att  = make_pid("pitch_attitude", "pitch_att")
        self.pid_pitch_rate = make_pid("pitch_rate",     "pitch_rate")
        self.pid_yaw_att    = make_pid("yaw_attitude",   "yaw_att")
        self.pid_yaw_rate   = make_pid("yaw_rate",       "yaw_rate")
        self.pid_u          = make_pid("u_velocity",     "u_vel")

        self.u_loop_enabled   = c["u_velocity"].get("enabled", True)
        self.yaw_loop_enabled = c["yaw_attitude"].get("enabled", False)

        # Trim
        t = c["trim"]
        self.trim_fwd = t["throttle_fwd"]
        self.trim_aft = t["throttle_aft"]
        self.lift_balance = t.get("lift_balance_factor", 0.892)
        self.trim_ail = t["aileron_deg"]
        self.trim_ele = t["elevator_deg"]
        self.trim_rud = t["rudder_deg"]
        self.base_flaps = c["flaps_value"]

        # Safety
        s = c["safety"]
        self.max_thr_step = s["max_throttle_step"]
        self.max_surf_step = s["max_surface_step_deg"]
        self.ramp_seconds = s["ramp_in_seconds"]
        self.warmup_seconds = s.get("warmup_seconds", 0.0)
        self.last_thr = [self.trim_fwd, self.trim_fwd, self.trim_aft, self.trim_aft]
        self.last_rud = self.trim_rud

        # Initial-state holders (populated on first state packet)
        self.initial_phi   = None
        self.initial_theta = None
        self.initial_alt   = None
        self.initial_u     = None

        # Setpoints
        self.schedule = c["setpoints"]["schedule"]
        self.heading_hold = None

        # Networking
        ports = c["ports"]
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.bind(("0.0.0.0", ports["state_receive"]))
        self.state_sock.setblocking(False)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_dest = (ports["control_ip"], ports["control_send"])

        # Logging
        lc = c["logging"]
        self.log = open(lc["filename"], 'w')
        self.log.write(
            "t,"
            "phi_cmd,phi,theta_cmd,theta,psi_cmd,psi,"
            "alt_cmd,alt,climb_cmd,climb,u_cmd,u,"
            "p_cmd,p,q_cmd,q,r_cmd,r,"
            "collective_delta,roll_diff,pitch_diff_fwd,pitch_diff_aft,rudder,"
            "thrFR,thrFL,thrAR,thrAL,"
            "aileron,elevator,"
            "flapsR,flapsL,yaw_diff_flaps,ramp\n"
        )
        self.log_dt = 1.0 / lc["rate_hz"]
        self.last_log = 0.0
        self.loop_hz = c["loop_rate_hz"]
        self.t_start = None
        self.running = True

    def get_setpoint(self, t):
        active = self.schedule[0]
        for sp in self.schedule:
            if t >= sp["time"]:
                active = sp
        return active

    def read_state(self):
        data = None
        try:
            while True:
                data, _ = self.state_sock.recvfrom(4096)
        except BlockingIOError:
            pass
        if data and len(data) >= 14 * 4:
            n = len(data) // 4
            return struct.unpack('<' + 'f' * n, data)
        return None

    def send_controls(self, ail, ele, rud, thrFR, thrFL, thrAR, thrAL, flapsR, flapsL):
        pkt = struct.pack('<9f', ail, ele, rud, thrFR, thrFL, thrAR, thrAL, flapsR, flapsL)
        self.ctrl_sock.sendto(pkt, self.ctrl_dest)

    def rate_limit(self, new_val, last_val, max_step):
        delta = clamp(new_val - last_val, -max_step, max_step)
        return last_val + delta

    def run(self):
        c = self.cfg
        print()
        print("=" * 72)
        print("  MAVRIK VTOL PID Controller v2  (asymmetric mixer)")
        print("=" * 72)
        print(f"  roll_rate kp={self.cfg['roll_rate']['kp']}  roll_att output_max={self.cfg['roll_attitude']['output_max']}")
        print(f"  Mode:           {c['mode']}  (base_flaps={self.base_flaps})")
        print(f"  Trim throttle:  fwd={self.trim_fwd:.3f}  aft={self.trim_aft:.3f}")
        print(f"  Lift balance:   {self.lift_balance:.3f}  (sin of fwd tilt angle)")
        print(f"  Loop rate:      {self.loop_hz} Hz")
        print(f"  Rate limits:    {self.max_thr_step}/tick throttle, "
              f"{self.max_surf_step}°/tick surface")
        print(f"  Ramp-in:        {self.ramp_seconds} s (output blend trim->controller)")
        print(f"  Warmup:         {self.warmup_seconds} s (setpoint blend initial->schedule)")
        print(f"  u-vel loop:     {'on' if self.u_loop_enabled else 'off'}")
        print(f"  Yaw loop:       {'on (diff tilt)' if self.yaw_loop_enabled else 'off'}")
        print(f"  Log file:       {c['logging']['filename']}")
        print()
        print("  Setpoint schedule:")
        for sp in self.schedule:
            print(f"    t={sp['time']:5.1f}s  phi={sp['phi_deg']:+5.1f}°  "
                  f"theta={sp['theta_deg']:+5.1f}°  alt={sp['alt_ft']:.0f} ft  "
                  f"u={sp.get('u_ft_s',0):+.1f} fps")
        print("=" * 72)
        print("  Waiting for MAVRIK state...\n")

        dt_loop = 1.0 / self.loop_hz
        last_print = 0.0

        while self.running:
            t0 = time.time()

            state = self.read_state()
            if state is None:
                time.sleep(0.005)
                continue

            t = state[0]
            u, v, w = state[1], state[2], state[3]
            p, q, r = state[4], state[5], state[6]
            xf, yf, zf = state[7], state[8], state[9]
            e0, ex, ey, ez = state[10], state[11], state[12], state[13]

            phi, theta, psi = quat_to_euler(e0, ex, ey, ez)
            alt = -zf
            climb = -w
            phi_d, theta_d, psi_d = phi * RAD2DEG, theta * RAD2DEG, psi * RAD2DEG
            p_d, q_d, r_d = p * RAD2DEG, q * RAD2DEG, r * RAD2DEG

            if self.t_start is None:
                self.t_start = t
                self.heading_hold = psi_d
                self.initial_phi   = phi_d
                self.initial_theta = theta_d
                self.initial_alt   = alt
                self.initial_u     = u
                print(f"  Initial: phi={phi_d:.1f}° theta={theta_d:.1f}° "
                      f"psi={psi_d:.1f}° alt={alt:.0f} ft  u={u:.1f} fps")
                print(f"  Holding initial state for {self.warmup_seconds:.1f}s, "
                      f"then blending into schedule.")

            # Ramp-in factor: 0 at start, 1 after ramp_seconds
            t_since_start = t - self.t_start
            ramp = clamp(t_since_start / max(self.ramp_seconds, 1e-6), 0.0, 1.0)

            sp = self.get_setpoint(t)
            phi_cmd_sched   = sp["phi_deg"]
            theta_cmd_sched = sp["theta_deg"]
            alt_cmd_sched   = sp["alt_ft"]
            u_cmd_sched     = sp.get("u_ft_s", 0.0)
            # When yaw loop is enabled, read heading from schedule; otherwise hold initial
            if self.yaw_loop_enabled and "psi_deg" in sp:
                psi_cmd = sp["psi_deg"] + self.heading_hold  # offset from initial heading
            else:
                psi_cmd = self.heading_hold

            # Warmup blend: hold initial state, then ease into the scheduled setpoint.
            # blend = 0 means use initial state, blend = 1 means use schedule.
            if self.warmup_seconds > 0:
                raw = (t_since_start) / self.warmup_seconds
                # Cosine ease for smooth start AND smooth end of blend
                blend = 0.5 * (1 - math.cos(math.pi * clamp(raw, 0, 1)))
            else:
                blend = 1.0

            phi_cmd_base   = (1-blend) * self.initial_phi   + blend * phi_cmd_sched
            theta_cmd_base = (1-blend) * self.initial_theta + blend * theta_cmd_sched
            alt_cmd        = (1-blend) * self.initial_alt   + blend * alt_cmd_sched
            u_cmd          = (1-blend) * self.initial_u     + blend * u_cmd_sched

            # === U-VELOCITY LOOP ===
            # Tilted fwd motors push the vehicle forward. To hold u_cmd, we nudge
            # the pitch setpoint: nose UP (positive theta) tilts fwd-thrust component
            # backward and reduces forward push.
            if self.u_loop_enabled:
                u_err = u_cmd - u   # negative when going too fast forward
                theta_nudge = self.pid_u.update(u_err, t)
                # u too high (moving forward fast) -> err < 0 -> theta_nudge < 0 -> nose DOWN
                # That's wrong; we want nose UP to slow down. So flip sign.
                theta_cmd = theta_cmd_base - theta_nudge
            else:
                theta_cmd = theta_cmd_base

            # === ALTITUDE CASCADE ===
            climb_cmd = self.pid_alt.update(alt_cmd - alt, t)
            collective_delta = self.pid_climb.update(climb_cmd - climb, t)

            # === ROLL CASCADE ===
            phi_err = wrap_angle((phi_cmd_base - phi_d) * DEG2RAD) * RAD2DEG
            p_cmd = self.pid_roll_att.update(phi_err, t)
            roll_diff = self.pid_roll_rate.update(p_cmd - p_d, t)

            # === PITCH CASCADE ===
            theta_err = wrap_angle((theta_cmd - theta_d) * DEG2RAD) * RAD2DEG
            q_cmd = self.pid_pitch_att.update(theta_err, t)
            pitch_diff = self.pid_pitch_rate.update(q_cmd - q_d, t)

            # === YAW CASCADE — differential tilt for yaw authority in hover ===
            # Positive yaw_diff_flaps = yaw RIGHT:
            #   flapsRight increases (right arm more vertical, less fwd push on right)
            #   flapsLeft decreases (left arm tilts forward, more fwd push on left)
            #   Net: left side pushes forward → vehicle yaws right
            if self.yaw_loop_enabled:
                psi_err = wrap_angle((psi_cmd - psi_d) * DEG2RAD) * RAD2DEG
                r_cmd_d = self.pid_yaw_att.update(psi_err, t)
                yaw_diff_flaps = self.pid_yaw_rate.update(r_cmd_d - r_d, t)
            else:
                r_cmd_d = 0.0
                yaw_diff_flaps = 0.0

            # === ASYMMETRIC MIXER ===
            #
            # collective: moves all motors together (up/down)
            #   thrFR/L += collective_delta
            #   thrAR/L += collective_delta
            #
            # pitch_diff > 0: nose UP. Achieved by ADDING to fwd, SUBTRACTING from aft.
            #   To keep total lift unchanged, the aft adjustment is scaled by lift_balance:
            #     fwd_extra_lift = +pitch_diff * fwd_T0 * fwd_vert_frac
            #     aft_lift_change_needed = -fwd_extra_lift  (so total stays the same)
            #     aft_throttle_change = aft_lift_change_needed / (aft_T0 * 1.0)
            #                         = -pitch_diff * fwd_vert_frac
            #     => aft delta = -pitch_diff * lift_balance
            #
            # roll_diff > 0: roll RIGHT. Add to LEFT motors, subtract from RIGHT.
            pitch_fwd =  pitch_diff
            pitch_aft = -pitch_diff * self.lift_balance

            base_fwd = self.trim_fwd + collective_delta
            base_aft = self.trim_aft + collective_delta

            thrFR_raw = base_fwd + pitch_fwd - roll_diff
            thrFL_raw = base_fwd + pitch_fwd + roll_diff
            thrAR_raw = base_aft + pitch_aft - roll_diff
            thrAL_raw = base_aft + pitch_aft + roll_diff

            # === RAMP-IN: blend toward pure trim during the first few seconds ===
            thrFR_ramped = ramp * thrFR_raw + (1 - ramp) * self.trim_fwd
            thrFL_ramped = ramp * thrFL_raw + (1 - ramp) * self.trim_fwd
            thrAR_ramped = ramp * thrAR_raw + (1 - ramp) * self.trim_aft
            thrAL_ramped = ramp * thrAL_raw + (1 - ramp) * self.trim_aft

            # === CLAMP + RATE-LIMIT ===
            thrFR_cmd = self.rate_limit(clamp(thrFR_ramped, 0, 1), self.last_thr[0], self.max_thr_step)
            thrFL_cmd = self.rate_limit(clamp(thrFL_ramped, 0, 1), self.last_thr[1], self.max_thr_step)
            thrAR_cmd = self.rate_limit(clamp(thrAR_ramped, 0, 1), self.last_thr[2], self.max_thr_step)
            thrAL_cmd = self.rate_limit(clamp(thrAL_ramped, 0, 1), self.last_thr[3], self.max_thr_step)
            self.last_thr = [thrFR_cmd, thrFL_cmd, thrAR_cmd, thrAL_cmd]

            rudder_raw = clamp(self.trim_rud, -15, 15)  # rudder at trim (no authority in hover)
            rudder_cmd = self.rate_limit(rudder_raw, self.last_rud, self.max_surf_step)
            self.last_rud = rudder_cmd

            aileron  = clamp(self.trim_ail, -15, 15)
            elevator = clamp(self.trim_ele, -15, 15)

            # === DIFFERENTIAL FLAPS FOR YAW ===
            flapsR_cmd = clamp(self.base_flaps + ramp * yaw_diff_flaps, 0, 1)
            flapsL_cmd = clamp(self.base_flaps - ramp * yaw_diff_flaps, 0, 1)

            # === SEND ===
            self.send_controls(aileron, elevator, rudder_cmd,
                               thrFR_cmd, thrFL_cmd, thrAR_cmd, thrAL_cmd,
                               flapsR_cmd, flapsL_cmd)

            # === LOG ===
            if t - self.last_log >= self.log_dt:
                self.log.write(
                    f"{t:.4f},"
                    f"{phi_cmd_base:.3f},{phi_d:.3f},{theta_cmd:.3f},{theta_d:.3f},"
                    f"{psi_cmd:.3f},{psi_d:.3f},"
                    f"{alt_cmd:.2f},{alt:.2f},{climb_cmd:.3f},{climb:.3f},"
                    f"{u_cmd:.3f},{u:.3f},"
                    f"{p_cmd:.3f},{p_d:.3f},{q_cmd:.3f},{q_d:.3f},"
                    f"{r_cmd_d:.3f},{r_d:.3f},"
                    f"{collective_delta:.4f},{roll_diff:.4f},"
                    f"{pitch_fwd:.4f},{pitch_aft:.4f},{rudder_cmd:.3f},"
                    f"{thrFR_cmd:.4f},{thrFL_cmd:.4f},{thrAR_cmd:.4f},{thrAL_cmd:.4f},"
                    f"{aileron:.3f},{elevator:.3f},"
                    f"{flapsR_cmd:.4f},{flapsL_cmd:.4f},{yaw_diff_flaps:.5f},{ramp:.3f}\n"
                )
                self.last_log = t

            # === CONSOLE ===
            if t - last_print >= 0.5:
                if ramp < 1:
                    status = "RMP"
                elif blend < 1:
                    status = f"W{blend:.1f}"
                else:
                    status = "   "
                print(f"  t={t:6.1f}s {status} | "
                      f"phi={phi_d:+5.1f}/{phi_cmd_base:+5.1f} | "
                      f"theta={theta_d:+5.1f}/{theta_cmd:+5.1f} | "
                      f"alt={alt:6.1f}/{alt_cmd:.0f} | "
                      f"psi={psi_d:+6.1f}/{psi_cmd:+.0f} | "
                      f"u={u:+5.1f} | "
                      f"thr=[{thrFR_cmd:.2f} {thrFL_cmd:.2f} {thrAR_cmd:.2f} {thrAL_cmd:.2f}] "
                      f"ydf={yaw_diff_flaps:+.4f}")
                last_print = t

            elapsed = time.time() - t0
            sleep = dt_loop - elapsed
            if sleep > 0:
                time.sleep(sleep)

        self.log.close()
        print("\n  Controller stopped. Log saved.")

    def stop(self):
        self.running = False


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else "pid_vtol_gains.json"
    ctrl = VTOLController(cfg)
    signal.signal(signal.SIGINT, lambda *_: ctrl.stop())
    ctrl.run()


if __name__ == "__main__":
    main()