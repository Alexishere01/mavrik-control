#!/usr/bin/env python3
"""
mavrik_ardupilot_bridge.py
==========================
Physics backend bridge: MAVRIK <-> ArduPilot SITL via the JSON interface.

ArduPilot SITL launches with its JSON backend pointed at this bridge:
    sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1

ArduPilot sends a binary PWM packet to UDP :9002. The bridge maps the PWM
channels to MAVRIK's 9 control effectors, forwards them to MAVRIK on :5006,
pulls the latest MAVRIK state from :5009, and replies to ArduPilot with a
JSON string containing timestamp, IMU, position, velocity, and attitude.

QGroundControl connects directly to ArduPilot on :14550 (unchanged from a
stock SITL setup). The MAVRIK-to-QGC telemetry bridge is not needed here.

Protocol reference:
    https://github.com/ardupilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md

MVP scope:
    - Frame: ArduCopter quad. Only PWM channels 1-4 are consumed.
    - Tilt: held at flaps=1.0 (motors exactly vertical, zero horizontal drift).
              Arms can go past 90deg; clamp is 1.1 for yaw authority.
    - Servos: aileron/elevator/rudder held at 0.
    - Accel: finite-differenced body velocity minus gravity-in-body.
"""

import argparse
import csv
import json as json_lib
import math
import os
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FT_TO_M = 0.3048
GRAVITY_MS2 = 9.80665
PWM_MIN = 1000
PWM_MAX = 2000
PWM_MID = 1500

# VTOL flap position in MAVRIK convention (0.99 = motors pointing up; 0.0 = forward/FW).
VTOL_FLAP = 0.99

# Zero-thrust freefall — used only before MAVRIK connects (no attitude data available).
PRE_ARM_FREEFALL = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VTOL_FLAP, VTOL_FLAP)

# Hover trim from Team1_VTOL.json — sent once MAVRIK is connected but before arming.
# This matches the physics-engine equilibrium exactly, so MAVRIK holds ~0° pitch/roll
# and we arm with near-level attitude instead of the ~30° freefall drift.
PRE_ARM_HOVER = (0.0, 0.0, 0.0, 0.122, 0.122, 0.127, 0.127, VTOL_FLAP, VTOL_FLAP)

PRE_ARM_HOVER_LEVEL = 0.125   # kept for ground_hold_controls() reference only
PRE_ARM_KP          = 0.6
PRE_ARM_KD          = 0.05


class PreArmController:
    """Cascaded PID hover for pre-arm stabilisation — mirrors pid_vtol.py.

    Attitude loop (deg → deg/s) feeds a rate loop (deg/s → throttle diff).
    Gains are from pid_vtol DEFAULT_CONFIG, designed for 50 Hz.
    Rate-limiting and a 2 s ramp-in prevent the startup kick that killed
    the direct-proportional ground_hold_controls() approach.
    """
    # Asymmetric trim matching Team1_VTOL.json initial conditions
    TRIM_FWD     = PRE_ARM_HOVER[3]   # 0.394 — thrFR, thrFL
    TRIM_AFT     = PRE_ARM_HOVER[5]   # 0.410 — thrAR, thrAL
    HZ           = 50
    RAMP_SECS    = 2.0
    MAX_THR_STEP = 0.05               # per 50 Hz tick

    # Gains from pid_vtol DEFAULT_CONFIG
    KP_ROLL_ATT = 2.0;  KD_ROLL_ATT = 0.2
    KP_ROLL_RAT = 0.002
    KP_PIT_ATT  = 3.0;  KD_PIT_ATT  = 0.2
    KP_PIT_RAT  = 0.003

    def __init__(self):
        self._last_thr  = [self.TRIM_FWD, self.TRIM_FWD, self.TRIM_AFT, self.TRIM_AFT]
        self._last_out  = PRE_ARM_HOVER
        self._t_start   = None
        self._next_tick = 0.0

    def reset(self):
        self._last_thr  = [self.TRIM_FWD, self.TRIM_FWD, self.TRIM_AFT, self.TRIM_AFT]
        self._last_out  = PRE_ARM_HOVER
        self._t_start   = None
        self._next_tick = 0.0

    def _rl(self, new: float, last: float) -> float:
        return last + max(-self.MAX_THR_STEP, min(self.MAX_THR_STEP, new - last))

    def update(self, pitch_rad: float, roll_rad: float,
               q_rads: float, p_rads: float) -> Tuple[float, ...]:
        now = time.time()
        if now < self._next_tick:
            return self._last_out
        self._next_tick = now + 1.0 / self.HZ
        if self._t_start is None:
            self._t_start = now
        ramp = min(1.0, (now - self._t_start) / self.RAMP_SECS)

        # Errors in degrees — setpoint is 0° pitch, 0° roll
        pitch_err = -math.degrees(pitch_rad)   # nose-down (neg rad) → positive error
        roll_err  = -math.degrees(roll_rad)    # right tilt (pos rad) → negative error
        q_d = math.degrees(q_rads)
        p_d = math.degrees(p_rads)

        # Attitude → rate command
        q_cmd = self.KP_PIT_ATT * pitch_err - self.KD_PIT_ATT * q_d
        p_cmd = self.KP_ROLL_ATT * roll_err  - self.KD_ROLL_ATT * p_d

        # Rate → throttle differential
        pitch_diff = self.KP_PIT_RAT * q_cmd
        roll_diff  = self.KP_ROLL_RAT * p_cmd

        # Motor mixing: pitch_diff>0 adds front/removes rear; roll_diff>0 adds right/removes left
        fr = max(0.0, min(1.0, self.TRIM_FWD + ramp * ( pitch_diff - roll_diff)))
        fl = max(0.0, min(1.0, self.TRIM_FWD + ramp * ( pitch_diff + roll_diff)))
        ar = max(0.0, min(1.0, self.TRIM_AFT + ramp * (-pitch_diff - roll_diff)))
        al = max(0.0, min(1.0, self.TRIM_AFT + ramp * (-pitch_diff + roll_diff)))

        fr = self._rl(fr, self._last_thr[0])
        fl = self._rl(fl, self._last_thr[1])
        ar = self._rl(ar, self._last_thr[2])
        al = self._rl(al, self._last_thr[3])
        self._last_thr = [fr, fl, ar, al]

        self._last_out = (0.0, 0.0, 0.0, fr, fl, ar, al, VTOL_FLAP, VTOL_FLAP)
        return self._last_out

# ArduPilot PWM frame formats
ARDUPILOT_MAGIC_16CH = 18458
ARDUPILOT_MAGIC_32CH = 29569
ARDUPILOT_FRAME_16CH_FMT = "<HHI16H"      # 40 bytes
ARDUPILOT_FRAME_16CH_SIZE = struct.calcsize(ARDUPILOT_FRAME_16CH_FMT)
ARDUPILOT_FRAME_32CH_FMT = "<HHI32H"      # 72 bytes
ARDUPILOT_FRAME_32CH_SIZE = struct.calcsize(ARDUPILOT_FRAME_32CH_FMT)

# MAVRIK state packet: 14 little-endian floats
MAVRIK_STATE_FMT = "<14f"
MAVRIK_STATE_SIZE = struct.calcsize(MAVRIK_STATE_FMT)

# MAVRIK controls packet: 9 little-endian floats
# Order defined by Team1_VTOL.json effector block:
#   aileron, elevator, rudder, thrFR, thrFL, thrAR, thrAL, flapsRight, flapsLeft
MAVRIK_CONTROL_FMT = "<9f"


# ---------------------------------------------------------------------------
# State containers
# ---------------------------------------------------------------------------

@dataclass
class MavrikState:
    t: float                       # sim time (s)
    u: float; v: float; w: float   # body velocity (ft/s)
    p: float; q: float; r: float   # body angular rate (rad/s)
    xf: float; yf: float; zf: float  # earth NED position (ft)
    e0: float; ex: float; ey: float; ez: float  # quaternion, scalar-first

    @classmethod
    def unpack(cls, data: bytes) -> "MavrikState":
        return cls(*struct.unpack(MAVRIK_STATE_FMT, data[:MAVRIK_STATE_SIZE]))


# Sent to ArduPilot before MAVRIK connects so the EKF has something to converge
# on immediately (fixes the QGC "waiting" delay).  Level attitude, at spawn altitude.
SYNTHETIC_INITIAL_STATE = MavrikState(
    t=0.0, u=0.0, v=0.0, w=0.0, p=0.0, q=0.0, r=0.0,
    xf=0.0, yf=0.0, zf=-3000.0,  # NED: spawn altitude in ft (matches input_Team1_hover.json)
    e0=1.0, ex=0.0, ey=0.0, ez=0.0,
)


def make_level_proxy(s: MavrikState) -> MavrikState:
    """Level proxy sent to ArduPilot during pre-arm.

    Zeros all body velocities (u, v, w) so body_to_earth always gives
    vn=ve=vd=0 — consistent with a stationary hover in earth frame.
    The old bug was passing real w (+1 fps ascending) with a faked level
    quaternion: body_to_earth then gave vd = +0.3 m/s (descending) while
    position was actually rising, causing EKF to see contradicting signals.

    Passes through real zf (actual MAVRIK altitude) so the EKF tracks
    live altitude during pre-arm. This means NO position jump when handover
    fires and the bridge switches to real state — a sudden zf step of even
    1-2m looks like a 200+ m/s velocity spike to the EKF and crashes SITL.
    """
    import copy as _copy
    lvl = _copy.copy(s)
    lvl.u = 0.0; lvl.v = 0.0; lvl.w = 0.0   # zero body velocities → vn=ve=vd=0
    lvl.p = 0.0; lvl.q = 0.0; lvl.r = 0.0   # zero rates → accel = gravity only
    lvl.e0 = 1.0; lvl.ex = 0.0; lvl.ey = 0.0; lvl.ez = 0.0  # force level quaternion
    # zf passes through — EKF tracks real altitude, no step at handover
    return lvl


def _mavrik_state_valid(s: MavrikState) -> bool:
    """Reject garbage packets from MAVRIK startup/crash before they reach ArduPilot."""
    vals = (s.t, s.u, s.v, s.w, s.p, s.q, s.r,
            s.xf, s.yf, s.zf, s.e0, s.ex, s.ey, s.ez)
    if not all(math.isfinite(v) for v in vals):
        return False
    # Quaternion magnitude must be close to 1
    if abs(s.e0**2 + s.ex**2 + s.ey**2 + s.ez**2 - 1.0) > 0.5:
        return False
    # Time must be non-negative
    if s.t < 0:
        return False
    # Altitude: NED zf negative = above ground (ft).  Allow -50000ft to +1000ft.
    if s.zf > 1000.0 or s.zf < -50000.0:
        return False
    return True


@dataclass
class PWMFrame:
    frame_rate: int
    frame_count: int
    pwm: List[int] = field(default_factory=list)   # microseconds


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def quat_to_euler(e0, ex, ey, ez) -> Tuple[float, float, float]:
    sinp = max(-1.0, min(1.0, 2.0 * (e0 * ey - ez * ex)))
    return (math.atan2(2.0 * (e0 * ex + ey * ez), 1.0 - 2.0 * (ex * ex + ey * ey)),
            math.asin(sinp),
            math.atan2(2.0 * (e0 * ez + ex * ey), 1.0 - 2.0 * (ey * ey + ez * ez)))


def body_to_earth(v, q) -> Tuple[float, float, float]:
    """Matches view.py body_2_fixed. NED earth frame."""
    x, y, z = v
    e0, ex, ey, ez = q
    T0 = x * ex + y * ey + z * ez
    T1 = x * e0 - y * ez + z * ey
    T2 = x * ez + y * e0 - z * ex
    T3 = y * ex - x * ey + z * e0
    return (e0 * T1 + ex * T0 + ey * T3 - ez * T2,
            e0 * T2 - ex * T3 + ey * T0 + ez * T1,
            e0 * T3 + ex * T2 - ey * T1 + ez * T0)


def earth_to_body(v, q) -> Tuple[float, float, float]:
    """Inverse rotation: earth NED vector -> body frame. Matches view.py fixed_2_body."""
    x, y, z = v
    e0, ex, ey, ez = q
    T0 = -x * ex - y * ey - z * ez
    T1 =  x * e0 + y * ez - z * ey
    T2 =  y * e0 - x * ez + z * ex
    T3 =  x * ey - y * ex + z * e0
    return (e0 * T1 - ex * T0 - ey * T3 + ez * T2,
            e0 * T2 + ex * T3 - ey * T0 - ez * T1,
            e0 * T3 - ex * T2 + ey * T1 + ez * T0)


# ---------------------------------------------------------------------------
# PWM <-> MAVRIK control mapping
# ---------------------------------------------------------------------------

def pwm_to_normalized(pwm_us: int, lo: int = PWM_MIN, hi: int = PWM_MAX) -> float:
    """1000..2000 us -> 0.0..1.0"""
    return max(0.0, min(1.0, (pwm_us - lo) / (hi - lo)))


def pwm_to_symmetric(pwm_us: int) -> float:
    """1000..2000 us -> -1.0..+1.0 centered at 1500"""
    return max(-1.0, min(1.0, (pwm_us - PWM_MID) / (PWM_MAX - PWM_MID)))


class PWMMapper:
    """ArduCopter quad -> MAVRIK 9 effectors.

    ArduCopter default motor order (frame X):
        CH1 = motor 1 (front-right, CW)
        CH2 = motor 2 (rear-left, CW)
        CH3 = motor 3 (front-left, CCW)
        CH4 = motor 4 (rear-right, CCW)

    MAVRIK Team1_VTOL effector order:
        aileron, elevator, rudder, thrFR, thrFL, thrAR, thrAL, flapsRight, flapsLeft
    """

    # Trim computed from Team1_VTOL.json mass properties:
    #   CG_x = -0.114 ft, fwd arm = 1.854 ft, aft arm = 1.778 ft
    #   T_fwd = 24.26 lbf (0.122), T_aft = 25.28 lbf (0.127)  (W=49.54, max T=396.96)
    HOVER_THR_FWD = 0.122
    HOVER_THR_AFT = 0.127

    def __init__(self, flaps_fixed: float = 1.0,
                 aileron_channel: Optional[int] = None,
                 elevator_channel: Optional[int] = None,
                 rudder_channel: Optional[int] = None):
        self.flaps_fixed = flaps_fixed
        self.aileron_channel = aileron_channel
        self.elevator_channel = elevator_channel
        self.rudder_channel = rudder_channel
        self.has_armed = False
        self._ap_has_armed = False   # True once AP sends real motor PWM (arm detected)
        self.last_pwm = [1000, 1000, 1000, 1000]   # for diagnostics
        self.last_controls = (0.0,) * 9              # for diagnostics
        # State feedback for pre-arm stabilization
        self._pitch = 0.0   # radians
        self._roll = 0.0    # radians
        self._yaw = 0.0     # radians
        self._q = 0.0       # pitch rate rad/s
        self._p = 0.0       # roll rate rad/s
        self._r = 0.0       # yaw rate rad/s
        self._u = 0.0       # forward body velocity ft/s
        self._v = 0.0       # lateral body velocity ft/s
        self._pitch_integ = 0.0
        self._roll_integ  = 0.0
        self._vel_pitch_integ = 0.0  # velocity PI: integrates u error to find trim pitch
        self._vel_roll_integ  = 0.0  # velocity PI: integrates v error to find trim roll

    def set_state(self, pitch_rad: float, roll_rad: float, yaw_rad: float,
                  q_rad_s: float, p_rad_s: float, r_rad_s: float):
        """Called each tick with latest MAVRIK attitude for pre-arm stabilization."""
        self._pitch = pitch_rad
        self._roll = roll_rad
        self._yaw = yaw_rad
        self._q = q_rad_s
        self._p = p_rad_s
        self._r = r_rad_s

    def set_velocity(self, u_fps: float, v_fps: float):
        """Feed forward velocity so pre-arm loop can damp out horizontal drift."""
        self._u = u_fps
        self._v = v_fps

    def ground_hold_controls(self) -> Tuple[float, ...]:
        """PD attitude hold for pre-arm hover. Keeps the drone level and near
        spawn altitude while ArduPilot's EKF converges. Positive pitch = nose up;
        correcting nose-up means adding front thrust and reducing rear thrust."""
        pitch_corr = PRE_ARM_KP * self._pitch + PRE_ARM_KD * self._q
        roll_corr  = PRE_ARM_KP * self._roll  + PRE_ARM_KD * self._p
        thr_fr = max(0.0, min(1.0, PRE_ARM_HOVER_LEVEL + pitch_corr - roll_corr))
        thr_fl = max(0.0, min(1.0, PRE_ARM_HOVER_LEVEL + pitch_corr + roll_corr))
        thr_ar = max(0.0, min(1.0, PRE_ARM_HOVER_LEVEL - pitch_corr - roll_corr))
        thr_al = max(0.0, min(1.0, PRE_ARM_HOVER_LEVEL - pitch_corr + roll_corr))
        # MAVRIK order: aileron, elevator, rudder, thrFR, thrFL, thrAR, thrAL, flapsR, flapsL
        return (0.0, 0.0, 0.0, thr_fr, thr_fl, thr_ar, thr_al, VTOL_FLAP, VTOL_FLAP)

    def map(self, pwm: List[int]) -> Tuple[float, ...]:
        # Pad to at least 9 channels
        while len(pwm) < 9:
            pwm.append(PWM_MID)

        self.last_pwm = list(pwm[:9])
        
        if not self.has_armed:
            if any(p > 1200 for p in pwm[:4]):
                self.has_armed = True
                print("\n  [Bridge] ArduPilot commanded thrust > 1200us. Handing over control to Ardupilot EKF!\n")

        # Pass-through: MAVRIK's 4 motors from ArduPlane CH1-4.
        # ArduPlane QuadX pitch mixing: raises CH2+CH4 ("rear" pair) to pitch nose-UP.
        # In MAVRIK: nose-UP requires FORWARD motors (x=+1.739) to increase.
        # Therefore CH2/CH4 must map to MAVRIK fwd slots, CH1/CH3 to aft slots.
        #
        # Roll is preserved: right-side = CH1+CH4, left-side = CH2+CH3.
        #   CH1=thrAR (aft-right), CH4=thrFR (fwd-right) → right side thrust
        #   CH2=thrFL (fwd-left),  CH3=thrAL (aft-left)  → left side thrust
        # AP roll-right raises CH1+CH4 → thrAR+thrFR (right side) UP → rolls RIGHT ✓
        # AP pitch-up  raises CH2+CH4 → thrFL+thrFR (fwd motors)  UP → nose UP  ✓
        #
        # Confirmed from bridge_diag.csv 2026-05-11: ap_split<0 (CH2>CH1) → pitch UP.
        # Corrected Mapping (ArduPlane QuadX -> MAVRIK Team1_VTOL):
        # AP CH1 = Front-Right -> MAVRIK Fwd-Right (thr_fr)
        # AP CH2 = Rear-Left   -> MAVRIK Aft-Left  (thr_al)
        # AP CH3 = Front-Left  -> MAVRIK Fwd-Left  (thr_fl)
        # AP CH4 = Rear-Right  -> MAVRIK Aft-Right (thr_ar)
        thr_fr = pwm_to_normalized(pwm[0])  # CH1 -> MAVRIK Fwd-Right
        thr_al = pwm_to_normalized(pwm[1])  # CH2 -> MAVRIK Aft-Left
        thr_fl = pwm_to_normalized(pwm[2])  # CH3 -> MAVRIK Fwd-Left
        thr_ar = pwm_to_normalized(pwm[3])  # CH4 -> MAVRIK Aft-Right

        # Pass-through: Aero surfaces from CH5-CH7
        aileron  = pwm_to_symmetric(pwm[4]) * 15.0  # CH5
        elevator = pwm_to_symmetric(pwm[5]) * 15.0  # CH6
        rudder   = pwm_to_symmetric(pwm[6]) * 15.0  # CH7

        # Tilt servos from CH8-CH9 (Ardupilot TiltMotorLeft/Right).
        # ArduPilot: 1000=VTOL hover (motors up), 2000=FW (motors forward).
        # MAVRIK: 0.99=VTOL (motors up), 0.0=FW (motors forward).
        # Inverted mapping: 1000→1.0(VTOL), 2000→0.0(FW).
        # SERVO8/9_TRIM=1500: AP outputs 1500 at hover → snap band → 0.99.
        # Cap result at VTOL_FLAP: AP sends 1000 (SERVO_MIN) in modes that don't
        # use vectored yaw, which would produce 1.0 (backward tilt). MAVRIK has no
        # backward-tilt range so clamp to 0.99.
        def _tilt(pwm_us: int) -> float:
            return min(VTOL_FLAP, 1.0 - pwm_to_normalized(pwm_us))
        flaps_l = _tilt(pwm[7])  # CH8
        flaps_r = _tilt(pwm[8])  # CH9

        # MAVRIK packet order (Team1_VTOL.json control_effectors):
        #   [0]=aileron [1]=elevator [2]=rudder
        #   [3]=throttleFwdRight [4]=throttleFwdLeft [5]=throttleAftRight [6]=throttleAftLeft
        #   [7]=flapsRight [8]=flapsLeft
        ctrl = (aileron, elevator, rudder, thr_fr, thr_fl, thr_ar, thr_al, flaps_r, flaps_l)
        self.last_controls = ctrl
        return ctrl


# ---------------------------------------------------------------------------
# ArduPilot JSON backend server
# ---------------------------------------------------------------------------

class ArduPilotBackend:
    def __init__(self, bind_ip: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)       # non-blocking: poll only, no sleep in recv
        self.last_reply_addr: Optional[Tuple[str, int]] = None
        self.last_frame_count: int = -1
        self.frames_rx = 0
        self.frames_skipped = 0
        self.json_sent = 0

    def recv_pwm(self) -> Optional[PWMFrame]:
        """Non-blocking PWM poll. Returns None immediately if no frame available."""
        try:
            data, addr = self.sock.recvfrom(2048)
        except (socket.timeout, BlockingIOError, OSError):
            return None

        self.last_reply_addr = addr

        if len(data) == ARDUPILOT_FRAME_16CH_SIZE:
            unpacked = struct.unpack(ARDUPILOT_FRAME_16CH_FMT, data)
            magic, frame_rate, frame_count = unpacked[0], unpacked[1], unpacked[2]
            pwm = list(unpacked[3:])
        elif len(data) == ARDUPILOT_FRAME_32CH_SIZE:
            unpacked = struct.unpack(ARDUPILOT_FRAME_32CH_FMT, data)
            magic, frame_rate, frame_count = unpacked[0], unpacked[1], unpacked[2]
            pwm = list(unpacked[3:])
        else:
            return None

        if magic not in (ARDUPILOT_MAGIC_16CH, ARDUPILOT_MAGIC_32CH):
            return None

        if self.last_frame_count >= 0 and frame_count != self.last_frame_count + 1:
            self.frames_skipped += frame_count - self.last_frame_count - 1
        self.last_frame_count = frame_count
        self.frames_rx += 1
        return PWMFrame(frame_rate=frame_rate, frame_count=frame_count, pwm=pwm)

    def send_json(self, payload: dict) -> bool:
        if self.last_reply_addr is None:
            return False
        text = json_lib.dumps(payload, separators=(",", ":"))
        try:
            self.sock.sendto((text + "\n").encode("utf-8"), self.last_reply_addr)
        except OSError:
            # SITL port closed (ICMP unreachable → ConnectionRefusedError).
            # Don't crash — SITL may restart on the same port.
            return False
        self.json_sent += 1
        return True

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# MAVRIK I/O
# ---------------------------------------------------------------------------

class MavrikIO:
    def __init__(self, state_port: int, control_ip: str, control_port: int,
                 state_bind_ip: str = "0.0.0.0"):
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind((state_bind_ip, state_port))
        self.state_sock.setblocking(False)

        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_dest = (control_ip, control_port)

        self.states_rx = 0
        self.controls_tx = 0

    def poll_latest_state(self) -> Optional[MavrikState]:
        latest = None
        try:
            while True:
                data, _ = self.state_sock.recvfrom(4096)
                if len(data) >= MAVRIK_STATE_SIZE:
                    latest = data
        except BlockingIOError:
            pass
        if latest is None:
            return None
        try:
            s = MavrikState.unpack(latest)
            self.states_rx += 1
            return s
        except struct.error:
            return None

    def send_controls(self, controls: Tuple[float, ...]):
        pkt = struct.pack(MAVRIK_CONTROL_FMT, *controls)
        self.ctrl_sock.sendto(pkt, self.ctrl_dest)
        self.controls_tx += 1

    def close(self):
        self.state_sock.close()
        self.ctrl_sock.close()


# ---------------------------------------------------------------------------
# State -> ArduPilot JSON
# ---------------------------------------------------------------------------

class StateToJSON:
    """Converts MAVRIK state to ArduPilot JSON. Holds previous state for
    finite-differenced body-frame accelerometer."""

    def __init__(self):
        self.prev_state: Optional[MavrikState] = None

    def reset(self):
        self.prev_state = None

    def _accel_body(self, s: MavrikState) -> Tuple[float, float, float]:
        """Specific-force body frame (m/s^2). Accelerometer reading.

        Follows f_body = dv_body/dt + omega x v_body - g_body
        At rest + level, this returns approximately (0, 0, -9.81).
        For MVP we drop the omega x v_body cross-coupling; small in hover.
        """
        q = (s.e0, s.ex, s.ey, s.ez)
        g_body = earth_to_body((0.0, 0.0, GRAVITY_MS2), q)

        if self.prev_state is None or s.t <= self.prev_state.t:
            self.prev_state = s
            # Bootstrap: assume static, only gravity
            return (-g_body[0], -g_body[1], -g_body[2])

        dt = max(s.t - self.prev_state.t, 0.005)
        u_prev, v_prev, w_prev = self.prev_state.u, self.prev_state.v, self.prev_state.w
        du = (s.u - u_prev) * FT_TO_M / dt
        dv = (s.v - v_prev) * FT_TO_M / dt
        dw = (s.w - w_prev) * FT_TO_M / dt
        
        # Apply a low-pass filter to prevent massive dV/dt noise from confusing ArduPilot's EKF
        alpha = 0.2
        if not hasattr(self, '_last_du'):
            self._last_du, self._last_dv, self._last_dw = du, dv, dw
        else:
            du = alpha * du + (1.0 - alpha) * self._last_du
            dv = alpha * dv + (1.0 - alpha) * self._last_dv
            dw = alpha * dw + (1.0 - alpha) * self._last_dw
            self._last_du, self._last_dv, self._last_dw = du, dv, dw

        u_ms, v_ms, w_ms = s.u * FT_TO_M, s.v * FT_TO_M, s.w * FT_TO_M
        du += (s.q * w_ms - s.r * v_ms)
        dv += (s.r * u_ms - s.p * w_ms)
        dw += (s.p * v_ms - s.q * u_ms)

        self.prev_state = s
        # specific force = dv_body/dt + omega x v - g_body
        return (du - g_body[0], dv - g_body[1], dw - g_body[2])

    def build(self, s: MavrikState, override_t: Optional[float] = None) -> dict:
        q = (s.e0, s.ex, s.ey, s.ez)
        roll, pitch, yaw = quat_to_euler(*q)

        # velocities
        u_ms, v_ms, w_ms = s.u * FT_TO_M, s.v * FT_TO_M, s.w * FT_TO_M
        vn, ve, vd = body_to_earth((u_ms, v_ms, w_ms), q)

        # position (already earth NED in ft)
        pn, pe, pd = s.xf * FT_TO_M, s.yf * FT_TO_M, s.zf * FT_TO_M

        # accel
        ax, ay, az = self._accel_body(s)

        t_out = override_t if override_t is not None else s.t

        return {
            "timestamp": t_out,
            "imu": {
                "gyro":       [s.p, s.q, s.r],
                "accel_body": [ax, ay, az],
            },
            "position":   [pn, pe, pd],
            "attitude":   [roll, pitch, yaw],
            "velocity":   [vn, ve, vd],
        }


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = {
    "ardupilot_in": {"bind_ip": "127.0.0.1", "port": 9002},
    "mavrik": {
        "state_port":   5009,
        "control_ip":   "127.0.0.1",
        "control_port": 5006
    },
    "pwm_mapping": {
        "flaps_fixed": 1.0,
        "aileron_channel":  None,
        "elevator_channel": None,
        "rudder_channel":   None,
        "_comment": "ArduCopter quad uses CH1-4 for motors. Leave servo channels null for hover MVP."
    },
    "status_print_hz": 2.0
}


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[bridge] writing default config to {path}")
        with open(path, "w") as f:
            json_lib.dump(DEFAULT_CONFIG, f, indent=4)
        return DEFAULT_CONFIG
    with open(path, "r") as f:
        return json_lib.load(f)


# ---------------------------------------------------------------------------
# MAVLink RC_CHANNELS_OVERRIDE builder (no external dependencies)
# ---------------------------------------------------------------------------

def _x25crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

_rc_seq = 0

def build_rc_override_pkt(channels_pwm: list) -> bytes:
    """Build a MAVLink v1 RC_CHANNELS_OVERRIDE packet for ArduPilot.
    channels_pwm: list of up to 8 PWM values (1000-2000), 0 = don't override.
    """
    global _rc_seq
    import struct
    # Pad to exactly 8 channels
    pwm = list(channels_pwm[:8])
    while len(pwm) < 8:
        pwm.append(0)
    target_sys, target_comp = 1, 1
    payload = struct.pack('<BBHHHHHHHH', target_sys, target_comp, *pwm)
    msg_id  = 70    # RC_CHANNELS_OVERRIDE
    crc_extra = 124  # MAVLink-defined for msg_id 70
    seq = _rc_seq & 0xFF
    _rc_seq += 1
    header = bytes([len(payload), seq, 255, 0, msg_id])  # len, seq, sysid=GCS, compid, msgid
    crc = _x25crc(header + payload + bytes([crc_extra]))
    return b'\xFE' + header + payload + struct.pack('<H', crc)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(config_path: str):
    cfg = load_config(config_path)

    ap = ArduPilotBackend(cfg["ardupilot_in"]["bind_ip"], cfg["ardupilot_in"]["port"])
    mx = MavrikIO(cfg["mavrik"]["state_port"],
                  cfg["mavrik"]["control_ip"], cfg["mavrik"]["control_port"])
    pwm_map = PWMMapper(
        flaps_fixed=cfg["pwm_mapping"]["flaps_fixed"],
        aileron_channel=cfg["pwm_mapping"].get("aileron_channel"),
        elevator_channel=cfg["pwm_mapping"].get("elevator_channel"),
        rudder_channel=cfg["pwm_mapping"].get("rudder_channel"),
    )
    to_json = StateToJSON()
    pre_arm_ctrl = PreArmController()

    print("=" * 70)
    print("  MAVRIK <-> ArduPilot SITL bridge")
    print("=" * 70)
    print(f"  Listening for ArduPilot PWM:  udp://{cfg['ardupilot_in']['bind_ip']}:{cfg['ardupilot_in']['port']}")
    print(f"  MAVRIK state (in):            udp://*:{cfg['mavrik']['state_port']}")
    print(f"  MAVRIK controls (out):        udp://{cfg['mavrik']['control_ip']}:{cfg['mavrik']['control_port']}")
    print(f"  flaps held at:                {cfg['pwm_mapping']['flaps_fixed']}")
    print()
    print("  Start ArduPilot with:")
    print("    sim_vehicle.py -v ArduCopter -f quad --model JSON:127.0.0.1 \\")
    print("                   --map --console")
    print()
    print("  Ctrl+C to stop.\n")

    running = {"ok": True}
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__("ok", False))

    # Grace period after handover: send hover trim to MAVRIK instead of AP's low PWM
    # so pid_vtol has time to exit before the bridge takes exclusive control of port 5006.
    # 150 ms = ~300 bridge loop ticks at 2 kHz, well past pid_vtol's ~20 ms shutdown.
    HANDOVER_GRACE_SECS = 0.15   # just long enough for pid_vtol to stop (~20ms)
    _handover_grace_until  = 0.0  # wall-clock: grace period end
    _soft_floor_until      = 0.0  # wall-clock: apply motor soft-floor until this time

    # Port 5012 is held by pid_vtol.py during pre-arm. Bind lazily after handover.
    rc_rx_sock = None
    _rc_sock_bound = False
    # Send MAVLink to ArduPilot's GCS MAVLink port (SITL default: 14550)
    mavlink_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # UDP socket used to signal pid_vtol.py to stop on handover
    _stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("  RC override (in):             udp://*:5012  (bound after handover to ArduPilot)")
    print("  RC override (out):            MAVLink RC_CHANNELS_OVERRIDE → 127.0.0.1:14552")
    print()
    latest_state: Optional[MavrikState] = None
    _mavrik_connected = False   # True once the first MAVRIK state packet arrives
    _mavrik_valid_count = 0     # consecutive valid packets received; gate for _mavrik_connected
    _last_mavrik_rx = 0.0       # wall-clock time of last valid MAVRIK packet
    MAVRIK_TIMEOUT = 0.5        # seconds without a packet before falling back to synthetic state
    bridge_start = time.time()
    _t_offset = 0.0             # added to MAVRIK sim-time so output timestamps are monotonic
    print_interval = 1.0 / max(0.1, cfg.get("status_print_hz", 10.0))
    next_print = time.time() + print_interval
    # Rate-limit how often we send JSON to ArduPilot (200 Hz).
    # Controls to MAVRIK are sent every loop iteration (no rate limit) so MAVRIK
    # never waits longer than one loop period (~500 µs) for a control packet.
    AP_JSON_INTERVAL = 1.0 / 200.0
    next_ap_json = time.time()

    # --- CSV diagnostic log ---
    log_path = "bridge_diag.csv"
    log_file = open(log_path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "wall_clock", "sim_t",
        "alt_ft", "roll_deg", "pitch_deg", "yaw_deg",
        "u_fps", "v_fps", "w_fps", "p_rads", "q_rads", "r_rads",
        "pwm1", "pwm2", "pwm3", "pwm4",
        "ail", "ele", "rud",
        "thrFR", "thrFL", "thrAR", "thrAL",
        "flapsR", "flapsL",
        "armed",
        # Live AP frame PWM — updated every loop tick from raw frame (not just post-handover)
        "ap_pwm1", "ap_pwm2", "ap_pwm3", "ap_pwm4", "ap_pwm8", "ap_pwm9",
        "ap_has_armed", "motor_active",
    ])
    log_hz = 50  # write at most 50 rows/sec
    log_interval = 1.0 / log_hz
    next_log = time.time()
    _latest_ap_pwm = [0] * 16   # raw AP frame PWM, updated every loop tick
    print(f"  Diagnostic log:  {os.path.abspath(log_path)}")
    print()

    while running["ok"]:
        now = time.time()

        # Watchdog: if MAVRIK was connected but stopped sending, fall back to synthetic state
        # so ArduPilot's JSON backend keeps receiving advancing timestamps and doesn't stall.
        if _mavrik_connected and (now - _last_mavrik_rx) > MAVRIK_TIMEOUT:
            _mavrik_connected = False
            _mavrik_valid_count = 0
            to_json.reset()
            print("  [Bridge] MAVRIK timed out — falling back to synthetic state.")

        # Drain any MAVRIK state packets regardless of whether ArduPilot is active
        s = mx.poll_latest_state()
        if s is not None:
            if not _mavrik_state_valid(s):
                # Garbage packet from MAVRIK startup/crash — discard and reset counter
                _mavrik_valid_count = 0
                print(f"  [Bridge] Invalid MAVRIK packet discarded "
                      f"(t={s.t:.3f} zf={s.zf:.1f} q={s.e0:.2f},{s.ex:.2f},{s.ey:.2f},{s.ez:.2f})")
            else:
                _mavrik_valid_count += 1
                _last_mavrik_rx = now
                latest_state = s
                if not _mavrik_connected and _mavrik_valid_count >= 3:
                    _mavrik_connected = True
                    pre_arm_ctrl.reset()   # restart ramp from the moment MAVRIK connects
                    # Compute offset so timestamps continue from where Phase 1 left off.
                    # Phase 1 sent wall-clock t; MAVRIK sim starts from ~0.
                    # Without this, ArduPilot sees time go backward and drops frames / resets.
                    _t_offset = (time.time() - bridge_start) - latest_state.t
                    to_json.reset()  # clear synthetic accel history so first real frame bootstraps cleanly
                    print(f"  [Bridge] MAVRIK connected (t_offset={_t_offset:.2f}s)")
                roll, pitch, yaw = quat_to_euler(s.e0, s.ex, s.ey, s.ez)
                pwm_map.set_state(pitch, roll, yaw, s.q, s.p, s.r)
                pwm_map.set_velocity(s.u, s.v)

        # ── Step 1: deliver controls to MAVRIK immediately (no blocking delays) ──────
        # MAVRIK blocks on its first recvfrom(5006) before advancing its sim loop.
        # Sending here — before any ArduPilot polling — minimises the wait to one
        # loop period (~500 µs).
        if not pwm_map.has_armed:
            # pid_vtol.py owns port 5006 throughout pre-arm.
            # It sends cached trim (0.125 symmetric) at 2 kHz even before MAVRIK connects,
            # so MAVRIK's recvfrom never blocks and we never fight pid_vtol with freefall zeros.
            pass

        # ── Step 2: non-blocking poll for ArduPilot PWM frame ────────────────────
        frame = ap.recv_pwm()   # returns None immediately if nothing queued
        if frame is not None:
            _latest_ap_pwm = frame.pwm  # track raw AP output for logging/diagnostics

        if not pwm_map.has_armed:
            # Detect AP arm: any motor > 1000 = AP has armed (disarmed = exactly 1000).
            # Fire handover on the VERY FIRST armed frame (~1006 PWM at sim_t≈4.27s).
            # SITL crashes ~0.6s after arm because the EKF sees AP commanding thrust
            # but the physics (controlled by pid_vtol) doesn't respond to AP's commands.
            # By handing over immediately, AP commands go to MAVRIK, physics is consistent,
            # EKF is satisfied, and SITL stays alive. At 3000ft there is ample buffer
            # for the motor ramp-up before MAVRIK approaches any altitude limit.
            if frame is not None and not pwm_map._ap_has_armed:
                if any(p > 1000 for p in frame.pwm[:4]):
                    pwm_map._ap_has_armed = True
                    print(f"  [Bridge] AP armed (pwm={frame.pwm[:4]}) — handing over immediately...")
            # Handover gate: hover thrust + near-level attitude.
            if pwm_map._ap_has_armed and frame is not None and latest_state is not None:
                roll_deg  = math.degrees(abs(pwm_map._roll))
                pitch_deg = math.degrees(abs(pwm_map._pitch))
                real_alt_ft = -latest_state.zf
                if roll_deg > 15.0 or pitch_deg > 15.0:
                    print(f"  [Gate] waiting attitude: R={roll_deg:.1f}° P={pitch_deg:.1f}°", end='\r')
                else:
                    pwm_map.has_armed = True
                    _handover_grace_until = now + HANDOVER_GRACE_SECS
                    _soft_floor_until     = now + HANDOVER_GRACE_SECS + 0.50  # 500ms soft floor after grace
                    to_json.reset()  # clear accel history so first live frame is clean
                    print(f"\n  [Bridge] Handover: roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° "
                          f"alt={real_alt_ft:.0f}ft "
                          f"pwm=[{frame.pwm[0]},{frame.pwm[1]},{frame.pwm[2]},{frame.pwm[3]}]\n")
                    _stop_sock.sendto(b'\x01', ('127.0.0.1', 5015))
                    print(f"  [Bridge] Stop signal sent to pid_vtol.py (port 5015). "
                          f"Grace period: {HANDOVER_GRACE_SECS*1000:.0f}ms hover trim.")
        else:
            if now < _handover_grace_until:
                # Grace period: keep the bridge's own PD controller active so
                # pitch/roll are actively corrected while pid_vtol winds down (~20ms).
                # Using constant PRE_ARM_HOVER caused pitch to drift unchecked,
                # arriving at handover with several degrees of built-up error.
                active_ctrl = pre_arm_ctrl.update(pwm_map._pitch, pwm_map._roll, pwm_map._q, pwm_map._p)
                mx.send_controls(active_ctrl)
                pwm_map.last_controls = active_ctrl
            else:
                if not getattr(pwm_map, '_grace_ended', False):
                    pwm_map._grace_ended = True
                    print("  [Bridge] Grace ended — forwarding AP to MAVRIK directly.")
                if frame is not None:
                    controls = pwm_map.map(frame.pwm)
                    pwm_map.last_controls = controls
                    mx.send_controls(controls)

        # ── Step 3: send state to ArduPilot at 200 Hz (rate-limited) ─────────────
        # Phase 1 — no MAVRIK yet:  synthetic level state at spawn altitude.
        # Phase 2 — pre-arm:        real altitude + forced level attitude.
        # Phase 3 — armed:          real MAVRIK state.
        if now >= next_ap_json and ap.last_reply_addr is not None:
            next_ap_json = now + AP_JSON_INTERVAL
            if not _mavrik_connected:
                payload = to_json.build(SYNTHETIC_INITIAL_STATE,
                                        override_t=now - bridge_start)
            elif not pwm_map.has_armed:
                payload = to_json.build(make_level_proxy(latest_state),
                                        override_t=latest_state.t + _t_offset)
            else:
                payload = to_json.build(latest_state,
                                        override_t=latest_state.t + _t_offset)
            ap.send_json(payload)

        # ── Step 4: yield CPU so we don't spin at 100% ───────────────────────────
        time.sleep(0.0005)   # 500 µs — loop runs at ~2 kHz, controls to MAVRIK every ~500 µs

        # --- CSV log ---
        if now >= next_log and latest_state is not None:
            next_log = now + log_interval
            ls = latest_state
            r, p, y = quat_to_euler(ls.e0, ls.ex, ls.ey, ls.ez)
            c = pwm_map.last_controls
            pw = pwm_map.last_pwm
            ap_pw = _latest_ap_pwm
            _motor_active_now = any(p > 1000 for p in ap_pw[:4])
            log_writer.writerow([
                f"{now:.3f}", f"{ls.t:.4f}",
                f"{-ls.zf:.1f}", f"{math.degrees(r):.2f}", f"{math.degrees(p):.2f}", f"{math.degrees(y):.2f}",
                f"{ls.u:.2f}", f"{ls.v:.2f}", f"{ls.w:.2f}",
                f"{ls.p:.4f}", f"{ls.q:.4f}", f"{ls.r:.4f}",
                pw[0], pw[1], pw[2], pw[3],
                f"{c[0]:.3f}", f"{c[1]:.3f}", f"{c[2]:.3f}",
                f"{c[3]:.4f}", f"{c[4]:.4f}", f"{c[5]:.4f}", f"{c[6]:.4f}",
                f"{c[7]:.4f}", f"{c[8]:.4f}",
                1 if pwm_map.has_armed else 0,
                ap_pw[0] if len(ap_pw) > 0 else 0,
                ap_pw[1] if len(ap_pw) > 1 else 0,
                ap_pw[2] if len(ap_pw) > 2 else 0,
                ap_pw[3] if len(ap_pw) > 3 else 0,
                ap_pw[7] if len(ap_pw) > 7 else 0,
                ap_pw[8] if len(ap_pw) > 8 else 0,
                1 if pwm_map._ap_has_armed else 0,
                1 if _motor_active_now else 0,
            ])
            log_file.flush()

        # --- RC override from web viewer (port 5012) → MAVLink to ArduPilot ---
        # Bind lazily after handover so pid_vtol.py can hold port 5012 during pre-arm.
        # Rate-limited to 1 Hz — creating a socket every 500µs (2 kHz loop) leaks
        # ~2000 OS file descriptors/s until pid_vtol releases the port.
        if pwm_map.has_armed and not _rc_sock_bound and now >= next_print:
            try:
                _tmp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                _tmp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                _tmp_sock.bind(('0.0.0.0', 5012))
                _tmp_sock.setblocking(False)
                rc_rx_sock = _tmp_sock
                _rc_sock_bound = True
                print("  [Bridge] RC override socket bound to port 5012.")
            except OSError:
                try: _tmp_sock.close()
                except Exception: pass

        # SAFETY: only forward when at least one axis is outside the deadband.
        # A centered/resting gamepad won't command zero throttle and crash.
        RC_DEADBAND = 0.05
        if rc_rx_sock is not None:
            try:
                rc_data, _ = rc_rx_sock.recvfrom(64)
                if len(rc_data) >= 16:
                    roll_n, pitch_n, yaw_n, thr_n = struct.unpack('<4f', rc_data[:16])
                    # All axes: center=0 → PWM 1500 (hover/neutral).
                    def _norm(v): return int(max(1000, min(2000, 1500 + v * 500)))
                    active = (abs(roll_n) > RC_DEADBAND or abs(pitch_n) > RC_DEADBAND or
                              abs(yaw_n)  > RC_DEADBAND or abs(thr_n)  > RC_DEADBAND)
                    if active:
                        # CH1=roll  CH2=pitch  CH3=throttle  CH4=yaw
                        channels = [_norm(roll_n), _norm(pitch_n), _norm(thr_n), _norm(yaw_n), 0, 0, 0, 0]
                        pkt = build_rc_override_pkt(channels)
                        mavlink_sock.sendto(pkt, ('127.0.0.1', 14552))
            except BlockingIOError:
                pass
            except Exception:
                pass

        # --- Console status ---
        if now >= next_print:
            next_print = now + print_interval
            pw = pwm_map.last_pwm
            c = pwm_map.last_controls
            armed_str = "ARMED" if pwm_map.has_armed else "PRE-ARM"
            if latest_state is None:
                print(f"  [{armed_str}] [waiting for MAVRIK]  "
                      f"pwm=[{pw[0]},{pw[1]},{pw[2]},{pw[3]}]  "
                      f"ap_rx={ap.frames_rx}  mavrik_rx={mx.states_rx}")
            else:
                r, p, y = quat_to_euler(latest_state.e0, latest_state.ex,
                                        latest_state.ey, latest_state.ez)
                alt_ft = -latest_state.zf
                u_fps  = latest_state.u
                # Pre-arm drift warning
                if not pwm_map.has_armed and abs(u_fps) > 5.0:
                    print(f"\033[91m  *** DRIFT WARNING: u={u_fps:+.1f} ft/s — ARM NOW or restart! ***\033[0m")
                flp_pwm_str = f"[{pw[7]},{pw[8]}]" if len(pw) > 8 else "[--,--]"
                print(f"  [{armed_str}] t={latest_state.t:6.1f}s  "
                      f"alt={alt_ft:5.0f}ft  "
                      f"R={math.degrees(r):+5.1f} P={math.degrees(p):+5.1f} Y={math.degrees(y):+5.1f}  "
                      f"u={u_fps:+5.1f}fps  "
                      f"pwm=[{pw[0]},{pw[1]},{pw[2]},{pw[3]}]  "
                      f"flp_pwm={flp_pwm_str} flp=[{c[7]:.3f},{c[8]:.3f}]")


    log_file.close()
    ap.close()
    mx.close()
    print("\n  Bridge stopped.")
    print(f"  ArduPilot frames rx: {ap.frames_rx} (skipped: {ap.frames_skipped})")
    print(f"  MAVRIK states rx:    {mx.states_rx}")
    print(f"  MAVRIK controls tx:  {mx.controls_tx}")
    print(f"  ArduPilot JSON tx:   {ap.json_sent}")
    print(f"  Diagnostic log:     {os.path.abspath(log_path)}")


def main():
    import traceback
    parser = argparse.ArgumentParser(description="MAVRIK <-> ArduPilot SITL JSON bridge.")
    parser.add_argument("--config", default="ardupilot_bridge_config.json",
                        help="Path to config JSON (created with defaults if missing).")
    args = parser.parse_args()
    try:
        run(args.config)
    except KeyboardInterrupt:
        print("\n[Bridge] KeyboardInterrupt — stopping.")
    except Exception:
        print(f"\n[Bridge] FATAL EXCEPTION:\n{traceback.format_exc()}")
        raise


if __name__ == "__main__":
    main()
