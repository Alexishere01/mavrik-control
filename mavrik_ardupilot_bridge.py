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
    #   T_fwd = 39.06 lbf (0.394), T_aft = 40.72 lbf (0.410)
    HOVER_THR_FWD = 0.394
    HOVER_THR_AFT = 0.410

    def __init__(self, flaps_fixed: float = 1.0,
                 aileron_channel: Optional[int] = None,
                 elevator_channel: Optional[int] = None,
                 rudder_channel: Optional[int] = None):
        self.flaps_fixed = flaps_fixed
        self.aileron_channel = aileron_channel
        self.elevator_channel = elevator_channel
        self.rudder_channel = rudder_channel
        self.has_armed = False
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

    def map(self, pwm: List[int]) -> Tuple[float, ...]:
        # Pad to at least 8 channels with midpoints
        while len(pwm) < 8:
            pwm.append(PWM_MID)

        self.last_pwm = list(pwm[:4])

        # Pre-arm hover hold: MAVRIK starts at 250ft and has no ground collision.
        # The vehicle is statically unstable in pitch, so we need active
        # stabilization (P+D on pitch & roll) to keep it level while ArduPilot
        # initializes its EKF.
        if not self.has_armed:
            if any(p > 1200 for p in pwm[:4]):
                self.has_armed = True
                print("\n  [Bridge] ArduPilot commanded thrust > 1200us. Handing over control!\n")
            else:
                # --- Simple P+D attitude hold ---
                # With flaps_fixed=1.0 (motors perfectly vertical), there is no
                # horizontal thrust component, so holding 0deg pitch = zero drift.
                KP_PITCH = 0.20
                KD_PITCH = 0.04
                KP_ROLL  = 0.10
                KD_ROLL  = 0.02
                KP_YAW   = 0.15
                KD_YAW   = 0.05

                pitch_corr = -KP_PITCH * self._pitch - KD_PITCH * self._q
                roll_corr  = -KP_ROLL  * self._roll  - KD_ROLL  * self._p
                yaw_corr   = -KP_YAW   * self._yaw   - KD_YAW   * self._r

                # pitch_corr > 0 means "need more nose-down" → increase fwd, decrease aft
                thr_fr = max(0.0, min(1.0, self.HOVER_THR_FWD + pitch_corr - roll_corr))
                thr_fl = max(0.0, min(1.0, self.HOVER_THR_FWD + pitch_corr + roll_corr))
                thr_ar = max(0.0, min(1.0, self.HOVER_THR_AFT - pitch_corr - roll_corr))
                thr_al = max(0.0, min(1.0, self.HOVER_THR_AFT - pitch_corr + roll_corr))

                # Arms can go past 90deg (flaps > 1.0)
                flaps_r = max(0.0, min(1.1, self.flaps_fixed + yaw_corr))
                flaps_l = max(0.0, min(1.1, self.flaps_fixed - yaw_corr))

                ctrl = (0.0, 0.0, 0.0,
                        thr_fr, thr_fl, thr_ar, thr_al,
                        flaps_r, flaps_l)
                self.last_controls = ctrl
                return ctrl

        # MAVRIK's 4 motors from ArduCopter's 4 motors, quad-X layout
        # ArduPilot outputs a "collective" thrust level that is symmetric.
        # We extract the collective and differential components separately so
        # we can layer the CG-compensation bias on top.
        raw_fr = pwm_to_normalized(pwm[0])  # CH1 -> front-right
        raw_al = pwm_to_normalized(pwm[1])  # CH2 -> rear-left (aft-left)
        raw_fl = pwm_to_normalized(pwm[2])  # CH3 -> front-left
        raw_ar = pwm_to_normalized(pwm[3])  # CH4 -> rear-right (aft-right)

        # Collective thrust (0..1) from ArduPilot
        collective = (raw_fr + raw_al + raw_fl + raw_ar) / 4.0

        # Differential corrections ArduPilot is requesting (relative to collective)
        diff_fr = raw_fr - collective
        diff_al = raw_al - collective
        diff_fl = raw_fl - collective
        diff_ar = raw_ar - collective

        # Apply CG-compensation bias + ArduPilot's differential corrections.
        # The MAVRIK airframe has a forward CG, so forward motors need more thrust
        # than aft motors to maintain level hover. We always keep this bias active.
        thr_fr = max(0.0, min(1.0, collective * self.HOVER_THR_FWD / 0.402 + diff_fr))
        thr_fl = max(0.0, min(1.0, collective * self.HOVER_THR_FWD / 0.402 + diff_fl))
        thr_ar = max(0.0, min(1.0, collective * self.HOVER_THR_AFT / 0.402 + diff_ar))
        thr_al = max(0.0, min(1.0, collective * self.HOVER_THR_AFT / 0.402 + diff_al))

        # Extract yaw demand from ArduPilot's quad-X mixer for differential tilt.
        # Arms can go past 90deg (flaps > 1.0), allow up to 1.1 (~99deg)
        yaw_diff = (raw_fr + raw_ar - raw_fl - raw_al) / 4.0
        flaps_r = max(0.0, min(1.1, self.flaps_fixed + yaw_diff * 2.0))
        flaps_l = max(0.0, min(1.1, self.flaps_fixed - yaw_diff * 2.0))

        aileron  = pwm_to_symmetric(pwm[self.aileron_channel - 1])  * 15.0 if self.aileron_channel  else 0.0
        elevator = pwm_to_symmetric(pwm[self.elevator_channel - 1]) * 15.0 if self.elevator_channel else 0.0
        rudder   = pwm_to_symmetric(pwm[self.rudder_channel - 1])   * 15.0 if self.rudder_channel   else 0.0

        # MAVRIK packet order: aileron, elevator, rudder, thrFR, thrFL, thrAR, thrAL, flapsRight, flapsLeft
        ctrl = (aileron, elevator, rudder, thr_fr, thr_fl, thr_ar, thr_al,
                flaps_r, flaps_l)
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
        self.sock.settimeout(0.005)       # short block so we can poll MAVRIK states rapidly
        self.last_reply_addr: Optional[Tuple[str, int]] = None
        self.last_frame_count: int = -1
        self.frames_rx = 0
        self.frames_skipped = 0
        self.json_sent = 0

    def recv_pwm(self) -> Optional[PWMFrame]:
        """Blocks up to 50 ms for a PWM frame. Returns None on timeout."""
        try:
            data, addr = self.sock.recvfrom(2048)
        except socket.timeout:
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
        # ArduPilot expects a trailing newline, according to the example backends
        self.sock.sendto((text + "\n").encode("utf-8"), self.last_reply_addr)
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

    # --- Port 5012: receive RC override from web viewer ---
    rc_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rc_rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rc_rx_sock.bind(('0.0.0.0', 5012))
    rc_rx_sock.setblocking(False)
    # Send MAVLink to ArduPilot's GCS MAVLink port (SITL default: 14550)
    mavlink_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("  RC override (in):             udp://*:5012  (from web viewer)")
    print("  RC override (out):            MAVLink RC_CHANNELS_OVERRIDE → 127.0.0.1:14550")
    print()
    latest_state: Optional[MavrikState] = None
    print_interval = 1.0 / max(0.1, cfg.get("status_print_hz", 10.0))
    next_print = time.time() + print_interval

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
    ])
    log_hz = 50  # write at most 50 rows/sec
    log_interval = 1.0 / log_hz
    next_log = time.time()
    print(f"  Diagnostic log:  {os.path.abspath(log_path)}")
    print()

    while running["ok"]:
        # Drain any MAVRIK state packets regardless of whether ArduPilot is active
        s = mx.poll_latest_state()
        if s is not None:
            latest_state = s
            # Feed attitude to the mapper for pre-arm stabilization
            roll, pitch, yaw = quat_to_euler(s.e0, s.ex, s.ey, s.ez)
            pwm_map.set_state(pitch, roll, yaw, s.q, s.p, s.r)
            # Feed velocity so the pre-arm loop can damp horizontal drift
            pwm_map.set_velocity(s.u, s.v)

        # Wait (up to 50ms) for a PWM frame from ArduPilot
        frame = ap.recv_pwm()

        if frame is not None:
            controls = pwm_map.map(frame.pwm)
            mx.send_controls(controls)
        elif not pwm_map.has_armed:
            controls = pwm_map.map([1000, 1000, 1000, 1000])
            mx.send_controls(controls)

        # Send state back to ArduPilot natively at 100Hz (when MAVRIK updates).
        # We've configured ArduCopter to natively accept 100Hz via SCHED_LOOP_RATE 
        # and INS_GYRO_RATE, so we don't need to artificially interpolate an 800Hz clock.
        if s is not None and ap.last_reply_addr is not None:
            payload = to_json.build(latest_state)
            ap.send_json(payload)

        now = time.time()

        # --- CSV log ---
        if now >= next_log and latest_state is not None:
            next_log = now + log_interval
            ls = latest_state
            r, p, y = quat_to_euler(ls.e0, ls.ex, ls.ey, ls.ez)
            c = pwm_map.last_controls
            pw = pwm_map.last_pwm
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
            ])
            log_file.flush()

        # --- RC override from web viewer (port 5012) → MAVLink to ArduPilot ---
        # SAFETY: only forward when at least one axis is outside the deadband.
        # A centered/resting gamepad won't command zero throttle and crash.
        RC_DEADBAND = 0.05
        try:
            rc_data, _ = rc_rx_sock.recvfrom(64)
            if len(rc_data) >= 16:
                roll_n, pitch_n, yaw_n, thr_n = struct.unpack('<4f', rc_data[:16])
                # All axes: center=0 → PWM 1500 (hover/neutral).
                # OLD _thr mapped 0 → 1000 (zero thrust) which caused crashes.
                def _norm(v): return int(max(1000, min(2000, 1500 + v * 500)))
                active = (abs(roll_n) > RC_DEADBAND or abs(pitch_n) > RC_DEADBAND or
                          abs(yaw_n)  > RC_DEADBAND or abs(thr_n)  > RC_DEADBAND)
                if active:
                    # CH1=roll  CH2=pitch  CH3=throttle  CH4=yaw
                    channels = [_norm(roll_n), _norm(pitch_n), _norm(thr_n), _norm(yaw_n), 0, 0, 0, 0]
                    pkt = build_rc_override_pkt(channels)
                    mavlink_sock.sendto(pkt, ('127.0.0.1', 14550))
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
                print(f"  [{armed_str}] t={latest_state.t:6.1f}s  "
                      f"alt={alt_ft:5.0f}ft  "
                      f"R={math.degrees(r):+5.1f} P={math.degrees(p):+5.1f} Y={math.degrees(y):+5.1f}  "
                      f"u={u_fps:+5.1f}fps  "
                      f"pwm=[{pw[0]},{pw[1]},{pw[2]},{pw[3]}]  "
                      f"flp=[{c[7]:.3f},{c[8]:.3f}]")

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
    parser = argparse.ArgumentParser(description="MAVRIK <-> ArduPilot SITL JSON bridge.")
    parser.add_argument("--config", default="ardupilot_bridge_config.json",
                        help="Path to config JSON (created with defaults if missing).")
    args = parser.parse_args()
    run(args.config)


if __name__ == "__main__":
    main()
