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
    - Tilt: held at flaps=0.99 (fwd motors near-vertical, with known drift).
    - Servos: aileron/elevator/rudder held at 0.
    - Accel: finite-differenced body velocity minus gravity-in-body.
"""

import argparse
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

    def __init__(self, flaps_fixed: float = 0.99,
                 aileron_channel: Optional[int] = None,
                 elevator_channel: Optional[int] = None,
                 rudder_channel: Optional[int] = None):
        self.flaps_fixed = flaps_fixed
        self.aileron_channel = aileron_channel
        self.elevator_channel = elevator_channel
        self.rudder_channel = rudder_channel

    def map(self, pwm: List[int]) -> Tuple[float, ...]:
        # Pad to at least 8 channels with midpoints
        while len(pwm) < 8:
            pwm.append(PWM_MID)

        # MAVRIK's 4 motors from ArduCopter's 4 motors, quad-X layout
        thr_fr = pwm_to_normalized(pwm[0])  # CH1 -> front-right
        thr_al = pwm_to_normalized(pwm[1])  # CH2 -> rear-left (aft-left)
        thr_fl = pwm_to_normalized(pwm[2])  # CH3 -> front-left
        thr_ar = pwm_to_normalized(pwm[3])  # CH4 -> rear-right (aft-right)

        aileron  = pwm_to_symmetric(pwm[self.aileron_channel - 1])  * 15.0 if self.aileron_channel  else 0.0
        elevator = pwm_to_symmetric(pwm[self.elevator_channel - 1]) * 15.0 if self.elevator_channel else 0.0
        rudder   = pwm_to_symmetric(pwm[self.rudder_channel - 1])   * 15.0 if self.rudder_channel   else 0.0

        # MAVRIK packet order: aileron, elevator, rudder, thrFR, thrFL, thrAR, thrAL, flapsRight, flapsLeft
        return (aileron, elevator, rudder, thr_fr, thr_fl, thr_ar, thr_al,
                self.flaps_fixed, self.flaps_fixed)


# ---------------------------------------------------------------------------
# ArduPilot JSON backend server
# ---------------------------------------------------------------------------

class ArduPilotBackend:
    def __init__(self, bind_ip: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(0.05)       # short block so we can still tick timers
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

        dt = s.t - self.prev_state.t
        u_prev, v_prev, w_prev = self.prev_state.u, self.prev_state.v, self.prev_state.w
        du = (s.u - u_prev) * FT_TO_M / dt
        dv = (s.v - v_prev) * FT_TO_M / dt
        dw = (s.w - w_prev) * FT_TO_M / dt
        self.prev_state = s
        # specific force = dv_body/dt - g_body
        return (du - g_body[0], dv - g_body[1], dw - g_body[2])

    def build(self, s: MavrikState) -> dict:
        q = (s.e0, s.ex, s.ey, s.ez)
        roll, pitch, yaw = quat_to_euler(*q)

        # velocities
        u_ms, v_ms, w_ms = s.u * FT_TO_M, s.v * FT_TO_M, s.w * FT_TO_M
        vn, ve, vd = body_to_earth((u_ms, v_ms, w_ms), q)

        # position (already earth NED in ft)
        pn, pe, pd = s.xf * FT_TO_M, s.yf * FT_TO_M, s.zf * FT_TO_M

        # accel
        ax, ay, az = self._accel_body(s)

        return {
            "timestamp": s.t,
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
        "flaps_fixed": 0.99,
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

    latest_state: Optional[MavrikState] = None
    print_interval = 1.0 / max(0.1, cfg.get("status_print_hz", 2.0))
    next_print = time.time() + print_interval

    while running["ok"]:
        # Drain any MAVRIK state packets regardless of whether ArduPilot is active
        s = mx.poll_latest_state()
        if s is not None:
            latest_state = s

        # Wait (up to 50ms) for a PWM frame from ArduPilot
        frame = ap.recv_pwm()

        if frame is not None:
            # Forward controls to MAVRIK
            controls = pwm_map.map(frame.pwm)
            mx.send_controls(controls)

            # Reply with current state as JSON
            if latest_state is not None:
                payload = to_json.build(latest_state)
                ap.send_json(payload)

        now = time.time()
        if now >= next_print:
            next_print = now + print_interval
            if latest_state is None:
                print(f"  [waiting for MAVRIK state]  "
                      f"ap_frames={ap.frames_rx}  mavrik_states={mx.states_rx}")
            else:
                r, p, y = quat_to_euler(latest_state.e0, latest_state.ex,
                                        latest_state.ey, latest_state.ez)
                alt_ft = -latest_state.zf
                print(f"  t={latest_state.t:7.2f}s  "
                      f"alt={alt_ft:6.0f}ft  "
                      f"roll={math.degrees(r):+6.1f}  pitch={math.degrees(p):+6.1f}  "
                      f"ap_rx={ap.frames_rx}  json_tx={ap.json_sent}  "
                      f"mavrik_rx={mx.states_rx}  ctrl_tx={mx.controls_tx}  "
                      f"ap_skipped={ap.frames_skipped}")

    ap.close()
    mx.close()
    print("\n  Bridge stopped.")
    print(f"  ArduPilot frames rx: {ap.frames_rx} (skipped: {ap.frames_skipped})")
    print(f"  MAVRIK states rx:    {mx.states_rx}")
    print(f"  MAVRIK controls tx:  {mx.controls_tx}")
    print(f"  ArduPilot JSON tx:   {ap.json_sent}")


def main():
    parser = argparse.ArgumentParser(description="MAVRIK <-> ArduPilot SITL JSON bridge.")
    parser.add_argument("--config", default="ardupilot_bridge_config.json",
                        help="Path to config JSON (created with defaults if missing).")
    args = parser.parse_args()
    run(args.config)


if __name__ == "__main__":
    main()
