#!/usr/bin/env python3
"""
mavrik_qgc_bridge.py
====================
Read-only telemetry bridge: MAVRIK simulator -> QGroundControl (MAVLink v2).

MAVRIK sends a 14-float little-endian state packet over UDP:
    [t, u, v, w, p, q, r, xf, yf, zf, e0, ex, ey, ez]

This script receives those packets, converts the state into MAVLink
messages (HEARTBEAT, ATTITUDE, VFR_HUD, GLOBAL_POSITION_INT), and
publishes them to QGroundControl's default UDP endpoint.

Run order:
    1. Start QGroundControl (listens on 14550).
    2. Start this bridge.
    3. Start MAVRIK with a connections.json entry that sends state on :5008.

Requires:
    pip install pymavlink
"""

import argparse
import json
import math
import os
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

FT_TO_M = 0.3048
M_PER_DEG_LAT = 111_320.0          # close enough for local flat-earth
STATE_PACKET_FLOATS = 14
STATE_PACKET_BYTES = STATE_PACKET_FLOATS * 4


# ---------------------------------------------------------------------------
# State structure
# ---------------------------------------------------------------------------

@dataclass
class MavrikState:
    t: float                       # sim time (s)
    u: float; v: float; w: float   # body velocities (ft/s)
    p: float; q: float; r: float   # body angular rates (rad/s)
    xf: float; yf: float; zf: float  # earth-fixed NED position (ft), z-down
    e0: float; ex: float; ey: float; ez: float  # quaternion, scalar first

    @classmethod
    def unpack(cls, data: bytes) -> "MavrikState":
        floats = struct.unpack("<14f", data[:STATE_PACKET_BYTES])
        return cls(*floats)


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def quat_to_euler(e0: float, ex: float, ey: float, ez: float) -> Tuple[float, float, float]:
    """Scalar-first quaternion -> (roll, pitch, yaw) radians, ZYX convention."""
    sinp = max(-1.0, min(1.0, 2.0 * (e0 * ey - ez * ex)))
    roll  = math.atan2(2.0 * (e0 * ex + ey * ez), 1.0 - 2.0 * (ex * ex + ey * ey))
    pitch = math.asin(sinp)
    yaw   = math.atan2(2.0 * (e0 * ez + ex * ey), 1.0 - 2.0 * (ey * ey + ez * ez))
    return roll, pitch, yaw


def body_to_earth(v_body: Tuple[float, float, float],
                  q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Rotate a body-frame vector to earth-fixed frame using quaternion (e0, ex, ey, ez).

    Matches view.py's body_2_fixed. NED earth frame (x north, y east, z down).
    """
    x, y, z = v_body
    e0, ex, ey, ez = q
    T0 = x * ex + y * ey + z * ez
    T1 = x * e0 - y * ez + z * ey
    T2 = x * ez + y * e0 - z * ex
    T3 = y * ex - x * ey + z * e0
    return (e0 * T1 + ex * T0 + ey * T3 - ez * T2,
            e0 * T2 - ex * T3 + ey * T0 + ez * T1,
            e0 * T3 + ex * T2 - ey * T1 + ez * T0)


def wrap_heading_deg(yaw_rad: float) -> float:
    """Yaw (rad) -> heading (deg) in [0, 360)."""
    deg = math.degrees(yaw_rad) % 360.0
    return deg if deg >= 0 else deg + 360.0


# ---------------------------------------------------------------------------
# Coordinate transform (local NED ft -> global lat/lon)
# ---------------------------------------------------------------------------

class CoordinateTransformer:
    """Flat-earth local NED (ft) -> WGS84 lat/lon/alt. Good for local-area sims."""

    def __init__(self, origin_lat_deg: float, origin_lon_deg: float, origin_alt_m: float):
        self.origin_lat_deg = origin_lat_deg
        self.origin_lon_deg = origin_lon_deg
        self.origin_alt_m = origin_alt_m
        self._m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(origin_lat_deg))

    def xf_yf_ft_to_latlon(self, xf_ft: float, yf_ft: float) -> Tuple[float, float]:
        """xf = north-positive (ft), yf = east-positive (ft) -> (lat_deg, lon_deg)."""
        dlat = (xf_ft * FT_TO_M) / M_PER_DEG_LAT
        dlon = (yf_ft * FT_TO_M) / self._m_per_deg_lon
        return self.origin_lat_deg + dlat, self.origin_lon_deg + dlon

    def zf_ft_to_alt_m(self, zf_ft: float) -> Tuple[float, float]:
        """zf (ft, positive-down NED) -> (alt_msl_m, alt_rel_to_origin_m)."""
        alt_rel_m = -zf_ft * FT_TO_M
        alt_msl_m = self.origin_alt_m + alt_rel_m
        return alt_msl_m, alt_rel_m


# ---------------------------------------------------------------------------
# MAVRIK state receiver
# ---------------------------------------------------------------------------

class MavrikReceiver:
    def __init__(self, bind_ip: str, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)
        self.packets = 0
        self.bad_packets = 0

    def poll_latest(self) -> Optional[MavrikState]:
        """Drain buffer and return the most recent valid packet, or None."""
        latest = None
        try:
            while True:
                data, _ = self.sock.recvfrom(4096)
                if len(data) < STATE_PACKET_BYTES:
                    self.bad_packets += 1
                    continue
                latest = data
        except BlockingIOError:
            pass

        if latest is None:
            return None

        try:
            state = MavrikState.unpack(latest)
            self.packets += 1
            return state
        except struct.error:
            self.bad_packets += 1
            return None

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# QGC publisher
# ---------------------------------------------------------------------------

class QGCPublisher:
    def __init__(self, host: str, port: int, system_id: int, component_id: int):
        self.conn = mavutil.mavlink_connection(
            f"udpout:{host}:{port}",
            source_system=system_id,
            source_component=component_id,
            dialect="common",
        )
        self.host = host
        self.port = port
        self.t_start = time.time()

    def _boot_ms(self) -> int:
        return int((time.time() - self.t_start) * 1000) & 0xFFFFFFFF

    def heartbeat(self):
        self.conn.mav.heartbeat_send(
            type=mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode=0,
            system_status=mavlink.MAV_STATE_ACTIVE,
        )

    def attitude(self, roll: float, pitch: float, yaw: float,
                 p: float, q: float, r: float):
        self.conn.mav.attitude_send(
            time_boot_ms=self._boot_ms(),
            roll=roll, pitch=pitch, yaw=yaw,
            rollspeed=p, pitchspeed=q, yawspeed=r,
        )

    def vfr_hud(self, airspeed_ms: float, groundspeed_ms: float,
                heading_deg: float, throttle_pct: float,
                alt_msl_m: float, climb_ms: float):
        self.conn.mav.vfr_hud_send(
            airspeed=airspeed_ms,
            groundspeed=groundspeed_ms,
            heading=int(heading_deg) % 360,
            throttle=max(0, min(100, int(throttle_pct))),
            alt=alt_msl_m,
            climb=climb_ms,
        )

    def global_position_int(self, lat_deg: float, lon_deg: float,
                            alt_msl_m: float, alt_rel_m: float,
                            vn_ms: float, ve_ms: float, vd_ms: float,
                            heading_deg: float):
        self.conn.mav.global_position_int_send(
            time_boot_ms=self._boot_ms(),
            lat=int(lat_deg * 1e7),
            lon=int(lon_deg * 1e7),
            alt=int(alt_msl_m * 1000),
            relative_alt=int(alt_rel_m * 1000),
            vx=int(vn_ms * 100),
            vy=int(ve_ms * 100),
            vz=int(vd_ms * 100),
            hdg=int(heading_deg * 100) % 36000,
        )


# ---------------------------------------------------------------------------
# Rate scheduler
# ---------------------------------------------------------------------------

class RateScheduler:
    """Fires true() at most `hz` times per second, paced by wall clock."""

    def __init__(self, hz: float):
        self.interval = 1.0 / hz if hz > 0 else float("inf")
        self.next_due = 0.0

    def due(self, now: float) -> bool:
        if now >= self.next_due:
            self.next_due = now + self.interval
            return True
        return False


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

DEFAULT_CONFIG = {
    "mavrik_in": {"bind_ip": "0.0.0.0", "port": 5008},
    "qgc_out":   {"ip": "127.0.0.1",    "port": 14550},
    "origin": {
        "lat_deg": 0.0,
        "lon_deg": 0.0,
        "alt_msl_m": 0.0,
        "_comment": "MAVRIK input.json defaults lat/lon to (0,0). Change these to a real "
                    "location (e.g. your test airfield) if you want a useful QGC map view."
    },
    "rates_hz": {
        "heartbeat": 1.0,
        "attitude": 30.0,
        "vfr_hud": 10.0,
        "global_position": 5.0
    },
    "mavlink": {
        "system_id": 1,
        "component_id": 1
    },
    "status_print_hz": 1.0
}


def load_config(path: str) -> dict:
    if not os.path.exists(path):
        print(f"[bridge] writing default config to {path}")
        with open(path, "w") as f:
            json.dump(DEFAULT_CONFIG, f, indent=4)
        return DEFAULT_CONFIG
    with open(path, "r") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(config_path: str):
    cfg = load_config(config_path)

    rx = MavrikReceiver(cfg["mavrik_in"]["bind_ip"], cfg["mavrik_in"]["port"])
    pub = QGCPublisher(
        cfg["qgc_out"]["ip"], cfg["qgc_out"]["port"],
        cfg["mavlink"]["system_id"], cfg["mavlink"]["component_id"],
    )
    xform = CoordinateTransformer(
        cfg["origin"]["lat_deg"],
        cfg["origin"]["lon_deg"],
        cfg["origin"]["alt_msl_m"],
    )

    sch_hb  = RateScheduler(cfg["rates_hz"]["heartbeat"])
    sch_att = RateScheduler(cfg["rates_hz"]["attitude"])
    sch_hud = RateScheduler(cfg["rates_hz"]["vfr_hud"])
    sch_pos = RateScheduler(cfg["rates_hz"]["global_position"])
    sch_log = RateScheduler(cfg.get("status_print_hz", 1.0))

    print("=" * 70)
    print("  MAVRIK -> QGroundControl bridge")
    print("=" * 70)
    print(f"  Listening:   udp://{cfg['mavrik_in']['bind_ip']}:{cfg['mavrik_in']['port']}")
    print(f"  Sending to:  udp://{cfg['qgc_out']['ip']}:{cfg['qgc_out']['port']}")
    print(f"  Origin:      lat={cfg['origin']['lat_deg']:.6f}  "
          f"lon={cfg['origin']['lon_deg']:.6f}  alt={cfg['origin']['alt_msl_m']:.0f}m")
    print(f"  Rates (Hz):  HB={cfg['rates_hz']['heartbeat']}  "
          f"ATT={cfg['rates_hz']['attitude']}  "
          f"HUD={cfg['rates_hz']['vfr_hud']}  "
          f"POS={cfg['rates_hz']['global_position']}")
    print("  Ctrl+C to stop.\n")

    running = {"ok": True}
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__("ok", False))

    last_state: Optional[MavrikState] = None
    msg_counts = {"hb": 0, "att": 0, "hud": 0, "pos": 0}

    while running["ok"]:
        now = time.time()

        # Heartbeat always — QGC needs it even before state arrives.
        if sch_hb.due(now):
            pub.heartbeat()
            msg_counts["hb"] += 1

        # Drain all queued state packets, keep the newest.
        state = rx.poll_latest()
        if state is not None:
            last_state = state

        if last_state is not None:
            # Derive once per loop; cheap.
            roll, pitch, yaw = quat_to_euler(
                last_state.e0, last_state.ex, last_state.ey, last_state.ez
            )
            vel_body_ms = (last_state.u * FT_TO_M,
                           last_state.v * FT_TO_M,
                           last_state.w * FT_TO_M)
            vn, ve, vd = body_to_earth(
                vel_body_ms,
                (last_state.e0, last_state.ex, last_state.ey, last_state.ez),
            )
            airspeed = math.sqrt(sum(c * c for c in vel_body_ms))
            groundspeed = math.sqrt(vn * vn + ve * ve)
            heading_deg = wrap_heading_deg(yaw)
            alt_msl_m, alt_rel_m = xform.zf_ft_to_alt_m(last_state.zf)
            lat, lon = xform.xf_yf_ft_to_latlon(last_state.xf, last_state.yf)

            if sch_att.due(now):
                pub.attitude(roll, pitch, yaw,
                             last_state.p, last_state.q, last_state.r)
                msg_counts["att"] += 1

            if sch_hud.due(now):
                pub.vfr_hud(airspeed, groundspeed, heading_deg,
                            throttle_pct=0,  # bridge can't see commanded throttle
                            alt_msl_m=alt_msl_m, climb_ms=-vd)
                msg_counts["hud"] += 1

            if sch_pos.due(now):
                pub.global_position_int(lat, lon, alt_msl_m, alt_rel_m,
                                        vn, ve, vd, heading_deg)
                msg_counts["pos"] += 1

        if sch_log.due(now):
            if last_state is None:
                print(f"  [waiting for MAVRIK]  rx_packets={rx.packets}")
            else:
                roll_d  = math.degrees(quat_to_euler(
                    last_state.e0, last_state.ex, last_state.ey, last_state.ez)[0])
                pitch_d = math.degrees(quat_to_euler(
                    last_state.e0, last_state.ex, last_state.ey, last_state.ez)[1])
                alt_ft = -last_state.zf
                print(f"  t={last_state.t:7.2f}s  "
                      f"roll={roll_d:+6.1f}°  pitch={pitch_d:+6.1f}°  "
                      f"alt={alt_ft:6.0f}ft  "
                      f"rx={rx.packets}  "
                      f"tx[hb/att/hud/pos]={msg_counts['hb']}/"
                      f"{msg_counts['att']}/{msg_counts['hud']}/{msg_counts['pos']}")

        time.sleep(0.005)  # ~200 Hz scheduling tick

    rx.close()
    print("\n  Bridge stopped.")
    print(f"  Total rx packets: {rx.packets}  (bad: {rx.bad_packets})")
    print(f"  Total tx messages: hb={msg_counts['hb']} "
          f"att={msg_counts['att']} hud={msg_counts['hud']} pos={msg_counts['pos']}")


def main():
    ap = argparse.ArgumentParser(description="MAVRIK -> QGroundControl MAVLink bridge.")
    ap.add_argument("--config", default="qgc_bridge_config.json",
                    help="Path to config JSON (created with defaults if missing).")
    args = ap.parse_args()
    run(args.config)


if __name__ == "__main__":
    main()
