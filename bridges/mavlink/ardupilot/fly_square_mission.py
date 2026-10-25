#!/usr/bin/env python3
"""Flies ArduCopter SITL through a square mission over MAVLink and reports how it went.

The v0.4 mission (docs/design/robot-classes.md §5) for ArduPilot: GUIDED take-off to the mission
altitude, a square in AUTO, landing (ArduCopter disarms by itself after it). Talks to SITL's
first serial port (TCP 5760 by default) with pymavlink. Exit status 0 when every waypoint was
reached and it landed, 3 otherwise.

Usage: fly_square_mission.py [--side 10] [--altitude 5] [--timeout 300] [--link tcp:127.0.0.1:5760]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

from pymavlink import mavutil

EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis: offsets of a few metres only
GUIDED, AUTO = 4, 3  # ArduCopter flight modes


def offset(lat: float, lon: float, north: float, east: float) -> tuple[int, int]:
    dlat = math.degrees(north / EARTH_RADIUS_M)
    dlon = math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return round((lat + dlat) * 1e7), round((lon + dlon) * 1e7)


def command(link: Any, cmd: int, *params: float) -> None:
    values = list(params) + [0.0] * (7 - len(params))
    link.mav.command_long_send(link.target_system, link.target_component, cmd, 0, *values)


def set_mode(link: Any, mode: int) -> None:
    command(
        link,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode,
    )


def upload(link: Any, home: tuple[float, float], side: float, altitude: float) -> int:
    """Uploads home (item 0, as ArduPilot keeps it), the square and a landing; the item count."""
    lat, lon = home
    corners = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, *offset(lat, lon, 0.0, 0.0), 0.0)]
    items += [
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, *offset(lat, lon, n, e), altitude)
        for n, e in corners
    ]
    items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, *offset(lat, lon, 0.0, 0.0), 0.0))
    link.mav.mission_count_send(link.target_system, link.target_component, len(items))
    sent = 0
    while sent < len(items):
        msg = link.recv_match(
            type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5.0
        )
        if msg is None:
            return 0
        cmd, la, lo, alt = items[msg.seq]
        link.mav.mission_item_int_send(
            link.target_system,
            link.target_component,
            msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            cmd,
            0,
            1,
            0.0,
            1.0,
            0.0,
            0.0,
            la,
            lo,
            alt,
        )
        sent = max(sent, msg.seq + 1)
    ack = link.recv_match(type="MISSION_ACK", blocking=True, timeout=5.0)
    return len(items) if ack is not None and ack.type == 0 else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--side", type=float, default=10.0, help="m")
    parser.add_argument("--altitude", type=float, default=5.0, help="m above the start")
    parser.add_argument("--timeout", type=float, default=300.0, help="s of wall time")
    parser.add_argument("--link", default="tcp:127.0.0.1:5760")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.link, retries=60)
    if link.wait_heartbeat(timeout=60) is None:
        print("no heartbeat from ArduPilot", file=sys.stderr)
        return 3
    link.mav.request_data_stream_send(
        link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )
    print(f"ArduPilot system {link.target_system}")
    deadline = time.monotonic() + args.timeout
    home = None
    while home is None and time.monotonic() < deadline:
        msg = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is not None and msg.lat != 0:
            home = (msg.lat / 1e7, msg.lon / 1e7)
    if home is None:
        return 3
    count = upload(link, home, args.side, args.altitude)
    if count == 0:
        print("mission upload failed", file=sys.stderr)
        return 3
    print(f"home {home[0]:.7f}, {home[1]:.7f}; mission of {count} items uploaded")

    # Arm in GUIDED once the EKF accepts it (it refuses while it settles), take off, then AUTO.
    armed = False
    while not armed and time.monotonic() < deadline:
        set_mode(link, GUIDED)
        command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        until = time.monotonic() + 3.0
        while time.monotonic() < until and not armed:
            msg = link.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print(f"ArduPilot: {msg.text}")
            elif msg.get_srcSystem() == link.target_system:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if not armed:
        print("ArduPilot did not arm", file=sys.stderr)
        return 3
    print("armed")
    command(link, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.altitude)
    while time.monotonic() < deadline:
        msg = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is not None and msg.relative_alt > 0.9 * args.altitude * 1000:
            break
    print("took off; AUTO")
    set_mode(link, AUTO)

    reached: set[int] = set()
    landed = False
    while time.monotonic() < deadline:
        msg = link.recv_match(
            type=["MISSION_ITEM_REACHED", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1.0
        )
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"ArduPilot: {msg.text}")
        elif msg.get_type() == "MISSION_ITEM_REACHED" and msg.seq not in reached:
            reached.add(msg.seq)
            print(f"reached item {msg.seq}")
        elif (
            msg.get_type() == "HEARTBEAT"
            and msg.get_srcSystem() == link.target_system
            and not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            landed = True
            print("landed and disarmed")
            break
    waypoints = set(range(1, count - 1))
    ok = landed and waypoints <= reached
    print(f"waypoints reached {sorted(reached)}; {'ok' if ok else 'FAILED'}")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
