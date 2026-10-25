#!/usr/bin/env python3
"""Flies PX4 SITL through a square mission over MAVLink and reports how it went.

The v0.4 acceptance mission (docs/design/robot-classes.md §5): take off, fly a square, land.
Talks to PX4's offboard API link (UDP 14540 by default) with pymavlink: uploads the mission,
arms, starts it, and follows it until PX4 reports it landed and disarmed. Exit status 0 when
every waypoint was reached and it landed, 3 otherwise.

Usage: fly_square_mission.py [--side 10] [--altitude 5] [--timeout 300] [--link udpin:0.0.0.0:14540]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any

from pymavlink import mavutil

EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis: offsets of a few metres only


def wait(link: Any, kinds: list[str], timeout: float) -> Any:
    return link.recv_match(type=kinds, blocking=True, timeout=timeout)


def offset(lat: float, lon: float, north: float, east: float) -> tuple[int, int]:
    """degE7 of a point `north`, `east` metres from (lat, lon) degrees."""
    dlat = math.degrees(north / EARTH_RADIUS_M)
    dlon = math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    return round((lat + dlat) * 1e7), round((lon + dlon) * 1e7)


def mission(lat: float, lon: float, side: float, altitude: float) -> list[tuple[int, int, int]]:
    """(command, latE7, lonE7) items: take-off, the four corners, back, land."""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    items = [(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, *offset(lat, lon, 0.0, 0.0))]
    items += [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, *offset(lat, lon, n, e)) for n, e in corners]
    items.append((mavutil.mavlink.MAV_CMD_NAV_LAND, *offset(lat, lon, 0.0, 0.0)))
    return items


def upload(link: Any, items: list[tuple[int, int, int]], altitude: float) -> bool:
    link.mav.mission_count_send(link.target_system, link.target_component, len(items))
    sent = set()
    while len(sent) < len(items):
        msg = wait(link, ["MISSION_REQUEST_INT", "MISSION_REQUEST"], 5.0)
        if msg is None:
            return False
        command, lat, lon = items[msg.seq]
        link.mav.mission_item_int_send(
            link.target_system,
            link.target_component,
            msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command,
            1 if msg.seq == 0 else 0,
            1,
            0.0,
            2.0,
            0.0,
            math.nan,
            lat,
            lon,
            altitude,
        )
        sent.add(msg.seq)
    ack = wait(link, ["MISSION_ACK"], 5.0)
    return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED


def command(link: Any, cmd: int, *params: float) -> None:
    values = list(params) + [0.0] * (7 - len(params))
    link.mav.command_long_send(link.target_system, link.target_component, cmd, 0, *values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--side", type=float, default=10.0, help="m")
    parser.add_argument("--altitude", type=float, default=5.0, help="m above the start")
    parser.add_argument("--timeout", type=float, default=300.0, help="s of wall time")
    parser.add_argument("--link", default="udpin:0.0.0.0:14540")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.link)
    if link.wait_heartbeat(timeout=60) is None:
        print("no heartbeat from PX4", file=sys.stderr)
        return 3
    print(f"PX4 system {link.target_system}")
    # Fly without a radio: no RC, no data link loss action (simulation only).
    for name, value in (("COM_RC_IN_MODE", 4), ("NAV_RCL_ACT", 0), ("NAV_DLL_ACT", 0)):
        link.mav.param_set_send(
            link.target_system,
            link.target_component,
            name.encode(),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
    home = None
    start = time.monotonic()
    while home is None and time.monotonic() - start < 120:
        msg = wait(link, ["GLOBAL_POSITION_INT"], 2.0)
        if msg is not None and msg.lat != 0:
            home = (msg.lat / 1e7, msg.lon / 1e7)
    if home is None:
        print("no position estimate", file=sys.stderr)
        return 3
    print(f"home {home[0]:.7f}, {home[1]:.7f}")
    items = mission(*home, args.side, args.altitude)
    if not upload(link, items, args.altitude):
        print("mission upload failed", file=sys.stderr)
        return 3
    print(f"mission of {len(items)} items uploaded")

    reached: set[int] = set()
    armed_once = False
    deadline = time.monotonic() + args.timeout
    next_try = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if not armed_once and now > next_try:
            command(link, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            command(link, mavutil.mavlink.MAV_CMD_MISSION_START, 0, len(items) - 1)
            next_try = now + 3.0
        msg = wait(
            link,
            [
                "MISSION_ITEM_REACHED",
                "HEARTBEAT",
                "STATUSTEXT",
                "EXTENDED_SYS_STATE",
                "GLOBAL_POSITION_INT",
            ],
            1.0,
        )
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            print(f"PX4: {msg.text}")
        elif kind == "MISSION_ITEM_REACHED" and msg.seq not in reached:
            reached.add(msg.seq)
            print(f"reached item {msg.seq}")
        elif kind == "HEARTBEAT" and msg.get_srcSystem() == link.target_system:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            armed_once = armed_once or armed
            if armed_once and not armed:
                print("landed and disarmed")
                break
        elif kind == "GLOBAL_POSITION_INT" and armed_once and int(now) % 5 == 0:
            print(f"  t {now - start:.0f} s alt {msg.relative_alt / 1000:.1f} m")
    waypoints = set(range(1, len(items) - 1))
    ok = armed_once and waypoints <= reached
    print(f"waypoints reached {sorted(reached)}; {'ok' if ok else 'FAILED'}")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
