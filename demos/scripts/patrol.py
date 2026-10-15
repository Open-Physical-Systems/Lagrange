#!/usr/bin/env python3
# Sends the AMR of amr_warehouse.launch.py around the warehouse aisles and reports each leg.
#
#   ros2 run lagrange_demos patrol                    # one round
#   ros2 run lagrange_demos patrol --rounds 3
#   ros2 run lagrange_demos patrol --minutes 10       # keep patrolling for 10 simulated minutes
#
# It also watches for collisions: the robot's footprint is tested against the occupancy map of
# the scenery at every true pose the server publishes (/ground_truth/odom; /odom may be wheel
# odometry, which drifts).
#
# Exits non-zero if a waypoint was missed or the robot touched anything (tools/ci/nav2_demo.sh
# runs it nightly: v0.2 acceptance criterion 1).
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

# (x, y, heading) in the map frame: around the pallet, along both racks and back.
WAYPOINTS = [
    (4.0, -1.5, 0.0),
    (7.5, 0.0, math.pi / 2),
    (4.0, 2.0, math.pi),
    (-7.5, 2.0, math.pi),
    (-7.5, -2.0, 0.0),
    (0.0, -1.5, 0.0),
]


# The robot's footprint in base_link: the 0.60 m chassis, and the wheels out to y = +-0.255 m.
FOOTPRINT_HALF_X = 0.30
FOOTPRINT_HALF_Y = 0.255


class CollisionWatch:
    """Tests the footprint against the occupancy map at every ground-truth pose.

    The map marks a cell occupied when its centre lies inside a fixed shape, so an occupied
    cell centre inside the footprint proves the two overlap: no false alarms, and any overlap
    deeper than half a cell (2.5 cm) is caught. Also keeps the closest approach.
    """

    def __init__(self, map_yaml: Path) -> None:
        info = yaml.safe_load(map_yaml.read_text())
        self.resolution = float(info["resolution"])
        self.origin = (float(info["origin"][0]), float(info["origin"][1]))
        data = (map_yaml.parent / info["image"]).read_bytes()
        # P5 header: magic, (comments), width, height, maxval, then one byte per pixel.
        fields: list[bytes] = []
        at = 0
        while len(fields) < 4:
            line_end = data.index(b"\n", at)
            line = data[at:line_end]
            at = line_end + 1
            if not line.startswith(b"#"):
                fields.extend(line.split())
        self.width, self.height = int(fields[1]), int(fields[2])
        self.pixels = data[at : at + self.width * self.height]
        self.collisions = 0
        self.clearance = math.inf  # m, closest approach of an occupied cell to the footprint
        self.where = (0.0, 0.0)

    def occupied(self, col: int, row_from_bottom: int) -> bool:
        if not (0 <= col < self.width and 0 <= row_from_bottom < self.height):
            return False
        return self.pixels[(self.height - 1 - row_from_bottom) * self.width + col] < 128

    def check(self, odom: Odometry) -> None:
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        reach = math.hypot(FOOTPRINT_HALF_X, FOOTPRINT_HALF_Y) + 0.5
        res = self.resolution
        c0 = int((p.x - reach - self.origin[0]) / res)
        c1 = int((p.x + reach - self.origin[0]) / res) + 1
        r0 = int((p.y - reach - self.origin[1]) / res)
        r1 = int((p.y + reach - self.origin[1]) / res) + 1
        touching = False
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                if not self.occupied(col, row):
                    continue
                dx = self.origin[0] + (col + 0.5) * res - p.x
                dy = self.origin[1] + (row + 0.5) * res - p.y
                local_x = abs(cos_yaw * dx + sin_yaw * dy) - FOOTPRINT_HALF_X
                local_y = abs(-sin_yaw * dx + cos_yaw * dy) - FOOTPRINT_HALF_Y
                distance = math.hypot(max(local_x, 0.0), max(local_y, 0.0))
                if distance < self.clearance:
                    self.clearance = distance
                    self.where = (p.x, p.y)
                touching = touching or (local_x <= 0.0 and local_y <= 0.0)
        if touching:
            self.collisions += 1
            if self.collisions == 1:
                print(f"COLLISION at ({p.x:.2f}, {p.y:.2f})", flush=True)


def pose(navigator: BasicNavigator, x: float, y: float, heading: float) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(heading / 2.0)
    goal.pose.orientation.w = math.cos(heading / 2.0)
    return goal


def wait_until_active(navigator: BasicNavigator, nodes: list[str]) -> None:
    """BasicNavigator.waitUntilNav2Active, with a timeout on each get_state call.

    nav2_simple_commander waits for each reply without a timeout, so one lost service response
    (DDS sometimes drops them while many nodes start at once) blocks the patrol for good.
    """
    for name in nodes:
        client = navigator.create_client(GetState, f"{name}/get_state")
        while not client.wait_for_service(timeout_sec=1.0):
            navigator.info(f"{name}/get_state service not available, waiting...")
        while True:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
            result = future.result() if future.done() else None
            if result is not None and result.current_state.label == "active":
                break
            if not future.done():
                future.cancel()
            time.sleep(1.0)
        navigator.destroy_client(client)
    navigator.info("Nav2 is ready for use!")


def main() -> int:
    parser = argparse.ArgumentParser(description="Patrol the warehouse with Nav2.")
    parser.add_argument("--rounds", type=int, default=1, help="patrol at least this many rounds")
    parser.add_argument(
        "--minutes", type=float, default=0.0, help="patrol at least this long in sim time"
    )
    parser.add_argument(
        "--leg-timeout", type=float, default=120.0, help="give up a leg after this (sim s)"
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    navigator = BasicNavigator(node_name="lagrange_patrol")
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    watch = CollisionWatch(
        get_package_share_path("lagrange_demos") / "maps" / "warehouse_small.yaml"
    )
    # Spun with the navigator: isTaskComplete() spins its node.
    navigator.create_subscription(Odometry, "/ground_truth/odom", watch.check, QoSProfile(depth=10))
    # With ros2_control, AMCL localizes and the diff drive publishes /odom once spawned: wait for
    # both, or the first goal goes out before the robot can move.
    deadline = time.monotonic() + 60.0
    while "amcl" not in navigator.get_node_names() and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.5)
        if "map_server" in navigator.get_node_names() and time.monotonic() > deadline - 50.0:
            break  # the builtin controller: ground-truth odometry, no localization
    localizer = "amcl" if "amcl" in navigator.get_node_names() else "map_server"
    wait_until_active(navigator, [localizer, "bt_navigator"])
    odometry: list[Odometry] = []
    navigator.create_subscription(Odometry, "/odom", odometry.append, QoSProfile(depth=1))
    while not odometry and time.monotonic() < deadline + 60.0:
        rclpy.spin_once(navigator, timeout_sec=0.5)
    if not odometry:
        print("no /odom: is the robot's drive running?", flush=True)
        return 1

    start = navigator.get_clock().now()
    reached = failed = 0
    leg = 0
    while True:
        x, y, heading = WAYPOINTS[leg % len(WAYPOINTS)]
        leg_start = navigator.get_clock().now()
        navigator.goToPose(pose(navigator, x, y, heading))
        while not navigator.isTaskComplete():
            elapsed = (navigator.get_clock().now() - leg_start).nanoseconds * 1e-9
            if elapsed > args.leg_timeout:
                navigator.cancelTask()
        seconds = (navigator.get_clock().now() - leg_start).nanoseconds * 1e-9
        if navigator.getResult() == TaskResult.SUCCEEDED:
            reached += 1
            print(f"leg {leg + 1}: reached ({x:.1f}, {y:.1f}) in {seconds:.1f} s", flush=True)
        else:
            failed += 1
            print(f"leg {leg + 1}: FAILED to reach ({x:.1f}, {y:.1f})", flush=True)
        leg += 1
        total = (navigator.get_clock().now() - start).nanoseconds * 1e-9
        rounds = leg // len(WAYPOINTS)
        if leg % len(WAYPOINTS) == 0 and rounds >= args.rounds and total >= args.minutes * 60.0:
            break

    print(
        f"patrol done: {reached} reached, {failed} failed, {total:.0f} s sim time; "
        f"{watch.collisions} collision samples, closest approach {watch.clearance:.2f} m "
        f"at ({watch.where[0]:.1f}, {watch.where[1]:.1f})",
        flush=True,
    )
    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if failed == 0 and watch.collisions == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
