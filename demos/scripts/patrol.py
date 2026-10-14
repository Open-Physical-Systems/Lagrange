#!/usr/bin/env python3
# Sends the AMR of amr_warehouse.launch.py around the warehouse aisles and reports each leg.
#
#   ros2 run lagrange_demos patrol                    # one round
#   ros2 run lagrange_demos patrol --rounds 3
#   ros2 run lagrange_demos patrol --minutes 10       # keep patrolling for 10 simulated minutes
#
# Exits non-zero if a waypoint was missed (tools/ci/nav2_demo.sh runs it nightly).
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

# (x, y, heading) in the map frame: around the pallet, along both racks and back.
WAYPOINTS = [
    (4.0, -1.5, 0.0),
    (7.5, 0.0, math.pi / 2),
    (4.0, 2.0, math.pi),
    (-7.5, 2.0, math.pi),
    (-7.5, -2.0, 0.0),
    (0.0, -1.5, 0.0),
]


def pose(navigator: BasicNavigator, x: float, y: float, heading: float) -> PoseStamped:
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(heading / 2.0)
    goal.pose.orientation.w = math.cos(heading / 2.0)
    return goal


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
    navigator.waitUntilNav2Active(localizer="map_server")

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

    print(f"patrol done: {reached} reached, {failed} failed, {total:.0f} s sim time", flush=True)
    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
