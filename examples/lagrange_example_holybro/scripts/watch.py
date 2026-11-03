#!/usr/bin/env python3
"""Reports the flight: once a second its height above the start, speed, climb and heading.

Its result is the hover the drone takes off into by itself. With once:=true the launch ends
there; else it keeps reporting while you fly it with teleop.
"""

import contextlib
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

HOVER_ABOVE = 1.8  # m above the start: the built-in controller hovers at 2 m
STEADY = 0.1  # m/s: slower than this up or down counts as hovering
EVERY = 1.0  # s of simulated time between reports


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("watch")
        self.declare_parameter("once", False)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.ground: float | None = None
        self.hovering = False
        self.last = -math.inf

    def on_odom(self, odom: Odometry) -> None:
        position = odom.pose.pose.position
        if self.ground is None:
            self.ground = position.z
        height = position.z - self.ground
        twist = odom.twist.twist.linear  # in the body frame
        speed = math.hypot(twist.x, twist.y)
        climb = twist.z
        q = odom.pose.pose.orientation
        heading = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if not self.hovering and height > HOVER_ABOVE and abs(climb) < STEADY:
            self.hovering = True
            self.get_logger().info(
                f"RESULT: hovering {height:.2f} m above the start at x {position.x:.2f}, "
                f"y {position.y:.2f}; fly it with teleop (i forward, t up, b down, j/l turn)"
            )
            if self.get_parameter("once").value:
                raise SystemExit(0)
        if t - self.last >= EVERY:
            self.last = t
            self.get_logger().info(
                f"t {t:6.1f} s  height {height:5.2f} m  speed {speed:4.2f} m/s  "
                f"climb {climb:+5.2f} m/s  heading {heading:+6.1f} deg"
            )


def main() -> None:
    rclpy.init()
    node = Watch()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
