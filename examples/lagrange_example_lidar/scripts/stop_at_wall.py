#!/usr/bin/env python3
"""Drives straight ahead until the lidar sees something within STOP_AT in front, then stops."""

import contextlib
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

STOP_AT = 1.0  # m
SPEED = 0.6  # m/s
AHEAD = math.radians(15.0)  # half the width of what counts as in front


class StopAtWall(Node):
    def __init__(self) -> None:
        super().__init__("stop_at_wall")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan) -> None:
        nearest = math.inf
        for i, distance in enumerate(scan.ranges):
            angle = math.remainder(scan.angle_min + i * scan.angle_increment, math.tau)
            if abs(angle) <= AHEAD and scan.range_min < distance < scan.range_max:
                nearest = min(nearest, distance)
        twist = TwistStamped()
        twist.header.stamp = scan.header.stamp
        if nearest > STOP_AT and not self.done:
            twist.twist.linear.x = SPEED
        self.command.publish(twist)
        if nearest <= STOP_AT and not self.done:
            self.finish(f"stopped with something {nearest:.2f} m ahead")

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = StopAtWall()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
