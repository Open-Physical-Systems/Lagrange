#!/usr/bin/env python3
"""Reports how far the lidar sees, over some scans."""

import contextlib
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SCANS = 30


class ReadRange(Node):
    def __init__(self) -> None:
        super().__init__("read_range")
        self.declare_parameter("once", False)
        self.done = False
        self.scans = 0
        self.farthest = 0.0
        self.hits = 0
        self.rays = 0
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan) -> None:
        if self.done:
            return
        for distance in scan.ranges:
            self.rays += 1
            if math.isfinite(distance) and scan.range_min < distance < scan.range_max:
                self.hits += 1
                self.farthest = max(self.farthest, distance)
        self.scans += 1
        if self.scans == SCANS:
            self.finish(
                f"farthest return {self.farthest:.1f} m, "
                f"{100.0 * self.hits / self.rays:.0f} % of the rays came back"
            )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ReadRange()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
