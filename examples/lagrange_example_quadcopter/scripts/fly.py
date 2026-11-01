#!/usr/bin/env python3
"""Waits for the hover, flies forward for a while and reports how far it got and how high."""

import contextlib

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

HOVER_ABOVE = 1.5  # m above the start: it hovers at 2 m
FORWARD = 1.0  # m/s
DISTANCE = 4.0  # m


class Fly(Node):
    def __init__(self) -> None:
        super().__init__("fly")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.ground = None
        self.start_x = None

    def on_odom(self, odom: Odometry) -> None:
        position = odom.pose.pose.position
        if self.ground is None:
            self.ground = position.z
        height = position.z - self.ground
        if self.start_x is None:
            if height < HOVER_ABOVE:
                return
            self.start_x = position.x
            self.get_logger().info(f"hovering {height:.2f} m up, flying forward")
        twist = TwistStamped()
        twist.header.stamp = odom.header.stamp
        gone = position.x - self.start_x
        if gone < DISTANCE and not self.done:
            twist.twist.linear.x = FORWARD
        self.command.publish(twist)
        if gone >= DISTANCE and not self.done:
            self.finish(f"flew {gone:.2f} m forward at {height:.2f} m above the start")

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = Fly()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
