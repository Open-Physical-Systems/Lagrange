#!/usr/bin/env python3
"""Asks for a speed by /cmd_vel and reports the speed reached and the way gone."""

import contextlib
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

SPEED = 4.0  # m/s
DISTANCE = 12.0  # m


class DriveCar(Node):
    def __init__(self) -> None:
        super().__init__("drive_car")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.start = None

    def on_odom(self, odom: Odometry) -> None:
        position = odom.pose.pose.position
        if self.start is None:
            self.start = (position.x, position.y)
        gone = math.hypot(position.x - self.start[0], position.y - self.start[1])
        twist = TwistStamped()
        twist.header.stamp = odom.header.stamp
        if gone < DISTANCE and not self.done:
            twist.twist.linear.x = SPEED
        self.command.publish(twist)
        if gone >= DISTANCE and not self.done:
            velocity = odom.twist.twist.linear
            speed = math.hypot(velocity.x, velocity.y)
            self.finish(f"went {gone:.1f} m, at {speed:.2f} m/s (asked {SPEED:.2f})")

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = DriveCar()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
