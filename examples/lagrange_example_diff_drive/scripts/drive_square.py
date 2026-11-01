#!/usr/bin/env python3
"""Drives the robot along the sides of a square by /cmd_vel, watching /odom."""

import contextlib
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

SIDE = 1.5  # m
SPEED = 0.5  # m/s
TURN_RATE = 0.8  # rad/s
SIDES = 4


def yaw_of(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveSquare(Node):
    def __init__(self) -> None:
        super().__init__("drive_square")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.side = 0
        self.turning = False
        self.start = None
        self.start_yaw = 0.0

    def on_odom(self, odom: Odometry) -> None:
        if self.done:
            return
        position = odom.pose.pose.position
        yaw = yaw_of(odom)
        if self.start is None:
            self.start = (position.x, position.y)
            self.start_yaw = yaw
        twist = TwistStamped()
        twist.header.stamp = odom.header.stamp
        if self.turning:
            turned = abs(math.remainder(yaw - self.start_yaw, math.tau))
            if turned >= math.pi / 2.0 - 0.02:
                self.turning = False
                self.start = (position.x, position.y)
            else:
                twist.twist.angular.z = TURN_RATE
        else:
            gone = math.hypot(position.x - self.start[0], position.y - self.start[1])
            if gone >= SIDE:
                self.side += 1
                self.get_logger().info(
                    f"side {self.side} done at x={position.x:.2f} y={position.y:.2f}"
                )
                if self.side == SIDES:
                    self.command.publish(TwistStamped())
                    self.finish(
                        f"drove {SIDES} sides of {SIDE} m, ended "
                        f"{math.hypot(position.x, position.y):.2f} m from where it started"
                    )
                    return
                self.turning = True
                self.start_yaw = yaw
            else:
                twist.twist.linear.x = SPEED
        self.command.publish(twist)

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = DriveSquare()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
