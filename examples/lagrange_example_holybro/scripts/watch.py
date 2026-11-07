#!/usr/bin/env python3
"""Reports the flight: once a second its height above the start, speed, climb and heading.

The drone waits on the ground until /cmd_vel asks it to climb. With once:=true this node asks
(it climbs at 1 m/s), and the launch ends when it is up; else you fly it with teleop and the
node keeps reporting.
"""

import contextlib
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

UP = 1.5  # m above the start: once:=true climbs this far
CLIMB = 1.0  # m/s
EVERY = 1.0  # s of simulated time between reports


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("watch")
        self.declare_parameter("once", False)
        self.once = bool(self.get_parameter("once").value)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.ground: float | None = None
        self.last = -math.inf

    def on_odom(self, odom: Odometry) -> None:
        position = odom.pose.pose.position
        if self.ground is None:
            self.ground = position.z
            if not self.once:
                self.get_logger().info(
                    "on the ground: press t in the teleop window to take off "
                    "(i forward, j/l turn, b down and land)"
                )
        height = position.z - self.ground
        twist = odom.twist.twist.linear  # in the body frame
        speed = math.hypot(twist.x, twist.y)
        q = odom.pose.pose.orientation
        heading = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2)))
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        if self.once:
            climb = TwistStamped()
            climb.header.stamp = odom.header.stamp
            climb.twist.linear.z = CLIMB
            self.command.publish(climb)
            if height >= UP:
                self.get_logger().info(
                    f"RESULT: took off on /cmd_vel and climbed to {height:.2f} m above the start"
                )
                raise SystemExit(0)
        if t - self.last >= EVERY:
            self.last = t
            self.get_logger().info(
                f"t {t:6.1f} s  height {height:5.2f} m  speed {speed:4.2f} m/s  "
                f"climb {twist.z:+5.2f} m/s  heading {heading:+6.1f} deg"
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
