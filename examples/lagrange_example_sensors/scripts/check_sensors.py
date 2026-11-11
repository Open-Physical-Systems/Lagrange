#!/usr/bin/env python3
"""Drives the sample robot up to a wall and checks what its built-in sensor kinds read.

The robot (amr_diffdrive_safety.urdf) carries bumpers, cliff sensors, wheel odometry, a safety
scanner and wheel torque sensors; config/topics.yaml puts their signals on /example/... topics.
The node drives ahead, slows when the scanner's warning field sees the wall and stops when its
protective field does, then compares the wheel odometry with the true distance (/odom), and checks
that the cliff sensor sees the floor, the bumper was never pressed and the wheel torque arrived.
"""

import contextlib
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float64

FAST = 0.5  # m/s, while the warning field is clear
SLOW = 0.2  # m/s, once something is in the warning field
SETTLE = 1.0  # s of sim time standing still before the checks
ODOMETRY_TOLERANCE = 0.05  # wheel odometry within 5 % of the true distance
FLOOR = (0.03, 0.08)  # m: the cliff sensor sits 5.5 cm over the floor


class CheckSensors(Node):
    def __init__(self) -> None:
        super().__init__("check_sensors")
        self.declare_parameter("once", False)
        self.done = False
        self.warning = False
        self.protective = False
        self.warning_at: float | None = None  # m driven (true) when the warning field tripped
        self.stopped_at: float | None = None  # sim time when the protective field tripped
        self.min_distance = math.inf
        self.bumped = False
        self.cliff_range = math.nan
        self.cliff = False
        self.torques: list[float] = []
        self.start: tuple[float, float] | None = None
        self.true_position: tuple[float, float] | None = None
        self.wheel_start: tuple[float, float] | None = None
        self.wheel_position: tuple[float, float] | None = None
        self.command = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.create_subscription(Bool, "/example/safety/warning", self.on_warning, qos)
        self.create_subscription(Bool, "/example/safety/protective", self.on_protective, qos)
        self.create_subscription(Float64, "/example/safety/min_distance", self.on_distance, qos)
        self.create_subscription(Bool, "/example/front_bumper", self.on_bumper, qos)
        self.create_subscription(Range, "/example/cliff_front_left", self.on_cliff_range, qos)
        self.create_subscription(Bool, "/example/cliff_front_left/cliff", self.on_cliff, qos)
        self.create_subscription(PoseStamped, "/example/wheel_odom/pose", self.on_wheel_odom, qos)
        self.create_subscription(Float64, "/example/left_wheel_torque", self.on_torque, qos)

    def on_warning(self, msg: Bool) -> None:
        self.warning = msg.data

    def on_protective(self, msg: Bool) -> None:
        self.protective = msg.data

    def on_distance(self, msg: Float64) -> None:
        self.min_distance = msg.data

    def on_bumper(self, msg: Bool) -> None:
        self.bumped = self.bumped or msg.data

    def on_cliff_range(self, msg: Range) -> None:
        self.cliff_range = msg.range

    def on_cliff(self, msg: Bool) -> None:
        self.cliff = self.cliff or msg.data

    def on_torque(self, msg: Float64) -> None:
        self.torques.append(msg.data)

    def on_wheel_odom(self, msg: PoseStamped) -> None:
        position = (msg.pose.position.x, msg.pose.position.y)
        if self.wheel_start is None:
            self.wheel_start = position
        self.wheel_position = position

    def on_odom(self, odom: Odometry) -> None:
        """Ground truth at the physics rate: the control loop runs here."""
        now = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        position = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        if self.start is None:
            self.start = position
        self.true_position = position
        driven = math.dist(self.start, position)
        if self.warning and self.warning_at is None:
            self.warning_at = driven
        if self.protective and self.stopped_at is None:
            self.stopped_at = now
        twist = TwistStamped()
        twist.header.stamp = odom.header.stamp
        if self.stopped_at is None and self.wheel_start is not None:
            twist.twist.linear.x = SLOW if self.warning else FAST
        self.command.publish(twist)
        if self.stopped_at is not None and now - self.stopped_at >= SETTLE and not self.done:
            self.check(driven)

    def check(self, driven: float) -> None:
        if self.start is None or self.wheel_start is None or self.wheel_position is None:
            return
        wheel = math.dist(self.wheel_start, self.wheel_position)
        problems = []
        if abs(wheel - driven) > ODOMETRY_TOLERANCE * driven:
            problems.append(f"wheel odometry {wheel:.3f} m against {driven:.3f} m driven")
        if not FLOOR[0] <= self.cliff_range <= FLOOR[1] or self.cliff:
            problems.append(f"the cliff sensor reads {self.cliff_range} m (cliff: {self.cliff})")
        if self.bumped:
            problems.append("the front bumper was pressed: the scanner should have stopped it")
        if not self.torques:
            problems.append("no wheel torque arrived")
        if self.warning_at is None:
            problems.append("the warning field never tripped before the protective field")
        if problems:
            self.get_logger().error("FAILED: " + "; ".join(problems))
            self.done = True
            if self.get_parameter("once").value:
                raise SystemExit(1)
            return
        self.finish(
            f"slowed at {self.warning_at:.2f} m and stopped by the safety scanner (nearest return "
            f"{self.min_distance:.2f} m); wheel odometry {wheel:.3f} m for {driven:.3f} m "
            f"driven; cliff sensor {self.cliff_range * 100:.1f} cm over the floor; bumper not "
            f"pressed; {len(self.torques)} wheel torque readings"
        )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = CheckSensors()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
