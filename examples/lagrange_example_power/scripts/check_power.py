#!/usr/bin/env python3
"""Reads the bus at rest against the hand formula, then spins the motor and sees it sag."""

import contextlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

OPEN_CIRCUIT = 24.0  # V, in robot/power.cir
INSIDE = 0.1  # ohm
LAMP = 12.0  # ohm
AT_REST = OPEN_CIRCUIT * LAMP / (LAMP + INSIDE)
TORQUE = 15.0  # N m at the joint


class CheckPower(Node):
    def __init__(self) -> None:
        super().__init__("check_power")
        self.declare_parameter("once", False)
        self.done = False
        self.volts = None
        self.amps = None
        self.rest = None
        self.ticks = 0
        self.lowest = float("inf")
        self.highest = 0.0
        self.create_subscription(Float64, "/power/bus_voltage", self.on_volts, 10)
        self.create_subscription(Float64, "/power/battery_current", self.on_amps, 10)
        self.command = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_timer(0.1, self.tick)

    def on_volts(self, message: Float64) -> None:
        self.volts = message.data

    def on_amps(self, message: Float64) -> None:
        self.amps = message.data

    def tick(self) -> None:
        if self.done or self.volts is None or self.amps is None:
            return
        self.ticks += 1
        if self.rest is None and self.ticks >= 5:
            self.rest = self.volts
            self.get_logger().info(
                f"at rest: bus {self.rest:.3f} V, the formula says {AT_REST:.3f} V"
            )
        if self.rest is None:
            return
        spin = JointState()
        spin.name = ["spin"]
        spin.effort = [TORQUE]
        self.command.publish(spin)
        # With nothing to push against it soon runs at the speed its voltage allows, and draws
        # nothing more: the sag is while it speeds up.
        self.lowest = min(self.lowest, self.volts)
        self.highest = max(self.highest, self.amps)
        if self.ticks == 30:
            self.finish(
                f"bus {self.rest:.2f} V at rest (formula {AT_REST:.2f} V), "
                f"down to {self.lowest:.2f} V at {self.highest:.1f} A while the motor sped up"
            )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = CheckPower()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
