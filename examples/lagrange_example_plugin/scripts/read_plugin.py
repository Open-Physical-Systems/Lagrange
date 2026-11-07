#!/usr/bin/env python3
"""Reads what the plugin writes."""

import contextlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

SAMPLES = 300  # at 100 Hz: three seconds


class ReadPlugin(Node):
    def __init__(self) -> None:
        super().__init__("read_plugin")
        self.declare_parameter("once", False)
        self.done = False
        self.samples = 0
        self.first = None
        self.create_subscription(Float64, "/distance", self.on_distance, 100)

    def on_distance(self, message: Float64) -> None:
        if self.done:
            return
        if self.first is None:
            self.first = message.data
        self.samples += 1
        if self.samples == SAMPLES:
            self.finish(
                f"{SAMPLES} samples from the plugin: {self.first:.3f} m, then {message.data:.3f} m"
            )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ReadPlugin()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
