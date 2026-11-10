#!/usr/bin/env python3
"""Reads the LED's, the board's hottest and the heat sink's temperatures as they warm up."""

import contextlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Temperature

WARM = 15.0  # °C over the start: warm enough to tell where the heat goes


class ReadHeat(Node):
    def __init__(self) -> None:
        super().__init__("read_heat")
        self.declare_parameter("once", False)
        self.done = False
        self.now: dict[str, float] = {}
        self.start: dict[str, float] = {}
        for name in ("led_left", "board_max", "sink"):
            self.create_subscription(
                Temperature, "/heat/" + name, lambda m, n=name: self.on_temperature(n, m), 10
            )

    def on_temperature(self, name: str, message: Temperature) -> None:
        self.start.setdefault(name, message.temperature)
        self.now[name] = message.temperature
        if self.done or len(self.now) < 3:
            return
        rise = self.now["led_left"] - self.start["led_left"]
        if rise >= WARM:
            self.finish(
                f"LED {self.now['led_left']:.1f} °C, board's hottest {self.now['board_max']:.1f}"
                f" °C, heat sink {self.now['sink']:.1f} °C: the LED is the hot spot and the sink"
                f" {'warmer' if self.now['sink'] > self.start['sink'] else 'not yet warmer'}"
            )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ReadHeat()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
