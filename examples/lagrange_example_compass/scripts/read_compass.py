#!/usr/bin/env python3
"""Reads what the battery leads do to the compass once the quadcopter hovers."""

import contextlib
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

HOVERING = 2.0  # A in one motor's lead: off the ground and flying
SETTLE = 5.0  # s after that: past the climb, holding its hover


class ReadCompass(Node):
    def __init__(self) -> None:
        super().__init__("read_compass")
        self.declare_parameter("once", False)
        self.done = False
        self.now: dict[str, float] = {}
        self.flying_since: float | None = None
        for name in ("heading_error", "interference", "lead_current"):
            self.create_subscription(
                Float64, "/compass/" + name, lambda m, n=name: self.on_value(n, m), 10
            )

    def on_value(self, name: str, message: Float64) -> None:
        self.now[name] = message.data
        if self.done or len(self.now) < 3 or self.now["lead_current"] < HOVERING:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.flying_since is None:
            self.flying_since = now
        if now - self.flying_since < SETTLE:
            return
        share = 100.0 * self.now["interference"]
        verdict = "fine" if share < 30 else "move the compass" if share > 60 else "grey zone"
        self.finish(
            f"{4 * self.now['lead_current']:.1f} A in the leads: heading off by "
            f"{math.degrees(self.now['heading_error']):+.2f} deg, disturbance {share:.1f} % of "
            f"the Earth's field ({verdict} by ArduPilot's compassmot bands)"
        )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ReadCompass()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
