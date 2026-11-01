#!/usr/bin/env python3
"""Waits for one of each image of the front camera and reports what is in them."""

import contextlib

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CAMERA = "/front_camera"


class ReadCamera(Node):
    def __init__(self) -> None:
        super().__init__("read_camera")
        self.declare_parameter("once", False)
        self.done = False
        self.colour = None
        self.depth = None
        self.instance = None
        qos = qos_profile_sensor_data
        self.create_subscription(Image, CAMERA + "/image_raw", self.on_colour, qos)
        self.create_subscription(Image, CAMERA + "/depth/image_raw", self.on_depth, qos)
        self.create_subscription(Image, CAMERA + "/instance/image_raw", self.on_instance, qos)

    def on_colour(self, image: Image) -> None:
        self.colour = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
        self.report()

    def on_depth(self, image: Image) -> None:
        self.depth = np.frombuffer(image.data, np.float32).reshape(image.height, image.width)
        self.report()

    def on_instance(self, image: Image) -> None:
        self.instance = np.frombuffer(image.data, np.int32).reshape(image.height, image.width)
        self.report()

    def report(self) -> None:
        if self.done or self.colour is None or self.depth is None or self.instance is None:
            return
        seen = np.unique(self.instance)
        things = int((seen >= 0).sum())
        finite = self.depth[np.isfinite(self.depth)]
        height, width = self.depth.shape
        self.finish(
            f"{width}x{height} image, mean colour {self.colour.mean():.0f}/255, depth "
            f"{finite.min():.1f} to {finite.max():.1f} m, {things} different things in view"
        )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ReadCamera()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
