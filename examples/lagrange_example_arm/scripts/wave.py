#!/usr/bin/env python3
"""Sends the arm to a few poses by /joint_commands and checks /joint_states reaches each."""

import contextlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint_1", "joint_2", "joint_3"]
POSES = [[0.8, 0.5, -0.6], [-0.8, 0.9, 0.4], [0.0, 0.2, 0.2]]  # rad
TOLERANCE = 0.05  # rad: a position loop holding against gravity sags slightly


class Wave(Node):
    def __init__(self) -> None:
        super().__init__("wave")
        self.declare_parameter("once", False)
        self.done = False
        self.command = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)
        self.pose = 0
        self.worst = 0.0

    def on_states(self, states: JointState) -> None:
        if self.done:
            return
        at = dict(zip(states.name, states.position, strict=True))
        if not all(joint in at for joint in JOINTS):
            return
        target = POSES[self.pose]
        command = JointState()
        command.header.stamp = states.header.stamp
        command.name = JOINTS
        command.position = target
        self.command.publish(command)
        error = max(abs(at[j] - t) for j, t in zip(JOINTS, target, strict=True))
        if error < TOLERANCE:
            self.worst = max(self.worst, error)
            self.get_logger().info(f"pose {self.pose + 1} reached, within {error:.3f} rad")
            self.pose += 1
            if self.pose == len(POSES):
                self.finish(f"reached {len(POSES)} poses, within {self.worst:.3f} rad")
                self.pose = 0

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = Wave()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
