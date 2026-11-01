#!/usr/bin/env python3
"""Sends joint_trajectory_controller one trajectory and reports how it ended."""

import contextlib

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
TARGET = [0.5, -0.8, 0.9, -0.5, 0.3, 0.2]  # rad
SECONDS = 3


class SendTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("send_trajectory")
        self.declare_parameter("once", False)
        self.done = False
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.sent = False
        self.create_timer(0.5, self.tick)

    def tick(self) -> None:
        if self.sent or not self.client.server_is_ready():
            return
        self.sent = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = TARGET
        point.time_from_start = Duration(sec=SECONDS)
        goal.trajectory.points = [point]
        self.client.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future) -> None:
        handle = future.result()
        self.get_logger().info(f"goal accepted: {handle.accepted}")
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        code = future.result().result.error_code
        self.finish(
            f"trajectory of {SECONDS} s ended with error code {code} "
            f"({'success' if code == FollowJointTrajectory.Result.SUCCESSFUL else 'failed'})"
        )

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = SendTrajectory()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
