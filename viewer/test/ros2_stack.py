"""A small user stack for test_ros2_tabs: two action servers and an action client."""

import contextlib

import rclpy
from control_msgs.action import FollowJointTrajectory
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    server = Node("arm_controller")
    ActionServer(
        server,
        FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory",
        lambda goal: FollowJointTrajectory.Result(),
    )
    ActionServer(server, Fibonacci, "/fibonacci", lambda goal: Fibonacci.Result())
    client = Node("mission_planner")
    ActionClient(client, Fibonacci, "/fibonacci")
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
        executor.spin()


if __name__ == "__main__":
    main()
