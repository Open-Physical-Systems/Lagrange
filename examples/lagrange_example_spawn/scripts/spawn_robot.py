#!/usr/bin/env python3
"""Spawns a robot, lists what is in the run, and deletes it, by simulation_interfaces services."""

import contextlib

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import DeleteEntity, GetEntities, SpawnEntity


class SpawnRobot(Node):
    def __init__(self) -> None:
        super().__init__("spawn_robot")
        self.declare_parameter("once", False)
        self.done = False
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.entities = self.create_client(GetEntities, "/get_entities")
        self.delete = self.create_client(DeleteEntity, "/delete_entity")
        self.step = 0
        self.future = None
        self.name = ""
        self.before = 0
        self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        if self.done:
            return
        if self.future is not None:
            if not self.future.done():
                return
            answer = self.future.result()
            self.future = None
            if answer.result.result != Result.RESULT_OK:
                raise RuntimeError(f"step {self.step}: {answer.result.error_message}")
            if self.step == 1:
                self.before = len(answer.entities)
                self.get_logger().info(f"before: {answer.entities}")
            elif self.step == 2:
                self.name = answer.entity_name
                self.get_logger().info(f"spawned {self.name}")
            elif self.step == 3:
                self.get_logger().info(f"with it: {answer.entities}")
                self.during = len(answer.entities)
            elif self.step == 5:
                self.finish(
                    f"entities {self.before} -> {self.during} with {self.name} -> "
                    f"{len(answer.entities)} after deleting it"
                )
                return
        if not self.spawn.service_is_ready() or not self.entities.service_is_ready():
            return
        self.step += 1
        if self.step in (1, 3, 5):
            self.future = self.entities.call_async(GetEntities.Request())
        elif self.step == 2:
            request = SpawnEntity.Request()
            request.name = "visitor"
            request.allow_renaming = True
            assets = get_package_share_directory("lagrange_assets")
            request.entity_resource.uri = f"{assets}/robots/amr_diffdrive/amr_diffdrive.urdf"
            self.future = self.spawn.call_async(request)
        elif self.step == 4:
            request = DeleteEntity.Request()
            request.entity = self.name
            self.future = self.delete.call_async(request)

    def finish(self, text: str) -> None:
        """The result. With once:=true the launch ends here; else the run stays up to look at."""
        self.get_logger().info("RESULT: " + text)
        self.done = True
        if self.get_parameter("once").value:
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = SpawnRobot()
    with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException, SystemExit):
        rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
