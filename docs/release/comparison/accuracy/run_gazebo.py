#!/usr/bin/env python3

"""The accuracy cases on Gazebo Jetty (gz-sim 10), stepped in-process through gz.sim's Python
TestFixture, with the physics engine Gazebo loads by default.

    source /opt/ros/lyrical/setup.bash
    run_gazebo.py SCENE_DIR OUT.json        (SCENE_DIR from scenes.py)
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import drive
import gz.math  # noqa: F401  (registers Pose3d for world_pose)
import scenes
from gz.msgs.contacts_pb2 import Contacts
from gz.sim import Joint, Link, Model, TestFixture, World, world_entity
from gz.transport import Node

DURATION = {
    "freefall": scenes.FALL_T,
    "pendulum_small": scenes.PEND_PERIOD_T,
    "pendulum_large": scenes.PEND_ENERGY_T,
    "slope_slide": scenes.SLOPE_T,
    "slope_static": scenes.SLOPE_T,
    "rest": scenes.REST_T,
}


class GazeboRun:
    def __init__(self, case: str) -> None:
        self.name = Path(case).name
        self.fixture = TestFixture(f"{case}.sdf")
        self.state: dict[str, Any] = {}
        self.history: list[dict[str, Any]] = []
        self.contact_fz: dict[int, float] = {}  # by sim time (ms) of the contact message
        if self.name == "rest":
            self.node = Node()
            self.node.subscribe(
                Contacts, "/world/rest/model/box/link/box/sensor/contact/contact", self._contacts
            )
        self.ready = False
        self.fixture.on_post_update(self._post)
        self.fixture.finalize()
        self.server = self.fixture.server()

    def _contacts(self, msg: Any) -> None:
        stamp = msg.header.stamp.sec * 1000 + round(msg.header.stamp.nsec / 1e6)
        self.contact_fz[stamp] = sum(
            abs(w.body_1_wrench.force.z) for c in msg.contact for w in c.wrench
        )

    def _post(self, info: Any, ecm: Any) -> None:
        world = World(world_entity(ecm))
        if self.name.startswith("pendulum"):
            model = Model(world.model_by_name(ecm, "pendulum"))
            link = Link(model.link_by_name(ecm, "bob"))
            joint = Joint(model.joint_by_name(ecm, "hinge"))
            if not self.ready:
                joint.enable_position_check(ecm, True)
                joint.enable_velocity_check(ecm, True)
            q, w = joint.position(ecm), joint.velocity(ecm)
            if q and w:
                self.state["joint"] = (q[0], w[0])
        else:
            model = Model(world.model_by_name(ecm, "box"))
            link = Link(model.link_by_name(ecm, "box"))
        self.ready = True
        pose = link.world_pose(ecm)
        if pose is not None:
            self.state["pos"] = [pose.pos().x(), pose.pos().y(), pose.pos().z()]
        if not info.paused:
            self.history.append(dict(self.state))

    def step_to(self, t: float) -> None:
        # One server.run per call costs milliseconds of synchronisation, so the whole case runs
        # in one call the first time, every step recorded, and later calls only move the cursor.
        if not self.history:
            self.server.run(True, round(DURATION[self.name] / scenes.DT), False)
        self.state = self.history[round(t / scenes.DT) - 1]
        self.t = t

    def position(self) -> list[float]:
        return list(self.state["pos"])

    def joint(self) -> tuple[float, float]:
        # The joint's position and velocity components exist from the second step on (the
        # first update asks for them); at the first step the hinge is still at rest at 0.
        return self.state.get("joint", (0.0, 0.0))

    def contact_force(self) -> float | None:
        time.sleep(0.2)  # let the last transport messages arrive
        return self.contact_fz.get(round(self.t * 1000))


def main() -> int:
    scene_dir, out = sys.argv[1], sys.argv[2]
    results = {"Gazebo Jetty (gz-sim 10.5.0)": drive.measure(GazeboRun, scene_dir)}
    Path(out).write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
