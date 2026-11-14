#!/usr/bin/env python3

"""Runs the 200-box pile for SECONDS and prints a SHA-256 of every box's final position and
orientation (as the exact bits of the doubles), for docs/comparison.md's determinism check: run
it twice, in two processes; the same digest means bit-identical trajectories' end states.

    final_poses.py lagrange-mujoco|lagrange-jolt|mujoco|pybullet|gazebo BOXES_DIR SECONDS

lagrange-*: the system Python with Lagrange sourced; gazebo: with /opt/ros/lyrical sourced;
mujoco and pybullet: the virtual environment with those packages.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import sys
from pathlib import Path


def digest(values: list[float]) -> str:
    return hashlib.sha256(b"".join(struct.pack("<d", v) for v in values)).hexdigest()


def lagrange_poses(folder: Path, seconds: float, backend: str) -> list[float]:
    import lagrange

    profile = folder / ("profile_boxes.yaml" if backend == "mujoco" else "profile_boxes_jolt.yaml")
    sim = lagrange.Simulation(str(folder / "boxes.xml"), world=None, profile=str(profile))
    sim.step_until(seconds)
    out: list[float] = []
    for n in range(200):
        out += sim.read(f"body/box{n}/pose")
    return out


def mujoco_poses(folder: Path, seconds: float) -> list[float]:
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(folder / "boxes.xml"))
    d = mujoco.MjData(m)
    for _ in range(round(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)
    return [float(x) for x in d.qpos]


def pybullet_poses(folder: Path, seconds: float) -> list[float]:
    import pybullet as p

    spec = json.loads((folder / "boxes.json").read_text())
    c = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81, physicsClientId=c)
    p.setTimeStep(0.001, physicsClientId=c)
    ground = p.createCollisionShape(p.GEOM_BOX, halfExtents=[20, 20, 0.1], physicsClientId=c)
    p.createMultiBody(0, ground, -1, [0, 0, -0.1], physicsClientId=c)
    h = spec["size"] / 2
    shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h, h, h], physicsClientId=c)
    bodies = []
    for x, y, z, yaw in spec["boxes"]:
        b = p.createMultiBody(
            spec["mass"],
            shape,
            -1,
            [x, y, z],
            [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)],
            physicsClientId=c,
        )
        p.changeDynamics(b, -1, linearDamping=0, angularDamping=0, physicsClientId=c)
        bodies.append(b)
    for _ in range(round(seconds / 0.001)):
        p.stepSimulation(physicsClientId=c)
    out: list[float] = []
    for b in bodies:
        pos, quat = p.getBasePositionAndOrientation(b, physicsClientId=c)
        out += list(pos) + list(quat)
    return out


def gazebo_poses(folder: Path, seconds: float) -> list[float]:
    import gz.math  # noqa: F401  (registers Pose3d)
    from gz.sim import Link, Model, TestFixture, World, world_entity

    final: dict[str, list[float]] = {}

    def post(info: object, ecm: object) -> None:
        world = World(world_entity(ecm))
        for n in range(200):
            model = Model(world.model_by_name(ecm, f"box{n}"))
            pose = Link(model.link_by_name(ecm, "l")).world_pose(ecm)
            if pose is not None:
                q = pose.rot()
                final[f"box{n}"] = [
                    pose.pos().x(),
                    pose.pos().y(),
                    pose.pos().z(),
                    q.w(),
                    q.x(),
                    q.y(),
                    q.z(),
                ]

    fixture = TestFixture(str(folder / "boxes.sdf"))
    fixture.on_post_update(post)
    fixture.finalize()
    fixture.server().run(True, round(seconds / 0.001), False)
    out: list[float] = []
    for n in range(200):
        out += final[f"box{n}"]
    return out


def main() -> int:
    which, folder, seconds = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3])
    if which.startswith("lagrange-"):
        values = lagrange_poses(folder, seconds, which.removeprefix("lagrange-"))
    elif which == "mujoco":
        values = mujoco_poses(folder, seconds)
    elif which == "pybullet":
        values = pybullet_poses(folder, seconds)
    else:
        values = gazebo_poses(folder, seconds)
    print(
        json.dumps(
            {
                "engine": which,
                "seconds": seconds,
                "values": len(values),
                "sha256": digest(values),
                "first_box": values[:7],
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
