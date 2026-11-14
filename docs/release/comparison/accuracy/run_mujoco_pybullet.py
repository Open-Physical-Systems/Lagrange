#!/usr/bin/env python3

"""The accuracy cases on standalone MuJoCo and PyBullet (pip packages, in a virtual environment).

    python3 -m venv venv && venv/bin/pip install mujoco pybullet
    venv/bin/python run_mujoco_pybullet.py SCENE_DIR OUT.json

MuJoCo reads the same MJCF as Lagrange, with its own defaults. PyBullet builds the same bodies
through its API: its linear and angular damping (0.04 by default) are set to 0 and a hinge's
default velocity motor is switched off, since the cases are undamped; Bullet multiplies the two
surfaces' friction coefficients, so each surface gets sqrt(0.3) for a pair coefficient of 0.3.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco

try:
    import pybullet as p
except ImportError:  # a MuJoCo-only environment (e.g. MuJoCo 3.12, the version Lagrange links)
    p = None

import drive
import scenes


class MujocoRun:
    def __init__(self, xml: str) -> None:
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        self.n = 0
        name = "bob" if mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "bob") >= 0 else "box"
        self.body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, name)

    def step_to(self, t: float) -> None:
        while self.n < round(t / scenes.DT):
            mujoco.mj_step(self.m, self.d)
            self.n += 1

    def position(self) -> list[float]:
        # mj_step leaves xpos at the start of the step it took: bring it to the new qpos.
        mujoco.mj_kinematics(self.m, self.d)
        return [float(x) for x in self.d.xpos[self.body]]

    def joint(self) -> tuple[float, float]:
        return float(self.d.qpos[0]), float(self.d.qvel[0])

    def contact_force(self) -> float | None:
        import numpy as np

        mujoco.mj_forward(self.m, self.d)  # contacts and their forces at the new state
        total = 0.0
        buf = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            mujoco.mj_contactForce(self.m, self.d, i, buf)
            frame = np.array(c.frame).reshape(3, 3)  # rows: normal, tangent 1, tangent 2
            world = frame.T @ buf[:3]
            total += abs(world[2])
        return total


def _quat_y(angle: float) -> list[float]:
    return [0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2)]


class BulletRun:
    MU = math.sqrt(scenes.SLOPE_MU)

    def __init__(self, case: str) -> None:
        self.c = p.connect(p.DIRECT)
        p.setGravity(0, 0, -scenes.G, physicsClientId=self.c)
        p.setTimeStep(scenes.DT, physicsClientId=self.c)
        self.n = 0
        self.link = -1
        name = Path(case).name
        h = scenes.CUBE / 2
        if name == "freefall":
            self.body = self._cube(0.05, 1.0, [0, 0, scenes.FALL_Z0], 0.0)
        elif name.startswith("pendulum"):
            a = scenes.PEND_SMALL if name == "pendulum_small" else scenes.PEND_LARGE
            i = scenes.sphere_inertia()
            self.body = p.createMultiBody(
                baseMass=0,
                basePosition=[0, 0, scenes.PEND_PIVOT_Z],
                linkMasses=[scenes.PEND_M],
                linkCollisionShapeIndices=[-1],
                linkVisualShapeIndices=[-1],
                linkPositions=[[0, 0, 0]],
                linkOrientations=[_quat_y(a)],
                linkInertialFramePositions=[[0, 0, -scenes.PEND_L]],
                linkInertialFrameOrientations=[[0, 0, 0, 1]],
                linkParentIndices=[0],
                linkJointTypes=[p.JOINT_REVOLUTE],
                linkJointAxis=[[0, 1, 0]],
                physicsClientId=self.c,
            )
            self.link = 0
            p.changeDynamics(
                self.body,
                0,
                localInertiaDiagonal=[i, i, i],
                linearDamping=0,
                angularDamping=0,
                jointDamping=0,
                physicsClientId=self.c,
            )
            p.setJointMotorControl2(
                self.body, 0, p.VELOCITY_CONTROL, force=0, physicsClientId=self.c
            )
        elif name.startswith("slope"):
            deg = scenes.SLOPE_SLIDE_DEG if name == "slope_slide" else scenes.SLOPE_STATIC_DEG
            th = math.radians(deg)
            slab = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[10, 1, 0.1], physicsClientId=self.c
            )
            s = p.createMultiBody(
                0, slab, -1, [0, 0, scenes.SLAB_Z], _quat_y(th), physicsClientId=self.c
            )
            p.changeDynamics(s, -1, lateralFriction=self.MU, physicsClientId=self.c)
            up = 0.1 + h
            self.body = self._cube(
                h, 1.0, [up * math.sin(th), 0, scenes.SLAB_Z + up * math.cos(th)], th, self.MU
            )
        else:  # rest
            slab = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[2, 2, 0.1], physicsClientId=self.c
            )
            p.createMultiBody(0, slab, -1, [0, 0, scenes.SLAB_Z], physicsClientId=self.c)
            self.body = self._cube(h, scenes.REST_M, [0, 0, scenes.SLAB_Z + 0.1 + h], 0.0)

    def _cube(
        self, h: float, m: float, pos: list[float], th: float, mu: float | None = None
    ) -> int:
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h, h, h], physicsClientId=self.c)
        b = p.createMultiBody(m, shape, -1, pos, _quat_y(th), physicsClientId=self.c)
        p.changeDynamics(b, -1, linearDamping=0, angularDamping=0, physicsClientId=self.c)
        if mu is not None:
            p.changeDynamics(b, -1, lateralFriction=mu, physicsClientId=self.c)
        return b

    def step_to(self, t: float) -> None:
        while self.n < round(t / scenes.DT):
            p.stepSimulation(physicsClientId=self.c)
            self.n += 1

    def position(self) -> list[float]:
        if self.link >= 0:
            return list(p.getLinkState(self.body, 0, physicsClientId=self.c)[4])
        return list(p.getBasePositionAndOrientation(self.body, physicsClientId=self.c)[0])

    def joint(self) -> tuple[float, float]:
        s = p.getJointState(self.body, 0, physicsClientId=self.c)
        return s[0], s[1]

    def contact_force(self) -> float | None:
        return sum(
            abs(c[9] * c[7][2]) for c in p.getContactPoints(bodyA=self.body, physicsClientId=self.c)
        )


def _pybullet_version() -> str:
    from importlib.metadata import version

    return version("pybullet")


def main() -> int:
    scene_dir, out = sys.argv[1], sys.argv[2]
    results = {
        f"MuJoCo {mujoco.__version__} (standalone)": drive.measure(
            lambda case: MujocoRun(f"{case}.xml"), scene_dir
        ),
    }
    if p is not None:
        results[f"PyBullet {_pybullet_version()} (Bullet)"] = drive.measure(BulletRun, scene_dir)
    Path(out).write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
