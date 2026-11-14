#!/usr/bin/env python3

"""The 200-box pile on standalone MuJoCo or PyBullet, timed like bench_lagrange.py.

    bench_standalone.py mujoco|mujoco-lagrange|pybullet BOXES_DIR SECONDS

mujoco: boxes.xml with MuJoCo's defaults (pyramidal cone, Euler); mujoco-lagrange: with the
options Lagrange's MuJoCo backend sets (elliptic cone, implicitfast); pybullet: the same boxes
(boxes.json) through PyBullet's API, its damping set to 0.
"""

import time

T0 = time.perf_counter()

import json  # noqa: E402
import math  # noqa: E402
import resource  # noqa: E402
import sys  # noqa: E402
from importlib.metadata import version  # noqa: E402
from pathlib import Path  # noqa: E402


def cpu() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return r.ru_utime + r.ru_stime


def main() -> int:
    which, folder, seconds = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3])
    dt = 0.001
    if which.startswith("mujoco"):
        import mujoco

        m = mujoco.MjModel.from_xml_path(str(folder / "boxes.xml"))
        if which == "mujoco-lagrange":
            m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
            m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        d = mujoco.MjData(m)

        def step() -> None:
            mujoco.mj_step(m, d)

        label = f"MuJoCo {mujoco.__version__}" + (
            " (Lagrange's options)" if which == "mujoco-lagrange" else ""
        )
    else:
        import pybullet as p

        spec = json.loads((folder / "boxes.json").read_text())
        c = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=c)
        p.setTimeStep(dt, physicsClientId=c)
        ground = p.createCollisionShape(p.GEOM_BOX, halfExtents=[20, 20, 0.1], physicsClientId=c)
        p.createMultiBody(0, ground, -1, [0, 0, -0.1], physicsClientId=c)
        h = spec["size"] / 2
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[h, h, h], physicsClientId=c)
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

        def step() -> None:
            p.stepSimulation(physicsClientId=c)

        label = f"PyBullet {version('pybullet')}"
    step()
    t_first = time.perf_counter()
    c0 = cpu()
    for _ in range(round(seconds / dt) - 1):
        step()
    t_end = time.perf_counter()
    stepping = t_end - t_first
    print(
        json.dumps(
            {
                "sim": label,
                "startup_s": t_first - T0,
                "stepping_s": stepping,
                "rtf": (seconds - dt) / stepping,
                "cpu_cores": (cpu() - c0) / stepping,
                "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
