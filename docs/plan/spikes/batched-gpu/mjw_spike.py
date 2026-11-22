"""RFC 0025 spike: MuJoCo Warp steps per second for the sample arm and quadcopter.

Not part of any package: nothing in the repository depends on MuJoCo Warp. Run:

    ~/.local/bin/uv venv scratch/venv-mjw
    VIRTUAL_ENV=scratch/venv-mjw ~/.local/bin/uv pip install mujoco-warp==4.0.1 matplotlib
    scratch/venv-mjw/bin/python docs/plan/spikes/batched-gpu/mjw_spike.py \
        assets/robots/arm6/arm6.urdf assets/robots/quad_x500/quad_x500.urdf OUT.json
    scratch/venv-mjw/bin/python docs/plan/spikes/batched-gpu/plot.py OUT.json OUT.png

Physics only (no electrical, thermal or sensor domain), MuJoCo Warp stepped through one CUDA
graph, against one CPU core of MuJoCo through its Python bindings. Results and caveats:
rfcs/0025-batched-gpu-simulation.md, section "Spike".
"""

import json
import sys
import time
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

STEPS = 500  # timed steps per measurement
WARMUP = 20
NS = [1, 64, 1024, 4096]


def options(spec: mujoco.MjSpec) -> None:
    # Lagrange's MuJoCo backend options: 1 ms step, elliptic cones, implicitfast (RFC 0017).
    spec.option.timestep = 0.001
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST


def floor(spec: mujoco.MjSpec) -> None:
    g = spec.worldbody.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [20, 20, 0.1]


def arm_model(path: str) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(path)
    options(spec)
    floor(spec)
    # A position servo on each hinge and slide, as JointControl gives the arm.
    for j in spec.joints:
        if j.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            a = spec.add_actuator()
            a.target = j.name
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.set_to_position(kp=200.0, kv=10.0)
    return spec.compile()


def quad_model(path: str) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(path)
    options(spec)
    floor(spec)
    base = spec.worldbody.first_body()
    base.pos = [0, 0, 0.3]
    base.add_freejoint()
    # Thrust along each rotor body's z as a site actuator (the coefficient rotor model's force).
    for b in spec.bodies:
        if b.name.startswith("rotor_"):
            s = b.add_site()
            s.name = b.name + "_thrust"
            a = spec.add_actuator()
            a.target = s.name
            a.trntype = mujoco.mjtTrn.mjTRN_SITE
            a.gear = [0, 0, 1, 0, 0, 0]
            a.ctrlrange = [0, 20]
            a.ctrllimited = True
    return spec.compile()


def cpu_rate(m: mujoco.MjModel, ctrl: np.ndarray) -> float:
    d = mujoco.MjData(m)
    d.ctrl[:] = ctrl
    for _ in range(WARMUP):
        mujoco.mj_step(m, d)
    t = time.perf_counter()
    for _ in range(STEPS * 4):
        mujoco.mj_step(m, d)
    return STEPS * 4 / (time.perf_counter() - t)


def gpu_rate(m: mujoco.MjModel, n: int, ctrl: np.ndarray, nconmax: int, njmax: int) -> dict:
    d0 = mujoco.MjData(m)
    d0.ctrl[:] = ctrl
    mujoco.mj_forward(m, d0)
    wm = mjw.put_model(m)
    wd = mjw.put_data(m, d0, nworld=n, nconmax=nconmax, njmax=njmax)
    # Different starting joint offsets per world so the worlds do not all do the same thing.
    rng = np.random.default_rng(1)
    qpos = np.tile(d0.qpos, (n, 1))
    qpos[:, -min(6, m.nq) :] += rng.normal(0, 0.05, (n, min(6, m.nq)))
    wp.copy(wd.qpos, wp.array(qpos.astype(np.float32)))
    with wp.ScopedCapture() as cap:
        mjw.step(wm, wd)
    for _ in range(WARMUP):
        wp.capture_launch(cap.graph)
    wp.synchronize()
    t = time.perf_counter()
    for _ in range(STEPS):
        wp.capture_launch(cap.graph)
    wp.synchronize()
    dt = time.perf_counter() - t
    overflow = int(np.count_nonzero(wd.overflow.numpy())) if hasattr(wd, "overflow") else -1
    nan = int(np.count_nonzero(~np.isfinite(wd.qpos.numpy()).all(axis=1)))
    return {
        "n": n,
        "batched_steps_per_s": STEPS / dt,
        "env_steps_per_s": n * STEPS / dt,
        "rtf_per_env": STEPS * m.opt.timestep / dt,
        "worlds_overflowed": overflow,
        "worlds_nan": nan,
    }


def main() -> None:
    arm, quad, out = sys.argv[1:4]
    wp.init()
    dev = wp.get_device()
    res = {
        "device": dev.name,
        "mujoco": mujoco.__version__,
        "mujoco_warp": getattr(mjw, "__version__", "?"),
        "warp": wp.config.version,
        "steps_timed": STEPS,
        "models": {},
    }
    for name, m, ncon, nj in [
        ("arm6", arm_model(arm), 16, 64),
        ("quad_x500", quad_model(quad), 16, 64),
    ]:
        c = np.zeros(m.nu)
        if name == "quad_x500":
            c[:] = 9.81 * m.body_subtreemass[1] / 4  # hover thrust per rotor
        info = {"nq": int(m.nq), "nv": int(m.nv), "nu": int(m.nu), "ngeom": int(m.ngeom)}
        info["cpu_mujoco_steps_per_s"] = cpu_rate(m, c)
        info["gpu"] = [gpu_rate(m, n, c, ncon, nj) for n in NS]
        res["models"][name] = info
        print(name, json.dumps(info, indent=1), flush=True)
    with Path(out).open("w") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
