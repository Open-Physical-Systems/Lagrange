#!/usr/bin/env python3
# ruff: noqa: N812  (physics notation in text; models imported as M)
"""Standalone MuJoCo (the version Lagrange vendors) on the spike's slope models: does MuJoCo's
noslip solver remove the creep, and is the articulated robot's creep MuJoCo's own (not
Lagrange's)? Test tool only; MuJoCo's Python package is not a Lagrange dependency.

    python -m venv V; V/bin/pip install mujoco==3.12.0 numpy
    V/bin/python mj_noslip.py MODEL_DIR OUT.json

Settings as Lagrange's backend: elliptic cone, implicitfast, 1 ms. Also: the articulated
robot with an armature (reflected rotor inertia) on its leg joints. The articulated quadruped is
held by the same impedance law as contact_tests.py (kp 100, kd 1, standing up over 0.5 s),
applied as joint forces each step.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import models as M

KP, KD = 100.0, 1.0


def stand_angles(height: float) -> tuple[float, float]:
    a, b = M.THIGH_L, M.SHIN_L
    knee = -math.acos((height * height - a * a - b * b) / (2 * a * b))
    return -math.atan2(b * math.sin(knee), a + b * math.cos(knee)), knee


def run(path: Path, deg: float, noslip: int, articulated: bool, armature: float = 0.0) -> dict:
    model = mujoco.MjModel.from_xml_path(str(path))
    model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.timestep = 0.001
    model.opt.noslip_iterations = noslip
    if articulated:
        model.dof_armature[6:] = armature  # the leg joints (dofs after the free joint)
    data = mujoco.MjData(model)
    # The soft contacts' regularisation uses the inverse weights computed at compile time
    # (body_invweight0, dof_invweight0): recompute them for the changed armature.
    mujoco.mj_setConst(model, data)
    th = math.radians(deg)
    down = np.array([math.cos(th), 0.0, -math.sin(th)])
    base = model.body("base_link").id
    joints = []
    if articulated:
        thigh, knee = stand_angles(0.28)
        for leg in M.LEGS:
            for j, target in (("hip", 0.0), ("thigh", thigh), ("knee", knee)):
                jid = model.joint(f"{leg}_{j}_joint").id
                joints.append((model.jnt_qposadr[jid], model.jnt_dofadr[jid], target))
    xs = {}
    n = 6000
    for i in range(n):
        t = i * 0.001
        s = min(1.0, t / 0.5)
        for qa, da, target in joints:
            data.qfrc_applied[da] = KP * (s * target - data.qpos[qa]) - KD * data.qvel[da]
        mujoco.mj_step(model, data)
        if i + 1 in (1000, 3000, 6000):
            xs[i + 1] = float(data.xpos[base] @ down)
    quat = data.xquat[base]
    pitch = 2 * math.atan2(quat[2], quat[0])
    return {
        "upright": abs(pitch - th) < 0.1,
        "base_height_m": float(data.xpos[base][2]),
        "foot_invweight0": float(model.body_invweight0[model.body("front_left_shin").id][0])
        if articulated
        else None,
        "creep_1_3s_mm": 1e3 * (xs[3000] - xs[1000]),
        "creep_3_6s_mm": 1e3 * (xs[6000] - xs[3000]),
        "creep_rate_3_6s_mm_s": 1e3 * (xs[6000] - xs[3000]) / 3.0,
    }


def main() -> int:
    model_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    results = {"mujoco": mujoco.__version__}
    for name, deg, articulated in (
        ("rigid_quad_slope_10.xml", 10.0, False),
        ("rigid_quad_slope_25.xml", 25.0, False),
        ("quad_slope_10.xml", 10.0, True),
        ("quad_slope_20.xml", 20.0, True),
    ):
        for noslip in (0, 5, 20):
            key = f"{name} noslip {noslip}"
            results[key] = run(model_dir / name, deg, noslip, articulated)
            print(key, results[key], flush=True)
        if articulated:
            # Reflected rotor inertia (MuJoCo armature), as geared leg motors have: does a
            # heavier joint side make the feet creep less?
            for armature in (0.005, 0.02):
                key = f"{name} noslip 0 armature {armature}"
                results[key] = run(model_dir / name, deg, 0, articulated, armature)
                print(key, results[key], flush=True)
    out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
