#!/usr/bin/env python3
"""The spike's slope creep (§5, §6) again, now through Lagrange's own levers: the profile key
`domains.rigid_body.noslip_iterations` and joint armature imported from MJCF (`armature` on the
leg joints, as a motor's reflected rotor inertia would give). MuJoCo, 1 ms.

    python3 noslip_armature.py MODEL_DIR OUT.json

MODEL_DIR is what `models.py` writes. Measured as contact_tests.py measures: the rigid robot's
creep along the slope over 3-6 s; the legged robot's feet's mean creep over 3-6 s, standing on
the same impedance law (kp 100, kd 1, standing up over 0.5 s).
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import contact_tests as C  # noqa: N812
import lagrange

DT = 0.001
NOSLIP = (0, 5, 20)
ARMATURE = (0.0, 0.005, 0.02)  # kg·m², as mj_noslip.py


def make(stage: Path, out: Path, noslip: int) -> lagrange.Simulation:
    extra = f"{DT}, noslip_iterations: {noslip}" if noslip else f"{DT}"
    profile = out / "profiles" / f"{stage.stem}.noslip{noslip}.yaml"
    profile.parent.mkdir(parents=True, exist_ok=True)
    profile.write_text(C.PROFILE.format(stage=stage, backend="mujoco", dt=extra), encoding="utf-8")
    return lagrange.Simulation(str(stage), world=None, profile=str(profile), joint_control=True)


def rigid(stage: Path, out: Path, deg: float, noslip: int) -> dict:
    th = math.radians(deg)
    down = np.array([math.cos(th), 0.0, -math.sin(th)])
    sim = make(stage, out, noslip)
    sim.step_until(3.0)
    x3 = np.asarray(sim.read("body/base_link/pose")[:3]) @ down
    start = time.perf_counter()
    sim.step_until(6.0)
    wall = time.perf_counter() - start
    x6 = np.asarray(sim.read("body/base_link/pose")[:3]) @ down
    return {"creep_rate_3_6s_mm_s": float(1e3 * (x6 - x3) / 3.0), "us_per_step": wall / 3e-3}


def legged(stage: Path, out: Path, deg: float, noslip: int) -> dict:
    th = math.radians(deg)
    down = np.array([math.cos(th), 0.0, -math.sin(th)])
    sim = make(stage, out, noslip)
    q = C.Quad(sim)
    marks: dict[float, float] = {}

    def sample(t: float) -> None:
        for mark in (1.0, 3.0, 6.0):
            if abs(t - mark) < DT / 2:
                marks[mark] = float(np.mean(list(q.feet().values()), axis=0) @ down)

    wall = C.run_steps(sim, DT, 6.0, before=q.apply, after=sample)
    return {
        "feet_creep_mm_1_to_3s": 1e3 * (marks[3.0] - marks[1.0]),
        "feet_creep_rate_3_6s_mm_s": 1e3 * (marks[6.0] - marks[3.0]) / 3.0,
        "base_z": float(sim.read("body/base_link/pose")[2]),
        "wall_s": wall,
    }


def with_armature(model_dir: Path, out: Path, deg: float, armature: float) -> Path:
    text = (model_dir / f"quad_slope_{deg:g}.xml").read_text(encoding="utf-8")
    if armature:
        text = text.replace('damping="0.05"/>', f'damping="0.05" armature="{armature}"/>')
    path = out / "models" / f"quad_slope_{deg:g}_armature_{armature:g}.xml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def main() -> int:
    model_dir, out_json = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    out = out_json.parent
    out.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict] = {}
    load = C.loadavg()
    for deg in (10.0, 25.0):
        for noslip in NOSLIP:
            key = f"rigid {deg:g} deg noslip {noslip}"
            results[key] = rigid(model_dir / f"rigid_quad_slope_{deg:g}.xml", out, deg, noslip)
            print(key, json.dumps(results[key]), flush=True)
    for deg in (10.0, 20.0):
        for noslip in NOSLIP:
            key = f"legged {deg:g} deg noslip {noslip}"
            results[key] = legged(model_dir / f"quad_slope_{deg:g}.xml", out, deg, noslip)
            print(key, json.dumps(results[key]), flush=True)
        for armature in ARMATURE[1:]:
            key = f"legged {deg:g} deg armature {armature:g}"
            stage = with_armature(model_dir, out, deg, armature)
            results[key] = legged(stage, out, deg, 0)
            print(key, json.dumps(results[key]), flush=True)
    results["_meta"] = {
        "load_start": load,
        "load_end": C.loadavg(),
        "lagrange": getattr(lagrange, "__file__", ""),
    }
    out_json.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
