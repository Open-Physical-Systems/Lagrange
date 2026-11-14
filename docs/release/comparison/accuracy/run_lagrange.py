#!/usr/bin/env python3

"""The accuracy cases on Lagrange's MuJoCo and Jolt backends, through the Python bindings.

    source /opt/ros/lyrical/setup.bash; source <lagrange install>/setup.bash
    run_lagrange.py SCENE_DIR OUT.json        (SCENE_DIR from scenes.py)

Each case's MJCF is the robot, with no world; a profile picks the backend with a 1 ms step and
switches every other domain to its substitute.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import drive
import lagrange

PROFILE = """schema_version: 1
stage: {stage}
domains:
  rigid_body:   {{ enabled: true,  backend: {backend}, dt: 0.001 }}
  electrical:   {{ enabled: false, substitute: ideal }}
  thermal:      {{ enabled: false, substitute: constant }}
  aerodynamics: {{ enabled: false, substitute: zero }}
  weather:      {{ enabled: false, substitute: clear }}
  emi:          {{ enabled: false, substitute: zero }}
  radiation:    {{ enabled: false, substitute: zero }}
  sensors:      {{ enabled: false, substitute: ground_truth }}
"""


class Run:
    def __init__(self, xml: str, backend: str) -> None:
        profile = Path(xml).with_name(f"{Path(xml).stem}.{backend}.yaml")
        profile.write_text(PROFILE.format(stage=xml, backend=backend), encoding="utf-8")
        self.sim = lagrange.Simulation(xml, world=None, profile=str(profile))
        self.body = "bob" if "bob" in self.sim.bodies() else "box"

    def step_to(self, t: float) -> None:
        self.sim.step_until(t)

    def position(self) -> list[float]:
        return list(self.sim.read(f"body/{self.body}/pose")[:3])

    def joint(self) -> tuple[float, float]:
        return self.sim.read("joint/hinge/position")[0], self.sim.read("joint/hinge/velocity")[0]

    def contact_force(self) -> float | None:
        return self.sim.read(f"body/{self.body}/contact_force")[2]


def main() -> int:
    scene_dir, out = sys.argv[1], sys.argv[2]
    results = {}
    for backend in ("mujoco", "jolt"):
        results[f"Lagrange {backend}"] = drive.measure(
            lambda case, b=backend: Run(f"{case}.xml", b), scene_dir
        )
    Path(out).write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
