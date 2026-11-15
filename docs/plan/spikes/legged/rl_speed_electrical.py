#!/usr/bin/env python3
"""How much the electrical domain costs a legged robot in lagrange.envs.

    rl_speed_electrical.py OUT.json [PROCESSES ...]      (default 4 8)

The sample quadruped (lagrange.sample_quadruped(), 12 joints, MuJoCo at 1 ms), random position
actions in -1..1 scaled by 0.5 rad, policy steps of 20 ms, episodes of 10 s, three cases:

- bare: profile quadruped_trot, no overlay (no power system; electrical substituted);
- ideal: profile quadruped_trot with the power-system overlay (battery, bus, 12 drives with the
  sample AMR's values), electrical substituted by the ideal source (commanded torque delivered);
- electrical: profile quadruped_trot_powered with the same overlay, the electrical network
  running at 100 us.

One process: the three environments side by side, stepped in turn for ROUNDS rounds of BLOCK
steps each, so that the machine's load (shared with other jobs) falls on all three alike; the
median over the rounds of policy steps per second of wall time and of this process's CPU time.
N processes: ParallelEnvs with send/recv (each environment at its own pace), the cases one after
another, policy steps per second summed over the processes. The load average goes with each.
"""

from __future__ import annotations

import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import lagrange
import numpy as np
from lagrange.envs import LagrangeEnv, ParallelEnvs

STEP = 0.02
ROUNDS = 15
BLOCK = 300
PARALLEL_STEPS = 3000
OPTIONS: dict[str, Any] = {"mode": "position", "action_scale": 0.5, "step_seconds": STEP}


def loadavg() -> float:
    return float(Path("/proc/loadavg").read_text().split()[0])


def cases() -> dict[str, dict[str, Any]]:
    overlay = lagrange.sample_quadruped_overlay()
    return {
        "bare": {"profile": "quadruped_trot", "overlay": None},
        "ideal": {"profile": "quadruped_trot", "overlay": overlay},
        "electrical": {"profile": "quadruped_trot_powered", "overlay": overlay},
    }


def single() -> dict[str, dict[str, Any]]:
    envs = {
        name: LagrangeEnv(lagrange.sample_quadruped(), **case, **OPTIONS)
        for name, case in cases().items()
    }
    rngs = {name: np.random.default_rng(0) for name in envs}
    wall: dict[str, list[float]] = {name: [] for name in envs}
    cpu: dict[str, list[float]] = {name: [] for name in envs}
    for env in envs.values():
        env.reset(seed=0)
    load0 = loadavg()
    for _ in range(ROUNDS):
        for name, env in envs.items():
            rng = rngs[name]
            start, start_cpu = time.perf_counter(), time.process_time()
            for _ in range(BLOCK):
                _o, _r, term, trunc, _ = env.step(rng.uniform(-1, 1, 12))
                if term or trunc:
                    env.reset()
            wall[name].append(BLOCK / (time.perf_counter() - start))
            cpu[name].append(BLOCK / (time.process_time() - start_cpu))
    load = (load0 + loadavg()) / 2
    return {
        name: {
            "processes": 1,
            "policy_steps_per_s": statistics.median(wall[name]),
            "policy_steps_per_cpu_s": statistics.median(cpu[name]),
            "rounds_min_max": [min(wall[name]), max(wall[name])],
            "load": load,
        }
        for name in envs
    }


def parallel(case: dict[str, Any], count: int) -> dict[str, float]:
    rng = np.random.default_rng(0)
    with ParallelEnvs(lagrange.sample_quadruped(), count, **case, env_options=OPTIONS) as envs:
        envs.reset(seed=0)
        for _ in range(20):
            envs.step(rng.uniform(-1, 1, (count, 12)))
        load0 = loadavg()
        done = 0
        start = time.perf_counter()
        envs.send(rng.uniform(-1, 1, (count, 12)))
        while done < PARALLEL_STEPS * count:
            ids, *_ = envs.recv()
            done += len(ids)
            envs.send(rng.uniform(-1, 1, (len(ids), 12)), ids)
        wall = time.perf_counter() - start
        envs.recv(min_envs=count)  # the steps still running
    return {
        "processes": count,
        "policy_steps_per_s": done / wall,
        "load": (load0 + loadavg()) / 2,
    }


def main() -> None:
    out = Path(sys.argv[1])
    counts = [int(a) for a in sys.argv[2:]] or [4, 8]
    results: dict[str, Any] = {
        "module": lagrange.__file__,
        "rounds": ROUNDS,
        "block": BLOCK,
        "parallel_steps": PARALLEL_STEPS,
        "cases": {name: [row] for name, row in single().items()},
    }
    for name, row in results["cases"].items():
        print(name, row[0], flush=True)
    for count in counts:
        for name, case in cases().items():
            row = parallel(case, count)
            print(name, row, flush=True)
            results["cases"][name].append(row)
    out.write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
