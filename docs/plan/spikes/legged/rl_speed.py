#!/usr/bin/env python3
# ruff: noqa: RUF002  (physics notation in text; models imported as M)
"""RFC 0019 step 1: how fast lagrange.envs runs legged robots, one process and N processes.

    rl_speed.py MODEL_DIR OUT.json [PROCESSES ...]      (default 1 2 4 8)

Robots: the example quadruped (examples/lagrange_example_quadruped/robot/quadruped.urdf, 12
joints) and the spike's hexapod (models.py, 18 joints), each on flat ground with the default
profile (MuJoCo, 1 ms). Actions: uniform random in -1..1, position mode, action_scale 0.5 rad,
a policy step of 20 ms (20 physics steps), episodes of 10 s. Reported: policy steps per second
summed over the processes, and physics steps per second (×20). The machine is shared, so the
load average is recorded with each number.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
from lagrange.envs import LagrangeEnv, ParallelEnvs

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
QUAD = str(REPO / "examples/lagrange_example_quadruped/robot/quadruped.urdf")
HEX = str(HERE / "models/hexapod.xml")
STEP = 0.02


def make_quad() -> LagrangeEnv:
    return LagrangeEnv(QUAD, mode="position", action_scale=0.5, step_seconds=STEP)


def make_hex() -> LagrangeEnv:
    return LagrangeEnv(HEX, mode="position", action_scale=0.5, step_seconds=STEP)


def loadavg() -> float:
    return float(Path("/proc/loadavg").read_text().split()[0])


def single(make, steps: int) -> dict:
    env = make()
    env.reset(seed=0)
    rng = np.random.default_rng(0)
    n = env.action_space.shape[0]
    for _ in range(50):  # warm up
        env.step(rng.uniform(-1, 1, n))
    load0 = loadavg()
    start = time.perf_counter()
    for _ in range(steps):
        _obs, _r, term, trunc, _ = env.step(rng.uniform(-1, 1, n))
        if term or trunc:
            env.reset()
    wall = time.perf_counter() - start
    # The same without Python per physics step: step_until over the same simulated time.
    sim = env.sim
    t0 = sim.time
    start = time.perf_counter()
    sim.step_until(t0 + steps * STEP)
    raw = time.perf_counter() - start
    return {
        "processes": 1,
        "joints": n,
        "policy_steps_per_s": steps / wall,
        "physics_steps_per_s": steps * STEP / 0.001 / wall,
        "rtf": steps * STEP / wall,
        "rtf_step_until_only": steps * STEP / raw,
        "load": (load0 + loadavg()) / 2,
    }


def parallel(make, count: int, steps: int) -> dict:
    envs = ParallelEnvs(make, count)
    envs.reset(seed=0)
    rng = np.random.default_rng(0)
    probe = envs.step(np.zeros((count, ACTIONS[make.__name__])))
    del probe
    for _ in range(20):
        envs.step(rng.uniform(-1, 1, (count, ACTIONS[make.__name__])))
    load0 = loadavg()
    start = time.perf_counter()
    for _ in range(steps):
        envs.step(rng.uniform(-1, 1, (count, ACTIONS[make.__name__])))
    wall = time.perf_counter() - start
    envs.close()
    return {
        "processes": count,
        "policy_steps_per_s": count * steps / wall,
        "physics_steps_per_s": count * steps * STEP / 0.001 / wall,
        "per_process": steps / wall,
        "load": (load0 + loadavg()) / 2,
    }


ACTIONS = {"make_quad": 12, "make_hex": 18}


def _alone(make, barrier, queue, steps: int) -> None:
    env = make()
    env.reset(seed=0)
    rng = np.random.default_rng(0)
    n = ACTIONS[make.__name__]
    for _ in range(50):
        env.step(rng.uniform(-1, 1, n))
    barrier.wait()
    start = time.perf_counter()
    for _ in range(steps):
        _o, _r, term, trunc, _ = env.step(rng.uniform(-1, 1, n))
        if term or trunc:
            env.reset()
    queue.put(steps / (time.perf_counter() - start))


def independent(make, count: int, steps: int) -> dict:
    """`count` processes stepping their own environment with no pipe in between (what a
    learner with its own process per environment, e.g. an asynchronous one, would see): the
    machine's limit, against which ParallelEnvs' lock-step overhead shows."""
    import multiprocessing

    context = multiprocessing.get_context("spawn")
    barrier = context.Barrier(count)
    queue = context.Queue()
    processes = [
        context.Process(target=_alone, args=(make, barrier, queue, steps)) for _ in range(count)
    ]
    load0 = loadavg()
    for p in processes:
        p.start()
    rates = [queue.get() for _ in processes]
    for p in processes:
        p.join()
    return {
        "processes": count,
        "independent": True,
        "policy_steps_per_s": float(sum(rates)),
        "physics_steps_per_s": float(sum(rates)) * STEP / 0.001,
        "slowest_process": float(min(rates)),
        "fastest_process": float(max(rates)),
        "load": (load0 + loadavg()) / 2,
    }


def _cpu_model() -> str:
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name"):
            return line.split(":", 1)[1].strip()
    return "unknown"


def main() -> int:
    model_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    del model_dir
    counts = [int(c) for c in sys.argv[3:]] or [1, 2, 4, 8]
    results: dict = {"cpus": os.cpu_count(), "cpu": _cpu_model()}
    for name, make in (("quadruped (example URDF)", make_quad), ("hexapod (spike MJCF)", make_hex)):
        rows = [single(make, 1500)]
        print(name, rows[-1], flush=True)
        for c in counts:
            if c == 1:
                continue
            rows.append(parallel(make, c, 400))
            print(name, rows[-1], flush=True)
        for c in counts:
            rows.append(independent(make, c, 800))
            print(name, rows[-1], flush=True)
        results[name] = rows
    out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
