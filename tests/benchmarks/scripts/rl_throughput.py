#!/usr/bin/env python3
"""Reinforcement-learning throughput (v1.6.0 plan, A5): environment steps per second on the CPU.

For each robot and each number of environments it runs the environments in four ways and gives
the median of `--repeats` runs of the aggregate steps per second (every environment's steps, over
the wall time), the start-up time and the memory (the sum of every process's PSS):

    processes   N independent processes, each stepping its own LagrangeEnv: the ceiling
    parallel    ParallelEnvs(N): one worker process per environment (the default)
    batched     ParallelEnvs(N, envs_per_worker=N/W): W processes, environments one after another
    threaded    ParallelEnvs(N, envs_per_worker=N/W, worker_threads=N/W): W processes, each
                stepping its environments on threads at once

Robots: the sample arm with its power system and no sensors (arm), the sample AMR with its power
system and two CPU lidars (amr) and with the sensors domain off (amr_nolidar), and the sample
quadcopter hovering (quad). Steps are LagrangeEnv's default 20 ms with uniform random actions.

    rl_throughput.py [--robots arm,amr,amr_nolidar,quad] [--counts 1,4,16,64]
                     [--modes processes,parallel,batched,threaded] [--repeats 3] [--seconds 3]
                     [--batch-workers 4] [--json OUT.json] [--markdown OUT.md] [--plot OUT.png]

Needs the workspace sourced (lagrange_python, lagrange_assets, lagrange_profiles); matplotlib
only for --plot. Timings on a shared machine are not pass/fail: the nightly job records them
(tools/ci/benchmarks.sh); docs/design/benchmarks.md §6 has the numbers and how to read them.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import multiprocessing
import os
import platform
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

MODES = ("processes", "parallel", "batched", "threaded")
ROBOTS = ("arm", "amr", "amr_nolidar", "quad")
WARMUP_STEPS = 5  # steps before the clock starts: first contacts, caches, the spin loop

# Every domain named, as a profile must (profiles/schema.json); the stage is the robot's file.
_SENSORS_OFF = """schema_version: 1
stage: {stage}
domains:
  rigid_body:   {{ enabled: true,  backend: mujoco, dt: 0.001 }}
  electrical:   {{ enabled: true,  fidelity: averaged, dt: 0.0001 }}
  thermal:      {{ enabled: false, substitute: ambient }}
  aerodynamics: {{ enabled: false, substitute: zero }}
  weather:      {{ enabled: false, substitute: clear }}
  emi:          {{ enabled: false, substitute: zero }}
  radiation:    {{ enabled: false, substitute: zero }}
  sensors:      {{ enabled: false, substitute: ground_truth }}
"""


def robot_options(robot: str, scratch: Path) -> dict[str, Any]:
    """LagrangeEnv's arguments for one of ROBOTS (picklable: plain strings and numbers)."""
    import lagrange
    from ament_index_python.packages import get_package_share_directory

    assets = Path(get_package_share_directory("lagrange_assets"))

    def sensors_off(name: str, stage: str) -> str:
        path = scratch / f"{name}_sensors_off.yaml"
        path.write_text(_SENSORS_OFF.format(stage=stage))
        return str(path)

    if robot == "arm":
        return {
            "robot": str(assets / "robots/arm6/arm6.urdf"),
            "overlay": str(assets / "robots/arm6/arm6.overlay.yaml"),
            "profile": sensors_off("arm", "robots/arm6/arm6.urdf"),
        }
    if robot in ("amr", "amr_nolidar"):
        options = {
            "robot": lagrange.sample_robot(),
            "overlay": str(assets / "robots/amr_diffdrive/amr_diffdrive.overlay.yaml"),
        }
        if robot == "amr_nolidar":
            options["profile"] = sensors_off("amr", "robots/amr_diffdrive/amr_diffdrive.urdf")
        return options
    if robot == "quad":
        return {"robot": lagrange.sample_quadcopter(), "profile": "quad_x500_hover"}
    raise ValueError(f"unknown robot {robot}: one of {', '.join(ROBOTS)}")


def make_env(options: dict[str, Any]) -> Any:
    from lagrange.envs import LagrangeEnv

    return LagrangeEnv(
        options["robot"],
        profile=options.get("profile"),
        overlay=options.get("overlay"),
        cameras=False,
    )


def pss_mb(pids: list[int]) -> float:
    """The proportional set size of these processes together, MB (shared pages split)."""
    total = 0
    for pid in pids:
        with contextlib.suppress(OSError):
            for line in Path(f"/proc/{pid}/smaps_rollup").read_text().splitlines():
                if line.startswith("Pss:"):
                    total += int(line.split()[1])
                    break
    return total / 1024.0


# --- processes: N independent processes ----------------------------------------------------


def _alone(
    options: dict[str, Any], seconds: float, seed: int, ready: Any, go: Any, out: Any
) -> None:
    env = make_env(options)
    env.reset(seed=seed)
    rng = np.random.default_rng(seed)
    shape = env.action_space.shape
    for _ in range(WARMUP_STEPS):
        env.step(rng.uniform(-1.0, 1.0, shape))
    ready.release()
    go.wait()
    steps = 0
    start = time.perf_counter()
    until = start + seconds
    while time.perf_counter() < until:
        env.step(rng.uniform(-1.0, 1.0, shape))
        steps += 1
    elapsed = time.perf_counter() - start
    out.put((steps, elapsed, pss_mb([os.getpid()])))  # while the others live: pages shared


def run_processes(options: dict[str, Any], count: int, seconds: float) -> dict[str, float]:
    context = multiprocessing.get_context("spawn")
    ready, go, out = context.Semaphore(0), context.Event(), context.Queue()
    begin = time.perf_counter()
    workers = [
        context.Process(target=_alone, args=(options, seconds, i, ready, go, out), daemon=True)
        for i in range(count)
    ]
    for process in workers:
        process.start()
    for _ in range(count):
        if not ready.acquire(timeout=600.0):
            raise RuntimeError("an environment process did not start")
    started = time.perf_counter() - begin
    go.set()
    results = [out.get(timeout=600.0) for _ in range(count)]
    for process in workers:
        process.join(timeout=30.0)
        if process.is_alive():
            process.terminate()
    steps = sum(r[0] for r in results)
    elapsed = max(r[1] for r in results)
    return {
        "steps_per_second": steps / elapsed,
        "start_seconds": started,
        "memory_mb": sum(r[2] for r in results) + pss_mb([os.getpid()]),
    }


# --- parallel, batched, threaded: ParallelEnvs -------------------------------------------------


def _maker(options: dict[str, Any]) -> Any:
    import functools

    return functools.partial(make_env, options)


def run_parallel(
    options: dict[str, Any], count: int, seconds: float, per_worker: int, threads: int
) -> dict[str, float]:
    from lagrange.envs import ParallelEnvs

    begin = time.perf_counter()
    envs = ParallelEnvs(_maker(options), count, envs_per_worker=per_worker, worker_threads=threads)
    try:
        envs.reset(seed=0)
        started = time.perf_counter() - begin
        rng = np.random.default_rng(0)
        shape = (count, *envs.action_space.shape)
        for _ in range(WARMUP_STEPS):
            envs.step(rng.uniform(-1.0, 1.0, shape))
        steps = 0
        start = time.perf_counter()
        until = start + seconds
        while time.perf_counter() < until:
            envs.step(rng.uniform(-1.0, 1.0, shape))
            steps += 1
        elapsed = time.perf_counter() - start
        memory = pss_mb([os.getpid()] + [p.pid for p in envs.processes])
    finally:
        envs.close()
    return {
        "steps_per_second": steps * count / elapsed,
        "start_seconds": started,
        "memory_mb": memory,
    }


def run_one(
    mode: str, options: dict[str, Any], count: int, seconds: float, batch_workers: int
) -> dict[str, float]:
    if mode == "processes":
        return run_processes(options, count, seconds)
    if mode == "parallel":
        return run_parallel(options, count, seconds, 1, 1)
    per_worker = math.ceil(count / min(count, batch_workers))
    return run_parallel(
        options, count, seconds, per_worker, per_worker if mode == "threaded" else 1
    )


# --- the machine and the report ----------------------------------------------------------------


def machine() -> dict[str, Any]:
    model = ""
    with contextlib.suppress(OSError):
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    memory = 0.0
    with contextlib.suppress(OSError):
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemTotal:"):
                memory = int(line.split()[1]) / 1024.0 / 1024.0
    commit = ""
    with contextlib.suppress(OSError, subprocess.SubprocessError):
        commit = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            check=False,
            cwd=Path(__file__).resolve().parent,
        ).stdout.strip()
    return {
        "cpu": model,
        "logical_cpus": os.cpu_count(),
        "memory_gb": round(memory, 1),
        "kernel": platform.release(),
        "python": platform.python_version(),
        "commit": commit,
        "date": time.strftime("%Y-%m-%d %H:%M %z"),
        "load_average": os.getloadavg(),
    }


def markdown(report: dict[str, Any]) -> str:
    m = report["machine"]
    lines = [
        f"Machine: {m['cpu']}, {m['logical_cpus']} logical CPUs, {m['memory_gb']} GB, "
        f"Linux {m['kernel']}, Python {m['python']}, commit {m['commit']}, {m['date']}; "
        f"load average {m['load_average'][0]:.1f} at the start, "
        f"{report['load_average_end'][0]:.1f} at the end. Medians of {report['repeats']} runs "
        f"of {report['seconds']} s; batched and threaded use "
        f"{report['batch_workers']} worker processes.",
        "",
        "| Robot | Envs | " + " | ".join(report["modes"]) + " |",
        "|---|---|" + "---|" * len(report["modes"]),
    ]
    for robot in report["robots"]:
        for count in report["counts"]:
            cells = []
            for mode in report["modes"]:
                row = report["results"].get(f"{robot}/{mode}/{count}")
                if row is None:
                    cells.append("—")
                    continue
                rate = f"{row['steps_per_second']:,.0f}".replace(",", " ")
                cells.append(
                    f"{rate} ({row['memory_mb'] / count:.0f} MB, {row['start_seconds']:.1f} s)"
                )
            lines.append(f"| {robot} | {count} | " + " | ".join(cells) + " |")
    lines.append("")
    lines.append(
        "Each cell: environment steps per second (memory per environment as PSS, start-up)."
    )
    return "\n".join(lines) + "\n"


def plot(report: dict[str, Any], path: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    robots = report["robots"]
    figure, axes = plt.subplots(1, len(robots), figsize=(4.2 * len(robots), 3.8), squeeze=False)
    for axis, robot in zip(axes[0], robots, strict=True):
        for mode in report["modes"]:
            counts, rates = [], []
            for count in report["counts"]:
                row = report["results"].get(f"{robot}/{mode}/{count}")
                if row is not None:
                    counts.append(count)
                    rates.append(row["steps_per_second"])
            axis.plot(counts, rates, marker="o", label=mode)
        axis.set_xscale("log", base=2)
        axis.set_xticks(report["counts"], [str(c) for c in report["counts"]])
        axis.set_title(robot)
        axis.set_xlabel("environments")
        axis.grid(True, alpha=0.3)
    axes[0][0].set_ylabel("environment steps per second")
    axes[0][-1].legend()
    m = report["machine"]
    figure.suptitle(
        f"Lagrange RL throughput on the CPU: {m['cpu']}, {m['logical_cpus']} logical CPUs, "
        f"load {m['load_average'][0]:.0f} to {report['load_average_end'][0]:.0f}; "
        f"batched and threaded on {report['batch_workers']} processes"
    )
    figure.tight_layout()
    figure.savefig(path, dpi=120)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--robots", default=",".join(ROBOTS))
    parser.add_argument("--counts", default="1,4,16,64")
    parser.add_argument("--modes", default=",".join(MODES))
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--batch-workers", type=int, default=4)
    parser.add_argument("--json")
    parser.add_argument("--markdown")
    parser.add_argument("--plot")
    parser.add_argument("--from-json", help="only write --markdown/--plot from an earlier --json")
    args = parser.parse_args()

    if args.from_json:
        report = json.loads(Path(args.from_json).read_text())
    else:
        robots = args.robots.split(",")
        modes = args.modes.split(",")
        for mode in modes:
            if mode not in MODES:
                parser.error(f"unknown mode {mode}: one of {', '.join(MODES)}")
        counts = [int(c) for c in args.counts.split(",")]
        report = {
            "machine": machine(),
            "robots": robots,
            "modes": modes,
            "counts": counts,
            "repeats": args.repeats,
            "seconds": args.seconds,
            "batch_workers": args.batch_workers,
            "results": {},
            "runs": {},
        }
        with tempfile.TemporaryDirectory(prefix="lagrange_rl_bench_") as scratch:
            for robot in robots:
                options = robot_options(robot, Path(scratch))
                for count in counts:
                    for mode in modes:
                        runs = []
                        for _ in range(args.repeats):
                            runs.append(
                                run_one(mode, options, count, args.seconds, args.batch_workers)
                            )
                        key = f"{robot}/{mode}/{count}"
                        report["runs"][key] = runs
                        report["results"][key] = {
                            q: statistics.median(r[q] for r in runs) for q in runs[0]
                        }
                        best = report["results"][key]
                        print(
                            f"{robot:12s} {mode:9s} {count:3d} envs: "
                            f"{best['steps_per_second']:9.0f} steps/s, "
                            f"{best['memory_mb'] / count:6.1f} MB/env, "
                            f"start {best['start_seconds']:.2f} s",
                            flush=True,
                        )
        report["load_average_end"] = os.getloadavg()
        if args.json:
            Path(args.json).write_text(json.dumps(report, indent=1) + "\n")
    text = markdown(report)
    print(text)
    if args.markdown:
        Path(args.markdown).write_text(text)
    if args.plot:
        plot(report, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
