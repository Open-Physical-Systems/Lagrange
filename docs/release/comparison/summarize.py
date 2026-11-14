#!/usr/bin/env python3
# ruff: noqa: E501, RUF001  (comparison scripts: generated XML, units in text)
"""Turns the raw results in docs/release/comparison/results/ into the tables of
docs/comparison.md (Markdown on stdout). Pure Python, no dependencies.

    summarize.py RESULTS_DIR
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

LIDAR_RAYS_PER_SIM_S = (720 + 16 * 900) * 10  # the sample AMR: 2D + 3D lidar, both at 10 Hz
CAMERA_HZ = 30


def fmt(x: float | None, digits: int = 3) -> str:
    if x is None:
        return "n/a"
    return f"{x:.{digits}g}"


def spread(values: list[float], digits: int = 3) -> str:
    if not values:
        return "n/a"
    med = statistics.median(values)
    if len(values) == 1:
        return fmt(med, digits)
    return f"{fmt(med, digits)} ({fmt(min(values), digits)}–{fmt(max(values), digits)})"


def accuracy(results: Path) -> None:
    rows: dict[str, dict] = {}
    for name in (
        "accuracy_lagrange.json",
        "accuracy_gazebo.json",
        "accuracy_mujoco312.json",
        "accuracy_mujoco_pybullet.json",
    ):
        path = results / name
        if path.exists():
            rows.update(json.loads(path.read_text()))
    print(
        "| Engine | Free fall, z after 1 s: error | Pendulum period (0.05 rad): error |"
        " Pendulum energy after 60 s (1 rad) | Slide on 30°, μ 0.3: acceleration error |"
        " Hold on 10°: creep in 2 s | Resting box: contact force error | Resting box: sinks |"
    )
    print("|---|---|---|---|---|---|---|---|")
    for engine, r in rows.items():
        force = r.get("rest_force")
        print(
            f"| {engine} | {r['freefall_error'] * 1000:+.3f} mm "
            f"| {r['pendulum_period_rel_error'] * 1e6:+.2f} ppm "
            f"| {r['energy_rel_drift_end'] * 100:+.2f} % (worst {r['energy_rel_max_dev'] * 100:.2f} %) "
            f"| {r['slide_acceleration_rel_error'] * 1e6:+.1f} ppm "
            f"| {r['static_displacement_2s'] * 1000:.3f} mm "
            f"| {'n/a' if force is None else format(r['rest_force_rel_error'] * 1e6, '+.2g') + ' ppm'} "
            f"| {r['rest_penetration'] * 1000:.4f} mm |"
        )
    print()


def performance(results: Path) -> None:
    path = results / "performance.jsonl"
    if not path.exists():
        return
    by: dict[str, list[dict]] = defaultdict(list)
    loads, clocks = [], []
    lines = [json.loads(line) for line in path.read_text().splitlines()]
    real = [rec for rec in lines if not rec.get("quick")]
    for rec in real or lines:  # a dry run (--quick) only when there is nothing else
        by[rec["scenario"]].append(rec)
        loads.append(rec["machine"]["loadavg"][0])
        clocks.append(rec["machine"]["gpu"])
    gpu = [[x.strip() for x in c.split(",")] for c in clocks if c.count(",") == 5]
    gr = [int(g[0].split()[0]) for g in gpu]
    temp = [int(g[3]) for g in gpu]
    print(
        f"Runs: {sum(len(v) for v in by.values())}. At the start of a run: 1-minute load average "
        f"{min(loads):.2f} to {max(loads):.2f}; GPU graphics clock {min(gr)} to {max(gr)} MHz, "
        f"{min(temp)} to {max(temp)} °C, power states {', '.join(sorted({g[4] for g in gpu}))}."
    )
    print()
    print(
        "| Scenario | Simulator | Real-time factor | Stepping wall time (s) | Start-up to first step (s) |"
        " Peak RSS (MB) | CPU while stepping (cores) | Sensor output | n |"
    )
    print("|---|---|---|---|---|---|---|---|---|")
    groups = sorted({k.split("/")[0] for k in by})
    for group in groups:
        names = [k for k in by if k.startswith(group + "/") and "_1step" not in k]
        for name in names:
            recs = [r["result"] for r in by[name] if "error" not in r["result"]]
            if not recs:
                why = by[name][0]["result"].get("error")
                print(
                    f"| {group} | {name.split('/')[1]} | did not finish ({why}) | | | | | | {len(by[name])} |"
                )
                continue
            sim_s = None
            if recs[0].get("sim") == "gazebo" and "iterations" in recs[0]:
                # Start-up: the same world run for one iteration (with the subscriber when there
                # is one; at one iteration nothing is rendered or received either way).
                partner = by.get(f"{group}/gazebo_1step") or by.get(
                    f"{group}/gazebo_1step_subscribed", []
                )
                one = [r["result"] for r in partner]
                start = statistics.median(r["wall_s"] for r in one) if one else None
                cpu1 = statistics.median(r["cpu_s"] for r in one) if one else None
                sim_s = (recs[0]["iterations"] - 1) * 0.001
                stepping = [r["wall_s"] - start for r in recs] if start is not None else []
                rtf = [sim_s / s for s in stepping]
                cpu = (
                    [(r["cpu_s"] - cpu1) / (r["wall_s"] - start) for r in recs]
                    if start is not None
                    else []
                )
                startup = [r["wall_s"] for r in one]
            else:
                stepping = [r["stepping_s"] for r in recs]
                rtf = [r["rtf"] for r in recs]
                cpu = [r["cpu_cores"] for r in recs]
                startup = [r["startup_s"] for r in recs]
                sim_s = recs[0].get("simulated_s")
            rss = [r["peak_rss_mb"] for r in recs]
            output = ""
            if group == "amr_sensors":
                if "messages" in recs[0]:
                    got = [sum(r["messages"].values()) for r in recs]
                    rays = [
                        sum(r["messages"][t] * r["size_per_message"][t] for t in r["messages"])
                        for r in recs
                    ]
                    per_s = [ry / st for ry, st in zip(rays, stepping, strict=True)]
                    output = f"{spread(per_s)} rays/s; scans received {spread(got)}"
                elif "lagrange" in name:
                    output = f"{spread([LIDAR_RAYS_PER_SIM_S * x for x in rtf])} rays/s"
                else:
                    output = "none (no subscriber: nothing rendered)"
            elif group == "camera":
                if "messages" in recs[0]:
                    got = [sum(r["messages"].values()) for r in recs]
                    fps = [g / s for g, s in zip(got, stepping, strict=True)]
                    output = f"{spread(fps)} frames/s; frames received {spread(got)}"
                else:
                    output = f"{spread([CAMERA_HZ * x for x in rtf])} frames/s"
            elif group == "quad":
                h = [r.get("height_at_end_m") for r in recs if r.get("height_at_end_m") is not None]
                output = f"height at the end {spread(h)} m"
            print(
                f"| {group} | {name.split('/')[1]} | {spread(rtf)} | {spread(stepping)} | {spread(startup)} "
                f"| {spread(rss, 4)} | {spread(cpu, 2)} | {output} | {len(recs)} |"
            )
    print()


def determinism(results: Path) -> None:
    path = results / "determinism.jsonl"
    if not path.exists():
        return
    runs: dict[str, list[str]] = defaultdict(list)
    for line in path.read_text().splitlines():
        rec = json.loads(line)
        if "sha256" in rec:
            runs[rec["engine"]].append(rec["sha256"])
        else:
            print(f"- `{rec['command']}`: {rec['output']}")
    print()
    print("| Engine | Run 1 digest | Run 2 digest | Bit-identical |")
    print("|---|---|---|---|")
    for engine, digests in runs.items():
        if len(digests) < 2:
            print(f"| {engine} | `{digests[0][:8]}` | not run | not known |")
            continue
        same = len(set(digests)) == 1
        print(
            f"| {engine} | `{digests[0][:8]}` | `{digests[-1][:8]}` | {'yes' if same else 'NO'} |"
        )
    print()


def main() -> int:
    results = Path(sys.argv[1])
    accuracy(results)
    performance(results)
    determinism(results)
    return 0


if __name__ == "__main__":
    sys.exit(main())
