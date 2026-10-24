#!/usr/bin/env python3
"""Compare Google Benchmark JSON results against recorded baselines (roadmap H6).

Every benchmark here reports items_per_second (module steps, bus accesses, lidar rays). A
result fails when it is slower than its baseline by more than --max-slowdown. Benchmarks that
were skipped (no Vulkan device) or have no baseline are listed but do not fail; a baseline with
no result fails, so a renamed benchmark cannot drop out unnoticed. With repetitions, the median
is compared. --record writes the results as the machine's baseline instead of checking.

    check_benchmarks.py --baseline baseline.json --machine ci-amd64 core.json lidar.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def load_results(paths: list[Path]) -> tuple[dict[str, float], list[str]]:
    rates: dict[str, float] = {}
    skipped: list[str] = []
    for path in paths:
        data: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
        for run in data.get("benchmarks", []):
            if run.get("run_type") == "aggregate" and run.get("aggregate_name") != "median":
                continue
            name = str(run.get("run_name", run["name"]))
            if run.get("error_occurred") or run.get("skipped"):
                skipped.append(f"{name}: {run.get('error_message', 'skipped')}")
            elif "items_per_second" in run:
                rates[name] = float(run["items_per_second"])
    return rates, skipped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("results", nargs="+", type=Path, help="--benchmark_out JSON files")
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--machine", required=True, help="key in the baseline file")
    parser.add_argument(
        "--max-slowdown",
        type=float,
        default=1.25,
        help="fail when baseline / result exceeds this (default 1.25: 25 %% slower)",
    )
    parser.add_argument("--record", metavar="DESCRIPTION", help="write the baseline instead")
    args = parser.parse_args()

    baselines: dict[str, Any] = json.loads(args.baseline.read_text(encoding="utf-8"))
    if args.record is not None:
        rates, skipped = load_results(args.results)
        baselines.setdefault("machines", {})[args.machine] = {
            "description": args.record,
            "items_per_second": {name: float(f"{r:.4g}") for name, r in sorted(rates.items())},
        }
        args.baseline.write_text(json.dumps(baselines, indent=2) + "\n", encoding="utf-8")
        print(f"recorded {len(rates)} baselines for {args.machine}; skipped: {len(skipped)}")
        return 0
    rates, skipped = load_results(args.results)
    machine = baselines.get("machines", {}).get(args.machine)
    if machine is None:
        for name, rate in sorted(rates.items()):
            print(f"NEW      {name}: {rate:.4g}/s")
        print(f"no baseline for machine {args.machine!r} in {args.baseline}", file=sys.stderr)
        return 2
    expected: dict[str, float] = machine["items_per_second"]

    failures = 0
    for name in sorted(set(expected) | set(rates)):
        base = expected.get(name)
        rate = rates.get(name)
        if rate is None:
            if any(line.startswith(name + ":") for line in skipped):
                continue
            print(f"MISSING  {name}: in the baseline but not run")
            failures += 1
        elif base is None:
            print(f"NEW      {name}: {rate:.4g}/s (no baseline)")
        else:
            slowdown = base / rate
            verdict = "SLOWER" if slowdown > args.max_slowdown else "ok"
            failures += verdict != "ok"
            print(f"{verdict:8} {name}: {rate:.4g}/s, baseline {base:.4g}/s ({slowdown:.2f}x)")
    for line in skipped:
        print(f"SKIPPED  {line}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
