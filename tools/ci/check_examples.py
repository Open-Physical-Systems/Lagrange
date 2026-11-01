#!/usr/bin/env python3
"""Runs every package under examples/ as a user would, without a viewer, and checks its result.

    tools/ci/check_examples.py [NAME...] [--timeout SECONDS]

Each example's launch file runs Lagrange and its node; with once:=true the node ends the launch
when it has its result, which it logs as "RESULT: ...". An example fails when it logs none, or the
launch does not end in time. Needs the workspace built and sourced.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def launches() -> dict[str, tuple[str, str]]:
    """Each example's name, package and launch file."""
    found = {}
    for package in sorted(EXAMPLES.iterdir()):
        for file in sorted((package / "launch").glob("*.launch.py")):
            found[package.name.removeprefix("lagrange_example_")] = (package.name, file.name)
    return found


def run(package: str, launch: str, domain: int, timeout: float) -> tuple[bool, str]:
    environment = {**os.environ, "ROS_DOMAIN_ID": str(domain)}
    process = subprocess.Popen(
        ["ros2", "launch", package, launch, "viewer:=false", "once:=true"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=environment,
        start_new_session=True,
    )
    started = time.monotonic()
    try:
        output, _ = process.communicate(timeout=timeout)
        timed_out = False
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate()
        timed_out = True
    results = [line.split("RESULT: ", 1)[1] for line in output.splitlines() if "RESULT: " in line]
    seconds = time.monotonic() - started
    if results and not timed_out:
        return True, f"{seconds:5.1f} s  {results[-1]}"
    tail = "\n".join("    " + line for line in output.splitlines()[-25:])
    return False, f"{'timed out' if timed_out else 'no result'} after {seconds:.1f} s\n{tail}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Runs the examples and checks their results.")
    parser.add_argument("names", nargs="*", help="examples to run (default: all)")
    parser.add_argument("--timeout", type=float, default=150.0, help="seconds for each")
    args = parser.parse_args()
    all_examples = launches()
    failed = 0
    for offset, name in enumerate(args.names or all_examples):
        if name not in all_examples:
            print(f"no example {name}; there are {', '.join(all_examples)}")
            return 2
        package, launch = all_examples[name]
        ok, text = run(package, launch, 60 + offset, args.timeout)
        failed += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {name:14} {text}", flush=True)
    print("PASSED" if not failed else f"FAILED: {failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
