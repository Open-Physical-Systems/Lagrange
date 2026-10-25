#!/usr/bin/env python3
"""Spawn ros2_control controllers, retrying in a fresh process if discovery fails.

    spawn_controllers.py NAME... -- SPAWNER_ARGS...

Runs controller_manager's standard spawner. Occasionally (Fast DDS, many nodes starting at
once on one machine) a spawner's participant never discovers the controller manager's services,
although a new process finds them at once; the spawner would then wait for ever. Here it gives
up after --controller-manager-timeout and the spawn is retried in a new process.
"""

from __future__ import annotations

import subprocess
import sys
import time

ATTEMPTS = 5
TIMEOUT_S = 30  # per attempt, for the controller manager's services to appear


def main() -> int:
    args = sys.argv[1:]
    split = args.index("--") if "--" in args else len(args)
    names, extra = args[:split], args[split + 1 :]
    command = [
        "ros2",
        "run",
        "controller_manager",
        "spawner",
        *names,
        # Before the passed-through arguments: launch appends --ros-args to them.
        "--controller-manager-timeout",
        str(TIMEOUT_S),
        *extra,
    ]
    for attempt in range(1, ATTEMPTS + 1):
        if subprocess.run(command, check=False).returncode == 0:
            return 0
        print(f"spawner attempt {attempt} of {ATTEMPTS} failed; retrying", flush=True)
        time.sleep(2.0)
    return 1


if __name__ == "__main__":
    sys.exit(main())
