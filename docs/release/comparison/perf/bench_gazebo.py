#!/usr/bin/env python3

"""One timed Gazebo Jetty run for docs/comparison.md, headless, as fast as possible.

    bench_gazebo.py WORLD.sdf ITERATIONS [TOPIC=laserscan|image ...]

Runs `gz sim -s -r --headless-rendering --iterations N WORLD` under GNU time (wall time, peak
RSS, CPU), with gz_count.py subscribed to the topics given (Gazebo renders a sensor only for a
subscriber) and counting what arrives. Prints one JSON line. Run with GZ_IP=127.0.0.1 and a
GZ_PARTITION of its own; __EGL_VENDOR_LIBRARY_FILENAMES picks the GPU for headless rendering.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main() -> int:
    world, iterations, topics = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    counter = None
    if topics:
        counter = subprocess.Popen(
            [sys.executable, str(HERE / "gz_count.py"), *topics],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert counter.stderr is not None
        line = counter.stderr.readline()
        if "ready" not in line:
            print(line + counter.stderr.read(), file=sys.stderr)
            return 1
    with tempfile.NamedTemporaryFile("r", suffix=".time") as timing:
        run = subprocess.run(
            [
                "/usr/bin/time",
                "-v",
                "-o",
                timing.name,
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                "--iterations",
                str(iterations),
                world,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        stats = timing.read()
    out: dict[str, object] = {"sim": "gazebo", "iterations": iterations, "exit": run.returncode}
    wall = re.search(r"Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)", stats)
    if wall:
        parts = [float(x) for x in wall.group(1).split(":")]
        seconds = 0.0
        for x in parts:
            seconds = seconds * 60 + x
        out["wall_s"] = seconds
    rss = re.search(r"Maximum resident set size \(kbytes\): (\d+)", stats)
    if rss:
        out["peak_rss_mb"] = int(rss.group(1)) / 1024.0
    user = re.search(r"User time \(seconds\): (\S+)", stats)
    system = re.search(r"System time \(seconds\): (\S+)", stats)
    if user and system and "wall_s" in out:
        out["cpu_s"] = float(user.group(1)) + float(system.group(1))
        out["cpu_cores"] = out["cpu_s"] / out["wall_s"]  # type: ignore[operator]
    engine = re.search(r"Loaded \[gz::physics::(\w+)::Plugin\]", run.stderr + run.stdout)
    if engine:
        out["engine"] = engine.group(1)
    if counter is not None:
        counter.send_signal(signal.SIGTERM)
        text, _ = counter.communicate(timeout=30)
        out.update(json.loads(text.strip().splitlines()[-1]))
    if run.returncode != 0:
        out["stderr_tail"] = run.stderr[-2000:]
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    sys.exit(main())
