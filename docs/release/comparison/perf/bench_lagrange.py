#!/usr/bin/env python3

"""One timed Lagrange run for docs/comparison.md, headless, as fast as possible.

    bench_lagrange.py ROBOT WORLD|none PROFILE SECONDS [--lidar cpu|gpu]

Prints one JSON line: start-up (process start to the end of the first 1 ms step: Python, the
bindings, loading and the first step), stepping time for the rest, real-time factor, CPU use
during stepping (in cores: 1.0 = one core busy), peak RSS of the process, and what the sensors
produced. The robot stands still: joint_control=True takes the demo drive away and nothing
commands the joints (as in Gazebo, where no controller runs).
"""

import time

T0 = time.perf_counter()

import json  # noqa: E402
import resource  # noqa: E402
import sys  # noqa: E402

import lagrange  # noqa: E402


def cpu() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return r.ru_utime + r.ru_stime


def main() -> int:
    robot, world, profile, seconds = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
    lidar = sys.argv[sys.argv.index("--lidar") + 1] if "--lidar" in sys.argv else "cpu"
    sim = lagrange.Simulation(
        robot,
        world=None if world == "none" else world,
        profile=profile,
        lidar=lidar,
        joint_control=True,
    )
    t_loaded = time.perf_counter()
    sim.step_until(0.001)
    t_first = time.perf_counter()
    c0 = cpu()
    sim.step_until(seconds)
    t_end = time.perf_counter()
    c1 = cpu()
    stepping = t_end - t_first
    names = [s["name"] for s in sim.signals()]
    sensors = sorted({n.split("/")[1] for n in names if n.startswith("sensor/")})
    out = {
        "sim": "lagrange",
        "simulated_s": sim.time,
        "load_s": t_loaded - T0,
        "startup_s": t_first - T0,
        "stepping_s": stepping,
        "rtf": (seconds - 0.001) / stepping,
        "cpu_cores": (c1 - c0) / stepping,
        "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        "sensors": sensors,
        "lidar": lidar,
    }
    for name in ("front_lidar", "top_lidar"):
        signal = f"sensor/{name}/ranges"
        if signal in names:
            ranges = sim.read(signal)
            out[f"{name}_rays_per_scan"] = len(ranges)
            out[f"{name}_finite_returns"] = sum(1 for r in ranges if r == r and r < 1e30)
    bodies = sim.bodies()
    if "base_link" in bodies and "rotor" in " ".join(names):
        out["height_at_end_m"] = sim.read("body/base_link/pose")[2]
    if "sensor/front_camera/image" in names:
        img = sim.image("sensor/front_camera/image") if hasattr(sim, "image") else None
        out["camera_image_bytes"] = len(img) if img is not None else None
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
