#!/usr/bin/env python3
# ruff: noqa: PTH123  (comparison scripts: generated XML, units in text)
"""Runs every performance measurement of docs/comparison.md, interleaved, and appends one JSON
line per run (with the load average and GPU clocks at its start) to OUT.jsonl.

    source /opt/ros/lyrical/setup.bash; source <lagrange install>/setup.bash
    run_all.py REPO SCENARIOS_DIR OUT.jsonl --venv VENV --venv312 VENV312 [--repeats N] [--quick]

SCENARIOS_DIR comes from make_scenarios.py. VENV has mujoco (latest) and pybullet; VENV312 has
mujoco==3.12.0, the version Lagrange links. --quick: short runs, one repeat (a dry run).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
# Runs too long to repeat five times on a laptop: Gazebo (DART) needs about 20 minutes of wall
# time for the 5 s of the 200-box pile. Run once.
ONCE = {"boxes/gazebo": 1}
EGL_NVIDIA = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
CAMERA_TOPIC = "/world/amr_camera/model/amr_diffdrive/link/base_link/sensor/front_camera/image"
LIDAR_TOPIC = "/world/warehouse_small/model/amr_diffdrive/link/base_link/sensor/{}/scan"


def machine() -> dict[str, object]:
    state: dict[str, object] = {
        "loadavg": os.getloadavg(),
        "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    try:
        q = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=clocks.gr,clocks.sm,clocks.mem,temperature.gpu,pstate,power.draw",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        state["gpu"] = q.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        state["gpu"] = f"nvidia-smi failed: {e}"
    return state


def run(
    cmd: list[str], env: dict[str, str] | None = None, timeout: float = 1800
) -> dict[str, object]:
    start = time.perf_counter()
    try:
        p = subprocess.run(
            cmd, capture_output=True, text=True, env=env, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        return {"error": "timeout", "process_wall_s": time.perf_counter() - start}
    wall = time.perf_counter() - start
    lines = [line for line in p.stdout.splitlines() if line.startswith("{")]
    if p.returncode != 0 or not lines:
        return {"error": p.returncode, "stderr": p.stderr[-1500:], "process_wall_s": wall}
    out = json.loads(lines[-1])
    out["process_wall_s"] = wall
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("repo")
    ap.add_argument("scenarios")
    ap.add_argument("out")
    ap.add_argument("--venv", required=True)
    ap.add_argument("--venv312", required=True)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default="", help="comma-separated scenario prefixes")
    ap.add_argument(
        "--box-seconds",
        type=float,
        default=5.0,
        help="simulated time of the 200-box pile; other than 5 names its runs boxes_<S>s/",
    )
    ap.add_argument(
        "--resume", action="store_true", help="skip the (scenario, repeat) runs already in OUT"
    )
    a = ap.parse_args()
    repo, sc = Path(a.repo).resolve(), Path(a.scenarios).resolve()
    q = a.quick
    amr = str(repo / "assets/robots/amr_diffdrive/amr_diffdrive.urdf")
    wh = str(repo / "assets/worlds/warehouse_small/warehouse_small.urdf")
    quad = str(repo / "assets/robots/quad_x500/quad_x500.usda")
    t_amr = 2.0 if q else 60.0
    t_cam = 1.0 if q else 20.0
    t_box = 0.3 if q else a.box_seconds
    py = sys.executable
    bl = str(HERE / "bench_lagrange.py")
    bg = str(HERE / "bench_gazebo.py")
    bs = str(HERE / "bench_standalone.py")
    venv = str(Path(a.venv) / "bin/python")
    venv312 = str(Path(a.venv312) / "bin/python")
    gz_env = dict(
        os.environ,
        GZ_IP="127.0.0.1",
        GZ_PARTITION="lagrange_compare",
        __EGL_VENDOR_LIBRARY_FILENAMES=EGL_NVIDIA,
    )
    lidars = [LIDAR_TOPIC.format(n) + "=laserscan" for n in ("front_lidar", "top_lidar")]
    it = lambda s: str(round(s / 0.001))  # noqa: E731

    scenarios: list[tuple[str, list[str], dict[str, str] | None]] = [
        (
            "amr_physics/lagrange_mujoco",
            [py, bl, amr, wh, str(sc / "profile_physics.yaml"), str(t_amr)],
            None,
        ),
        (
            "amr_physics/lagrange_jolt",
            [py, bl, amr, wh, str(sc / "profile_physics_jolt.yaml"), str(t_amr)],
            None,
        ),
        ("amr_physics/gazebo", [py, bg, str(sc / "amr_physics.sdf"), it(t_amr)], gz_env),
        ("amr_physics/gazebo_1step", [py, bg, str(sc / "amr_physics.sdf"), "1"], gz_env),
        (
            "amr_sensors/lagrange_cpu_lidar",
            [py, bl, amr, wh, str(sc / "profile_sensors.yaml"), str(t_amr), "--lidar", "cpu"],
            None,
        ),
        (
            "amr_sensors/lagrange_gpu_lidar",
            [py, bl, amr, wh, str(sc / "profile_sensors.yaml"), str(t_amr), "--lidar", "gpu"],
            None,
        ),
        (
            "amr_sensors/gazebo_subscribed",
            [py, bg, str(sc / "amr_sensors.sdf"), it(t_amr), *lidars],
            gz_env,
        ),
        (
            "amr_sensors/gazebo_1step_subscribed",
            [py, bg, str(sc / "amr_sensors.sdf"), "1", *lidars],
            gz_env,
        ),
        (
            "amr_sensors/gazebo_no_subscriber",
            [py, bg, str(sc / "amr_sensors.sdf"), it(t_amr)],
            gz_env,
        ),
        (
            "camera/lagrange",
            [py, bl, str(sc / "amr_camera.urdf"), wh, str(sc / "profile_camera.yaml"), str(t_cam)],
            None,
        ),
        (
            "camera/gazebo_subscribed",
            [py, bg, str(sc / "gz_amr_camera.sdf"), it(t_cam), CAMERA_TOPIC + "=image"],
            gz_env,
        ),
        (
            "camera/gazebo_1step_subscribed",
            [py, bg, str(sc / "gz_amr_camera.sdf"), "1", CAMERA_TOPIC + "=image"],
            gz_env,
        ),
        (
            "boxes/lagrange_mujoco",
            [py, bl, str(sc / "boxes.xml"), "none", str(sc / "profile_boxes.yaml"), str(t_box)],
            None,
        ),
        (
            "boxes/lagrange_jolt",
            [
                py,
                bl,
                str(sc / "boxes.xml"),
                "none",
                str(sc / "profile_boxes_jolt.yaml"),
                str(t_box),
            ],
            None,
        ),
        ("boxes/gazebo", [py, bg, str(sc / "boxes.sdf"), it(t_box)], gz_env),
        ("boxes/gazebo_1step", [py, bg, str(sc / "boxes.sdf"), "1"], gz_env),
        (
            "quad/lagrange_full",
            [py, bl, quad, "none", str(sc / "profile_quad_full.yaml"), str(t_amr)],
            None,
        ),
        (
            "quad/lagrange_lean",
            [py, bl, quad, "none", str(sc / "profile_quad_lean.yaml"), str(t_amr)],
            None,
        ),
        (
            "quad/gazebo_x3",
            [py, str(HERE / "bench_gazebo_quad.py"), str(sc / "gz_quad.sdf"), str(t_amr)],
            gz_env,
        ),
        ("boxes/mujoco_latest", [venv, bs, "mujoco", str(sc), str(t_box)], None),
        ("boxes/mujoco_3.12", [venv312, bs, "mujoco", str(sc), str(t_box)], None),
        (
            "boxes/mujoco_3.12_lagrange_options",
            [venv312, bs, "mujoco-lagrange", str(sc), str(t_box)],
            None,
        ),
        ("boxes/pybullet", [venv, bs, "pybullet", str(sc), str(t_box)], None),
    ]
    if a.box_seconds != 5.0 and not q:
        tag = f"boxes_{a.box_seconds:g}s/"
        scenarios = [(n.replace("boxes/", tag), c, e) for n, c, e in scenarios]
        ONCE.update({k.replace("boxes/", tag): v for k, v in list(ONCE.items())})
        if a.box_seconds <= 1.0:
            ONCE.clear()
    if a.only:
        prefixes = a.only.split(",")
        scenarios = [s for s in scenarios if any(s[0].startswith(p) for p in prefixes)]
    repeats = 1 if q else a.repeats
    done: set[tuple[str, int]] = set()
    if a.resume and Path(a.out).exists():
        for line in Path(a.out).read_text(encoding="utf-8").splitlines():
            rec = json.loads(line)
            done.add((rec["scenario"], rec["repeat"]))
    with open(a.out, "a", encoding="utf-8") as out:
        for r in range(repeats):
            for name, cmd, env in scenarios:
                if (name, r) in done or (r >= ONCE.get(name, repeats)):
                    continue
                before = machine()
                result = run(cmd, env)
                record = {
                    "scenario": name,
                    "repeat": r,
                    "quick": q,
                    "machine": before,
                    "result": result,
                    "command": " ".join(cmd),
                }
                out.write(json.dumps(record) + "\n")
                out.flush()
                brief = {
                    k: result.get(k)
                    for k in ("rtf", "wall_s", "stepping_s", "startup_s", "error", "messages")
                }
                print(f"[{r}] {name}: {brief} load {before['loadavg'][0]:.2f}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
