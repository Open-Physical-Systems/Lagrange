#!/usr/bin/env bash
# Every measurement of docs/comparison.md, in order: the scenarios, the physics accuracy cases,
# the timed runs (interleaved, REPEATS each) and the determinism check, into OUT_DIR.
#
#   source /opt/ros/lyrical/setup.bash; source <lagrange install>/setup.bash
#   docs/release/comparison/run.sh OUT_DIR [REPEATS]
#
# Needs: Gazebo Jetty (ros-lyrical-ros-gz), the X3 UAV model from Fuel (make_scenarios.py says
# how), and two virtual environments made beside OUT_DIR:
#   python3 -m venv venv && venv/bin/pip install mujoco pybullet numpy
#   python3 -m venv venv312 && venv312/bin/pip install mujoco==3.12.0 numpy
# QUICK=1: a dry run (short runs, one repeat). Run it on a quiet machine: the timings are only
# as good as the machine's idleness.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
out="$(mkdir -p "$1" && cd "$1" && pwd)"
repeats="${2:-5}"
work="$out/work"
venv="${VENV:-$out/../venv}"
venv312="${VENV312:-$out/../venv312}"
mkdir -p "$work/scenes" "$work/perf"

{
    echo "date: $(date -Iseconds)"
    echo "host: $(lscpu | sed -n 's/^Model name: *//p'), $(nproc) threads, $(free -g | awk '/Mem:/ {print $2}') GB RAM"
    echo "gpu: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null || echo none)"
    echo "kernel: $(uname -r); os: $(lsb_release -ds 2>/dev/null)"
    echo "lagrange: $(ros2 run lagrange_server lagrange info --version 2>/dev/null) ($(git -C "$repo" describe --always --dirty))"
    echo "gazebo: gz-sim $(gz sim --versions 2>/dev/null)"
    echo "mujoco (standalone): $("$venv/bin/python" -c 'import mujoco; print(mujoco.__version__)') and $("$venv312/bin/python" -c 'import mujoco; print(mujoco.__version__)')"
    echo "pybullet: $("$venv/bin/python" -c 'from importlib.metadata import version; print(version("pybullet"))' 2>/dev/null)"
    echo "python: $(python3 --version)"
} >"$out/machine.txt"
cat "$out/machine.txt"

python3 "$here/accuracy/scenes.py" "$work/scenes" >"$out/accuracy_closed_forms.txt"
python3 "$here/accuracy/run_lagrange.py" "$work/scenes" "$out/accuracy_lagrange.json" >/dev/null 2>&1
GZ_IP=127.0.0.1 GZ_PARTITION=lagrange_compare_acc \
    python3 "$here/accuracy/run_gazebo.py" "$work/scenes" "$out/accuracy_gazebo.json" >/dev/null 2>&1
"$venv/bin/python" "$here/accuracy/run_mujoco_pybullet.py" "$work/scenes" "$out/accuracy_mujoco_pybullet.json" >/dev/null 2>&1
"$venv312/bin/python" "$here/accuracy/run_mujoco_pybullet.py" "$work/scenes" "$out/accuracy_mujoco312.json" >/dev/null 2>&1

python3 "$here/perf/make_scenarios.py" "$repo" "$work/perf"
python3 "$here/perf/run_all.py" "$repo" "$work/perf" "$out/performance.jsonl" \
    --venv "$venv" --venv312 "$venv312" --repeats "$repeats" ${QUICK:+--quick}
det_seconds=1
[[ -n "${QUICK:-}" ]] && det_seconds=0.3
"$here/determinism/run.sh" "$work/perf" "$venv" "$out/determinism.jsonl" "$det_seconds"

rm -rf "$work"
python3 "$here/summarize.py" "$out" >"$out/tables.txt"
echo "done: $out/tables.txt"
