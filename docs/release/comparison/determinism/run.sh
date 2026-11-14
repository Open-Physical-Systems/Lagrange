#!/usr/bin/env bash
# Determinism check of docs/comparison.md: each engine runs the 200-box pile twice, in two
# separate processes, and the SHA-256 of every box's final pose is compared; then Lagrange's own
# `lagrange determinism` on the sample AMR with its sensors.
#
#   source /opt/ros/lyrical/setup.bash; source <lagrange install>/setup.bash
#   determinism/run.sh SCENARIOS_DIR VENV OUT.jsonl [SECONDS]   (docs/comparison.md: 1 s)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
scenarios="$1"
venv="$2"
out="$3"
seconds="${4:-5}"
export GZ_IP=127.0.0.1 GZ_PARTITION=lagrange_compare_det
for engine in lagrange-mujoco lagrange-jolt gazebo mujoco pybullet; do
    case "$engine" in
        mujoco | pybullet) py="$venv/bin/python" ;;
        *) py=python3 ;;
    esac
    for run in 1 2; do
        echo "$engine run $run" >&2
        timeout 1200 "$py" "$here/final_poses.py" "$engine" "$scenarios" "$seconds" 2>/dev/null | grep '^{' |
            python3 -c "import json,sys; r=json.loads(sys.stdin.read()); r['run']=$run; print(json.dumps(r))" >>"$out" || echo "$engine run $run: failed or timed out (1200 s)" >&2
    done
done
profile="$scenarios/profile_sensors.yaml"
echo "lagrange determinism $profile --seconds 60" >&2
{
    printf '{"engine": "lagrange determinism (CLI)", "command": "ros2 run lagrange_server lagrange determinism %s --seconds 60", "output": ' "$profile"
    ros2 run lagrange_server lagrange determinism "$profile" --seconds 60 2>/dev/null | tail -1 |
        python3 -c "import json,sys; print(json.dumps(sys.stdin.read().strip()), end='')"
    printf '}\n'
} >>"$out"
