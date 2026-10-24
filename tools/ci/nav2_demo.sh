#!/usr/bin/env bash
# The v0.2 acceptance test (criterion 1): the sample AMR in the small warehouse, Nav2 driving it
# through the patrol waypoints for a number of simulated minutes. Fails if any waypoint is
# missed or the robot touches the scenery.
# Usage (from the colcon workspace root, after building): tools/ci/nav2_demo.sh [minutes]
# LIDAR=auto|cpu|gpu picks where the lidars cast (default auto: the CPU); CONTROL=ros2_control|
# builtin who drives the wheels (default ros2_control).
set -euo pipefail

minutes="${1:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

# Shared-memory segments left by killed ROS processes (Fast DDS) disturb discovery between new
# ones: remove the ones no live process holds.
if command -v fastdds >/dev/null; then
    fastdds shm clean >/dev/null 2>&1 || true
fi

launch=""
# shellcheck disable=SC2329  # called by the EXIT trap and between attempts
stop() {
    [[ -n "$launch" ]] || return 0
    # SIGTERM, not SIGINT: a background job of a non-interactive shell ignores SIGINT.
    kill -TERM -- "-$launch" 2>/dev/null || return 0
    for _ in $(seq 30); do
        kill -0 -- "-$launch" 2>/dev/null || return 0
        sleep 1
    done
    kill -KILL -- "-$launch" 2>/dev/null || true
}
trap stop EXIT

# One attempt: the stack in its own process group, then the patrol. Returns the patrol's status.
attempt() {
    local dir log status=0
    dir="$(mktemp -d)"
    log="$dir/launch.log"
    setsid ros2 launch lagrange_demos amr_warehouse.launch.py viewer:=false \
        endpoint:="unix:$dir/demo.sock" lidar:="${LIDAR:-auto}" control:="${CONTROL:-ros2_control}" \
        >"$log" 2>&1 &
    launch=$!
    # Real time, plus start-up and the round that is under way when the time is up.
    timeout $((minutes * 90 + 300)) ros2 run lagrange_demos patrol --minutes "$minutes" ||
        status=$?
    if [[ $status -ne 0 ]]; then
        echo "--- launch log (last 80 lines) ---"
        tail -n 80 "$log"
    fi
    stop
    launch=""
    return "$status"
}

# Exit 3 from the patrol means the stack never became ready (ROS discovery between a dozen
# processes starting at once occasionally fails on one machine); that alone is retried, once,
# with fresh processes. A missed waypoint or a collision (exit 1) is never retried.
status=0
attempt || status=$?
if [[ $status -eq 3 ]]; then
    echo "--- the Nav2 stack did not come up; relaunching once ---"
    fastdds shm clean >/dev/null 2>&1 || true
    status=0
    attempt || status=$?
fi
exit "$status"
