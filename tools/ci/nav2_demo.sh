#!/usr/bin/env bash
# The v0.2 acceptance test: the sample AMR in the small warehouse, Nav2 driving it through the
# patrol waypoints. Fails if any waypoint is missed.
# Usage (from the colcon workspace root, after building): tools/ci/nav2_demo.sh [rounds]
set -euo pipefail

rounds="${1:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
log="$dir/launch.log"
# In its own process group, so that everything it started can be stopped together.
setsid ros2 launch lagrange_demos amr_warehouse.launch.py viewer:=false \
    endpoint:="unix:$dir/demo.sock" >"$log" 2>&1 &
launch=$!
# shellcheck disable=SC2329  # called by the EXIT trap
stop() {
    # SIGTERM, not SIGINT: a background job of a non-interactive shell ignores SIGINT.
    kill -TERM -- "-$launch" 2>/dev/null || return 0
    for _ in $(seq 30); do
        kill -0 -- "-$launch" 2>/dev/null || return 0
        sleep 1
    done
    kill -KILL -- "-$launch" 2>/dev/null || true
}
trap stop EXIT

# One round is six legs, about 75 s of sim time at real time.
status=0
timeout $((300 * rounds)) ros2 run lagrange_demos patrol --rounds "$rounds" || status=$?
if [[ $status -ne 0 ]]; then
    echo "--- launch log (last 80 lines) ---"
    tail -n 80 "$log"
fi
exit "$status"
