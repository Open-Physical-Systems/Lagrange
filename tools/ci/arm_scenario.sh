#!/usr/bin/env bash
# The v0.3 arm acceptance scenario (docs/design/robot-classes.md §4): MoveIt 2 runs the sample
# arm's pick-and-place cycle at full speed. With the undersized shoulder and elbow motors
# (arm6_undersized.overlay.yaml) the windings heat until derating limits torque and the
# trajectory controller aborts a move on its tolerance; with electrical off (the ideal torque
# source) the same cycles all complete. Runs at 8x real time; about ten minutes in all.
# Usage (from the colcon workspace root, after building): tools/ci/arm_scenario.sh
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
dir="$(mktemp -d)"
launch=""
# shellcheck disable=SC2329  # called by the EXIT trap and between runs
stop() {
    [[ -n "$launch" ]] || return 0
    kill -TERM -- "-$launch" 2>/dev/null || true
    for _ in $(seq 30); do
        kill -0 -- "-$launch" 2>/dev/null || break
        sleep 0.5
    done
    kill -KILL -- "-$launch" 2>/dev/null || true
    launch=""
}
trap 'stop; rm -rf "$dir"' EXIT

# run NAME ELECTRICAL CYCLES: the workcell and pick_place; prints pick_place's exit status.
run() {
    if command -v fastdds >/dev/null; then
        fastdds shm clean >/dev/null 2>&1 || true
    fi
    setsid ros2 launch lagrange_demos arm6_workcell.launch.py rtf:=8 motors:=undersized \
        "electrical:=$2" >"$dir/$1.launch.log" 2>&1 &
    launch=$!
    sleep 10
    local status=0
    timeout 1800 ros2 run lagrange_demos pick_place --cycles "$3" >"$dir/$1.log" 2>&1 || status=$?
    stop
    echo "$status"
}

echo "== electrical on: the windings heat until a move fails"
status=$(run hot on 300)
cycles=$(grep -c '^cycle' "$dir/hot.log" || true)
echo "pick_place exit $status after $cycles cycles"
grep -m1 -A1 'tolerances failed' "$dir/hot.launch.log" || true
if [[ "$status" != 3 ]] || ! grep -q 'tolerances failed' "$dir/hot.launch.log"; then
    echo "FAILED: expected a tolerance violation once the motors derate" >&2
    exit 1
fi

echo "== electrical off: the same cycles, ten more than ran above, all complete"
status=$(run ideal off $((cycles + 10)))
echo "pick_place exit $status after $(grep -c '^cycle' "$dir/ideal.log" || true) cycles"
if [[ "$status" != 0 ]] || grep -q 'tolerances failed' "$dir/ideal.launch.log"; then
    echo "FAILED: with the ideal torque source every cycle should complete" >&2
    exit 1
fi
echo "arm scenario: passed"
