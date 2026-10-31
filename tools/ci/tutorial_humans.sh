#!/usr/bin/env bash
# Runs docs/guide/tutorial-humans.md headless, step by step, and fails when a step does not do what
# the tutorial says.
# Usage (from the source tree, after building): tools/ci/tutorial_humans.sh
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

work="$(mktemp -d)"
export ROS_DOMAIN_ID=$((120 + $$ % 80))
server=""
cleanup() {
    if [[ -n "$server" ]]; then
        kill -TERM "$server" 2>/dev/null || true
        wait "$server" 2>/dev/null || true
    fi
    rm -rf "$work"
}
trap cleanup EXIT

echo "== 1. a crowd: the server and a headless viewer"
lagrange people --no-ros -- --headless --frames 5 --wait-ms 60000 >"$work/crowd.log" 2>&1
grep -q '6 people in the run' "$work/crowd.log"

echo "== 2. one person at a time"
lagrange --server-only empty --listen "unix:$work/s.sock" >"$work/server.log" 2>&1 &
server=$!
[[ "$(lagrange spawn assets/humans/adult.usda --at 3,0 --name ana)" == "/ana" ]]
[[ "$(lagrange spawn assets/humans/walker.usda --name bo)" == "/bo" ]]
entities="$(ros2 service call /get_entities simulation_interfaces/srv/GetEntities \
    '{filters: {categories: [{category: 2}]}}')"
grep -q "/ana" <<<"$entities"
grep -q "/bo" <<<"$entities"
lagrange delete /ana
grep -q 'spawned ana' "$work/server.log"
grep -q 'a person' "$work/server.log"

echo "the humans tutorial passed"
