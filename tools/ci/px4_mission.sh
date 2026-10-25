#!/usr/bin/env bash
# The v0.4 mission (docs/design/robot-classes.md §5): PX4 SITL flies the sample quadcopter in
# lockstep with lagrange_server through the MAVLink bridge: take-off, a 10 m square at 5 m,
# landing. Exit 0 when every waypoint was reached and it landed.
# Usage (from the colcon workspace root, after building, with PX4 SITL built in $PX4_DIR):
#   tools/ci/px4_mission.sh [--wind]
#   --wind: the gusty profile (8 m/s from the west at 10 m, Dryden turbulence)
# PX4_DIR (default ~/PX4-Autopilot) must hold PX4 v1.17.0 built with `make px4_sitl_default`,
# and pymavlink in $PX4_DIR/.venv (PX4's Tools/setup/requirements.txt).
# shellcheck disable=SC1091
set -euo pipefail

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
OUT=${OUT:-/tmp/lagrange-px4}
mkdir -p "$OUT"
build=$PX4_DIR/build/px4_sitl_default
python=$PX4_DIR/.venv/bin/python
[ -x "$build/bin/px4" ] || {
    echo "PX4 SITL not built in $PX4_DIR (make px4_sitl_default)" >&2
    exit 2
}
[ -x "$python" ] || python=python3

set +u
source /opt/ros/lyrical/setup.bash
source install/setup.bash
set -u
share=$(ros2 pkg prefix lagrange_mavlink)/share/lagrange_mavlink/px4
cp "$share/22100_lagrange_quad_x500" "$build/etc/init.d-posix/airframes/"
mkdir -p "$build/rootfs"

site=(--air-density 1.225 --geo-origin "47.397742,8.545594,488")  # PX4 SITL's default home
robot=(--robot assets/robots/quad_x500/quad_x500.usda --world assets/worlds/open_field/open_field.usda)
socket=${XDG_RUNTIME_DIR:-/tmp}/lagrange-px4.sock
if [ "${1:-}" = "--wind" ]; then
    ros2 run lagrange_server lagrange run profiles/examples/quad_x500_gusty.yaml "${robot[@]}" \
        "${site[@]}" --mavlink px4 --no-ros --listen "unix:$socket" >"$OUT/server.log" 2>&1 &
else
    ros2 run lagrange_server lagrange_server "${robot[@]}" "${site[@]}" --mavlink px4 --no-ros \
        --listen "unix:$socket" >"$OUT/server.log" 2>&1 &
fi
server=$!
px4=""
# shellcheck disable=SC2329  # called by the EXIT trap
cleanup() {
    for pid in $px4 $server; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 2
    for pid in $px4 $server; do kill -KILL "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
sleep 4
(cd "$build/rootfs" && PX4_SYS_AUTOSTART=22100 PX4_SIM_MODEL=lagrange_quad_x500 \
    exec ../bin/px4 -d ../etc >"$OUT/px4.log" 2>&1) &
px4=$!
status=0
timeout 360 "$python" bridges/mavlink/px4/fly_square_mission.py --timeout 300 | tee "$OUT/mission.log" ||
    status=$?
echo "logs in $OUT"
exit "$status"
