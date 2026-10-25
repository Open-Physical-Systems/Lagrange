#!/usr/bin/env bash
# The v0.4 mission with ArduPilot: ArduCopter SITL flies the sample quadcopter in lockstep with
# lagrange_server through ArduPilot's JSON interface: take-off, a 10 m square at 5 m, landing.
# Usage (from the source tree, after building, with ArduCopter SITL built in $ARDUPILOT_DIR):
#   tools/ci/ardupilot_mission.sh [--wind]
# ARDUPILOT_DIR (default ~/ardupilot) must hold ArduPilot (Copter-4.7.2) built with
# `./waf configure --board sitl && ./waf copter`, and pymavlink in $ARDUPILOT_DIR/.venv.
# shellcheck disable=SC1091
set -euo pipefail

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
OUT=${OUT:-/tmp/lagrange-ardupilot}
mkdir -p "$OUT"
copter=$ARDUPILOT_DIR/build/sitl/bin/arducopter
python=$ARDUPILOT_DIR/.venv/bin/python
[ -x "$copter" ] || {
    echo "ArduCopter SITL not built in $ARDUPILOT_DIR (./waf configure --board sitl && ./waf copter)" >&2
    exit 2
}
[ -x "$python" ] || python=python3

set +u
source /opt/ros/lyrical/setup.bash
source install/setup.bash
set -u
params=$(ros2 pkg prefix lagrange_mavlink)/share/lagrange_mavlink/ardupilot/lagrange_quad_x500.parm

site=(--air-density 1.225 --geo-origin "47.397742,8.545594,488")
robot=(--robot assets/robots/quad_x500/quad_x500.usda --world assets/worlds/open_field/open_field.usda)
socket=${XDG_RUNTIME_DIR:-/tmp}/lagrange-ardupilot.sock
if [ "${1:-}" = "--wind" ]; then
    ros2 run lagrange_server lagrange run profiles/examples/quad_x500_gusty.yaml "${robot[@]}" \
        "${site[@]}" --mavlink ardupilot --no-ros --listen "unix:$socket" >"$OUT/server.log" 2>&1 &
else
    ros2 run lagrange_server lagrange_server "${robot[@]}" "${site[@]}" --mavlink ardupilot \
        --no-ros --listen "unix:$socket" >"$OUT/server.log" 2>&1 &
fi
server=$!
copter_pid=""
# shellcheck disable=SC2329  # called by the EXIT trap
cleanup() {
    for pid in $copter_pid $server; do kill -TERM "$pid" 2>/dev/null || true; done
    sleep 2
    for pid in $copter_pid $server; do kill -KILL "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
sleep 4
work=$(mktemp -d)  # ArduPilot keeps its parameters (eeprom.bin) in its working directory
(cd "$work" && exec "$copter" --model JSON:127.0.0.1 --home "47.397742,8.545594,488,0" -w \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$params" \
    >"$OUT/ardupilot.log" 2>&1) &
copter_pid=$!
status=0
mission_timeout=${MISSION_TIMEOUT:-300}
timeout $((mission_timeout + 60)) "$python" "$(dirname "$params")/fly_square_mission.py" --timeout "$mission_timeout" |
    tee "$OUT/mission.log" || status=$?
echo "logs in $OUT"
exit "$status"
