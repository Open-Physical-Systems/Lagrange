#!/usr/bin/env bash
# Runs docs/guide/tutorial-drone.md headless, step by step, and fails when a step does not do
# what the tutorial says. The viewer is left out; the PX4 step runs when PX4 SITL is built in
# $PX4_DIR (default ~/PX4-Autopilot).
# Usage (from the source tree, after building): tools/ci/tutorial_drone.sh
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
step() { echo; echo "=== $* ==="; }
fail() { echo "FAILED: $*" >&2; exit 1; }

step "1. the quadcopter in three files"
ros2 run lagrange_server lagrange_import assets/robots/quad_x500/quad_x500.urdf "$dir/quad.usda" \
    --overlay assets/robots/quad_x500/quad_x500.overlay.yaml
[ "$(grep -c RotorAPI_1 "$dir/quad.usda")" -ge 4 ] || fail "the rotors are not in the USD"

step "2. the site"
ros2 run lagrange_server lagrange validate profiles/examples/quad_x500_hover.yaml \
    profiles/examples/quad_x500_gusty.yaml

step "3. hover"
python3 - <<'PY'
import lagrange
sim = lagrange.Simulation(lagrange.sample_quadcopter(), world=None,
                          profile=lagrange.sample_profile("quad_x500_hover"))
start = sim.read("body/base_link/pose")[2]
sim.step_until(8.0)
height = sim.read("body/base_link/pose")[2] - start
power = sim.read("battery/battery/power")[0]
pressure = sim.read("sensor/barometer/pressure")[0]
print(f"height {height:.3f} m, {power:.0f} W, {pressure:.0f} Pa")
assert abs(height - 2.0) < 0.01, height
assert 150.0 < power < 250.0, power
assert 95000.0 < pressure < 96000.0, pressure
PY

step "4. gusty wind"
python3 - <<'PY'
import math
import lagrange
sim = lagrange.Simulation(lagrange.sample_quadcopter(), world=None,
                          profile=lagrange.sample_profile("quad_x500_gusty"))
start = sim.read("body/base_link/pose")
sim.step_until(15.0)
pose = sim.read("body/base_link/pose")
drift = math.hypot(pose[0] - start[0], pose[1] - start[1])
print(f"drift {drift:.2f} m, airspeed {sim.read('body/base_link/airspeed')[0]:.1f} m/s")
assert drift < 0.3, drift
PY

step "5. PX4 flies it"
if [ -x "${PX4_DIR:-$HOME/PX4-Autopilot}/build/px4_sitl_default/bin/px4" ]; then
    tools/ci/px4_mission.sh || fail "the PX4 mission"
else
    echo "PX4 SITL is not built in ${PX4_DIR:-$HOME/PX4-Autopilot}: step left out"
fi

echo
echo "tutorial ok"
