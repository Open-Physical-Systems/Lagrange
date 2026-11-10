#!/usr/bin/env bash
# Runs docs/guide/tutorial-heat-and-emi.md headless, step by step, and fails when a step does not
# do what the tutorial says. OpenFOAM is not run: step 3 writes the case only.
# Usage (from the source tree, after building): tools/ci/tutorial_heat_and_emi.sh
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

work="$(mktemp -d)"
export ROS_DOMAIN_ID=$((120 + $$ % 80))
trap 'rm -rf "$work"' EXIT
robot=assets/robots/led_heatsink/led_heatsink.urdf

echo "== 1. where the heat goes"
cat >"$work/profile.yaml" <<'EOF'
schema_version: 1
stage: led_heatsink.urdf
domains:
  rigid_body:   { enabled: true, backend: mujoco }
  electrical:   { enabled: false, substitute: ideal }
  thermal:      { enabled: true, ambient_temperature: 298.15, dt: 1.0 }
  aerodynamics: { enabled: false, substitute: zero }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: false, substitute: zero }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: false, substitute: ground_truth }
EOF
python3 - "$robot" "$work/profile.yaml" <<'EOF'
import sys

import lagrange

sim = lagrange.Simulation(sys.argv[1], world=None, profile=sys.argv[2])
sim.step_until(600.0)
led = sim.read("thermal/led_left/temperature")[0]
board = sim.read("thermal/board/temperature")[0]
sink = sim.read("thermal/sink/temperature")[0]
print(f"LED {led:.1f} K, board {board:.1f} K, sink {sink:.1f} K")
assert led > board > sink > 298.15 + 5.0, (led, board, sink)
EOF

echo "== 1b. the Thermal shading, headless"
lagrange "$robot" --ambient-temperature 298.15 --no-ros -- --headless --shading thermal \
    --frames 30 --wait-ms 60000 >"$work/viewer.log" 2>&1
grep -q '"shading": 3' "$work/viewer.log"

echo "== 1c, 2, 4. on ROS 2: the heat and the compass examples"
python3 tools/ci/check_examples.py heat compass

echo "== 3. the OpenFOAM case, written"
lagrange foam-cht "$robot" --ambient 298.15 --radiation --out "$work/case" --write-only >/dev/null
for file in constant/regionProperties constant/air/radiationProperties 0.regions/board/T \
    system/board/fvOptions system/snappyHexMeshDict Allrun; do
    [[ -e "$work/case/$file" ]] || { echo "missing $file"; exit 1; }
done

echo "the heat and EMI tutorial passed"
