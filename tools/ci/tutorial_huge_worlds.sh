#!/usr/bin/env bash
# Runs docs/guide/tutorial-huge-worlds.md headless, step by step, and fails when a step does not
# do what the tutorial says. The viewer is left out; the 40 km sample (network) only with
# --network.
# Usage (from the source tree, after building): tools/ci/tutorial_huge_worlds.sh [--network]
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
trap 'rm -rf "$dir"' EXIT
step() { echo; echo "=== $* ==="; }

step "1. a world in tiles"
python3 - <<'PY'
import lagrange

sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_tiled_town(),
                          spawn=[1036, 1036])
sim.step_until(5.0)
count = sim.colliders()
print(f"{count} collision shapes from the tiles near the robot")
assert 50 < count < 400, count
again = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_tiled_town(),
                            spawn=[1036, 1036])
again.step_until(5.0)
assert again.state_hash() == sim.state_hash(), "the same path loads the same tiles"
PY

step "2. the floating origin"
python3 - <<'PY'
import lagrange

sim = lagrange.Simulation(lagrange.sample_robot(), spawn=[999.5, 0])
assert sim.region_origin() == [0.0, 0.0, 0.0], sim.region_origin()
sim.step_until(20.0)
origin = sim.region_origin()
print(f"the region origin is at {origin} m after 20 s")
assert origin[0] >= 1000.0 and origin[1] == 0.0, origin
PY

step "3. robots far apart"
echo "(C++: server/test/test_regions.cpp, run by colcon test)"

step "4. a world from open data"
python3 - "$dir/area.osm" <<'PY'
import math
import sys

# Three buildings and a road around the origin, as an OpenStreetMap XML file.
lat0, lon0 = 35.6812, 139.7671
m_lat = 1 / 111_320
m_lon = 1 / (111_320 * math.cos(math.radians(lat0)))
nodes, ways, n = [], [], 0


def node(x, y):
    global n
    n += 1
    nodes.append(f"<node id='{n}' lat='{lat0 + y * m_lat:.9f}' lon='{lon0 + x * m_lon:.9f}'/>")
    return n


for i, (x, y, h) in enumerate([(-60, 40, 12), (20, 60, 30), (80, -50, 18)]):
    ring = [node(x, y), node(x + 30, y), node(x + 30, y + 20), node(x, y + 20)]
    refs = "".join(f"<nd ref='{r}'/>" for r in ring + ring[:1])
    ways.append(f"<way id='{100 + i}'>{refs}<tag k='building' v='yes'/>"
                f"<tag k='height' v='{h}'/></way>")
a, b = node(-200, 0), node(200, 0)
ways.append(f"<way id='200'><nd ref='{a}'/><nd ref='{b}'/><tag k='highway' v='primary'/></way>")
with open(sys.argv[1], "w") as f:
    f.write("<?xml version='1.0'?><osm version='0.6'>" + "".join(nodes + ways) + "</osm>")
PY
ros2 run lagrange_geospatial lagrange_build_world "$dir/my_world" --name my_world \
    --origin 35.6812,139.7671,40 --size 1024 --osm "$dir/area.osm"
[ -f "$dir/my_world/ATTRIBUTION.md" ] || { echo "FAILED: no ATTRIBUTION.md" >&2; exit 1; }
python3 - "$dir/my_world/my_world.usda" <<'PY'
import sys
import lagrange

sim = lagrange.Simulation(lagrange.sample_robot(), world=sys.argv[1], spawn=[-20, -20])
sim.step_until(5.0)
count = sim.colliders()
print(f"{count} collision shapes: terrain fields and the three buildings")
assert count >= 4, count
PY
if [ "${1:-}" = "--network" ]; then
    tools/ci/build_sample_world.sh "$dir/zurich_40km"
fi

echo
echo "tutorial ok"
