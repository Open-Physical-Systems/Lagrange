#!/usr/bin/env bash
# Runs docs/guide/tutorial-environment.md headless, step by step, and fails when a step does not
# do what the tutorial says: fog and rain on the AMR's lidars, a fog bank from a field, wind from
# a CFD case, a power line next to the quadcopter's magnetometer, and the AMR's Wi-Fi through
# the warehouse wall.
# Usage (from the source tree, after building): tools/ci/tutorial_environment.sh
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
trap 'rm -r "$dir"' EXIT
step() { echo; echo "=== $* ==="; }

step "1. the site and the weather"
ros2 run lagrange_server lagrange validate profiles/examples/amr_fog.yaml \
    profiles/examples/quad_x500_power_line.yaml

step "2. fog and rain on the lidars"
python3 - "$dir" <<'PY'
import math
import sys
import lagrange

def returns(profile: str | None) -> int:
    sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world(),
                              profile=profile)
    sim.step_until(1.0)
    ranges = sim.read("sensor/front_lidar/ranges")
    return sum(1 for r in ranges if math.isfinite(r))

clear = returns(None)
fog = returns(lagrange.sample_profile("amr_fog"))
print(f"front lidar returns: {clear} in clear air, {fog} in 60 m fog")
assert fog < clear, (fog, clear)
PY

step "3. a fog bank from a field"
python3 - "$PWD" <<'PY'
import math
import pathlib
import sys
import lagrange

root = pathlib.Path(sys.argv[1])
text = (root / "profiles/examples/amr_fog.yaml").read_text()
bank = root / "assets/fields/warehouse_fog_bank.nvdb"
profile = pathlib.Path(sys.argv[1]) / "build" / "tutorial_fog_bank.yaml"
profile.write_text(text.replace("visibility: 60,", f"fog_field: {bank},"))
sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world(),
                          profile=str(profile))
sim.step_until(1.0)
lost = sum(1 for r in sim.read("sensor/front_lidar/ranges") if not math.isfinite(r))
profile.unlink()
print(f"beams lost in the fog bank: {lost}")
assert lost > 50, lost
PY

step "4. wind from a CFD case"
python3 - "$dir" <<'PY'
import pathlib
import sys

# A stand-in for an OpenFOAM case's ASCII C and U: 1 m cells, wind from the west growing with
# height (what `postProcess -func writeCellCentres` and the solver would write).
out = pathlib.Path(sys.argv[1])
cells = [(x + 0.5, y + 0.5, z + 0.5) for z in range(10) for y in range(-10, 10)
         for x in range(-10, 10)]
def write(name: str, values: list[tuple[float, float, float]]) -> None:
    body = "\n".join(f"({a} {b} {c})" for a, b, c in values)
    (out / name).write_text(
        "FoamFile\n{\n    version 2.0;\n    format ascii;\n    class volVectorField;\n"
        f"    object {name};\n}}\ninternalField nonuniform List<vector>\n{len(values)}\n(\n"
        f"{body}\n)\n;\nboundaryField\n{{\n}}\n")
write("C", cells)
write("U", [(1.0 + 0.5 * z, 0.0, 0.0) for (_, _, z) in cells])
PY
ros2 run lagrange_fields lagrange_foam_to_vdb "$dir/C" "$dir/U" "$dir/wind.nvdb" --voxel 1 \
    | tee "$dir/foam.txt"
grep -q "4000 cells in 4000 voxels" "$dir/foam.txt"

step "5. a power line and the magnetometer"
python3 - "$PWD" <<'PY'
import sys
import lagrange

root = sys.argv[1]
def field(profile: str) -> list[list[float]]:
    sim = lagrange.Simulation(root + "/assets/robots/quad_x500/quad_x500_emi.usda",
                              world=root + "/assets/worlds/power_line/power_line.usda",
                              profile=profile)
    sim.step_until(5.0)
    samples = []
    for i in range(100):
        sim.step_until(5.0 + (i + 1) * 0.01)
        samples.append(sim.read("sensor/magnetometer/field"))
    return samples

def spread(samples: list[list[float]]) -> float:
    xs = [s[0] for s in samples]
    mean = sum(xs) / len(xs)
    return (sum((x - mean) ** 2 for x in xs) / len(xs)) ** 0.5

on = spread(field(root + "/profiles/examples/quad_x500_power_line.yaml"))
print(f"magnetometer x spread next to the line: {on * 1e6:.2f} µT")
assert 5e-6 < on < 8e-6, on
PY

step "6. Wi-Fi through the warehouse wall"
python3 - "$PWD" <<'PY'
import math
import sys
import lagrange

root = sys.argv[1]
sim = lagrange.Simulation(root + "/assets/robots/amr_diffdrive/amr_diffdrive_wifi.usda",
                          world=root + "/assets/worlds/warehouse_small/warehouse_small.usda")
sim.step_until(1.0)
rx = sim.read("link/wifi/rx_power")[0]
snr = sim.read("link/wifi/snr")[0]
loss = sim.read("link/wifi/packet_error_rate")[0]
print(f"received {10 * math.log10(rx / 1e-3):.1f} dBm, SNR {10 * math.log10(snr):.1f} dB, "
      f"packets lost {loss:.3g}, CAN state {sim.read('can/can0/state')[0]:.0f}")
assert -90.0 < 10 * math.log10(rx / 1e-3) < -40.0, rx
assert sim.read("link/wifi/up")[0] == 1.0
PY

echo
echo "tutorial ok"
