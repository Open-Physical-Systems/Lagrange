#!/usr/bin/env bash
# The realism checks: every model checked against measured data or a published reference
# (docs/validation/report.md, "Since v1.0"), in one run, with their numbers in OUT/realism.md.
# Exit 1 when any check fails. Needs a workspace built with tests, sourced (install/setup.bash);
# the NeuroBEM step downloads 627 MB once into ~/.cache/lagrange (skip it with --offline).
#
#   tools/ci/realism.sh [--offline] [OUT_DIR]   (default realism-results)
set -uo pipefail
here="$(cd "$(dirname "$0")/../.." && pwd)"
offline=false
if [[ "${1:-}" == "--offline" ]]; then
    offline=true
    shift
fi
out="${1:-realism-results}"
mkdir -p "$out"
build="${LAGRANGE_BUILD_DIR:-$here/build}"
summary="$out/realism.md"
failed=0
{
    echo "# Realism checks, $(date -u +%Y-%m-%dT%H:%MZ), $(git -C "$here" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo
} >"$summary"

# run NAME PATTERN COMMAND...: runs a check, keeps its log, and puts the lines matching PATTERN
# (the numbers it prints) in the summary.
run() {
    local name="$1" pattern="$2"
    shift 2
    local log="$out/$name.log"
    if "$@" >"$log" 2>&1; then
        echo "## $name: passed" >>"$summary"
    else
        echo "## $name: FAILED (exit $?)" >>"$summary"
        failed=1
    fi
    { echo; echo '```'; grep -E "$pattern" "$log" || true; echo '```'; echo; } >>"$summary"
}

run "Battery: LG HG2 pulses and drive cycles (Kollmeyer et al.)" "^HG2" \
    "$build/lagrange_electrical/test_battery_dataset" --gtest_also_run_disabled_tests
run "Drone under a drone (Kiran et al. 2025)" "alone|constant voltage" \
    "$build/lagrange_server/test_server" --gtest_filter='DownwashPair.*Lower*:DownwashPair.AtConstant*'
run "BlueROV2, Fossen's model (von Benzon et al. 2022)" "^BlueROV2" \
    "$build/lagrange_server/test_server" --gtest_filter='Water.TheBlueRov2*'
run "Pushing (Yu et al. 2016) and frictionless sliding, every engine" "push on|OK|FAILED" \
    "$build/lagrange_server/lagrange_physics_conformance" --gtest_filter='*Pushed*:*Frictionless*'
run "Thermal camera radiometry (Incropera Table 12.1, Planck, Swinbank)" "OK|FAILED" \
    "$build/lagrange_sensors/test_thermal_camera"
run "Wind tunnel wall friction (Prandtl-Schlichting; a known defect, measured)" "plate along" \
    "$build/lagrange_aerodynamics/test_lbm" --gtest_also_run_disabled_tests \
    --gtest_filter='*AFlatPlatesFriction*'
if [[ "$offline" == false ]]; then
    run "Quadrotor rotors against NeuroBEM flights (Bauersfeld et al. 2021)" "^all|NeuroBEM" \
        "$here/tools/validation/neurobem.sh"
else
    echo "## Quadrotor rotors against NeuroBEM flights: skipped (--offline)" >>"$summary"
fi
cat "$summary"
exit "$failed"
