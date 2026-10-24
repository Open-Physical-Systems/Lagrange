#!/usr/bin/env bash
# Performance benchmarks (roadmap H6): runs bench_core and bench_lidar three times each and
# compares the medians with tests/benchmarks/baseline.json (docs/design/benchmarks.md).
# Usage (from the colcon workspace root, after a RelWithDebInfo build):
#   tools/ci/benchmarks.sh MACHINE [OUT_DIR]
# MACHINE is a key of baseline.json. When it has no baseline yet, the results are printed and
# kept in OUT_DIR, and `check_benchmarks.py --record` can add them. MAX_SLOWDOWN (default 1.25)
# is the factor below the baseline that fails.
set -euo pipefail

machine="$1"
out="${2:-benchmark-results}"
mkdir -p "$out"
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
bin=install/lagrange_benchmarks/lib/lagrange_benchmarks
baseline=install/lagrange_benchmarks/share/lagrange_benchmarks/baseline.json

for bench in bench_core bench_lidar; do
    "$bin/$bench" --benchmark_repetitions=3 --benchmark_report_aggregates_only=true \
        --benchmark_out="$out/$bench.json" --benchmark_out_format=json
done

status=0
"$bin/check_benchmarks.py" --baseline "$baseline" --machine "$machine" \
    --max-slowdown "${MAX_SLOWDOWN:-1.25}" \
    "$out/bench_core.json" "$out/bench_lidar.json" || status=$?
if [[ $status -eq 2 ]]; then
    echo "no baseline for $machine yet; results are in $out" >&2
    exit 0
fi
exit "$status"
