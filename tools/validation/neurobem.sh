#!/usr/bin/env bash
# Lagrange's rotor models against the NeuroBEM flights (rotor_aero.md §9): downloads the dataset
# (Bauersfeld et al., RSS 2021; https://rpg.ifi.uzh.ch/NeuroBEM.html, 627 MB, no licence stated,
# so it is fetched here and never committed), checks its SHA-256, and replays the 13 test
# segments the authors list through lagrange_rotor_replay. Exit 1 when the total force RMS error
# passes the bound for either fidelity.
#
#   tools/validation/neurobem.sh [DATA_DIR]   (default ~/.cache/lagrange/neurobem)
set -euo pipefail
here="$(cd "$(dirname "$0")/../.." && pwd)"
dir="${1:-${XDG_CACHE_HOME:-$HOME/.cache}/lagrange/neurobem}"
base="https://download.ifi.uzh.ch/rpg/NeuroBEM"
zip_sha="a3e2e83d156b48a95c346c4fd9b3ded0b62781331dc613f1cb9ab285be2b36a8"
# Bounds: what this commit reads (rotor_aero.md §9) plus 5 %.
coefficients_bound=1.53  # N, 1.458 measured
bem_bound=1.18           # N, 1.122 measured
mkdir -p "$dir"
cd "$dir"
if [[ ! -f processed_data.zip ]] || ! echo "$zip_sha  processed_data.zip" | sha256sum -c --quiet; then
    curl -fL -o processed_data.zip "$base/processed_data.zip"
    echo "$zip_sha  processed_data.zip" | sha256sum -c --quiet
fi
[[ -f testset.txt ]] || curl -fsSL -o testset.txt "$base/testset.txt"
[[ -d processed_data ]] || unzip -q processed_data.zip
mapfile -t test < <(sed 's|^|processed_data/merged_|; s|$|.csv|' testset.txt)
replay="$(ros2 pkg prefix lagrange_rotor_aero)/lib/lagrange_rotor_aero/lagrange_rotor_replay"
total() { awk '/^all/ { for (i = 1; i <= NF; ++i) if ($i == "F") print $(i + 1) }'; }
# The coefficients fitted to the other segments (CT 0.17928, CP 0.00215), and the propeller's
# blade angle +2.0 deg likewise (rotor_aero.md §9).
c=$("$replay" --radius 0.064770 --ct 0.17928 --cp 0.00215 "${test[@]}" | tee /dev/stderr | total)
b=$("$replay" --propeller "$here/assets/propellers/neurobem_5in_3blade_ccw.prop.yaml" --pitch 2.0 \
    "${test[@]}" | tee /dev/stderr | total)
echo "NeuroBEM test flights: coefficients F $c N (bound $coefficients_bound), bem F $b N (bound $bem_bound)"
awk -v c="$c" -v b="$b" -v cb="$coefficients_bound" -v bb="$bem_bound" 'BEGIN { exit !(c <= cb && b <= bb) }'
