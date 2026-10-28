#!/usr/bin/env bash
# Builds a sample world from open data (roadmap §8, huge-worlds.md §4) and checks that it streams:
# terrain from the Copernicus DEM GLO-30 (heights on the EGM2008 geoid, made ellipsoidal with
# PROJ's EGM96 grid), buildings and roads from OpenStreetMap for the centre. Places:
#   zurich (default): 40 km round the PX4 home that the drone samples use, OpenStreetMap for the
#     central 4 km (EGM96 and EGM2008 are under a metre apart there); about 700 MB of disk
#   tokyo: 10 km round Nishi-Shinjuku (the Tokyo Metropolitan Government Building), the car
#     samples' city, OpenStreetMap for the central 3 km, on flat ground at street level. The
#     Copernicus DEM is a surface model: among Shinjuku's towers its "terrain" carries the
#     rooftops (up to 233 m above the origin round the Government Building), which buries the
#     streets, while the ground there is within a few metres of flat. Street level, 68.5 m
#     above the ellipsoid, is the lowest DEM surface round the origin (an estimate)
# Nothing it writes is committed: the data's licences ask for the notices in the world's
# ATTRIBUTION.md. Needs network access (AWS open data, the Overpass API).
# Usage (from the source tree, after building):
#   tools/ci/build_sample_world.sh [--place zurich|tokyo] [OUT_DIR]
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

place=zurich
if [ "${1:-}" = "--place" ]; then
    place="${2:?--place needs zurich or tokyo}"
    shift 2
fi
case "$place" in
    zurich)
        name=zurich_40km lat=47.397742 lon=8.545594 size=40960 tile=N47_00_E008_00
        height=ground
        bbox="47.379772,8.519034,47.415712,8.572154" # 2 km either way at 47.4 N
        osm_note="the central 4 km"
        ;;
    tokyo)
        name=tokyo_shinjuku_10km lat=35.689568 lon=139.691717 size=10240 tile=
        height=68.5
        bbox="35.676093,139.675152,35.703043,139.708282" # 1.5 km either way at 35.69 N
        osm_note="the central 3 km"
        ;;
    *)
        echo "unknown place $place: zurich or tokyo" >&2
        exit 2
        ;;
esac
out="${1:-$HOME/.cache/lagrange/worlds/$name}"
cache="${XDG_CACHE_HOME:-$HOME/.cache}/lagrange/downloads"
mkdir -p "$cache"
geoid=/usr/share/proj/egm96_15.gtx  # proj-data
[ -f "$geoid" ] || { echo "FAILED: $geoid missing: apt install proj-data" >&2; exit 1; }

terrain=()
attribution=()
if [ -n "$tile" ]; then
    dem="$cache/Copernicus_DSM_COG_10_${tile}_DEM.tif"
    terrain=(--dem "$dem" --geoid "$geoid")
    attribution=(
        --attribution "Elevation: produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved. The organisations in charge of the Copernicus programme by law or by delegation do not incur any liability for any use of the Copernicus WorldDEM-30."
        --attribution "Geoid: EGM96 15' grid of PROJ's proj-data (US National Geospatial-Intelligence Agency), used for the Copernicus heights' EGM2008 geoid."
    )
    if [ ! -s "$dem" ]; then
        echo "downloading the Copernicus DEM tile $tile (about 40 MB)"
        curl -fsSL -o "$dem.part" \
            "https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_${tile}_DEM/Copernicus_DSM_COG_10_${tile}_DEM.tif"
        mv "$dem.part" "$dem"
    fi
fi

osm="$cache/${name}.osm"
if [ ! -s "$osm" ]; then
    echo "downloading buildings and roads of $osm_note from the Overpass API"
    curl -fsSL -A "lagrange_build_world (https://github.com/Open-Physical-Systems/Lagrange)" \
        -o "$osm.part" --data-urlencode \
        "data=[out:xml][timeout:180];(way[\"building\"]($bbox);way[\"highway\"]($bbox););(._;>;);out body;" \
        https://overpass-api.de/api/interpreter
    mv "$osm.part" "$osm"
fi

rm -rf "$out"
time ros2 run lagrange_geospatial lagrange_build_world "$out" --name "$name" \
    --origin "$lat,$lon,$height" --size "$size" "${terrain[@]}" --osm "$osm" \
    --fine-tile 512 --fine-cell 16 --collision-cell 32 --coarse-tile 4096 --coarse-cell 128 \
    "${attribution[@]}"
du -sh "$out"

echo "checking that it streams: the AMR at the origin, 20 s headless"
python3 - "$out/$name.usda" <<'PY'
import sys
import lagrange
sim = lagrange.Simulation(lagrange.sample_robot(), world=sys.argv[1])
sim.step_until(20.0)
pose = sim.read("body/base_link/pose")
print(f"after 20 s the robot is at {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f} m")
assert abs(pose[2]) < 5.0, "the robot is on the terrain, not under or above it"
PY
echo "sample world ok: $out"
