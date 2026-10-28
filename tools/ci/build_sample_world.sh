#!/usr/bin/env bash
# Builds the 40 km sample world from open data (roadmap §8, huge-worlds.md §4) and checks that it
# streams: terrain from the Copernicus DEM GLO-30 (heights on the EGM2008 geoid, made ellipsoidal
# with PROJ's EGM96 grid: under a metre apart here), buildings and roads from OpenStreetMap for
# the central 4 km, centred on the PX4 home in Zurich that the drone samples use. Nothing it
# writes is committed: the data's licences ask for the notices in the world's ATTRIBUTION.md.
# Needs network access (AWS open data, the Overpass API) and about 700 MB of disk.
# Usage (from the source tree, after building): tools/ci/build_sample_world.sh [OUT_DIR]
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

out="${1:-$HOME/.cache/lagrange/worlds/zurich_40km}"
cache="${XDG_CACHE_HOME:-$HOME/.cache}/lagrange/downloads"
mkdir -p "$cache"
lat=47.397742
lon=8.545594
geoid=/usr/share/proj/egm96_15.gtx  # proj-data
[ -f "$geoid" ] || { echo "FAILED: $geoid missing: apt install proj-data" >&2; exit 1; }

dem="$cache/Copernicus_DSM_COG_10_N47_00_E008_00_DEM.tif"
if [ ! -s "$dem" ]; then
    echo "downloading the Copernicus DEM tile N47 E008 (about 40 MB)"
    curl -fsSL -o "$dem.part" \
        "https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_N47_00_E008_00_DEM/Copernicus_DSM_COG_10_N47_00_E008_00_DEM.tif"
    mv "$dem.part" "$dem"
fi

osm="$cache/zurich_4km.osm"
if [ ! -s "$osm" ]; then
    echo "downloading buildings and roads of the central 4 km from the Overpass API"
    # 2 km either way: 0.01797 degrees of latitude, 0.02656 of longitude at 47.4 N.
    bbox="47.379772,8.519034,47.415712,8.572154"
    curl -fsSL -A "lagrange_build_world (https://github.com/Open-Physical-Systems/Lagrange)" \
        -o "$osm.part" --data-urlencode \
        "data=[out:xml][timeout:180];(way[\"building\"]($bbox);way[\"highway\"]($bbox););(._;>;);out body;" \
        https://overpass-api.de/api/interpreter
    mv "$osm.part" "$osm"
fi

rm -rf "$out"
time ros2 run lagrange_geospatial lagrange_build_world "$out" --name zurich_40km \
    --origin "$lat,$lon,ground" --size 40960 \
    --dem "$dem" --geoid "$geoid" --osm "$osm" \
    --fine-tile 512 --fine-cell 16 --collision-cell 32 --coarse-tile 4096 --coarse-cell 128 \
    --attribution "Elevation: produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved. The organisations in charge of the Copernicus programme by law or by delegation do not incur any liability for any use of the Copernicus WorldDEM-30." \
    --attribution "Geoid: EGM96 15' grid of PROJ's proj-data (US National Geospatial-Intelligence Agency), used for the Copernicus heights' EGM2008 geoid: they differ by under a metre here."
du -sh "$out"

echo "checking that it streams: the AMR at the origin, 20 s headless"
python3 - "$out/zurich_40km.usda" <<'PY'
import sys
import lagrange
sim = lagrange.Simulation(lagrange.sample_robot(), world=sys.argv[1])
sim.step_until(20.0)
pose = sim.read("body/base_link/pose")
print(f"after 20 s the robot is at {pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f} m")
assert abs(pose[2]) < 5.0, "the robot is on the terrain, not under or above it"
PY
echo "sample world ok: $out"
