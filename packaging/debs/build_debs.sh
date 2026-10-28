#!/usr/bin/env bash
# Builds every Lagrange package as a ros-lyrical-* .deb with bloom, in a clean
# ros:lyrical-ros-base-resolute container (roadmap §9, binary releases). The packages are built
# in dependency order, each installed before the next, as the ROS buildfarm would; nothing is
# published (that is the owner's step: a rosdistro release or an apt repository).
# Usage (from the source tree): packaging/debs/build_debs.sh OUT_DIR
# About an hour on 8 cores (OpenUSD's vendor package is most of it); needs ~25 GB of disk.
set -euo pipefail

out="$(realpath -m "${1:?usage: packaging/debs/build_debs.sh OUT_DIR}")"
mkdir -p "$out"
image="docker.io/library/ros:lyrical-ros-base-resolute"
podman run --rm \
    -v "$PWD:/src:ro,z" -v "$out:/out:z" \
    -e DEB_BUILD_OPTIONS="parallel=${JOBS:-6} nocheck" \
    "$image" bash /src/packaging/debs/in_container.sh
ls -la "$out"
