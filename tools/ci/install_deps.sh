#!/usr/bin/env bash
# Installs the build dependencies of the workspace inside a ros:lyrical container.
# Usage: tools/ci/install_deps.sh [extra apt packages...]
# Expects the repository at src/lagrange relative to the current directory.
# ROSDEP_SKIP_KEYS (space-separated) leaves run-time-only dependencies out, e.g. Nav2 for the
# demos on pull requests.
set -euo pipefail

repo="$PWD/src/lagrange"

apt-get update
apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-rosdep python3-pip "$@"

# Keys that are not in ros/rosdistro yet (see packaging/rosdep/lagrange.yaml).
echo "yaml file://$repo/packaging/rosdep/lagrange.yaml" \
    >/etc/ros/rosdep/sources.list.d/10-lagrange.list
rosdep update --rosdistro "$ROS_DISTRO"

# rosdep installs pip keys into the system Python. Ubuntu marks it as externally managed,
# which is fine to override in a throwaway container. Versions come from the constraints file.
PIP_BREAK_SYSTEM_PACKAGES=1 PIP_CONSTRAINT="$repo/packaging/pip-constraints.txt" \
    rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO" \
        --skip-keys "${ROSDEP_SKIP_KEYS:-}"
