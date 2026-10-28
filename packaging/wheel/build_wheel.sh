#!/usr/bin/env bash
# Builds the Python wheel of Lagrange (roadmap §9): the `lagrange` package and its compiled
# extension, for virtual environments on Ubuntu 26.04 where the ros-lyrical-lagrange-* debs
# (or a source build) are installed. The extension links the simulator's libraries and ROS
# Lyrical's, so the wheel is a platform wheel for that system, not a self-contained manylinux
# one: `pip install` it into a venv made with --system-site-packages after sourcing ROS (or
# with --target).
# Usage: packaging/wheel/build_wheel.sh PACKAGE_DIR OUT_DIR
#   PACKAGE_DIR: the installed package, e.g. install/lagrange_python/lib/python3.14/site-packages/lagrange
#   or /opt/ros/lyrical/lib/python3.14/site-packages/lagrange from the deb.
set -euo pipefail

source_dir="$(realpath "${1:?usage: build_wheel.sh PACKAGE_DIR OUT_DIR}")"
out="$(realpath -m "${2:?usage: build_wheel.sh PACKAGE_DIR OUT_DIR}")"
version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$(dirname "$0")/../../python/package.xml")"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/lagrange" "$out"
cp "$source_dir"/*.py "$source_dir"/_lagrange*.so "$work/lagrange/"
cat > "$work/setup.py" <<PY
from setuptools import setup
from setuptools.dist import Distribution


class BinaryDistribution(Distribution):
    """The package holds a compiled extension: a platform wheel."""

    def has_ext_modules(self):
        return True


setup(
    name="lagrange",
    version="$version",
    description="Lagrange robot simulator, headless from Python (needs the ROS 2 Lyrical install)",
    license="Apache-2.0",
    url="https://github.com/Open-Physical-Systems/Lagrange",
    packages=["lagrange"],
    package_data={"lagrange": ["_lagrange*.so"]},
    python_requires="==$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")').*",
    distclass=BinaryDistribution,
)
PY
printf '[build-system]\nrequires = ["setuptools"]\nbuild-backend = "setuptools.build_meta"\n' \
    > "$work/pyproject.toml"
(cd "$work" && python3 -m pip wheel --no-deps --no-build-isolation --use-pep517 -w "$out" . \
    >/dev/null)
ls "$out"/lagrange-"$version"-*.whl
