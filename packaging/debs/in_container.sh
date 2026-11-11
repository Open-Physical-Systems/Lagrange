#!/usr/bin/env bash
# shellcheck disable=SC1091,SC2012  # ROS setup scripts; bloom names its own .debs
# Runs inside the ros:lyrical container of build_debs.sh: bloom and debian/rules for each
# package in dependency order. /src is the source tree (read-only), /out receives the debs.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends python3-bloom python3-rosdep fakeroot debhelper \
    dh-python python3-colcon-common-extensions python3-pip build-essential git

# A writable copy of the tracked files: bloom writes debian/ into each package.
mkdir -p /work/src/lagrange
git config --global --add safe.directory /src
git -C /src ls-files -z | (cd /src && tar --null -T - -cf -) | tar -C /work/src/lagrange -xf -
cd /work

# rosdep: the project's extra keys, and our own packages under their future deb names.
python3 - <<'PY' > /tmp/lagrange-self.yaml
import pathlib, re
for p in sorted(pathlib.Path("/work/src/lagrange").rglob("package.xml")):
    name = re.search(r"<name>([^<]+)</name>", p.read_text()).group(1)
    print(f"{name}:\n  ubuntu: [ros-lyrical-{name.replace('_', '-')}]")
PY
echo "yaml file:///work/src/lagrange/packaging/rosdep/lagrange.yaml" \
    > /etc/ros/rosdep/sources.list.d/10-lagrange.list
echo "yaml file:///tmp/lagrange-self.yaml" > /etc/ros/rosdep/sources.list.d/11-lagrange-self.list
rosdep update --rosdistro lyrical
# Build dependencies of everything at once (pip keys into this throwaway system Python).
PIP_BREAK_SYSTEM_PACKAGES=1 PIP_CONSTRAINT=/work/src/lagrange/packaging/pip-constraints.txt \
    rosdep install --from-paths src --ignore-src -y --rosdistro lyrical

# ROS's setup scripts read unset variables.
set +u
source /opt/ros/lyrical/setup.bash
set -u
built=0
for pkg in $(colcon list --base-paths src --topological-order --names-only --packages-skip-regex 'lagrange_example.*'); do
    path="$(colcon list --base-paths src --packages-select "$pkg" --paths-only)"
    echo "=== $pkg ($path)"
    # Built by an earlier run into /out: install it and go on (a resumed build).
    existing="$(ls /out/ros-lyrical-"${pkg//_/-}"_*.deb 2>/dev/null | head -1 || true)"
    if [ -n "$existing" ]; then
        apt-get install -y "$existing" >/dev/null
        set +u
        source /opt/ros/lyrical/setup.bash
        set -u
        built=$((built + 1))
        continue
    fi
    (
        cd "$path"
        bloom-generate rosdebian --ros-distro lyrical >/dev/null
        # pip keys (mcap, coacd) are not Debian packages: bloom writes their pip names as
        # dependencies, which apt cannot satisfy. Drop them; the image and the install guide
        # pip-install them from packaging/pip-constraints.txt.
        python3 /work/src/lagrange/packaging/debs/drop_pip_depends.py debian/control \
            /work/src/lagrange/packaging/rosdep/lagrange.yaml
        # bloom's rules are debhelper compat 9, where dh builds serially unless told: with
        # --parallel it takes the job count from DEB_BUILD_OPTIONS (OpenUSD: hours at -j1).
        sed -i 's/^\tdh \$@ -v /\tdh $@ -v --parallel /' debian/rules
        fakeroot debian/rules binary >/tmp/"$pkg".log 2>&1 || {
            tail -40 /tmp/"$pkg".log
            exit 1
        }
    )
    deb="$(ls "$(dirname "$path")"/ros-lyrical-*.deb | head -1)"
    apt-get install -y "./$deb" >/dev/null || dpkg -i "$deb"
    mv "$(dirname "$path")"/ros-lyrical-*.deb /out/
    # The next package's CMake finds this one through the ROS environment.
    set +u
    source /opt/ros/lyrical/setup.bash
    set -u
    built=$((built + 1))
done
echo "built $built packages"
