#!/usr/bin/env bash
# Installs Lagrange on Ubuntu 26.04 (amd64) from the debs attached to a GitHub release, and
# ROS 2 Lyrical first when it is not there yet:
#
#   curl -fsSL https://raw.githubusercontent.com/Open-Physical-Systems/Lagrange/main/packaging/install.sh | bash
#
# Options (environment variables):
#   LAGRANGE_VERSION=1.2.0    a release (default: the latest)
#   LAGRANGE_DEBS=DIR         install the debs in DIR instead of downloading (packaging/debs)
#   LAGRANGE_YES=1            do not ask before installing
# It uses sudo for apt. Nothing is built: it takes a few minutes and about 3 GB of disk.
set -euo pipefail

repo="Open-Physical-Systems/Lagrange"
say() { printf '\033[1;33m==>\033[0m %s\n' "$*"; }
fail() {
    printf '\033[1;31merror:\033[0m %s\n' "$*" >&2
    exit 1
}
sudo=""
if [ "$(id -u)" -ne 0 ]; then
    command -v sudo >/dev/null || fail "run as root, or install sudo"
    sudo="sudo"
fi

# 1. The platform: Lagrange's packages are built for Ubuntu 26.04 on amd64 only.
# shellcheck disable=SC1091
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "resolute" ] ||
    fail "Lagrange's packages are for Ubuntu 26.04 (resolute); this is ${PRETTY_NAME:-unknown}. Build from source instead: https://open-physical-systems.github.io/Lagrange/guide/install.html"
[ "$(dpkg --print-architecture)" = "amd64" ] ||
    fail "the released packages are amd64; on $(dpkg --print-architecture), build from source"

# Piped from curl, stdin is the script: ask on the terminal, and only when there is one.
if [ "${LAGRANGE_YES:-}" != "1" ] && { : </dev/tty; } 2>/dev/null; then
    read -r -p "Install ROS 2 Lyrical (if missing) and Lagrange with apt? [Y/n] " answer </dev/tty
    case "${answer:-y}" in [Yy]*) ;; *) exit 1 ;; esac
fi

export DEBIAN_FRONTEND=noninteractive
$sudo apt-get update -qq
$sudo apt-get install -y -qq curl ca-certificates python3-pip >/dev/null

# 2. ROS 2 Lyrical, from ROS's apt source (the official install guide's steps).
if [ ! -f /opt/ros/lyrical/setup.bash ]; then
    say "installing ROS 2 Lyrical"
    $sudo apt-get install -y -qq software-properties-common locales >/dev/null
    $sudo add-apt-repository -y universe >/dev/null
    tag="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        grep -F '"tag_name"' | head -1 | cut -d'"' -f4)"
    [ -n "$tag" ] || fail "could not find ROS's apt source release"
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${tag}/ros2-apt-source_${tag}.${VERSION_CODENAME}_all.deb"
    $sudo dpkg -i /tmp/ros2-apt-source.deb >/dev/null
    $sudo apt-get update -qq
    $sudo apt-get install -y -qq ros-lyrical-ros-base >/dev/null
fi

# 3. Lagrange's debs: from the release, checked against its SHA-256 list.
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
if [ -n "${LAGRANGE_DEBS:-}" ]; then
    debs="$(realpath "$LAGRANGE_DEBS")"
    say "installing the debs in $debs"
else
    version="${LAGRANGE_VERSION:-}"
    if [ -z "$version" ]; then
        version="$(curl -fsSL "https://api.github.com/repos/$repo/releases/latest" |
            grep -F '"tag_name"' | head -1 | cut -d'"' -f4)"
        version="${version#v}"
    fi
    [ -n "$version" ] || fail "could not find the latest release of $repo"
    base="https://github.com/$repo/releases/download/v$version"
    archive="lagrange-$version-debs-amd64.tar"
    say "downloading Lagrange $version"
    if ! curl -fL --progress-bar -o "$work/$archive" "$base/$archive" ||
        ! curl -fsSL -o "$work/$archive.sha256" "$base/$archive.sha256"; then
        fail "release v$version has no $archive; pick one that has with LAGRANGE_VERSION, or build the debs (docs/guide/install.md, B)"
    fi
    (cd "$work" && sha256sum -c --quiet "$archive.sha256") || fail "the download's checksum is wrong"
    mkdir "$work/debs"
    tar -C "$work/debs" -xf "$work/$archive"
    debs="$work/debs"
fi
ls "$debs"/ros-lyrical-lagrange*.deb >/dev/null 2>&1 || fail "no ros-lyrical-lagrange*.deb in $debs"
say "installing Lagrange"
# apt resolves their dependencies from Ubuntu and ROS.
$sudo apt-get install -y -qq "$debs"/*.deb >/dev/null
# Two Python packages Ubuntu does not have, at the versions Lagrange is tested with.
constraints="$debs/pip-constraints.txt"
if [ ! -f "$constraints" ]; then
    printf 'mcap==1.5.0\ncoacd==1.0.14\n' >"$work/pip-constraints.txt"
    constraints="$work/pip-constraints.txt"
fi
$sudo env PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install -q --root-user-action=ignore -c "$constraints" mcap coacd

# 4. A check: the command answers.
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
set -u
ros2 run lagrange_server lagrange info --version >/dev/null || fail "lagrange did not start"
say "Lagrange $(ros2 run lagrange_server lagrange info --version | awk '{print $2}') is installed"
cat <<'EOF'

  source /opt/ros/lyrical/setup.bash      (add it to ~/.bashrc to have it in every shell)
  lagrange amr                            the sample AMR in its warehouse, with the viewer
  lagrange quad                           a quadcopter
  lagrange samples                        everything else

  Tutorials: https://open-physical-systems.github.io/Lagrange/guide/tutorial-amr.html
EOF
