#!/usr/bin/env bash
# Puts a source build of Lagrange (a colcon install) in the desktop's app grid and dock, with its
# icon; clicking it sources that install's setup.bash and opens the viewer's launcher.
#
#   tools/desktop/install_desktop_entry.sh [--install DIR] [--system] [--uninstall]
#
#   --install DIR  the colcon install (default: install/ beside src/ or this repository, or
#                  the workspace of $COLCON_PREFIX_PATH's first entry)
#   --system, --uninstall   as for share/lagrange_viewer/desktop/install_desktop_entry.sh
# The debs' packaging/install.sh does the same for /opt/ros/lyrical.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
install_dir=""
pass=()
while [ $# -gt 0 ]; do
    case "$1" in
    --install)
        install_dir="${2:?--install takes a directory}"
        shift 2
        ;;
    -h | --help)
        sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        pass+=("$1")
        shift
        ;;
    esac
done

if [ -z "$install_dir" ]; then
    for candidate in "$repo/install" "$repo/../../install" "${COLCON_PREFIX_PATH%%:*}"; do
        if [ -n "$candidate" ] && [ -f "$candidate/setup.bash" ]; then
            install_dir="$candidate"
            break
        fi
    done
fi
[ -n "$install_dir" ] && [ -f "$install_dir/setup.bash" ] || {
    echo "install_desktop_entry.sh: no colcon install found; build first, or give --install DIR" >&2
    exit 1
}
install_dir="$(realpath "$install_dir")"

# The installed scripts (isolated or merged layout), else the ones in this checkout.
script="$repo/viewer/assets/desktop/install_desktop_entry.sh"
for candidate in "$install_dir/lagrange_viewer/share/lagrange_viewer/desktop" \
    "$install_dir/share/lagrange_viewer/desktop"; do
    if [ -x "$candidate/install_desktop_entry.sh" ]; then
        script="$candidate/install_desktop_entry.sh"
        break
    fi
done
exec "$script" --setup "$install_dir/setup.bash" "${pass[@]}"
