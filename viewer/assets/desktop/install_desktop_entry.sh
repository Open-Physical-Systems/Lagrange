#!/usr/bin/env bash
# Puts Lagrange in the desktop's app grid and dock: its desktop entry and icons, for this user
# (~/.local/share) or, with --system, for everyone (/usr/share, through sudo). Clicking it runs
# lagrange-desktop beside this script, which sources a ROS 2 setup and opens the viewer.
#
#   install_desktop_entry.sh [--setup SETUP.bash] [--system] [--uninstall]
#
#   --setup FILE   the setup the entry sources: default the setup.bash of the install this
#                  script is in when there is one (a colcon install), else
#                  /opt/ros/lyrical/setup.bash (the debs)
#   --system       for every user, in /usr/share (sudo when not root)
#   --uninstall    remove the entry and the icons again
set -euo pipefail

id="io.github.Open_Physical_Systems.Lagrange"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setup=""
system=0
uninstall=0
while [ $# -gt 0 ]; do
    case "$1" in
    --setup)
        setup="${2:?--setup takes a file}"
        shift 2
        ;;
    --system)
        system=1
        shift
        ;;
    --uninstall)
        uninstall=1
        shift
        ;;
    -h | --help)
        sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *)
        echo "install_desktop_entry.sh: unknown option $1 (--help)" >&2
        exit 2
        ;;
    esac
done

run=()
if [ "$system" = 1 ]; then
    data="/usr/share"
    [ "$(id -u)" -eq 0 ] || run=(sudo)
else
    data="${XDG_DATA_HOME:-$HOME/.local/share}"
fi
apps="$data/applications"
icons="$data/icons/hicolor"
sizes=(48 128 256 512)

refresh() {
    if command -v update-desktop-database >/dev/null; then
        "${run[@]}" update-desktop-database -q "$apps" || true
    fi
    # Only refresh an icon cache that is already there: a new cache in ~/.local/share would
    # hide icons that later installs add without updating it.
    if [ -f "$icons/icon-theme.cache" ] && command -v gtk-update-icon-cache >/dev/null; then
        "${run[@]}" gtk-update-icon-cache -q -f "$icons" || true
    fi
    if [ -d "$icons" ]; then
        "${run[@]}" touch "$icons" || true
    fi
}

if [ "$uninstall" = 1 ]; then
    "${run[@]}" rm -f "$apps/$id.desktop" "$icons/scalable/apps/$id.svg"
    for n in "${sizes[@]}"; do
        "${run[@]}" rm -f "$icons/${n}x${n}/apps/$id.png"
    done
    # The icon folders this made, when nothing else is in them.
    for n in "${sizes[@]}"; do
        "${run[@]}" rmdir "$icons/${n}x${n}/apps" "$icons/${n}x${n}" 2>/dev/null || true
    done
    "${run[@]}" rmdir "$icons/scalable/apps" "$icons/scalable" "$icons" "$data/icons" \
        2>/dev/null || true
    refresh
    echo "removed Lagrange's desktop entry from $apps"
    exit 0
fi

if [ -z "$setup" ]; then
    # The install this script is in: <prefix>/share/lagrange_viewer/desktop, with
    # <prefix>/setup.bash (merged) or <prefix>/../setup.bash (colcon's isolated layout).
    prefix="$(cd "$here/../../.." && pwd)"
    for candidate in "$prefix/setup.bash" "$prefix/../setup.bash"; do
        if [ -f "$candidate" ] && [ "$prefix" != "/opt/ros/lyrical" ]; then
            setup="$(realpath "$candidate")"
            break
        fi
    done
    [ -n "$setup" ] || setup="/opt/ros/lyrical/setup.bash"
fi
[ -f "$setup" ] || {
    echo "install_desktop_entry.sh: no setup file at $setup" >&2
    exit 1
}
setup="$(realpath "$setup")"
launcher="$here/lagrange-desktop"
[ -x "$launcher" ] || {
    echo "install_desktop_entry.sh: $launcher is missing or not executable" >&2
    exit 1
}

# The Exec line quotes each path; the few characters the desktop entry spec would need
# escaped are refused instead.
for path in "$launcher" "$setup"; do
    case "$path" in
    *[\"\`\$\\%]*)
        echo "install_desktop_entry.sh: cannot put a path with \" \` \$ \\ or % in a desktop entry: $path" >&2
        exit 1
        ;;
    esac
done
exec_line="\"$launcher\" \"$setup\""

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
sed "s|@EXEC@|$exec_line|" "$here/lagrange.desktop.in" >"$tmp/$id.desktop"
if command -v desktop-file-validate >/dev/null; then
    desktop-file-validate --no-hints "$tmp/$id.desktop"
fi

"${run[@]}" install -d "$apps" "$icons/scalable/apps"
"${run[@]}" install -m 644 "$tmp/$id.desktop" "$apps/$id.desktop"
"${run[@]}" install -m 644 "$here/icons/lagrange.svg" "$icons/scalable/apps/$id.svg"
for n in "${sizes[@]}"; do
    "${run[@]}" install -d "$icons/${n}x${n}/apps"
    "${run[@]}" install -m 644 "$here/icons/lagrange-$n.png" "$icons/${n}x${n}/apps/$id.png"
done
refresh
echo "Lagrange is in the app grid ($apps/$id.desktop), sourcing $setup"
