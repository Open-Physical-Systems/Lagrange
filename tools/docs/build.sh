#!/usr/bin/env bash
# Builds the documentation site from docs/ into docs/_build/html (docs/conf.py), with the tools
# of tools/docs/requirements.txt in a virtual environment of their own (uv when installed, else
# python3 -m venv), kept in ~/.cache/lagrange/docs-venv.
#
#   tools/docs/build.sh            build
#   tools/docs/build.sh --check    build, and fail on any warning (what the docs workflow runs)
#   tools/docs/build.sh --serve    build, then serve it on http://localhost:8000 (PORT=... to change)
set -euo pipefail
here="$(cd "$(dirname "$0")/../.." && pwd)"
venv="${LAGRANGE_DOCS_VENV:-${XDG_CACHE_HOME:-$HOME/.cache}/lagrange/docs-venv}"
requirements="$here/tools/docs/requirements.txt"
out="$here/docs/_build/html"

mode="build"
case "${1:-}" in
    "") ;;
    --check | --serve) mode="${1#--}" ;;
    *)
        echo "usage: tools/docs/build.sh [--check | --serve]" >&2
        exit 2
        ;;
esac

# (Re)make the environment when it is missing or the requirements changed since.
stamp="$venv/.requirements"
if [[ ! -x "$venv/bin/sphinx-build" ]] || ! cmp -s "$requirements" "$stamp"; then
    echo "docs: installing the site's tools into $venv"
    rm -rf "$venv"
    mkdir -p "$(dirname "$venv")"
    if command -v uv >/dev/null 2>&1; then
        uv venv -q "$venv"
        uv pip install -q -p "$venv" -r "$requirements"
    else
        python3 -m venv "$venv"
        "$venv/bin/pip" install -q -r "$requirements"
    fi
    cp "$requirements" "$stamp"
fi

flags=(-q -b html)
if [[ "$mode" == "check" ]]; then
    flags+=(-W --keep-going)
fi
"$venv/bin/sphinx-build" "${flags[@]}" "$here/docs" "$out"
echo "docs: $out/index.html"

if [[ "$mode" == "serve" ]]; then
    port="${PORT:-8000}"
    echo "docs: serving on http://localhost:$port (Ctrl+C to stop)"
    exec python3 -m http.server "$port" --bind 127.0.0.1 --directory "$out"
fi
