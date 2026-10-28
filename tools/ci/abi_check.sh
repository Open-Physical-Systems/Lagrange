#!/usr/bin/env bash
# Checks the plugin C ABI against its 1.0 baseline with libabigail (RFC 0007): the sample
# plugin, built for ABI major 1 with debug information, carries every type of lagrange_plugin.h,
# and abidiff reports any change to them other than an addition. Without libabigail
# (apt install abigail-tools) it says so and exits 0: core/plugin_abi/test/test_abi_layout.c
# pins the same sizes and offsets in every build anyway.
# The first run with libabigail writes the baseline (core/plugin_abi/abi/plugin_abi_1.0.xml):
# commit it.
# Usage (from the source tree, after building): tools/ci/abi_check.sh
set -euo pipefail

if ! command -v abidiff >/dev/null || ! command -v abidw >/dev/null; then
    echo "abi_check: libabigail is not installed (apt install abigail-tools); skipped"
    exit 0
fi
plugin="build/lagrange_core/libsample_plugin_current.so"
[ -f "$plugin" ] || plugin="$(find build/lagrange_core -name 'libsample_plugin_current*.so' | head -1)"
[ -n "$plugin" ] && [ -f "$plugin" ] || { echo "FAILED: build lagrange_core first" >&2; exit 1; }
baseline="core/plugin_abi/abi/plugin_abi_1.0.xml"
current="$(mktemp)"
trap 'rm -f "$current"' EXIT
abidw --headers-dir core/plugin_abi/include --out-file "$current" "$plugin"
if [ ! -f "$baseline" ]; then
    mkdir -p "$(dirname "$baseline")"
    cp "$current" "$baseline"
    echo "abi_check: wrote the baseline $baseline; commit it"
    exit 0
fi
# abidiff exits with bit 3 (8) set for changes that break the ABI, bit 2 (4) for any change.
set +e
abidiff --headers-dir1 core/plugin_abi/include --headers-dir2 core/plugin_abi/include \
    "$baseline" "$current"
status=$?
set -e
if (( status & 8 )); then
    echo "FAILED: the plugin ABI changed incompatibly (RFC 0007: append only within major 1)" >&2
    exit 1
fi
echo "abi_check: compatible with plugin ABI 1.0"
