#!/usr/bin/env bash
# Builds and tests the plugin template (tools/templates/lagrange_plugin_template) in a scratch
# colcon workspace on top of this one, as a user who copies it would (RFC 0008 §6).
# Needs the workspace built and sourced (install/setup.bash).
set -euo pipefail
here="$(cd "$(dirname "$0")/../.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/src"
cp -r "$here/tools/templates/lagrange_plugin_template" "$work/src/my_lagrange_plugin"
cd "$work"
colcon build --event-handlers console_direct- > build.log 2>&1 || { cat build.log; exit 1; }
colcon test --event-handlers console_direct- > /dev/null 2>&1 || true
colcon test-result --verbose
