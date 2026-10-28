#!/usr/bin/env bash
# The clean-machine install test (roadmap §9): a fresh ros:lyrical container with only the
# released debs installed runs the CLI, a headless simulation from Python, the determinism
# check and the migration tools. Builds the release image (Containerfile.release) first.
# Usage (from the source tree): packaging/test_install.sh DEB_DIR [TAG]
set -euo pipefail

debs="$(realpath "${1:?usage: packaging/test_install.sh DEB_DIR [TAG]}")"
tag="${2:-lagrange:release-test}"
cp packaging/pip-constraints.txt "$debs/"
podman build -f packaging/container/Containerfile.release -t "$tag" "$debs"

check() {
    echo "=== $1"
    podman run --rm "$tag" bash -c "source /opt/ros/lyrical/setup.bash && $2"
}
check "the CLI" "ros2 run lagrange_server lagrange info"
check "a run from Python" "python3 -c '
import lagrange
s = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
s.step_until(5.0)
p = s.read(\"body/base_link/pose\")
print(p)
assert abs(p[0]) + abs(p[1]) > 0.1, \"the robot drove\"
'"
check "determinism" "ros2 run lagrange_server lagrange determinism --seconds 2"
check "a streamed world" "python3 -c '
import lagrange
s = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_tiled_town(), spawn=[1036, 1036])
s.step_until(3.0)
assert s.colliders() > 50, s.colliders()
print(s.colliders(), \"colliders streamed\")
'"
check "the migration tools" "ros2 run lagrange_migrate migrate_profile --check \
    /opt/ros/lyrical/share/lagrange_profiles/examples/amr_sample_urdf.yaml"
check "the world builder" "ros2 run lagrange_geospatial lagrange_build_world --help | head -3"
echo
echo "clean-machine install ok: $tag"
