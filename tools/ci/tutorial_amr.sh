#!/usr/bin/env bash
# Runs docs/guide/tutorial-amr.md headless, step by step, and fails when a step does not do what
# the tutorial says (roadmap 0.2-H9: "the tutorial runs in CI"). The viewer steps are left out;
# the Nav2 step runs when Nav2 is installed.
# Usage (from the colcon workspace root, after building): tools/ci/tutorial_amr.sh
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
step() { echo; echo "=== $* ==="; }
fail() { echo "FAILED: $*" >&2; exit 1; }

# Step 1: what is installed.
step "1. lagrange info"
ros2 run lagrange_server lagrange info

# Step 2: the profile, checked and drawn.
profile="$(ros2 pkg prefix lagrange_profiles)/share/lagrange_profiles/examples/amr_sample_urdf.yaml"
step "2. validate and graph"
ros2 run lagrange_server lagrange validate "$profile"
ros2 run lagrange_server lagrange graph "$profile" >"$dir/wiring.dot"
grep -q '"rigid_body/mujoco' "$dir/wiring.dot" || fail "graph has no physics module"

# Step 3: run it with ROS 2, drive it from /cmd_vel, record it.
step "3. run, drive over ROS 2, record"
setsid ros2 run lagrange_server lagrange run "$profile" \
    --listen "unix:$dir/sim.sock" --record "$dir/run.mcap" >"$dir/server.log" 2>&1 &
server=$!
# shellcheck disable=SC2329  # called by the EXIT trap
stop() {
    # SIGTERM, not SIGINT: a background job of a non-interactive shell ignores SIGINT.
    kill -TERM -- "-$server" 2>/dev/null || return 0
    for _ in $(seq 20); do
        kill -0 -- "-$server" 2>/dev/null || return 0
        sleep 0.5
    done
    kill -KILL -- "-$server" 2>/dev/null || true
}
trap 'stop; echo "--- server log ---"; tail -n 30 "$dir/server.log"' EXIT

timeout 30 ros2 topic echo --once /odom nav_msgs/msg/Odometry >/dev/null ||
    fail "no /odom within 30 s"
timeout 15 ros2 topic echo --once /scan sensor_msgs/msg/LaserScan >/dev/null ||
    fail "no /scan"
timeout 15 ros2 topic echo --once /imu sensor_msgs/msg/Imu >/dev/null || fail "no /imu"
# Straight ahead at 0.4 m/s: /odom's forward speed must settle close to it. Discovery and the
# publisher's start take a moment, so poll for up to 10 s rather than read once.
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.4}}}" >/dev/null &
publisher=$!
followed=false
for _ in $(seq 10); do
    sleep 1
    speed="$(timeout 10 ros2 topic echo --once --field twist.twist.linear.x /odom \
        nav_msgs/msg/Odometry | head -n 1)"
    echo "forward speed while commanded 0.4 m/s: $speed m/s"
    if awk -v v="$speed" 'BEGIN { exit !(v > 0.35 && v < 0.45) }'; then
        followed=true
        break
    fi
done
kill "$publisher" 2>/dev/null || true
wait "$publisher" 2>/dev/null || true
$followed || fail "the robot did not follow /cmd_vel"
stop
trap - EXIT
size="$(stat -c %s "$dir/run.mcap")"
echo "recorded $size bytes"
[[ $size -gt 100000 ]] || fail "the recording is too small"

# Step 4: switch the sensors off; ground-truth substitutes take their place.
step "4. a substituted domain"
sed 's/sensors:      { enabled: true }/sensors:      { enabled: false, substitute: ground_truth }/' \
    "$profile" >"$dir/no_sensors.yaml"
ros2 run lagrange_server lagrange validate "$dir/no_sensors.yaml" \
    --robot "$(ros2 pkg prefix lagrange_assets)/share/lagrange_assets/robots/amr_diffdrive/amr_diffdrive.urdf"
ros2 run lagrange_server lagrange graph "$dir/no_sensors.yaml" \
    --robot "$(ros2 pkg prefix lagrange_assets)/share/lagrange_assets/robots/amr_diffdrive/amr_diffdrive.urdf" |
    grep -q 'substitute' || fail "graph shows no substitute"

# Step 5: the module of docs/guide/writing-a-module.md.
step "5. your own module"
ros2 run lagrange_tutorial odometer_demo 5
if ros2 run lagrange_tutorial odometer_demo --wrong-unit 2>"$dir/wrong.txt"; then
    fail "a wrong unit was accepted"
fi
grep -q UnitMismatch "$dir/wrong.txt" || fail "no UnitMismatch reported"
cat "$dir/wrong.txt"

# Step 6: Nav2.
step "6. Nav2"
if ros2 pkg prefix nav2_simple_commander >/dev/null 2>&1; then
    "$(dirname "$0")/nav2_demo.sh" 1
else
    echo "skipped: Nav2 is not installed (sudo apt install ros-lyrical-navigation2)"
fi

echo
echo "tutorial: all steps passed"
