#!/usr/bin/env bash
# Runs docs/guide/tutorial-cars.md headless, step by step, and fails when a step does not do what
# the tutorial says. The viewer is left out; the Autoware step runs when lagrange_autoware is
# built (it needs the Autoware messages, docs/guide/autoware.md).
# Usage (from the source tree, after building): tools/ci/tutorial_cars.sh
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

dir="$(mktemp -d)"
server=""
cleanup() {
    if [ -n "$server" ] && kill -0 "$server" 2>/dev/null; then
        kill -TERM "$server"
        wait "$server" || true
    fi
    rm -rf "$dir"
}
trap cleanup EXIT
step() { echo; echo "=== $* ==="; }
fail() { echo "FAILED: $*" >&2; exit 1; }
# A ROS domain of its own, on this machine only.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

step "1. the sedan in two layers"
ros2 run lagrange_server lagrange_import assets/robots/sedan/sedan.urdf "$dir/sedan_base.usda"
grep -q 'def Camera "front_camera"' "$dir/sedan_base.usda" || fail "the URDF's camera is not in the USD"
[ "$(grep -cF 'apiSchemas = ["WheelAPI_1"]' assets/robots/sedan/sedan.usda)" -eq 4 ] \
    || fail "four WheelAPI_1 wheels"
ros2 run lagrange_server lagrange validate profiles/examples/sedan_city.yaml

step "2. roads from OpenDRIVE"
ros2 run lagrange_opendrive lagrange_opendrive_import assets/worlds/city_block/city_block.xodr \
    "$dir/roads.usda"
lanes="$(grep -c RoadLaneAPI_1 "$dir/roads.usda")"
echo "$lanes lanes"
[ "$lanes" -ge 20 ] || fail "the city block's lanes"

step "3. the demo drive, traffic and the sensors"
python3 - <<'PY'
import math
import lagrange
sim = lagrange.Simulation(lagrange.sample_sedan(), world=lagrange.sample_city())
sim.step_until(20.0)
pose = sim.read("body/base_link/pose")
speed = sim.read("vehicle/sedan/speed")[0]
radius = math.hypot(pose[0], pose[1])
agents = len(sim.read("traffic/agents")) // 8
detections = sim.read("sensor/front_radar/count")[0]
exposure = sim.read("sensor/front_camera/exposure_time")[0]
print(f"{speed:.2f} m/s, {radius:.1f} m from the junction's centre, {agents} traffic agents, "
      f"{detections:.0f} radar detections, exposure {exposure * 1000:.2f} ms")
assert abs(speed - 3.0) < 0.1, speed
assert 3.0 < radius < 6.0, radius
assert agents == 14, agents
assert detections > 0, detections
assert 0.0 < exposure <= 0.01, exposure
PY

step "4. a cut-in scenario"
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --scenario assets/scenarios/cut_in.xosc --no-ros --rtf 0 --listen "unix:$dir/s.sock" \
    > "$dir/scenario.log" 2>&1 &
server=$!
for _ in $(seq 60); do
    grep -q "scenario ended" "$dir/scenario.log" && break
    sleep 1
done
kill -TERM "$server"
wait "$server" || true
server=""
grep "scenario ended" "$dir/scenario.log" || fail "the scenario did not reach its stop trigger"

step "5. drive it over ROS 2 with /ackermann_cmd"
ros2 run lagrange_server lagrange_server --robot assets/robots/sedan/sedan.usda \
    --world assets/worlds/city_block/city_block.usda --no-demo --traffic vehicles=0,pedestrians=0 \
    --listen "unix:$dir/r.sock" > "$dir/ros.log" 2>&1 &
server=$!
python3 - <<'PY'
import time
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node("tutorial_cars")
drive = node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 1)
odom = {}
node.create_subscription(Odometry, "/odom", lambda m: odom.update(last=m), 1)
command = AckermannDriveStamped()
command.drive.speed = 4.0
command.drive.steering_angle = 0.1
start = time.time()
while time.time() - start < 40.0:
    drive.publish(command)
    rclpy.spin_once(node, timeout_sec=0.05)
    if "last" in odom and odom["last"].header.stamp.sec >= 6:
        break
twist = odom["last"].twist.twist
# The bicycle model: yaw rate v·tan(δ)/L with the sedan's 2.7 m wheelbase.
expected = 4.0 * 0.10033 / 2.7
print(f"{twist.linear.x:.2f} m/s, yaw rate {twist.angular.z:.3f} rad/s (bicycle model {expected:.3f})")
assert abs(twist.linear.x - 4.0) < 0.1, twist.linear.x
assert abs(twist.angular.z - expected) < 0.01, twist.angular.z
PY
kill -TERM "$server"
wait "$server" || true
server=""

step "6. Autoware's vehicle interface"
if ros2 pkg executables lagrange_autoware 2>/dev/null | grep -q lagrange_autoware_interface; then
    colcon test --packages-select lagrange_autoware --event-handlers console_direct- > /dev/null
    colcon test-result --test-result-base build/lagrange_autoware || fail "the Autoware interface"
else
    echo "lagrange_autoware is not built (no Autoware messages): step left out"
fi

echo
echo "tutorial ok"
