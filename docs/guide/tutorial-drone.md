# Tutorial: a quadcopter, from hover to a PX4 mission

In this tutorial you look at how the sample quadcopter is described, fly it with the simulator's
built-in controller, put it in gusty wind, read its flight sensors, and finally let PX4 fly a
mission with the simulator in lockstep. It takes about 30 minutes, plus building PX4 once.

`tools/ci/tutorial_drone.sh` runs every step below without the viewer and checks the results
(the PX4 step only when PX4 is built), so the commands here are known to work.

## Before you start

You need Lagrange built as in the [README](../../README.md#build), and the `mavlink` package
(`rosdep install` brings it; it is `ros-lyrical-mavlink`). In every terminal, from the source
tree:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/lagrange_ws/src/lagrange && source ../../install/setup.bash
alias lagrange='ros2 run lagrange_server lagrange'
```

## 1. The quadcopter in three files

`assets/robots/quad_x500/` holds the sample, a generic 500 mm X quadcopter:

- `quad_x500.urdf`: the frame and four rotor links on continuous joints, numbered and turning
  as PX4 expects them (rotor 1 front right, counter-clockwise).
- `quad_x500.overlay.yaml`: what URDF cannot say. Each rotor joint gets a `RotorAPI_1` (radius
  and the propeller's thrust and power coefficients) and a motor; a 4S battery feeds a 14.8 V
  bus; the frame gets drag (`AeroAPI_1`) and an IMU, a GNSS receiver, a magnetometer and a
  barometer.
- `quad_x500.usda`: both together, in one USD file, made by:

```bash
ros2 run lagrange_server lagrange_import assets/robots/quad_x500/quad_x500.urdf /tmp/quad.usda \
    --overlay assets/robots/quad_x500/quad_x500.overlay.yaml
grep -c RotorAPI_1 /tmp/quad.usda    # 4: the rotors are in the USD
```

## 2. The site

A drone needs three facts about where it flies, and Lagrange has no defaults for them because
they differ from site to site: the air density (for the rotors), the place on the Earth (for
GNSS, the magnetic field and the air pressure), and the wind. The example profiles give them:

```bash
lagrange validate profiles/examples/quad_x500_hover.yaml profiles/examples/quad_x500_gusty.yaml
```

`quad_x500_hover.yaml` sets `rotor_aero.air_density: 1.225` (sea level) and
`world.geo_origin` at PX4's default home in Zurich; `quad_x500_gusty.yaml` adds 8 m/s of wind
from the west with Dryden turbulence.

## 3. Hover

Without an autopilot the simulator flies a multicopter itself: it takes off after a second and
holds 2 m above where it started.

```bash
ros2 run lagrange_server lagrange_server --robot assets/robots/quad_x500/quad_x500.usda \
    --world none --air-density 1.225 --geo-origin 47.397742,8.545594,488
```

`--world assets/worlds/open_field/open_field.usda` in place of `--world none` puts it on the
landing pad of an open field with trees, a shed and a windsock, where the missions below fly.
Watch it with `ros2 run lagrange_viewer lagrange_viewer --connect
unix:$XDG_RUNTIME_DIR/lagrange/default.sock`. In the Electrical workspace the battery gives
about 190 W; the Signals workspace has the four rotors' speed (about 550 rad/s) and thrust.

### Fly it yourself

With ROS (the server's default), `/cmd_vel` flies it: `linear.x` forward and `linear.y` left
in the drone's heading frame, `linear.z` up (m/s), `angular.z` the yaw rate (rad/s), as
`geometry_msgs/TwistStamped`. Without a command for half a second it stops and holds. From the
keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

`i` forward, `,` back, `j` and `l` turn, Shift with `J` and `L` sideways, `t` up, `b` down,
`k` stop; `q` and `z` change the speed. Or one command at a time:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0}}}"
```

Speeds are capped at 5 m/s across, 2 m/s up or down and 1.5 rad/s of yaw, and it never goes
below 0.3 m above where it took off.

From Python, without ROS or the viewer:

```python
import lagrange

sim = lagrange.Simulation(
    lagrange.sample_quadcopter(), world=None, profile=lagrange.sample_profile("quad_x500_hover")
)
sim.step_until(8.0)
print(sim.read("body/base_link/pose")[2])  # 2.125: 2 m above the start
print(sim.read("battery/battery/power")[0])  # about 190 W
print(sim.read("sensor/barometer/pressure")[0])  # about 95 570 Pa at 490 m
```

### Propellers as built

Each rotor also names its propeller: `assets/propellers/generic_10in_2blade_ccw.prop.yaml` and
its `cw` twin, the blade's chord and twist from root to tip, its sections' lift and drag, and
its hand. The viewer draws the rotors as these blades. Fly the quadcopter on them, every
rotor's thrust and torque computed from its blades (blade-element momentum):

```bash
lagrange run profiles/examples/quad_x500_bem.yaml \
    --robot assets/robots/quad_x500/quad_x500.usda --world none
```

It hovers as before, its blades having been chosen to match the measured propeller. Put a `cw`
propeller on a `ccw` motor in the overlay (`aero:rotor:propeller`) and the server warns at load:
that rotor turns its propeller backwards, trailing edge first, and gives less than half its
thrust for more power, as a real one does.

Two variants show other propellers. Folding blades, which hang folded when the motors stop and
open as they spin up:

```bash
ros2 run lagrange_server lagrange_server --robot assets/robots/quad_x500/quad_x500.urdf \
    --overlay assets/robots/quad_x500/quad_x500_folding.overlay.yaml --world none \
    --air-density 1.225 --geo-origin 47.397742,8.545594,488
```

and the motors upside down under the arms, the propellers pushing
(`assets/robots/quad_x500_inverted/quad_x500_inverted.usda` in place of the sample).

## 4. Gusty wind

```bash
lagrange run profiles/examples/quad_x500_gusty.yaml \
    --robot assets/robots/quad_x500/quad_x500.usda --world none
```

The wind grows with height (a logarithmic profile over open terrain), and turbulence adds gusts
of a metre or two per second; `env/wind/gust` is the gust at the frame. The controller leans
into it and stays within a few centimetres of its spot.

### See the downwash

```bash
lagrange quad -- --overlays airflow --slice aero/wind_speed:xz
```

The rotors' air is simulated live: a column under each rotor that merges into one jet a metre
below the drone and spreads along the ground as outwash
([rotor_aero.md](../design/domains/rotor_aero.md)). In the 3D view the *airflow · particles*
overlay moves particles through it, coloured by speed (legend in m/s). In the Environment
workspace the Slice 2D panel cuts through the drone: XZ and YZ upright, XY at the height slider.
`lagrange set wind_speed=4 wind_from=270` leans the wake downwind. A second drone flown under
this one gets less thrust from its propellers (at the `bem` fidelity), and with
`--weather ...,ground_dust_diameter=1e-4,ground_dust_concentration=1e-5` the outwash lifts
dust that the cameras and lidars see.

## 5. PX4 or ArduPilot flies it

PX4 runs as its own program and talks to the simulator through its simulator interface: the
simulator sends it IMU, magnetometer, barometer and GNSS readings, PX4 answers with motor
outputs, and neither runs ahead of the other (lockstep). Build PX4 v1.17.0 once:

```bash
git clone --branch v1.17.0 --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl_default
```

(PX4's Python packages: `pip install -r Tools/setup/requirements.txt`, in a virtual environment
`~/PX4-Autopilot/.venv` if you like.) Then, from the Lagrange source tree:

```bash
tools/ci/px4_mission.sh          # take off, fly a 10 m square at 5 m, land
tools/ci/px4_mission.sh --wind   # the same in 8 m/s of gusty wind
```

The script starts `lagrange_server --mavlink px4` with the quadcopter, starts PX4 with the
airframe `22100_lagrange_quad_x500` (the quadcopter's rotor layout for PX4), uploads the mission
over MAVLink and follows it until PX4 has landed and disarmed. QGroundControl can watch it too:
PX4 sends to UDP 14550.

ArduPilot flies it too, through its JSON simulator interface. Build ArduCopter once:

```bash
git clone --branch Copter-4.7.2 --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && ./waf configure --board sitl && ./waf copter
```

(and `pymavlink` in `~/ardupilot/.venv`). Then:

```bash
tools/ci/ardupilot_mission.sh          # the same square, flown by ArduCopter
tools/ci/ardupilot_mission.sh --wind
```

The server runs with `--mavlink ardupilot` and answers each of ArduCopter's servo frames with
the vehicle's state, 1000 times a simulated second; ArduCopter takes its parameters from
`bridges/mavlink/ardupilot/lagrange_quad_x500.parm`.

## 6. A low battery

The motors draw from the simulated battery, so its voltage limits how fast they can turn. At
10 % charge the pack sags from 16.2 V to 13.9 V at full throttle and the rotors give 29 N
instead of 37.5 N; at 3 % the bus drops below its trip voltage and the quadcopter does not take
off. `test_server`'s `Multicopter.ALowBatterySagsAndGivesLessThrust` flies both.

## Where next

- [Concepts](concepts.md): domains, substitutes, signals and units.
- `docs/design/robot-classes.md` §5: what the drone class covers and what is still to come.
