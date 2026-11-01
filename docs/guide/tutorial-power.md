# Tutorial: a robot's power system from a schematic

You write a circuit as a SPICE netlist, name it from your URDF, see it in the viewer and check its numbers by hand. The finished result is `examples/lagrange_example_power`.

## 1. The circuit

`power.cir`, a 24 V battery with 100 mΩ inside, a bus capacitor and a lamp. Its first line is a title, as in any SPICE file:

```spice
Power system of my robot
V_pack in 0 DC 24
R_pack in dc 0.1
C_bus dc 0 4.7m IC=24
R_lamp dc 0 12
.tran 1m 0.2 uic
.end
```

Check it in ngspice before anything else: `ngspice power.cir`, then `print v(dc)`. By hand, `24 * 12 / (12 + 0.1) = 23.80 V`.

## 2. Name it from the URDF

SPICE cannot say what a battery holds, where a bus trips or which joint a motor turns. Say it in a `<lagrange>` block inside `<robot>`:

```xml
<lagrange>
  <schematic file="power.cir"/>
  <battery source="V_pack" capacity="72000" cutoff="18" max_charge="10"/>
  <bus node="dc" undervoltage="15" overvoltage="30"/>
  <motor joint="spin" node="dc" r="0.2" l="0.0003" kt="0.1" imax="20" gear="10" rotor_inertia="0.0001"/>
</lagrange>
```

Capacity is in coulombs (20 Ah is 72000), voltages in volts, currents in amps. Other URDF readers skip the block. What the importer reads and refuses is in [formats.md §9.1](../design/formats.md); a part it cannot simulate stops the load with its line number. That includes diodes, inductors and anything between two buses, for now.

Before running, `lagrange check robot.urdf` says whether each motor and joint can carry the robot.

## 3. See it

```bash
lagrange robot.urdf
```

Press `Ctrl+3` for the Electrical workspace:

| Tab | What it is |
|---|---|
| Schematic | Live blocks, wires whose width follows the current. Drag a block to move it; Auto-layout puts them back; Export SPICE writes the circuit |
| Circuit | The same as a circuit diagram, with symbols and values |
| 3D overlay | The robot, each motor's link tinted by its current, blue when reversed |
| Netlist | Edit a copy of the circuit on a form and save it as a `.cir`. The running simulation does not change: run again with the file |

Click the battery for its state of charge, sag and place on its voltage curve.

## 4. Read it from your code

Any signal goes on a ROS topic with `--ros-topics`:

```yaml
publish:
  - {signal: bus/dc/voltage, topic: /power/bus_voltage, type: std_msgs/msg/Float64}
```

## 5. More parts

A voltage curve, for a battery whose voltage follows its charge, given for the whole pack:

```spice
*lagrange battery V_pack capacity=72000 cutoff=30 max_charge=10 soc=0.5 ocv_soc=0,1 ocv=44,52
```

The source must equal the curve at that `soc` (here 48 V), or it is refused: ngspice and the run would answer for different batteries.

A DC-DC converter, with a source standing for its output in ngspice and a behavioural source for what it draws:

```spice
V_rail rail 0 DC 12
B_rail_in dc 0 I = -I(V_rail) * 12 / 0.9 / max(V(dc), 0.001) * ((V(dc) >= 0) ? 1 : 0)
R_lamp rail 0 6
*lagrange bus rail undervoltage=9 overvoltage=15
*lagrange converter dcdc source=V_rail in=dc max_current=5 efficiency=0.9 input_sink=B_rail_in
```

## 6. How you know it is right

`tools/ci/check_electrical.py --verbose` works out simple circuits three ways, by formula, in ngspice and in Lagrange, and fails on a difference. What it does not cover: switching ripple, which is averaged in the run and is for ngspice offline, temperature, trips and regeneration.
