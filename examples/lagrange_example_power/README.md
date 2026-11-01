# A power system from a schematic

A URDF whose `<lagrange>` block names a SPICE netlist, so one file is the circuit for ngspice and the power system of the run. Open the viewer's Electrical workspace (Ctrl+3) for its Schematic, Circuit, 3D overlay and Netlist tabs. A node checks the bus against the hand formula, then spins the motor and sees it sag.

```bash
ros2 launch lagrange_example_power power.launch.py                         # with the viewer
ros2 launch lagrange_example_power power.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: bus 23.80 V at rest (formula 23.80 V), down to 22.58 V at 14.2 A while the motor sped up
```

Where to look: `robot/flywheel.urdf`, `robot/power.cir`, `config/topics.yaml`, `scripts/check_power.py`.

What it shows: `<lagrange><schematic>`, `--ros-topics`, `bus/dc/voltage`.
