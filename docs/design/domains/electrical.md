# Domain: electrical

## 1. Scope and release

DC power distribution on a robot: battery, bus, motor drives, loads, converters. Voltages and
currents are simulated with averaged (cycle-mean) models that run in real time. Switching-level
detail is exported to SPICE for offline study and is never simulated in the loop (CLAUDE.md
§11.7). This domain is the project's wedge. Release: v0.3 (a thin battery + DC-motor slice in
v0.2 is proposed in open question Q3).

Sub-packages: `network/` (builds the circuit graph from `PortAPI`/`ElectricalBusAPI` prims),
`fmu_host/` (FMI 3.0 co-simulation), `substitutes/`.

## 2. Models per fidelity level

| `fidelity` | Battery | Motor + drive | Bus |
|---|---|---|---|
| `ideal_bus` | Ideal voltage source at nominal voltage; SoC by coulomb counting | Torque = command, clipped by `maxCurrent·Kt`; current from power balance `P = τ·ω / η` | No dynamics |
| `averaged` (default) | ECM: `V = OCV(SoC) − I·R0 − V1 − V2`, RC pairs, `R0(T)` | dq-frame PMSM or DC model with PI current loops (FOC), voltage limit from bus voltage (field weakening not modelled at first), copper loss `I²R(T)`, simple iron/friction loss | Bus capacitor node; regeneration raises voltage; brake resistor; under- and over-voltage trips (brownout) |
| `fmu` | FMU if `elec:battery:behaviorModel` is set | FMU if `elec:behaviorModel` is set, else `averaged` | as `averaged` |

Equations of the averaged PMSM model, per motor, integrated per step with the exact solution of
each axis's R-L equation for the voltages held over the step. That is unconditionally stable,
like the backward-Euler update first planned, and exact for a voltage step, where backward Euler
is 2.8 % off at 10 steps per time constant against the 0.1 % that §8 asks:

```
v_d = R·i_d + L_d·di_d/dt − ω_e·L_q·i_q
v_q = R·i_q + L_q·di_q/dt + ω_e·(L_d·i_d + λ)
τ   = 1.5·p·(λ·i_q + (L_d − L_q)·i_d·i_q)
ω_e = p·N·ω_joint          (p pole pairs, N gear ratio)
R   = R_ref·(1 + α·(T − T_ref))
i_bus = (v_d·i_d + v_q·i_q)·1.5 / (η_inv·V_bus)
```

The flux linkage `λ` and the torque constant in the schema are related by `Kt = 1.5·p·λ` for the
peak-amplitude dq convention. The convention (peak vs RMS, line-line vs line-neutral) is the most
common source of factor errors in motor models; it is fixed in the schema documentation and
checked by a unit test that converts a datasheet example (risk R13).

Network solution: components connect to bus nodes. Each bus node has a capacitance, so node
voltages are states and the network is solved explicitly per step as a set of current sums
(nodal analysis with capacitor companion models). A bus without capacitance gets a small default
(1 µF) with a warning.

## 3. Signals

```cpp
// modules/electrical/network — declare()
for (auto& m : world.instancesOf("ElectricalMotorAPI_1")) {
    auto j = jointOf(m);  auto p = name(m);
    d.input ({.name = fmt("joint/{}/torque_command", j), .type = SignalType::Scalar, .unit = units::N_m});
    d.input ({.name = fmt("joint/{}/velocity", j),       .type = SignalType::Scalar, .unit = units::rad_per_s,
              .hold = Hold::LinearExtrapolate, .feedback = true});
    d.input ({.name = fmt("joint/{}/effective_inertia", j), .type = SignalType::Scalar, .unit = units::kg_m2,
              .optional = true, .defaultValue = 0.0, .feedback = true});      // enables speed prediction
    d.input ({.name = fmt("thermal/{}/temperature", p),  .type = SignalType::Scalar, .unit = units::K,
              .optional = true, .defaultValue = 298.15, .feedback = true});
    d.input ({.name = fmt("thermal/{}/derating", p),     .type = SignalType::Scalar, .unit = units::one,
              .optional = true, .defaultValue = 1.0, .feedback = true});

    d.output({.name = fmt("joint/{}/torque", j),         .type = SignalType::Scalar, .unit = units::N_m});
    d.output({.name = fmt("motor/{}/current_d", p),      .type = SignalType::Scalar, .unit = units::A});
    d.output({.name = fmt("motor/{}/current_q", p),      .type = SignalType::Scalar, .unit = units::A});
    d.output({.name = fmt("motor/{}/voltage_d", p),      .type = SignalType::Scalar, .unit = units::V});
    d.output({.name = fmt("motor/{}/voltage_q", p),      .type = SignalType::Scalar, .unit = units::V});
    d.output({.name = fmt("motor/{}/bus_current", p),    .type = SignalType::Scalar, .unit = units::A});
    d.output({.name = fmt("motor/{}/loss_power", p),     .type = SignalType::Scalar, .unit = units::W});
    d.output({.name = fmt("motor/{}/fault", p),          .type = SignalType::Int,    .unit = units::one});
}
for (auto& b : world.instancesOf("BatteryAPI_1")) {
    auto p = name(b);
    d.input ({.name = fmt("thermal/{}/temperature", p),  .type = SignalType::Scalar, .unit = units::K,
              .optional = true, .defaultValue = 298.15, .feedback = true});
    d.output({.name = fmt("battery/{}/soc", p),              .type = SignalType::Scalar, .unit = units::one});
    d.output({.name = fmt("battery/{}/terminal_voltage", p), .type = SignalType::Scalar, .unit = units::V});
    d.output({.name = fmt("battery/{}/current", p),          .type = SignalType::Scalar, .unit = units::A});
    d.output({.name = fmt("battery/{}/loss_power", p),       .type = SignalType::Scalar, .unit = units::W});
    d.output({.name = fmt("battery/{}/power", p),            .type = SignalType::Scalar, .unit = units::W});   // at its terminals
}
for (auto& bus : world.instancesOf("ElectricalBusAPI_1")) {
    d.output({.name = fmt("bus/{}/voltage", name(bus)),    .type = SignalType::Scalar, .unit = units::V});
    d.output({.name = fmt("bus/{}/brownout", name(bus)),   .type = SignalType::Bool,   .unit = units::one});
}
for (auto& l : world.instancesOf("ElectricalLoadAPI_1")) {
    d.input ({.name = fmt("load/{}/power_demand", name(l)), .type = SignalType::Scalar, .unit = units::W,
              .optional = true});                           // sensors and computers can modulate their draw
    d.output({.name = fmt("load/{}/current", name(l)),     .type = SignalType::Scalar, .unit = units::A});
    d.output({.name = fmt("load/{}/powered", name(l)),     .type = SignalType::Bool,   .unit = units::one});
}
```

`joint/{j}/torque_command` is written by the ROS 2 control bridge, the MAVLink bridge or a
scripted controller. `load/{}/powered` lets a brownout switch off a sensor or the computer,
which is one of the effects this simulator can show and a pure-physics simulator cannot.

## 4. Step size

Default 100 µs. Range 10 µs to 1 ms. Guidance: at least 10 steps per electrical time constant
`L/R` of the fastest motor is not required for stability (the update is implicit) but is needed
for current-loop accuracy; the module warns when `dt > 0.2 / currentLoopBandwidth`.

## 5. Substitutes and contract

Contract: `joint/{j}/torque` for every joint driven by a motor prim (and, when the stage has no
motor prims at all, for every actuated joint with a `torque_command`).

| Profile token | Kind | Behaviour |
|---|---|---|
| `ideal` | ideal | `IdealTorqueSource` from CLAUDE.md §11.5: torque = commanded torque. Also writes `bus/*/voltage` at nominal and `load/*/powered = true` so that sensors stay on |
| `replay` | replay | `joint/*/torque`, bus voltages and currents from MCAP |

`IdealTorqueSource::step` keeps the body shown in CLAUDE.md §11.5, with `torqueOut` and
`torqueCommand` being `SignalId` values resolved in `init()`.

## 6. Placement

CPU on all tiers. One robot has tens of states; the cost is dominated by call overhead, so motors
are stored SoA and updated in one loop. A batched GPU path for thousands of environments is a
later RFC.

## 7. Offline export

*Status (v0.3):* `lagrange export spice|modelica [PROFILE] [options] [--out FILE]`
(`Export.hpp` in `lagrange_electrical`) rather than separate `tools/` programs: the export needs
the loaded stage, overlays included, as the simulator sees it. The drives are averaged sinks of
their electrical power (a parameter to edit, or a waveform such as one recorded from a run); the
open-circuit voltage is held at the initial state of charge; charge limits, trips, brake
resistors and converters are not exported. The Modelica model uses the Modelica Standard
Library 4's `Electrical.Analog`; it is checked for its structure only, as OpenModelica is not
part of the build.

`tools/export_spice` writes an ngspice netlist of the topology (battery ECM, bus capacitors,
ideal averaged drive sources or a three-phase inverter subcircuit per motor) for switching-level
study. `tools/export_modelica` writes a Modelica model with the same topology. Both are derived
from the same USD prims as the runtime network, so the schematic, the 3D view and the exports
agree.

## 8. Validation

| Test | Reference | Tolerance |
|---|---|---|
| RL step response of the d and q axes | closed form | 0.1 % at 10 steps per time constant |
| DC motor no-load speed, stall current, torque-speed line | closed form from `R`, `Kt`, `V` | 0.5 % |
| Coupled motor + inertia step (coupling test) | monolithic stiff ODE solution | 2 % RMS (architecture §8) |
| Battery ECM pulse discharge | published cell characterisation data with OCV and pulse tests: LG HG2 cell, Kollmeyer et al., doi:10.17632/b5mj79w5w9.2, CC BY 4.0 (HPPC and C/20 at 25 °C) | terminal-voltage RMS error < 20 mV per cell |
| Energy balance | battery energy out = mechanical work + losses + stored | closed to 0.5 % over a drive cycle |
| Netlist round trip | exported netlist run in ngspice (averaged sources) vs runtime | bus voltage within 1 % |
| Brownout scenario | scripted stall of both AMR motors on a low battery | bus trips, `load/computer/powered` goes false, recovers after load removal |
| Sim-vs-real (v0.3) | bench recording of a real BLDC drive: phase current, bus voltage, speed (to be recorded; Q16) | within measurement uncertainty bands stated with the dataset |

## 9. Status (v0.3)

`lagrange_electrical` (`modules/electrical/`): `ElectricalNetwork` (`network/`, fidelity
`averaged`, default step 100 µs) and the `IdealTorqueSource` substitute. The models are in
`Models.hpp` so the tests can drive them without a scheduler. The server runs the network when
the robot has `ElectricalMotorAPI_1` prims (from USD, or a URDF with an overlay file such as the
sample AMR's) and the profile or `--domain electrical=off` does not switch it to the
substitute.

Choices made while implementing, beyond §2:

- **One flux linkage.** The torque and the back-EMF come from λ = Kt / (1.5·p), so power is
  conserved by construction. An authored `backEmfConstant` is a check: it must be within 3 % of
  Kt / 1.5 (dq) or Kt (brushed), or the motor is refused as a convention mix-up (risk R13).
- **Voltage limit** of a drive: Vbus/√3 peak phase voltage (sine-triangle with third harmonic or
  space-vector modulation, no overmodulation) for dq machines, Vbus for an H bridge. Saturation
  scales the voltage vector and holds the PI integrators.
- **Bus node** solved with the capacitor's backward-Euler companion against the batteries'
  Thévenin sources; drives and loads draw at the previous step's voltage (explicit), so a load
  notices a brownout one electrical step late.
- **Brownout** latches each drive's fault until the bus has recovered by 2 % and the torque
  command is back to zero; loads come back with the bus. Overvoltage switches in the brake
  resistor, or switches the drives off when there is none, with the same 2 % hysteresis.
- **Battery cut-off** when the open-circuit voltage reaches `cutoffVoltage` (empty), not on the
  terminal voltage under load: sag under load is the bus's undervoltage trip's job.
- **Resistive loads** take their resistance from `elec:load:power` at the bus's starting
  voltage; the bus schema has no nominal voltage.

- **Coupling with physics** (architecture.md §8.1): speed prediction with the current and the
  rotor speed advanced together, the startup check on H/τm, and the rotor inertia added to the
  joint's armature (`applyRotorInertia`, called by the server).
- **Current-loop bandwidth** is capped at 1/dt with a warning: a discrete loop faster than its
  step diverges. (§4's "warn when dt > 0.2 / bandwidth" would warn at the default 100 µs step
  for the default 1 kHz loop, which runs well; the cap is where it matters.)

- **Fidelity `fmu`** (formats.md §6 status): a motor with `elec:behaviorModel` is simulated by
  its FMU, the network feeding its bus; `averaged` ignores the behaviour models.

- **DC-DC converters** (`ElectricalConverterAPI_1`) are averaged regulators. On the output bus
  the converter is an ideal source at `outputVoltage` inside the node equation, giving 0 to
  `maxOutputCurrent` (no reverse flow); at the limit the bus sags and its loads drop at their
  `minVoltage` or with its undervoltage trip. On the input bus it draws the output power over
  `efficiency`, one step late as loads do. It stops while its input bus is browned out. One
  converter per output bus. Signals `converter/<c>/output_current`, `input_current` and
  `loss_power`; `test_network` checks regulation, the power drawn and the current limit.

`elec:controlMode` (as built): `torque` follows `joint/<j>/torque_command`; `velocity` follows
`joint/<j>/velocity_command` through a PI speed loop in the drive, critically damped on the joint's
inertia (`elec:velocityLoopBandwidth`, illustrative), held by the current limit; `voltage` puts
`joint/<j>/voltage_command` on the winding, limited by the bus, with no current loop and so no current
limit. Those two commands are required: `--joint-control` writes them (`Server::commandJoint`, and
`/joint_commands`, whose `effort` field carries volts for a voltage motor), a plugin or `--ros-topics`
can too, and the built-in drive controllers and ros2_control, which write torques, stop the start with
`MissingInput`. On a prismatic joint the drive is linear: `elec:gearRatio` is motor radians per metre
(2π over a screw's lead), and torque, speed and inertia at the joint are force, m/s and mass.
Checked against the closed forms: stall and no-load in voltage mode, steady speed under load in
velocity mode, on a hinge and a slide (`server/test/test_motor_modes.cpp`).

Not yet: battery FMUs and the rotor inertia in Jolt.

| §8 check | Status | Test |
|---|---|---|
| RL step response, d and q | met (exact up to rounding) | `test_models` `RlStepResponseMatchesTheClosedForm` |
| DC motor no-load speed, stall current, torque-speed line | met, 0.5 % | `BrushedDcTorqueSpeedLineMatchesTheClosedForm` |
| Coupled motor + inertia | met, < 2 % RMS in speed and current against a monolithic 1 µs solution, inertias 6e-7 to 1e-2 kg·m², no growth; and (v0.4) the sample quadcopter's dq drive with the network at 50 µs on 1e-5 and 9e-5 kg·m² rotors | `test_coupling` (`Coupling`, `DroneCoupling`) |
| Battery ECM pulse | closed form met exactly; against the LG HG2 HPPC pulses, one fitted parameter set: 5.4 to 6.2 mV RMS on held-out blocks from 46 to 89 % state of charge, 21.5 mV at 24 % (not met: r0 and the OCV slope rise below about 35 %; a state-of-charge-dependent r0 is the next step). 25 °C only | `PulseDischargeMatchesTheClosedForm`, `test_battery_dataset` (data and fit in `test/data/lg_hg2`) |
| Energy balance over a drive cycle | met, 0.5 % | `test_network` `EnergyBalancesOverADriveCycle` |
| Netlist round trip | met: with a drive cycle sagging a weak pack's bus by 2.4 V, ngspice's bus voltage is within 0.05 % of the network's (sag within 0.16 % RMS) | `test_export` (needs ngspice) |
| Brownout scenario | met | `AStalledAmrOnALowBatteryBrownsOutAndRecovers` |
| Sim-vs-real | owed: needs the bench recording (Q16) | — |

## 10. Hand check

`tools/ci/check_electrical.py` draws simple circuits and works each out three ways, independently:
a closed-form formula, ngspice on a netlist written by the script (not `lagrange export`), and a
Lagrange run of a robot imported with `lagrange_import --overlay`, once from an overlay file and
once from the netlist (formats.md §9.1), and once from a URDF's `<lagrange>` block (§9.2). Lagrange must stay within 0.5 %
of the formula and ngspice within 0.1 %. It covers a battery port's harness (`wireResistance`, in the bus's solution and between the
battery's terminals and the bus; the batteries' other ports and loads' and drives' do not count), a resistive load, a constant-power load, an RC
pair's transient, two batteries in parallel, the charge drained, and a PMSM or brushed motor
turning a flywheel. It does not cover switching, thermal effects, a sloped voltage curve, trips,
regeneration or converters. The network only runs for a robot with a motor, so circuits without
one carry an idle motor.
