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

Equations of the averaged PMSM model, per motor, integrated with a fixed-step implicit
(backward-Euler) update on the two currents, which is unconditionally stable for the R-L part:

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
| Battery ECM pulse discharge | published cell characterisation data with OCV and pulse tests (candidate: an openly licensed Li-ion cell dataset; to be selected and its license checked) | terminal-voltage RMS error < 20 mV per cell |
| Energy balance | battery energy out = mechanical work + losses + stored | closed to 0.5 % over a drive cycle |
| Netlist round trip | exported netlist run in ngspice (averaged sources) vs runtime | bus voltage within 1 % |
| Brownout scenario | scripted stall of both AMR motors on a low battery | bus trips, `load/computer/powered` goes false, recovers after load removal |
| Sim-vs-real (v0.3) | bench recording of a real BLDC drive: phase current, bus voltage, speed (to be recorded; Q16) | within measurement uncertainty bands stated with the dataset |
