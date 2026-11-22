# RFC 0024: Fluid power: pneumatic and hydraulic cylinders, valves, pumps and lines

| | |
|---|---|
| Status | Draft (for the owner) |
| Created | 2026-11-22 |
| Owner | Lagrange core maintainers; the project owner accepts or refuses |
| Affects | `schemas/actuator` (five new classes, additions only), `modules/fluid_power/` (new domain and package `lagrange_fluid_power`), `modules/actuators/` (`iso6358MassFlow` gains a critical-ratio argument, an overload), `server/` (URDF `<gazebo>` and `<lagrange>` forms, default ROS topics), `bridges/ros2_control/` (GPIO lookup of `fluid/` signals), `profiles/schema.json` (one domain key), `tools/ci/` (a hand check against other tools), `assets/robots/` (two samples), `docs/` |
| Not affected | `proto/` (the viewer reads the new signals through the existing signal subscription), `core/plugin_abi/` (open question 10), `modules/rigid_body/` (it already takes joint forces, body wrenches and `friction_limit`) |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0005 (profiles), RFC 0007 (the 1.x freeze), RFC 0008 (plugin substitutes), RFC 0010 (electrical nodes), RFC 0020 (actuators: §4.7, §4.8, §5, step 7, and open question 9) |
| Review | Touches `schemas/`: core-maintainer review and **2 approvals** (CLAUDE.md §9, CODEOWNERS) |

## Summary

Pneumatic and hydraulic actuation, as RFC 0020 step 7 asks: cylinders, valves, pumps, relief
and check valves, accumulators and the lines between them. A robot's fluid circuit is a network
of volumes (lines and cylinder chambers), each with one pressure, joined by restrictions
(valves, orifices, pipes) and fed by supplies (a regulated pressure or a pump). A new domain,
`fluid_power`, solves it on the signal bus every 100 µs by backward Euler with Newton
iterations, as the electrical domain solves its buses. A cylinder pushes a prismatic joint, or
two pin points on two bodies (an excavator's boom cylinder), with
`p_A·A_A − p_B·A_B − friction`. The piston's speed is advanced inside the fluid step on the
joint's effective inertia, so the stiff oil column does not ring against the 1 ms physics step.
Pumps load an electric motor's shaft, valves can be moved by the existing solenoid, and every
loss is heat for the thermal domain. Every schema change is an addition under RFC 0007. The
model is built in Lagrange, because the coupling to physics, electrical and thermal is the
point. It is checked against closed forms, against Hopsan (Apache-2.0) and OpenModelica with
OpenHydraulics (BSD-3-Clause) offline, and against a published measured crane dataset.

## Motivation

RFC 0020 ranked fluid power last (rank 7 and 8, phase 3) and left it for its own RFC (open
question 9). Phases 1 and 2 are built: suction and magnet grippers, lead screws, servos,
steppers, solenoids and GPIO in `LagrangeSystem` (docs/physics/actuators.md). The gaps that
remain are fluid ones:

- **Pneumatic grippers and slides.** Most factory end effectors are a pneumatic parallel
  gripper on a 5/2 solenoid valve. Today a user fakes one with `JointControl` and gets no
  closing time, no force that follows supply pressure, and no fall when the air goes.
- **Hydraulic machines.** Excavators, loaders, forestry cranes, agricultural implements and
  heavy legged robots are hydraulic. Their behaviour comes from the oil: a boom that sinks under
  load, a relief valve that limits the push, a pump that the engine or motor must turn, a
  pressure spike when a valve shuts.
- **The coupling is the point.** A pump is a load on an electric motor (electrical), every
  valve loss is heat (thermal), and a cylinder is a force on the multibody (physics). Lagrange's
  wedge is exactly this coupling. A separate fluid tool cannot see the battery sag when the boom
  lifts.

### What exists

| Piece | Where | What it gives fluid power |
|---|---|---|
| ISO 6358 mass flow, `iso6358MassFlow(C, p_up, p_dn, T)` | `modules/actuators` (`Gripper.hpp`) | the pneumatic valve flow, with the ideal critical ratio `b` = 0.528 only |
| Air constants (`kAirGasConstant`, ISO 6358 reference density and temperature) | `Gripper.hpp` | the pneumatic medium |
| Solenoid on a prismatic joint (`SolenoidAPI_1`) | `modules/actuators` | a valve's spool can be a solenoid's plunger |
| Nozzle and pump power (`NozzleAPI_1`, orifice `C_d·√(2Δp/ρ)`, `pump_power` as a load) | `modules/actuators` (RFC 0022 §8) | the orifice law and the pump-on-the-bus pattern |
| `joint/<j>/friction_limit`, `load_torque`, `effective_inertia` | `modules/rigid_body` | holding, shaft loads, the piston prediction |
| `body/<b>/wrench/<source>` | `modules/rigid_body` | forces between two pins (tether, hydrodynamics use it) |
| Nodal network with backward-Euler companions, dense LU with pivoting | `modules/electrical/network` (RFC 0010) | the pattern for the solver |
| Speed prediction across the physics step | architecture.md §8.1 | the pattern for the piston |
| FMU host, FMI 3.0 co-simulation (FMI Library 3.0.4, BSD-3-Clause) | `modules/electrical/fmu_host` | a route for a circuit authored in Modelica (§9) |
| GPIO in `LagrangeSystem`, `SignalTopics` | `bridges/ros2_control`, `bridges/ros2` | ROS and ros2_control access |

## Non-goals

- **Flow inside lines.** No CFD and no distributed pipe model in the first version. A line is
  one volume with one pressure. Wave effects (water hammer) are open question 6.
- **Cavitation and aeration in detail.** A hydraulic pressure is held at a floor, and the node
  raises a `cavitation` flag. Gas release, vapour bubbles and erosion are left out.
- **Oil temperature.** Viscosity, density and bulk modulus are constants. Losses are reported
  as heat, but the oil does not warm. Open question 5.
- **Valve and pump internals.** No spool flow forces, no pump ripple, no swash-plate dynamics.
  A spool is a first- or second-order lag. A pump is a displacement with leakage and a
  hydromechanical efficiency.
- **Load-sensing and electro-hydraulic controllers.** No load-sensing pump compensator, no
  valve electronics with their own loops. A user closes loops in ROS 2 or a plugin.
- **Seals and wear.** Friction is a Stribeck curve with constants. No seal temperature, no
  pressure-dependent friction in the first version (open question 4).
- **Hydraulic motors.** Rotary actuators are open question 8.
- **GPU.** Nothing here runs on the GPU (§7).

## Design

### 1. The network

A fluid circuit is a graph:

- **Nodes** are volumes with one pressure each: a line or hose (`FluidLineAPI_1`), each
  cylinder chamber, each accumulator's oil or air side. A node's pressure is a state.
- **Branches** carry flow between two nodes or between a node and a reservoir: valve paths,
  orifices, pipes, check and relief valves, pump outlets, internal leaks.
- **Reservoirs** are fixed pressures: the supply of a pressure source, the tank of a
  hydraulic circuit, the atmosphere for pneumatic exhaust.

Each connected circuit has one medium, given by its supply (`FluidSupplyAPI_1`,
`act:fluid:medium` = `air` or `hydraulic_oil`). A circuit with two media, or none, stops the
start with a message naming the prims.

This mirrors the electrical network (docs/design/domains/electrical.md §2 and §10): pressure is
voltage, volume flow is current, a node's capacity `V/β` is a bus capacitor, an orifice is a
nonlinear resistor, a pump is a current source behind a shaft.

### 2. Models

#### 2.1 Hydraulic node

For a node of volume `V` (m³) and effective bulk modulus `β_e` (Pa):

```
dp/dt = (β_e / V) · (Σ Q_in − Σ Q_out − dV/dt)
1/β_e = 1/β_fluid + 1/β_wall
```

`β_fluid` is the circuit's `act:fluid:bulkModulus`. `β_wall` is the line's
`act:fluid:line:wallBulkModulus` (a hose's compliance), infinite when 0. Entrained air is folded
into `β_fluid` by the author (open question 3). The series form is the standard one for a
compliant container (Merritt 1967, ch. 2; Jelali and Kroll 2003, ch. 4; TODO(verify) the
section: neither book is openly readable). The pressure is held at
`act:fluid:minimumPressure` (absolute, fallback 0 Pa); while held there, the node's
`cavitation` flag is on.

#### 2.2 Pneumatic node

Air is an ideal gas at the circuit's temperature `T` (the weather's `env/air_temperature`, else
the site's ambient, else 293.15 K as the grippers use), polytropic with index `n` (RFC 0020
§4.7):

```
dp/dt = (n·R·T / V) · (Σ ṁ_in − Σ ṁ_out) − (n·p / V) · dV/dt
```

`R` is `kAirGasConstant`. `n` is `act:fluid:polytropicIndex`: 1 is isothermal, 1.4 adiabatic.
Fallback 1.4 (open question 2). Chamber temperature as a state, with its energy equation, is
open question 2 as well. Pneumatic pressures are absolute and never below 0.

#### 2.3 Orifice and valve flow

Hydraulic, turbulent orifice:

```
Q = C_d · A · √(2·|Δp| / ρ) · sign(Δp)
```

with `C_d` the path's discharge coefficient and `A` its open area. The square root has an
infinite slope at `Δp = 0`, which Newton cannot use. Below a transition pressure `Δp_t` the
flow is laminar, linear in `Δp`, joined to the root with equal value at `Δp_t`:

```
Q = C_d · A · √(2·Δp_t / ρ) · Δp / Δp_t           for |Δp| < Δp_t
```

`Δp_t` follows from a transition Reynolds number of the orifice, `Re_t`, through its hydraulic
diameter `d_h = √(4A/π)` and the oil's kinematic viscosity `ν`:
`Δp_t = (ρ/2)·(Re_t·ν / (C_d·d_h))²`. `Re_t` is a named constant (proposed 10; TODO(verify)
a published value for sharp-edged orifices, e.g. in Merritt ch. 3 or Jelali and Kroll ch. 4).
It is a regularisation, written down, not a device value.

Pneumatic, ISO 6358 (RFC 0020 §4.7):

```
ṁ = C · p_up · ρ0 · √(T0 / T_up) · { 1                              if p_dn/p_up ≤ b
                                     √(1 − ((p_dn/p_up − b)/(1 − b))²)  otherwise }
```

`C` (m³/(s·Pa)) is the sonic conductance and `b` the critical pressure ratio of the path, both
from the valve's datasheet. `iso6358MassFlow` in `modules/actuators` today fixes `b` at the ideal
nozzle's 0.528. Step 3 adds an overload with `b` (the old function calls it with 0.528, so the
grippers do not change). The same slope problem at `p_dn → p_up` is solved the same way: linear
in `Δp` over the last 0.1 % of the ratio (a named constant, TODO(verify) that the band does not
move the closed-form checks of §8 beyond their tolerance). ISO 6358-1:2013 is sold, so the form
above stays as RFC 0020 wrote it (TODO(verify) the clause against the standard before step 3).

#### 2.4 Pipes

A pipe between two nodes (`ValveAPI_1` kind `pipe`, §4) with length `L`, bore `d`:

```
laminar (Re < 2300):  Δp = 128·μ·L·Q / (π·d⁴)           (Hagen–Poiseuille)
turbulent:            Δp = f·(L/d)·(ρ·v²/2),  f = 0.316·Re^(−1/4)   (Blasius, smooth pipe)
```

`μ = ρ·ν`. Between the two the friction factor is blended over 2300 to 4000. Hagen–Poiseuille is
exact. TODO(verify) the Blasius range and the transition band against a fluid-mechanics text
(White, *Fluid Mechanics*; not checked in this pass). Pipe inertance (the oil's mass) is left
out with wave effects (open question 6). Pneumatic pipes are not modelled: a pneumatic tube is
a volume and an ISO 6358 restriction with its own `C` and `b`.

#### 2.5 Valves

| Kind | Paths | Opening |
|---|---|---|
| `directional` | 4/3 or 5/2-style: P→A, B→T (command > 0); P→B, A→T (command < 0); centre by `act:fluid:valve:center` (`closed`, `open`, `tandem`, `float`) | on/off: full open beyond ±0.5 |
| `proportional` | the same paths, open area linear in the spool's travel past the overlap | continuous |
| `on_off` | one path, 2/2 | command > 0.5 |
| `check` | one path, opens past `crackingPressure` | `(Δp − p_crack)/(p_full − p_crack)`, clamped to 0…1 |
| `relief` | node to tank or atmosphere | the same ramp |
| `orifice` | one fixed restriction | always open |
| `pipe` | §2.4 | always open |

A spool's travel `s` (−1…1) follows the command `u` through a lag. Second order with natural
frequency `ω_v` and damping ratio `ζ_v`, or first order when `ζ_v` is 0
(`τ = 1/ω_v`), or at once when `ω_v` is 0. It is solved exactly over the step (it is linear).
A spool can instead follow a solenoid: with `rel act:fluid:valve:solenoid` set to a joint with
`SolenoidAPI_1`, `s` is that joint's `position / stroke`. The solenoid then gets the spool's
mass and return spring from its own joint, and the valve no command.

A closed path leaks through `act:fluid:valve:leakArea` (hydraulic) or `leakConductance`
(pneumatic), fallback 0. A 4/3 spool's overlap (`act:fluid:valve:overlap`, 0…1 of the travel,
fallback 0) is a dead band at centre, as a real closed-centre spool has.

A relief valve's ramp from cracking to full open is its pressure override. It needs no
hysteresis for the solver: it is continuous, and Newton converges on it. A check valve the same.

#### 2.6 Supplies and pumps

`FluidSupplyAPI_1` kinds:

- `pressure`: a reservoir at `act:fluid:supply:pressure` (gauge), with an optional flow limit
  `maxFlow` (m³/s, or kg/s for air). Beyond the limit it gives the limit, and its pressure
  falls. This is a compressor with a regulator, or a hydraulic power unit, at the level of
  detail of a datasheet.
- `fixed_pump`: displacement `D` (m³/rad) at shaft speed `ω`:
  `Q = D·ω − k_leak·(p_out − p_in)`; shaft torque
  `τ = D·(p_out − p_in) / η_hm`.
- `variable_pump`: `D` scaled by `fluid/<s>/displacement_command` (0…1), and by a pressure
  compensator: from `compensatorPressure` the displacement falls linearly to 0 over
  `compensatorBand`.

The shaft speed is the joint's `joint/<j>/velocity` when `rel act:fluid:supply:shaftJoint`
names a joint (an electric motor's, so the electrical domain drives it), and the pump writes
its torque there as `joint/<j>/load_torque`. Without a shaft joint, the speed is
`act:fluid:supply:speed` and the shaft power `τ·ω / η_drive` is
`fluid/<s>/shaft_power`, which an `ElectricalLoadAPI_1` on the same prim can take as its
`power_demand` (the nozzle's pump does the same, RFC 0022 §8).

The pump draws from the tank (hydraulic) or the atmosphere (pneumatic compressors are
`pressure` supplies in this RFC). `act:fluid:supply:returnPressure` is the tank's absolute
pressure, fallback the site's atmosphere.

#### 2.7 Accumulator

A gas-charged accumulator on a node: precharge `p0` (absolute) at gas volume `V0` (the whole
shell), polytropic `n`. While the node is above `p0`, the gas volume is
`V_g = V0·(p0/p)^(1/n)` and the oil volume `V0 − V_g` adds to the node:

```
dp/dt = (Σ Q) / (V/β_e + V_g / (n·p))
```

Below `p0` the bladder is empty and adds nothing. Closed form for the check: `p·V_g^n`
constant (§8). Gas temperature is not a state (as §2.2).

#### 2.8 Cylinder

A cylinder has chamber A (cap side, area `A_A = π·D²/4`) and B (rod side,
`A_B = π·(D² − d²)/4`; equal to `A_A` for a through-rod, `rodEnds = double`). Its stroke
position `x` (0 retracted, `stroke` extended) sets the chamber volumes:

```
V_A = V_A0 + A_A·x          V_B = V_B0 + A_B·(stroke − x)
F   = p_A·A_A − p_B·A_B − p_env·(A_A − A_B) − F_f(ẋ) − F_spring
```

`V_A0`, `V_B0` are the dead volumes, required (a zero dead volume is an infinitely stiff end).
`p_env` is the atmosphere for the rod's outside face (a pneumatic and a hydraulic rod alike).
`F_spring = F0 + k·x` for a single-acting cylinder's return spring. A single-acting cylinder's
B side is open to the atmosphere (pneumatic) or the tank (hydraulic). Internal leakage across
the piston is a linear branch A→B (`act:fluid:cylinder:internalLeakage`, m³/(s·Pa), fallback 0).

**Friction**, Stribeck with viscous term (Armstrong-Hélouvry, Dupont and Canudas de Wit 1994):

```
F_f(v) = (F_c + (F_s − F_c)·exp(−(v/v_s)²))·sign(v) + c_v·v
```

`F_c` Coulomb, `F_s` breakaway, `v_s` Stribeck velocity, `c_v` viscous. Fallbacks 0 (no
friction), with `F_s = F_c` when `F_s` is not set. Sticking is not done by the force law: the
static part `F_s` goes to physics as `joint/<j>/friction_limit` (the screw's path, RFC 0020
§4.4), so the engine holds the piston still while the pressure force is below it. A LuGre model
(Canudas de Wit et al. 1995) is open question 4.

**End cushioning.** In the last `cushionLength` of travel into either end, the exhausting
chamber's outflow passes only through its cushion restriction (`cushionArea` with the valve's
`C_d` for oil, `cushionConductance` and `b` for air), in series with what it was flowing
through. Its pressure rises and brakes the piston. The cushion's check valve lets the inflow
bypass it on the way out of the end. `cushioning` flag while active. The end stops themselves
are the joint's limits in physics (joint mode) or the driven joint's (strut mode, §3).

### 3. Coupling to the multibody

A cylinder acts in one of two ways.

- **Joint mode.** `CylinderAPI_1` on a prismatic joint's prim. `x` is the joint's position
  (plus `act:fluid:cylinder:offset`). The cylinder writes `F` as `joint/<j>/torque_command`
  (N on a slide), as the solenoid does, and the joint is made actuated. This fits grippers and
  linear slides, and any URDF.
- **Strut mode.** `CylinderAPI_1` on its own prim, with `rel barrelBody`, `barrelAnchor`,
  `rel rodBody`, `rodAnchor` and `closedLength` (pin-to-pin at `x = 0`). `x` is the pin
  distance less `closedLength`. The force acts along the line between the pins, as an equal and
  opposite pair `body/<rod>/wrench/fluid_power` and `body/<barrel>/wrench/fluid_power`, as the
  tether does. This is the excavator's boom cylinder: it closes a loop that URDF cannot hold,
  without a loop joint. The cylinder's own mass is lumped into the two bodies by the author.
  `rel drivenJoint` names the hinge the cylinder turns. Its limits must match the stroke (the
  start warns when they do not), and it is the joint the prediction uses.

**Why the piston is predicted.** The oil column is a stiff spring. A cylinder of mass `m`
between chambers A and B has the hydraulic natural frequency

```
ω_h² = (β_e/m) · (A_A²/V_A + A_B²/V_B)
```

(Merritt 1967; Jelali and Kroll 2003, ch. 4; TODO(verify) the equation numbers). It reaches
hundreds of hertz (RFC 0020 §4.8). Exchanging force and speed only once per 1 ms physics step
is then an explicit integrator on that pole, which is unstable for `ω_h·H` above about 2, as
architecture.md §8 shows for a light motor. The fix is the electrical one (architecture.md §8.1):

- Physics publishes `joint/<j>/effective_inertia` for every actuated joint already. Joint mode
  reads it as the moving mass `m` (kg on a slide). Strut mode reads the driven hinge's `J` and
  turns it into a mass along the cylinder, `m = J / r²`, with `r` the moment arm of the pin
  line about the hinge at the current pose.
- Inside each 100 µs fluid step, the piston speed `v` is an unknown of the implicit solve, with
  `m·dv/dt = F(p_A, p_B, v) − F_load`. The pressures and the speed are advanced together, so the
  oil spring is integrated implicitly.
- `F_load` is estimated from the previous physics window, as the electrical domain estimates its
  load torque. Physics receives the window average of `F`, so its impulse matches the
  prediction. At each physics step the predicted speed and position are replaced by the true
  ones.
- **Startup check.** With `m`, `V_A`, `V_B` at the start pose, the module computes `ω_h·H`. It
  refuses to start above a limit and names the cylinder and a cure (a smaller physics step,
  larger dead volumes, or `act:fluid:bulkModulus` of the hose). The limit is set from the
  coupling test (§8, test 14), as the motor's `H/τm` was.

`fluid_power.piston_prediction: false` in the profile turns the prediction off, for the
coupling test only.

### 4. Domain, step and solver

**Its own domain.** RFC 0020 §3.1 placed fluid power as a module inside `actuators`. This RFC
proposes a domain of its own, `fluid_power` (package `lagrange_fluid_power`,
`modules/fluid_power/`), for three reasons. It is a solved network with its own step and
solver, as `electrical` is, not a per-device model. It has its own substitutes (§5), which users
want to switch apart from the grippers and servos. And a profile can then run electrical with
fluid power replaced by an ideal force, or the reverse. The reserved schema names stay as RFC
0020 reserved them (open question 1).

**Step.** 100 µs by default (RFC 0020 §3.1), range 10 µs to 1 ms. The module warns when the
step is above a tenth of the fastest spool's time constant `1/ω_v`.

**Why implicit.** A hydraulic node's time constant is its capacity over its conductance,
`τ = V / (β_e·∂Q/∂p)`. For a 1 cm³ volume of oil at an illustrative `β_e` of 10⁹ Pa behind a
valve path with `∂Q/∂p` of 10⁻¹¹ m³/(s·Pa), `τ` is 100 µs. Forward Euler needs a step under
`2τ`, and the small volumes inside fittings and valves are smaller still. Pneumatic nodes are
softer but choke and unchoke. An explicit scheme would set the step from the smallest volume in
the circuit. Backward Euler is stable at any step, and a 100 µs step stays a choice about
accuracy, as for the electrical motors.

**Solver.** Unknowns per step: every node's pressure and every piston's speed. Spool travel is
advanced first (linear, exact). Then backward Euler on

```
C(p)·(p_{n+1} − p_n)/Δt = Σ Q(p_{n+1}, v_{n+1}, s)    (one row per node)
m·(v_{n+1} − v_n)/Δt    = F(p_{n+1}, v_{n+1}) − F_load (one row per piston)
```

solved by Newton's method with the analytic Jacobian (every flow law of §2 is C¹ after the
regularisations), dense LU with partial pivoting (a robot's circuit has tens of unknowns, as an
electrical network has), from the last step's solution. At most 8 Newton iterations to a fixed
relative tolerance of 10⁻⁹ on pressure; when the cap is reached without convergence, the step
is halved, once, and a counter `fluid/<circuit>/newton_failures` rises. Separate circuits are
solved apart.

**Determinism (invariant 7).** One thread, nodes and branches in prim-path order, a fixed
iteration cap and tolerance, no fast-math. The same run twice is identical bit for bit, as the
electrical network is (`test_fluid_power_run`, §8).

### 5. Substitutes (invariant 3)

RFC 0020 §3.3's row, refined:

| Token | Kind | Behaviour |
|---|---|---|
| `ideal` | ideal | A cylinder's force is `fluid/<c>/force_command` if written, else its valve's command times `p_supply·A` on the side it opens, bounded by `p_supply·A_A` extending and `p_supply·A_B` retracting. No pressure dynamics, no friction, no flow limit. Pumps load their shaft with nothing. Pressures are written at the supply's. |
| `constant` | constant | Every cylinder is locked where it starts: `joint/<j>/friction_limit` on its joint (joint mode) or its driven joint (strut mode) is set to `p_supply·A_A` (holds any load it could have pushed). Pressures stay at their start. |
| `replay` | replay | `fluid/*` signals, and the cylinders' forces and wrenches, from MCAP. |

`zero` is `fluid_power: {enabled: false}` with no substitute named: no force, cylinders float.
Every output read by another domain is optional there (`load_torque`, `loss_power`,
`power_demand`, `friction_limit`), so switching the domain off never stops a run.

### 6. Electrical and thermal

- A pump on a motor's shaft joint loads that motor through `joint/<j>/load_torque`, so the bus
  current, the battery's sag and the motor's heat follow the boom's load. An electric mini
  excavator is the sample (§10).
- A valve moved by a solenoid (`rel act:fluid:valve:solenoid`) draws the solenoid's coil
  current. The solenoid's supply is not yet a bus (docs/physics/actuators.md, "Solenoids"); that
  stays RFC 0020's TODO.
- Every branch and pump writes `fluid/<x>/loss_power` (W): `Δp·Q` for a hydraulic path,
  `ṁ·R·T·ln(p_up/p_dn)` for an isothermal pneumatic one (TODO(verify) this throttling-loss form
  for air before step 5), and `τ·ω − Δp·Q` for a pump. A `ThermalAPI_1` node or a
  `HeatSourceAPI_1` with `powerSignal` on the same prim takes it as heat (RFC 0012), as a motor's
  `loss_power` is taken today. The oil's own temperature is open question 5.

### 7. Tiers (invariant 9)

| Tier | Behaviour |
|---|---|
| 0 (CPU only) | Full model. The reference. |
| 1 (raster GPU) | The same CPU model. |
| 2 (full) | The same CPU model. |

Nothing runs on the GPU. A robot's circuit has tens of unknowns: a dense solve is microseconds.
A batched GPU path for thousands of RL environments is a later RFC, behind the same interface
(invariant 8), as for the electrical network (electrical.md §6).

### 8. Schemas (RFC 0007: additions only)

All in `schemas/actuator/schema.usda`, namespace `act:fluid:`, under the names RFC 0020
reserved (`FluidSupplyAPI_1`, `ValveAPI_1`, `CylinderAPI_1`) plus two new ones
(`FluidLineAPI_1`, `AccumulatorAPI_1`). `frozen_1_0.json` is not changed. Each class goes into
`schemas/CHANGELOG.md` and formats.md §3.16. A test, `test_fluid_power_schemas_are_additions`,
checks that no existing class or attribute changes. Every numeric attribute carries
`lagrangeUnit` (`test_every_numeric_attribute_has_a_unit`). As for the grippers, numbers that
describe a device have no fallback: they come from its datasheet.

`FluidSupplyAPI_1` (single apply). Defines a circuit's medium and feeds it.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `act:fluid:medium` | token `air`, `hydraulic_oil` | | required | the circuit's medium |
| `act:fluid:density` | double | kg/m3 | required for oil | `ρ` |
| `act:fluid:bulkModulus` | double | Pa | required for oil | `β_fluid`, effective, with entrained air |
| `act:fluid:kinematicViscosity` | double | m2/s | required for oil | `ν` |
| `act:fluid:minimumPressure` | double | Pa | 0 | absolute floor of oil pressure; the cavitation flag below |
| `act:fluid:polytropicIndex` | double | 1 | 1.4 | air: `n`; also the accumulators' gas when they give none |
| `act:fluid:supply:kind` | token `pressure`, `fixed_pump`, `variable_pump` | | required | |
| `act:fluid:supply:pressure` | double | Pa | required for `pressure` | gauge |
| `act:fluid:supply:maxFlow` | double | m3/s or kg/s | 0 | 0: unlimited |
| `act:fluid:supply:displacement` | double | m3/rad | required for pumps | `D` (full stroke) |
| `act:fluid:supply:leakage` | double | m3/(s.Pa) | 0 | `k_leak` |
| `act:fluid:supply:mechanicalEfficiency` | double | 1 | 1 | `η_hm` |
| `act:fluid:supply:compensatorPressure` | double | Pa | 0 | gauge; 0: none |
| `act:fluid:supply:compensatorBand` | double | Pa | 0 | |
| `act:fluid:supply:speed` | double | rad/s | 0 | without a shaft joint |
| `act:fluid:supply:driveEfficiency` | double | 1 | 1 | without a shaft joint: shaft power over this is `shaft_power` |
| `act:fluid:supply:returnPressure` | double | Pa | site atmosphere | tank, absolute |
| `rel act:fluid:supply:shaftJoint` | | | none | the joint that turns the pump |
| `rel act:fluid:supply:port` | | | required | the line it feeds |

`FluidLineAPI_1` (single apply). A node: a line, hose or manifold.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `act:fluid:line:volume` | double | m3 | required | |
| `act:fluid:line:wallBulkModulus` | double | Pa | 0 | a hose's compliance; 0: rigid |
| `act:fluid:line:initialPressure` | double | Pa | the return pressure | absolute |

`ValveAPI_1` (single apply). A restriction or a valve.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `act:fluid:valve:kind` | token (§2.5) | | required | |
| `rel act:fluid:valve:portP`, `portT`, `portA`, `portB` | | | | lines; `portT` none: the tank or atmosphere. One path kinds use `portA` → `portB` |
| `act:fluid:valve:center` | token `closed`, `open`, `tandem`, `float` | | `closed` | |
| `act:fluid:valve:dischargeCoefficient` | double | 1 | required for oil | `C_d` |
| `act:fluid:valve:maxArea` | double | m2 | required for oil | open area of each path |
| `act:fluid:valve:sonicConductance` | double | m3/(s.Pa) | required for air | `C` per path |
| `act:fluid:valve:criticalPressureRatio` | double | 1 | 0.528282 | `b`; the fallback is the ideal nozzle's, as the grippers use |
| `act:fluid:valve:overlap` | double | 1 | 0 | dead band at centre, share of travel |
| `act:fluid:valve:leakArea` | double | m2 | 0 | oil, closed |
| `act:fluid:valve:leakConductance` | double | m3/(s.Pa) | 0 | air, closed |
| `act:fluid:valve:naturalFrequency` | double | rad/s | 0 | spool; 0: at once |
| `act:fluid:valve:dampingRatio` | double | 1 | 0 | 0: first order |
| `act:fluid:valve:crackingPressure` | double | Pa | required for `check`, `relief` | |
| `act:fluid:valve:fullOpenPressure` | double | Pa | required for `check`, `relief` | |
| `act:fluid:valve:length`, `diameter` | double | m | required for `pipe` | |
| `rel act:fluid:valve:solenoid` | | | none | a `SolenoidAPI_1` joint that moves the spool |

`CylinderAPI_1` (single apply). On a prismatic joint (joint mode) or its own prim (strut mode).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `act:fluid:cylinder:boreDiameter` | double | m | required | `D` |
| `act:fluid:cylinder:rodDiameter` | double | m | required | `d` |
| `act:fluid:cylinder:rodEnds` | token `single`, `double` | | `single` | |
| `act:fluid:cylinder:action` | token `double_acting`, `single_acting` | | `double_acting` | |
| `act:fluid:cylinder:stroke` | double | m | required | |
| `act:fluid:cylinder:deadVolumeA`, `deadVolumeB` | double | m3 | required | |
| `act:fluid:cylinder:offset` | double | m | 0 | joint mode: `x` = joint position + offset |
| `rel act:fluid:cylinder:portA`, `portB` | | | required (B: none when single acting) | |
| `act:fluid:cylinder:internalLeakage` | double | m3/(s.Pa) | 0 | |
| `act:fluid:cylinder:coulombFriction` | double | N | 0 | `F_c` |
| `act:fluid:cylinder:staticFriction` | double | N | `coulombFriction` | `F_s` |
| `act:fluid:cylinder:stribeckVelocity` | double | m/s | 0 | `v_s`; 0: no Stribeck term |
| `act:fluid:cylinder:viscousFriction` | double | N.s/m | 0 | `c_v` |
| `act:fluid:cylinder:springRate`, `springPreload` | double | N/m, N | 0 | single acting return spring |
| `act:fluid:cylinder:cushionLength` | double | m | 0 | 0: no cushions |
| `act:fluid:cylinder:cushionArea` | double | m2 | required for oil with cushions | |
| `act:fluid:cylinder:cushionConductance` | double | m3/(s.Pa) | required for air with cushions | |
| `rel act:fluid:cylinder:barrelBody`, `rodBody`, `drivenJoint` | | | strut mode | |
| `act:fluid:cylinder:barrelAnchor`, `rodAnchor` | double3 | m | strut mode | in each body's frame |
| `act:fluid:cylinder:closedLength` | double | m | strut mode, required | pin to pin at `x` = 0 |

`AccumulatorAPI_1` (single apply).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `act:fluid:accumulator:volume` | double | m3 | required | `V0`, the shell |
| `act:fluid:accumulator:prechargePressure` | double | Pa | required | `p0`, absolute |
| `act:fluid:accumulator:polytropicIndex` | double | 1 | the circuit's | `n` |
| `rel act:fluid:accumulator:port` | | | required | |

**URDF and overlay.** A cylinder in joint mode is a `<gazebo reference="JOINT">` tag
(`<fluid_cylinder bore= rod= stroke= dead_volume_a= dead_volume_b= port_a= port_b= .../>`),
read beside `<solenoid>`. The rest of a circuit has no link to sit on, so it goes in the URDF's
`<lagrange>` block (formats.md §9.2), as the power system does:

```xml
<lagrange>
  <fluid>
    <supply name="air" medium="air" kind="pressure" pressure="600000" port="manifold"/>
    <line name="manifold" volume="2e-6"/>
    <line name="line_a" volume="1e-6"/>
    <line name="line_b" volume="1e-6"/>
    <valve name="grip_valve" kind="directional" port_p="manifold" port_a="line_a"
           port_b="line_b" sonic_conductance="6e-9" critical_pressure_ratio="0.3"/>
  </fluid>
</lagrange>
```

The overlay YAML takes the same keys, and `lagrange_import` writes the prims under
`/<robot>/fluid/`. The numbers above are illustrative.

### 9. Signals

| Signal | Unit | |
|---|---|---|
| `fluid/<line>/pressure` | Pa | absolute |
| `fluid/<line>/cavitation` | bool | at `minimumPressure` |
| `fluid/<v>/command` | 1 | in, optional: −1…1 (`directional`, `proportional`), 0…1 (`on_off`) |
| `fluid/<v>/spool` | 1 | travel |
| `fluid/<v>/flow` | m³/s | oil, P→A positive |
| `fluid/<v>/mass_flow` | kg/s | air |
| `fluid/<v>/open` | bool | `check`, `relief` |
| `fluid/<v>/loss_power` | W | |
| `fluid/<c>/pressure_a`, `pressure_b` | Pa | absolute |
| `fluid/<c>/position`, `velocity` | m, m/s | `x`, `ẋ` |
| `fluid/<c>/force` | N | `F`, on the rod |
| `fluid/<c>/friction` | N | |
| `fluid/<c>/force_command` | N | in, optional, the `ideal` substitute only |
| `fluid/<c>/cushioning` | bool | |
| `fluid/<c>/natural_frequency` | rad/s | `ω_h` at the current pose |
| `fluid/<s>/pressure`, `flow` | Pa, m³/s or kg/s | at the outlet |
| `fluid/<s>/displacement_command` | 1 | in, optional, `variable_pump` |
| `fluid/<s>/shaft_power`, `loss_power` | W | |
| `fluid/<a>/pressure`, `gas_volume` | Pa, m³ | |
| `fluid/<circuit>/newton_failures` | 1 | count |
| `joint/<j>/torque_command` | N | out, joint mode |
| `body/<b>/wrench/fluid_power` | Wrench | out, strut mode |
| `joint/<j>/friction_limit` | N or N·m | out, the breakaway part of friction |
| `joint/<j>/load_torque` | N·m | out, on a pump's shaft joint |
| `joint/<j>/position`, `velocity`, `effective_inertia` | | in, from physics |

### 10. ROS 2 and ros2_control

As RFC 0020 §5 sets for every actuator, through `SignalTopics`, with a default set written by
the server:

- `/<v>/command` (`std_msgs/Float64`) in for each valve with a command; `/<s>/displacement_command`
  for a variable pump.
- `/<line>/pressure`, `/<c>/pressure_a`, `/<c>/pressure_b`, `/<s>/pressure`
  (`sensor_msgs/FluidPressure`, Pa) out: the signals a pressure transducer reads.
- `/<c>/position` (`std_msgs/Float64`) out: a cylinder's position sensor.
- `Server::commandActuator(name, value)` commands a valve without ROS.

ros2_control: a joint-mode cylinder's joint keeps its `position` and `velocity` state
interfaces. Valves and pressures are GPIO:

```xml
<gpio name="boom_valve">
  <command_interface name="command"/>
  <state_interface name="spool"/>
</gpio>
<gpio name="boom_cylinder">
  <state_interface name="pressure_a"/>
  <state_interface name="pressure_b"/>
  <state_interface name="position"/>
</gpio>
```

`LagrangeSystem` maps a GPIO interface to `act/<gpio>/<interface>` today. Step 6 adds
`fluid/<gpio>/<interface>` when the GPIO names a fluid prim (prim names are unique, so there is
no clash), and `<param name="signal">` still names any signal.
`gpio_controllers/GpioCommandController` (ros2_controllers 6.10.1 for Lyrical, as RFC 0020 §5
checked) then drives the valves. No built-in position loop on a cylinder in this RFC: a user's
controller closes it from `position` to the valve's `command` (open question 7).

### 11. Profile

`domains.fluid_power` in `profiles/schema.json` (an addition): `enabled`, `dt`, `substitute`
(`ideal`, `constant`, `replay`), `source` (replay), `fidelity` (`lumped`, later `fmu`), and
`piston_prediction` (bool, default true).

## Integrate before building (invariant 12)

| Option | Licence (checked 2026-11-22) | Fit |
|---|---|---|
| **Hopsan** (Linköping University): HopsanCore, the default component library and HopsanCLI | Apache-2.0 for the core, component library and CLI; GPL-3.0 for HopsanGUI only (README "License", `LICENSE-ALv2`, `LICENSE-GPLv3`; the default library's own `LICENSE` is Apache-2.0). Latest release v2.24.4 (2026-09-21). HopsanCore bundles SUNDIALS, libnumhop, rapidxml and a CSV parser | A mature fluid power tool with hydraulic and pneumatic libraries (`Valves`, `LinearActuators`, `Pumps&Motors`, `Volumes&Lines`, `Pneumatic`). It is built on transmission line modelling (TLM): components are joined by lines with a delay, so each solves on its own. Embedding it means a second model graph, its own time step tied to line delays, and its own component and parameter system beside USD. **Used as the independent reference** (§12, test 15), as ngspice is for electrical. |
| **OpenHydraulics** (modelica-3rdparty) | BSD-3-Clause (`LICENSE`, "Copyright (c) 2022 Chris Paredis and contributors"); v2.0.0 (2022-10-14), on MSL 4.0.0. Versions 1.x were under the Modelica License 2 | Modelica cylinders, lines, motors and pumps, valves, volumes. Needs a Modelica tool to compile. **Used as a second reference**, and as the target of `lagrange export modelica` for a fluid circuit (open question 9). |
| **Modelica Standard Library, `Modelica.Fluid`** | BSD-3-Clause ("Copyright (c) 1998-2025, Modelica Association and contributors"); 4.1.0 documented | Thermo-fluid pipes, vessels, valves, pumps and machines, with full media models. No ready fluid-power cylinder with Stribeck friction and cushions was found in its package list (TODO(verify) its `Machines` package in full). Heavier than a robot's circuit needs. |
| **An FMU through `fmu_host`** | FMI Library 3.0.4, BSD-3-Clause (dependencies.md). The FMU's own licence is its author's. OpenModelica, the tool that would export it, is OSMC-PL and its FMI 3.0 export is experimental (dependencies.md; the licence of generated FMUs is risk R5) | Works today for a user's own model. But the circuit's topology would be compiled offline, not read from USD, so the 3D view, the schematic and the run could disagree, and an FMU's internal solver is the author's, outside invariant 7. **Offered as fidelity `fmu` later** (step 8), as `electrical` offers it. |

**Proposed: build the network in Lagrange**, as the electrical network is. Reasons:

- The coupling is the product. The piston must be advanced inside the fluid step on physics'
  effective inertia (§3), pumps must load the electrical motors, and losses must reach thermal.
  None of the options above has those seams.
- Topology from USD. The circuit is authored where the robot is, with the same prims the
  viewer shows. No compile step between editing and running.
- Size. Five component laws and a dense Newton solve on tens of unknowns. The validation (§12)
  is the larger part of the work, and the two open tools above make it independent.
- No GPL anywhere in the path: Hopsan's GUI is the only GPL piece, and nothing here uses it.

## Validation

Each model ships with tests against closed forms (CLAUDE.md §10), in `test_fluid_power`, unless
noted. Tolerances are proposals, fixed when the tests first run.

| # | Check | Reference | Tolerance |
|---|---|---|---|
| 1 | Closed oil volume filled at constant flow | `Δp = β_e·Q·t / V` | 0.1 % |
| 2 | Orifice between two fixed pressures | `Q = C_d·A·√(2Δp/ρ)` | 0.1 % above `Δp_t` |
| 3 | Laminar pipe | Hagen–Poiseuille | 0.1 % |
| 4 | Air volume charged through a choked valve from a fixed supply | linear rise `dp/dt = n·R·T·ṁ*/V` while `p ≤ b·p_s`; the subsonic part against a 1 µs RK4 reference of the same ISO 6358 law | 0.5 % |
| 5 | Air volume blown down to the atmosphere | the same, the other way | 0.5 % |
| 6 | Closed pneumatic chamber as a spring | `k = n·p·A²/V`, the mass's period `2π√(m/k)` | 1 % |
| 7 | Hydraulic natural frequency of a loaded cylinder at mid-stroke | `ω_h` of §3 (RFC 0020 §4.8) | 3 % |
| 8 | Steady extension speed from a fixed pump, relief shut | `v = Q/A_A` | 0.5 % |
| 9 | Blocked cylinder behind a relief valve | pressure at the relief's ramp for the pump's flow | 0.5 % |
| 10 | Accumulator charged and discharged | `p·V_g^n` constant | 0.5 % |
| 11 | Breakaway | the piston stays still until `Δp·A` passes `F_s`, then follows the Stribeck curve (Armstrong-Hélouvry et al. 1994) | 1 % of `F_s` |
| 12 | Cushioning | a mass driven into the end at a set speed: peak cushion pressure and rest within the cushion length, against a 1 µs monolithic reference | 2 % |
| 13 | Energy balance over a cycle | pump shaft work = cylinder work + losses + stored (`∫p dV`) | 0.5 % |
| 14 | Coupling (`test_fluid_coupling`) | joint-mode and strut-mode cylinders with physics at 1 ms against a monolithic 1 µs solution, `ω_h·H` from 0.1 to 10 | 2 % RMS in position and pressure, no growth over 10 s; sets the startup limit |
| 15 | Hand check (`tools/ci/check_fluid.py`, like `check_electrical.py`) | the same small circuits (pump, relief, valve, cylinder; supply, 5/2 valve, pneumatic cylinder) in HopsanCLI and in OpenModelica with OpenHydraulics, when installed | within a tolerance set from the first runs, written in docs/validation |
| 16 | Measured crane | *PATU655Stress*, a hydraulically actuated forestry crane (S. Saeed, V. Zhidchenko, H. Handroos; Mendeley Data, version 2, CC BY 4.0): measured valve commands in, chamber and supply pressures and cylinder displacements compared. Fetched once to the cache by `realism.sh`, never committed | RMS errors reported, not gated, until the crane's geometry and valve data are confirmed (TODO(verify): the dataset's pages do not state cylinder sizes or valve ratings; the paper behind it may) |
| 17 | Pneumatic cylinder | Richer and Hurmuzlu (2000), a nonlinear model of a pneumatic force actuator with its parameters | TODO(verify): whether the paper prints all parameters and measured responses (the paper is not openly readable) |
| 18 | Determinism (`test_fluid_power_run`) | the same run twice; the domain off; each substitute | bit for bit |

## Samples

- **Pneumatic gripper on the arm** (`assets/robots/arm6/arm6_pneumatic.urdf`): a parallel
  gripper, two fingers on prismatic joints with a double-acting cylinder each, one 5/2 valve on
  a 6 bar supply, in the arm's workcell picking the same parcel as the suction tool. Closing
  time and grip force follow supply pressure; with the supply cut the fingers relax.
- **Electric mini excavator** (`assets/robots/mini_excavator/`): a tracked base fixed to the
  ground, a boom, an arm and a bucket, each turned by a strut-mode cylinder; a gear pump on an
  electric motor and battery; three 4/3 proportional valves (GPIO), a relief valve and a check
  valve; an optional accumulator. Lifting a load sags the battery; a held load sinks by the
  leakage. Its numbers are illustrative, of the size a 1 t class machine has, not a real
  product's (as the solenoid bench's are).
- **A bench** (`assets/robots/fluid_bench/`): one cylinder with cushions on a slide, for the
  tests and the tutorial.

## Open questions (with a proposed answer)

1. *Own domain `fluid_power`, or a module of `actuators` as RFC 0020 §3.1 placed it?* Own
   domain (§4). The schemas stay in `schemas/actuator` under the reserved names, so RFC 0020's
   reservation holds.
2. *Air: polytropic at a fixed temperature, or chamber temperature as a state with its energy
   equation?* Polytropic first, `n` fallback 1.4. The energy equation (a state per chamber,
   heat exchange with the wall) is a later fidelity `thermal_air`. TODO(verify) the index that
   Richer and Hurmuzlu (2000) and Beater (2007) use for charging and discharging before
   choosing the fallback.
3. *Effective bulk modulus with entrained air, pressure dependent?* Not in the first version:
   the author gives `β` with the air folded in. A pressure-dependent model with an air fraction
   is a later addition (TODO(verify) the form in Jelali and Kroll ch. 4 or Merritt ch. 2).
4. *Friction: Stribeck static map, or LuGre with a bristle state?* Stribeck with the breakaway
   through `friction_limit`. LuGre (Canudas de Wit et al. 1995) later, if the stick-slip of a
   slow pneumatic cylinder is asked for. Pressure-dependent seal friction later as well.
5. *Oil temperature?* Not a state now. Losses are written as `loss_power`; a later step gives
   the tank a `ThermalAPI_1` node and makes `ν` follow its temperature.
6. *Line dynamics (inertance, waves, water hammer)?* Lumped volumes now. For long lines, a
   transmission-line element (TLM, as Hopsan uses; Krus's tutorial) is the natural later
   addition: it has a delay, so it fits a multirate scheduler.
7. *A built-in cylinder position loop (a servo valve with its own controller)?* No: as a real
   machine, the loop is the user's controller. `JointControl` stays motor-less and does not
   drive cylinders.
8. *Hydraulic and pneumatic motors (rotary)?* Later, as `MotorAPI` of this library, reusing the
   pump's equations backwards. Not reserved now.
9. *`lagrange export modelica` for fluid circuits?* Yes, later, to OpenHydraulics components,
   so a user can check a circuit in a Modelica tool as the electrical export allows.
10. *Plugin ABI?* No change. A plugin can write `fluid/<v>/command` and read every `fluid/`
    signal through the bus; a plugin substitute for the domain uses RFC 0008's mechanism.
11. *Viewer?* No new protocol. Scopes and the inspector read the signals. A fluid schematic
    beside the electrical one, and pressure colours on the cylinders, are a later viewer step.
12. *Pre-approve the gated schema merge (step 2) once its tests pass?* Proposed: no; schemas
    are the largest change here, and come back to the owner, as RFC 0020 did for phase 3.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Explicit integration at a small step | the step would follow the smallest volume in the circuit (§4); implicit is stable at any step |
| Embed HopsanCore | a second model graph and parameter system beside USD; TLM's step tied to line delays; the piston prediction and the bus seams would be patches into it. Used as a reference instead |
| FMU per circuit | topology compiled offline, solver outside invariant 7; kept as fidelity `fmu` later |
| Cylinder as a prismatic joint only | cannot close an excavator's loop in URDF; strut mode adds the pin pair |
| Exchange force and speed once per physics step | unstable for `ω_h·H` above about 2 (§3) |
| One RFC with RFC 0020 | RFC 0020 open question 9 chose a separate RFC: these schemas are the largest |

## Steps (each a PR)

1. This RFC.
2. Schemas: `FluidSupplyAPI_1`, `FluidLineAPI_1`, `ValveAPI_1`, `CylinderAPI_1`,
   `AccumulatorAPI_1`; additions test; formats.md §3.16; CHANGELOG; `domains.fluid_power` in
   `profiles/schema.json`.
3. The network core, free of the bus (`modules/fluid_power`): nodes, orifice, pipe and ISO 6358
   flows (the `b` overload in `modules/actuators`), Newton with dense LU; tests 1–6.
4. Cylinders: joint mode, friction with `friction_limit`, cushions, the piston prediction and
   the startup check; tests 7, 11, 12, 14.
5. Pumps, relief and check valves, accumulators; shaft load torque, `shaft_power`,
   `loss_power`; tests 8, 9, 10, 13.
6. Strut mode; URDF `<gazebo>` tag, the `<lagrange><fluid>` block and the overlay; default ROS
   topics; GPIO lookup of `fluid/`; substitutes; the three samples; test 18;
   docs/physics/fluid-power.md and a tutorial with its `tools/ci` script.
7. `tools/ci/check_fluid.py` against HopsanCLI and OpenModelica with OpenHydraulics (test 15);
   the crane dataset in `realism.sh` (test 16); test 17 if its parameters are confirmed.
8. Later: fidelity `fmu`, `lagrange export modelica` for fluid circuits, the open questions'
   later items.

## References

Checked on 2026-11-22. Bibliographic records were checked through Crossref; the books, the
paper and ISO 6358 are not openly readable, so their equations and sections stay marked
TODO(verify) and are read before the step that implements them.

- ISO 6358-1:2013, *Pneumatic fluid power: determination of flow-rate characteristics of
  components using compressible fluids*, Part 1 (sonic conductance `C`, critical pressure ratio
  `b`; TODO(verify) the clause, as RFC 0020 §4.7 notes).
- H. E. Merritt, *Hydraulic Control Systems*, Wiley, 1967 (TODO(verify) chapters, as RFC 0020
  §4.8 notes).
- M. Jelali and A. Kroll, *Hydraulic Servo-systems: Modelling, Identification and Control*,
  Springer (Advances in Industrial Control), 2003. DOI 10.1007/978-1-4471-0099-7.
- N. D. Manring and R. C. Fales, *Hydraulic Control Systems*, 2nd ed., Wiley, 2019.
  DOI 10.1002/9781119418528 (a modern alternative to Merritt; TODO(verify) the edition).
- P. Beater, *Pneumatic Drives*, Springer, 2007. DOI 10.1007/978-3-540-69471-7.
- E. Richer and Y. Hurmuzlu, "A High Performance Pneumatic Force Actuator System: Part I,
  Nonlinear Mathematical Model", *J. Dyn. Sys., Meas., Control* 122(3), 416–425, 2000.
  DOI 10.1115/1.1286336.
- B. Armstrong-Hélouvry, P. Dupont and C. Canudas de Wit, "A survey of models, analysis tools
  and compensation methods for the control of machines with friction", *Automatica* 30(7),
  1083–1138, 1994. DOI 10.1016/0005-1098(94)90209-7.
- C. Canudas de Wit, H. Olsson, K. J. Åström and P. Lischinsky, "A new model for control of
  systems with friction", *IEEE Trans. Automatic Control* 40(3), 419–425, 1995.
  DOI 10.1109/9.376053.
- S. Saeed, V. Zhidchenko and H. Handroos, *A Dataset from a Hydraulically Actuated Forestry
  Crane for Data-Driven Stress Estimation (PATU655Stress)*, Mendeley Data, version 2, CC BY 4.0.
  https://data.mendeley.com/datasets/rn5j42rpyr
- Hopsan, Linköping University. https://github.com/Hopsan/hopsan (licences in its README,
  `LICENSE-ALv2`, `LICENSE-GPLv3`; release v2.24.4).
- P. Krus, *Transmission Line Modelling for System Simulation* (Hopsan tutorial).
  https://hopsan.github.io/tutorials/mainTLMtutorial.pdf
- OpenHydraulics, modelica-3rdparty, v2.0.0, BSD-3-Clause.
  https://github.com/modelica-3rdparty/OpenHydraulics
- Modelica Standard Library, BSD-3-Clause. https://github.com/modelica/ModelicaStandardLibrary ;
  `Modelica.Fluid` documentation: https://build.openmodelica.org/Documentation/Modelica.Fluid.html
- The design this continues: [RFC 0020](0020-actuators.md) §4.7, §4.8, §5, §6 step 7.
