# RFC 0020: Actuators: what Lagrange has, and which to add

| | |
|---|---|
| Status | Accepted (2026-11-17, by the owner: every open question as proposed) |
| Created | 2026-11-17 |
| Affects | `schemas/electrical` (appended tokens and optional attributes), `schemas/actuator` (one new library), `modules/electrical/` (a stepper model, a position mode, screw back-drive), `modules/actuators/` (new: servo electronics, grippers, solenoids, later fluid power), `modules/rigid_body/` (one optional input per joint), `server/` (JointControl, ROS topics), `bridges/ros2_control/` (GPIO interfaces), `io/importers/` (URDF `<gazebo>` forms), `profiles/schema.json` (additions), `docs/` |
| Not affected | `proto/` (the viewer reads the new signals through the existing signal subscription), `core/plugin_abi/` (open question 12) |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0005 (profiles), RFC 0007 (the 1.x freeze), RFC 0008 (plugins, substitutes), RFC 0010 (electrical nodes), RFC 0019 (Draft: impedance mode, `JointTransmissionAPI_1`) |

## Summary

An inventory of every way a Lagrange robot moves itself today, and a ranked plan for the
actuators it still lacks: suction and magnetic grippers, hobby and smart servos, lead screws and
electric linear actuators, steppers, solenoids, and, last, pneumatic and hydraulic cylinders
with their valves. Each one gets model equations, a published reference or closed form to
check against, its signals with units, how it is authored (URDF `<gazebo>` tags, an overlay, USD
schema additions), its ROS 2 and ros2_control interface, its behaviour on each tier, and its
substitute. Everything is an addition under RFC 0007. Where the existing electrical motor can
do the job, the new actuator reuses it rather than duplicating it.

## Motivation

The owner asked for two things: list the actuators Lagrange already models, and decide which to
add. The arm sample shows the gap. Its gripper opens and closes, but it has no object to hold
(robot-classes.md §4: "there is no object to grasp"). Small robots built from hobby servos or
Dynamixels have no servo model. 3D-printer-style gantries have no stepper. Electric linear
actuators lack the self-locking screw that sets their behaviour. Mobile manipulators in
logistics pick parcels with suction cups, and Lagrange cannot simulate that either. These
are common actuators, and their behaviour (a servo's deadband, a stepper's missed steps, a
screw that holds its load with the power off, a suction cup that lets go on a porous box) is
exactly what robot software has to handle.

## 1. Inventory: what exists (2026-11-17, origin/dev at 1.4.0)

| Actuator | Where | Inputs | Model | Notes |
|---|---|---|---|---|
| PMSM / BLDC / brushed DC motor + drive | `ElectricalMotorAPI_1` (`elec:motorType` `pmsm`, `bldc`, `dc_brushed`), `modules/electrical/network` | `joint/<j>/torque_command`; `velocity_command` (`elec:controlMode = velocity`, PI speed loop); `voltage_command` (`voltage`, no current loop) | averaged dq / DC model, FOC current loops, bus voltage limit, `I²R(T)`, gear ratio and efficiency, rotor inertia; prismatic joints through `elec:gearRatio` in rad/m | electrical.md §2–§3; `ideal` and `replay` substitutes |
| Battery, bus, converters, loads, fuses, switches, diodes | `BatteryAPI_1`, `ElectricalBusAPI_1`, `ElectricalConverterAPI_1`, `ElectricalLoadAPI_1` (`signal` model takes `load/<n>/power_demand`), `ElectricalBranchAPI_1` | | ECM battery, nodal bus | RFC 0010 |
| Rotors (propellers) | `RotorAPI_1`, `modules/rotor_aero` | the motor on the rotor joint | coefficient or BEM thrust and torque, ground effect, downwash | |
| Road vehicle powertrain, brakes, steering | `VehicleAPI_1`, `WheelAPI_1`, `modules/vehicle_dynamics` | `/cmd_vel`, `/ackermann_cmd` | electric or `ice_map` powertrain, brakes, Ackermann steering, MF-Tyre | |
| Winch | `TetherAPI_1` `tether:winch` (`fixed`, `tension`, `free`) | | speed-limited pay-out | RFC 0011; no electrical load yet |
| Joint servo loops without a motor | `server::JointControl` (`--joint-control`, `/joint_commands`, `Server::commandJoint`) | position, velocity, effort, voltage per joint | critically damped second-order loop at `--joint-loop-hz`, clamped to the effort limit | the "ideal servo" of today |
| ros2_control hardware | `lagrange_ros2_control/LagrangeSystem` | `position`, `velocity`, `effort` command interfaces | through `torque_command` | no GPIO interfaces |
| MAVLink ESCs | `bridges/mavlink` | PX4 / ArduPilot actuator outputs | rotor speed loops (`Escs`) | no servo or gripper outputs |
| Joint damping, friction, mimic | `JointDynamicsAPI_1` | | passive | |
| Kinematic bodies | `body/<b>/pose_command` | | moved exactly (traffic, scenarios) | |
| Impedance mode, backlash, series elastic | RFC 0019 (Draft) | `kp`, `kd`, `τ_ff` | | not built |
| Gripper *sensing* | sensor kind `gripper_sensor` | | opening, current, held, slip | sensing only: nothing holds an object but friction |

What is missing falls into three groups. Some actuators *hold* things without friction (suction,
magnets). Some have their own servo electronics (hobby servos, Dynamixels, steppers with
drivers). Some move by something other than an electric motor's torque (screws that self-lock,
solenoids, fluids).

## 2. Ranking

Ranked by what users can do with the actuator, how much existing code it reuses, and the risk.

| Rank | Actuator | Why this rank | Size |
|---|---|---|---|
| 1 | Suction cup (vacuum gripper) | pick-and-place is the arm sample's missing piece and the most common mobile-manipulation end effector; needs no engine change (§4.1) | M |
| 2 | Magnetic gripper (electromagnet, electropermanent) | same machinery as suction; drones and bin picking; `sensmat:relativePermeability` already exists | S |
| 3 | Hobby PWM servo and smart servo (Dynamixel-like) | the largest group of small robots; the motor is the existing `dc_brushed` in `voltage` mode, so the new work is the servo's electronics | M |
| 4 | Lead screw / electric linear actuator | a back-drive efficiency and a load-dependent friction; prismatic motors exist already | S |
| 5 | Stepper motor with a chopper driver | gantries, sliders, printers, camera pan-tilt units; a new motor model in the electrical domain | M |
| 6 | Solenoid (push/pull, latch, valve) | small; locks, kickers, valves; needed by pneumatics | S |
| 7 | Pneumatic cylinder and valves | factory grippers, soft robots; a new fluid network | L |
| 8 | Hydraulic cylinder, pump, servo valve | excavators, heavy legged robots (RFC 0019 non-goal); the largest and stiffest | L |

## 3. Shared design

### 3.1 Where the code goes

- **Motor physics stays in `electrical`.** A servo's motor is a `dc_brushed` motor. A stepper
  is a new `elec:motorType` value. The screw is a property of the motor's gear. That keeps one
  owner for current, back-EMF, bus load, `motor/<m>/loss_power` heat and EMI. Invariant 1 holds
  because modules only exchange signals.
- **Actuator electronics and non-motor actuators go in a new domain, `actuators`**
  (`modules/actuators`, package `lagrange_actuators`, 1 ms by default, 100 µs for solenoids and
  fluid power). It holds one module per family: `servo`, `gripper`, `solenoid`, and later
  `fluid_power`. Each runs only when its prims exist, as `emi` and `comms` do. `--domain
  actuators=off` and profile `domains.actuators` pick its substitute.
- **Holding is done with forces, never with welds.** A gripper pulls the object toward itself and
  pushes itself toward the object with an equal and opposite force, through
  `body/<b>/wrench/<source>`, the same path that hydrodynamics, the tether and rain use. Shear is
  then carried by contact friction from the physics engine. That is how a suction cup works, it
  works the same on MuJoCo, Jolt and a plugin backend, and the object falls off when the
  physics says it should. Alternatives are in the last section.

### 3.2 Schemas (RFC 0007: additions only)

- `schemas/electrical`: tokens appended to `elec:motorType` (`stepper`) and `elec:controlMode`
  (`position`), and new optional attributes whose fallbacks keep today's behaviour (§4.3–§4.5).
- New library `schemas/actuator`, namespace `act:`: `ServoAPI_1`, `SuctionGripperAPI_1`,
  `MagnetGripperAPI_1`, `SolenoidAPI_1`, and in phase 3 `FluidSupplyAPI_1`, `ValveAPI_1` and
  `CylinderAPI_1`.
- `frozen_1_0.json` is not changed. Each new class is added to `schemas/CHANGELOG.md` and
  formats.md. The overlay YAML (formats.md §9) applies any registered schema, so overlays need
  no new code.

### 3.3 Substitutes (invariant 3)

| Family | `ideal` | `constant` | `replay` |
|---|---|---|---|
| servo | `JointControl`'s position loop, bounded by the servo's torque–speed line (§4.3) | holds the start position | `joint/<j>/torque` from MCAP (already the electrical replay) |
| gripper | holds when commanded and in contact, with no pressure dynamics and no leak | never holds | `act/<g>/holding` and the wrenches from MCAP |
| solenoid | force = rated force while on | off | from MCAP |
| fluid power | cylinder force = commanded force, bounded by supply pressure × area | locked at the start | from MCAP |

`zero` (everything off) is the profile's `actuators: {enabled: false}`, as for the other domains.

### 3.4 Tiers

Every model in this RFC runs on the CPU, deterministically (invariant 7), on Tier 0, 1 and 2
alike. None uses the GPU. Suction sealing uses the scene's ray queries (`ShapeCaster`, already
used by the CPU lidar and the plugin ABI's `scene_rays`), which are deterministic on every tier.

## 4. The actuators

### 4.1 Suction cup (rank 1)

**Model.** Cup pressure `p` in the cup volume `V` (with the hose's), from the ideal gas law at
constant temperature:

```
V/(R·T) · dp/dt = ṁ_leak(p) − ṁ_gen(p)
F_hold = (p_atm − p) · A_eff        (A_eff: the cup's effective area)
```

`ṁ_gen` is the vacuum generator: an ejector or pump curve given as a linear one, suction flow
`q0` at atmospheric falling to zero at the generator's ultimate vacuum `p_min`. `ṁ_leak` is the
flow through the seal: zero when sealed. Unsealed, or on a porous surface, it is an orifice of
area `A_leak`, with ISO 6358's choked/unchoked flow (`act:suction:leakConductance`). Sealing
happens when the cup's rim touches one body's surface all round. That is tested with `n` rays
(default 8) along the rim normal: every hit is within `act:suction:sealGap` of the rim, on the
same body, and the surface normal is within `act:suction:maxTilt` of the cup's axis. Porosity is
a property of the target, so `SensorMaterialAPI_1` gains an optional
`sensmat:airPermeability` (open question 3). The atmospheric pressure is `env/pressure` when the
weather runs, else the site's (101 325 Pa when none is given, the ISO 2533 standard atmosphere
already used by the barometer).

**Force path.** While `p < p_atm`, `F_hold` acts along the cup axis on the target body (toward
the cup) and in reverse on the cup's body, both at the cup centre. These are
`body/<target>/wrench/suction` and `body/<cup body>/wrench/suction`. The engine's contact between
the lip collider and the target carries the reaction and the friction. When the pull-off load
exceeds `F_hold`, the target separates and the seal breaks (`p` rises through the leak), and the
part drops. The pull is applied only while sealed, so a cup can never pull a part through the
air.

**Reference.** `F = Δp·A` is the definition of pressure. The vendor engineering guides add a
safety factor and a friction term for shear: Schmalz's "Theoretical holding force of a suction
cup" gives `F_TH = m·(g + a)·S` (cup horizontal, force vertical), `m·(g + a/μ)·S` (cup
horizontal, force horizontal) and `(m/μ)·(g + a)·S` (cup vertical), with S at least 1.5 for
smooth, dense parts and 2.0 or more for "critical, heterogeneous, porous, rough or oiled"
ones, and μ to be measured on the part (reference values 0.2–0.3 wet, 0.5 wood, metal, glass,
stone, 0.6 rough). Lagrange computes the physics without S; the sample sizing in the docs uses
these forms. The evacuation time of a sealed volume through a
linear pump curve has a closed form (exponential approach to `p_min`).

**Validation.** (a) A sealed cup holding a hanging mass increased step by step lets go at
`Δp·A` within 1 %. (b) Pressure decay while evacuating matches the closed form within 1 %. (c) A
part held sideways slides at `μ·Δp·A` within 3 % (the engine's friction). (d) A cup tilted
beyond `maxTilt`, or over an edge, never seals. (e) Determinism.

**Schema `SuctionGripperAPI_1`** (on the cup's link):

| Attribute | Unit | Fallback | |
|---|---|---|---|
| `act:suction:diameter` | m | required | effective diameter |
| `act:suction:volume` | m³ | required | cup + hose |
| `act:suction:flow` | kg/s | required | generator's suction at atmospheric |
| `act:suction:ultimatePressure` | Pa | required | `p_min`, absolute |
| `act:suction:sealGap` | m | 0.002 | (a modelling tolerance, not a device value) |
| `act:suction:maxTilt` | rad | 0.26 | (same) |
| `act:suction:leakConductance` | m³/(s·Pa) | 0 | ISO 6358 `C` of an open cup |
| `act:suction:axis` | 1 | 0, 0, 1 | in the link frame |

The required numbers come from the datasheet. No fallback is a device value.

**Signals.** In: `act/<g>/command` (Bool). Out: `act/<g>/pressure` (Pa), `act/<g>/sealed`
(Bool), `act/<g>/holding` (Int: the target body index, −1 for none), `act/<g>/force` (N), and
the wrenches. A vacuum switch is a threshold on `act/<g>/pressure`, read through `--ros-topics`
or a plugin.

**Authoring.** URDF `<gazebo reference="tool0"><suction_cup diameter=... volume=... flow=...
ultimate_pressure=.../></gazebo>`, the overlay's `suction_grippers:` list, or USD.

**ROS 2.** `/<g>/command` `std_msgs/Bool`, `/<g>/state` `std_msgs/Bool` (holding) and
`/<g>/pressure` `sensor_msgs/FluidPressure`. In ros2_control, a `<gpio name="<g>">` with command
interface `vacuum` and state interfaces `pressure` and `holding` (§5).

### 4.2 Magnetic gripper (rank 2)

**Model.** The pull of an electromagnet across an air gap `g` onto a ferromagnetic target, from a
magnetic circuit of two gaps in series with the iron:

```
B = μ0·N·i / (2g + l_iron/μr_eff)
F = B²·A_pole / μ0                  (both poles, A_pole each; Maxwell stress)
```

The target's `μr` is `sensmat:relativePermeability` (it exists). It is saturated at
`act:magnet:saturation` (T): a thin sheet saturates and holds less. The coil is an R–L circuit
on its bus port (an `ElectricalLoadAPI_1` of model `signal`, whose `power_demand` the module
writes). An electropermanent magnet is a latch: a pulse switches it on or off, and it holds
with no current. The force is applied by the same wrench pair as suction, at the gap computed
by one ray per pole.

**Reference.** The Maxwell stress pull `B²A/(2μ0)` per pole face (any electromagnetism text;
Furlani, *Permanent Magnet and Electromechanical Devices*, Academic Press (Elsevier) 2001, has a
chapter "Electromechanical Devices", DOI 10.1016/B978-012269951-1/50006-1, per Crossref;
TODO(verify) the equation and its page: the book is not openly readable). Validation: the force against the closed form at 5 gaps within
1 %, and an electromagnet dropping a part at the gap and current the closed form gives.

**Schema `MagnetGripperAPI_1`:** `act:magnet:kind` (`electro`, `electropermanent`),
`turns`, `poleArea`, `ironPathLength`, `ironPermeability`, `saturation`, `coilResistance`,
`coilInductance`, `ratedVoltage`, `axis`. Signals: `act/<g>/command`, `act/<g>/current` (A),
`act/<g>/force` (N), `act/<g>/holding`. ROS as §4.1.

### 4.3 Hobby PWM servo and smart servo (rank 3)

A hobby servo is a brushed DC motor, a gear train and a position loop on a potentiometer,
driven by a pulse (typically 1–2 ms at 50 Hz; the map is the servo's). A smart servo
(Dynamixel-class) has the same parts plus an encoder, current sensing, a control table (PID
gains, velocity and acceleration profile limits, current limit) and a serial bus.

**Model (fidelity `electrical`, when the electrical domain runs).** The motor is the existing
`ElectricalMotorAPI_1` with `motorType = dc_brushed` and `controlMode = voltage`. The new
`actuators/servo` module is the servo's board. It reads the command and `joint/<j>/position`
(quantised by the potentiometer's or encoder's resolution) and writes
`joint/<j>/voltage_command`:

```
e   = deadband(θ* − θ, act:servo:deadband)
u   = clamp(Kp·e + Ki·∫e + Kd·ė, −1, 1)                  (duty cycle)
v   = u · V_bus                                          (H-bridge)
θ*  follows the command through the profile limits (velocity, acceleration) when set
```

A hobby servo's board is only `Kp` with a deadband (`Ki = Kd = 0`). A smart servo exposes all
three, plus a current limit, which is the existing `elec:maxCurrent`. The stall torque and the
no-load speed then come out of the motor model (`τ_stall = Kt·V/R`, `ω0 = V/Ke`, the line between
them). Writing them out is how a servo is checked against its datasheet.

**Model (fidelity `ideal`, without the electrical domain).** `τ = clamp(Kp'·e − Kd'·ω, ±τ_line(ω))`
with `τ_line(ω) = τ_stall·(1 − |ω|/ω0)`: the datasheet's two numbers and nothing else.

**References.** The DC motor's linear torque–speed line (any machines text; Hughes and Drury,
*Electric Motors and Drives*, Newnes (Elsevier) 2019, ISBN 978-0-08-102615-1, chapter 3 "D.C.
motors", DOI 10.1016/B978-0-08-102615-1.00003-9, per Crossref; TODO(verify) the section: the
book is not openly readable). The Dynamixel control table (ROBOTIS e-Manual, XM430-W350 page,
read 2026-11-17): `Operating Mode` (11; 0 current, 1 velocity, 3 position (default), 4 extended
position, 5 current-based position, 16 PWM), `Position D/I/P Gain` (80, 82, 84; no unit),
`Profile Acceleration` (108; 214.577 rev/min² velocity-based, or 1 ms time-based),
`Profile Velocity` (112; 0.229 rev/min), `Goal Position` (116; 1 pulse, 0.088°, 4096 per
turn), `Current Limit` (38) and `Present Current` (126), both 2.69 mA; at 12 V a stall torque of
4.1 N·m at 2.3 A and 46 rev/min no load. The gains' conversion to physical units is on the same
page and is read before step 4. **Validation:** stall torque and no-load speed against
the datasheet's two numbers at rated voltage within 2 %. A step response that stops inside the
deadband (hunting-free). A profile move whose velocity is trapezoidal to 1 %. The ideal and
electrical fidelities agree on the torque–speed line within 2 %.

**Schema `ServoAPI_1`** (on the joint):

| Attribute | Unit | Fallback | |
|---|---|---|---|
| `act:servo:kind` | | `pwm` | `pwm`, `smart` |
| `act:servo:pulseMin`, `pulseMax` | s | required for `pwm` | the pulse widths of the two ends |
| `act:servo:angleMin`, `angleMax` | rad | required for `pwm` | the joint positions they map to |
| `act:servo:deadband` | rad | 0 | |
| `act:servo:kp`, `ki`, `kd` | 1/rad, 1/(rad·s), s/rad | `kp` required, others 0 | duty per error |
| `act:servo:profileVelocity`, `profileAcceleration` | rad/s, rad/s² | 0 (none) | |
| `act:servo:resolution` | rad | 0 | position sensing step |
| `act:servo:stallTorque`, `noLoadSpeed` | N·m, rad/s | required for the `ideal` fidelity only | |

**Signals.** In: `act/<s>/pulse_width` (s; written by a PWM bridge such as ArduPilot's servo
outputs) or `act/<s>/goal_position` (rad). Out: `act/<s>/present_position`, `present_velocity`,
`present_current` (A), `act/<s>/overload` (Bool, after the smart servo's shutdown rules; open
question 6).

**ROS 2 / ros2_control.** `LagrangeSystem`'s `position` command interface on a joint with
`ServoAPI_1` goes to `goal_position` rather than through `JointControl`. State interfaces are
`position`, `velocity`, `effort` (from the current). `/joint_commands`' position does the same.
A ROS 2 hardware package for real Dynamixels exists: ROBOTIS's `dynamixel_hardware_interface`,
released for Lyrical at 1.5.1 (rosdistro `lyrical/distribution.yaml`, 2026-11-17), so a user
can swap it for `LagrangeSystem` with the same controllers.

### 4.4 Lead screw and electric linear actuator (rank 4)

**Model.** A motor on a prismatic joint already maps `elec:gearRatio` in rad/m with
`elec:gearEfficiency`. Today the back-driven direction divides by the same efficiency
(`Models.cpp`). A screw's efficiencies differ by direction. For a thread of lead angle `λ` and
friction angle `φ = atan(μ / cos α_n)`, the efficiencies are:

```
η_forward  = tan λ / tan(λ + φ)        (torque → force)
η_back     = tan(λ − φ) / tan λ        (force → torque);  η_back ≤ 0 ⇔ φ ≥ λ: self-locking
```

A self-locking screw holds any axial load with the motor off. It is modelled as a Coulomb
friction limit on the prismatic joint that grows with the load,
`F_f = |F_axial|·(1 − η_back⁺)` when back-driven. This needs one new optional input to the
rigid-body contract, `joint/<j>/friction_limit` (N or N·m, this step's Coulomb limit, added to the
static `JointDynamicsAPI_1` friction). MuJoCo can take it per step as the dof's `frictionloss`
and Jolt as the constraint's friction. Checked on the versions in use: MuJoCo 3.12.0's
`mjModel::dof_frictionloss` (`mjmodel.h`), a real-valued field, which MuJoCo's "Model changes"
documentation says is safe to change between steps ("real-valued parameters are safe to change,
while structural integer parameters are not"); Jolt 5.6.0's `HingeConstraint::SetMaxFrictionTorque`
and `SliderConstraint::SetMaxFrictionForce`. The
conformance suite checks it. The same input serves worm gears on revolute joints.

**Reference.** The power-screw equations of Shigley's *Mechanical Engineering Design* (Budynas
and Nisbett, 11th ed., McGraw-Hill, chapter 8, section 8–2 "The Mechanics of Power Screws", per
the publisher's list of what the 11th edition changed; the equations themselves not re-read).
**Validation:** `η_forward` and `η_back` against the closed form for a 2 mm-lead trapezoidal
thread and a ball screw. A self-locking actuator holds 100 kg with the motor unpowered (no creep
over 60 s beyond the engine's contact compliance). A back-drivable ball screw drops the load
at the speed the motor's back-EMF and damping give.

**Schema** (appended to `ElectricalMotorAPI_1`): `elec:gearBackEfficiency` (1, fallback −1:
"the same as `gearEfficiency`", which is today's behaviour; open question 4). An overlay helper
`lead_screw: {lead, diameter, friction, thread_angle}` computes `gearRatio = 2π/lead` and both
efficiencies. Limit switches are the existing joint limits plus a `bumper`-style kind. ROS is
unchanged: the motor's modes already cover a linear actuator.

### 4.5 Stepper motor with a chopper driver (rank 5)

**Model.** A two-phase hybrid stepper with `N_r` rotor teeth (50 for 1.8° steps), per phase `R`,
`L` and back-EMF constant `K_m`:

```
v_a = R·i_a + L·di_a/dt − K_m·ω·sin(N_r·θ)
v_b = R·i_b + L·di_b/dt + K_m·ω·cos(N_r·θ)
τ   = −K_m·i_a·sin(N_r·θ) + K_m·i_b·cos(N_r·θ) − T_d·sin(4·N_r·θ)
```

with detent torque `T_d`. The driver is a current chopper. Each phase's reference is the
microstep table's sine and cosine at the commanded step position, times the run or hold current.
The bridge voltage is `±V_bus`, averaged over the PWM period like the existing drives. At speed,
the phase current cannot follow its reference (`L/R` and back-EMF), and the torque falls. That
gives the pull-out torque curve, which the model produces rather than looks up. Missed steps also
emerge: when the load angle passes a quarter of a tooth pitch, the rotor slips to the next
equilibrium. The electrical step must resolve the electrical frequency `N_r·ω/2π` (250 Hz at
1000 full steps/s, at 100 µs that is 40 steps per period). A warning fires below 20.

**Reference.** The standard model above: Acarnley, *Stepping Motors: a guide to theory and
practice* (IET, 4th ed. 2002), and Kenjo and Sugawara, *Stepping Motors and their
Microprocessor Controls* (Oxford, 2nd ed. 1994) (both books found on Crossref: Acarnley, IET,
DOI 10.1049/PBCE063E; Kenjo and Sugawara, OUP, second edition; TODO(verify) the chapters and the
detent term's harmonic: neither text is openly readable). **Validation:** the static torque curve `τ(θ)` at rated current against
`K_m·I·sin(N_r·Δθ)` within 1 %. Holding torque against the datasheet value (the user's motor). The
pull-out curve against the steady-state phase circuit's closed form,
`I(ω) = V/√(R² + (N_r·ω·L)²)` under full voltage, within 5 % (TODO(verify) whether a published
measured pull-out curve with full motor parameters can be found, and add it: not searched in
this pass). A deliberate
overload loses whole steps (multiples of 4 full steps), never fractions.

**Schema** (appended to `ElectricalMotorAPI_1`; used when `motorType = stepper`):
`elec:stepper:rotorTeeth` (fallback 50), `elec:stepper:detentTorque` (N·m, 0),
`elec:stepper:holdCurrent` (A, fallback = `maxCurrent`), `elec:stepper:microsteps` (fallback 1).
The phase `R` and `L` and `Kt` reuse `windingResistance`, `inductance` and `torqueConstant` (the
holding torque per ampere).

**Signals.** In: `joint/<j>/step_target` (Int, microsteps) or `joint/<j>/step_rate` (Hz, signed)
and `joint/<j>/stepper_enable` (Bool). Out: `motor/<m>/current_a`, `current_b`,
`motor/<m>/lost_steps` (Int, from the rotor angle against the step count, which a real driver
does not know but a test wants). ros2_control's `position` command converts to microsteps, and
`velocity` to a step rate. The ideal substitute moves the joint exactly by the steps.

### 4.6 Solenoid (rank 6)

**Model.** A plunger in a coil, with inductance `L(x)` along its stroke:

```
v = R·i + d(L(x)·i)/dt
F = ½·i²·dL/dx − k·(x − x0)          (magnetic pull, return spring)
```

`L(x)` is given as a table over the stroke, or by the two-gap magnetic circuit of §4.2. The
solenoid is a prismatic joint that the module drives with `joint/<j>/torque` (N on a slide) in
place of a motor, plus a load on the bus. Back-EMF from the moving plunger appears in the coil
current, which is how a real valve driver detects that the plunger has moved.

**Reference.** The co-energy force `½ i² dL/dx` (any electromechanics text; TODO(verify) cite the
edition: no openly readable text was found in this pass). **Validation:** the pull at three strokes against the closed form for a table `L(x)`
within 1 %, and the current's dip as the plunger moves.

**Schema `SolenoidAPI_1`:** `act:solenoid:inductanceTable` (pairs of m and H), `resistance`,
`springRate`, `springPreload`, `stroke`, `ratedVoltage`. Signals: `act/<s>/command` (Bool or duty),
`act/<s>/current`, `act/<s>/position`, `act/<s>/force`.

### 4.7 Pneumatic cylinder and valves (rank 7, phase 3)

**Model.** Each chamber is an ideal gas volume, polytropic:

```
dp/dt = (n·R·T/V)·ṁ − (n·p/V)·dV/dt
ṁ     = ISO 6358: C·p_up·ρ0·√(T0/T_up) · { 1 if p_dn/p_up ≤ b;  √(1 − ((p_dn/p_up − b)/(1 − b))²) otherwise }
F     = p_A·A_A − p_B·A_B − p_atm·(A_A − A_B) − F_friction(v)
```

Valves are on/off (a solenoid, §4.6), proportional, or check valves. The supply is a
reservoir at a regulated pressure.

**References.** ISO 6358-1:2013 (the sonic conductance `C` and critical pressure ratio `b`;
TODO(verify) the clause's exact form: iso.org refused the request and the standard is sold).
Beater, *Pneumatic Drives* (Springer 2007, DOI 10.1007/978-3-540-69471-7, per Crossref).
**Validation:** charging time of a closed volume through choked flow against the closed form; a
cylinder's extension time under a load against a measured case from Beater (TODO(verify) one with
all parameters: the book is not openly readable).

### 4.8 Hydraulic cylinder, pump and servo valve (rank 8, phase 3)

**Model.**

```
dp_A/dt = β/V_A(x) · (Q_A − A_A·ẋ − Q_leak)
Q       = C_d·A_v(x_v)·√(2·|Δp|/ρ)·sign(Δp)       (orifice)
F       = p_A·A_A − p_B·A_B − F_friction(ẋ)
```

These are spool valve dynamics as a first- or second-order lag, a pump with a relief valve, and
a tank. The stiff chambers (the hydraulic natural frequency
`ω_h = √(β·A²/m·(1/V_A + 1/V_B))` reaches hundreds of Hz) run at 100 µs with a semi-implicit
update.

**References.** Merritt, *Hydraulic Control Systems* (Wiley 1967) and Jelali and Kroll,
*Hydraulic Servo-systems* (Springer 2003; its chapter 4 is "Physically Based Modelling", DOI
10.1007/978-1-4471-0099-7_4, per Crossref) (TODO(verify) the chapters' sections and Merritt's
chapter: neither book is openly readable). **Validation:** `ω_h` of a
loaded cylinder at mid-stroke against Merritt's closed form within 3 %. Steady speed `Q/A`.
The relief valve caps the pressure.

Pneumatics and hydraulics share one module (`fluid_power`) and one set of schemas
(`FluidSupplyAPI_1`, `ValveAPI_1`, `CylinderAPI_1`, with `act:fluid:medium` `air` or
`hydraulic_oil`). They get their own small RFC before phase 3, since their schemas are the
largest here (open question 9).

## 5. ROS 2 and ros2_control, common to all

- Every actuator gets topics under its name, as sensors do: commands `std_msgs/Bool` or
  `Float64`, and states `Bool`, `Float64`, `sensor_msgs/FluidPressure`. Mapping is through the
  bridge's existing `SignalTopics` machinery (`--ros-topics`), with a default set written by the
  server so that nothing must be configured.
- ros2_control: `LagrangeSystem` gains `<gpio>` command and state interfaces bound to actuator
  signals by name (`<gpio name="suction"><command_interface name="vacuum"/>...`). GPIO
  interfaces are part of ros2_control's hardware description, and `gpio_controllers`'
  `GpioCommandController` is released for Lyrical (`ros2_controllers` 6.10.1 in rosdistro, whose
  `gpio_controllers_plugin.xml` exports `gpio_controllers::GpioCommandController`). Joint interfaces stay as
  they are, routed to a servo's or stepper's own command when the joint has one.
- MAVLink: ArduPilot's and PX4's servo and gripper outputs map to `act/<s>/pulse_width` and
  `act/<g>/command`. ArduPilot: `SERVOx_FUNCTION` = 28 (Gripper), with `GRIP_TYPE` 1 (servo) and
  `GRIP_GRAB`, `GRIP_NEUTRAL`, `GRIP_RELEASE` in µs (Copter docs, "Grippers"). PX4: the
  `Gripper` output function in Actuator Configuration, with `PD_GRIPPER_TYPE` and
  `PD_GRIPPER_TO`, and the payload deliverer module built in (PX4 user guide, "Gripper").

## 6. Steps (each a PR)

1. This RFC.
2. **Phase 1, grippers:** `schemas/actuator` with `SuctionGripperAPI_1` and `MagnetGripperAPI_1`,
   `sensmat:airPermeability`, `modules/actuators` (gripper module, substitutes), signals, ROS
   topics, URDF tags, the arm sample with a suction tool picking a box, tests of §4.1–§4.2,
   docs/physics/actuators.md.
3. **Phase 1, screws:** `elec:gearBackEfficiency`, `joint/<j>/friction_limit` in both engines and
   the conformance suite, the overlay helper, tests of §4.4.
4. **Phase 2, servos:** `ServoAPI_1`, `elec:controlMode = position` (for a smart servo whose
   loop runs on its own motor drive), the servo module, ros2_control routing, a small sample
   (a pan-tilt of two hobby servos), tests of §4.3.
5. **Phase 2, steppers:** the stepper motor model and driver in `electrical`, a gantry sample,
   tests of §4.5.
6. **Phase 2, solenoids** and GPIO in `LagrangeSystem`.
7. **Phase 3:** the fluid-power RFC, then the module.

## Also worth adding

- **Holding brakes** on joints (spring-applied, released by a powered coil): every industrial arm
  joint has one. It is the `friction_limit` input of §4.4 driven by a solenoid-like coil.
- **Marine thrusters** (hydrodynamics.md lists "thrusters as rotors in water"): a rotor in water
  with the advance-ratio coefficients of the hydrodynamics design.
- **Conveyor belts** (a surface velocity on a fixed collider), common in warehouse and factory
  scenes. Engines differ in how they offer it: Jolt 5.6.0 has
  `ContactSettings::mRelativeLinearSurfaceVelocity` ("can be used to create a conveyor belt
  effect", `ContactListener.h`) and a `ConveyorBeltTest` sample; TODO(verify) MuJoCo's way, not
  found in this pass.
- **Tendon and cable drives** (RFC 0019's transmission extended to coupled joints).
- **A water or foam nozzle with a tank**, specified in RFC 0022 §8 (its schema `NozzleAPI_1`
  goes in this RFC's `schemas/actuator`): jet reaction on the robot, tank weight, pump on the bus.
- **The winch's electrical load** (RFC 0011's non-goal): the winch as a motor on the bus.

## Open questions (with a proposed answer)

1. *Hold grasped objects by force pairs (§3.1) or by temporary welds?* Force pairs. They behave
   like the real thing (pull-off, slip, peel), need no engine change, and work on plugin
   backends. A weld would need a runtime constraint API in both engines and the ABI.
2. *One `actuators` domain, or a domain per family?* One domain with a module per family, each
   running only when its prims exist. `--domain actuators=off` turns them all off.
3. *Porosity of target surfaces?* Add `sensmat:airPermeability` (optional, fallback 0:
   airtight) to `SensorMaterialAPI_1` (an addition), plus a `cardboard` preset value
   (TODO(verify) a published value: not searched in this pass).
4. *`elec:gearBackEfficiency`'s fallback?* −1, meaning "use `gearEfficiency`", so every 1.x file
   behaves as before. A sentinel is ugly, but a fallback must be a constant, and the old
   behaviour is not one.
5. *Servo pulse input or angle input as the primary command?* Both. `pulse_width` for
   flight-controller outputs and hobby boards, `goal_position` for ROS and ros2_control. The
   pulse is converted by the servo's own map, so a miscalibrated map shows up as it does on a
   real servo.
6. *Smart-servo overload shutdown (Dynamixel's hardware error status)?* Model the overload and
   overheating flags only (current over the limit for longer than a set time; winding
   temperature from the thermal node). The error-status bit layout is out of scope.
7. *The Dynamixel serial protocol itself (Protocol 2.0 over a virtual port)?* Not in this RFC.
   ros2_control and topics cover the use. A protocol emulator is "also worth adding" if users ask.
8. *Stepper `lost_steps`, a quantity no real driver reports?* Keep it as a ground-truth signal,
   documented as such. Stall detection as a driver reports it (back-EMF based, as some
   integrated drivers do) is a later addition (TODO(verify) which drivers publish their method:
   Trinamic's StallGuard is the first candidate; its product page timed out in this pass).
9. *Pneumatics and hydraulics in this RFC or their own?* Their own RFC before phase 3. This RFC
   only reserves the names `fluid_power`, `FluidSupplyAPI_1`, `ValveAPI_1` and `CylinderAPI_1`.
10. *Does the `ideal` servo substitute replace `JointControl`?* No. `JointControl` stays the
    motor-less "move this joint" tool. A joint with `ServoAPI_1` follows its servo instead and
    logs it once.
11. *Pre-approve the gated merges (schemas) of phase 1 once each step passes its tests, as for
    RFC 0011?* Yes, phase 1 only. Phases 2 and 3 come back to the owner.
12. *Plugin ABI?* No change. A plugin can already write `body/<b>/wrench/<source>`,
    `joint/<j>/torque_command` and `load/<n>/power_demand`, which is enough for a custom
    actuator. A plugin substitute for the `actuators` domain uses RFC 0008's mechanism.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Weld the object on contact (Gazebo's detachable joint, Isaac's surface gripper style) | holds through any load, so it shows nothing a suction cup does wrong; needs runtime constraints in each engine |
| Servos as `JointControl` with limits only | no deadband, no supply-voltage dependence, no current, no heat; that is the `ideal` substitute |
| Stepper as a position-controlled joint | cannot miss steps, which is the behaviour users need to test |
| An FMU per actuator (`elec:behaviorModel`) | available today for a user's own model; a built-in model is needed for samples, tests and users without Modelica |

## References checked for this draft

- The repository at origin/dev 1.4.0: `schemas/electrical/schema.usda` (motor types and
  control modes), `modules/electrical/network/src/Models.cpp` (gear efficiency in both
  directions), `server/include/lagrange/server/JointControl.hpp`,
  `bridges/ros2_control/src/LagrangeSystem.cpp` (no GPIO), `modules/sensors/src/kinds/GripperSensor.cpp`,
  `schemas/sensor_material/schema.usda` (`sensmat:relativePermeability`), rigid_body.md §3 and §8.
- Checked on 2026-11-17 against the sources below. The textbooks and ISO 6358-1 are not openly
  readable: their bibliographic records were checked, not their equations, which stay marked
  TODO(verify) and are read before the step that implements them.

## Sources

Read for the 2026-11-17 check:

- J. Schmalz GmbH, "Theoretical holding force of a suction cup" (vacuum knowledge, system design
  calculation example). https://www.schmalz.com/en/vacuum-knowledge/the-vacuum-system-and-its-components/system-design-calculation-example/theoretical-holding-force-of-a-suction-cup
- ROBOTIS e-Manual, XM430-W350 (specifications and control table).
  https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
- ROS rosdistro, `lyrical/distribution.yaml` (`dynamixel_hardware_interface` 1.5.1-3,
  `ros2_controllers` 6.10.1-1 with `gpio_controllers`).
  https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml
- `ros2_controllers` 6.10.1, `gpio_controllers/gpio_controllers_plugin.xml`.
  https://github.com/ros-controls/ros2_controllers/tree/6.10.1/gpio_controllers
- MuJoCo 3.12.0 `include/mujoco/mjmodel.h`; MuJoCo documentation, Simulation, "Model changes".
  https://mujoco.readthedocs.io/en/3.2.2/programming/simulation.html
- Jolt Physics v5.6.0: `HingeConstraint.h`, `SliderConstraint.h`, `ContactListener.h`,
  `Samples/Tests/General/ConveyorBeltTest.cpp`. https://github.com/jrouwe/JoltPhysics/tree/v5.6.0
- ArduPilot Copter documentation, "Grippers".
  https://ardupilot.org/copter/docs/common-gripper-landingpage.html
- PX4 user guide (main), "Gripper". https://docs.px4.io/main/en/peripherals/gripper.html
- McGraw-Hill, "Shigley's Mechanical Engineering Design, 11e: what's new".
  https://www.mheducation.com/unitas/highered/changes/budynas-mech-engineering-design-11e.pdf
- Crossref records for Furlani (2001), Hughes and Drury (2019), Acarnley (2002), Kenjo and
  Sugawara, Beater (2007), Jelali and Kroll (2003). https://api.crossref.org/works
