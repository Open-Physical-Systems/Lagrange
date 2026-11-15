# Domain: vehicle dynamics

## 1. Scope and release

Tyres, suspension, powertrain, brakes and steering for wheeled road vehicles. The chassis and
wheels remain rigid bodies in the rigid-body backend; this module computes the forces that a
generic contact solver models poorly (tyre slip forces) and the drivetrain that turns pedal
commands into wheel torques. Release: v0.6.

PhysX ships a vehicle SDK. The plan is to implement tyre and drivetrain models in this module on
top of wheel-ground ray or sweep queries, so that the same model runs on every backend. Using the
PhysX vehicle SDK directly is the fallback if this costs too much (open question Q10).

## 2. Models per fidelity level

| Part | Level | Content |
|---|---|---|
| Tyre | `linear` | Longitudinal and lateral force proportional to slip ratio and slip angle, saturated by a friction ellipse at `μ·Fz` |
| Tyre | `pacejka` (default) | Magic Formula `F = D·sin(C·atan(B·x − E·(B·x − atan(B·x))))` for pure longitudinal and lateral slip with load dependence, combined slip by weighting functions, relaxation length for transient response. Parameter sets in the `.tir` property-file convention are imported |
| Suspension | `spring_damper` | Per wheel: vertical spring, damper, bump stops, anti-roll bar coupling across an axle; wheel travel along a fixed axis |
| Suspension | `kinematic` (later) | Camber and toe change with travel from lookup tables |
| Powertrain | `electric` | Motor torque from the electrical domain (or its substitute) through a fixed reduction and an open differential |
| Powertrain | `ice_map` | Engine torque map (speed × throttle), torque converter or clutch, geared transmission with a shift schedule, differential |
| Brakes | `torque` | Brake torque proportional to pedal pressure per axle with a bias; ABS as a slip-limiting controller (optional) |
| Steering | `ackermann` | Road-wheel angles from the steering-wheel angle with Ackermann geometry, rate limit and a first-order lag |

Friction `μ` at each contact patch = surface material friction × `env/friction_scale` ×
the material's wet scale.

## 3. Signals

```cpp
// modules/vehicle_dynamics — declare()
for (auto& veh : vehicles(world)) {                     // VehicleAPI_1 (schema added with v0.6)
    auto v = name(veh);
    d.input ({.name = fmt("vehicle/{}/throttle", v),       .type = SignalType::Scalar, .unit = units::one});
    d.input ({.name = fmt("vehicle/{}/brake", v),          .type = SignalType::Scalar, .unit = units::one});
    d.input ({.name = fmt("vehicle/{}/steering_angle", v), .type = SignalType::Scalar, .unit = units::rad});
    d.input ({.name = fmt("vehicle/{}/gear", v),           .type = SignalType::Int,    .unit = units::one, .optional = true});
    d.input ({.name = fmt("body/{}/pose", chassisOf(veh)),  .type = SignalType::Pose,   .unit = units::m,       .feedback = true});
    d.input ({.name = fmt("body/{}/twist", chassisOf(veh)), .type = SignalType::Wrench, .unit = units::m_per_s, .feedback = true});
    d.output({.name = fmt("vehicle/{}/wrench", v),         .type = SignalType::Wrench, .unit = units::N, .frame = "world_local"});
    d.output({.name = fmt("vehicle/{}/speed", v),          .type = SignalType::Scalar, .unit = units::m_per_s});
    d.output({.name = fmt("vehicle/{}/engine_speed", v),   .type = SignalType::Scalar, .unit = units::rad_per_s});
    for (auto& w : wheelsOf(veh)) {
        auto j = jointOf(w);
        d.input ({.name = fmt("joint/{}/velocity", j),      .type = SignalType::Scalar, .unit = units::rad_per_s, .feedback = true});
        d.output({.name = fmt("joint/{}/load_torque", j),   .type = SignalType::Scalar, .unit = units::N_m});   // tyre reaction + brake
        d.output({.name = fmt("wheel/{}/slip_ratio", j),    .type = SignalType::Scalar, .unit = units::one});
        d.output({.name = fmt("wheel/{}/slip_angle", j),    .type = SignalType::Scalar, .unit = units::rad});
        d.output({.name = fmt("wheel/{}/normal_force", j),  .type = SignalType::Scalar, .unit = units::N});
        d.output({.name = fmt("wheel/{}/steer_angle", j),   .type = SignalType::Scalar, .unit = units::rad});
    }
}
d.input({.name = "env/friction_scale", .type = SignalType::Scalar, .unit = units::one, .optional = true, .defaultValue = 1.0});
```

For an electric powertrain, drive torque comes from the electrical domain as
`joint/{j}/torque` like any other motor. For `ice_map`, this module writes
`joint/{j}/torque_command` and the electrical substitute passes it through.

## 4. Step size

Default 1 ms, equal to the physics step. Tyre relaxation dynamics and ABS need 1–2 ms.

## 5. Substitutes and contract

Contract: `vehicle/{v}/wrench`, `joint/{j}/load_torque`, `vehicle/{v}/speed`.

| Profile token | Kind | Behaviour |
|---|---|---|
| `ideal` | ideal | Kinematic bicycle model: the chassis follows the commanded speed and steering exactly (written as a wrench that enforces it, or as a kinematic body override when rigid body is also substituted). For planner tests |
| `replay` | replay | From MCAP |

When the stage contains no vehicle prims the domain is absent from the wiring, and the profile
may omit it (it is not in the schema's required list).

## 6. Placement

CPU on all tiers. Wheel-ground queries use the rigid-body backend's scene query interface, which
is exposed to modules through a core service (`core/world/SceneQuery`) so that this module does
not link the backend (invariant 1).

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Tyre force curves | curves regenerated from a published Magic Formula parameter set (for example the parameter tables in Pacejka's *Tire and Vehicle Dynamics*; cite edition and table) | 0.5 % |
| Steady-state cornering | understeer gradient from the linear bicycle model | 3 % in the linear range |
| Standard manoeuvres | ISO 4138 steady-state circle, ISO 3888-2 obstacle avoidance, ISO 7401 step steer, run as scripted scenarios | qualitative agreement plus metrics compared with an open reference vehicle model (candidate: published results for a documented reference vehicle; to be chosen) |
| Braking distance, dry and wet | `v² / (2·μ·g)` with ABS off and tyre curves on | 5 % |
| Energy | drivetrain power balance | 1 % |
| Sim-vs-real | a public vehicle-dynamics dataset with IMU, wheel speeds and steering (to be selected; license check) | reported per manoeuvre |

## 8. Status (v0.6)

`lagrange_vehicle_dynamics` (`modules/vehicle_dynamics/`): `VehicleDynamics` at 1 ms, a raycast
vehicle on top of any rigid-body backend, and its `ideal` substitute (kinematic bicycle). Q10
is settled for now: the model is our own, with no PhysX vehicle SDK.

- **Wheels and suspension.** Each `WheelAPI_1` wheel is a spin joint under the chassis,
  possibly through a steering knuckle. A ray down the chassis's up axis from its suspension
  mount finds the ground through `services::SceneQuery`, and the spring, damper and bump stop
  act along it.
  - The model's wheel positions are the ride height: each spring is preloaded with its static
    share of the weight, by the centre of mass between the axles.
  - Each ray returns the surface's outward normal where it meets the ground
    (`services::RayHit::normal`, from `sensors::ShapeCaster::ray`), and the tyre works in the
    contact frame: z along that normal, x where the wheel plane meets the road plane, y = z × x
    (the TYDEX W-axis system; ISO 8855 with z up; Pacejka, *Tire and Vehicle Dynamics*,
    TODO(verify) the section). The normal load acts along the normal and the MF-Tyre
    longitudinal and lateral forces lie in the road plane, so slopes and banks are felt as they
    are. A hit whose normal is more than about 84° from the chassis's up (a wall) is taken as
    level ground, as is a query that gives no normal. The rays stand on whatever collides, so a
    car drives on raised and banked OpenDRIVE roads, whose lanes become convex slabs
    (formats.md §3.11). On the sample `assets/worlds/banked_ring` (a 5 m ring 1.5 m up, 5° of
    bank) the sedan's demo circle keeps its base within 2.2 cm of its ride height above the
    banked surface and its roll within 0.018 rad of the bank over 14 s at 3 m/s (`test_server`
    BankedRing); the normal loads the tyres report are within 0.8 % of the weight of
    m·(g cos β + a sin β) on average there (1.3 % with the forces in the level plane).
  - Parked with its brakes on a 10° slope, the test saloon stands still and its tyres carry
    m·g·sin θ along the road and m·g·cos θ across it to 1e-7 of each (`test_vehicle_dynamics`
    ParkedOnASlope; with the forces in the level plane the loads read m·g / cos θ, 3 % over,
    and it crept at 13 cm/s).
  - Anti-roll bars move load across an axle in proportion to the travel difference.
- **Tyres** (`Tyre.hpp`).
  - `pacejka` (default): MF-Tyre 5.2 pure slip with load dependence and its combined-slip
    weighting, from `.tir` files. Camber, shifts, conicity, ply steer, the aligning moment and
    turn slip are not applied; the curves are made odd so the forces oppose the slip.
  - `linear`: slip stiffnesses on a friction ellipse.
  - Slip comes from first-order relaxation states integrated exactly over the step, with
    damping blended in below 3 m/s.
  - μ = the tyre's × the ground shape's friction × `env/friction_scale`.
- **Brakes.** Torque per wheel by the axle bias, as stick-slip friction (Karnopp): the torque
  that stops the wheel within the step against the tyre's, up to what the brake gives, so a
  held brake holds the car on a slope and never reverses the wheel; optional slip-limiting ABS.
- **Steering.** Ackermann road-wheel angles from the bicycle angle (rate limit, first-order
  lag). The steering joints are turned by a PD at 15 Hz; they are visual, since the tyre forces
  use the computed angles.
- **Powertrain.**
  - `electric`: throttle × `vehicle:maxDriveTorque`, split equally (an open differential), as
    `torque_command` to the driven wheels' motors or the electrical substitute.
  - `ice_map`: full-throttle torque map, a clutch that slips below idle, an automatic gearbox
    with shift speeds, final drive and efficiency; reverse is first gear backwards.
- **Validation** (`test_vehicle_dynamics`, on MuJoCo):
  - ride height and static loads;
  - braking distance from 20 m/s within 5 % of v²/(2μg) (3.2 % long);
  - steady-state yaw rate within 3 % of the linear bicycle model with its understeer gradient
    (1.4 %);
  - gear shifting;
  - the ideal substitute.
  - The Magic Formula's peak, slope and asymptote are exact (`test_tyre`).
- **Owed:**
  - a comparison with a published MF parameter set (no openly licensed `.tir` was found; the
    sample `assets/tyres/generic_205_55R16.tir` is illustrative);
  - the ISO manoeuvres as scenarios;
  - the kinematic suspension.
