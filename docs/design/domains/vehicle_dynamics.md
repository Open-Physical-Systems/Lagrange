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
