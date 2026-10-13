# Domain: traffic

## 1. Scope and release

Other road users around the robot under test: vehicles, pedestrians, cyclists and traffic lights.
The domain produces agent poses and signal states. It does not simulate their physics in detail.
Release: v0.6. A small built-in pedestrian and AMR-traffic model for warehouses may arrive with
v0.2's Open-RMF demo if time allows; it is not a v0.2 acceptance item.

## 2. Models per fidelity level

| `backend` | Content |
|---|---|
| `builtin` | Scripted agents: waypoint followers with speed profiles; pedestrians with a social-force model; traffic lights on fixed cycles. Enough for warehouse and simple road scenes |
| `sumo` | Co-simulation with Eclipse SUMO as a separate process through TraCI (`sumo` 1.25.0 is in Resolute). SUMO owns route choice, car-following and lane changing; this module mirrors agent states into the world each traffic step and reports the ego vehicle back to SUMO so that agents react to it |
| OpenSCENARIO actions | The scenario runner (`io/openscenario/`) writes commands to this module (spawn, speed, lane change, trajectory). Candidate engine: esmini (MPL-2.0), used as a library or as a reference to test against (decision in v0.6 planning) |

Agent bodies: near the robot (inside the active region) agents are kinematic rigid bodies with
collision shapes, so sensors see them and contacts are detected. Far agents are positions only
(see `docs/design/huge-worlds.md`, simplified distant agents).

SUMO is EPL-2.0 and runs as its own process; nothing from SUMO is linked into `lagrange` binaries.

## 3. Signals

```cpp
// modules/traffic — declare()
d.input ({.name = "traffic/command", .type = SignalType::Bytes, .unit = units::one, .optional = true});   // scenario runner
for (auto& ego : egoVehicles(world)) {
    d.input({.name = fmt("body/{}/pose", ego),  .type = SignalType::Pose,   .unit = units::m});
    d.input({.name = fmt("body/{}/twist", ego), .type = SignalType::Wrench, .unit = units::m_per_s});
}
d.output({.name = "traffic/agents",        .type = SignalType::Bytes, .unit = units::one});   // packed: id, class, pose (f64), velocity, bbox, blinkers
d.output({.name = "traffic/agent_count",   .type = SignalType::Int,   .unit = units::one});
d.output({.name = "traffic/lights",        .type = SignalType::Bytes, .unit = units::one});   // packed: light prim id, state enum
d.output({.name = "traffic/sim_time_skew", .type = SignalType::Scalar, .unit = units::s});    // SUMO time − sim time
```

Agent states are variable-length, so they travel as a packed `Bytes` signal with a versioned
layout documented in `modules/traffic/AgentRecord.hpp`. The core applies them to kinematic
bodies through a world-update service; the rigid-body module then reports their poses like any
other body. Positions inside the record are `double` in the local frame.

## 4. Step size

Default 100 ms (SUMO's common step). Poses of near agents are interpolated to the physics rate
by the kinematic body update.

## 5. Substitutes and contract

Contract: `traffic/agents`, `traffic/agent_count`, `traffic/lights`.

| Profile token | Kind | Behaviour |
|---|---|---|
| `zero` | constant | No agents; all lights in a fixed state taken from the scene (default: green for the ego lane is **not** assumed; each light prim must author its static state) |
| `replay` | replay | Agent tracks and light states from MCAP, for re-running a recorded scenario exactly |

## 6. Placement

CPU on all tiers. Rendering thousands of agents uses instancing in the shared renderer.

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| TraCI synchronisation | SUMO's own positions vs mirrored positions each step | exact after coordinate conversion; skew signal stays below one traffic step |
| OpenDRIVE ↔ SUMO network alignment | lane centre lines from both representations of the same road | < 5 cm |
| Social-force pedestrians | fundamental diagram (speed vs density) from the pedestrian-dynamics literature | within the published band |
| Determinism | same seed, `builtin` backend | identical tracks. With SUMO, determinism depends on SUMO's seed and version; documented, not guaranteed |
| Scenario conformance | a subset of the ASAM OpenSCENARIO example scenarios | each runs to its end condition; ego-relative distances at key events compared with esmini |
