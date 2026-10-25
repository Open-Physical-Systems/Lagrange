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

## 8. Status (v0.6)

`lagrange_traffic` (`modules/traffic/`): the `builtin` backend at 10 ms and its `zero` substitute.
SUMO co-simulation is not built (the owner's v0.6 decision: no new external engines).

- **Agents.** At load, when the world has `RoadLaneAPI_1` lanes (from OpenDRIVE), the server adds
  `--traffic vehicles=N,pedestrians=M,seed=S` agents (default 8, 6, 1). They are placed apart and
  at least 15 m from the robots.
  - They are kinematic bodies (`World::Bodies::kinematic`) with a box or a capsule.
  - MuJoCo moves them as mocap bodies and Jolt as kinematic bodies, both by
    `body/<agent>/pose_command`.
  - Sensors see them, and the robot can touch and be pushed by them.
  - The counts are set when the server starts; a profile applied later keeps them.
- **Vehicles.**
  - They follow their lanes by the IDM (Treiber, Hennecke and Helbing 2000; a = 1.5 m/s²,
    b = 2 m/s², s0 = 2 m, T = 1.5 s) at the lane's speed limit, behind the agent or robot ahead.
  - They take a seeded random successor at a lane's end, and come back in at a free entry lane at
    the network's edge.
  - Into a junction they stop on red and yellow, and wait while a crossing or merging junction
    lane is taken or a vehicle nearer its line is about to take one (first come, first served).
  - No lane changes, no priority rules, no turn signals.
- **Pedestrians.** They walk their sidewalk at 1.34 m/s (Weidmann 1993), turn back at its ends,
  and step aside from each other and from the robots by a social force (Helbing and Molnár 1995,
  A = 2 m/s², B = 0.3 m). They do not cross roads yet.
- **Lights.** Fixed cycle per junction, illustrative: approaches facing each other share a group,
  and groups take turns at 20 s green, 3 s yellow and 2 s all red. They are not drawn yet. Their
  states are `traffic/lights`.
- **Outputs.** `traffic/agents` (8 numbers per agent), `traffic/agent_count`, `traffic/lights`.
- **Validation** (`test_traffic`, on MuJoCo with the city block):
  - the IDM's equilibrium gap;
  - the light cycle;
  - two minutes of traffic that flows, keeps its distance and keeps pedestrians on their
    sidewalks;
  - three minutes without an entry on red;
  - the same seed giving the same tracks;
  - the zero substitute.
- **Owed:**
  - the fundamental-diagram check for pedestrians;
  - scenario conformance (with the OpenSCENARIO runner);
  - SUMO, esmini.

### 8.1 OpenSCENARIO runner (v0.6)

`lagrange_openscenario` (`io/openscenario/`) is an own reader and runner for a subset of
OpenSCENARIO 1.x XML. esmini is not used (the owner's v0.6 decision).
`lagrange_server --scenario FILE.xosc [--scenario-ego NAME]`:
- runs the scenario;
- uses its `RoadNetwork/LogicFile` (OpenDRIVE) as the world when `--world` is not given;
- puts the robot at the ego entity's Init teleport (`Ego` by default);
- turns the builtin traffic and the robot's demo drive off.

**Read:**
- `ParameterDeclarations` (`$name` substitution).
- Vehicle and Pedestrian entities with their bounding boxes (no catalogs).
- `Init` with `TeleportAction` (`LanePosition`, `WorldPosition`) and `SpeedAction`.
- Stories, acts, maneuver groups, maneuvers and events.
- Private `SpeedAction` (absolute, or relative as a delta; step, linear, sinusoidal or cubic
  over a time, a distance or at a rate).
- `LaneChangeAction` (relative to an entity's lane, or an absolute lane id; over a time or a
  distance).
- Start and stop triggers of condition groups, with delays and edges:
  - `SimulationTimeCondition`, `ReachPositionCondition`;
  - `RelativeDistanceCondition` (longitudinal, lateral or Euclidean; freespace);
  - `SpeedCondition`, `TraveledDistanceCondition`.

Anything else is refused with its name.

**Entities.** Entities other than the ego are kinematic bodies (`scenario_<name>`) that the
runner moves along the road lanes, going straight on at a lane's end. The ego is the robot,
which its own stack drives; actions on the ego are not applied.

**Outputs.** `scenario/state` (1 once the stop trigger has held), `scenario/time` and
`scenario/entities`.

**Sample and checks.** The sample is `assets/scenarios/cut_in.xosc` on the city block.
`test_openscenario` checks it runs to its end with its lane change at 3.5 s (35 m at 10 m/s),
halfway at 5 s, in the ego's lane at 6.6 s, 8 m/s at 7.5 s on its linear slow-down, and ended
at 12 s. `ScenarioServerTest` runs it around the sample sedan.

**Owed:**
- catalogs;
- routing and trajectory actions;
- controllers;
- the ASAM example scenarios, and a comparison with esmini.
