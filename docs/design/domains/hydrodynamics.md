# Domain: hydrodynamics (marine)

## 1. Scope and release

Forces on surface and underwater vehicles. Marine robots are listed as "later" in CLAUDE.md §1,
so this domain is **post-1.0**. The contract and the `zero` substitute are defined now so that
adding it later does not change other domains. No implementation issues are planned before 1.0.

## 2. Models per fidelity level

| Level | Content |
|---|---|
| `fossen` | Fossen's six-degree-of-freedom manoeuvring model: rigid-body and added-mass inertia, Coriolis terms from both, linear and quadratic damping matrices, restoring forces from weight and buoyancy with separate centres, ocean current as a relative-velocity offset |
| `fossen_waves` | Adds first-order wave excitation from a wave spectrum (JONSWAP or Pierson–Moskowitz) through force response amplitude operators supplied as tables |
| `buoyancy_mesh` | For surface vehicles without coefficients: per-triangle hydrostatic pressure on the submerged hull mesh against a wave height field; simple drag |

Thrusters: thrust and torque from propeller coefficients as functions of advance ratio, the same
form as `rotor_aero` with water density; the two modules share the coefficient code.

Water density and gravity are scene parameters (fresh or sea water differ by about 2.5 %).

## 3. Signals

```cpp
// modules/hydrodynamics — declare()
for (auto& h : world.instancesOf("HydroAPI_1")) {          // schema defined when the domain is built
    auto b = bodyOf(h);
    d.input ({.name = fmt("body/{}/pose", b),  .type = SignalType::Pose,   .unit = units::m,       .feedback = true});
    d.input ({.name = fmt("body/{}/twist", b), .type = SignalType::Wrench, .unit = units::m_per_s, .feedback = true});
    d.output({.name = fmt("body/{}/hydro_wrench", b), .type = SignalType::Wrench, .unit = units::N, .frame = "world_local"});
    d.output({.name = fmt("body/{}/submerged_fraction", b), .type = SignalType::Scalar, .unit = units::one});
}
d.input({.name = "env/water/current",      .type = SignalType::Vec3,    .unit = units::m_per_s,  .optional = true});
d.input({.name = "env/water/density",      .type = SignalType::Scalar,  .unit = units::kg_per_m3});
d.input({.name = "env/water/surface_field", .type = SignalType::Field3D, .unit = units::m,       .optional = true});
```

Added mass makes the hydrodynamic force depend on acceleration, which an explicit exchange with
the rigid-body module handles badly. The planned approach is to pass the added-mass matrix to the
rigid-body backend as extra inertia where the backend supports it, and to treat only the
remaining terms as an external wrench. This is the same class of coupling problem as
architecture §8 and is noted there for when the domain is built.

## 4. Step size

Default 5 ms.

## 5. Substitutes and contract

Contract: `body/{b}/hydro_wrench`, `body/{b}/submerged_fraction`.

| Profile token | Kind | Behaviour |
|---|---|---|
| `zero` | constant | Zero wrench, `submerged_fraction = 0` |
| `replay` | replay | From MCAP |

## 6. Placement

CPU on all tiers. `buoyancy_mesh` with large hulls may use a compute kernel on Tier 1 and 2.

## 7. Validation (when built)

| Test | Reference |
|---|---|
| Hydrostatic equilibrium and metacentric restoring moment of a box | closed form |
| Terminal velocity under quadratic drag | closed form |
| Turning circle and zig-zag of a documented vessel or underwater vehicle | published model coefficients and trial results for an open reference vehicle (to be chosen from the literature at that time) |
| Wave response | response amplitude operators from the supplied tables reproduced in regular waves |

## 9. Water, as built

`WaterAPI_1`, `--water` or the run file's `water:` make a box of still water: `lagrange_hydrodynamics`'s
`Water`, not Fossen's model. Every moving, non-kinematic body with colliders gets buoyancy and quadratic
drag (`Cd`, illustrative) on cells sampled in its colliders, so a hull tilts and rights itself:
`body/<b>/wrench/water`, `body/<b>/submerged`. The frontal area for drag is taken as a box's, so it is
approximate for other shapes. `simulate: {water: false}` or `--domain hydrodynamics=off` turns the forces off.

Not modelled: added mass, waves, currents, a hull's waterplane beyond the cells, wet friction on what
drives through. Water has no collider, and a robot on the ground under it stays on the ground. It is
drawn as an opaque sheet (the viewer has no transparency), and cameras see that sheet. A lidar
sees its surface as a mirror: `water:reflectivity` straight back near normal, no return at a slant,
and no return from the bottom, as near-infrared is absorbed. Both lidar tiers see it.

Checked against the closed forms (`server/test/test_water.cpp`): a floating box's draft, an off-centre hull's, and a sinking box's terminal speed.
