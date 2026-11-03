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

The water can move: `--water-current VX,VY[,VZ]` (m/s) is a steady current, and
`--water-waves AMPLITUDE,PERIOD[,TOWARDS_DEG]` one train of linear (Airy) waves, `ω² = g·k·tanh(k·d)`
over the water's depth, the surface at `η = a·cos(k·x − ω·t)` and the water under it in the orbits of
linear theory (Dean and Dalrymple, *Water Wave Mechanics for Engineers and Scientists*, 1991, ch. 3).
Cells count as wet under the moving surface, and the drag works on a cell's speed through the water.
A submerged body also gets Morison's inertia terms at its centre of mass: `ρ·V·a_w` from the water's
acceleration and `m_a·(a_w − a_b)` from its added mass, `m_a = ½·ρ·V` (a sphere's in potential flow,
Lamb, *Hydrodynamics*, §92) along every axis. The body's acceleration `a_b` is last step's, smoothed
with gain `1/(1 + m_a/m)`: an explicit added mass larger than the body's own would otherwise diverge,
and with this gain a steady push settles in one step on the exact `F/(m + m_a)`.

Not modelled: an added mass for other shapes than a sphere (a cube's is larger, about 0.7·ρ·V
TODO(verify)), added inertia in rotation and the Coriolis terms of Fossen's model, wave radiation
and diffraction (the waves pass through bodies), a hull's waterplane beyond the cells, wet friction
on what drives through. The viewer and the lidars still see a flat surface. Water has no collider, and a robot on the ground under it stays on the ground. It is
drawn as an opaque sheet (the viewer has no transparency), and cameras see that sheet. A lidar
sees its surface as a mirror: `water:reflectivity` straight back near normal, no return at a slant,
and no return from the bottom, as near-infrared is absorbed. Both lidar tiers see it.

Checked against the closed forms (`server/test/test_water.cpp`): a floating box's draft, an off-centre hull's, and a sinking box's terminal speed; a sphere half as dense as water released under it starts rising at `g(ρ_w·V − m)/(m + ½ρ_w·V)` = g/2 within 3% (without added mass it would be g), on MuJoCo and Jolt; a floating box in a 0.5 m/s current drifts with it within 0.02 m/s; a 0.4 m float on 0.1 m, 4 s waves (25 m long) heaves 0.1 m within 1 cm.

### 9.1 A vehicle's own coefficients: Fossen's model (2026-11-03)

A body named in `--hydro-coefficients FILE.yaml` (`ServerOptions::hydroCoefficients`; module
config `fossen:<body>`) takes Fossen's model (Handbook of Marine Craft Hydrodynamics and Motion
Control, 2011, ch. 6) instead of the cells' drag and the sphere's added mass, in its body frame on
its velocity through the water ν_r = (u, v, w, p, q, r), scaled by the share of it under water:

    τ = −M_A·ν̇_r − C_A(ν_r)·ν_r − (D + D_n(|ν_r|))·ν_r,   −C_A·ν = (ω × A_t·u, u × A_t·u + ω × A_r·ω)

with a diagonal added mass M_A = diag(A_t, A_r), linear damping D and quadratic damping D_n, all
measured for the vehicle. ν̇_r is smoothed per axis with gain 1/(1 + m_a/m), as the sphere's is.
Buoyancy stays the cells', and the water's acceleration still pushes the displaced volume.

The sample is the BlueROV2 Heavy (`assets/vehicles/bluerov2_heavy.hydro.yaml`, from von Benzon
et al., J. Mar. Sci. Eng. 10, 1898, 2022, Table A1, CC BY 4.0: damping identified from open-loop
runs in a pool and validated on repeat runs to about 10 %, added mass by Eidsvik's method).
`TheBlueRov2ReachesItsTopSpeedsWithFossensModel` pushes it by its full thrust along each axis
(gravity standing in for the thrusters: the vehicle weighted that much beyond its buoyancy and
turned so the axis points down) in MuJoCo and Jolt:

| Axis | Thrust | Top speed, their Table 4 | Lagrange | Start, F/(m + m_a) | Lagrange |
|---|---|---|---|---|---|
| Surge | 85 N | 0.72 m/s | 0.729 | 2.99 m/s² | 3.01 |
| Sway | 85 N | 0.63 m/s | 0.626 | 2.91 m/s² | 2.95 |
| Heave | 120 N | 0.71 m/s | 0.713 to 0.720 | 2.71 m/s² | 2.78 |

What this shows is that the model is implemented as published: Table 4's speeds are their
model's balance of damping and thrust (0.729, 0.626, 0.713 from Table A1), and their model is
within about 10 % of the pool (Fig. 6), so ours is too to the same extent, for this vehicle. The
coefficients belong on the vehicle in USD (a `HydrodynamicsAPI` schema, owed: a schema addition
for the owner to approve); the file stands in until then. Not yet: off-diagonal added mass and
damping, wave radiation and diffraction, thrusters as rotors in water.
