# RFC 0011: Tethers and cables

| | |
|---|---|
| Status | Accepted (2026-11-10) by the owner, with the proposed answer to each question below; gated merges of its steps pre-approved |
| Created | 2026-11-10 |
| Affects | `schemas/` (one new library), `modules/tether/` (new), `io/stage/`, `server/`, `profiles/schema.json` (addition), `proto/` (one enum value), `viewer/` (drawing), `docs/` |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0003 (protocol), RFC 0005 (profiles), RFC 0007 (the 1.x freeze) |

## Summary

A cable between a fixed anchor and a body, simulated as a chain of point masses: it sags, swings,
goes taut, drags in the wind, lies on the ground, catches on scenery and stays caught, wraps and
tangles on itself, and hits and pulls the body it is tied to. The first use is a power tether from
a ground station to a drone; the same model fits a crane line, a towed load or a cable to a robot.
A new domain `tether`, a new schema `TetherAPI_1`, and one new value in the protocol's existing
`Overlay` message so the viewer draws each cable. Every change is an addition under RFC 0007.

## Motivation

Tethered drones (power over the tether, an anchor that keeps them up for hours) are common in
inspection and surveillance, and the tether is what goes wrong: it snags on a bridge deck or a
tree, its weight and drag pull the drone off its spot, it wraps a rotor arm, it piles up and
tangles when the winch pays out faster than the drone climbs. Lagrange cannot show any of this:
there is no cable, and its bodies form trees, so a body cannot be held at both ends.

## Non-goals

- Twist and bending stiffness beyond a light bending term: a power tether is limp. Rods and
  hoses that hold their shape are a later RFC.
- Cable-to-cable contact between two tethers (each tether collides with itself, scenery and
  bodies).
- Breaking. The tension is a signal; a test or a plugin can decide what is too much.
- Downwash on the cable in the first version (§6).
- Winch electrical load and conductor heating (later, through the electrical and thermal
  domains, from `tether:resistancePerLength`).

## Design

### 1. Built in, not a plugin

A plugin could do most of it (it can write `body/<b>/wrench/<source>` and query the scene), but
not draw it, and a tether is a physical object a user expects in every run, on both engines, in
recordings and in the Python API. So it is a domain like `hydrodynamics`, with a zero substitute
(`--domain tether=off`, profile `domains.tether`).

### 2. The data: `TetherAPI_1`

Applied to a prim whose position is the anchor (the winch's outlet): on the world, on a fixed
body, or on a moving body (a tether between two robots, or a truck with a winch). New library
`schemas/tether`, namespace `tether:`.

| Attribute | Type | Unit | Default | Meaning |
|---|---|---|---|---|
| `tether:anchorPoint` | double3 | m | 0, 0, 0 | the anchor in the prim's frame (an overlay's new prims have no pose) |
| `tether:attach` | rel | | required | the body (link) the far end is tied to |
| `tether:attachPoint` | double3 | m | 0, 0, 0 | where on that body, in its frame |
| `tether:length` | double | m | required | length paid out at the start |
| `tether:diameter` | double | m | 0.005 | for contact, drag and drawing |
| `tether:linearDensity` | double | kg/m | 0.02 | mass per metre |
| `tether:axialStiffness` | double | N | 2e4 | EA: tension per unit strain |
| `tether:dampingRatio` | double | 1 | 0.05 | internal (axial) damping |
| `tether:bendStiffness` | double | N·m² | 0 | EI; small values keep a limp cable from kinking |
| `tether:friction` | double | 1 | 0.4 | Coulomb, on scenery, bodies and itself |
| `tether:dragCoeff` | double | 1 | 1.2 | normal drag of a cylinder; tangential is 1/50 of it |
| `tether:segmentLength` | double | m | 0.2 | target spacing of the point masses |
| `tether:winch` | token | | `fixed` | `fixed`, `tension` (keeps `winchTension` at the anchor) or `free` (pays out freely) |
| `tether:winchTension` | double | N | 5 | `tension` mode's set point |
| `tether:winchSpeed` | double | m/s | 1 | most the winch pays out or reels in |
| `tether:maxLength` | double | m | `length` | the drum's end: beyond it the winch holds |
| `tether:resistancePerLength` | double | ohm/m | 0 | both conductors, round trip (for later use, §Non-goals) |

Authored in USD, in an overlay YAML (formats.md §9, no new code: the overlay applies any
registered schema), or created by the server from a run file's `tethers:` list (§8).

### 3. The model

A chain of point masses at `segmentLength` spacing (the last segment takes the remainder), the
first held at the anchor, the last at the attach point. Solved by extended position-based
dynamics with small steps (Macklin et al. 2019, "Small Steps in Physics Simulation"): every 1 ms
the step is split into substeps (default 20: at 10 a taut string rang 3.5 % flat, at 20 within
0.3 %), each a prediction under gravity and air drag, one
Gauss-Seidel pass of the length constraints with compliance `segment / EA` (forward, then back),
the bend constraints, the contacts, and the velocity update. It is unconditionally stable for a
stiff tether at 1 ms (a mass-spring chain would need about 20 µs), and deterministic: fixed
order, no threads, no randomness (invariant 7).

The tension of each segment is its constraint's multiplier, `λ / h²`. A prototype of a 12 m
cable hung between points 10 m apart settled within 0.05 % of the catenary's sag and 0.6 % of its
end tension (§Validation repeats this as a test).

The winch changes the first segment's rest length, splitting it when it passes 1.5 segments and
merging it below 0.5, so the masses stay evenly spaced. In `tension` mode it pays out when the
anchor's tension is over the set point and reels in when under, at most `winchSpeed`.

### 4. Contact

Every point mass is a sphere of the cable's radius; every segment is a capsule for its own
contacts.

- **Scenery and bodies.** The module places every collider at its body's pose this step
  (`body/<b>/pose`, declared as feedback inputs so that physics runs first) and keeps those near
  the cable each step. It needs the closest point on a shape and the normal there, which
  `services::SceneQuery` does not give (it has ray and overlap only). As built that is
  `tether::surfacePoint(shape, point)` in the tether module (not in `sensors`, which would bring
  Embree along), for every collider shape: plane, sphere, capsule, cylinder, box, convex hull (its
  farthest face plane) and height field (along its slope). A penetrating
  mass is moved out along the normal. Friction removes the sliding move up to `μ` times the
  penetration (static, so a cable stays caught on an edge until pulled harder than friction
  allows).
- **Itself.** Segment against segment (closest points of two capsules), segments one or two
  apart excluded. The pairs that could touch within a step are found once per step through a
  grid of midpoints, with a margin for how far they can move, and resolved every substep. That
  is what lets it wrap and knot. Turns of the cable do not yet rub on each other (no friction).
- **The body it is tied to and every other moving body.** A push on the cable is a push back on
  the body: the move times the mass over `h²`, applied at the contact point. Summed over the
  substeps and written as `body/<b>/wrench/tether`, as the water and rain push links.

### 5. The pull on the body

The last segment's tension along it, applied at the attach point, plus the contact pushes of §4,
is `body/<attach>/wrench/tether` (N, about the centre of mass, world frame). Physics and the
cable exchange once per 1 ms step (the cable sees the body's pose of this step, the body the
cable's force of the last), as the electrical domain couples to physics.

### 6. Air

Each segment's drag from the air velocity at its middle less its own: normal `½ ρ Cd d L |v_n| v_n`
and tangential `Cd/50`. In the first version the air is the site's wind (`env/wind/velocity`, as a
feedback input, ground-truth mean plus gusts) and the air density (`rotor_aero.air_density`, or
`env/air_density` with the weather). Downwash under the drone's rotors needs the aerodynamics
domain to offer air velocity at a point as a service, which it does not yet (the server queries
it directly for the viewer); that is the second step of the plan.

### 7. Signals

Per tether `<n>` (the prim's name):

| Signal | Type | Unit | |
|---|---|---|---|
| `tether/<n>/tension_attach` | Scalar | N | at the body |
| `tether/<n>/tension_anchor` | Scalar | N | at the winch |
| `tether/<n>/length` | Scalar | m | paid out |
| `tether/<n>/winch_speed` | Scalar | m/s | positive paying out |
| `tether/<n>/contacts` | Int | | point masses touching anything this step |
| `tether/<n>/caught` | Bool | | held by static friction somewhere |
| `tether/<n>/nodes` | ScalarArray (3 × most nodes) | m | positions, world frame; unused entries NaN |
| `body/<b>/wrench/tether` | Wrench | N | for the attached body and every body it touched |

`nodes` is sized at load for `maxLength`. Over 1024 components it is left out of recordings, as
other large arrays are; `--record` then keeps the tensions and length, and replay draws no cable.

### 8. Starting one without authoring USD

`lagrange_server --tether ANCHOR_X,ANCHOR_Y,ANCHOR_Z,LINK[,LENGTH]` and a run file's
`tethers: [{anchor: [x, y, z], attach: link, length: m, ...}]` create the prim, the defaults of §2
filling the rest. The drone workspace's launch file takes `tether:=true`.

### 9. Drawing

The protocol's `SceneDelta` already carries `repeated Overlay overlays`, which no server sends
yet. One value is added to `Overlay.Kind`:

```proto
KIND_CABLE = 5; // data: radius (1 f64), then count points (3 f64 each), world-local
```

The server adds one per tether to every delta from `tether/<n>/nodes`. The viewer draws it in the
viewport as scene geometry, always (it is an object, not a debugging aid): a tube of the cable's
diameter, at least a pixel wide, in a dark rubber colour, shaded like other geometry, through the
existing line and mesh path of `SceneRenderer`. No panel, menu or token changes; a viewer that
does not know the kind ignores it (proto3 enum), so an older viewer still connects.

### 10. Substitute

`zero`: no cable, no force, no signals but the tensions at 0 (so a profile that plots them still
wires). Constant kind, as `humans` and `hydrodynamics`.

## Validation

| Check | Against | Test |
|---|---|---|
| Hanging shape and end tension | catenary through two fixed points | sag within 0.5 %, tension within 2 % (built: 0.03 %, 0.5 %) |
| Ringing (in place of a hanging chain, whose free end this model does not have) | a taut string's fundamental, `sqrt(T/μ) / 2L` | within 3 % (built: 0.3 %) |
| Friction over a fixed cylinder | capstan equation `T₁/T₂ = e^{μθ}` | within 5 % at θ = π, μ = 0.3 |
| Caught on an edge | a cable dragged over a box edge stops until the pull exceeds friction | qualitative, in a test |
| Tangling | a cable dropped in a pile keeps every segment apart by its diameter | no interpenetration |
| Hits the drone | a cable swung into a hovering quad pushes it, equal and opposite | momentum balance within 1 % |
| Determinism | the same run twice | identical state hash |
| Tethered quad | the sample quad with 30 m of tether hovers; the tension equals the hanging weight's vertical share | within 2 % |

## Compatibility under RFC 0007

| Surface | Change | Allowed because |
|---|---|---|
| Schemas | new `TetherAPI_1` library | addition |
| Profile schema | new `domains.tether` key | addition |
| Protocol | new `Overlay.Kind` value | addition; old viewers ignore unknown enum values |
| Plugin ABI | none | |
| Recordings | new signals | addition |

## Implementation plan

One PR each, in order:

1. This RFC.
2. `TetherAPI_1` (schema, stage namespace, schema tests, CHANGELOG, formats.md).
3. `modules/tether`: the solver and its contacts (`tether::surfacePoint`), tests of §Validation
   that need no server.
4. The domain in the server: module, signals, wrenches, substitute, profile key, `--tether`, run
   file, the tethered-quad test, docs (`docs/physics/tether.md`, a guide section, CLAUDE.md).
5. Drawing: `KIND_CABLE`, the server's overlays, the viewer's tube, a headless screenshot test.
6. Downwash on the cable: the aerodynamics domain's air at a point as a service.
7. The drone workspace (outside this repository): small_drone's tether under the bridge deck.

## Alternatives considered

| Alternative | Why not |
|---|---|
| A chain of rigid bodies in the physics engine | needs a body held at both ends (a loop the world's tree cannot hold; MuJoCo equalities and Jolt constraints differ), hundreds of engine bodies, and the cable's contacts then depend on the engine |
| A plugin in the drone workspace | cannot draw it; not in recordings or Python by default; every tether user would rebuild it |
| Mass-spring chain, explicit | unstable at 1 ms for a stiff tether without µs steps |
| Kinematic capsule bodies for drawing (as humans) | hundreds of engine bodies only to draw; the `Overlay` message exists for this |

## Questions for the owner

1. Built in as a domain (§1)? *Proposed: yes.*
2. The schema of §2, including the winch modes? *Proposed: as written.*
3. `KIND_CABLE` in the existing `Overlay` (§9) rather than a new message? *Proposed: yes.*
4. Downwash on the cable as a second step (§6), not in the first version? *Proposed: yes.*
5. Pre-approve the gated merges (schema and protocol) of the steps above once each passes its
   tests, as for RFC 0009? *Proposed: yes.*

## Resolutions on acceptance (2026-11-10)

The owner accepted the design and asked for the physics in Lagrange and the drawing in its
viewer, ready to use. Every question takes its proposed answer. The first user's tether, for the
drone workspace (step 7): 20 m, 4 mm diameter, 23.5 g/m, from a ground station with a dock on top
of a bridge deck; its stiffness, friction and winch are the defaults of §2 until measured.
