# The ground under wheels, beams and contacts

What a surface's material (`SurfaceMaterial`, authored by USD `sensmat:preset` or URDF
`<gazebo reference=L><sensor_material preset=.../></gazebo>`) does to what touches it. The presets
`grass`, `wet_grass` and `tall_grass` live in `core/world/src/World.cpp`.

## Wheels on grass

`lagrange_vehicle_dynamics` (raycast suspension, MF-Tyre or linear tyres) reads, for the ground
shape each wheel's ray hits:

- **Friction.** The tyre's friction is scaled by the collider's friction, the surface's
  `frictionScale` and the weather's `env/friction_scale`, all three multiplied
  (`mu = friction · frictionScale · env/friction_scale`; the scale is folded into the per-shape
  friction by the server's `groundQuery`). Braking and traction are both bounded by it.
- **Rolling resistance.** `F = Crr · Fz` against the rolling, with `Crr` the surface's
  `rollingResistance` (on top of the tyre's own, which the tyre models here leave out) and `Fz`
  the wheel's load. It is applied as a moment `Crr · Fz · r` on the wheel through the same
  stick-slip (Karnopp) clamp as the brake: it slows the wheel to rest within a step at most and
  never turns it backwards, so a parked car does not creep or chatter on level grass, and the
  tyre passes it on to the chassis as a longitudinal force. Signal
  `wheel/<j>/rolling_resistance` (N), declared only when some ground surface has `Crr > 0`
  (so that runs without one keep their bus layout and their saved states).

Closed forms checked in `modules/vehicle_dynamics/test/test_vehicle_dynamics.cpp`:
coasting on level grass decelerates at `g · Crr` (default ground: almost not at all), locked-wheel
braking on `frictionScale = 0.5` stops in `v² / (2 · 0.5 · μ · g)` and the run-up's acceleration
stays under `0.5 · μ · g`, and a car parked on level grass holds still.

### Rain on grass

`grass` is **dry** grass. Rain wets every surface through the weather's `env/friction_scale`
(`frictionScale(wetness, ...)` in `modules/weather/atmosphere/src/Weather.cpp`), which the wheels
already multiply in, so it is not counted again in the preset. `wet_grass` is for ground that is
wet whatever the weather (dew, irrigated turf) and should not be combined with rain, or the
wetness counts twice.

TODO(verify): the weather's wet factor is one number for every surface, but a tyre loses more of
its grip on wet grass (water films on the leaves) than on wet asphalt, whose texture drains it.
If a published dry/wet tyre-on-grass friction ratio supports it, the surface material could carry
its own wet factor and the vehicle module would blend it by `env/wetness`-like state; that needs
the wetness on the bus (today only `env/friction_scale` is), so it is left for now.

### Numbers and sources

| Preset | frictionScale | rollingResistance | Source |
|---|---|---|---|
| grass | 0.9 | 0.08 | Crr: within J. Y. Wong, *Theory of Ground Vehicles*, Table 1.1 (after Taborek): car tyres, unpaved road 0.05, field 0.1–0.35. TODO(verify) the table and a turf measurement (Holloway, Wilson and Drach, "Examination of ATV tire forces generated on clay, grass and sand surfaces", SAE, 1989). frictionScale TODO(verify) |
| wet_grass | 0.6 | 0.1 | TODO(verify) both |
| tall_grass | 0.9 | 0.12 | TODO(verify) both: the standing sward adds bulldozing drag |

## Engine contacts on grass

Robots whose wheels are bodies in the physics engine (the sample diff-drive AMR, arms on
casters, anything without `VehicleAPI_1`) feel a grass surface through two terms of its
`SurfaceMaterial` (`sensmat:preset = "grass"`, URDF `<sensor_material preset="grass"/>`). Both
engines read them through `lagrange/world/SurfaceContact.hpp`, so MuJoCo and Jolt agree.

**Sliding friction.** When the engine model is built, each collider's Coulomb coefficient is
its own times its surface's `frictionScale` (`colliderFrictionOnSurface`). Both engines combine
a pair by the larger of the two coefficients (MuJoCo's rule for geom friction; the Jolt backend's
`SetCombineFriction`), so a grass ground lowers the grip of a wheel only when the wheel's own
coefficient is not higher than the ground's scaled one. The weather's `env/friction_scale` still
multiplies on top at run time.

**Rolling resistance.** A coefficient of rolling resistance Crr is the resisting force over the
load (Wong, *Theory of Ground Vehicles*, 4th ed., ch. 1 "Rolling resistance"; TODO(verify) the section number). After each MuJoCo step, for every
contact with a surface of `rollingResistance` > 0 (the larger of the two geoms', the same pair
rule as friction), the backend applies to each moving body a torque

    τ = −Crr · N · r · ω̂_t

with N the contact's normal force, r the distance from the body's centre of mass to the contact
point (the rolling radius for a wheel, sphere or cylinder), and ω̂_t the direction of the
bodies' relative spin less its part about the contact normal (turning on the spot is not
rolling). The other body takes the opposite torque. The torque is a Coulomb-like limit: it is
never more than stops the relative rolling in one step (I_contact · |ω_t| / dt), so a wheel at
rest stays at rest and is never rolled backwards. It is applied as a generalized force for the
next step (one step of lag, 1 ms by default).

MuJoCo's own rolling friction (condim 6, `friction[2]`) was not used: its coefficient is a
length (Crr · r) and the pair takes the larger geom's, so a ground geom cannot carry one value
that is right for wheels of different radii.

For a free roller of mass m, moment of inertia I and radius r down a slope θ this gives the
closed form

    a = g (sin θ − Crr cos θ) · m r² / (I + m r²)

and on flat ground a deceleration Crr · g · m r² / (I + m r²). The tests
`MujocoBackend.RollingResistanceSlowsABallAtTheClosedFormRate` (a solid ball on a 10° slope,
within 1 % of g sin θ), `RollingResistanceHoldsABallOnAGentleSlope` (Crr above tan θ holds it)
and `SurfaceFrictionScaleMakesTheBoxSlide` check it (`test_mujoco_backend`).

**Jolt.** Jolt scales friction the same way. Rolling resistance in Jolt is TODO: Jolt has no
rolling friction, and the same torque has to be applied from its contacts after each step with
`rollingResistanceTorque` (the helper is shared for that purpose). Until then a robot on grass
rolls farther under `--physics jolt` than under MuJoCo.

Road vehicles (`VehicleAPI_1`) do not use engine contacts for their wheels; see
"Wheels on grass".

## Lidar over grass

A collider with a grass preset (`sensmat:preset = "grass"`, `wet_grass`, `tall_grass`, or URDF
`<sensor_material preset="grass"/>`) is the soil, with blades of height `canopyHeight` standing
on it. A lidar beam that reaches the soil has crossed a path

    L = min(h / cos θ, r)

of canopy, where h is the canopy height, θ the beam's incidence on the soil and r the range. The
blades are treated as a turbid medium: a beam goes a free path s into it with probability
exp(−μ s), with μ = G · LAI / h. This is the gap fraction exp(−G · LAI / cos θ) of Nilson (1971)
spread evenly over the layer (Beer–Lambert), with G = 0.5 for a spherical leaf angle
distribution (Ross 1981). The return comes from the first blade, L − s before the soil, or from
the soil when s ≥ L. A grazing beam has a long path through the layer and mostly ends in the blade
tops; a steep beam reaches deeper and sometimes the soil. The mean shortening is

    E[Δr] = L − (1 − exp(−μ L)) / μ

which `test_shape_caster` (`CanopyReturn.*`) checks over 200 000 beams.

The draw comes from each beam's own Philox stream (seed, step, `kCanopyStream` + beam), so the CPU
reference stays bit-reproducible and the result does not depend on the number of threads. Tiers:
Tier 0 casts on the CPU; on Tiers 1 and 2 the GPU ray scan returns the soil hit and the same
shortening is applied on the CPU afterwards, so every tier gives the same canopy model. Surfaces
with `canopyHeight` 0 (every other preset) are unchanged.

Numbers and their status:

| Preset | ρ 905 nm | ρ 1550 nm | specular | canopy height |
|---|---|---|---|---|
| grass | 0.5 | 0.3 | 0 | 0.05 m |
| wet_grass | 0.45 | 0.2 | 0.05 | 0.05 m |
| tall_grass | 0.5 | 0.3 | 0 | 0.4 m |

- Reflectivities: green leaves reflect about half at the near-infrared plateau (905 nm) and less
  at 1550 nm, between the leaf water absorption bands. TODO(verify): read both values from the
  USGS Spectral Library v7 (Kokaly et al. 2017, USGS Data Series 1035) "Lawn_Grass GDS91"
  spectrum; the wet-grass numbers are not measured.
- Canopy height: 5 cm for a mown lawn, 40 cm for uncut grass. TODO(verify) against a mowing-height
  source.
- Leaf area index: 3 for all three (`kCanopyLeafAreaIndex`). TODO(verify) against turf
  measurements (e.g. Lee et al. 2011, Crop Science, mowing height and Kentucky bluegrass canopy).
- Not modelled: blades bending in the wind, multiple returns per beam (only the first), the
  blades' own reflectivity differing from the preset's, and grass seen from the side above its
  top (the canopy is only a layer on the collider's surface).
- The lidar uses the shaped canopy of "The canopy's shape" below (heights by place, leaf area by
  height). The closed forms in this section are those of its even-layer limit, which
  `CanopyReturn.*` still checks.

References: T. Nilson (1971), "A theoretical analysis of the frequency of gaps in plant stands",
Agricultural Meteorology 8, 25–38. J. Ross (1981), *The Radiation Regime and Architecture of
Plant Stands*, Junk. W. Ni-Meister et al. (2001), "Modeling lidar waveforms in heterogeneous and
discrete canopies", IEEE TGRS 39(9), 1943–1958. R. F. Kokaly et al. (2017), USGS Spectral
Library Version 7, USGS Data Series 1035.

## Reeds and shrubs

Reeds on a channel bank, shrubs and tall grass stand much higher than a lawn, so a beam or a robot
can be inside them without touching the soil. They come from the same surface presets (`reeds`,
`shrub`, `tall_grass`; USD `sensmat:preset`, URDF `<sensor_material preset="reeds"/>`), whose
`canopyHeight` h and `leafAreaIndex` (LAI) describe the layer of stems and leaves over the
collider.

**The canopy as a volume.** On a fixed box the canopy is the box's top face extruded upward by h;
on a plane it is the slab 0 ≤ z ≤ h (in the collider's frame, z up). Other shapes (hulls, height
fields, meshes) keep only the return at their hit described in "Lidar over grass".

**Lidar.** Inside the volume the leaves are a turbid medium, as for grass: a beam's free path s
has P(s > x) = exp(−μx), with μ = G·LAI/h and G = 0.5 (Nilson 1971; Ross 1981). A beam crossing
the volume over [enter, leave] stops at enter + s when that is before `leave`. This applies to
every beam that crosses a canopy, not only the ones that hit the top: a drone's lidar flying level
1 m over a reed bank sees the reeds, and the share of its beams that stop in a bank of width L is
1 − exp(−μL). A stopped beam takes the canopy's reflectivities, with the mean |cos| of a
spherical leaf angle distribution (0.5) as its incidence. The draws come from the beam's own
Philox stream, one per canopy crossed in collider order. The CPU and GPU tiers share this step:
the GPU scan returns the hit, and the canopies are crossed on the CPU after it. Tests:
`CanopyVolume.*` and `CanopyReturn.LeafAreaIndexOfTheSurface` in
`modules/sensors/test/test_shape_caster.cpp` (the stop share and the mean depth against the
closed form of a truncated exponential).

**Drag on bodies.** The `vegetation` domain (`lagrange_vegetation`, `modules/vegetation`) runs
whenever the world has such a canopy (`--domain vegetation=off` leaves it out, the zero
substitute). At 1 ms it cuts each moving body's colliders into cells, and each cell inside a
canopy is slowed by the quadratic drag of emergent stems per unit volume (Nepf 1999, eq. 1):

F = −½ ρ C_d a V |v| v

with a = G·LAI/h the frontal area per volume, V the cell's volume, v its velocity (with the
body's spin), C_d = 1, the value Nepf reports for sparse cylinder arrays (TODO(verify) for reeds
and shrubs, whose leaves are not cylinders), and ρ the site's air density (`--air-density`; no
air density, no drag). It writes `body/<b>/wrench/vegetation` (summed into the body's wrench) and
`body/<b>/in_vegetation`, the share of the body's volume in a canopy. There is no force at rest.

Nepf's model is for water flowing past fixed stems. In air the same form gives a small force: a
0.1 m³ robot at 1 m/s through 1.5 m reeds with LAI 4 feels about 0.08 N. A real robot pushing
reeds aside mostly works against the stems bending, which this model does not include
(TODO(verify) against a push-through test; a module config `density` can stand in for an
effective density). Tests: `modules/vegetation/test/test_vegetation.cpp` (the closed form,
quadratic in speed, opposite the motion, zero at rest, outside the layer and at h = 0).

### Numbers and sources

| Preset | h (m) | LAI | Source |
|---|---|---|---|
| `reeds` | 1.5 | 4 | Common reed (*Phragmites australis*) grows up to about 3–4 m in full stands; 1.5 m is a low or young bank stand. TODO(verify) the height against a survey and the LAI against reedbed measurements (LAI-2000 surveys, or LAI from specific leaf area) |
| `shrub` | 0.8 | 3 | TODO(verify) |
| `tall_grass` | 0.4 | 3 | TODO(verify) |

- Nepf, H. M. (1999). Drag, turbulence, and diffusion in flow through emergent vegetation.
  *Water Resources Research* 35(2), 479–489. doi:10.1029/1998WR900069.
- Ross, J. (1981). *The Radiation Regime and Architecture of Plant Stands*. Junk, The Hague.
- Nilson, T. (1971). A theoretical analysis of the frequency of gaps in plant stands.
  *Agricultural Meteorology* 8, 25–38.

## The canopy's shape

The even layer above is right on average but wrong in shape: a lidar sees it as a box, with a top
flat at exactly h, walls straight up the collider's edges and points spread evenly inside. Every
canopy (lawns, tall grass, reeds, shrubs) therefore has a shape that the lidar uses on every tier
(`canopyShapedStop`, `canopyShapedShortening` in `modules/sensors/src/ShapeCaster.cpp`).

**Height by place.** The local height is h(x, y) = H · f · j · g · e:

- f = 1 + spread · (2 n₁ − 1): tufts, clumps or bushes of different heights, from a smooth value
  noise n₁ ∈ [0, 1] at the form's patch size;
- j = 1 + jitter · (2 n₄ − 1): the same at the size of single stems, so the top is ragged, not a
  smooth dome;
- g: bare ground where a second noise is below the form's gap level;
- e: a box's border, which ends the plants at a distance from the edge that wanders with a third
  noise, up to the form's edge length, so it is ragged and not a wall.

The noise depends only on the position in the collider's frame, so every run and every tier sees
the same plants. The leaf area per ground area scales with the height (LAI · f · j · g · e), so a tall
tuft holds more leaf and the density inside the plants stays G · LAI / H. The tallest plants reach
H · (1 + spread) · (1 + jitter), which is the height of the volume a beam can stop in.

**Leaf area by height.** The leaf area density at a relative height ζ = z / h(x, y) is
(LAI_local / h) · p(ζ), where p integrates to 1 over [0, 1]:

| Form (`SurfaceMaterial::canopyForm`) | Presets | p(ζ) | patch | spread | gap | edge | stem | jitter |
|---|---|---|---|---|---|---|---|---|
| Grass | `grass`, `wet_grass`, `tall_grass` | 1.5 (1 − ζ²): dense at the base, thin at the tips | 0.25 m | 0.35 | 0.08 | 0.15 m | 0.04 m | 0.25 |
| Reeds | `reeds` | 0.3 + 0.7 · 12 ζ² (1 − ζ): stems all the way, leaf peaking at ⅔ | 1.2 m | 0.35 | 0.12 | 0.6 m | 0.12 m | 0.25 |
| Shrub | `shrub` | 0.1 + 0.9 · 20 ζ³ (1 − ζ): a crown at ¾ over bare stems | 1.5 m | 0.3 | 0.3 | 0.5 m | 0.2 m | 0.15 |

A beam's optical depth ∫ G · a ds is summed along it in midpoint steps of a twelfth of the
tallest height up or down, or half a stem across, whichever is shorter for its slope. The
beam stops where the sum reaches −ln(1 − u), for its own Philox draw u. Straight down through a
point with height factor f, it reaches the soil with Nilson's gap fraction exp(−G · LAI · f).

Tests in `modules/sensors/test/test_shape_caster.cpp`, `CanopyShape.*`:

- the profiles integrate to 1;
- heights vary by place within their bounds and leave some bare ground;
- returns from above a reed bank are not one plane, and its border is ragged;
- a vertical beam's share of soil returns matches exp(−G · LAI · f) for each form;
- the same beam always stops in the same place.

Every number in the table is TODO(verify). The patch sizes come from photographs of the plants,
not from measurements. The profile shapes are simple polynomials, of the kind measured in stands
(e.g. Lalic and Mihailovic 2004, *J. Appl. Meteor.* 43, a leaf area density with its maximum in
the upper canopy). They are not fitted to these plants.

The realistic view draws its own reeds and shrubs (`vsVeg`), which are not yet the same plants
the lidar sees. The `vegetation` domain's drag still uses the even layer.

## Grass in the rotor downwash

The realistic look (viewer and camera sensors, invariant 13) bends grass and vegetation by the air
over the ground, read only through `groundAir` in `gpu/kernels/camera/realistic.slang`: the site
wind plus the outwash of every hovering robot's rotors. The outwash is the aerodynamics domain's
own (`aerodynamics::Downwash`, [downwash.md](downwash.md)), ported to `lagrange/gpu/GroundAir.hpp`
for a level ground and no wake skew:

- per robot, the wake goes down from its mean hub along the summed thrust axes and arrives at the
  ground with the near wake's speed `v_i·(1 + s/√(s² + R²))` (Leishman 2006, §2.7), blended into
  the merged jet of Bauersfeld et al. (2024, arXiv:2403.13321) between 1.5 and 2.5 motor
  distances below;
- the radial outwash there follows the fit to Tanner et al. (2015, AHS Forum 71,
  NTRS 20160006428): a peak of 1.9 V_h at 1.7 R from the axis, 0.05 R above the ground, in a
  layer 0.15 R deep, slower and wider for a higher rotor.

Blades stand taller than that wall layer, so the grass is bent by the jet's peak speed at each
radius. The rotors come from `rotor/<j>/thrust`, `induced_velocity` and `thrust_axis` and the
rotor link's pose: the viewer subscribes to them through the protocol, camera sensors read them
from the bus (optional inputs, last step's values); at most 8 robots per frame.

The outwash is unsteady (Tanner et al. report fluctuations of the order of the mean); it is drawn
as gusts running outward at half its speed, broken by noise. Their size and strength are a
drawing choice (TODO(verify): the measured spectrum). `test_downwash`'s `GroundAir.*` check the
port against `Downwash::velocityAt` (to 1e-9) and that no rotors leave the site wind alone.

## Cutting the plants

A spinning propeller that reaches into grass, reeds or shrubs cuts them. The `vegetation` domain
(`modules/vegetation`, `CutMap`) does this for every rotor (`RotorAPI_1` joint) while the run has
canopies. It needs no air density.

- **Where it cuts.** Each millisecond, a rotor whose blade tips move at 5 m/s or faster sweeps its
  disc: the hub at the rotor link's origin, the joint axis as normal, `aero:rotor:radius`. The disc
  is sampled on rings half a cell apart. Every 4 cm cell of a canopy's top that the disc passes
  over, with the blades below the plants (twice the preset height bounds the tallest), is cut to
  the blades' height over the soil, or to the soil when the blades go into it. A tilted disc cuts
  a slope.
- **It stays cut.** A cell is cut again only when the blades go at least 1 cm (`kMinCutPiece`)
  lower than before: a shorter stub bends away. The plants do not grow back.
- **What it costs.** Each cell cut lower takes `kCutEnergyPerArea` (100 J per m² of ground) to
  sever its stems. This is counted only where the plants stood above the cut. The rotor pays it as
  a torque against its spin on the propeller's link (`body/<link>/wrench/cutting`), so the motor
  sees the load. That torque is shown as `rotor/<j>/cutting_power` (W). A deep plunge is paid over
  several steps, never more than a twentieth of the propeller's kinetic energy per step, so it
  slows the propeller instead of reversing it.
- **Who sees it.** The cut map is published as `vegetation/canopy_cut`, one array with the layout
  in `lagrange/world/CanopyCut.hpp`: up to 16384 cells, 26 m² of cut ground; cells beyond that
  are not kept. The lidars read it as an optional input and stop no beams above the cut in a box
  or plane canopy. The stubble keeps the leaves it had below the cut, so a cut patch shows up as a
  hollow in the point cloud. The stems' drag (`body/<b>/wrench/vegetation`) also stops at the cut.
  The map is saved and restored with the run's state.

The energy comes from cutting tests on forage stems: of the order of 10–60 mJ per mm² of stem cut
(Persson 1987, *Principles, Design and Testing of Reaper Forage Harvesters*, ASAE). A lawn stands
some 4000–10000 mm² of stem per m², which gives 40–600 J/m². 100 J/m² is one value in that range.
The value, its spread between grass, reeds and shrubs, the 1 cm piece and the 5 m/s tip speed are
all TODO(verify).

Checks:

- `test_vegetation`'s `Cutting.*`: a level disc cuts the cells under it and its area; the same
  sweep again cuts nothing; a tilted disc cuts a slope; only the part over a box is cut; the drag
  stops at the cut; a saved map comes back whole.
- `test_shape_caster`'s `CanopyShape.CutPlantsStopBeamsNoHigherThanTheCut`.
- `test_server`'s `Multicopter.TakingOffFromTallGrassCutsItUnderTheRotors`: the sample quadcopter
  takes off from tall grass. Each rotor cuts a disc to its blades' height, the energy paid is four
  discs' worth, the grass 3 m away stands, and it still climbs to its 2 m hover.

Not modelled yet:

- Other blades: a mower's or a robot's own cutters.
- Meshes and height fields as canopies.
- The clippings.
- Damage to the propeller.
- The viewer: it still draws the grass uncut. Camera depth does not see the stubble either.

## Trees

A tree is a `TreeAPI_1` prim at the foot of its trunk (formats.md §3.17). Its trunk is whatever
collider the stage gives it, a solid cylinder in the sample. Its crown is not a collider. It is a
volume of leaves and twigs: an ellipsoid over the crown base with semi-axes `R`, `R` and
`c = (height − crownBase)/2`, through which things move as through a canopy.

**What pushes through it.** A body's cells inside the crown are dragged as a canopy drags them
(§ "Reeds and shrubs"), `F = −½ ρ Cd a V |v| v`, with the frontal area per volume
`a = G·LAD` (`tree:leafAreaDensity`, G = 0.5), relative to the swaying crown.

**Sway.** The crown has one sway mode in each horizontal direction, a damped oscillator of the
crown's mass `m` (`tree:crownMass`), natural frequency `f` and damping ratio `ζ`:

```
m x'' + 2 ζ ω m x' + ω² m x = F,    ω = 2π f
F = ½ ρ Cd A |u − x'| (u − x')       A = π R c
```

`u` is the horizontal air at the crown: the site wind (`env/wind/velocity`) plus the rotors' wake
there (`tree/<n>/downwash`, which the aerodynamics domain writes as the mean of its downwash at the
crown's centre and four points half way to its edge). Taking the air relative to the moving crown
gives the aerodynamic damping that real crowns have. In a steady wind the crown settles at
`F/(m ω²)`; let go, it swings at `f√(1 − ζ²)` with a logarithmic decrement `2πζ/√(1 − ζ²)`. The
crown moves as a whole by its centre's displacement (`tree/<n>/sway`), for the drag, the cuts and
the lidars.

**Cutting.** A spinning rotor whose blade tips move faster than 5 m/s cuts every cell of leaves
its disc passes through (cells of 10 cm in the crown's own frame, kept in `vegetation/tree_cut`,
lagrange/world/Trees.hpp). It pays `tree:cutEnergy` per m³ cut as a torque against its spin, as for
the grass. The volume cut so far is `tree/<n>/cut_volume`, and the leaves it cut off,
`cut volume × LAD / tree:leafArea`, are `tree/<n>/leaves_shed`. Cut cells hold no leaves: nothing
drags there, and the lidars see through them.

**Lidars.** A beam crossing a crown stops in it with the gap probability: its free path through
the leaves has `P(s > x) = exp(−G·LAD·x)`, counted only through cells not cut away, the crown at
its sway (`sensors::crownStop`). The return is a leaf's (the `shrub` preset's reflectivity) facing
the beam at random.

Checked:

| Check | Reference | Result |
|---|---|---|
| The crown's ellipsoid, swayed | its equation | `test_vegetation` `Trees.TheCrownIsAnEllipsoidOverItsBase` |
| 8 m/s steady on a 10 m oak (R 3 m, c 3.5 m, Cd 0.3, 0.5 Hz) | `F/(m ω²)` | 387.9 N, 0.149 m, within 1e-4 |
| Let go from 0.3 m | period `1/(f√(1 − ζ²))`, decrement `2πζ/√(1 − ζ²)` | within 0.2 % and 1 % |
| A 10-inch disc level through the centre | the cells under the disc, one cell thick | cut, the cells above and beside it not; nothing more on a second pass |
| A beam through 6 m of crown, LAD 0.5 | passes with `exp(−G·LAD·6)` = 0.223; mean stop depth of the truncated exponential | within 0.002; within 1 % (`test_shape_caster` `Crown.*`) |
| Cut cells and sway | the beam passes the cut line; the crown moves with its sway | `Crown.CutCellsLetTheBeamThroughAndTheSwayMovesTheCrown` |
| The quadcopter taking off under the open field's tree_1 | its rotors in the crown | 0.118 m³ cut, 39 leaves, up to 41.6 W of cutting; bit for bit twice (`test_trees_run`) |

### Numbers and sources

Every fallback of `TreeAPI_1` is TODO(verify) against measurements of the species: a leaf area
density of 1 m²/m³ (broadleaf crowns run from about 0.5 to 2), a crown drag coefficient of 0.3
(crowns streamline and their CdA falls with the wind, Vollsinger et al. 2005, Can. J. For. Res.
35, 1238), a first sway frequency of 0.5 Hz and a damping ratio of 0.1 (open-grown trees of some
10 m; Moore and Maguire 2004, Trees 18, 195, collect measured frequencies and damping ratios),
a crown mass of 2 kg per m³ of crown, 2000 J to cut a m³ of crown, a leaf of 30 cm². The sway is
one mode of the whole crown: branches do not sway on their own, and the crown does not bend
(its top moves as its centre). The crown's air is sampled at its centre's height; rotors inside
the crown under it give it their inflow, not their wake. Not yet: leaves and twigs that fly
from a cut or a gust, the realistic view's trees, the crown's shelter of the air behind it,
branches that stop a propeller, trees broken by a vehicle.

