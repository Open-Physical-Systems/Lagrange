# RFC 0018: The radiation domain (ionizing radiation)

| | |
|---|---|
| Status | Draft (for the owner to accept or reject) |
| Created | 2026-11-15 |
| Affects | `schemas/` (one new library, replacing the unshipped design of formats.md §3.6), `modules/radiation/` (new), `modules/sensors/` (a `dosimeter` kind, camera hits, fault triggers), `core/signals/` (unit `Sv`, two appended `QuantityKind`s), `io/fields/` (a dose-map converter), `server/` (field slices, an export), `profiles/schema.json` (additions), `viewer/` (Environment panel, overlay), `tools/cli/`, `docs/` |
| Not affected | `proto/` (the existing field slice and `FieldVolume` messages carry it), `core/plugin_abi/` |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas, its open question 3), RFC 0004 (tiers), RFC 0005 (profiles), RFC 0007 (the 1.x freeze), RFC 0008 (custom sensor kinds), RFC 0009 (humans), RFC 0013 (the closest analogue: EMI sources, victims, effects) |

## Summary

Gamma-ray dose in the scene: sources (a point, a line, an area, a uniform background, or a dose
map computed offline), shielding by the materials of the scenery and the robot, the dose rate
and the accumulated dose at each part that cares (an electronics board, a camera, a person), and
what that does: a dosimeter reads it, a camera's frames get the bright "snow" of gamma hits,
electronics fail past their total-dose limit, and single-event upsets fire faults through the
sensor-fault machinery that already exists. A new domain `radiation` with its `zero`, constant
and replay substitutes; three new schemas; additions to the profile, the units and the sensors.
In the loop the transport is a point kernel (straight rays, exponential attenuation, a build-up
factor), with its limits stated; anything a point kernel cannot do (scatter round a corner,
neutrons, skyshine) comes in as a dose map from an offline Monte Carlo code (Geant4, OpenMC, MCNP)
converted to NanoVDB. Every change is an addition under RFC 0007.

## Motivation

Radiation is the one domain of CLAUDE.md §1 with no module (`docs/plan/evaluation-2026-11.md`
item 11): profiles name it, and only the `zero` substitute runs. It is post-1.0 roadmap work
(CLAUDE.md §2). The users it serves:

- **Nuclear decommissioning and inspection.** Robots sent into cells, ponds and reactor
  buildings to map dose, find hot spots and handle waste. The tasks that need a simulator are
  the robot's own: plan a survey that maps a room's dose rate, choose a route that keeps the
  robot's (and its electronics') dose under budget, test software against a camera that
  degrades with dose rate and boards that die after a total dose.
- **Space and high altitude.** A uniform background of particles in which electronics take
  total dose and upsets; the orbit's spectrum comes from offline tools, Lagrange only applies it.
- **Medical and industrial sites.** Robots near X-ray and isotope sources (radiography, sterilisation
  cells). Low-energy, scatter-dominated fields: field maps only (non-goals).
- **People in the scene.** The walking humans of RFC 0009 near a source, so a planner that keeps
  people out of a dose zone can be tested.

The design doc `docs/design/domains/radiation.md` (written before v0.2) and formats.md §3.6
sketch a module and a single `RadiationAPI_1`. Neither shipped (no `schemas/radiation`, nothing in
`schemas/test/frozen_1_0.json`), so this RFC is free to revise them, and does in three places:
the schema split (RFC 0002 open question 3), materials instead of one attenuation coefficient
per prim, and upsets from particle flux rather than from dose (§5).

## Non-goals

- A radiation-protection or shielding-design tool. The documentation says so, as EMI's does
  (docs/plan/00-overview.md). Dose numbers are for testing robot software.
- Transport of neutrons, charged particles and betas in the loop. Neutrons enter through dose
  maps and as a particle flux for upsets (§5); alphas and betas are stopped by the robot's skin
  and are not modelled.
- Scatter beyond the build-up factor: no skyshine, no streaming through ducts and mazes, no
  backscatter from walls. Dose maps carry them (§3).
- Contamination (a robot picking up activity), activation, and decay chains. A source's
  activity is constant during a run (open question 9).
- Spectroscopy and gamma imaging (a spectrometer's pulse-height spectrum, a Compton camera).
  A later RFC (open question 11).
- Orbital environment models (trapped belts, solar particle events). Users bring their numbers
  from offline tools.

## Design

### 1. Data: three schemas (new library `schemas/radiation`, namespace `rad:`)

RFC 0002's open question 3 asked whether one `RadiationAPI` with a role token or separate
schemas; formats.md §3.6 chose one. This RFC proposes three (open question 1), because a source
is often several nuclides on one prim, shielding is a material property that belongs where other
materials are resolved, and a victim's attributes have nothing in common with a source's. The
`rad:` namespace and the units `Gy`, `Bq` are those RFC 0002 reserved. Energies are in J (the
schema unit grammar has no eV); the URDF and overlay forms take MeV, converted with
1 MeV = 1.602176634e-13 J (exact, from the SI's fixed elementary charge).

**`RadiationSourceAPI_1`** (multiple apply, instance `rad:source:<n>:`), on any prim: its pose,
and its body's when it is on one, place it, so a source carried by a robot moves with it (as EMI
sources do, `SchemaInstance::body`).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `shape` | token | — | `point` | `point`, `line` (along the prim's x, centred), `area` (a rectangle in its xy plane), `uniform` (the whole scene, unshielded: a background), `field` (a dose map) |
| `nuclide` | token | — | `none` | an entry of the nuclide table (§2): its photon lines and yields |
| `energies` | double[] | J | `[]` | or the photon lines themselves |
| `yields` | double[] | 1 | `[]` | photons per decay of each line (with `energies`) |
| `activity` | double | Bq | none | required with `nuclide` or `energies` (for `line` and `area`, the total) |
| `airKermaRateAt1m` | double | Gy/s | none | instead of `activity`: the unshielded air kerma rate at 1 m from a point source, as survey data give it; needs a single line in `energies` for the attenuation |
| `length` | double | m | none | `line` |
| `size` | double2 | m | none | `area` |
| `field` | asset | — | none | `field`: a NanoVDB scalar grid in the prim's frame (§3) |
| `fieldQuantity` | token | — | none | `field`, `uniform`: `air_kerma_rate` (Gy/s), `silicon_dose_rate` (Gy/s), `ambient_dose_equivalent_rate` (Sv/s), `particle_flux` (1/(m2.s)) |
| `fieldValue` | double | per `fieldQuantity` | none | `uniform`: its value |
| `particle` | token | — | `photon` | `photon`, `neutron`, `proton`, `heavy_ion`: what a `particle_flux` counts, for upsets (§5) |
| `quadratureNodes` | int | 1 | `0` | `line`, `area`: points the source is split into; 0: chosen from the distance (§2) |

No numeric fallback is a physical value: a source without its activity or rate is a load error
(RFC 0002 rule 5).

**`RadiationMaterialAPI_1`** (single apply), on a prim or on a `UsdShade` material bound to it,
resolved like `SensorMaterialAPI_1` and `ThermalMaterialAPI_1` (prim, ancestor, bound material):

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `rad:material:preset` | token | — | `none` | a material of the attenuation table (§2): elements by name (`lead`, `iron`, `tungsten`, `aluminium`, `copper`, `silicon`) and mixtures (`water`, `air`, `concrete_ordinary`, `polyethylene`, `tissue_soft`); TODO(verify) each against NIST SRD 126's list before it is added |
| `rad:material:density` | double | kg/m3 | none | required (a preset fixes the composition, not the density: concrete and soil vary) |
| `rad:material:elements` | token[] | — | `[]` | or a composition: element symbols |
| `rad:material:massFractions` | double[] | 1 | `[]` | their mass fractions (summing to 1) |
| `rad:material:energies` | double[] | J | `[]` | or a table of its own: energies |
| `rad:material:massAttenuation` | double[] | m2/kg | `[]` | μ/ρ at those energies |
| `rad:material:massEnergyAbsorption` | double[] | m2/kg | `[]` | μen/ρ at those energies (needed only where dose is scored in this material) |

A prim with no radiation material does not shield (open question 7 on what the start logs).

**`RadiationSusceptibilityAPI_1`** (single apply), the victim, on a part's prim (a board, a
camera, a motor driver), named like EMI's `EmiSusceptibilityAPI_1`:

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `rad:victim:medium` | token | — | `silicon` | the material the dose is scored in: `silicon`, `air`, `water`, `tissue_soft` |
| `rad:victim:point` | double3 | m | `(0, 0, 0)` | where it is scored, in the prim's frame |
| `rad:victim:totalDoseLimit` | double | Gy | none | absent: never fails from total dose |
| `rad:victim:totalDoseEffect` | token | — | `disconnect` | past the limit: `disconnect`, `noise` (grows with dose past the limit, `rad:victim:noisePerGray`), `none` (only the `failed` flag) |
| `rad:victim:noisePerGray` | double | 1/Gy | `0` | `noise`: extra noise as a share of the quantity's full scale per gray past the limit |
| `rad:victim:seuCrossSection` | double | m2 | `0` | per bit |
| `rad:victim:seuBits` | int | 1 | `0` | bits exposed |
| `rad:victim:seuParticles` | token[] | — | `[neutron, proton, heavy_ion]` | which `particle_flux` fields cause upsets |
| `rad:victim:seuEffect` | token | — | `stuck` | the sensor fault an upset fires: `stuck`, `bias`, `dropout`, `reset` (a dropout of `rad:victim:resetTime`) |
| `rad:victim:resetTime` | double | s | `0` | `reset` |
| `rad:victim:target` | rel | — | the prim itself | the sensor(s) whose outputs it corrupts |
| `rad:victim:pixelHitsPerGray` | double | 1/Gy | `0` | on a camera: gamma hits per pixel per gray (silicon) |
| `rad:victim:hitSignal` | double | 1 | `0` | on a camera: a hit's mean signal as a share of full scale |

`0` here means "this effect is off", not a guess at a device. There is no default for any device
constant: TID limits, cross-sections and hit rates come from the user's datasheet, test report or
measurement (CLAUDE.md §10).

In URDF (`<gazebo reference="L">`, read in `addSensors` beside `<thermal_material>`) and the
overlay YAML, the same keys: `<radiation_material preset="lead" density="..."/>`,
`<radiation_victim medium="silicon" total_dose_limit="..." .../>`, and sources
`<radiation_source name="s1" nuclide="cs137" activity="..."/>`.

### 2. Model in the loop: point kernel (`radiation.fidelity: point_kernel`)

For a point source emitting photons of energy `E_k` at `S_k = A·y_k` per second, the uncollided
fluence rate at distance `r`, through materials `i` crossed for path lengths `t_i`, is
`φ_k = S_k / (4π r²) · exp(−Σ μ_i(E_k) t_i)`, and with scatter folded into a build-up factor `B`,
the dose rate in medium `m` is

`Ḋ = Σ_k φ_k · B_k · E_k · (μen/ρ)_m(E_k)`

(the point-kernel method; J. K. Shultis and R. E. Faw, *Radiation Shielding*, TODO(verify) edition
and chapter). Air kerma uses `m = air`; silicon dose `m = silicon` (charged-particle equilibrium
assumed, stated in the docs).

- **Attenuation data.** μ/ρ and μen/ρ from NIST Standard Reference Database 126 (J. H. Hubbell
  and S. M. Seltzer, *Tables of X-Ray Mass Attenuation Coefficients and Mass Energy-Absorption
  Coefficients from 1 keV to 20 MeV for Elements Z = 1 to 92 and 48 Additional Substances of
  Dosimetric Interest*, NISTIR 5632, doi:10.18434/T4D01F), transcribed by a script with the
  retrieval date and a checksum, in m²/kg (cm²/g × 0.1). Between table energies, log-log linear
  interpolation (open question 6 on edges). Example row, read from the database's lead table:
  at 0.6 MeV, μ/ρ = 1.248e-1 cm²/g and μen/ρ = 6.819e-2 cm²/g. Mixtures by the mixture rule
  `μ/ρ = Σ w_j (μ/ρ)_j` (as XCOM computes compounds: Berger et al., *XCOM: Photon Cross Sections
  Database*, NIST SRD 8, doi:10.18434/T48G6X), which ignores chemical binding, negligible above
  a few keV (TODO(verify) the statement's source). TODO(verify): NIST's terms for redistributing
  the tables in an Apache-2.0 repository.
- **Nuclide table.** Each nuclide's photon lines and yields with its source (NNDC NuDat / ENSDF,
  with the retrieval date); a nuclide without a cited source is not added. First set proposed:
  `cs137`, `co60`, `am241`, `ir192`; TODO(verify) every line and yield from NuDat. No number in
  this RFC is offered for them.
- **Build-up.** The geometric-progression (G-P) fitting formula of Harima et al. (Harima,
  Sakamoto, Tanaka, Kawai, Nucl. Sci. Eng. 94, 1986, TODO(verify) volume and pages), adopted in
  ANSI/ANS-6.4.3-1991, *Gamma-Ray Attenuation Coefficients and Buildup Factors for Engineering
  Materials*. Its five coefficients per material and energy come from the standard or from
  RSICC's DLC-129 (ANS643) data package; the standard's reproduction is restricted by the ANS, so
  open question 5 asks where the coefficients may come from. The data cover up to 40 mean free
  paths (Harima, Kurosawa and Sakamoto, 2014, extended the formula to 100 mfp; TODO(verify)
  citation). Profile `radiation.buildup: gp | none`. Through several materials along one ray,
  the build-up of the material with the most mean free paths on it (open question 4).
- **Path lengths.** Rays from the victim to each source point through the collision shapes of
  the scenery and every body at its pose this step (`body/<b>/pose` as feedback inputs, like the
  tether module). Unlike lidar, which needs the first hit, attenuation needs every entry and exit
  along the ray: a new segment query beside `sensors::BodyScene`'s rays (`segmentsAlong(origin,
  target)` → `(material, length)` pairs). Closed convex shapes give exact chords; a concave mesh
  baked by `collision_bake` is convex parts already. Hollow objects are open question 8.
- **Line and area sources.** Split into `quadratureNodes` points (auto: a node spacing under a
  fifth of the distance to the nearest victim, capped; logged), each a point kernel with its share
  of the activity. Checked against the unshielded closed forms (§Validation).
- **Uniform sources.** `fieldValue` everywhere, unshielded: a background (space, a site's
  natural background), and the particle flux for upsets.

### 3. Dose maps from offline Monte Carlo (`field_map`, `mixed`)

Point kernels miss what scatter does round corners, through mazes and from the sky, and do not
do neutrons. Those come as `shape = field` sources: a NanoVDB scalar grid of one
`fieldQuantity`, computed offline and sampled trilinearly at each victim (CPU NanoVDB, as the
weather's fields). `field_map` uses the maps only; `mixed` adds the point kernels of other
sources (the user makes sure one source is not in both).

- `ros2 run lagrange_fields lagrange_dose_to_vdb IN OUT.nvdb --quantity Q [--scale S]
  [--offset X,Y,Z]`, beside `lagrange_foam_to_vdb`, reads a mesh tally as CSV (voxel centre,
  value, relative error): Geant4's command-based scoring dump (`/score/dumpQuantityToFile`;
  TODO(verify) its column layout in the Geant4 version pinned), OpenMC's mesh tally exported to
  CSV by a documented script, and a plain `x,y,z,value` file for MCNP FMESH output or anything
  else. Voxels whose relative error is over `--max-error` (default none: kept) are flagged in
  the grid's metadata and logged.
- `lagrange export gdml [PROFILE|ROBOT WORLD] --out FILE.gdml` writes the scene's shielding
  geometry (collision shapes with their radiation materials, sources as comments with their
  nuclide and activity) in GDML, Geant4's geometry format, so the offline run sees the same
  room. Geant4 (TODO(verify) its licence's compatibility statement), OpenMC and MCNP stay
  separate programs, files only (invariant 12); MCNP is export-controlled and never referenced
  beyond its output format.
- A map is static: it holds for the scene as it was when computed. Moving shields (a door, the
  robot itself) are not in it; the docs say so, and `mixed` lets the robot's own parts still
  shield its victims from point sources.

### 4. Signals

All per step of the domain (default 100 ms, open question 10), `rad/` prefix as the design doc:

| Signal | Type | Unit | |
|---|---|---|---|
| `rad/<v>/dose_rate` | Scalar | Gy/s | in the victim's medium |
| `rad/<v>/dose` | Scalar | Gy | accumulated since the run's start (kept by `saveState`) |
| `rad/<v>/particle_flux` | Scalar | 1/(m2.s) | of its `seuParticles` |
| `rad/<v>/failed` | Bool | 1 | past `totalDoseLimit` |
| `rad/<v>/seu_count` | Int | 1 | upsets so far |
| `rad/body/<b>/dose_rate`, `dose` | Scalar | Gy/s, Gy | silicon at each body's origin, when `radiation.body_dose: true` (default false) |
| `rad/human/<n>/dose_rate`, `dose` | Scalar | Sv/s, Sv | ambient dose equivalent at each human's chest, when humans run (open question 3) |
| `rad_source/<n>/enable` | Bool input | 1 | optional, default 1: scenarios switch a source (a shutter) |

`Sv` is added to the unit grammar (RFC 0002 rule 9, an addition) and `QuantityKind` gains
`DoseEquivalent` and `DoseEquivalentRate`, appended (the enum's own rule), so a `Gy` output
cannot be wired to a `Sv` input although their dimensions are equal (invariant 2).

### 5. Effects (applied where the thing they corrupt is modelled, as EMI's are)

- **Dosimeter.** A built-in custom sensor kind `dosimeter` (RFC 0008 kinds, a sensor-catalog
  entry; URDF `<sensor type="dosimeter">`, no schema change): parameters `quantity`
  (`air_kerma`, `silicon`, `ambient_dose_equivalent`), `sensitivity` (counts per gray or per
  sievert, required for counting detectors), `dead_time` (s), `noise`, `range`, `update_rate`.
  Counts per sample are Poisson from the counter-based RNG, with the non-paralysable dead-time
  loss `m = n / (1 + nτ)` (G. F. Knoll, *Radiation Detection and Measurement*, TODO(verify)
  edition and section); the reading is counts back to dose rate, saturating past `range`.
  Outputs `sensor/<n>/dose_rate`, `sensor/<n>/dose`, `sensor/<n>/counts`; ROS
  `/<name>/dose_rate` as `std_msgs/Float64` (no standard ROS message for dose was found;
  TODO(verify) before choosing). This is the sensor a dose-mapping robot carries.
- **Camera hits.** On a camera with `pixelHitsPerGray > 0`, the sensors module reads its optional
  input `rad/<camera>/dose_rate` (default 0, so nothing changes without the domain) and per frame
  adds Poisson(`pixelHitsPerGray · Ḋ · t_exposure`) hits per pixel, each of signal
  `hitSignal` (exponentially distributed; TODO(verify) the distribution against the measurements
  below) before the sensor model's quantisation. This is the transient "snow" reported for CMOS
  cameras under gamma (Khanam et al., "Gamma-Induced Image Degradation Analysis of Robot Vision
  Sensor for Autonomous Inspection of Nuclear Sites", IEEE Sensors Journal, 2022; TODO(verify)
  volume and the dose rates quoted before using their numbers). Depth and instance images are
  unchanged. Total-dose effects on cameras (dark current, hot pixels) are `noise` past the limit
  for now.
- **Total ionizing dose.** Past `totalDoseLimit`, `failed` turns true and the target sensors get
  the `totalDoseEffect` through the sensor-fault machinery (`sensors::withFaults`): `disconnect`
  or a `noise` fault whose sigma grows with the dose past the limit. A real part's degradation is
  gradual and dose-rate dependent; one threshold is a behavioural model and is documented as one.
- **Single-event upsets.** Expected upsets per step `λ = σ · N_bits · Φ · Δt`, drawn Poisson from
  the victim's own Philox stream (`random::Stream`, so a run and a restored checkpoint repeat).
  The formats.md §3.6 sketch derived the flux from the dose (`fluenceToDose`); that is dropped:
  upsets come from the flux of particles that deposit enough charge in a sensitive volume
  (neutrons, protons, heavy ions), not from the gamma dose, so `Φ` is the `particle_flux` of the
  victim's `seuParticles` from `uniform` and `field` sources, and a gamma-only scene gives no
  upsets. A flat cross-section above an implicit threshold is the simplest of the forms used in
  soft-error testing (the JEDEC JESD89 family covers its measurement; TODO(verify) which part
  states the reporting form). Each upset fires the victim's `seuEffect` as a one-sample (or
  `resetTime`) fault on its target: an addition to sensor faults, a fault triggered by an Int
  signal's increment (`trigger: rad/<v>/seu_count`) instead of a time (profile `sensor_faults`
  gets an optional `trigger` key; the CLI's `--sensor-fault` the same). Plugins read `seu_count`
  for anything else (a flipped bit in their own state). Profile `radiation.seu_injection`, which
  exists with default false, turns this on.
- **People.** `rad/human/<n>/*` for planners and tests; nothing happens to the walkers.

### 6. Substitutes

| Profile token | Kind | Behaviour |
|---|---|---|
| `zero` | constant | every `rad/*` output 0, `failed` false, no upsets (exists today) |
| `{kind: constant, preset: uniform, value: Ḋ}` | constant | every victim at `value` Gy/s (and Sv/s for humans), no shielding, no particle flux; for testing a robot's reaction to a dose level |
| `replay` | replay | from MCAP |

### 7. Tiers and determinism

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Point kernels at victims (victims × source points × lines) | CPU | CPU | CPU |
| Dose maps at victims | CPU NanoVDB | CPU NanoVDB | CPU NanoVDB |
| Overlay slice for the viewer | CPU, coarse (16 cm cells) at 1 Hz | `gpu/kernels/fields/dose.slang` over the compute BVH shared with the lidar | same kernel with ray queries |

Victims are few (tens of rays per step), so the in-loop path is CPU on every tier and is the
deterministic reference (invariant 7): fixed order of victims, sources, nodes and segments; no
threads; random draws only from Philox streams per victim. GPU is used only for the picture.

### 8. Viewer

The server adds a field `rad/dose_rate` (log scale, Gy/s in air kerma, or Sv/s with
`--slice rad/dose_rate:xy,sv`), sliced on the server like `emi/flux_density` (point kernels
evaluated per cell, maps resampled) and sent as the existing slice messages and `FieldVolume`
boxes; no protocol change. The Environment panel gets a Radiation section (as RFC 0013's EMI
section): each source (nuclide, activity, its air kerma rate at 1 m) and each victim (dose rate,
dose, percent of its limit, upsets), with *Overlays → dose* and *Coupled into* listing the
effects. Following the binding UI design (`docs/design/ui/`); its spec is a phase-6 deliverable.

## Validation

In `test_radiation` unless named:

1. **Inverse square.** An unshielded point source: `φ = S/(4πr²)`, 1e-12 relative.
2. **Narrow-beam slab.** One lead slab, `B = 1`: `exp(−μt)` with μ/ρ read from the SRD 126
   table at a tabulated energy (no interpolation), 1e-12 relative to the formula; the table
   values themselves traced to the transcription's checksum.
3. **Line source.** Unshielded finite line, on its perpendicular bisector and off it:
   `φ = S_L/(4πh)·(θ1 + θ2)`; the quadrature converges to it (error under 1e-4 at the auto node
   count).
4. **Disc/area source.** On the axis of a disc: `φ = (S_A/4)·ln(1 + R²/h²)`; the rectangle's
   quadrature against a numerically integrated reference.
5. **Build-up.** The G-P formula reproduces the tabulated build-up factors it was fitted to
   within the fit's stated accuracy (a few percent, Harima et al.), for the materials and
   energies shipped; source of the table per open question 5.
6. **Dose from fluence.** `Ḋ = φ E μen/ρ` for air and silicon at tabulated energies, exact.
7. **Statistics.** Upset counts and dosimeter counts: Poisson mean and variance over 10⁶ draws
   within 3σ; dead-time loss against `n/(1 + nτ)`; repeat after `restoreState`.
8. **Maps.** An analytic 1/r² field baked by `lagrange_dose_to_vdb` and sampled back within the
   trilinear interpolation bound.
9. **Against Monte Carlo** (`docs/validation/`). A sample cell (`assets/worlds/hot_cell`: a
   concrete room, a source, a lead wall, a doorway and a maze) exported by `lagrange export gdml`
   and run offline in Geant4 by a maintainer; the result stored as a small reference table. The
   point kernel is compared behind the wall and in the doorway's line of sight (where it should
   agree within a tolerance written down after the first comparison, as RFC 0012 test 6 did), and
   round the maze's corner (where it cannot, which the table shows and the docs state). The
   design doc's "within a factor of 2" is not claimed until measured.
10. **Published benchmark.** None open has been found that fits: SINBAD (OECD NEA and RSICC)
    holds shielding experiments, but its maintained version's distribution is restricted and on
    request, and its photon entries found so far (skyshine) are outside a point kernel's scope.
    Open question 12.
11. **Camera.** The hit model's parameters fitted to the dark-frame measurements of Khanam et al.
    (2022) when their numbers are read from the published paper, kept as a regression; until then
    the camera model ships documented as uncalibrated.
12. **Determinism.** `lagrange determinism` on the sample with every effect on.

## Steps (each a PR)

1. This RFC.
2. Schemas `RadiationSourceAPI_1`, `RadiationMaterialAPI_1`, `RadiationSusceptibilityAPI_1`;
   frozen-schema additions; formats.md §3.6 rewritten; unit `Sv` and the two `QuantityKind`s.
3. Data: the SRD 126 transcription script and tables, the nuclide table with sources, the G-P
   coefficients (per open question 5); tests 2, 5, 6.
4. `lagrange_radiation`: point kernels, segment queries, line/area/uniform sources, victims, TID,
   substitutes, profile keys (`fidelity`, `buildup`, `body_dose`), URDF and overlay reading;
   tests 1, 3, 4, 7.
5. Effects in `lagrange_sensors`: the `dosimeter` kind, camera hits, fault triggers and upsets;
   test 11's model with its parameters as user inputs.
6. Maps: `lagrange_dose_to_vdb`, `lagrange export gdml`, `field_map`/`mixed`; test 8; the hot
   cell sample and the Geant4 comparison (test 9).
7. Viewer: the field, the Environment section, the overlay; UI spec first; screenshots.
8. GPU slice kernel, `docs/physics/radiation.md`, a tutorial (a dose survey with an AMR and a
   dosimeter, `tools/ci/tutorial_radiation.sh`), an example package
   (`lagrange_example_radiation_survey`), `docs/design/domains/radiation.md` updated to as built.

## Open questions (with a proposed answer)

1. *One `RadiationAPI_1` with a role (formats.md §3.6) or three schemas?* Three: source
   (multiple apply), material (resolved like the other materials), susceptibility (like EMI's).
   §3.6 never shipped, so nothing migrates.
2. *Which medium is a victim's dose in?* `silicon` by default (electronics are the common
   victim; it is a choice of quantity, not a device number), `air` for air kerma, `water` and
   `tissue_soft` on request; dosimeters by their `quantity`.
3. *Dose to people: effective dose or ambient dose equivalent, and from which coefficients?*
   Ambient dose equivalent H*(10), what area monitors and survey meters report. The
   air-kerma-to-H*(10) coefficients are ICRP Publication 74's (the current fluence-to-dose set is
   ICRP Publication 116, 2012); both are copyrighted publications, so: no table ships until their
   reuse terms are checked (TODO(verify)); until then `rad/human/*` and the `ambient_dose_equivalent`
   dosimeter need a user-given conversion table (`radiation.h10_per_air_kerma`), and are off
   without one.
4. *Build-up through several materials?* The build-up of the material with the most mean free
   paths along the ray, documented as an approximation [Low confidence that this is the best
   simple rule]; multilayer formulas (Broder's and later fits) are a later addition if the Monte
   Carlo comparison shows the need.
5. *Where may the G-P coefficients come from?* ANSI/ANS-6.4.3 is a sold standard whose
   reproduction the ANS restricts, and DLC-129 comes through RSICC. Proposed: ask RSICC and the
   ANS about redistribution (the owner); meanwhile ship `buildup: none` as the default and let a
   user point `radiation.buildup_table` at their own copy of the coefficients. TODO(verify) whether
   open publications reproduce enough coefficients.
6. *Interpolating μ/ρ across absorption edges?* Log-log linear between table rows, with the
   edges' duplicate rows kept as the table gives them, so no interpolation crosses an edge. The
   K edge of lead (88.0 keV in the SRD 126 table) matters only for low-energy sources such as Am-241's line.
7. *Prims without a radiation material?* They do not shield, and the start logs how many
   colliders have none (a robot in a concrete room with untagged walls would otherwise see
   through them silently). A `lagrange check` warning when a scene has sources and no materials.
8. *Hollow objects (a lead pot, a cask)?* Collision shapes are closed and convex, so a hollow
   pot's hull is solid lead. Proposed: the user authors such a shield as convex parts (walls and a
   lid; `collision_bake` splits concave meshes) and the docs show how; a thin-shell attribute is
   not added.
9. *Radioactive decay during a run?* Off: runs are minutes to hours. A `rad:source:<n>:halfLife`
   could be added later as an optional attribute.
10. *Step size?* 100 ms by default (`radiation.dt`): a robot at 1 m/s moves 10 cm per step,
    finer than any practical survey. The module warns when a victim's dose rate changes by more
    than 20 % between steps, as the design doc proposed.
11. *Spectrometers, directional detectors, gamma cameras?* A later RFC: they need energy-resolved
    transport and a detector response, which the point kernel's single energy per line can
    approximate only for uncollided photons.
12. *A published benchmark?* Propose that the owner request SINBAD access through the NEA Data
    Bank and check whether a gamma shielding experiment there fits (a point source behind a
    slab); until then validation is the closed forms and the Geant4 comparison (tests 1 to 9).
13. *The segment query in the plugin ABI?* Not in this RFC: it stays internal. A plugin that
    needs path lengths gets `scene_rays` today; a `scene_segments` call would be a 1.x MINOR
    append in its own RFC.
14. *Neutron point kernels?* No: neutron transport through shields is not well described by a
    single attenuation coefficient, so neutrons come only as maps and as particle flux.

## References checked for this draft

- NIST SRD 126 (Hubbell and Seltzer), doi:10.18434/T4D01F; the lead row quoted in §2 was read
  from its lead table on 2026-11-15, as was the K edge (8.80045e-2 MeV) of open question 6.
- NIST SRD 8, XCOM (Berger, Hubbell, Seltzer, Chang, Coursey, Sukumar, Zucker, Olsen),
  doi:10.18434/T48G6X.
- ANSI/ANS-6.4.3-1991 (approved 1991-08-26) and RSICC DLC-129 (ANS643): their scope and the
  G-P formula's adoption; the coefficients themselves were not read.
- SINBAD: OECD NEA and RSICC; version 2's distribution is restricted.
- ICRP Publication 116 (2012): scope only; no coefficient was read.
- Khanam et al., IEEE Sensors Journal, 2022: abstract only.
- JEDEC JESD89 family: scope only.

Everything else marked TODO(verify) above is to be read from its source before the step that
uses it.
