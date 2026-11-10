# RFC 0012: Heat conduction in solids, from materials

| | |
|---|---|
| Status | Accepted (2026-11-10) by the owner, with the proposed answer to each open question; gated merges of its steps pre-approved |
| Created | 2026-11-10 |
| Affects | `schemas/` (one new library), `modules/thermal/` (a new fidelity), `io/stage/`, `server/` (URDF reading), `profiles/schema.json` (addition), `proto/` (one message, additions), `viewer/` (drawing), `tools/cli/` (an OpenFOAM case writer), `docs/` |
| Builds on | RFC 0001 (modules), RFC 0002 (schemas), RFC 0003 (protocol), RFC 0005 (profiles), RFC 0007 (the 1.x freeze) |

## Summary

Temperatures inside the parts of a robot, not one per part: each link is given a material
(conductivity, density, specific heat, emissivity, by a preset or by its numbers) and heat
sources (an LED, a driver chip, a motor's losses), and the thermal domain works out how the heat
spreads through it, crosses from part to part and leaves by convection and radiation. The
viewer colours every surface by its temperature. The materials are written where the robot is,
in the URDF (`<gazebo reference>`), the overlay or USD, so a user changes a part from aluminium
to PLA and runs again. A case writer gives the same robot to OpenFOAM's `chtMultiRegionFoam`
(conjugate heat transfer) when the air's flow around the parts matters. Every change is an
addition under RFC 0007.

## Motivation

The thermal domain today is a lumped network: one temperature per `ThermalAPI_1` prim, with
conductances the user works out by hand. That answers "how hot does the motor get", not "where
on this board is the hot spot", "does the heat reach the plastic housing", or "is an aluminium
bracket enough for a 3 W LED". Those need the part's shape and material, which Lagrange already
has the shape of (its meshes) but no thermal material for: `SensorMaterialAPI_1` has an
emissivity and an electrical conductivity only.

Illuminators (visible or near-infrared LEDs on a camera or a drone), motor drivers, batteries in
a closed housing and processors on a heat sink are the common cases. In each, most of the input
power becomes heat in a few square millimetres, and what matters is how fast the material around
it spreads it.

## Non-goals

- The air's flow. Convection is a film coefficient at each surface (natural, or forced by the
  link's airspeed); the flow itself is OpenFOAM's job (the case writer below). No fluid cells in
  Lagrange.
- Light. An LED's radiant power leaves the part (its efficiency); where the light goes, and what
  it heats elsewhere, is left out. Optical simulation of an illuminator is a later RFC.
- Thermal stress, expansion, phase change (melting, solder reflow), and ablation.
- Junction models of semiconductors beyond a junction-to-case resistance (the open questions).

## Design

### 1. Data: two schemas (added to `schemas/thermal/`)

`ThermalMaterialAPI_1` (single apply) on a link's prim, or on a `UsdShade` material bound to it
(resolved like `SensorMaterialAPI_1`, by prim, then ancestor, then bound material):

| Attribute | Unit | Meaning |
|---|---|---|
| `thermal:material:preset` | token | `aluminium_6061`, `aluminium_cast`, `copper`, `steel_304`, `fr4`, `pla`, `abs`, `petg`, `nylon_pa12`, `polycarbonate`, `cfrp_epoxy`, `glass`, `silicone`, `air_gap`; its numbers come from a cited table (below) |
| `thermal:material:conductivity` | W/(m·K) | `k`, overrides the preset |
| `thermal:material:density` | kg/m³ | `ρ` |
| `thermal:material:specificHeat` | J/(kg·K) | `c_p` |
| `thermal:material:emissivity` | 1 | for radiation from its surfaces |
| `thermal:material:conductivityAlong` | W/(m·K) | optional: `k` along `axis` when it differs (a PCB's copper planes, carbon fibre) |
| `thermal:material:axis` | 1 (vec3, link frame) | the direction `conductivityAlong` applies in; `conductivity` is across it |
| `thermal:material:initialTemperature` | K | optional; else the site's ambient |
| `thermal:material:maxTemperature` | K | optional; `overtemp` flag past it (PLA softens near 333 K) |

`HeatSourceAPI_1` (multiple apply, `thermal:source:<name>`) on a link's prim:

| Attribute | Unit | Meaning |
|---|---|---|
| `thermal:source:<n>:power` | W | electrical input power, constant |
| `thermal:source:<n>:powerSignal` | rel/token | or a signal in W (a motor's `loss_power`, an LED driver's output), read each step |
| `thermal:source:<n>:efficiency` | 1 | the share that leaves as light (0 for a resistor or chip); the heat is `(1 − η)·P` |
| `thermal:source:<n>:position` | m (vec3, link frame) | centre of the heated volume |
| `thermal:source:<n>:size` | m (vec3) | full size of the heated box; the heat is shared over the cells inside it |

Presets are a table in `modules/thermal/` with a source per row (Incropera et al., *Fundamentals
of Heat and Mass Transfer*, 7th ed., Appendix A, for metals and glass; the makers' data sheets for
FR-4 and the printing plastics, cited per row). A preset without a source is not added.

Contact between two links (screwed, glued, a thermal pad) is `thermal:material:contactConductance`
(W/(m²·K)) on the child link, used where its cells touch its parent's. No default: without it the
two parts do not exchange heat by conduction, and the start logs which touching pairs have none.

### 2. In URDF and the overlay

```xml
<gazebo reference="led_board">
  <thermal_material preset="fr4" conductivity_along="20" axis="0 0 1" contact_conductance="3000"/>
  <heat_source name="led_1" power="3.0" efficiency="0.35" xyz="0.01 0 0.0016" size="0.003 0.003 0.001"/>
  <heat_source name="driver" power="0.4" xyz="-0.01 0 0.0016" size="0.004 0.004 0.001"/>
</gazebo>
<gazebo reference="heat_sink">
  <thermal_material preset="aluminium_6061" emissivity="0.85"/>  <!-- anodised -->
</gazebo>
```

Read in `addSensors` beside `<sensor_material>`, into the two schemas, so `lagrange_import` writes
them to USD and the server reads both the same way. The overlay YAML takes the same keys.

### 3. Model: fidelity `conduction` of the thermal domain

`thermal.fidelity: conduction` in the profile (with `thermal.cell`, the cell size in m, default
the smallest of 2 mm and a tenth of the thinnest part, and `thermal.geometry: visual|collision`,
default `visual`). The `lumped` network keeps working beside it, unchanged.

- **Cells.** Each link with a thermal material is cut into cubic cells in its own frame from its
  visual meshes (closed or nearly closed; inside by the generalised winding number, so small
  holes do not leak) or its collision shapes. Links move; their cells move with them.
- **Conduction.** Finite volumes, one temperature per cell; between neighbours the harmonic mean
  of their `k` (orthotropic `k` along its axis); between two links' touching cells the contact
  conductance. Backward Euler, the linear system solved by conjugate gradients (it is symmetric
  positive definite), warm started from the last step. Unconditionally stable, so the domain's
  100 ms step stays.
- **Surfaces.** Each outside face loses `h·A·(T − T_air)` and `ε·σ·A·(T⁴ − T_sky⁴)` (linearised
  around the current temperature, as in `lumped`), with `h` from the natural convection
  correlation for its orientation (Churchill and Chu for vertical faces; McAdams for horizontal
  ones, hot side up and down), forced by `body/<b>/airspeed` when the aerodynamics domain gives
  it (flat plate, Incropera §7.2), and the larger of the two. Solar load on the faces that see the
  sun when `env/solar_irradiance` is given.
- **Sources.** Each heat source's `(1 − η)·P` is shared over the cells inside its box; a
  `loss_power` signal on a link's prim heats the link's cells evenly (a motor's windings).
- **Determinism.** The CPU path is the reference (invariant 7): fixed cell order, fixed iteration
  count to a fixed tolerance. A Slang GPU path is later and behind the same interface.

Signals per link `L` and source `S`:

| Signal | Unit | |
|---|---|---|
| `thermal/L/temperature` | K | volume average |
| `thermal/L/max_temperature`, `min_temperature` | K | |
| `thermal/L/heat_flow_out` | W | to the air, by convection and radiation |
| `thermal/L/overtemp` | bool | past `maxTemperature` |
| `thermal/S/temperature` | K | the average over its box (the case temperature) |
| `thermal/S/heat` | W | `(1 − η)·P` |
| `thermal/S/radiant_power` | W | `η·P`, the light that left |

Tiers: CPU on every tier now; GPU later.

Substitute: `ambient` (constant, as today).

### 4. Viewer

A new message `BodyTemperature` (link, cell size, origin and dimensions in the link frame, the
cell temperatures as f32), sent at the thermal step to a viewer that subscribed to it, with the
shared-memory path when local. The viewport's existing *Thermal* shading then colours each
vertex by the cell under it (trilinear) instead of one colour per prim, with the legend's range
from the run; the inspector shows a link's min, mean and max, and each source's temperature. A
section plane (the existing field slice tool) cuts a link to show the inside.

### 5. OpenFOAM: `lagrange foam-cht`

`lagrange foam-cht ROBOT [--overlay ...] [--ambient K] [--speed M_S] [--time S] [--steady]
[--out DIR] [--container]` writes a `chtMultiRegionFoam` case from the same data:

- one solid region per link with a material (its surface from the same meshes, `snappyHexMesh`
  per region), `thermophysicalProperties` from its `k`, `ρ`, `c_p`; one fluid region of air around
  them (still, or a wind of `--speed`), with buoyancy;
- each heat source as an `fvOptions` `scalarSemiImplicitSource` on the cells of its box in the
  region of its link;
- each solid-solid and solid-fluid interface as
  `compressible::turbulentTemperatureCoupledBaffleMixed`, a contact conductance as its
  `thicknessLayers`/`kappaLayers` pair (a layer whose `κ/t` equals it; TODO(verify) the keyword
  names in OpenFOAM v2512);
- radiation by `fvDOM` when asked.

It runs with an installed OpenFOAM or the container, reads each region's min, mean and max and
each source box's mean back into `report.json`, and `--compare RUN.json` sets them beside the
`conduction` model's. OpenFOAM stays a separate program, files only (invariant 12).

## Validation

Each against a closed-form solution or a published reference, in `test_conduction`:

1. A semi-infinite solid with a step in surface temperature: `erfc` (Carslaw and Jaeger, *Conduction of Heat in Solids*, ch. 2).
2. A rod with a fixed base temperature and a convecting tip: the fin solution with tip
   convection (Incropera, Table 3.4, case A); the tip temperature within 1 %.
3. A two-layer wall with a contact conductance: the series resistance in steady state.
4. Lumped limit: a small copper cube (Biot ≪ 0.1) cooling like `exp(−hAt/ρcV)`.
5. Grid convergence: the fin at three cell sizes, second order.
6. A sample LED board on an aluminium heat sink (`assets/robots/led_heatsink/`), `conduction`
   against `lagrange foam-cht` with still air: the hot spot and the sink's mean within a
   tolerance written down after the first comparison, kept as a regression (`docs/validation/`).

## Steps (each a PR)

1. This RFC.
2. Schemas `ThermalMaterialAPI_1`, `HeatSourceAPI_1`, frozen-schema test additions, formats.md.
3. URDF and overlay reading, `lagrange_import` writing them; the preset table with sources.
4. The `conduction` model with tests 1–5, signals, profile keys.
5. Viewer: `BodyTemperature`, per-vertex thermal shading, inspector.
6. `lagrange foam-cht`, the sample, test 6, docs (`docs/physics/heat-conduction.md`, a tutorial).

## Open questions (with a proposed answer)

1. *Cells from visual or collision geometry?* Visual by default: heat follows the real shape,
   and the collision shapes are convex hulls that fill in hollows. Collision on request.
2. *A default contact conductance?* No: it ranges from about 100 W/(m²·K) (dry, light pressure)
   to over 10⁴ (a thermal pad, solder), so a default would be a silent guess. Touching parts
   without one are logged.
3. *Junction temperature?* Not in the first version: a source's temperature is its case. A later
   addition may give a source `thermal:source:<n>:junctionToCase` (K/W), giving
   `T_j = T_case + R_jc·(1 − η)·P`, the number LED data sheets give.
4. *Efficiency that falls with temperature (LED droop)?* Later, as an optional curve.
5. *Cell count?* A 2 mm cell over a 0.3 m frame is about 10⁵ cells per robot at a 100 ms step;
   conjugate gradients on that is milliseconds on a CPU. Larger robots set a larger cell, or
   give a material only to the parts that matter.
