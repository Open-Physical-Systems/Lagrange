# RFC 0002: USD custom schemas

| | |
|---|---|
| Status | Draft |
| Created | 2026-10-13 |
| Affects | `schemas/`, `core/stage/`, importers, every domain module, the viewer inspector |
| Detail | `docs/design/formats.md` §2–§4, §10 |

## Summary

Define the conventions, the first set of applied API schemas, and the versioning and migration
policy for the project's OpenUSD schemas.

## Motivation

USD is the single authoring format for robots and worlds (CLAUDE.md §4). Electrical topology,
thermal nodes, aerodynamic coefficients, interference sources and sensor-relevant material
properties have no standard USD schema, so the project must define them. These schemas are a
long-lived public surface: user assets will be written against them. They need consistent rules
before the first one is implemented, and a migration path from the start (invariant 11).

## Design

### Conventions

1. Applied API schemas only; no new prim types.
2. One namespace per library: `elec:`, `thermal:`, `aero:`, `weather:`, `emi:`, `rad:`,
   `sensmat:`, `sensor:`, `world:`.
3. SI units for every attribute, recorded in the registered metadata field `lagrangeUnit` on
   the attribute definition and parsed by the loader into `lagrange::Unit`. A schema attribute
   without a unit entry fails a repository test. (Changed from `customData["lagrange:unit"]`
   after the OpenUSD spike: `usdGenSchema` deletes property customData and the schema registry
   refuses it; a field registered through `SdfMetadata` in `plugInfo.json` survives generation
   and resolves on every attribute. Evidence: `docs/plan/spikes/0.2-B1-openusd.md`.)
4. Physical parameters are `double`.
5. Site-dependent quantities (mains frequency, ambient temperature, Earth magnetic field, water
   density) have no fallback value. Missing values are load errors.
6. Topology (ports, connections, thermal links, driven joints, power ports) is expressed with
   relationships on the component prims, so the 3D and schematic views cannot disagree.
7. Schemas are codeless (`skipCodeGeneration = true`). `core/stage/` reads attributes by name.
8. Family names are versioned with OpenUSD's convention `<Family>_<N>`.
9. Unit strings use a small grammar: symbols from a fixed table (`m`, `kg`, `s`, `A`, `K`, `mol`,
   `cd`, `rad`, `N`, `J`, `W`, `V`, `ohm`, `F`, `H`, `C`, `T`, `Pa`, `Hz`, `Gy`, `Bq`, `S`, `1`),
   `.` for product, `/` for quotient, integer exponents as suffix digits (`m2`), parentheses.
   Example: `W/(m2.K)`.

### Schema set

| Library | Schemas | First release |
|---|---|---|
| `schemas/electrical` | `PortAPI_1` (multiple-apply), `BatteryAPI_1`, `ElectricalMotorAPI_1`, **`ElectricalBusAPI_1`**, **`ElectricalLoadAPI_1`**, **`ElectricalConverterAPI_1`** | v0.2 (definitions), v0.3 (used) |
| `schemas/thermal` | `ThermalAPI_1`, **`ThermalLinkAPI_1`** (multiple-apply) | v0.3 |
| `schemas/aero` | `AeroAPI_1`, **`RotorAPI_1`** | v0.4 |
| `schemas/weather` | **`WeatherAPI_1`** | v0.3 (state), v0.5 (full) |
| `schemas/emi` | `EmiSourceAPI_1`, `EmiSusceptibilityAPI_1` | v0.5 |
| `schemas/radiation` | `RadiationAPI_1` | with the radiation domain |
| `schemas/sensor_material` | `SensorMaterialAPI_1` | v0.2 |
| `schemas/sensor` | **`SensorAPI_1`**, **`LidarAPI_1`**, **`CameraSensorAPI_1`**, **`RadarAPI_1`**, **`ImuAPI_1`**, **`GnssAPI_1`**, **`MagnetometerAPI_1`** | v0.2 onward |
| `schemas/world` | **`WorldTileAPI_1`** | v0.7 |
| later | **`VehicleAPI_1`**, **`RadioLinkAPI_1`**, **`HydroAPI_1`** | v0.6, v0.5, post-1.0 |

Bold names are additions to the list in CLAUDE.md §11.3. Draft attribute sets with types, units
and fallbacks for every schema in the first eight libraries are in `formats.md` §3, and a complete
sample robot in §4. Those drafts are the starting point; each schema is finalised in the release
that implements its domain.

Standard schemas are used wherever they exist: `UsdPhysics` for bodies, joints, colliders,
materials and articulation roots; `UsdGeomCamera` for camera intrinsics; `UsdShade` with
`UsdPreviewSurface` for render materials; `UsdLux` for lights.

### Loading

`core/stage/` converts each applied schema instance to a row in `SchemaInstances` with values in
typed pools (invariant 5). It validates: required attributes present, array lengths consistent
(`ocvSoc` and `ocvVoltage`), relationship targets exist and have the expected schema, tokens
within `allowedTokens`, values within stated ranges (0…1 for fractions). All problems are
collected and reported together with prim paths.

### Versioning and migration

- Within a version: only additive changes with fallbacks.
- A rename, unit change, removal, or changed fallback creates `<Family>_<N+1>`.
- The loader accepts the current and previous version, with a warning for the previous one.
- `tools/migrate_schema` holds one function per (family, from, to) with a round-trip test.
- Until product v1.0 every schema is version 1 and may change without migration; this is stated
  in the docs. The migration machinery is built and exercised in v0.3 with a synthetic case.
- Changes need this RFC process or an explicit PR note, a `schemas/CHANGELOG.md` entry, and two
  approvals including a core maintainer (CLAUDE.md §9).

## Alternatives considered

| Alternative | Why not |
|---|---|
| Typed (concrete) prim schemas such as `ElectricalMotor` | Would force components to be separate prims from their geometry and joints; applied schemas let one prim be a body, a motor and a thermal node |
| Generated C++ classes via `usdGenSchema` | Adds a build step depending on USD's Python tooling; the runtime never touches USD in `step()`, so typed accessors bring little |
| `_v1` suffix as written in CLAUDE.md | Not USD's versioning convention; USD tools would treat it as an unrelated name |
| Units as a comment or documentation only | Cannot be checked; the FMU mapping and the inspector need machine-readable units |
| Separate topology file (netlist) next to the USD | Two sources of truth; the schematic and 3D views could disagree |
| Reusing Isaac Sim / Omniverse extension schemas | They cover physics and sensors for that product, not electrical/thermal/EMI; depending on them ties assets to another vendor's versioning. Import mapping from common ones can be added later |
| MJCF or SDF extensions as the source of truth | Neither has layering, references, payloads or a schema mechanism comparable to USD; both remain import formats |
| `float` attributes as in the reference example | Precision and silent conversions; `double` costs nothing at authoring scale |

## Unresolved questions

1. **Electrical bus, load and converter schemas** are additions. Are they accepted, and is
   `ElectricalBusAPI` a schema on a dedicated prim (proposed) or should nets be implicit from
   port connections only?
2. **Motor constant conventions.** Proposed: `torqueConstant` in N·m per peak q-axis ampere,
   `backEmfConstant` in V·s/rad line-neutral peak per mechanical rad/s. Needs review by someone
   with drive-design experience, and a datasheet conversion guide.
3. **One `RadiationAPI` with a role token** versus separate source, victim and shield schemas.
4. **Codeless schemas** (open question Q7): confirm.
5. **Schema suffix** (open question F3): `<Family>_<N>` is what OpenUSD v26.08 parses as
   family and version (verified in the OpenUSD spike); `_v1` is not. Recommend `_N`.
6. **Relationship of `SensorAPI` to existing sensor schemas** in other ecosystems (Isaac Sim's
   sensor schemas, SDFormat sensor elements): importer mappings only, or aliases?
7. **MaterialX**: when to read `UsdMtlx` materials directly.
8. **Where calibration data lives.** Per-robot EMI calibration parameters are authored on prims.
   Should a robot instance be able to override them from a separate calibration layer? Proposed:
   yes, as an ordinary USD sublayer; no schema change needed.
9. **Validation tool.** A `lagrange check <stage>` command that reports schema and unit problems
   without starting a simulation. Proposed for v0.2.
