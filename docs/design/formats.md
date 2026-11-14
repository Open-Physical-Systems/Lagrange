# Formats

Status: draft for review. Schema details are decided in `rfcs/0002-usd-schemas.md`; the profile
format in `rfcs/0005-sim-profile-format.md`. This document expands CLAUDE.md §4, §11.1 and §11.3.

## 1. One home per kind of data

| Data | Format | Written by | Read by | Notes |
|---|---|---|---|---|
| Robot, world, component placement | OpenUSD (`.usda` in git, `.usdc` in bundles) | importers, users, viewer (session layer) | `core/stage/` | Never read in `step()` |
| Rigid bodies, joints, colliders | UsdPhysics schemas | importers | `core/stage/` | Standard schema, not ours |
| Electrical topology, thermal, aero, EMI, radiation, sensor materials | Custom applied API schemas (`schemas/`) | importers, users | `core/stage/` | Versioned (section 3) |
| Component behaviour | FMU, FMI 3.0 co-simulation | `models/` (Modelica) or vendors | `modules/electrical/fmu_host/` | Referenced by `asset` attributes |
| Volumetric fields | OpenVDB on disk, NanoVDB at runtime | offline tools (`io/fields/`) | field-sampling kernels | Referenced by `asset` attributes; unit in metadata |
| Simulation profile | YAML, validated by `profiles/schema.json` | users, viewer domain panel | server, mock server | Section 5 |
| Recordings | MCAP, Protobuf-encoded messages | `io/recording/` | replay substitutes, mock server, viewer | Section 7 |
| Mesh import | glTF 2.0, OBJ, STL, DAE | — | `io/importers/` only | Never loaded at runtime |
| Materials | UsdPreviewSurface (baseline); MaterialX (optional, later) | importers | viewer renderer, camera sensor | Section 8 |
| Collision geometry | Convex hulls in USD (`UsdPhysicsMeshCollisionAPI`, approximation `convexDecomposition` baked to explicit hull prims); terrain as height fields (a grid mesh, approximation `none`, with `int lagrange:heightField:columns` and `rows`, points in row order from -x -y; v0.7) | `tools/collision_bake`, `lagrange_build_world` | rigid-body backends, sensors | Never the render mesh |
| UI↔server protocol | Protobuf (proto3) | server, mock server | viewer | `docs/design/protocol.md` |
| Robot description import | URDF/xacro, SDFormat, MJCF | — | `io/importers/` → USD | Section 9 |
| Roads, scenarios | OpenDRIVE, OpenSCENARIO | — | `io/opendrive/`, `io/openscenario/` | v0.6 |
| Geospatial | OGC 3D Tiles, GeoTIFF/DEM, OSM | — | `io/geospatial/` → USD payload tiles | v0.7 |

## 2. Schema conventions

1. **Applied API schemas only.** Our schemas add properties to existing prims (`Xform`, joints,
   `Mesh`, `Material`). No new concrete prim types, so files stay readable in any USD tool.
2. **Namespaces.** `elec:`, `thermal:`, `aero:`, `emi:`, `rad:`, `sensmat:`, `sensor:`. One
   namespace per schema library.
3. **Units.** Every attribute is SI. Every attribute definition carries
   `lagrangeUnit = "<unit>"`, a metadata field registered by the `lagrange_schemas` plugin
   (`SdfMetadata` in its `plugInfo.json`). `core/stage/` parses that string into a
   `lagrange::Unit` (`lagrange::parseUnit`) and exposes it to modules, so the schema is the
   single source of unit truth. `customData` cannot carry it: `usdGenSchema` deletes property
   customData and the schema registry refuses it as a fallback (OpenUSD spike,
   `docs/plan/spikes/0.2-B1-openusd.md`).
4. **Types.** Physical parameters are `double`. The reference example in CLAUDE.md §11.3 uses
   `float`; see open question F4 for the reason to change.
5. **No hidden site constants.** Attributes whose value depends on the site (mains frequency,
   ambient temperature, latitude) have **no fallback value**. A missing value is a load error.
   This implements the CLAUDE.md §10 rule on scenario parameters.
6. **Versioning.** Each schema is a family with an integer version suffix. This document writes
   `ElectricalMotorAPI_1`. CLAUDE.md writes `ElectricalMotorAPI_v1`. OpenUSD v26.08 parses
   `ElectricalMotorAPI_1` as family `ElectricalMotorAPI`, version 1
   (`Usd.SchemaRegistry.ParseSchemaFamilyAndVersionFromIdentifier`, verified in the OpenUSD
   spike); `_v1` is not recognised as a version. Open question F3 recommends the USD convention.
7. **Codeless schemas.** Schemas are registered with `skipCodeGeneration = true`. `core/stage/`
   reads attributes by name into SoA pools, so generated C++ classes are not needed. This removes
   `schemas/generated/` C++ from the build (open question Q7).
8. **Topology lives on the 3D prims.** Electrical ports and connections are relationships on the
   component prims, so the schematic view and the 3D view are two projections of the same data
   (CLAUDE.md §11.3).

Library header used by every `schema.usda` below (shown once):

```usda
#usda 1.0
(
    subLayers = [ @usd/schema.usda@ ]
)

over "GLOBAL" (
    customData = {
        string libraryName = "lagrangeElectrical"        # lagrangeThermal, lagrangeAero, ...
        string libraryPath = "."
        bool skipCodeGeneration = true
    }
)
{
}
```

## 3. Draft schemas

Attribute tables give name, type, unit and fallback. "—" in the fallback column means the
attribute is required when the schema is applied. `.usda` text follows each table.

### 3.1 `schemas/electrical/schema.usda`

**`PortAPI_1`** (multiple-apply, instance name = port name, e.g. `elec:port:dc_in`). An
electrical terminal pair on a component.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `elec:port:<n>:kind` | token | — | `dc` | `dc`, `ac1`, `ac3`, `signal` |
| `elec:port:<n>:nominalVoltage` | double | V | — | Rated voltage, used for sanity checks |
| `elec:port:<n>:maxCurrent` | double | A | — | Continuous rating |
| `elec:port:<n>:connectedTo` | rel | — | — | Target port(s) or bus prim. Defines the net |
| `elec:port:<n>:wireResistance` | double | ohm | `0.0` | Lumped harness resistance to the net |
| `elec:port:<n>:wireInductance` | double | H | `0.0` | Lumped harness inductance |

**`BatteryAPI_1`** (single-apply). Equivalent-circuit model: open-circuit voltage curve, series
resistance, two RC pairs, thermal dependence.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `elec:battery:chemistry` | token | — | `li_ion_nmc` | `li_ion_nmc`, `lfp`, `lead_acid`, `custom` |
| `elec:battery:seriesCells` | int | 1 | — | Cells in series |
| `elec:battery:parallelCells` | int | 1 | `1` | Cells in parallel |
| `elec:battery:capacity` | double | C | — | Rated charge of the pack (1 Ah = 3600 C) |
| `elec:battery:initialSoc` | double | 1 | `1.0` | 0…1 |
| `elec:battery:ocvSoc` | double[] | 1 | — | SoC breakpoints, ascending |
| `elec:battery:ocvVoltage` | double[] | V | — | Cell open-circuit voltage at each breakpoint |
| `elec:battery:r0` | double | ohm | — | Cell series resistance at `referenceTemperature` |
| `elec:battery:r1`, `c1` | double | ohm, F | `0.0` | First RC pair (per cell) |
| `elec:battery:r2`, `c2` | double | ohm, F | `0.0` | Second RC pair (per cell) |
| `elec:battery:referenceTemperature` | double | K | `298.15` | Temperature at which `r0` is given |
| `elec:battery:r0TempCoeff` | double | 1/K | `0.0` | Fractional change of `r0` per kelvin |
| `elec:battery:cutoffVoltage` | double | V | — | Pack under-voltage cut-off |
| `elec:battery:maxChargeCurrent` | double | A | — | Regeneration limit; excess raises bus voltage |
| `elec:battery:behaviorModel` | asset | — | none | Optional FMU that replaces the built-in ECM |

**`ElectricalMotorAPI_1`** (single-apply, on the prim that represents the motor; `elec:joint`
points at the driven joint).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `elec:motorType` | token | — | `pmsm` | `pmsm`, `bldc`, `dc_brushed`, `induction` (later) |
| `elec:windingResistance` | double | ohm | — | Phase (or armature) resistance at `referenceTemperature` |
| `elec:inductance` | double | H | — | d-axis (or armature) inductance |
| `elec:inductanceQ` | double | H | = `inductance` | q-axis inductance (salient machines) |
| `elec:torqueConstant` | double | N·m/A | — | Torque per q-axis ampere (peak) |
| `elec:backEmfConstant` | double | V·s/rad | = `torqueConstant`/1.5 (pmsm, bldc), = `torqueConstant` (dc_brushed) | Line-neutral peak per mechanical rad/s; an authored value must agree within 3 % |
| `elec:polePairs` | int | 1 | `1` | |
| `elec:rotorInertia` | double | kg·m² | `0.0` | Added to the joint's armature inertia |
| `elec:gearRatio` | double | 1 | `1.0` | Motor revolutions per joint revolution |
| `elec:gearEfficiency` | double | 1 | `1.0` | 0…1 |
| `elec:maxCurrent` | double | A | — | Drive current limit (peak) |
| `elec:referenceTemperature` | double | K | `298.15` | |
| `elec:resistanceTempCoeff` | double | 1/K | `0.00393` | Copper, per kelvin (source: standard value for annealed copper at 20 °C; cite in code) |
| `elec:controlMode` | token | — | `torque` | `torque`, `velocity`, `voltage` — what the command signal means |
| `elec:currentLoopBandwidth` | double | rad/s | `6283.0` | FOC current-loop bandwidth (1 kHz) |
| `elec:pwmFrequency` | double | Hz | `20000.0` | Used for ripple estimate and EMI source spectrum |
| `elec:rotorIntegration` | token | — | `mechanical` | `mechanical` or `electrical` (architecture §8) |
| `elec:behaviorModel` | asset | — | none | Optional FMU that replaces the built-in model |
| `elec:joint` | rel | — | — | Driven joint prim |
| `elec:powerPort` | rel | — | — | Port or bus the drive draws from |

```usda
class "PortAPI_1" (
    inherits = </APISchemaBase>
    customData = {
        token apiSchemaType = "multipleApply"
        token propertyNamespacePrefix = "elec:port"
    }
    doc = "An electrical terminal pair on a component. Instance name is the port name."
)
{
    uniform token kind = "dc" ( allowedTokens = ["dc", "ac1", "ac3", "signal"] )
    double nominalVoltage ( lagrangeUnit = "V" )
    double maxCurrent ( lagrangeUnit = "A" )
    rel connectedTo ( doc = "Ports or bus prims on the same net." )
    double wireResistance = 0.0 ( lagrangeUnit = "ohm" )
    double wireInductance = 0.0 ( lagrangeUnit = "H" )
}

class "BatteryAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
    doc = "Battery pack, equivalent-circuit model with two RC pairs."
)
{
    uniform token elec:battery:chemistry = "li_ion_nmc" (
        allowedTokens = ["li_ion_nmc", "lfp", "lead_acid", "custom"] )
    int elec:battery:seriesCells
    int elec:battery:parallelCells = 1
    double elec:battery:capacity ( lagrangeUnit = "C" )
    double elec:battery:initialSoc = 1.0 ( lagrangeUnit = "1" )
    double[] elec:battery:ocvSoc ( lagrangeUnit = "1" )
    double[] elec:battery:ocvVoltage ( lagrangeUnit = "V" )
    double elec:battery:r0 ( lagrangeUnit = "ohm" )
    double elec:battery:r1 = 0.0 ( lagrangeUnit = "ohm" )
    double elec:battery:c1 = 0.0 ( lagrangeUnit = "F" )
    double elec:battery:r2 = 0.0 ( lagrangeUnit = "ohm" )
    double elec:battery:c2 = 0.0 ( lagrangeUnit = "F" )
    double elec:battery:referenceTemperature = 298.15 ( lagrangeUnit = "K" )
    double elec:battery:r0TempCoeff = 0.0 ( lagrangeUnit = "1/K" )
    double elec:battery:cutoffVoltage ( lagrangeUnit = "V" )
    double elec:battery:maxChargeCurrent ( lagrangeUnit = "A" )
    asset elec:battery:behaviorModel
}

class "ElectricalMotorAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
    doc = "Electric motor with its drive (inverter + current control), averaged model."
)
{
    uniform token elec:motorType = "pmsm" ( allowedTokens = ["pmsm", "bldc", "dc_brushed"] )
    double elec:windingResistance ( lagrangeUnit = "ohm" )
    double elec:inductance ( lagrangeUnit = "H" )
    double elec:inductanceQ ( lagrangeUnit = "H" )
    double elec:torqueConstant ( lagrangeUnit = "N.m/A" )
    double elec:backEmfConstant ( lagrangeUnit = "V.s/rad" )
    int elec:polePairs = 1
    double elec:rotorInertia = 0.0 ( lagrangeUnit = "kg.m2" )
    double elec:gearRatio = 1.0 ( lagrangeUnit = "1" )
    double elec:gearEfficiency = 1.0 ( lagrangeUnit = "1" )
    double elec:maxCurrent ( lagrangeUnit = "A" )
    double elec:referenceTemperature = 298.15 ( lagrangeUnit = "K" )
    double elec:resistanceTempCoeff = 0.00393 ( lagrangeUnit = "1/K" )
    uniform token elec:controlMode = "torque" ( allowedTokens = ["torque", "velocity", "voltage"] )
    double elec:currentLoopBandwidth = 6283.0 ( lagrangeUnit = "rad/s" )
    double elec:pwmFrequency = 20000.0 ( lagrangeUnit = "Hz" )
    uniform token elec:rotorIntegration = "mechanical" ( allowedTokens = ["mechanical", "electrical"] )
    asset elec:behaviorModel
    rel elec:joint
    rel elec:powerPort
}
```

Proposed additions, not in the CLAUDE.md list, needed to describe a real DC bus (RFC 0002,
unresolved question 1):

| Schema | Purpose | Key attributes |
|---|---|---|
| `ElectricalBusAPI_1` | A named net with capacitance; brownout thresholds | `elec:bus:capacitance` (F), `elec:bus:undervoltageTrip` (V), `elec:bus:overvoltageTrip` (V), `elec:bus:brakeResistor` (ohm, 0 = none) |
| `ElectricalLoadAPI_1` | Constant-power or resistive load (computer, lidar, heater) | `elec:load:model` (`constant_power`, `resistive`, `signal`), `elec:load:power` (W), `elec:load:minVoltage` (V, fallback 0), `elec:powerPort` |
| `ElectricalConverterAPI_1` | DC-DC converter between two ports | `elec:conv:outputVoltage` (V), `elec:conv:efficiency` (1), `elec:conv:maxOutputCurrent` (A), `elec:conv:input` and `elec:conv:output` (relationships to buses or ports) |

Implemented in `schemas/electrical/schema.usda` (v0.3); `schemas/CHANGELOG.md` lists the
differences from these drafts.

### 3.2 `schemas/thermal/schema.usda`

**`ThermalAPI_1`** (single-apply): one lumped thermal node.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `thermal:heatCapacity` | double | J/K | — | Lumped capacity of the node |
| `thermal:initialTemperature` | double | K | none | If absent, the profile's ambient temperature |
| `thermal:convectionArea` | double | m² | `0.0` | Area exchanging with air |
| `thermal:convectionCoeff` | double | W/(m²·K) | `10.0` | Still-air natural convection; forced convection scales with airspeed |
| `thermal:forcedConvectionExponent` | double | 1 | `0.8` | `h = h0·(1 + v/v_ref)^n` |
| `thermal:emissivity` | double | 1 | `0.9` | For radiative exchange with sky/ambient |
| `thermal:radiativeArea` | double | m² | `0.0` | |
| `thermal:solarAbsorptivity` | double | 1 | `0.7` | Fraction of irradiance absorbed |
| `thermal:solarArea` | double | m² | `0.0` | Projected area for irradiance |
| `thermal:maxTemperature` | double | K | none | Fault / shutdown threshold |
| `thermal:deratingStart` | double | K | none | Above this, the owning component derates linearly to zero at `maxTemperature` |

**`ThermalLinkAPI_1`** (multiple-apply, proposed addition): a conductance to another node.

| Attribute | Type | Unit | Fallback |
|---|---|---|---|
| `thermal:link:<n>:conductance` | double | W/K | — |
| `thermal:link:<n>:target` | rel | — | — |

```usda
class "ThermalAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
    doc = "One lumped thermal node."
)
{
    double thermal:heatCapacity ( lagrangeUnit = "J/K" )
    double thermal:initialTemperature ( lagrangeUnit = "K" )
    double thermal:convectionArea = 0.0 ( lagrangeUnit = "m2" )
    double thermal:convectionCoeff = 10.0 ( lagrangeUnit = "W/(m2.K)" )
    double thermal:forcedConvectionExponent = 0.8 ( lagrangeUnit = "1" )
    double thermal:emissivity = 0.9 ( lagrangeUnit = "1" )
    double thermal:radiativeArea = 0.0 ( lagrangeUnit = "m2" )
    double thermal:solarAbsorptivity = 0.7 ( lagrangeUnit = "1" )
    double thermal:solarArea = 0.0 ( lagrangeUnit = "m2" )
    double thermal:maxTemperature ( lagrangeUnit = "K" )
    double thermal:deratingStart ( lagrangeUnit = "K" )
}

class "ThermalLinkAPI_1" (
    inherits = </APISchemaBase>
    customData = {
        token apiSchemaType = "multipleApply"
        token propertyNamespacePrefix = "thermal:link"
    }
)
{
    double conductance ( lagrangeUnit = "W/K" )
    rel target
}
```

### 3.3 `schemas/aero/schema.usda`

**`AeroAPI_1`** (single-apply, on a rigid-body link).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `aero:model` | token | — | `drag_box` | `drag_box`, `drag_lift_table`, `none` |
| `aero:referenceArea` | double[3] | m² | — | Projected areas along body x, y, z |
| `aero:dragCoeff` | double[3] | 1 | `(1.0, 1.0, 1.0)` | Per-axis drag coefficient |
| `aero:centerOfPressure` | double[3] | m | `(0, 0, 0)` | In body frame |
| `aero:liftSlope` | double | 1/rad | `0.0` | For `drag_lift_table` without a table |
| `aero:coeffTable` | asset | — | none | CSV: angle of attack (rad), CL, CD, CM |
| `aero:angularDamping` | double[3] | N·m·s/rad | `(0, 0, 0)` | |

**`RotorAPI_1`** (on the rotor joint prim; used by `modules/rotor_aero/`). Defined in
`schemas/aero` since v0.4, with `aero:rotor:direction` meaning the sense in which the motor
turns the rotor, seen from the front (from where the thrust points). With a propeller file the
propeller's own hand decides what that rotation does: a propeller of the other hand turns the
wrong way and gives little thrust, as on a real vehicle.

| Attribute | Type | Unit | Fallback |
|---|---|---|---|
| `aero:rotor:radius` | double | m | — |
| `aero:rotor:bladeCount` | int | 1 | — |
| `aero:rotor:thrustCoeff` | double | 1 | — (`T = CT·ρ·n²·D⁴`, `n` in rev/s) |
| `aero:rotor:powerCoeff` | double | 1 | — |
| `aero:rotor:direction` | token | — | — (`cw`, `ccw`) |
| `aero:rotor:propeller` | asset | — | none (propeller file, §3.3.1; required for the `bem` fidelity) |
| `aero:rotor:mount` | token | — | `puller` (`puller`: thrust along the joint axis, which points from the motor to the propeller; `pusher`: against it) |

```usda
class "AeroAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    uniform token aero:model = "drag_box" ( allowedTokens = ["drag_box", "drag_lift_table", "none"] )
    double3 aero:referenceArea ( lagrangeUnit = "m2" )
    double3 aero:dragCoeff = (1.0, 1.0, 1.0) ( lagrangeUnit = "1" )
    double3 aero:centerOfPressure = (0, 0, 0) ( lagrangeUnit = "m" )
    double aero:liftSlope = 0.0 ( lagrangeUnit = "1/rad" )
    asset aero:coeffTable
    double3 aero:angularDamping = (0, 0, 0) ( lagrangeUnit = "N.m.s/rad" )
}

class "RotorAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    double aero:rotor:radius ( lagrangeUnit = "m" )
    int aero:rotor:bladeCount
    double aero:rotor:thrustCoeff ( lagrangeUnit = "1" )
    double aero:rotor:powerCoeff ( lagrangeUnit = "1" )
    uniform token aero:rotor:direction ( allowedTokens = ["cw", "ccw"] )
    asset aero:rotor:propeller
    uniform token aero:rotor:mount = "puller" ( allowedTokens = ["puller", "pusher"] )
}
```

#### 3.3.1 Propeller files

A propeller as built, for the `bem` fidelity of `rotor_aero` and for drawing its blades:
YAML, `*.prop.yaml`, `format: lagrange-propeller-1`. Samples are in `assets/propellers/`.

| Key | Unit | Meaning |
|---|---|---|
| `name` | — | What it is, for people |
| `diameter` | m | Tip diameter |
| `blades` | 1 | Number of blades |
| `hand` | — | `ccw` or `cw`: the sense of rotation, seen from the front (from where its thrust points), in which it makes thrust. A "right-hand" aircraft propeller is `ccw` here (clockwise seen from behind) |
| `aerofoils.<name>` | — | Section data: `lift_slope` (1/rad, two-dimensional), `zero_lift_angle` (deg), `cl_max`, `cl_min`, `cd_min`, `cl_at_cd_min`, `cd_lift` (drag polar `cd = cd_min + cd_lift·(cl − cl_at_cd_min)²`); for drawing only, `camber` and `camber_position` (NACA four-digit mean line, fractions of the chord) |
| `stations[]` | — | Blade stations from root to tip: `r` (r/R), `chord` (c/R), `twist` (deg, the section's reference line against the plane of rotation), `thickness` (t/c, for drawing), `aerofoil` (a name above), and optionally `zero_lift_angle` and `cd_min` for that station |

| `folding` | — | Optional, folding blades: `hinge` (r/R of the hinges), `blade_mass` (kg, one blade outboard of its hinge), `limit` (deg, the hinge's stop either way; fallback 85) |

Every file says where its numbers come from in comments at its top; a propeller made up for a
sample says so.

### 3.4 `schemas/weather/schema.usda`

Defined in v0.5 as drafted, plus `weather:roughnessLength` (m, fallback 0.03, open terrain) for
the logarithmic wind profile that the aerodynamics domain already uses.

Weather is authored on one environment prim (the `environment:` file of a profile).

**`WeatherAPI_1`** (single-apply, proposed; CLAUDE.md lists the `weather/` directory but no
schema name).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `weather:ambientTemperature` | double | K | — | Site parameter, no fallback |
| `weather:pressure` | double | Pa | — | Site parameter |
| `weather:relativeHumidity` | double | 1 | — | 0…1 |
| `weather:meanWind` | double3 | m/s | `(0, 0, 0)` | ENU, at `windReferenceHeight` |
| `weather:windReferenceHeight` | double | m | `10.0` | |
| `weather:turbulenceModel` | token | — | `none` | `none`, `dryden`, `von_karman` |
| `weather:turbulenceIntensity` | double | 1 | `0.0` | σ / mean wind |
| `weather:windField` | asset | — | none | NanoVDB vector grid (m/s), overrides `meanWind` inside its bounds |
| `weather:visibility` | double | m | none | Meteorological optical range; sets fog extinction through Koschmieder (`σ = 3.912 / V`) |
| `weather:fogField` | asset | — | none | NanoVDB scalar grid of extinction (1/m) |
| `weather:rainRate` | double | m/s | `0.0` | Rain intensity (1 mm/h = 2.778e-7 m/s) |
| `weather:dustConcentration` | double | kg/m³ | `0.0` | |
| `weather:dustMedianDiameter` | double | m | `1.0e-5` | |
| `weather:solarIrradiance` | double | W/m² | `0.0` | Direct normal |
| `weather:sunDirection` | double3 | 1 | `(0, 0, 1)` | Unit vector to the sun, ENU |
| `weather:skyTemperature` | double | K | none | For radiative exchange; defaults to ambient |

```usda
class "WeatherAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    double weather:ambientTemperature ( lagrangeUnit = "K" )
    double weather:pressure ( lagrangeUnit = "Pa" )
    double weather:relativeHumidity ( lagrangeUnit = "1" )
    double3 weather:meanWind = (0, 0, 0) ( lagrangeUnit = "m/s" )
    double weather:windReferenceHeight = 10.0 ( lagrangeUnit = "m" )
    uniform token weather:turbulenceModel = "none" ( allowedTokens = ["none", "dryden", "von_karman"] )
    double weather:turbulenceIntensity = 0.0 ( lagrangeUnit = "1" )
    asset weather:windField
    double weather:visibility ( lagrangeUnit = "m" )
    asset weather:fogField
    double weather:rainRate = 0.0 ( lagrangeUnit = "m/s" )
    double weather:dustConcentration = 0.0 ( lagrangeUnit = "kg/m3" )
    double weather:dustMedianDiameter = 0.00001 ( lagrangeUnit = "m" )
    double weather:solarIrradiance = 0.0 ( lagrangeUnit = "W/m2" )
    double3 weather:sunDirection = (0, 0, 1) ( lagrangeUnit = "1" )
    double weather:skyTemperature ( lagrangeUnit = "K" )
}
```

### 3.5 `schemas/emi/schema.usda`

Defined in v0.5, with three changes to the draft below: `emi:direction` (the conductor's
direction or the dipole axis, in the prim's frame) and `emi:dipoleMomentPerAmpere` (m², for
motor drives and welders) added, which the analytic models need; `emi:adcNoisePerField` split
into `emi:accelNoisePerField` ((m/s²)/(V/m)) and `emi:gyroNoisePerField` ((rad/s)/(V/m)), since
the two channels have different units.

**`EmiSourceAPI_1`** (single-apply).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `emi:sourceType` | token | — | — | `power_line`, `motor_drive`, `transmitter`, `welder`, `custom` |
| `emi:frequency` | double | Hz | — | Fundamental. **No fallback** (site parameter: 50 or 60 Hz mains) |
| `emi:current` | double | A | none | RMS current for `power_line`, `welder` |
| `emi:radiatedPower` | double | W | none | For `transmitter` (EIRP) |
| `emi:harmonics` | double[] | 1 | `[]` | Relative amplitude of harmonics 2, 3, … |
| `emi:dutyCycle` | double | 1 | `1.0` | For intermittent sources (welders) |
| `emi:bandwidth` | double | Hz | `0.0` | Occupied bandwidth for `transmitter` |
| `emi:fieldMap` | asset | — | none | NanoVDB vector grid of B (T) or E (V/m), at `emi:current` or `emi:radiatedPower` |
| `emi:fieldMapQuantity` | token | — | `B` | `B` or `E` |
| `emi:drivenBy` | rel | — | none | Signal-producing prim (a motor) whose current scales the source at runtime |
| `emi:path` | double[] | m | `[]` | For `wire` (RFC 0013, added in 1.x): x, y, z of each point of its route, prim frame |
| `emi:returnOffset` | double3 | m | `(0, 0, 0)` | For `wire`: the return conductor, the route shifted by this; zero: none |
| `emi:currentSignal` | token | — | `""` | A signal (A) that sets the current every step, signed for a direct current; over `emi:drivenBy` |

`wire` (RFC 0013) was appended to `emi:sourceType`'s tokens: a conductor along `emi:path`, its
field the Biot–Savart sum over its straight segments.

**`EmiSusceptibilityAPI_1`** (single-apply, on a sensor, bus or radio prim). These are the
per-robot calibration parameters.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `emi:victimType` | token | — | — | `magnetometer`, `imu_adc`, `encoder`, `can_bus`, `radio` |
| `emi:magneticGain` | double | 1 | `1.0` | Magnetometer: fraction of ambient AC field that appears in the reading |
| `emi:adcNoisePerField` | double | V/(V/m) | `0.0` | IMU/ADC: added noise RMS per unit E-field |
| `emi:encoderGlitchField` | double | V/m | none | Field level at which the glitch probability is 50 % per edge |
| `emi:canErrorField` | double | V/m | none | Field level at which the bit-error rate reaches `canErrorRateAtField` |
| `emi:canErrorRateAtField` | double | 1 | `1.0e-5` | |
| `emi:receiverNoiseFigure` | double | 1 | `1.0` | Radio: linear noise figure |
| `emi:couplingLoopArea` | double | m² | `0.0` | Effective loop area of the harness for induced voltage `V = ω·B·A` |
| `emi:shieldingEffectiveness` | double | 1 | `1.0` | Linear field attenuation factor ≥ 1 |

```usda
class "EmiSourceAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    uniform token emi:sourceType (
        allowedTokens = ["power_line", "motor_drive", "transmitter", "welder", "custom"] )
    double emi:frequency ( lagrangeUnit = "Hz"
        doc = "Site parameter. No fallback: 50 Hz and 60 Hz grids both exist." )
    double emi:current ( lagrangeUnit = "A" )
    double emi:radiatedPower ( lagrangeUnit = "W" )
    double[] emi:harmonics = [] ( lagrangeUnit = "1" )
    double emi:dutyCycle = 1.0 ( lagrangeUnit = "1" )
    double emi:bandwidth = 0.0 ( lagrangeUnit = "Hz" )
    asset emi:fieldMap
    uniform token emi:fieldMapQuantity = "B" ( allowedTokens = ["B", "E"] )
    rel emi:drivenBy
}

class "EmiSusceptibilityAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    uniform token emi:victimType (
        allowedTokens = ["magnetometer", "imu_adc", "encoder", "can_bus", "radio"] )
    double emi:magneticGain = 1.0 ( lagrangeUnit = "1" )
    double emi:adcNoisePerField = 0.0 ( lagrangeUnit = "V/(V/m)" )
    double emi:encoderGlitchField ( lagrangeUnit = "V/m" )
    double emi:canErrorField ( lagrangeUnit = "V/m" )
    double emi:canErrorRateAtField = 0.00001 ( lagrangeUnit = "1" )
    double emi:receiverNoiseFigure = 1.0 ( lagrangeUnit = "1" )
    double emi:couplingLoopArea = 0.0 ( lagrangeUnit = "m2" )
    double emi:shieldingEffectiveness = 1.0 ( lagrangeUnit = "1" )
}
```

### 3.6 `schemas/radiation/schema.usda`

**`RadiationAPI_1`** (single-apply). One schema with a `rad:role` token, because the same prim is
rarely both a source and a victim and the attribute sets are small.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `rad:role` | token | — | — | `source`, `victim`, `shield` |
| `rad:sourceType` | token | — | none | `point_gamma`, `field_map` |
| `rad:activity` | double | Bq | none | Point source activity |
| `rad:gammaEnergy` | double | J | none | Mean photon energy (1 MeV = 1.602e-13 J) |
| `rad:doseRateAt1m` | double | Gy/s | none | Alternative to activity + energy |
| `rad:doseField` | asset | — | none | NanoVDB scalar grid of dose rate (Gy/s), e.g. from Geant4 offline |
| `rad:attenuationCoeff` | double | 1/m | none | Shield: linear attenuation at the source energy |
| `rad:buildupFactor` | double | 1 | `1.0` | Shield: simple build-up correction |
| `rad:totalDoseLimit` | double | Gy | none | Victim: failure threshold |
| `rad:seuCrossSection` | double | m² | `0.0` | Victim: single-event-upset cross-section per bit |
| `rad:sensitiveBits` | int | 1 | `0` | Victim: number of bits exposed to SEU |
| `rad:fluenceToDose` | double | Gy·m² | none | Victim: conversion used for SEU rate from dose rate |

```usda
class "RadiationAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    uniform token rad:role ( allowedTokens = ["source", "victim", "shield"] )
    uniform token rad:sourceType ( allowedTokens = ["point_gamma", "field_map"] )
    double rad:activity ( lagrangeUnit = "Bq" )
    double rad:gammaEnergy ( lagrangeUnit = "J" )
    double rad:doseRateAt1m ( lagrangeUnit = "Gy/s" )
    asset rad:doseField
    double rad:attenuationCoeff ( lagrangeUnit = "1/m" )
    double rad:buildupFactor = 1.0 ( lagrangeUnit = "1" )
    double rad:totalDoseLimit ( lagrangeUnit = "Gy" )
    double rad:seuCrossSection = 0.0 ( lagrangeUnit = "m2" )
    int rad:sensitiveBits = 0
    double rad:fluenceToDose ( lagrangeUnit = "Gy.m2" )
}
```

### 3.7 `schemas/sensor_material/schema.usda`

**`SensorMaterialAPI_1`** (single-apply, on a `Material` prim). Non-visual surface properties.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `sensmat:reflectivity905` | double | 1 | `0.5` | Diffuse reflectivity at 905 nm |
| `sensmat:reflectivity1550` | double | 1 | `0.5` | Diffuse reflectivity at 1550 nm |
| `sensmat:retroreflective` | bool | — | `false` | Road signs, safety vests |
| `sensmat:specularFraction905` | double | 1 | `0.0` | Fraction of return that is specular |
| `sensmat:transmission905` | double | 1 | `0.0` | Glass: fraction transmitted |
| `sensmat:radarCrossSectionDensity` | double | 1 | `0.1` | RCS per unit projected area (m²/m²) at 77 GHz |
| `sensmat:radarRoughness` | double | m | `0.001` | RMS surface roughness |
| `sensmat:relativePermittivity` | double | 1 | `1.0` | |
| `sensmat:conductivity` | double | S/m | `0.0` | Electrical conductivity |
| `sensmat:relativePermeability` | double | 1 | `1.0` | Magnetic permeability relative to vacuum |
| `sensmat:thermalEmissivity` | double | 1 | `0.9` | Long-wave IR emissivity (thermal cameras) |
| `sensmat:frictionWetScale` | double | 1 | `0.7` | Multiplier on friction when wet (weather coupling) |

```usda
class "SensorMaterialAPI_1" (
    inherits = </APISchemaBase>
    customData = { token apiSchemaType = "singleApply" }
)
{
    double sensmat:reflectivity905 = 0.5 ( lagrangeUnit = "1" )
    double sensmat:reflectivity1550 = 0.5 ( lagrangeUnit = "1" )
    bool sensmat:retroreflective = false
    double sensmat:specularFraction905 = 0.0 ( lagrangeUnit = "1" )
    double sensmat:transmission905 = 0.0 ( lagrangeUnit = "1" )
    double sensmat:radarCrossSectionDensity = 0.1 ( lagrangeUnit = "1" )
    double sensmat:radarRoughness = 0.001 ( lagrangeUnit = "m" )
    double sensmat:relativePermittivity = 1.0 ( lagrangeUnit = "1" )
    double sensmat:conductivity = 0.0 ( lagrangeUnit = "S/m" )
    double sensmat:relativePermeability = 1.0 ( lagrangeUnit = "1" )
    double sensmat:thermalEmissivity = 0.9 ( lagrangeUnit = "1" )
    double sensmat:frictionWetScale = 0.7 ( lagrangeUnit = "1" )
}
```

The fallback values above are placeholders so that untagged assets still produce returns. They
are not measured data. Realistic values per material class ship in a library file
`assets/materials/sensor_materials.usda`, each entry with its source (risk R12).

### 3.8 `schemas/sensor/schema.usda` (proposed addition)

Sensors need schemas that CLAUDE.md does not list. USD has `UsdGeomCamera` for camera intrinsics;
the rest is ours. Draft attribute sets (full `.usda` text is written in RFC 0002 once the list is
agreed):

| Schema | Attributes (unit, fallback) |
|---|---|
| `SensorAPI_1` (common) | `sensor:rate` (Hz, —), `sensor:latency` (s, 0), `sensor:jitter` (s, 0), `sensor:frameId` (string, prim name), `sensor:topic` (string, derived), `sensor:powerPort` (rel, none) |
| `LidarAPI_1` | `sensor:lidar:wavelength` (m, 9.05e-7), `channels` (int, —), `horizontalSamples` (int, —), `verticalFov` (rad[2], —), `horizontalFov` (rad[2], (−π, π)), `rangeMin`/`rangeMax` (m, —), `rangeNoiseStd` (m, 0.02), `beamDivergence` (rad, 0.003), `returnMode` (token: `strongest`, `last`, `dual`), `scanPattern` (token: `rotating`, `solid_state`, `custom`, and from RFC 0015 `flash`, `rosette`), `rotationRate` (Hz, = rate), `motionDistortion` (bool, true); RFC 0015: `channelElevations` (rad[], []: a channel table), `rosette:fov` (rad), `rosette:pointRate` (1/s), `rosette:speed1`/`speed2` (rad/s), `rosette:prismRatio` (1), the five without fallbacks |
| `CameraSensorAPI_1` | on a `UsdGeomCamera`: `sensor:camera:width`/`height` (int, —), `shutter` (token: `global`, `rolling`), `rollingShutterTime` (s, 0), `exposureTime` (s, —; the longest with auto exposure), `autoExposure` (bool, false), `electronRate` (1/s, —: e⁻ per second at the renderer's full white), `vignetting` (1, 0: fraction of cos⁴θ), `gain` (1, 1), `readNoiseStd` (electrons, 2), `fullWellCapacity` (electrons, 10000), `bitDepth` (int, 8), `distortionModel` (token: `none`, `plumb_bob`, `equidistant`), `distortionCoeffs` (double[], []); RFC 0015: `lensType` (token: `gnomonical`, `equidistant`), `cutoffAngle` (rad, —) |
| `RadarAPI_1` | `sensor:radar:carrierFrequency` (Hz, 7.7e10), `rangeMax` (m, —), `azimuthFov`/`elevationFov` (rad, —), `azimuthResolution` (rad, 0.035), `rangeResolution` (m, 0.2), `velocityResolution` (m/s, 0.1), `txPower` (W, —), `antennaGain` (linear, 1), `noiseFloor` (W, —: effective, after processing gain), `detectionThreshold` (linear SNR, 10) |
| `ImuAPI_1` | v0.2 implements per-sample white noise, `sensor:imu:accelNoiseStd` (m/s², 0) and `gyroNoiseStd` (rad/s, 0), plus the bias instabilities: densities need half powers in the unit grammar. Draft: `sensor:imu:accelNoiseDensity` (m/s²/√Hz, —), `accelBiasInstability` (m/s², —), `accelRandomWalk` (m/s³/√Hz, —), `gyroNoiseDensity` (rad/s/√Hz, —), `gyroBiasInstability` (rad/s, —), `gyroRandomWalk` (rad/s²/√Hz, —), `accelRange` (m/s², —), `gyroRange` (rad/s, —) |
| `GnssAPI_1` | `sensor:gnss:horizontalStd` (m, —), `verticalStd` (m, —), `velocityStd` (m/s, —), `multipathModel` (token: `none`, `urban`), `minElevation` (rad, 0.17) |
| `MagnetometerAPI_1` | `sensor:mag:noiseStd` (T, —), `hardIron` (T[3], 0), `softIron` (double[9], identity), `earthFieldModel` (token: `constant`, `wmm`), `earthField` (T[3], — when `constant`; site parameter, no fallback) |
| `CustomSensorAPI_1` | (RFC 0008 §3) `sensor:custom:kind` (string, —: what a plugin looks for), and any number of `sensor:custom:param:<name>` (double or string; `:` in the name is `/` in the parameter path). From URDF and SDFormat, every `<sensor>` of a type Lagrange does not simulate becomes one: its leaf elements are its parameters by path (`<range><max>` is `range/max`; a first element named as the type is left out) |

**Lidar scan patterns and fisheye lenses (RFC 0015, as built).** `scanPattern = "flash"` is a
focal-plane array (every beam at once, the beams evenly spaced on the image plane, outer angles
within ±π/2, `motionDistortion` not used); `"rosette"` is two Risley prisms with
`horizontalSamples` points per scan and one channel, read from the five `rosette:*` attributes
(unauthored: the loader's values in `sensors.md`, with a note; no schema fallback, so unmeasured
prism speeds are not frozen for 1.x). `rotating`, `solid_state` and `custom` stay the even grid.
A non-empty `channelElevations` gives the rings (sorted from the lowest) instead of
`verticalFov`; an authored `channels` must equal its length. `lensType = "equidistant"` makes the
camera a fisheye (Kannala–Brandt; k1..k4 from `distortionModel = "equidistant"` and
`distortionCoeffs`, or `"none"` for all zero; `"plumb_bob"` is refused): its angle across the
width is `2 θ_d⁻¹(horizontalAperture / (2 focalLength))` through the lens's own mapping, so a
200° fisheye has a positive focal length; `cutoffAngle` (at most π) defaults to the larger of
half that angle and π/2. `UsdGeomCamera.projection` stays `perspective` (the only other token
OpenUSD allows is `orthographic`). The stage writer writes all of it, so URDF → `lagrange_import`
→ USD → run gives the same beams bit for bit and the same lens (to the float precision of
`focalLength`). A file using them, opened by Lagrange 1.2 or older, runs as a grid lidar and a
pinhole camera.

### 3.9 `schemas/comms/schema.usda`

Defined in v0.5 (`domains/comms.md`). Powers and ratios are linear SI values; only the
shadowing spread is given in decibels, as path-loss models state it.

**`RadioLinkAPI_1`** (single-apply, on the receiving radio's prim).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `comms:transmitter` | rel | — | — | The transmitting radio's prim. Required |
| `comms:frequency` | double | Hz | — | Carrier. Required |
| `comms:bandwidth` | double | Hz | — | Channel bandwidth. Required |
| `comms:txPower` | double | W | — | Transmit power. Required |
| `comms:txAntennaGain`, `comms:rxAntennaGain` | double | 1 | `1.0` | Linear gains |
| `comms:pathLossExponent` | double | 1 | `2.0` | Log-distance exponent |
| `comms:referenceDistance` | double | m | `1.0` | d0, where free-space loss holds |
| `comms:wallLoss` | double | 1 | `1.0` | Linear loss per wall crossed |
| `comms:shadowingStd` | double | 1 | `0.0` | Log-normal shadowing, dB |
| `comms:shadowingCorrelationDistance` | double | m | `0.0` | 0: independent each step |
| `comms:noiseFigure` | double | 1 | `1.0` | Linear |
| `comms:modulation` | token | — | `qpsk` | `bpsk`, `qpsk`, `qam16`, `qam64` |
| `comms:packetBits` | int | 1 | `12000` | 1500 bytes |
| `comms:maxRetries` | int | 1 | `0` | Retransmissions before a packet is lost |

**`CanBusAPI_1`** (single-apply, on a bus prim): `can:bitRate` (double, 1/s, required) and
`can:frameBits` (int, fallback 108: an 11-bit identifier and 8 data bytes, before stuffing).

### 3.10 `schemas/vehicle/schema.usda`

Defined in v0.6 (`domains/vehicle_dynamics.md`). A road vehicle is a chassis body with wheels
on revolute spin joints; the module finds the ground under each wheel by a ray and models the
suspension and tyres itself, so wheel bodies carry no collision shapes.

**`VehicleAPI_1`** (single-apply, on the chassis body).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `vehicle:powertrain` | token | — | `electric` | `electric` or `ice_map` |
| `vehicle:maxDriveTorque` | double | N·m | — | electric: total at the driven wheels at full throttle |
| `vehicle:maxSteerAngle` | double | rad | — | Inner road wheel. Required with a steered wheel |
| `vehicle:steerRate` | double | rad/s | `1.0` | Rate limit |
| `vehicle:steerTimeConstant` | double | s | `0.05` | First-order lag |
| `vehicle:maxBrakeTorque` | double | N·m | — | Total at full pedal. Required |
| `vehicle:brakeBias` | double | 1 | `0.6` | Front fraction |
| `vehicle:abs` | bool | — | `false` | Slip-limiting anti-lock braking |
| `vehicle:engineSpeeds`, `vehicle:engineTorques` | double[] | rad/s, N·m | — | ice_map: full-throttle curve |
| `vehicle:engineIdleSpeed` | double | rad/s | `80` | ice_map |
| `vehicle:gearRatios` | double[] | 1 | — | ice_map: forward gears |
| `vehicle:finalDriveRatio` | double | 1 | `1.0` | ice_map |
| `vehicle:shiftUpSpeed`, `vehicle:shiftDownSpeed` | double | rad/s | `600`, `250` | ice_map: automatic shifts |
| `vehicle:driveEfficiency` | double | 1 | `0.92` | ice_map |
| `vehicle:antiRollStiffnessFront`, `...Rear` | double | N/m | `0` | Per metre of travel difference across the axle |

**`WheelAPI_1`** (single-apply, on a wheel's spin joint).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `wheel:radius` | double | m | — | Loaded rolling radius. Required |
| `wheel:width` | double | m | `0.2` | |
| `wheel:axle` | token | — | — | `front` or `rear`. Required |
| `wheel:steered`, `wheel:driven` | bool | — | `false` | |
| `wheel:steerJoint` | rel | — | — | A steered wheel's steering joint |
| `wheel:tyre` | asset | — | — | `.tir` Magic Formula file (pacejka fidelity) |
| `wheel:longitudinalStiffness` | double | N | — | linear fidelity, per unit slip ratio |
| `wheel:corneringStiffness` | double | N/rad | — | linear fidelity |
| `wheel:friction` | double | 1 | `1.0` | Peak μ on the reference surface |
| `wheel:relaxationLength` | double | m | `0.3` | Lateral transient |
| `wheel:restLength` | double | m | — | Mount to wheel centre at ride height (the model's wheel position; the spring is preloaded with its static load there). Required |
| `wheel:travel` | double | m | — | From ride height to the bump stop. Required |
| `wheel:springStiffness` | double | N/m | — | Required |
| `wheel:damping` | double | N·s/m | — | Required |

### 3.11 `schemas/road/schema.usda`

Defined in v0.6 and written by the OpenDRIVE importer (`io/opendrive/`, `lagrange_opendrive_import`).

**`RoadLaneAPI_1`** (single-apply, on an Xform per lane and lane section).

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `lane:centreLine` | double[] | m | — | x y z of each centre-line point, in the direction of travel. Required |
| `lane:width` | double[] | m | — | Width at each point |
| `lane:type` | token | — | `driving` | `driving`, `sidewalk`, `biking`, `parking`, `other` |
| `lane:speedLimit` | double | m/s | `0` | 0: none given |
| `lane:road`, `lane:lane` | int | 1 | `0` | OpenDRIVE road and lane ids |
| `lane:sStart`, `lane:sEnd` | double | m | `0` | The lane section's range along its road (OpenDRIVE s) |
| `lane:junction` | bool | — | `false` | A junction's connecting road |
| `lane:successors`, `lane:predecessors` | rel | — | — | Lanes it continues into, and those leading into it |

**OpenDRIVE import** (v0.6). An own reader covers the subset that carries geometry and lanes:
- **Reference lines:** line, arc, spiral (clothoid, by Gauss–Legendre integration), poly3 and
  paramPoly3 (both by arc length).
- **Roads:** elevation, lane offset, lane sections with width polynomials, road marks.
- **Links:** lane, road and junction links.

Superelevation, lane heights, objects and signals are not read yet. Road surfaces become meshes
per lane type, road marks become thin meshes (broken marks are 3 m on and 6 m off,
illustrative), and every driving, biking, parking or sidewalk lane becomes a `RoadLaneAPI_1`
prim with its links. The surfaces have no colliders: they lie 1 cm above the ground plane that
the loader adds, so roads with elevation are drawn but not yet driven on.

### 3.12 `schemas/tether/schema.usda`

Added in 1.x by RFC 0011 and simulated by the tether domain (`modules/tether/`).

**`TetherAPI_1`** (single-apply). The prim's origin is the anchor, such as a ground station's winch
outlet. It can sit on the world, a fixed body or a moving body.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `tether:anchorPoint` | double3 | m | `(0, 0, 0)` | The anchor in the prim's frame; on the world, a world position |
| `tether:attach` | rel | — | — | The body (link) the far end is tied to. Required |
| `tether:attachPoint` | double3 | m | `(0, 0, 0)` | Where on that body, in its frame |
| `tether:length` | double | m | `0` | Length paid out at the start. Required, over 0 |
| `tether:diameter` | double | m | `0.005` | For contact, drag and drawing |
| `tether:linearDensity` | double | kg/m | `0.02` | Mass per metre |
| `tether:axialStiffness` | double | N | `20000` | EA |
| `tether:dampingRatio` | double | 1 | `0.05` | Axial damping, a share of critical per segment |
| `tether:bendStiffness` | double | N·m² | `0` | EI |
| `tether:friction` | double | 1 | `0.4` | Coulomb, on scenery, bodies and itself |
| `tether:dragCoeff` | double | 1 | `1.2` | Normal; tangential is 1/50 of it |
| `tether:segmentLength` | double | m | `0.2` | Spacing of the point masses |
| `tether:winch` | token | — | `fixed` | `fixed`, `tension` or `free` |
| `tether:winchTension` | double | N | `5` | Set point of `tension` |
| `tether:winchSpeed` | double | m/s | `1` | Most it pays out or reels in |
| `tether:maxLength` | double | m | `0` | The drum's end; `0` is `length` |
| `tether:resistancePerLength` | double | ohm/m | `0` | Both conductors, round trip; not simulated yet |

An overlay applies it like any other schema. A new prim has no pose, so `anchorPoint` places the
station:

```yaml
apply:
  - create: ground_station
    schemas:
      TetherAPI_1:
        tether:anchorPoint: [2.0, 0.0, 0.3]
        tether:attach: base_link
        tether:length: 5.0
        tether:diameter: 0.006
        tether:linearDensity: 0.03
        tether:winch: tension
```

### 3.13 Thermal materials and heat sources (`schemas/thermal/schema.usda`)

Added in 1.x by RFC 0012, for the thermal domain's `conduction` fidelity
(docs/physics/heat-conduction.md).

**`ThermalMaterialAPI_1`** (single-apply), on a link's prim or a material bound to it. A preset
gives every number; an authored number overrides it.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `thermal:material:preset` | token | — | `""` | A row of the cited preset table, e.g. `aluminium_6061`, `fr4`, `pla` |
| `thermal:material:conductivity` | double | W/(m·K) | — | `k` (in the two directions across `axis` when `conductivityAlong` is given) |
| `thermal:material:density` | double | kg/m³ | — | `ρ` |
| `thermal:material:specificHeat` | double | J/(kg·K) | — | `c_p` |
| `thermal:material:emissivity` | double | 1 | — | Of its surfaces, 0 to 1 |
| `thermal:material:conductivityAlong` | double | W/(m·K) | — | Along `axis` (through a board or a laminate); unauthored: isotropic |
| `thermal:material:axis` | double3 | 1 | `(0, 0, 1)` | In the link's frame |
| `thermal:material:contactConductance` | double | W/(m²·K) | — | Where it touches its parent link; unauthored: no conduction across |
| `thermal:material:initialTemperature` | double | K | — | Unauthored: the ambient |
| `thermal:material:maxTemperature` | double | K | — | Over it anywhere: `overtemp` |

**`HeatSourceAPI_1`** (multiple-apply, `thermal:source:<name>:`), on a link's prim. Its heat,
`(1 − efficiency) · power`, is shared over the link's cells inside the box.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `power` | double | W | `0` | Electrical input power |
| `powerSignal` | token | — | `""` | Or a signal in W, read each step; overrides `power` |
| `efficiency` | double | 1 | `0` | Share leaving as light |
| `position` | double3 | m | `(0, 0, 0)` | Centre of the box, link frame |
| `size` | double3 | m | `(0.001, 0.001, 0.001)` | Full size of the box |

```yaml
apply:
  - prim: led_board
    schemas:
      ThermalMaterialAPI_1:
        thermal:material:preset: fr4
        thermal:material:contactConductance: 3000.0
      HeatSourceAPI_1:led_1:
        thermal:source:led_1:power: 3.0
        thermal:source:led_1:efficiency: 0.35
        thermal:source:led_1:position: [0.01, 0.0, 0.0016]
        thermal:source:led_1:size: [0.003, 0.003, 0.001]
```

In a URDF, on the link (read by the server and `lagrange_import`, which writes the schemas):

```xml
<gazebo reference="led_board">
  <thermal_material preset="fr4" conductivity="20" conductivity_along="0.3" axis="0 0 1" contact_conductance="3000"/>
  <heat_source name="led_1" power="3.0" efficiency="0.35" xyz="0.01 0 0.0016" size="0.003 0.003 0.001"/>
  <heat_source name="driver" power_signal="motor/left/loss_power"/>
</gazebo>
```

`<thermal_material>` takes `preset`, `conductivity`, `density`, `specific_heat`, `emissivity`,
`conductivity_along`, `axis`, `contact_conductance`, `initial_temperature`, `max_temperature`;
`<heat_source>` takes `name` (required), `power`, `power_signal`, `efficiency`, `xyz`, `size`.

## 4. Sample robot: differential-drive AMR

`assets/robots/amr_diffdrive/amr_diffdrive.usda`. Geometry is primitive shapes so that the file
stays tiny and has no external meshes.

```usda
#usda 1.0
(
    defaultPrim = "amr"
    metersPerUnit = 1
    upAxis = "Z"
    kilogramsPerUnit = 1
    doc = "Sample differential-drive AMR for lagrange. Parameters are illustrative, not a real product."
)

def Xform "amr" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    kind = "component"
)
{
    def Xform "base_link" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "AeroAPI_1", "ThermalAPI_1"]
    )
    {
        float physics:mass = 38.0                       # kg
        point3f physics:centerOfMass = (0, 0, 0.12)     # m
        double3 aero:referenceArea = (0.18, 0.24, 0.30) # m², faces along x, y, z
        double3 aero:dragCoeff = (1.05, 1.05, 1.05)
        double thermal:heatCapacity = 9000.0            # J/K, chassis
        double thermal:convectionArea = 0.9             # m²
        double3 xformOp:translate = (0, 0, 0.10)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "visual" { double size = 1 float3 xformOp:scale = (0.60, 0.44, 0.20)
                            uniform token[] xformOpOrder = ["xformOp:scale"]
                            rel material:binding = </amr/materials/painted_steel> }
        def Cube "collision" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )
        {
            double size = 1
            float3 xformOp:scale = (0.60, 0.44, 0.20)
            uniform token[] xformOpOrder = ["xformOp:scale"]
            uniform token purpose = "guide"
        }
    }

    def Xform "left_wheel" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 1.4
        double3 xformOp:translate = (0, 0.25, 0.08)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cylinder "collision" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )
        { double radius = 0.08 double height = 0.04 uniform token axis = "Y"
          rel material:binding:physics = </amr/materials/rubber> }
    }

    def Xform "right_wheel" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 1.4
        double3 xformOp:translate = (0, -0.25, 0.08)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cylinder "collision" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )
        { double radius = 0.08 double height = 0.04 uniform token axis = "Y"
          rel material:binding:physics = </amr/materials/rubber> }
    }

    def Xform "caster" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"] )
    {
        float physics:mass = 0.3
        double3 xformOp:translate = (-0.24, 0, 0.03)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Sphere "collision" ( prepend apiSchemas = ["PhysicsCollisionAPI"] )
        { double radius = 0.03 rel material:binding:physics = </amr/materials/low_friction> }
    }

    def Scope "joints"
    {
        def PhysicsRevoluteJoint "left_wheel_joint" ( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )
        {
            rel physics:body0 = </amr/base_link>
            rel physics:body1 = </amr/left_wheel>
            uniform token physics:axis = "Y"
            point3f physics:localPos0 = (0, 0.25, -0.02)
        }
        def PhysicsRevoluteJoint "right_wheel_joint" ( prepend apiSchemas = ["PhysicsDriveAPI:angular"] )
        {
            rel physics:body0 = </amr/base_link>
            rel physics:body1 = </amr/right_wheel>
            uniform token physics:axis = "Y"
            point3f physics:localPos0 = (0, -0.25, -0.02)
        }
        def PhysicsSphericalJoint "caster_joint"
        {
            rel physics:body0 = </amr/base_link>
            rel physics:body1 = </amr/caster>
            point3f physics:localPos0 = (-0.24, 0, -0.07)
        }
    }

    def Scope "power"
    {
        def Xform "battery" (
            prepend apiSchemas = ["BatteryAPI_1", "ThermalAPI_1", "PortAPI_1:dc_out"]
        )
        {
            uniform token elec:battery:chemistry = "lfp"
            int elec:battery:seriesCells = 16
            int elec:battery:parallelCells = 2
            double elec:battery:capacity = 144000.0          # C (40 Ah)
            double[] elec:battery:ocvSoc = [0.0, 0.1, 0.5, 0.9, 1.0]
            double[] elec:battery:ocvVoltage = [2.80, 3.20, 3.28, 3.33, 3.45]   # V per cell
            double elec:battery:r0 = 0.0015                  # ohm per cell
            double elec:battery:r1 = 0.0008
            double elec:battery:c1 = 12000.0
            double elec:battery:cutoffVoltage = 44.8         # V
            double elec:battery:maxChargeCurrent = 20.0      # A
            double thermal:heatCapacity = 14000.0            # J/K
            double elec:port:dc_out:nominalVoltage = 51.2    # V
            double elec:port:dc_out:maxCurrent = 80.0        # A
            rel elec:port:dc_out:connectedTo = </amr/power/dc_48v>
            rel thermal:link:chassis:target = </amr/base_link>
            double thermal:link:chassis:conductance = 2.5    # W/K
        }

        def Xform "dc_48v" ( prepend apiSchemas = ["ElectricalBusAPI_1"] )
        {
            double elec:bus:capacitance = 0.0047             # F
            double elec:bus:undervoltageTrip = 42.0          # V
            double elec:bus:overvoltageTrip = 60.0           # V
        }

        def Xform "computer" ( prepend apiSchemas = ["ElectricalLoadAPI_1"] )
        {
            uniform token elec:load:model = "constant_power"
            double elec:load:power = 45.0                    # W
            double elec:load:minVoltage = 36.0               # V
            rel elec:powerPort = </amr/power/dc_48v>
        }
    }

    def Xform "left_drive_motor" (
        prepend apiSchemas = ["ElectricalMotorAPI_1", "ThermalAPI_1", "EmiSusceptibilityAPI_1", "EmiSourceAPI_1"]
    )
    {
        double elec:windingResistance = 0.12       # ohm @ 25 °C
        double elec:torqueConstant = 0.085         # N·m/A
        double elec:inductance = 0.00031           # H
        int elec:polePairs = 4
        double elec:rotorInertia = 0.00012         # kg·m²
        double elec:gearRatio = 20.0
        double elec:gearEfficiency = 0.92
        double elec:maxCurrent = 30.0              # A
        rel elec:joint = </amr/joints/left_wheel_joint>
        rel elec:powerPort = </amr/power/dc_48v>
        double thermal:heatCapacity = 450.0        # J/K
        double thermal:convectionArea = 0.03       # m²
        double thermal:deratingStart = 373.15      # K
        double thermal:maxTemperature = 403.15     # K
        uniform token emi:victimType = "encoder"
        double emi:encoderGlitchField = 200.0      # V/m
        uniform token emi:sourceType = "motor_drive"
        double emi:frequency = 20000.0             # Hz, the drive's PWM frequency (a component property)
        rel emi:drivenBy = </amr/left_drive_motor>
    }

    def Xform "right_drive_motor" (
        prepend apiSchemas = ["ElectricalMotorAPI_1", "ThermalAPI_1", "EmiSusceptibilityAPI_1"]
    )
    {
        double elec:windingResistance = 0.12
        double elec:torqueConstant = 0.085
        double elec:inductance = 0.00031
        int elec:polePairs = 4
        double elec:rotorInertia = 0.00012
        double elec:gearRatio = 20.0
        double elec:gearEfficiency = 0.92
        double elec:maxCurrent = 30.0
        rel elec:joint = </amr/joints/right_wheel_joint>
        rel elec:powerPort = </amr/power/dc_48v>
        double thermal:heatCapacity = 450.0
        double thermal:convectionArea = 0.03
        uniform token emi:victimType = "encoder"
        double emi:encoderGlitchField = 200.0
    }

    def Scope "sensors"
    {
        def Xform "front_lidar" ( prepend apiSchemas = ["SensorAPI_1", "LidarAPI_1"] )
        {
            double3 xformOp:translate = (0.25, 0, 0.28)
            uniform token[] xformOpOrder = ["xformOp:translate"]
            double sensor:rate = 15.0                        # Hz
            int sensor:lidar:channels = 1
            int sensor:lidar:horizontalSamples = 1440
            double2 sensor:lidar:verticalFov = (0, 0)
            double sensor:lidar:rangeMin = 0.05              # m
            double sensor:lidar:rangeMax = 25.0              # m
        }
        def Xform "imu" ( prepend apiSchemas = ["SensorAPI_1", "ImuAPI_1", "EmiSusceptibilityAPI_1"] )
        {
            double sensor:rate = 200.0
            double sensor:imu:accelNoiseDensity = 0.002      # m/s²/√Hz
            double sensor:imu:accelBiasInstability = 0.0004  # m/s²
            double sensor:imu:accelRandomWalk = 0.00006
            double sensor:imu:gyroNoiseDensity = 0.00017     # rad/s/√Hz
            double sensor:imu:gyroBiasInstability = 0.00002
            double sensor:imu:gyroRandomWalk = 0.000002
            double sensor:imu:accelRange = 156.9             # m/s² (16 g)
            double sensor:imu:gyroRange = 34.9               # rad/s (2000 °/s)
            uniform token emi:victimType = "imu_adc"
        }
    }

    def Scope "materials"
    {
        def Material "painted_steel" ( prepend apiSchemas = ["SensorMaterialAPI_1"] )
        {
            double sensmat:reflectivity905 = 0.45
            double sensmat:conductivity = 6.0e6              # S/m, order of magnitude for steel
            double sensmat:relativePermeability = 100.0
        }
        def Material "rubber" ( prepend apiSchemas = ["PhysicsMaterialAPI", "SensorMaterialAPI_1"] )
        {
            float physics:staticFriction = 0.9
            float physics:dynamicFriction = 0.8
            double sensmat:reflectivity905 = 0.05
            double sensmat:frictionWetScale = 0.65
        }
        def Material "low_friction" ( prepend apiSchemas = ["PhysicsMaterialAPI"] )
        { float physics:staticFriction = 0.05 float physics:dynamicFriction = 0.05 }
    }
}
```

All numeric values in this sample are illustrative. They are chosen to be plausible, not taken
from a datasheet, and the file says so in its `doc` string.

## 5. Simulation profiles

### 5.1 Changes to the reference example

The examples in CLAUDE.md §11.1 stay valid under the schema below. Four fields are added
(open question F5):

- `schema_version` (required): lets `tools/migrate_profile` upgrade old files.
- `seed`, `deterministic`, `tier`: needed for reproducible runs and tier forcing.
- `time`: real-time factor target and optional end time.
- `substitute` accepts either the short token used in CLAUDE.md (`zero`, `clear`,
  `ground_truth`, `replay`, …) or an object `{kind, preset, source}`.

`dt` stays a number in seconds, as in the reference. The loader parses its decimal text exactly
and requires a whole number of nanoseconds (architecture §2).

### 5.2 `profiles/schema.json` (draft)

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://lagrange.example/schemas/profile/1/schema.json",
  "title": "lagrange simulation profile",
  "type": "object",
  "additionalProperties": false,
  "required": ["schema_version", "stage", "domains"],
  "properties": {
    "schema_version": { "const": 1 },
    "stage": { "type": "string", "minLength": 1, "description": "USD file with the robot(s) and world." },
    "environment": { "type": "string", "description": "USD file with a WeatherAPI prim and environment sources." },
    "seed": { "type": "integer", "minimum": 0, "default": 0 },
    "deterministic": { "type": "boolean", "default": false },
    "tier": { "enum": ["auto", 0, 1, 2], "default": "auto" },
    "batch": { "type": "integer", "minimum": 1, "default": 1, "description": "Parallel environments." },
    "time": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "real_time_factor": { "type": "number", "minimum": 0, "default": 1.0, "description": "0 = as fast as possible." },
        "end": { "type": "number", "exclusiveMinimum": 0, "description": "Stop after this many simulated seconds." }
      }
    },
    "domains": {
      "type": "object",
      "additionalProperties": false,
      "required": ["rigid_body", "electrical", "thermal", "aerodynamics", "weather", "emi", "radiation", "sensors"],
      "properties": {
        "rigid_body":       { "$ref": "#/$defs/rigid_body" },
        "electrical":       { "$ref": "#/$defs/electrical" },
        "thermal":          { "$ref": "#/$defs/thermal" },
        "aerodynamics":     { "$ref": "#/$defs/aerodynamics" },
        "weather":          { "$ref": "#/$defs/weather" },
        "emi":              { "$ref": "#/$defs/emi" },
        "radiation":        { "$ref": "#/$defs/radiation" },
        "sensors":          { "$ref": "#/$defs/sensors" },
        "vehicle_dynamics": { "$ref": "#/$defs/vehicle_dynamics" },
        "rotor_aero":       { "$ref": "#/$defs/rotor_aero" },
        "hydrodynamics":    { "$ref": "#/$defs/generic_domain" },
        "traffic":          { "$ref": "#/$defs/traffic" },
        "comms":            { "$ref": "#/$defs/generic_domain" }
      }
    },
    "world": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "geo_origin": {
          "type": "object",
          "additionalProperties": false,
          "required": ["latitude", "longitude", "height"],
          "properties": {
            "latitude":  { "type": "number", "minimum": -90,  "maximum": 90,  "description": "degrees, WGS84" },
            "longitude": { "type": "number", "minimum": -180, "maximum": 180, "description": "degrees, WGS84" },
            "height":    { "type": "number", "description": "metres above the WGS84 ellipsoid" }
          }
        },
        "streaming": {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "enabled": { "type": "boolean", "default": false },
            "load_radius":   { "type": "number", "exclusiveMinimum": 0, "description": "m" },
            "unload_radius": { "type": "number", "exclusiveMinimum": 0, "description": "m" },
            "collision_radius": { "type": "number", "exclusiveMinimum": 0, "description": "m: tiles with colliders load only this near a robot (default 300)" },
            "active_region_radius": { "type": "number", "exclusiveMinimum": 0, "description": "m" },
            "rebase_distance": { "type": "number", "exclusiveMinimum": 0, "default": 1000, "description": "m" }
          }
        }
      }
    },
    "bridges": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "ros2": {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "enabled": { "type": "boolean", "default": true },
            "namespace": { "type": "string", "default": "" },
            "use_sim_time": { "type": "boolean", "default": true },
            "gpu_buffers": { "type": "boolean", "default": false, "description": "Publish images/point clouds through rosidl::Buffer." }
          }
        },
        "ros2_control": { "type": "object", "additionalProperties": false,
          "properties": { "enabled": { "type": "boolean", "default": false } } },
        "mavlink": {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "enabled": { "type": "boolean", "default": false },
            "autopilot": { "enum": ["px4", "ardupilot"] },
            "lockstep": { "type": "boolean", "default": true },
            "endpoint": { "type": "string", "description": "e.g. tcp://127.0.0.1:4560" }
          }
        },
        "autoware": { "type": "object", "additionalProperties": false,
          "properties": { "enabled": { "type": "boolean", "default": false } } }
      }
    },
    "scenario": {
      "type": "object",
      "additionalProperties": false,
      "properties": {
        "opendrive": { "type": "string" },
        "openscenario": { "type": "string" }
      }
    },
    "views": {
      "type": "array",
      "uniqueItems": true,
      "items": { "enum": ["physics_3d", "electrical_schematic", "waveforms", "thermal", "fields", "sensors"] }
    },
    "record": {
      "type": "object",
      "additionalProperties": false,
      "required": ["format", "path"],
      "properties": {
        "format": { "const": "mcap" },
        "path": { "type": "string", "minLength": 1 },
        "signals": { "type": "array", "items": { "type": "string" }, "description": "Glob patterns; default all." },
        "compression": { "enum": ["none", "zstd", "lz4"], "default": "zstd" }
      }
    }
  },
  "$defs": {
    "dt": { "type": "number", "exclusiveMinimum": 0, "maximum": 10, "description": "seconds; must be a whole number of nanoseconds" },
    "substitute": {
      "oneOf": [
        { "enum": ["ideal", "zero", "clear", "ambient", "ground_truth", "fixed", "load_profile", "replay"] },
        {
          "type": "object",
          "additionalProperties": false,
          "required": ["kind"],
          "properties": {
            "kind": { "enum": ["ideal", "constant", "replay"] },
            "preset": { "type": "string" },
            "source": { "type": "string" },
            "value": { "type": "number" }
          }
        }
      ]
    },
    "enabled_or_substituted": {
      "description": "A disabled domain must name a substitute; a replay substitute must name a source.",
      "allOf": [
        { "if": { "properties": { "enabled": { "const": false } } },
          "then": { "required": ["substitute"] } },
        { "if": { "properties": { "substitute": { "const": "replay" } }, "required": ["substitute"] },
          "then": { "required": ["source"] } }
      ]
    },
    "common": {
      "type": "object",
      "required": ["enabled"],
      "properties": {
        "enabled": { "type": "boolean" },
        "dt": { "$ref": "#/$defs/dt" },
        "substitute": { "$ref": "#/$defs/substitute" },
        "source": { "type": "string", "description": "MCAP file for a replay substitute." }
      }
    },
    "generic_domain": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "unevaluatedProperties": false
    },
    "rigid_body": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "backend": { "enum": ["physx", "mujoco", "jolt"] },
        "solver_iterations": { "type": "integer", "minimum": 1 },
        "gpu": { "type": "boolean", "default": false },
        "load_profile": { "type": "string", "description": "CSV torque-vs-speed table for the load_profile substitute." }
      },
      "unevaluatedProperties": false
    },
    "electrical": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "fidelity": { "enum": ["ideal_bus", "averaged", "fmu"], "default": "averaged" },
        "speed_prediction": { "type": "boolean", "default": true }
      },
      "unevaluatedProperties": false
    },
    "thermal": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "ambient_temperature": { "type": "number", "exclusiveMinimum": 0, "description": "K; required for the ambient substitute when no environment file is given" }
      },
      "unevaluatedProperties": false
    },
    "aerodynamics": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "fidelity": { "enum": ["drag", "drag_plus_field", "lbm"], "default": "drag" }
      },
      "unevaluatedProperties": false
    },
    "weather": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "effects": { "type": "array", "uniqueItems": true,
          "items": { "enum": ["fog", "rain", "dust", "wet_friction", "lens_contamination"] } }
      },
      "unevaluatedProperties": false
    },
    "emi": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "coupling": { "type": "array", "uniqueItems": true,
          "items": { "enum": ["magnetometer", "imu", "encoder", "can_bus", "wifi"] } }
      },
      "unevaluatedProperties": false
    },
    "radiation": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "seu_injection": { "type": "boolean", "default": false }
      },
      "unevaluatedProperties": false
    },
    "sensors": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "only": { "type": "array", "items": { "type": "string" }, "description": "Sensor prim names to simulate; others get ground truth." }
      },
      "unevaluatedProperties": false
    },
    "vehicle_dynamics": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "tire_model": { "enum": ["linear", "pacejka"], "default": "pacejka" }
      },
      "unevaluatedProperties": false
    },
    "rotor_aero": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "fidelity": { "enum": ["coefficients", "bem"], "default": "coefficients" },
        "ground_effect": { "type": "boolean", "default": true }
      },
      "unevaluatedProperties": false
    },
    "traffic": {
      "allOf": [ { "$ref": "#/$defs/common" }, { "$ref": "#/$defs/enabled_or_substituted" } ],
      "properties": {
        "backend": { "enum": ["builtin", "sumo"], "default": "builtin" },
        "sumo_config": { "type": "string" }
      },
      "unevaluatedProperties": false
    }
  }
}
```

Checks that JSON Schema cannot express and that the loader performs: every `dt` is a whole number
of nanoseconds; steps are commensurate (architecture §6); referenced files exist; a `replay`
source contains the channels the domain contract needs; `emi.coupling` entries have matching
`EmiSusceptibilityAPI` prims; the requested tier is available.

CI validates every file in `profiles/examples/` with `check-jsonschema` (TODO(verify) version)
and then loads it with the mock server in `--validate-only` mode.

### 5.3 Example profiles, one per robot class

The two profiles from CLAUDE.md §11.1 remain in `profiles/examples/` with `schema_version: 1`
added. The six below are new.

```yaml
# profiles/examples/amr_warehouse_nav2.yaml — AMR, ros2_control + Nav2, electrical on
schema_version: 1
stage: worlds/warehouse_small.usda
environment: environments/indoor_20c.usda
seed: 1

domains:
  rigid_body:   { enabled: true,  backend: mujoco, dt: 0.001 }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: false, substitute: zero }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: false, substitute: zero }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: true,  dt: 0.005 }

bridges:
  ros2:         { enabled: true, use_sim_time: true }
  ros2_control: { enabled: true }

views: [physics_3d, waveforms, sensors]
record: { format: mcap, path: runs/amr_warehouse_001.mcap }
```

```yaml
# profiles/examples/arm_pick_place_moveit.yaml — 6-DoF arm, MoveIt 2, electrical + thermal
schema_version: 1
stage: worlds/workcell_arm6.usda
environment: environments/indoor_20c.usda

domains:
  rigid_body:   { enabled: true,  backend: mujoco, dt: 0.001 }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: false, substitute: zero }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: true,  coupling: [encoder], dt: 0.001 }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: true,  dt: 0.01, only: [wrist_camera] }

bridges:
  ros2:         { enabled: true }
  ros2_control: { enabled: true }

views: [physics_3d, electrical_schematic, waveforms, thermal]
record: { format: mcap, path: runs/arm_pick_place_001.mcap }
```

```yaml
# profiles/examples/quad_px4_gusty.yaml — quadcopter, PX4 SITL lockstep, wind gusts
schema_version: 1
stage: worlds/open_field_quad.usda
environment: environments/gusty_8ms_dryden.usda
deterministic: false

domains:
  rigid_body:   { enabled: true,  backend: mujoco, dt: 0.001 }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.00005 }
  thermal:      { enabled: true,  dt: 0.1 }
  aerodynamics: { enabled: true,  fidelity: drag_plus_field, dt: 0.001 }
  rotor_aero:   { enabled: true,  fidelity: coefficients, ground_effect: true, dt: 0.001 }
  weather:      { enabled: true,  dt: 0.01 }
  emi:          { enabled: true,  coupling: [magnetometer], dt: 0.001 }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: true,  dt: 0.001 }

bridges:
  ros2:    { enabled: true }
  mavlink: { enabled: true, autopilot: px4, lockstep: true, endpoint: "tcp://127.0.0.1:4560" }

views: [physics_3d, waveforms, fields]
record: { format: mcap, path: runs/quad_gusty_001.mcap }
```

```yaml
# profiles/examples/car_skidpad_wet.yaml — passenger car, vehicle dynamics on a wet skid pad
schema_version: 1
stage: worlds/skidpad_sedan.usda
environment: environments/rain_10mmh_day.usda

domains:
  rigid_body:       { enabled: true,  backend: physx, dt: 0.001 }
  vehicle_dynamics: { enabled: true,  tire_model: pacejka, dt: 0.001 }
  electrical:       { enabled: false, substitute: ideal }
  thermal:          { enabled: false, substitute: ambient }
  aerodynamics:     { enabled: true,  fidelity: drag, dt: 0.01 }
  weather:          { enabled: true,  effects: [rain, wet_friction], dt: 0.05 }
  emi:              { enabled: false, substitute: zero }
  radiation:        { enabled: false, substitute: zero }
  sensors:          { enabled: false, substitute: ground_truth }

bridges:
  ros2: { enabled: true }

views: [physics_3d, waveforms]
record: { format: mcap, path: runs/car_skidpad_001.mcap }
```

```yaml
# profiles/examples/av_urban_autoware.yaml — autonomous car, sensors + traffic + scenario
schema_version: 1
stage: worlds/city_block_av.usda
environment: environments/fog_150m_dusk.usda
tier: 2

domains:
  rigid_body:       { enabled: true,  backend: physx, dt: 0.002 }
  vehicle_dynamics: { enabled: true,  tire_model: pacejka, dt: 0.002 }
  electrical:       { enabled: false, substitute: ideal }
  thermal:          { enabled: false, substitute: ambient }
  aerodynamics:     { enabled: true,  fidelity: drag, dt: 0.01 }
  weather:          { enabled: true,  effects: [fog, lens_contamination], dt: 0.05 }
  emi:              { enabled: false, substitute: zero }
  radiation:        { enabled: false, substitute: zero }
  sensors:          { enabled: true,  dt: 0.002 }
  traffic:          { enabled: true,  backend: sumo, sumo_config: scenarios/city_block.sumocfg, dt: 0.1 }
  comms:            { enabled: false, substitute: ideal }

scenario:
  opendrive: scenarios/city_block.xodr
  openscenario: scenarios/cut_in_fog.xosc

bridges:
  ros2:     { enabled: true, gpu_buffers: true }
  autoware: { enabled: true }

views: [physics_3d, sensors, fields]
record: { format: mcap, path: runs/av_urban_001.mcap, signals: ["body/*", "sensor/*/meta", "traffic/*"] }
```

```yaml
# profiles/examples/huge_world_survey.yaml — drone survey over a 40 km geospatial tile set
schema_version: 1
stage: worlds/coastal_survey_root.usda        # root layer; terrain tiles are USD payloads
environment: environments/clear_day.usda

world:
  geo_origin: { latitude: 33.5902, longitude: 130.4017, height: 20.0 }   # scenario parameter
  streaming:
    enabled: true
    load_radius: 3000
    unload_radius: 3500
    active_region_radius: 500
    rebase_distance: 1000

domains:
  rigid_body:   { enabled: true,  backend: mujoco, dt: 0.002 }
  electrical:   { enabled: true,  fidelity: averaged, dt: 0.0001 }
  thermal:      { enabled: false, substitute: ambient, ambient_temperature: 293.15 }
  aerodynamics: { enabled: true,  fidelity: drag_plus_field, dt: 0.002 }
  rotor_aero:   { enabled: true,  fidelity: coefficients, dt: 0.002 }
  weather:      { enabled: false, substitute: clear }
  emi:          { enabled: false, substitute: zero }
  radiation:    { enabled: false, substitute: zero }
  sensors:      { enabled: true,  dt: 0.01, only: [gnss, imu, nadir_camera] }

bridges:
  ros2:    { enabled: true }
  mavlink: { enabled: true, autopilot: ardupilot, lockstep: true }

views: [physics_3d, sensors]
record: { format: mcap, path: runs/coastal_survey_001.mcap }
```

In `quad_px4_gusty.yaml` the electrical step is 50 µs and physics is 1 ms; the base tick is 50 µs
and the hyperperiod is 100 ms. In `amr_warehouse_nav2.yaml` the sensors step of 5 ms is the
scheduling period of the sensor module; each sensor's own rate comes from `sensor:rate`.

## 6. FMU usage

- Standard: FMI 3.0 co-simulation. Model exchange is not hosted.
- An FMU is referenced from USD with an `asset` attribute (`elec:behaviorModel`). Its variables
  are mapped to signals by name through a sidecar file next to the FMU:

```yaml
# models/bldc_drive.fmu.map.yaml
fmu: bldc_drive.fmu
fmi_version: "3.0"
step: 0.0001                      # s, communication step; must divide the electrical module dt
inputs:
  - { variable: omega_mech,  signal: "joint/{joint}/velocity", unit: rad/s }
  - { variable: tau_cmd,     signal: "joint/{joint}/torque_command", unit: N.m }
  - { variable: v_bus,       signal: "bus/{bus}/voltage", unit: V }
  - { variable: T_winding,   signal: "thermal/{prim}/temperature", unit: K }
outputs:
  - { variable: tau_shaft,   signal: "joint/{joint}/torque", unit: N.m }
  - { variable: i_bus,       signal: "motor/{prim}/bus_current", unit: A }
  - { variable: p_loss,      signal: "motor/{prim}/loss_power", unit: W }
parameters:
  - { variable: R_s, attribute: "elec:windingResistance" }
  - { variable: L_d, attribute: "elec:inductance" }
  - { variable: K_t, attribute: "elec:torqueConstant" }
```

- The FMU host checks each mapped variable's declared FMI unit against the signal unit. FMI units
  are converted to a `lagrange::Unit` from the `BaseUnit` exponents in `modelDescription.xml`; a
  mismatch is a wiring error.
- FMUs must be source FMUs or carry a `linux64` (amd64) and `aarch64-linux` binary. Binary-only
  FMUs for one architecture make the profile fail on the other with a clear message.
- Determinism: an FMU is treated as deterministic only if it declares `canGetAndSetFMUState`
  and passes a save/restore round-trip test at load.
- Built-in C++ models are the default for the battery ECM and the dq motor. FMUs are the
  extension path. See open question Q8 for why the plan does not depend on FMUs for v0.3.

**Status (v0.3).** `FmuComponent` (`modules/electrical/fmu_host/`, on FMI Library 3.0.4 through
`fmilibrary3_vendor`) hosts FMI 3.0 co-simulation FMUs as above: the map, unit checks from the
BaseUnit exponents (SI units only: a factor or offset is refused), parameters from the
component's attributes, one `fmi3DoStep` per module step at the map's `step`, and determinism and
save/restore through `fmi3GetFMUState` and serialization when the FMU declares both. Additions to
the sketch above: an input may say `optional: <value>`, read while its signal is not written (a
winding temperature without the thermal domain); every input is read as a feedback input, so an
FMU and the network can read each other's outputs. At electrical fidelity `fmu` the network leaves
a motor with `elec:behaviorModel` to its FMU and takes its `motor/<m>/bus_current` into the bus;
battery FMUs are not hosted yet. The reference FMU is `lagrange_models`' `dc_drive` (a brushed DC
drive, hand-written in C, Apache-2.0), which matches the built-in brushed model step for step.

## 7. Recordings (MCAP)

- One file per run. Chunked, zstd by default, with the summary section and index.
- Message encoding: Protobuf, schemas from `proto/lagrange/record/v1/`. The same
  `SignalBlock` message as the live protocol is used, so replay is a file-backed protocol
  source.
- Channels:

| Channel topic | Message | Content |
|---|---|---|
| `/lagrange/meta` | `RunMeta` | profile (YAML text), stage path and hash, schema versions, seed, tier, git describe |
| `/lagrange/catalog` | `SignalCatalog` | every signal's name, type, unit, writer |
| `/lagrange/scene/snapshot` | `SceneSnapshot` | one at start and every 10 s (seek points) |
| `/lagrange/scene/delta` | `SceneDelta` | transform and attribute changes |
| `/lagrange/signals/<dt>ns` | `SignalBlock` | signals grouped by writer step size, e.g. `/lagrange/signals/100000ns` |
| `/lagrange/state_hash` | `StateHash` | determinism check values |
| `/lagrange/scene/frame` | `LocalFrame` | the region frame from that time on, at each rebase of the floating origin (v0.7); positions recorded after it are in the new frame |
| `/lagrange/log` | `LogRecord` | console messages |
| ROS topics (optional) | CDR, as rosbag2 writes them | when `record.ros_topics` is used (later) |

- Implementation (v0.2, `lagrange_server --record`): every bus signal at its native rate from
  t = 0 (a sample written late is NaN, so runs stay contiguous), signals with more than
  `--record-max-components` components (default 1024, e.g. a 3D lidar's ranges) left out, and
  one `scene<prim path>/pose` signal at 50 Hz per moving prim, bound to its prim in
  `RunMeta.pose_bindings` so that a replay moves visuals and sensors too. `RunMeta` is written
  at the start and again at the end with the duration; readers keep the last. Scene deltas,
  log records and sensor frames are not recorded yet. The sample AMR records about 1.5 MB per
  simulated second without its power system and about 2.8 MB with it (the 10 kHz electrical
  signals; zstd does little for noisy doubles).
- Replay (v0.3, `lagrange_recording`): `Recording::open` loads a file's signals into memory, and
  `ReplaySubstitute(domain, file)` writes every signal a module of that domain wrote, at its
  recorded rate and unit, committing the sample for its step's end time as a live writer does.
  Bench mode replays `rigid_body` (`--bench`, or the profile's `replay` substitute with a
  `source`). Physics stepping at the end of its window (architecture.md §8.1) made the same
  sample visible one step later, so a bench run matches the original closely, not bit for bit.
- Units are stored in the catalog. A replay substitute refuses a channel whose recorded unit
  differs from the contract.
- Compatibility: files written by rosbag2 with the MCAP storage plugin can be opened by the mock
  server for ROS-topic display (pose, joint states, images). They do not contain our signal
  catalog, so scopes show only numeric fields that the viewer can extract from known message
  types. This is the "MCAP compatibility" item in the v0.1 scope.

## 8. Materials and meshes

- Render materials: `UsdPreviewSurface` is the baseline that every importer writes and the viewer
  renders. MaterialX (`UsdMtlx`) is read when present and falls back to its
  `UsdPreviewSurface` approximation; a MaterialX-to-Slang path is a later RFC.
- Importers convert glTF PBR metallic-roughness, OBJ MTL and DAE Phong/Lambert to
  `UsdPreviewSurface`. Textures are copied next to the USD file; files over 1 MB are not
  committed to the main repo (CLAUDE.md §9).
- Collision: `tools/collision_bake` runs CoACD (default) or V-HACD on each render mesh marked for
  collision and writes explicit convex hull `Mesh` prims with `purpose = "guide"` under a
  `collision` scope. Backends load only those hulls and primitive shapes. A stage whose collider
  references a render mesh directly produces a load warning and an automatic single convex hull.

## 9. Importers

All importers are command-line tools and library functions under `io/importers/`, and produce a
USD layer plus a report of what was dropped. The viewer's "Open URDF…" action runs the importer
in the server (or mock server) and receives the result through the protocol.

| Source | Library | Covered | Not covered at first |
|---|---|---|---|
| URDF / xacro | `urdfdom` (ROS package `urdf`), `xacro` executable | links, joints (all standard types), inertials, visual/collision geometry (primitives + meshes via Assimp), materials, mimic joints, limits including URDF 1.2 acceleration/jerk, `<ros2_control>` blocks → joint interfaces | `<gazebo>` extension blocks: mapped where a rule exists (sensors, friction), otherwise listed in the report; transmissions other than simple reductions |
| SDFormat | `sdformat_vendor` (0.3.4 in Lyrical) | models, links, joints, sensors (lidar, camera, imu, gps/navsat, magnetometer), lights, nested models, `<include>` of local URIs | Gazebo system plugins (reported, not converted), Fuel URIs without a local cache, heightmaps (v0.7) |
| MJCF | MuJoCo's own parser (`mj_loadXML`) then model traversal | bodies, joints, geoms, actuators (motor, position, velocity), sensors that have an equivalent, tendons (fixed) | spatial tendons, flex/deformables, plugin-defined elements |
| glTF 2.0 / OBJ / STL / DAE | Assimp 6.0.4 (apt) | meshes, node hierarchy, PBR/Phong materials, textures (baked into colours, below) | animations, skins, morph targets |

Meshes in v0.2: `lagrange_meshes` (`io/meshes/`) reads glTF, OBJ, STL and Collada through
Assimp into triangles grouped by material, with the node hierarchy baked in, Collada units
applied and Collada's `<up_axis>` ignored (the geometry keeps its authored axes, as ROS tools
expect). `package://`, `file://` and relative references resolve through the ament index. The
URDF loader uses it for visual meshes: one prim per material, coloured by the URDF material when
there is one and by the file's otherwise. Textures are not imported yet (a note says so), and
collision meshes collide as their convex hull (concave ones on fixed links are skipped; `collision_bake` splits them).

**Status in v0.2.** `lagrange_importers` reads SDFormat and MJCF into the same world tables and
scene as the URDF loader, and `lagrange_import` writes any of the three as USD
(`io/importers/include/lagrange/importers/Importers.hpp` lists the rules). Not yet: mesh visuals in SDFormat
and MJCF (the URDF loader reads them through `lagrange_meshes`), cameras and other
sensors without a Lagrange model, URDF planar joints, and MJCF
tendons, flex, servo actuators (their joints take torque commands instead) and equalities other than one joint copying another. Each is listed in
the import report. MJCF has no lidar; lidars on an MJCF robot come from an overlay (0.2-G2).

Electrical, thermal and EMI data do not exist in URDF/SDF/MJCF. Importers accept an optional
**overlay** file that applies our API schemas by link or joint name:

```yaml
# amr_diffdrive.overlay.yaml (excerpt; the full file is in assets/robots/amr_diffdrive/)
schema_version: 1
apply:
  - prim: left_wheel_joint            # a link, joint or sensor by name, or a prim path
    create_child: left_drive_motor    # a new prim, under a joint's parent link
    schemas:
      ElectricalMotorAPI_1:
        elec:windingResistance: 0.12
        elec:inductance: 0.00031
        elec:torqueConstant: 0.085
        elec:maxCurrent: 8.0
        elec:joint: left_wheel_joint  # relationships: names or paths
        elec:powerPort: power/dc_48v
  - create: power/dc_48v              # a new prim at this path under the robot's top prim
    schemas:
      ElectricalBusAPI_1: { elec:bus:undervoltageTrip: 42.0, elec:bus:overvoltageTrip: 60.0 }
```

**Status (v0.3).** `lagrange_stage::applyOverlay` implements this format (rules on its
declaration in `io/stage/include/lagrange/stage/Stage.hpp`); `lagrange_server --overlay` and
`lagrange_import --overlay` use it, and the stage writer writes the instances, so an imported
robot keeps them in USD. Values are checked as the stage loader checks a stage, and every
problem is reported together. Overlays add schema instances; they do not yet create simulated
sensors, so lidars on an MJCF robot still need USD for now.

### 9.1 A SPICE netlist as an overlay

`--overlay` also takes a netlist (`.cir`, `.sp`, `.spice`, `.net`, `.ckt`, `io/stage/src/Netlist.cpp`), so one file is the circuit for ngspice and the power system of a run. It becomes the same schemas an overlay file makes, at load. A netlist from KiCad comes through `kicad-cli sch export netlist --format spice`.

| In the netlist | In the run |
|---|---|
| `V n 0 [DC] v` then a series chain to a node: resistors, resistor-capacitor pairs (at most two) | `BatteryAPI_1`, one cell of a flat voltage `v`; the plain resistors add up to `r0` |
| The node a chain reaches, with the `C n 0 c` on it | `ElectricalBusAPI_1`, named after the node |
| `R n 0 r` | `ElectricalLoadAPI_1` `resistive`, its power `V²/r` at the bus's start voltage |
| `B n 0 I = P / max(V(n), e) [* ((V(n) >= m) ? 1 : 0)]` | `ElectricalLoadAPI_1` `constant_power` |
| `*lagrange battery ELEMENT capacity= cutoff= max_charge= [soc=] [ocv_soc=0,0.5,1 ocv=44,48,54]` (C, V, A) | what SPICE cannot say about a battery; required. A curve is for the whole pack and must give the source's voltage at its `soc`, or ngspice and the run would answer for different batteries |
| `R a b r` between two buses, where a chain does not take it | `ElectricalBranchAPI_1` `resistor` |
| `*lagrange fuse R_x i2t=` (A²s), `*lagrange switch R_x [closed=0]` on such a resistor | a `fuse`, or a `switch` on `branch/<name>/command` |
| `D a b MODEL` with `*lagrange diode D_x drop= r=` (V, Ω) | a `diode`: a drop and a resistance forward, open backward; the SPICE model is for ngspice only |
| `*lagrange converter NAME source=V_x in=BUS max_current= [efficiency=] [input_sink=B_y]` | `ElectricalConverterAPI_1`: `V_x` from its output to ground stands for it in ngspice and sets its voltage; `B_y`, what it draws from `in` there, is read by ngspice only (`I = -I(V_x) * 12 / 0.9 / max(V(dc), 0.001)`). One per output bus |
| `*lagrange bus NODE undervoltage= overvoltage=` (V) | the bus's trips; required |
| `*lagrange motor JOINT node= r= l= kt= imax= [name=] [type=] [gear=] [efficiency=] [pole_pairs=] [rotor_inertia=]` | `ElectricalMotorAPI_1`, named `JOINT_motor` unless `name=`; a drive is not a SPICE element |

The first line is the title, as in SPICE; analysis commands and `.control` blocks are ignored. Everything else is refused with its line, every problem together, and nothing is left out in silence: transistors, inductors, subcircuits, SPICE switches, controlled and current sources, `.param`, `.include`, sources other than DC, a diode with no `*lagrange diode`, and a capacitor between two buses. So is a drive of `lagrange export spice` (`V(p_drive)`), so an exported netlist does not read back yet.  `tools/ci/check_electrical.py` runs every circuit through this route and through an overlay.

### 9.2 A URDF that names its power system

A `<lagrange>` block in a URDF (under `<robot>`), or an SDFormat file (under `<model>`), gives the robot its power system with no `--overlay`; URDF readers such as urdfdom skip the unknown element, so the file stays valid for other tools:

```xml
<lagrange>
  <schematic file="power/arm.cir"/>  <!-- relative to the URDF, or package:// -->
  <battery source="V_pack" capacity="72000" cutoff="44.8" max_charge="10"/>
  <bus node="dc" undervoltage="42" overvoltage="60"/>
  <motor joint="shoulder" node="dc" r="0.12" l="0.00031" kt="0.085" imax="8" gear="20"/>
</lagrange>
```

The attributes are the keys of §9.1's `*lagrange` lines, which the netlist may hold instead; a mistake stops the load with its message. `lagrange_import` keeps the result in the USD it writes. Adding `--overlay` to a robot with a block is refused for anything it applies twice. Names and values cannot hold spaces. MJCF has no such block: its parser refuses elements it does not know, so use `--overlay`.

**Meshes too large for the repository** (no file over 1 MB, CLAUDE.md §9) are fetched, not committed:
`assets/external_assets.yaml` lists each file with its upstream repository, pinned commit,
SHA-256 and licence, and `ros2 run lagrange_assets fetch_assets [ID...]` downloads them into
`$LAGRANGE_ASSET_DIR` (default `~/.local/share/lagrange/assets/<id>/`) with the licence and a
NOTICE. Robot files name them `lagrange-asset://<id>/<file>`; an asset that is not fetched is a
note, and the link shows a magenta box in its place. **Textures** are not drawn: the importer
bakes a textured part into parts of one colour each (each triangle takes the texture's colour
under it, PNG or JPEG, embedded or a file next to the mesh; the triangles are grouped into 16
colours at most by k-means), which keeps the viewer protocol unchanged.

Without an overlay, an imported robot runs with the electrical domain substituted by the ideal
torque source. The viewer's domain panel shows why the electrical toggle is unavailable
("no ElectricalMotorAPI prims in stage").

### 9.3 Lagrange's own elements in SDFormat

What SDFormat has no element for, Lagrange reads from custom elements prefixed `lagrange:`
(RFC 0015 §4). The namespace URI for `xmlns:lagrange` is the documentation site,
`https://open-physical-systems.github.io/Lagrange`; libsdformat does not require the
declaration, keeps prefixed elements without a warning, and Gazebo ignores them. The plain
(unprefixed) names the URDF `<gazebo>` form uses are read too, so one `<sensor>` block pastes
between the two formats; libsdformat warns that it copies them.

| Lagrange | SDFormat |
|---|---|
| fisheye lens | `<camera><lens><type>equidistant</type>` (SDFormat's own; `kannala_brandt` and `fisheye` as in URDF); other types are a note and a pinhole |
| cutoff | `<lens><cutoff_angle>` (SDFormat's own; read only when the file gives it) |
| field of view | `<horizontal_fov>` as the angle across the width (`<scale_to_hfov>false` is a note) |
| k1..k4 | `<lens><lagrange:k1>`..`<lagrange:k4>` (SDFormat's `<distortion>` is Brown–Conrady) |
| scan pattern | `<lidar><lagrange:pattern>grid\|flash\|rosette</lagrange:pattern>` |
| channel table | `<lidar><scan><vertical><lagrange:angles>` (rad, space separated) |
| rosette | `<lidar><lagrange:rosette>` with `fov`, `point_rate`, `speed_1`, `speed_2`, `prism_ratio` |

```xml
<sdf version="1.12" xmlns:lagrange="https://open-physical-systems.github.io/Lagrange">
  ...
  <sensor name="livox" type="gpu_lidar"><update_rate>10</update_rate><lidar>
    <lagrange:pattern>rosette</lagrange:pattern>
    <scan><horizontal><samples>10000</samples></horizontal></scan>
    <range><min>0.1</min><max>260</max></range>
    <lagrange:rosette><fov>0.670206</fov><point_rate>100000</point_rate></lagrange:rosette>
  </lidar></sensor>
```

## 10. Versioning and migration

| Artefact | Version carrier | Compatibility rule | Migration tool |
|---|---|---|---|
| USD schemas | family + integer suffix (`ElectricalMotorAPI_1`) | Within a version: only additions with fallbacks. Renames, unit changes, removed attributes, changed fallbacks → new version | `migrate_schema` (package `lagrange_migrate`, `tools/migrate/`) rewrites `apiSchemas` and attributes; one function per (family, from version) |
| Profiles | `schema_version` integer | Same as above | `migrate_profile` (`lagrange_migrate`); version 0 is a profile without `schema_version` |
| Protocol | package `lagrange.viewer.v1` + handshake `(major, minor)` | `buf breaking` with the `FILE` rule set against the `v1.0.0` tag (RFC 0007; `WIRE_JSON` before 1.0) | none; old clients are refused at handshake with a readable message |
| Recordings | `RunMeta.format_version` + embedded schemas | Readers accept all older versions of the same major; new channels and fields only within 1.x | `migrate_recording` (`lagrange_migrate`): checks the version, refuses newer files; a step comes with version 2 |
| Plugin ABI | `LAGRANGE_PLUGIN_ABI_MAJOR/MINOR` (1.0 at v1.0) | append only within a major (RFC 0007): `core/plugin_abi/test/test_abi_layout.c` pins sizes and offsets, `tools/ci/abi_check.sh` runs `abidiff` against the 1.0 baseline | none |
| FMU map files | `schema_version` | as profiles | `migrate_profile --fmu-map` (not built: no FMU map files exist yet) |

Loader behaviour: `core/stage/` supports the current and the previous version of each schema
family. An older version loads with a warning that names the migration command. Two versions of
the same family applied to one prim is an error.

Every schema change needs: an RFC or a PR note (CLAUDE.md §9), a migration function with a test
that round-trips a sample file, an entry in `schemas/CHANGELOG.md`, and two approvals including a
core maintainer.

From v1.0 the schemas, the plugin ABI, the protocol, profiles and recordings are frozen for the
1.x line (RFC 0007): they change only by addition, and `schemas/test/frozen_1_0.json` lists every
schema attribute as it was at 1.0. `tools/migrate/test/test_old_releases.py` takes every sample
stage and example profile of v0.5.0 and later from the git tags and checks that this release's
tools read, migrate and validate them. Before v1.0 all schemas were version 1 and could change
without a migration. The migration machinery is built in v0.3 and exercised with a synthetic `_0 → _1`
migration so that it is proven before it is needed: the family `ExampleMotorAPI`, which no real
file uses (`tools/migrate/test/test_schema.py`). The stage loader notes a prim that applies an
older version of a registered family with the command that migrates it, and refuses two versions
of one family on one prim.
