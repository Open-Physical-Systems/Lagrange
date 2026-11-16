# RFC 0021: Lights on robots and in scenes

| | |
|---|---|
| Status | Draft (for the owner to accept or reject) |
| Created | 2026-11-17 |
| Affects | `schemas/` (one new library `schemas/light`), `modules/lights/` (new, small), `modules/sensors/` (cameras and `light_sensor` see lamps; the renderer's sun from the site), `gpu/kernels/camera/realistic.slang` and `gpu/raster/` (lamps, night), `modules/weather/` (sun position from date and place), `viewer/render/` (drawing), `bridges/ros2/` (topics), `bridges/autoware/` (indicators), `io/importers/` (URDF/SDFormat forms), `profiles/schema.json` (additions), `docs/` |
| Not affected | `proto/` (§7: the scene snapshot, `GetAttributes` and signal subscription already carry everything), `core/plugin_abi/` (§8) |
| Builds on | RFC 0001, RFC 0002, RFC 0004 (tiers), RFC 0005, RFC 0007 (the 1.x freeze), RFC 0008 (plugins), RFC 0010 (electrical loads), RFC 0012 (heat sources) |

## Summary

Lamps a robot or a scene can carry: headlights, indicators and brake lights, status LEDs,
spotlights, rotating and flashing beacons, and street and traffic lights. A lamp has a luminous
intensity in candela over a beam, a colour temperature or colour, a rise time, and blink and
beacon patterns. It draws power from its bus (an existing `ElectricalLoadAPI_1` of model
`signal`), heats its thermal node (the existing `loss_power` convention), lights what cameras
see at night in the realistic renderer, and is measured by the existing `light_sensor` kind. It is
switched and dimmed from ROS 2, ros2_control GPIO, Autoware's indicator commands, or a plugin.
The renderer's sun comes from the site's date, time and place, so a night actually happens.
The changes are one new schema library, one small domain, and additions to the renderer and
sensors. No protocol or ABI change.

## Motivation

The owner asked for "a lamp plugin, so a robot can have lights". Today nothing in Lagrange emits
light but the sun, and the sun is fixed. `gpu/raster/src/Realistic.cpp` draws every frame with
the constant `kSun{0.52, 0.34, 0.78}`, and the weather's `env/sun_direction` is an authored
constant (`weather:sunDirection`). So there is no night for cameras, and nothing a robot carries
can light the way. Users need lights to:

- test perception at night and at dusk (a car's headlights on the road, an AMR's spotlight in a
  dark warehouse aisle, a drone's searchlight);
- show signals other agents read (indicators, brake lights, status LEDs, beacons on AGVs,
  traffic lights at night in the town);
- count the lights' power and heat (a drone's floodlight is a large share of its power budget).

## Non-goals

- Global illumination, light bouncing between surfaces, and volumetric light shafts through fog
  in the first version (the fog veil the cameras already have darkens the scene but does not
  scatter lamp light; open question 9).
- Photometric files beyond a cone in phase 1 (IES LM-63 profiles are "also worth adding").
- Lidar interference from lamps. Lidars work at 905/1550 nm, and visible lamps are not modelled
  in those bands.
- Light pollution, sky glow and moonlight phases beyond one night-sky illuminance (§6).

## Design

### 1. A plugin is not enough (and why)

What a plugin can do with ABI 1.1 today: read reals of a registered schema
(`world_schema_real`), write `load/<n>/power_demand` (power), write `<class>/<node>/loss_power`
(heat, the thermal network picks it up by name), and take commands from ROS through
`--ros-topics` `subscribe`. What it cannot do: put light into a camera image, into the
`light_sensor`, or into the viewer. Both renderers live in Lagrange (`gpu::RealisticRenderer` for
cameras and viewport, invariant 13), and neither has a way to take light from a module. So the
lamp itself must be built in, as data plus a small domain. A plugin stays useful for what a lamp
*does*: custom flash patterns, a robot's status logic. It writes the lamp's command signal, or
it replaces the domain as a substitute under RFC 0008 (§8).

### 2. The data: `LampAPI_1` (new library `schemas/light`, namespace `lamp:`)

Applied to a prim whose frame places the lamp (the light leaves along its +X axis, the forward
axis of URDF links and cameras' bodies; open question 2). It moves with its body.

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `lamp:kind` | token | | `spot` | `spot` (a cone), `point` (all round), `beacon` (a spot turning about the prim's +Z) |
| `lamp:luminousIntensity` | double | cd | required | on the axis, at full command |
| `lamp:beamAngle` | double | rad | required for `spot`/`beacon` | full angle where intensity falls to 50 % of the axis value |
| `lamp:fieldAngle` | double | rad | 2 × `beamAngle` | full angle where it falls to 10 %; zero beyond |
| `lamp:colorTemperature` | double | K | 0 | correlated colour temperature; 0: use `lamp:color` |
| `lamp:color` | color3f | 1 | 1, 1, 1 | linear sRGB chromaticity of a coloured lamp (LED indicators, traffic lights), normalised to its luminance |
| `lamp:luminousEfficacy` | double | lm/W | required when powered | electrical power per lumen emitted |
| `lamp:heatFraction` | double | 1 | 1 | share of the electrical power that heats the lamp's own thermal node |
| `lamp:riseTime` | double | s | 0 | first-order lag of the light output (a filament's warm-up; LEDs ≈ 0) |
| `lamp:blinkFrequency` | double | Hz | 0 | when blinking |
| `lamp:blinkDuty` | double | 1 | 0.5 | |
| `lamp:rotationSpeed` | double | rad/s | 0 | `beacon` |
| `lamp:voltageExponent` | double | 1 | 0 | output ∝ (V/V_nom)^n below nominal (0: constant-current LED driver) |
| `lamp:initialCommand` | double | 1 | 0 | brightness at t = 0 |

The 50 % and 10 % definitions of beam and field angle are those of ANSI C78.379 for reflector
lamps (TODO(verify) the clause: ANSI C78.379 is sold, not openly readable, and was not opened).
The intensity between them falls smoothly (a cosine-squared
taper, a drawing choice to replace with a measured profile when LM-63 comes). Required numbers
are the lamp's datasheet values. No fallback is a device value.

A lamp that draws power also gets `ElectricalLoadAPI_1` with `elec:load:model = signal` on the
same prim (it exists, RFC 0010). One that heats a thermal node needs a `ThermalAPI_1` node of the
same name, or a `HeatSourceAPI_1` with `power_signal` for the conduction fidelity.

**Interop with UsdLux.** OpenUSD's own lights (`SphereLight` with `ShapingAPI` cone angle,
`colorTemperature`; part of OpenUSD 26.08, which Lagrange vendors: `pxr/usd/CMakeLists.txt` at
tag v26.08 builds `usdLux` unconditionally, beside `usdGeom`, and `openusd_vendor` turns off only
imaging, so the vendor build has it) are what other USD tools render. Corrected: UsdLux's
intensity is photometric. Its "Quantities and Units" section defines a light of `intensity` 1 and
white `color` as one of 1 nit (cd/m²) of luminance at its surface. That is not what a datasheet
gives (luminous flux in lm, or intensity in cd): converting needs the emitting area and the
ShapingAPI cone, so Lagrange keeps the datasheet's values in `lamp:`. The stage writer (`lagrange_import`, the builder) also writes a matching
`SphereLight` under the prim for other tools, and the loader reads a bare UsdLux light (no
`LampAPI_1`) as a lamp with a warning that its intensity is a guess (open question 3).

**URDF and SDFormat.** URDF: `<gazebo reference="L"><lamp name=... kind=spot intensity=...
beam_angle=... color_temperature=... efficacy=... xyz=... rpy=.../></gazebo>` (a frame on link L).
SDFormat's `<light>` inside a `<link>` (allowed from SDFormat 1.6, whose `link.sdf` is the first to
include `light.sdf`; `<intensity>`, added in 1.8, is "Scale factor to set the relative power of a
light", with no unit; from the spec files in `gazebosim/sdformat`) is read as a lamp: `spot` from `<spot><outer_angle>`, colour from
`<diffuse>`. Its intensity has no photometric unit, so a `lamp:luminousIntensity` must be given
in an overlay, or the loader warns and uses a stated placeholder. Overlay YAML: a `lamps:` list
(formats.md §9 applies any schema).

### 3. The model: domain `lights` (`modules/lights`, 10 ms)

Per lamp `<n>`:

```
c       = clamp(command, 0, 1)·blink(t)·powered          (blink: square wave of frequency, duty)
I_out  += (c·I_v·g(V) − I_out) · (1 − e^(−dt/riseTime))     (I_v: lamp:luminousIntensity)
g(V)    = (V/V_nom)^n for V < V_nom, else 1                (n: lamp:voltageExponent)
Φ       = I_out · Ω_eff;   Ω_eff = ∫ f(θ) dΩ over the beam  (f: the cone's taper, f(0) = 1)
P_elec  = Φ_full · c / η_v                                  (η_v: lamp:luminousEfficacy)
P_heat  = lamp:heatFraction · P_elec
```

The step is 10 ms, so a 1.5 Hz indicator's edges land within 10 ms. `Ω_eff` is computed once at
load. For a sharp cone of full angle `2α` it is `2π(1 − cos α)`, the closed form the test checks.
`P_elec` is drawn even while ramping, so a filament's inrush is not modelled (open question 7).
`powered` is `load/<n>/powered` (a brownout turns lamps off, as it does sensors).

**Signals.**

| Signal | Type | Unit | |
|---|---|---|---|
| `lamp/<n>/command` | Scalar | 1 | in: brightness 0–1, held between writes (ROS, ros2_control, plugin, Autoware) |
| `lamp/<n>/blink` | Bool | 1 | in, optional: blink while true |
| `lamp/<n>/intensity` | Scalar | cd | out: on the axis, now (what renderers and sensors read) |
| `lamp/<n>/on` | Bool | 1 | out: emitting |
| `load/<n>/power_demand` | Scalar | W | out: to the electrical network |
| `lamp/<n>/loss_power` | Scalar | W | out: to the thermal node named `<n>` (the existing `<class>/<node>/loss_power` rule) |

Units: candela and lux are not in `lagrange::units` today (the light sensor defines a local
`kLux`). This RFC adds luminous intensity as a dimension (cd, and lx = cd·sr/m²). That is a C++
change, not a frozen surface. The recorded unit string is new text, which old readers display as
is (open question 6).

**Substitutes.** `zero`: every lamp dark, no power, no heat (a run that does not care about
light). `constant`: every lamp at its `initialCommand`, ignoring commands. `replay`:
`lamp/*/intensity` from MCAP.

### 4. What cameras see

`gpu::RealisticRenderer` (cameras and the viewport, invariant 13) gets up to `kMaxLamps` (64,
open question 5) lamps per frame in a storage buffer: world-local position, axis, cosines of
beam and field angles, intensity in cd, linear RGB from the colour temperature or colour.
Per fragment, the illuminance from each lamp in reach is added to the sun's:

```
E = I_out · f(θ) · cos(incidence) / d²          (inverse-square law; d beyond a near limit)
```

The colour of a colour temperature is Planck's law weighted by the CIE 1931 colour-matching
functions and converted to linear sRGB, tabulated at load. The functions come from CIE's own
dataset, "CIE 1931 colour-matching functions, 2 degree observer" (DOI 10.25039/CIE.DS.xvudnb9b,
360–830 nm in 1 nm steps, from CIE 018:2019 Table 6, with CIE 015:2018 *Colorimetry* 4th ed.,
which replaces the CIE 15:2004 named here before), published under CC BY-SA 4.0. A table derived
from it carries that licence: it is recorded in `docs/design/dependencies.md` and REUSE before it
ships, or the loader reads the CSV at build time. The renderer works
today in units "by eye" (`kSunStrength`, `kExposure`). It gets one calibration: renderer units
per lux, fixed so that the sun's illuminance is the IES daylight value the `light_sensor`
already computes (`kExtraterrestrialIlluminance`, `sensors::surfaceIlluminance`). Then the
existing camera model's exposure, auto exposure and noise (`CameraSensorAPI_1`) give a dark,
noisy night image and a correctly bright headlight pool from the same numbers. The lamp's
own emitter is drawn as an emissive disc on its prim, so a camera sees the lamp itself (bloom
is a drawing choice for the viewport only, not for camera sensors).

Shadows from lamps: none in phase 1 (light passes through walls into the next aisle, a stated
limit). Phase 2: shadow maps for the brightest spot lamps per frame (Tier 1 and 2), and
ray-query shadows on Tier 2 hardware (open question 8).

**Tiers.** Tier 2 and 1: as above. Tier 0 (software Vulkan, `simple` camera look): the simple
look adds lamps with the same formula, unshadowed. A run without a usable Vulkan device has no
camera images at all, as today. Depth and instance images do not change. Thermal cameras see
lamps only as their thermal node's temperature.

### 5. What light sensors measure

The `light_sensor` kind adds each lamp's `E = I_out·f(θ)·cos(incidence)/d²` to the daylight it
computes, with one occlusion ray per lamp through the scene's ray query (CPU, deterministic, all
tiers). This is the closed form the renderer is checked against (§Validation).

### 6. Night: the sun from the site's date, time and place

- The weather's `env/sun_direction` comes from the NOAA solar position algorithm the
  `light_sensor` already uses (`SunCommon.hpp`), given the site's start time (new site value
  `start_utc`, `--start-utc YYYY-MM-DDTHH:MM:SSZ`, profile `world.start_utc`) and the geo origin.
  Without both, it stays the authored constant (today's behaviour). The weather also writes
  `env/sky_illuminance` (lx, horizontal), from the same IES daylight model by day and a
  night-sky value at night: 0.8 mlx, the US National Park Service's "natural reference condition
  for moonless nights ... for horizontal illuminance" (Night Skies Report metrics guide).
  Whether to model the moon's phase is open question 12.
- The renderer reads the sun from `env/sun_direction` and `env/sky_illuminance` instead of `kSun`.
  When the weather does not run it keeps `kSun` (today's look), so every existing run looks the
  same. The seasons' tints stay as drawing choices on top.
- Street lights, building lights and traffic lights in the sample town become `LampAPI_1` prims
  (traffic lights' colours driven by `traffic/lights`), so the town works at night.

### 7. The viewer (invariant 14: only through the protocol)

No protocol change is needed:

- Lamps arrive in the scene snapshot as prims whose `api_schemas` contain `LampAPI_1`. Their
  attributes come over `GetAttributes`, as the Electrical workspace reads its values.
- Their live intensity comes over the existing signal subscription (`lamp/<n>/intensity`).
  The sun comes from `env/sun_direction` and `env/sky_illuminance`.
- The viewport's Realistic shading uses the same renderer and calibration as cameras. Lit and
  the other shadings draw a lamp as a small glyph with its cone outline when selected (an
  overlay drawn by the viewer, per `docs/design/ui/`; the UI work follows
  `.claude/rules/viewer-ui.md`). The inspector shows the `lamp:` attributes with the existing
  `SetAttribute` editing.
- A *Lights* row in the Environment panel's time controls sets `start_utc` through the existing
  `SetAttribute` with `site:start_utc` (the site path the Environment panel already uses).

### 8. Plugins

ABI 1.1 is enough for plugins to *control* lamps: write `lamp/<n>/command` and `lamp/<n>/blink`,
and read lamp numbers through `world_schema_real` (the `lamp:kind` token is not readable, since
the ABI reads reals only; a plugin that needs it reads the lamp's name convention). A plugin can
also replace the `lights` domain as a substitute (`domains.lights.substitute: <name>`, RFC 0008)
if it writes `lamp/<n>/intensity` for every lamp. No ABI change (open question 10).

### 9. ROS 2

- Per lamp: `/lights/<n>/command` `std_msgs/Float64` (0–1), `/lights/<n>/blink` `std_msgs/Bool`,
  `/lights/<n>/state` `std_msgs/Float64` (the output as a share of full, latched, published on
  change). Through the existing `SignalTopics` (`--ros-topics`), with this default set written by
  the server so that nothing must be configured.
- ros2_control: a `<gpio name="<n>">` with command interface `brightness` and state interface
  `brightness` (with RFC 0020 §5's GPIO support in `LagrangeSystem`).
- Autoware: the interface already echoes `/control/command/turn_indicators_cmd` and
  `hazard_lights_cmd` as reports. With lamps named by convention on a `VehicleAPI_1` robot
  (`indicator_left`, `indicator_right`), it drives their `blink` instead, and reports what the
  lamps do. The sample sedan gets headlights, brake lights (on while braking, from
  `vehicle/<v>/brake`, unit 1, the signal `CarDrive` writes and `VehicleDynamics` reads) and
  indicators. The indicators' 1.5 Hz is the middle of UN Regulation No. 48's "flashing 90 ± 30
  times per minute" (paragraph 6.5.9, Revision 6 with the 03 series of amendments, as
  incorporated by reference in 49 CFR 571.108; later series may renumber it).

## Validation

| Check | Against | Test |
|---|---|---|
| Illuminance at a `light_sensor` from a spot, on axis and at 3 angles, 3 distances | `I·f(θ)·cos/d²` (inverse-square law) | within 0.1 % |
| Occluded lamp | a box between lamp and sensor | 0 lx from that lamp |
| Luminous flux of a sharp cone | `Φ = I·2π(1 − cos α)` | within 0.5 % (the integration) |
| Electrical draw | `P = Φ/η_v` through the bus: `load/<n>/current · V` | within 0.5 % |
| Lamp node temperature at steady state | `T = T_amb + P_heat/(h·A)` of the lumped node | within 1 % |
| Renderer calibration | a grey Lambertian plane under one spot, linear pixel values (before tone mapping) against `ρ·E/π` at 5 points | within 2 % (float, filtering) |
| Sun position | the NREL SPA published example the NOAA algorithm is already checked against (`test_kinds_light_sensor`) | as that test |
| Blink | edges of a 1.5 Hz, 50 % blink at the 10 ms step | within one step |
| Brownout | bus below `minVoltage` | lamp off, `on` false |
| Determinism | the same run twice | identical state hash (the lamp module and the light sensor are CPU; images are not hashed) |

## Compatibility under RFC 0007

| Surface | Change | Allowed because |
|---|---|---|
| Schemas | new `LampAPI_1` library | addition |
| Profile schema | `domains.lights`, `world.start_utc` | additions with defaults that keep today's behaviour |
| Protocol | none | |
| Plugin ABI | none | |
| Recordings | new signals and a new unit string | addition |
| Look of existing runs | unchanged unless `start_utc` is given or lamps exist | the renderer keeps `kSun` without the site's time |

## Steps (each a PR)

1. This RFC.
2. `LampAPI_1` schema, stage loading, URDF/SDFormat/overlay forms, `units::cd` and `lx`.
3. `modules/lights`: the model, signals, power, heat, substitutes, ROS topics, tests.
4. `light_sensor` sees lamps.
5. Renderer: lamps in Realistic and simple looks, the lux calibration, the golden test of
   §Validation, headless screenshots of a lit AMR in the dark warehouse.
6. The sun from `start_utc` (weather), `env/sky_illuminance`, the renderer reading them; the
   Environment panel's time row.
7. Samples: the sedan's lights with Autoware indicators, an AMR with a beacon and status LED,
   the town's street and traffic lights; the viewer's lamp glyphs; docs/physics/lights.md.
8. Phase 2: lamp shadows (shadow maps; Tier 2 ray queries), LM-63 profiles.

## Also worth adding

- **IES LM-63 photometric files** (`lamp:photometry = @file.ies@`) for measured beam patterns of
  real headlights and floodlights (ANSI/IES LM-63-19, the fifth revision, files tagged `IES:LM-63-2019`, per ANSI's summary;
  resellers list a 2025 reaffirmation, not confirmed at IES).
- **Infrared illuminators** (850/940 nm) for night cameras: a lamp with a band, seen by cameras
  whose band includes it, not by people.
- **Structured light and laser pointers** for depth cameras: a lamp with a pattern image.
- **Lamp failure** (a filament that burns out, an LED that dims with temperature) through the
  sensor-fault machinery.

## Open questions (with a proposed answer)

1. *Built in (a `lights` domain plus renderer support), or a plugin as asked?* Built in, for the
   reasons of §1. A plugin is still the way to give a lamp custom behaviour.
2. *Which axis is the beam?* +X of the lamp's prim (URDF's forward), stated in the schema doc.
   SDFormat's `<light>` points along −Z (`<direction>` defaults to `0 0 -1`, "only applicable for
   spot and directional lights", `light.sdf`), so the importer rotates it.
3. *Lagrange's own `lamp:` attributes, or UsdLux's?* Own, for photometric units from datasheets.
   Lagrange writes matching UsdLux for other tools and reads bare UsdLux lights with a warning.
4. *One schema for every lamp kind, or one per kind?* One, with `lamp:kind`. Indicators, LEDs
   and headlights differ only in numbers.
5. *How many lamps per frame?* 64 in a storage buffer, nearest and brightest first by
   `I/d²` at the camera, with a log line once when more exist. Enough for a car (about 12) and a
   street of lamps.
6. *Add luminous intensity as a unit dimension?* Yes (cd, lx), since the light sensor's local
   `kLux` is a workaround. The recorded unit strings `cd` and `lx` are new text.
7. *Filament inrush current?* Not modelled. A note in the docs; a later optional attribute.
8. *Lamp shadows in phase 1?* No: phase 2. Without them the formulas are exact for open scenes,
   and the limit is stated.
9. *Lamp light scattered by fog (a headlight's glare in fog, "veiling luminance")?* Phase 2, with
   the existing per-pixel fog veil: the in-scattered light of each lamp along the pixel's ray in
   a homogeneous medium has a closed form (single scattering). TODO(verify) the reference to use:
   not searched in this pass.
10. *Plugin ABI additions (a `world_lamp` getter, tokens)?* None now. A plugin controls lamps
    through signals. A getter would be an ABI MINOR append in its own RFC if a user needs it.
11. *Pre-approve the gated merges (schema) of steps 2–7 once each passes its tests?* Yes.
12. *Model the moon's phase in `env/sky_illuminance`?* Not in phase 1: the moonless 0.8 mlx at
    night, documented. Adding it needs the moon's position and
    phase beside the NOAA sun, and a cited value for its illuminance.

## References checked for this draft

- The repository at origin/dev 1.4.0: `gpu/raster/src/Realistic.cpp` (`kSun`, `kSunStrength`,
  `kExposure`, seasons), `gpu/kernels/camera/realistic.slang` (one sun, emissive),
  `modules/weather/atmosphere/src/Weather.cpp` (`weather:sunDirection` constant),
  `modules/sensors/src/kinds/LightSensor.cpp` and `SunCommon.hpp` (NOAA sun, IES daylight, local
  `kLux`), `schemas/electrical/schema.usda` (`ElectricalLoadAPI_1` model `signal`),
  `modules/thermal/lumped/src/ThermalNetwork.cpp` (`<class>/<node>/loss_power`),
  `proto/lagrange/viewer/v1/scene.proto` and `command.proto` (`GetAttributes`, signal
  subscription), `core/plugin_abi/include/lagrange_plugin.h` (ABI 1.1 functions),
  `bridges/autoware/src/AutowareInterface.cpp` (indicator commands echoed), and
  `vendor/openusd_vendor/CMakeLists.txt` (OpenUSD 26.08).
- Checked on 2026-11-17 against the sources below. Still unread and marked TODO(verify): ANSI
  C78.379 and a fog-glare reference.

## Sources

Read for the 2026-11-17 check:

- OpenUSD, UsdLux overview, "Quantities and Units".
  https://openusd.org/release/api/usd_lux_page_front.html
- OpenUSD `pxr/usd/CMakeLists.txt` at tag v26.08.
  https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/pxr/usd/CMakeLists.txt
- SDFormat specification files `sdf/1.5/link.sdf`, `sdf/1.6/link.sdf`, `sdf/1.7/light.sdf`,
  `sdf/1.8/light.sdf`, `sdf/1.12/light.sdf`. https://github.com/gazebosim/sdformat/tree/main/sdf
- CIE, "CIE 1931 colour-matching functions, 2 degree observer", dataset and its metadata
  (licence). https://cie.co.at/datatable/cie-1931-colour-matching-functions-2-degree-observer
- UN ECE Regulation No. 48, Revision 6 (E/ECE/TRANS/505/Rev.1/Add.47/Rev.6), paragraph 6.5.9, as
  incorporated by reference in 49 CFR 571.108.
  https://law.resource.org/pub/us/cfr/ibr/006/un.ece.uniform.1996.pdf
- ANSI blog, "Standard File Format for Electronic Transfer of Photometric Data (IES LM-63-19)".
  https://blog.ansi.org/ansi/standard-file-photometric-data-ies-lm-63-19/ (search summary only)
- US National Park Service, "Metrics Guide to Night Skies Report".
  https://www.nps.gov/subjects/nightskies/NSQmetrics.html
- This repository: `server/src/CarDrive.cpp` and
  `modules/vehicle_dynamics/src/VehicleDynamics.cpp` (`vehicle/<v>/brake`).
