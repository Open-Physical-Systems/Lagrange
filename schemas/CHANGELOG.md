# Schema changelog

Every change to a schema library (RFC 0002, "Versioning and migration"). Until product v1.0
every schema was version 1 and could change without a migration. From v1.0 they are frozen
(RFC 0007): within 1.x an attribute may be added and allowed tokens appended; anything else is
a `*_2` schema with a migration. `schemas/test/frozen_1_0.json` lists every attribute at 1.0
and `test_schemas.py` checks it.

## Unreleased

- **Sensor library** (RFC 0015): `flash` and `rosette` appended to `sensor:lidar:scanPattern`;
  `sensor:lidar:channelElevations` (a channel table) and five `sensor:lidar:rosette:*` (fov,
  pointRate, speed1, speed2, prismRatio; no fallbacks) on `LidarAPI_1`; `sensor:camera:lensType`
  (`gnomonical`, `equidistant`) and `sensor:camera:cutoffAngle` (no fallback) on
  `CameraSensorAPI_1`, the fisheye's k1..k4 being the existing `distortionModel = "equidistant"`
  and `distortionCoeffs`. Additions, nothing to migrate; a file that uses them, opened by
  Lagrange 1.2 or older, runs as a grid lidar and a pinhole camera.

- **EMI library** (RFC 0013): `wire` appended to `emi:sourceType`; `emi:path`, `emi:returnOffset`
  and `emi:currentSignal` on `EmiSourceAPI_1`. Additions, nothing to migrate.

- **Thermal library** (RFC 0012): `ThermalMaterialAPI_1`, the solid a link is made of (a preset or
  its conductivity, density, specific heat, emissivity, an optional conductivity along an axis,
  the contact conductance to its parent), and `HeatSourceAPI_1`, multiple-apply, heat put into a
  box of a link (power or a power signal, the share that leaves as light). Additions, nothing to
  migrate.

- **Tether library** (RFC 0011): `TetherAPI_1`, a cable from an anchor prim to a point on a body,
  with its winch. New library, nothing to migrate.

- **Electrical library** (RFC 0010): `ElectricalBranchAPI_1`, a resistor, fuse, switch or diode between two
  buses; `elec:load:resistance` on `ElectricalLoadAPI_1`, for a resistive load given as a resistance.
  Additions, nothing to migrate.

- **`ElectricalMotorAPI_1`**: `elec:velocityLoopBandwidth`; `elec:controlMode` `velocity` and `voltage` now simulated, on prismatic joints too. Addition, nothing to migrate.

- **Joint library**: `JointDynamicsAPI_1` on a physics joint: damping, dry friction and copying another joint (mimic).
  New library, nothing to migrate.

- **`LidarAPI_1`**: `sensor:lidar:motionDistortion` (bool, default true), as `formats.md` already listed it.
  Addition, nothing to migrate.

- **Water library**: `WaterAPI_1`, a box of still water (size, depth, density, drag, reflectivity). New
  library, nothing to migrate.

- **`SensorMaterialAPI_1`** gains `sensmat:preset` (a named surface for lidars). An addition; lidars
  now read the schema's reflectivity, specular fraction, transmission and retroreflective values.

- **Human library** (RFC 0009): `HumanAPI_1`, a person who walks (height, speed, behaviour,
  waypoints). New library, nothing to migrate.

- **`CustomSensorAPI_1`** in the sensor library (RFC 0008 §3): a sensor a plugin implements, by
  its `sensor:custom:kind`, with free `sensor:custom:param:<name>` attributes. A new schema under
  the 1.x rules (RFC 0007); nothing in `frozen_1_0.json` changes.

- **Frozen at 1.0** (RFC 0007): the 27 schemas and 240 properties listed in
  `schemas/test/frozen_1_0.json`.

- **World library** for v0.7 (huge-worlds.md §2): `WorldTileAPI_1`, one tile of a tiled world
  (level, index, bounds in world ENU, geometric error, whether it has colliders) on a prim whose
  payload holds the tile. New library, nothing to migrate.

- **`CameraSensorAPI_1`** gains `electronRate` (e⁻/s at the renderer's full white: the light
  level the raster level does not model) and `vignetting` (fraction of the cos⁴θ fall-off) for
  the v0.6 sensor model (formats.md §3.8); `exposureTime` is now the longest exposure with
  `autoExposure`. Additions with no effect on existing files.

- **`RadarAPI_1`** in the sensor library for v0.6 (formats.md §3.8): carrier frequency, range,
  fields of view, resolutions, transmit power, antenna gain, effective noise floor, detection
  threshold; `bandwidth` as drafted is left out (the range resolution stands for it).

- **Road library** for v0.6 (formats.md §3.11): `RoadLaneAPI_1`, one lane of a road network
  (centre line in the direction of travel, widths, type, speed limit, OpenDRIVE road and lane
  ids, the section's range along the road, successors and predecessors), written by the
  OpenDRIVE importer.

- **Vehicle library** for v0.6 (formats.md §3.10): `VehicleAPI_1` on a chassis body
  (powertrain, steering, brakes, anti-roll bars, the engine map and gears of an `ice_map`
  powertrain) and `WheelAPI_1` on a wheel's spin joint (radius, axle, steered and driven, tyre
  file or linear stiffnesses, friction, relaxation length, suspension).

- **Weather, EMI, sensor-material and comms libraries** for v0.5: `WeatherAPI_1`,
  `EmiSourceAPI_1`, `EmiSusceptibilityAPI_1`, `SensorMaterialAPI_1` as drafted in formats.md
  §3.4, §3.5 and §3.7, and new `RadioLinkAPI_1` and `CanBusAPI_1` (§3.9). Additions to the
  drafts: `weather:roughnessLength`; `emi:direction`, `emi:dipoleMomentPerAmpere`;
  `emi:adcNoisePerField` split into `emi:accelNoisePerField` and `emi:gyroNoisePerField`.

- **Aerodynamics library**: on `RotorAPI_1`, `aero:rotor:propeller` (a propeller file,
  formats.md §3.3.1, for the `bem` fidelity and the drawn blades) replaces the unused
  `aero:rotor:bladeTable`; new `aero:rotor:mount` (`puller` or `pusher`, fallback `puller`);
  `aero:rotor:direction` now documented as the sense in which the motor turns the rotor, seen
  from the front. `RotorAPI_1` has not been released, so no migration.

- **Sensor library**: `GnssAPI_1`, `MagnetometerAPI_1` and `BarometerAPI_1` for v0.4's drones,
  each with its white-noise standard deviations (formats.md §3.8 lists the sensor kinds but no
  attributes for these).

- **Aerodynamics library** (`schemas/aero`): `AeroAPI_1` and `RotorAPI_1`, as drafted in
  `docs/design/formats.md` §3.3, for v0.4's drones. Additions to the drafts: `lagrangeUnit = "1"`
  on `aero:rotor:bladeCount`, and the meaning of `aero:rotor:direction` (the sense of rotation
  that makes thrust, seen from above along the thrust direction).

- **Sensor library**: `CameraSensorAPI_1`, as drafted in `docs/design/formats.md` §3.8, applied
  to a `UsdGeomCamera` whose lens attributes give the field of view and clip range.

- **Thermal library** (`schemas/thermal`): `ThermalAPI_1` and `ThermalLinkAPI_1`
  (multiple-apply), as drafted in `docs/design/formats.md` §3.2.

- **Electrical library** (`schemas/electrical`): `PortAPI_1` (multiple-apply), `BatteryAPI_1`,
  `ElectricalMotorAPI_1`, `ElectricalBusAPI_1`, `ElectricalLoadAPI_1`,
  `ElectricalConverterAPI_1`, from the drafts in `docs/design/formats.md` §3.1. Additions to the
  drafts: `elec:conv:input` and `elec:conv:output` relationships on the converter (the draft had
  no topology for it), and a fallback of 0 V for `elec:load:minVoltage`.

- `elec:backEmfConstant`'s documented fallback corrected: torqueConstant / 1.5 for pmsm and bldc
  (Kt = 1.5·p·λ and Ke = p·λ in the peak dq convention), torqueConstant for dc_brushed; the
  draft said torqueConstant for all. Documentation only: the attribute still has no authored
  fallback, and the electrical network derives the value.

## v0.2.0

- **Sensor library** (`schemas/sensor`): `SensorAPI_1`, `LidarAPI_1`, `ImuAPI_1`.
