# Schema changelog

Every change to a schema library (RFC 0002, "Versioning and migration"). Until product v1.0
every schema is version 1 and may change without a migration.

## Unreleased

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
