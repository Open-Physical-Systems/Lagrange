# Schema changelog

Every change to a schema library (RFC 0002, "Versioning and migration"). Until product v1.0
every schema is version 1 and may change without a migration.

## Unreleased

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
