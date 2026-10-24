# Schema changelog

Every change to a schema library (RFC 0002, "Versioning and migration"). Until product v1.0
every schema is version 1 and may change without a migration.

## Unreleased

- **Electrical library** (`schemas/electrical`): `PortAPI_1` (multiple-apply), `BatteryAPI_1`,
  `ElectricalMotorAPI_1`, `ElectricalBusAPI_1`, `ElectricalLoadAPI_1`,
  `ElectricalConverterAPI_1`, from the drafts in `docs/design/formats.md` §3.1. Additions to the
  drafts: `elec:conv:input` and `elec:conv:output` relationships on the converter (the draft had
  no topology for it), and a fallback of 0 V for `elec:load:minVoltage`.

## v0.2.0

- **Sensor library** (`schemas/sensor`): `SensorAPI_1`, `LidarAPI_1`, `ImuAPI_1`.
