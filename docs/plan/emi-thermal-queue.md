# EMI from motors, heat conduction, thermal sensors: work queue

Started 2026-11-10. Each item is a PR into `dev`, tested locally (CI is off). Mark `[x]` when
merged, `[~]` while in progress, and write what is left under it.

## A. Heat conduction (RFC 0012)

- [ ] A1. RFC 0012 accepted; schemas `ThermalMaterialAPI_1`, `HeatSourceAPI_1`, frozen test,
      formats.md
- [ ] A2. URDF `<thermal_material>`, `<heat_source>` and overlay keys; `lagrange_import`; preset
      table with sources
- [ ] A3. `conduction` fidelity: cells, conjugate gradients, surfaces, sources, signals, profile
      keys; analytic tests (erfc, fin, two-layer wall, lumped limit, grid convergence)
- [ ] A4. Viewer: `BodyTemperature`, per-vertex thermal shading, inspector
- [ ] A5. `lagrange foam-cht`, sample `led_heatsink`, comparison, docs/physics/heat-conduction.md

## B. EMI from motors (RFC 0013)

- [ ] B1. Research: field of a motor and its leads from its currents (dipole, wire segments),
      PWM switching emission spectrum, published magnetometer interference data (compass
      interference vs current)
- [ ] B2. RFC 0013 and schema additions (an EMI source driven by an electrical signal)
- [ ] B3. Model: motor and lead fields from `motor/<m>/current`, `battery/<b>/current`, into the
      EMI domain's field sum; magnetometer, IMU, CAN and radio effects; tests against the
      Biot–Savart closed forms and a published reference
- [ ] B4. Visualization: |B| slices with motor sources, an EMI effects view (each victim's error
      over time), docs

## C. Thermal sensors as a plugin

- [ ] C1. Sample C plugin `lagrange_thermal_probe`: contact thermistor (time constant, noise,
      quantisation) and IR spot thermometer (cone, emissivity, apparent temperature) on
      `CustomSensorAPI_1` / URDF `<sensor type="thermistor|ir_thermometer">`; tests

## D. Everything tested

- [ ] D1. Full local check: `colcon test --packages-above lagrange_core`, viewer headless,
      determinism; fix what fails
