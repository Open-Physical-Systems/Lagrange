# EMI from motors, heat conduction, thermal sensors: work queue

Started 2026-11-10. Each item is a PR into `dev`, tested locally (CI is off). Mark `[x]` when
merged, `[~]` while in progress, and write what is left under it.

## A. Heat conduction (RFC 0012)

- [x] A1. RFC 0012 accepted; schemas `ThermalMaterialAPI_1`, `HeatSourceAPI_1`, frozen test,
      formats.md
- [x] A2. URDF `<thermal_material>`, `<heat_source>` and overlay keys; `lagrange_import`; preset
      table with sources
- [x] A3. `conduction` fidelity: cells, conjugate gradients, surfaces, sources, signals, profile
      keys; analytic tests (erfc, fin, two-layer wall, lumped limit, grid convergence)
- [x] A4. Viewer: per-cell thermal shading (`thermal/cells` as FieldVolumes in each link's frame,
      `FieldVolume.prim_path`), not a new message; the inspector's min/max is still owed
- [x] A5. `lagrange foam-cht`, sample `led_heatsink`, comparison, docs/physics/heat-conduction.md

## B. EMI from motors (RFC 0013)

- [x] B1. Research: field of a motor and its leads from its currents (dipole, wire segments),
      PWM switching emission spectrum, published magnetometer interference data (compass
      interference vs current)
- [x] B2. RFC 0013 and schema additions (an EMI source driven by an electrical signal)
- [x] B3. Model: motor and lead fields from `motor/<m>/current`, `battery/<b>/current`, into the
      EMI domain's field sum; magnetometer, IMU, CAN and radio effects; tests against the
      Biot–Savart closed forms and a published reference
- [x] B4. Visualization: |B| slices with motor sources, an EMI effects view (each victim's error
      over time), docs

## C. Thermal sensors as a plugin

- [x] C1. Sample C plugin `lagrange_thermal_probe`: contact thermistor (time constant, noise,
      quantisation) and IR spot thermometer (cone, emissivity, apparent temperature) on
      `CustomSensorAPI_1` / URDF `<sensor type="thermistor|ir_thermometer">`; tests

Merged: #300 (schemas), #301 (URDF), #302 (conduction), #303 (viewer cells), #304 (thermal
probes), #305 (EMI wires, motor leads, Coupled into). Still owed: view factors for the conduction fidelity's radiation (faces that see each other,
fins: it radiates too much there, against OpenFOAM's fvDOM); GNSS and radio noise from PWM
switching (no general coupling model found, RFC 0013 open question 3); a finer OpenFOAM mesh (about 4 million cells) for the sample, whose 203k and 640k
meshes differ by 16 % in the sink's rise.

## D. Everything tested

- [x] D1. Full local check (2026-11-11 01:50 JST: 1196 tests; two suites timed out while an
      8-process OpenFOAM run shared the CPU and pass alone, 238 and 36; lagrange_autoware needs its
      Autoware underlay sourced): `colcon test --packages-above lagrange_core`, viewer headless,
      determinism; fix what fails
