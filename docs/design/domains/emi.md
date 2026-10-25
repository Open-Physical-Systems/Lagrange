# Domain: electromagnetic interference (EMI)

## 1. Scope and release

Low-frequency and radio-frequency disturbance fields from sources in the scene, and their effect
on sensors, buses and radios. The aim is behaviourally correct interference for testing robot
software (a magnetometer heading that swings near a power line, encoder glitches near a welder,
packet loss near a transmitter). It is not an electromagnetic compatibility certification tool,
and the documentation says so. Release: v0.5.

## 2. Models per fidelity level

Sources:

| Source type | Analytic model (no field map) | With `emi:fieldMap` |
|---|---|---|
| `power_line` | Infinite straight conductor: `B = μ0·I / (2π·d)`, direction by the right-hand rule, sinusoid at `emi:frequency` plus harmonics | NanoVDB vector grid of B per ampere, scaled by current |
| `motor_drive` | Magnetic dipole at the motor with moment proportional to phase current (from `motor/{p}/current_q` through `emi:drivenBy`); broadband E-field noise at the PWM frequency and harmonics | grid scaled by current |
| `transmitter` | Free-space far field: `E = sqrt(30·P_eirp) / d` (V/m, RMS); optional two-ray ground reflection | grid |
| `welder` | Pulsed dipole, duty cycle from `emi:dutyCycle`, broadband | grid |

Constants: `μ0 = 1.25663706212e-6 N·A⁻²` (CODATA 2018). Occlusion and shielding by scene geometry
are not computed in v0.5, except a per-victim `emi:shieldingEffectiveness` factor.

Victims (`emi.coupling` in the profile selects which are active):

| Victim | Effect model | Parameters |
|---|---|---|
| Magnetometer | Adds the instantaneous disturbance B-field (sampled at the sensor rate, so mains-frequency fields alias realistically) scaled by `emi:magneticGain` | gain |
| IMU / ADC | Adds Gaussian noise with RMS `adcNoisePerField·|E|` to accelerometer and gyro channels (in their output units through the sensor's scale factor) | noise per field |
| Encoder | Each edge is corrupted (extra or missing count) with probability `p = 1 / (1 + (E50/|E|)^4)`, where `E50 = emi:encoderGlitchField` | 50 % field |
| CAN bus | Bit-error rate rises with field: `BER = BER0·(|E| / E_ref)²`, capped at 0.5; frames with an error are retransmitted; error counters follow the CAN fault-confinement rules (error-passive at 128, bus-off at 256) | reference field and BER |
| Radio (Wi-Fi) | Interference power added to the noise floor of the `comms` link budget → SNR → packet error rate | noise figure, bandwidth overlap |
| Harness | Induced voltage `V = 2π·f·B·A_loop` for analogue lines | loop area |

The exponents and curve shapes for encoder and CAN are behavioural models chosen to give a
controllable threshold. They are not derived from first principles [Low confidence as physics;
stated as such in user documentation]. The per-robot calibration parameters exist so that a user
with measurements can fit them.

## 3. Signals

```cpp
// modules/emi — declare()
for (auto& s : world.instancesOf("EmiSourceAPI_1")) {
    if (auto drv = drivenBy(s))                    // e.g. motor current
        d.input({.name = fmt("motor/{}/current_q", *drv), .type = SignalType::Scalar, .unit = units::A,
                 .feedback = true});
    d.input({.name = fmt("emi_source/{}/enable", name(s)), .type = SignalType::Bool, .unit = units::one,
             .optional = true, .defaultValue = 1.0});           // scenario scripts switch sources
}
for (auto& v : world.instancesOf("EmiSusceptibilityAPI_1")) {
    auto p = name(v);
    d.input ({.name = fmt("body/{}/pose", bodyOf(v)), .type = SignalType::Pose, .unit = units::m, .feedback = true});
    d.output({.name = fmt("emi/{}/b_field", p),       .type = SignalType::Vec3,   .unit = units::T,       .frame = "body"});
    d.output({.name = fmt("emi/{}/e_field_rms", p),   .type = SignalType::Scalar, .unit = units::V_per_m});
    d.output({.name = fmt("emi/{}/bit_error_rate", p),   .type = SignalType::Scalar, .unit = units::one});   // can_bus
    d.output({.name = fmt("emi/{}/glitch_probability", p), .type = SignalType::Scalar, .unit = units::one}); // encoder
    d.output({.name = fmt("emi/{}/interference_power", p), .type = SignalType::Scalar, .unit = units::W});   // radio
}
d.output({.name = "env/emi/b_field_rms", .type = SignalType::Field3D, .unit = units::T});   // for the viewer overlay
```

The EMI module computes fields and victim-level disturbance quantities. The sensor, comms and
bridge modules apply them: the magnetometer adds `emi/{p}/b_field` to its reading, the ros2_control
bridge applies `glitch_probability` to encoder counts, comms uses `interference_power`. This keeps
each effect next to the model of the thing it corrupts.

## 4. Step size

Default 1 ms, so that 50/60 Hz fields and their third harmonic are sampled adequately. PWM-rate
content is represented by its RMS amplitude, not as a waveform.

## 5. Substitutes and contract

Contract: every `emi/{p}/*` output and `env/emi/b_field_rms`.

| Profile token | Kind | Behaviour |
|---|---|---|
| `zero` | constant | All fields and probabilities zero; empty grid |
| `replay` | replay | From MCAP |

## 6. Placement

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Analytic sources at victim positions | CPU | CPU | CPU |
| Field-map sampling | CPU NanoVDB | CPU for few victims; GPU kernel `gpu/kernels/fields/` for the overlay volume | same |
| Overlay volume for the viewer | coarse CPU grid at 1 Hz | GPU | GPU |

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Field of a straight conductor and of a dipole | closed form | 1e-6 relative |
| Far field of a transmitter | closed form | 1e-6 relative |
| Magnetometer near a power line | field strengths at stated distances from published measurements near overhead lines and from guideline documents (for example ICNIRP reference material); plus heading-error geometry | field within the published range for the stated current and distance |
| Mains aliasing | a 60 Hz field sampled at 100 Hz produces a 40 Hz alias in the magnetometer signal | spectral peak location exact |
| CAN error handling | CAN fault-confinement rules from ISO 11898-1 | counter transitions exact for injected error sequences |
| End to end | a recording from a real robot near a known source (to be recorded; Q16) | calibrated parameters reproduce the heading-error amplitude within 20 % |

Site rule: `emi:frequency` has no fallback. Sample scenes state their grid frequency in the
scene file with a comment naming the region.

## 8. Status (v0.5)

`lagrange_emi` (`modules/emi/`): `Emi` at 1 ms and its `zero` substitute (`Emi(true)`). It runs
when the stage has `EmiSourceAPI_1` or `EmiSusceptibilityAPI_1` prims; `emi.coupling` in the
profile selects the victims (`magnetometer`, `imu`, `encoder`, `can_bus`, `wifi`; all when
absent). Sources and victims are placed by their prims: the loader records each schema prim's
body and its pose in it (`SchemaInstance::body`, `poseInBody`), so a source on the robot moves
with it.

- Sources: the power line (infinite straight conductor), the motor drive and the welder
  (dipoles along `emi:direction`, `emi:dipoleMomentPerAmpere` times the current; a motor drive's
  current is `motor/<m>/current_q` of its `emi:drivenBy` motor when the electrical network runs,
  else `emi:current`), the transmitter (far field `√(30·EIRP)/d`), and NanoVDB field maps in the
  source prim's frame (B scaled by the current over `emi:current`, or E). Alternating sources
  carry `√2·(sin ωt + Σ h_k sin kωt)`; the welder's duty cycle is an RMS weight (its arc's on/off
  timing is not modelled). Distances under 1 cm are taken as 1 cm. No two-ray ground reflection,
  occlusion or low-frequency E-field yet.
- Site rule: a power line's and a welder's `emi:frequency` is required, never defaulted.
- Victims write `emi/<p>/b_field` and `e_field_rms`, and by type `accel_noise`/`gyro_noise`,
  `glitch_probability`, `bit_error_rate`, `interference_power` (the received power of in-band
  sources: `E²/Z0·λ²/(4π)` times the band overlap with the radio's `RadioLinkAPI_1` band), plus
  `induced_voltage` (`2π·f·B·A`, RMS) for a loop area.
- Applied (`lagrange_sensors`): the magnetometer adds `b_field` at its own rate (a 60 Hz field
  sampled at 100 Hz shows at 40 Hz, `test_emi`), the IMU adds `accel_noise` and `gyro_noise` from
  its own random stream. Optional inputs: without the EMI domain they read zero. An encoder
  victim on a joint's prim (`emi/<joint>/glitch_probability`) corrupts the counts of that
  joint's encoder in the ros2_control bridge (`encoder_resolution`): each edge, an extra or a
  missing count that stays in the counter. CAN errors and the radio's SNR are applied by the
  comms domain (comms.md §8).
- The viewer: the server lists the sources in `EnvironmentState` and slices `emi/flux_density`
  (|B| RMS, sources added in power) analytically; `env/emi/b_field_rms` as a Field3D signal is
  not written yet.
- Sample: `assets/worlds/power_line/power_line.usda` (400 A at 60 Hz, 10 m up) and
  `assets/robots/quad_x500/quad_x500_emi.usda`, run by `profiles/examples/quad_x500_power_line.yaml`:
  hovering 2 m up, 9.9 m from the line, the magnetometer sees 8.1 µT RMS, as `μ0·I/(2πd)` gives.
- Validation done: the conductor, dipole and far-field closed forms (1e-6), the aliasing, Friis
  for the interference power. Owed: field strengths near real lines against published
  measurements, and the end-to-end recording (Q16).
