# Domain: radiation

## 1. Scope and release

Two different things carry this name in the project brief, and they are placed differently:

- **Thermal and solar radiation** (heat transfer) is handled by the thermal and weather domains.
  It is not part of this module.
- **Ionizing radiation** (this module): dose-rate fields from sources, shielding by scene
  geometry, accumulated dose per component, and single-event upsets (bit flips) injected into
  simulated devices.

Release: after v0.7 by default. The roadmap in CLAUDE.md does not assign it a release; see open
question Q2. Until then the domain exists as a contract with its `zero` substitute only, so that
profiles are complete.

Transport physics (scattering, energy spectra, neutron activation) is out of scope in the loop.
Geant4 or similar is used offline to produce dose fields that the module samples (CLAUDE.md
§11.7).

## 2. Models per fidelity level

| Level | Content |
|---|---|
| `point_kernel` | For each `rad:role = source` with `point_gamma`: dose rate at a point `Ḋ = Ḋ1m·(1 m / r)²·B·exp(−Σ μi·ti)`, where `ti` is the path length through each shield prim along the straight ray, `μi` its `rad:attenuationCoeff`, `B` a build-up factor. Path lengths come from ray casts |
| `field_map` | Dose rate sampled from a NanoVDB grid (`rad:doseField`) produced offline; shielding is already in the field |
| `mixed` | Field map plus point sources added |

Victims accumulate `D(t) = ∫ Ḋ dt`. When `D` exceeds `rad:totalDoseLimit` the victim's `failed`
flag is set, which downstream modules interpret (a camera stops, a motor controller faults).

Single-event upsets: the expected upset rate per victim is
`λ = σ_seu·N_bits·Φ̇`, with particle flux `Φ̇ = Ḋ / fluenceToDose`. Upset counts per step are
Poisson-distributed (counter-based RNG). Each upset is published as an event with a bit index;
bridges or device models decide what a flipped bit means (a corrupted sensor sample, a watchdog
reset). In v1 the only built-in consumer flips a bit in a sensor's output sample.

## 3. Signals

```cpp
// modules/radiation — declare()
for (auto& v : victims(world)) {                    // RadiationAPI_1 with rad:role = "victim"
    auto p = name(v);
    d.input ({.name = fmt("body/{}/pose", bodyOf(v)), .type = SignalType::Pose, .unit = units::m, .feedback = true});
    d.output({.name = fmt("rad/{}/dose_rate", p),     .type = SignalType::Scalar, .unit = units::Gy_per_s});
    d.output({.name = fmt("rad/{}/accumulated_dose", p), .type = SignalType::Scalar, .unit = units::Gy});
    d.output({.name = fmt("rad/{}/failed", p),        .type = SignalType::Bool,   .unit = units::one});
    d.output({.name = fmt("rad/{}/seu_events", p),    .type = SignalType::Bytes,  .unit = units::one});   // packed (time, bit index)
}
for (auto& s : sources(world))
    d.input({.name = fmt("rad_source/{}/enable", name(s)), .type = SignalType::Bool, .unit = units::one,
             .optional = true, .defaultValue = 1.0});
d.output({.name = "env/rad/dose_rate_field", .type = SignalType::Field3D, .unit = units::Gy_per_s});   // viewer overlay
```

## 4. Step size

Default 1 s. Dose changes slowly compared with robot motion. A robot moving at 1 m/s past a
steep gradient may need 100 ms; the module warns when the dose rate changes by more than 20 %
between steps.

## 5. Substitutes and contract

Contract: all `rad/{p}/*` outputs and the field.

| Profile token | Kind | Behaviour |
|---|---|---|
| `zero` | constant | Zero dose rate, zero accumulated dose, `failed = false`, no events |
| `replay` | replay | From MCAP |

## 6. Placement

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Shielding rays (victims × sources) | Embree | compute BVH kernel shared with lidar | ray query |
| Field sampling | CPU NanoVDB | CPU or GPU | CPU or GPU |
| Overlay volume | coarse CPU grid, low rate | GPU | GPU |

The number of rays is small (victims × sources), so the CPU path is adequate for single robots.
GPU paths matter for the overlay volume and for batches.

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Inverse-square law | closed form | 1e-9 relative |
| Slab shielding | `exp(−μt)` with attenuation coefficients from the NIST XCOM tables (public; cite the table and energy) | 1e-6 relative to the formula; coefficient values traced to the table |
| Build-up | tabulated build-up factors for point isotropic sources (ANSI/ANS-6.4.3 values as reproduced in open references; check availability) | within table interpolation error |
| Field-map sampling | analytic field baked to VDB | interpolation bound |
| SEU statistics | Poisson mean and variance over 1e6 steps | 3σ |
| Comparison with an offline transport code | a Geant4 run of a simple room with one source and one wall (run offline by a maintainer; results stored as a small reference table) | point-kernel within a factor of 2 outside the wall; the documentation states this accuracy level |

The factor-of-two statement is the honest accuracy of point-kernel methods for scattered fields
[Medium, general shielding-practice knowledge]. Users who need better accuracy use `field_map`.
