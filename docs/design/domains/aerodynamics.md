# Domain: aerodynamics, wind and rotor aero

Two modules share this document because they share the wind field: `modules/aerodynamics/`
(forces on links) and `modules/rotor_aero/` (propellers and rotors).

## 1. Scope and release

Drag and lift per link, wind and gusts, sampled CFD fields, and rotor thrust and torque with
ground effect. True CFD does not run in the loop (CLAUDE.md §11.7). Release: v0.4 with drones.

## 2. Models per fidelity level

Aerodynamics (`fidelity`):

| Level | Content |
|---|---|
| `drag` | Per link with `AeroAPI`: `F = −½·ρ·|v_rel|·(Cd ∘ A ∘ v_rel)` in the body frame, applied at the centre of pressure; angular damping. Wind = mean wind with a logarithmic height profile + turbulence |
| `drag_plus_field` | Same forces, but the air velocity at each link is sampled from a NanoVDB vector grid (`weather:windField`, baked from OpenFOAM) where one exists, and from the mean-wind model elsewhere |
| `lbm` | A GPU lattice Boltzmann solver (D3Q19) in a moving box around the robot provides the local velocity field. Small volumes only. Experimental; after v0.4 |

Turbulence: Dryden or von Kármán spectra shaped by forming filters driven by white noise
(counter-based RNG, so deterministic). Parameters follow the low-altitude model of
MIL-HDBK-1797 (scale lengths and intensities as functions of height and mean wind at 6 m).
The handbook must be cited in code with the equation numbers used.

Air density from the environment: `ρ = p / (R_specific·T)` with humidity correction;
`R_specific = 287.05 J·kg⁻¹·K⁻¹` for dry air.

Rotor aero (`fidelity`):

| Level | Content |
|---|---|
| `coefficients` | `T = CT·ρ·n²·D⁴`, `Q = CP·ρ·n²·D⁵ / (2π)` with `n` in rev/s; first-order inflow lag; thrust and drag variation with advance ratio through a linear correction; blade-flapping moment as a simple coefficient |
| `bem` | Blade-element momentum with the blade table (chord, twist, aerofoil polars); solves induced velocity per annulus; captures forward flight and descent better |

Ground effect: Cheeseman–Bennett correction `T/T∞ = 1 / (1 − (R / 4z)²)` for `z/R > 0.25`,
clamped below that. Height `z` is found by a downward ray cast against collision geometry.

## 3. Signals

```cpp
// modules/aerodynamics — declare()
for (auto& a : world.instancesOf("AeroAPI_1")) {
    auto b = bodyOf(a);
    d.input ({.name = fmt("body/{}/pose", b),  .type = SignalType::Pose,   .unit = units::m,       .feedback = true});
    d.input ({.name = fmt("body/{}/twist", b), .type = SignalType::Wrench, .unit = units::m_per_s, .feedback = true});
    d.output({.name = fmt("body/{}/aero_wrench", b), .type = SignalType::Wrench, .unit = units::N, .frame = "world_local"});
    d.output({.name = fmt("body/{}/airspeed", b),    .type = SignalType::Scalar, .unit = units::m_per_s});
}
d.input ({.name = "env/wind/mean",           .type = SignalType::Vec3,    .unit = units::m_per_s});   // weather
d.input ({.name = "env/wind/velocity_field", .type = SignalType::Field3D, .unit = units::m_per_s, .optional = true});
d.input ({.name = "env/air_density",         .type = SignalType::Scalar,  .unit = units::kg_per_m3});
d.output({.name = "env/wind/gust",           .type = SignalType::Vec3,    .unit = units::m_per_s});   // turbulence at the robot

// modules/rotor_aero — declare()
for (auto& r : world.instancesOf("RotorAPI_1")) {
    auto j = jointOf(r);  auto b = hubBodyOf(r);
    d.input ({.name = fmt("joint/{}/velocity", j),  .type = SignalType::Scalar, .unit = units::rad_per_s, .feedback = true});
    d.input ({.name = fmt("body/{}/pose", b),       .type = SignalType::Pose,   .unit = units::m,         .feedback = true});
    d.input ({.name = fmt("body/{}/twist", b),      .type = SignalType::Wrench, .unit = units::m_per_s,   .feedback = true});
    d.output({.name = fmt("rotor/{}/thrust", j),    .type = SignalType::Scalar, .unit = units::N});
    d.output({.name = fmt("rotor/{}/aero_torque", j), .type = SignalType::Scalar, .unit = units::N_m});
    d.output({.name = fmt("rotor/{}/wrench", j),    .type = SignalType::Wrench, .unit = units::N, .frame = "world_local"});
    d.output({.name = fmt("rotor/{}/induced_velocity", j), .type = SignalType::Scalar, .unit = units::m_per_s});
}
d.input({.name = "env/air_density", .type = SignalType::Scalar, .unit = units::kg_per_m3});
d.input({.name = "env/wind/gust",   .type = SignalType::Vec3,   .unit = units::m_per_s, .optional = true});
```

Several modules produce wrenches on the same body (aero, rotor, hydro). The single-writer rule
forbids them all writing `body/{b}/external_wrench`. A small core module, `wrench_sum`, reads
every `*/wrench` signal declared for a body and writes `body/{b}/external_wrench`. It is always
loaded and has no substitute. The rotor's reaction torque on the motor shaft
(`rotor/{j}/aero_torque`) is summed into the joint by the same mechanism through
`joint/{j}/load_torque`.

## 4. Step size

Aerodynamics: default 1 ms for drones (same as physics), 10 ms for ground vehicles. Rotor aero:
1 ms; `bem` may need the rotor module at 0.5 ms for small fast propellers.

## 5. Substitutes and contract

Aerodynamics contract: `body/{b}/aero_wrench`, `body/{b}/airspeed`, `env/wind/gust`.
Rotor contract: `rotor/{j}/wrench`, `rotor/{j}/aero_torque`.

| Domain | Token | Kind | Behaviour |
|---|---|---|---|
| aerodynamics | `zero` | constant | Zero wrench; airspeed = body speed; gust = 0 |
| aerodynamics | `replay` | replay | From MCAP |
| rotor_aero | `ideal` | ideal | `T = k·ω²` with `k` from `thrustCoeff` at sea-level density; no inflow dynamics, no ground effect |
| rotor_aero | `replay` | replay | From MCAP |

A drone cannot fly with rotor aero replaced by `zero`, so `rotor_aero` has no zero preset. The
profile validator rejects it with that explanation.

## 6. Placement

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Drag, turbulence, coefficients, BEM | CPU | CPU | CPU |
| Field sampling for a few links | CPU NanoVDB | CPU NanoVDB | CPU NanoVDB |
| Field sampling for many environments | CPU | GPU kernel `gpu/kernels/fields/sample_vec.slang` | same |
| LBM | unsupported | ≤ 64³ cells | supported |

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Terminal velocity of a sphere and a plate | closed form | 0.5 % |
| Turbulence spectra | analytic Dryden / von Kármán power spectral densities | within the 95 % confidence band of the periodogram |
| Static thrust and torque vs speed | published propeller test data (candidate: the UIUC propeller database; check its terms of use before redistributing any data) | 5 % with coefficients taken from the same dataset |
| Ground effect | Cheeseman–Bennett curve and published small-rotor ground-effect measurements | 10 % for `z/R` in 0.5–3 |
| Field sampling | analytic field baked to VDB | interpolation error bound from grid spacing |
| Hover power and step response of the sample quadcopter | flight log of a comparable real vehicle (to be sourced; Q16) | reported, with tolerance set from data spread |

## 8. Status (v0.4)

`lagrange_rotor_aero` (`modules/rotor_aero/`): `RotorAero` at the `coefficients` fidelity and its
`ideal` substitute (`RotorAero(true)`), over the `RotorAPI_1` joints of `schemas/aero`.

- The air density is a site parameter with no default (`rotor_aero.air_density`, kg/m³), as the
  ambient temperature is for thermal; the rotors refuse to start without it.
- Thrust acts along the joint axis on the rotor's link, at the link's origin; the wrench goes to
  `body/<link>/wrench/rotor` and `WrenchSum` adds it to the link's `external_wrench`. The shaft
  torque is the joint's `load_torque`, opposing the rotation.
- Inflow lag: first order with config `inflow_time_constant` (s), exact over the step; 0 (no lag)
  unless set, because a published time constant for the sample propeller is not in hand.
- Ground effect: Cheeseman–Bennett with the height above the world's ground plane only (scenery
  needs scene queries as a core service: the lidar's caster is in the sensors module, which
  rotor_aero may not use, invariant 1). Below z = 0.5 R the factor is held at its value there:
  the design's clamp at 0.25 R is where the formula diverges.
- Not yet: advance-ratio corrections in forward flight, blade flapping, `bem`.

`lagrange_aerodynamics` (`modules/aerodynamics/`): `AeroDrag` at the `drag` fidelity over
`AeroAPI_1` links: per-axis drag at the centre of pressure and angular damping against the air,
which is the mean wind of a logarithmic profile plus Dryden turbulence (MIL-F-8785C low altitude,
its forming filters integrated exactly over the step). Until the weather domain gives the site's
wind (v0.5), the wind is the aerodynamics domain's config: `wind_speed` at
`wind_reference_height`, `wind_from`, the terrain's `roughness_length` and `turbulence`, all
without defaults; the air density as for the rotors. Not yet: lift tables, VDB wind fields, the
gust in the rotors' inflow.

| §7 check | Status | Test |
|---|---|---|
| Terminal velocity | met, sphere within 0.5 % | `test_aerodynamics` |
| Turbulence spectra | variances of the three components within 5 % of σ² over 4000 correlation times, longitudinal autocorrelation within 0.03 of e^(−τV/L); the periodogram band test is owed | `test_aerodynamics` |
| Static thrust and torque | closed form met exactly; published propeller data owed (with the sample quadcopter's propeller) | `test_rotor_aero` |
| Ground effect | Cheeseman–Bennett met exactly for z/R 0.5 to 3; published small-rotor measurements owed | `test_rotor_aero` |
