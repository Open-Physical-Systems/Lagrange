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

### 2.1 Blade-element momentum (`bem`)

The `bem` fidelity computes a propeller's loads from what it is: its blades' chord and twist
along the radius, their sections' lift and drag, the number of blades and the hand, read from a
propeller file (formats.md §3.3.1) named by the rotor's `aero:rotor:propeller`.

- **Per annulus** (40 along the blade), the axial induced velocity `w` and the swirl `w_t` make
  the blade-element forces equal the momentum change of the air through the annulus:
  `dT = 4πrρF·|V + w|·w`, `dQ = 4πr²ρF·|V + w|·w_t`, with Prandtl's tip-loss factor `F`. The
  axial balance is solved by bracketed root finding (Illinois), the swirl by a few fixed-point
  passes. In descent through the rotor's own wake (the vortex-ring state, `−2w < V < 0`)
  momentum theory has no valid solution; the result there is the momentum solution, flagged.
- **Sections** over the full circle of angles of attack: linear lift with the Prandtl–Glauert
  factor when the speed of sound is given, a parabolic drag polar, a blend over 8° past stall
  into a flat plate (normal force `2·sin α`), which also covers reversed flow. So a propeller
  turning the wrong way (its hand against the motor's direction) or mounted back to front gives
  what a real one gives: little thrust for much power.
- **Hand and mount.** The rotor's `aero:rotor:direction` is how the motor turns it, seen from
  the front; the file's `hand` is how the propeller must turn to make thrust; `aero:rotor:mount`
  is `puller` (thrust along the joint axis, which points from the motor to the propeller) or
  `pusher` (against it). The loader warns when hand and direction disagree.
- **Cost.** Loads scale exactly with `ρ·(Ω²R² + V²)` in incompressible flow, so each propeller
  is solved once at load over the full circle of inflow angles `θ = atan2(V, ΩR)` (0.25°
  steps; with a speed of sound, also over tip Mach 0 to 0.9 in 0.1 steps, rows in parallel
  threads) and the step interpolates. Deterministic, and about 0.3 s per propeller at load.
- **Folding blades** (a file's `folding:` section: hinge radius, blade mass, stop angle), at
  either fidelity: each blade's angle on its hinge is quasi-static, where the moments of its
  thrust (at the BEM thrust-weighted radius; 0.75 R at `coefficients`) and its weight about the
  hinge balance the centrifugal moment `Ω²·sin β·(r_h·S1 + cos β·S2)` (S1, S2 the blade's mass
  moments about the hinge, mass along the blade as chord²·thickness). Stopped, the blades hang
  folded against their stop; spinning up they open, then cone a few degrees forward. Thrust and
  torque are the open propeller's times `cos β`; `rotor/<j>/blade_angle` drives the drawn
  blades. Quasi-static is right when the blade's flapping frequency (about Ω) is far above the
  vehicle's; hinge friction and the blades' own dynamics at spin-up are not modelled.
- **Inputs.** The axial speed is the hub's velocity along the thrust direction (still air).
- **Limits.** No edgewise flow yet (forward flight's in-plane velocity, the H-force and blade
  flapping), no wind at the rotors, no stall delay on rotating blades (so thrust near the
  static condition of a highly pitched propeller is under-predicted; §7.1), no Reynolds-number
  effects beyond what the file's section data are for.

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
| `bem` against measured propellers | NACA Report 640 (1938, public domain): propeller 5868-9, two blades, Clark Y, at 15° and 25°; §7.1 | CT within 8 %, CP within 6 % (or 0.003) over the attached-flow range |
| `bem` against momentum theory | ideal rotor (many blades, ideal twist, no drag): uniform inflow, figure of merit 1 | 3–4 % |
| Ground effect | Cheeseman–Bennett curve and published small-rotor ground-effect measurements | 10 % for `z/R` in 0.5–3 |
| Field sampling | analytic field baked to VDB | interpolation error bound from grid spacing |
| Hover power and step response of the sample quadcopter | flight log of a comparable real vehicle (to be sourced; Q16) | reported, with tolerance set from data spread |

### 7.1 The NACA Report 640 comparison

`test_bem`'s `NacaReport640` runs propeller 5868-9 (10 ft, two blades, Clark Y; geometry in
`assets/propellers/naca_5868-9.prop.yaml`) and compares CT and CP with the report's
measurements. Geometry and measurements were read off the report's charts (figures 2, 6 and
8) with the grid lines as reference: chord to about ±1.5 %, blade angle ±0.3°, CT and CP to
about ±0.001. Section data are the Clark Y's from NACA Report 502 (lift slope, zero-lift angle,
minimum drag, maximum lift), scaled per station for thickness as the file explains; nothing was
fitted to the propeller data.

| Blade angle | Range | CT | CP | Thrust vanishes |
|---|---|---|---|---|
| 15° | J = 0.3–0.7 | within 2–4 % | within 2–5 % (7 % at J = 0.2) | J = 0.82, as measured |
| 25° | J = 0.5–1.1 | within 0–7 % | within 0–5 % | J = 1.29, as measured |

Below J ≈ 0.5 at 25° the inner blade stalls. The real propeller keeps its thrust (CT 0.106 at
J = 0.3 against 0.087 computed) and absorbs more power, as rotating blades delay stall; the
model has no stall-delay correction yet, so that range is not part of the test.

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
- `bem` (§2.1): blade-element momentum on the propeller file's geometry, with hand and mount;
  the server draws such rotors as their blades (`rotor_aero::addPropellerBlades`, NACA four-
  digit sections from the file's thickness and drawing camber);
  checked against NACA Report 640 (§7.1). The built-in controller and the autopilot bridges'
  ESCs turn a pusher's motor the right way about its joint axis.
- Not yet: advance-ratio corrections at the `coefficients` fidelity, edgewise flow and blade
  flapping at `bem`, wind at the rotors.

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
