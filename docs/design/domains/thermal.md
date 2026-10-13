# Domain: thermal

## 1. Scope and release

Component temperatures from lumped thermal networks: heating from electrical losses, cooling by
convection and radiation, solar load, and derating fed back to the electrical domain.
Release: v0.3 together with electrical.

## 2. Models per fidelity level

| Level | Content |
|---|---|
| `lumped` (only level before 1.0) | One node per `ThermalAPI` prim with capacity `C`. Heat flows: conduction through `ThermalLinkAPI` conductances, convection `h·A·(T − T_air)` with `h` increased by local airspeed, radiation `ε·σ·A·(T⁴ − T_sky⁴)`, solar `α·A_sun·G`, internal sources from loss-power signals |

Stefan–Boltzmann constant `σ = 5.670374419e-8 W·m⁻²·K⁻⁴` (CODATA 2018 exact value), named and
cited in code.

Integration: backward Euler on the linearised network (radiation linearised around the current
temperature each step), solved as a small dense or sparse linear system. Unconditionally stable,
so a 100 ms step is safe even for small nodes.

Derating: for a node with `thermal:deratingStart` (`T1`) and `thermal:maxTemperature` (`T2`),
`derating = clamp((T2 − T) / (T2 − T1), 0, 1)`. At `T ≥ T2` a fault flag is raised.

## 3. Signals

```cpp
// modules/thermal/lumped — declare()
for (auto& n : world.instancesOf("ThermalAPI_1")) {
    auto p = name(n);
    // Heat sources: every loss_power signal whose prim is this node, discovered at wiring.
    for (auto& src : lossSignalsOn(n))        // e.g. "motor/{p}/loss_power", "battery/{p}/loss_power"
        d.input({.name = src, .type = SignalType::Scalar, .unit = units::W, .hold = Hold::WindowAverage});

    d.output({.name = fmt("thermal/{}/temperature", p), .type = SignalType::Scalar, .unit = units::K});
    d.output({.name = fmt("thermal/{}/derating", p),    .type = SignalType::Scalar, .unit = units::one});
    d.output({.name = fmt("thermal/{}/overtemp", p),    .type = SignalType::Bool,   .unit = units::one});
    d.output({.name = fmt("thermal/{}/heat_flow_out", p), .type = SignalType::Scalar, .unit = units::W});
}
d.input({.name = "env/air_temperature",  .type = SignalType::Scalar, .unit = units::K});          // weather
d.input({.name = "env/sky_temperature",  .type = SignalType::Scalar, .unit = units::K, .optional = true});
d.input({.name = "env/solar_irradiance", .type = SignalType::Scalar, .unit = units::W_per_m2,
         .optional = true, .defaultValue = 0.0});
d.input({.name = "env/sun_direction",    .type = SignalType::Vec3,   .unit = units::one, .optional = true});
for (BodyId b : bodiesWithThermalNodes())
    d.input({.name = fmt("body/{}/airspeed", b), .type = SignalType::Scalar, .unit = units::m_per_s,
             .optional = true, .defaultValue = 0.0});                                              // aerodynamics
```

## 4. Step size

Default 100 ms. Range 1 ms to 10 s.

## 5. Substitutes and contract

Contract: `thermal/{p}/temperature` and `thermal/{p}/derating` for every `ThermalAPI` prim.

| Profile token | Kind | Behaviour |
|---|---|---|
| `ambient` | constant | Every node at the environment's air temperature (or the profile's `ambient_temperature`); `derating = 1` |
| `replay` | replay | Temperatures from MCAP |

The ambient temperature has no code default: with neither an environment file nor
`ambient_temperature`, profile loading fails (site parameter rule).

## 6. Placement

CPU on all tiers. The viewer's thermal false-colour view maps node temperatures onto the prims
that own them; that is a viewer shader, not a module.

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Single node cooling | closed-form exponential | 0.1 % |
| Two-node network step | closed-form two-pole response | 0.5 % |
| Radiative equilibrium of a plate in sunlight | closed form | 0.5 K |
| Energy conservation | sum of stored energy and flows | 0.1 % |
| Motor heat run (with electrical) | published motor thermal test with winding temperature rise at constant load, or a recorded bench test (Q16) | within 3 K at steady state |
