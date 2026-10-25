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

## 8. Status (v0.3)

`lagrange_thermal` (`modules/thermal/`): `ThermalNetwork` (`lumped/`, default step 100 ms) with
the network solver in `Lumped.hpp`, and the `AmbientTemperature` substitute. The schemas are in
`schemas/thermal` (`ThermalAPI_1`, `ThermalLinkAPI_1`). The sample AMR's overlay gives the
motors, the battery and the chassis thermal nodes, with links to the chassis.

The server runs thermal when the robot has `ThermalAPI_1` prims **and** the site's ambient
temperature is known: the profile's `thermal.ambient_temperature` or `--ambient-temperature K`.
Without one nothing stands in, and the electrical network uses its components' reference
temperatures; the domain panel says why. `--domain thermal=on` without it is an error.

Choices made while implementing, beyond §2–§3:

- Heat sources are found by name: every `<class>/<node>/loss_power` declared before the module,
  read with `Hold::WindowAverage` so a 100 ms step sees the mean of the electrical network's
  100 µs losses.
- `env/air_temperature` and `env/sky_temperature` are optional inputs whose default is the
  ambient temperature; a weather module will write them. `env/solar_irradiance` defaults to 0.
- `heat_flow_out` is the node's convection and radiation to the air and the sky (links and the
  sun not included), computed with the same linearisation as the step, so energy balances
  exactly.
- Not yet: airspeed (forced convection, with the aerodynamics domain) and the sun's direction
  (the sun-facing area is taken as facing it).

| §7 check | Status | Test |
|---|---|---|
| Single node cooling | met, 0.1 % | `test_lumped` `ASingleNodeCoolsExponentially` |
| Two-node network step | met, 0.5 % | `TwoNodesFollowTheTwoPoleStepResponse` |
| Radiative equilibrium of a plate in sunlight | met, 0.5 K | `ASunlitPlateReachesItsRadiativeEquilibrium` |
| Energy conservation | met, every step to 1e-9 | `EnergyIsConserved` |
| Motor heat run | the closed-form steady state of a motor linked to the chassis is met (`test_thermal_network`); the comparison with a published or recorded heat run is owed (Q16) | — |
