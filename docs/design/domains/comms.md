# Domain: comms

## 1. Scope and release

Quality of wireless and wired links between simulated devices: Wi-Fi and generic radio links
(received power, signal-to-noise ratio, packet loss, latency), and wired buses that EMI can
disturb (CAN). The module computes link quality. Bridges apply it to the messages they carry.
Release: v0.5 with EMI.

## 2. Models per fidelity level

| Level | Content |
|---|---|
| `ideal` (substitute) | Every link delivers every packet with zero added latency |
| `link_budget` | Per link (transmitter prim → receiver prim): received power from transmit power, antenna gains, and path loss. Path loss is log-distance `PL(d) = PL(d0) + 10·n·log10(d / d0)` with exponent `n` per environment class (free space 2, indoor line-of-sight about 1.6–1.8, obstructed 3–4), plus a wall-count term from a ray cast (fixed loss per intersected prim by material), plus log-normal shadowing with spatial correlation. SNR from thermal noise `k·T·B` × noise figure plus `emi/{p}/interference_power`. Packet error rate from SNR through a modulation-dependent curve; throughput and retry-induced latency follow |
| `raytraced` (later) | Multipath from a small number of specular reflections using the shared BVH; fast fading statistics |

Boltzmann constant `k = 1.380649e-23 J/K` (exact SI value).

Applying the result: the ROS 2 bridge can drop or delay messages on topics mapped to a link (for
example, the fleet-management link of an AMR), and the MAVLink bridge can do the same for a
telemetry radio. A mapping file names which topics ride on which link. DDS discovery traffic is
not modelled.

## 3. Signals

```cpp
// modules/comms — declare()
for (auto& link : links(world)) {                       // RadioLinkAPI_1 (schema added with v0.5): tx prim, rx prim, band
    auto l = name(link);
    d.input ({.name = fmt("body/{}/pose", txBodyOf(link)), .type = SignalType::Pose, .unit = units::m});
    d.input ({.name = fmt("body/{}/pose", rxBodyOf(link)), .type = SignalType::Pose, .unit = units::m});
    d.input ({.name = fmt("emi/{}/interference_power", rxPrimOf(link)), .type = SignalType::Scalar, .unit = units::W,
              .optional = true, .defaultValue = 0.0});
    d.input ({.name = fmt("load/{}/powered", rxLoadOf(link)), .type = SignalType::Bool, .unit = units::one,
              .optional = true, .defaultValue = 1.0});
    d.output({.name = fmt("link/{}/rx_power", l),          .type = SignalType::Scalar, .unit = units::W});
    d.output({.name = fmt("link/{}/snr", l),               .type = SignalType::Scalar, .unit = units::one});   // linear ratio
    d.output({.name = fmt("link/{}/packet_error_rate", l), .type = SignalType::Scalar, .unit = units::one});
    d.output({.name = fmt("link/{}/latency", l),           .type = SignalType::Scalar, .unit = units::s});
    d.output({.name = fmt("link/{}/throughput", l),        .type = SignalType::Scalar, .unit = units::bit_per_s});
    d.output({.name = fmt("link/{}/up", l),                .type = SignalType::Bool,   .unit = units::one});
}
for (auto& bus : canBuses(world)) {
    auto c = name(bus);
    d.input ({.name = fmt("emi/{}/bit_error_rate", c), .type = SignalType::Scalar, .unit = units::one,
              .optional = true, .defaultValue = 0.0});
    d.output({.name = fmt("can/{}/frame_error_rate", c), .type = SignalType::Scalar, .unit = units::one});
    d.output({.name = fmt("can/{}/state", c),            .type = SignalType::Int,    .unit = units::one});   // active, passive, bus-off
}
```

Powers and ratios on the bus are linear SI values (W, dimensionless). Decibels appear only in
the UI. This avoids a `dBm` pseudo-unit in the unit system.

## 4. Step size

Default 10 ms.

## 5. Substitutes and contract

Contract: all `link/{l}/*` and `can/{c}/*` outputs.

| Profile token | Kind | Behaviour |
|---|---|---|
| `ideal` | ideal | `packet_error_rate = 0`, `latency = 0`, `up = true`, `can state = active`; SNR and power set to large finite sentinel values documented in the header |
| `replay` | replay | From MCAP |

## 6. Placement

CPU on all tiers. Wall-count ray casts use Embree on every tier (a handful of rays per link per
step). The `raytraced` level would use the shared BVH kernels on Tier 1 and 2.

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Free-space path loss | Friis equation | 1e-9 relative |
| Thermal noise floor | `k·T·B` | exact |
| Packet error rate curves | analytic bit-error expressions for the modelled modulations in additive white Gaussian noise | 1 % |
| Indoor attenuation | path-loss exponents and wall losses from published indoor propagation models (for example the ITU-R P.1238 recommendation; cite the version used) | parameters traceable to the source; the model is a range check, not a site prediction |
| CAN state machine | ISO 11898-1 fault confinement | exact |
| End to end | an AMR driving away from an access point in a real building with logged RSSI (to be recorded; Q16) | RSSI trend within the shadowing spread |
