# Domain: weather

## 1. Scope and release

The state of the atmosphere and its effect on sensors and surfaces: air temperature, pressure,
humidity, mean wind, fog, rain, dust, sun. The weather module owns the environment signals
(`env/*`) that thermal, aerodynamics, sensors and rigid body read. Release: v0.5. A minimal
version that only publishes constant `env/*` values from `WeatherAPI` ships with v0.3 because
thermal needs air temperature.

## 2. Models per fidelity level

The module has `effects` switches (profile key) instead of a single fidelity ladder.

| Effect | Model | Consumed by |
|---|---|---|
| Atmosphere state | Values from `WeatherAPI`; optional time series from a CSV; density from the ideal gas law with humidity | thermal, aerodynamics |
| `fog` | Extinction coefficient `σ` (1/m). Uniform from visibility by Koschmieder (`σ = 3.912 / V`, a 2 % contrast threshold; the WMO's meteorological optical range uses 5 %, `σ = 3.0 / V`), or a NanoVDB grid. Lidar: two-way Beer–Lambert attenuation `exp(−2σr)` plus a backscatter return whose probability rises with `σ`. Camera: per-pixel `L = L0·exp(−σd) + L_air·(1 − exp(−σd))` | sensors |
| `rain` | Drop size distribution Marshall–Palmer `N(D) = N0·exp(−Λ·D)`, `N0 = 8000 m⁻³·mm⁻¹`, `Λ = 4.1·R^−0.21 mm⁻¹` with `R` in mm/h (Marshall and Palmer 1948). Gives extinction for lidar and camera, a rate of spurious near-range lidar returns from individual drops, radar attenuation, drops on lenses (accumulation rate proportional to rain rate and exposed area), and wetness of surfaces | sensors, rigid body |
| `dust` | Mie-regime extinction from mass concentration and median diameter using tabulated extinction efficiency; lens accumulation; a slow ingress counter per `SensorAPI` prim | sensors |
| `wet_friction` | `env/friction_scale` and per-material `sensmat:frictionWetScale`, rising with accumulated wetness and decaying with drying time | rigid body, vehicle dynamics |
| `lens_contamination` | Per-sensor contamination state (0…1) that blurs and attenuates the image or the lidar window | sensors |

The Marshall–Palmer parameters are in mixed units in the original paper. The implementation
converts them to SI once, with the source cited in a comment and a unit test on the integrated
rain rate.

## 3. Signals

```cpp
// modules/weather — declare()
d.output({.name = "env/air_temperature",   .type = SignalType::Scalar,  .unit = units::K});
d.output({.name = "env/sky_temperature",   .type = SignalType::Scalar,  .unit = units::K});
d.output({.name = "env/pressure",          .type = SignalType::Scalar,  .unit = units::Pa});
d.output({.name = "env/relative_humidity", .type = SignalType::Scalar,  .unit = units::one});
d.output({.name = "env/air_density",       .type = SignalType::Scalar,  .unit = units::kg_per_m3});
d.output({.name = "env/wind/mean",         .type = SignalType::Vec3,    .unit = units::m_per_s, .frame = "world_local"});
d.output({.name = "env/wind/velocity_field", .type = SignalType::Field3D, .unit = units::m_per_s});
d.output({.name = "env/solar_irradiance",  .type = SignalType::Scalar,  .unit = units::W_per_m2});
d.output({.name = "env/sun_direction",     .type = SignalType::Vec3,    .unit = units::one, .frame = "world_local"});
d.output({.name = "env/extinction/visible",   .type = SignalType::Scalar, .unit = units::per_m});
d.output({.name = "env/extinction/nir_905",   .type = SignalType::Scalar, .unit = units::per_m});
d.output({.name = "env/extinction/nir_1550",  .type = SignalType::Scalar, .unit = units::per_m});
d.output({.name = "env/extinction/field",     .type = SignalType::Field3D, .unit = units::per_m});   // fog/dust grid
d.output({.name = "env/rain_rate",         .type = SignalType::Scalar,  .unit = units::m_per_s});
d.output({.name = "env/radar_attenuation_77ghz", .type = SignalType::Scalar, .unit = units::per_m});
d.output({.name = "env/friction_scale",    .type = SignalType::Scalar,  .unit = units::one});
for (auto& s : world.instancesOf("SensorAPI_1"))
    d.output({.name = fmt("sensor/{}/contamination", name(s)), .type = SignalType::Scalar, .unit = units::one});
```

The weather module has no inputs from other domains in v0.5. Wiper and washer actuators would be
inputs later.

## 4. Step size

Default 50 ms. Environment values change slowly; sensor kernels sample the latest values.

## 5. Substitutes and contract

Contract: all `env/*` outputs above and `sensor/{s}/contamination`.

| Profile token | Kind | Behaviour |
|---|---|---|
| `clear` | constant | Temperature, pressure and humidity from the environment file if present, otherwise from the profile; a profile with neither fails validation. Zero wind, zero extinction, zero rain, `friction_scale = 1`, contamination 0 |
| `replay` | replay | From MCAP |

## 6. Placement

| Feature | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| State and scalar extinction | CPU | CPU | CPU |
| Per-ray attenuation and backscatter | inside the CPU lidar path | `gpu/kernels/weather/attenuate.slang` in the lidar compute pass | same kernel on ray-query results |
| Camera fog and rain streaks | CPU post-process at low resolution | fragment/compute pass | same |
| NanoVDB extinction grid marching | CPU, coarse steps | GPU | GPU |

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Koschmieder and Beer–Lambert | closed form | exact to rounding |
| Rain rate from the drop distribution | integrate `N(D)` with a terminal-velocity law and recover `R` | 5 % |
| Lidar range reduction in fog | published lidar-in-fog chamber measurements (candidate: results from the European DENSE project and related papers; obtain data and check licenses) | maximum detection range within 20 % for stated visibilities |
| Lidar false returns in rain | published rain-chamber statistics for 905 nm sensors | return-rate order of magnitude and its trend with rain rate |
| Camera contrast in fog | contrast of a black/white target at distance vs Koschmieder | 5 % |
| Wet friction | tyre–road friction ranges from the literature for dry and wet asphalt | within the published range; this is a range check, not a point validation |

Weather effects on sensors are statistical. Each validation states the quantity compared
(detection range, return rate), not image similarity.

## 8. Status (v0.5)

`lagrange_weather` (`modules/weather/`): `Weather` and its `clear` substitute (`Weather(true)`).

- The site (air temperature, pressure, relative humidity) has no defaults: from `--weather`, a
  `WeatherAPI_1` prim or the profile's `weather` keys (the profile wins; the temperature falls
  back to thermal's ambient one). Without it the module does not run.
- Moist air density from Alduchov and Eskridge's saturation vapour pressure (1996): the ICAO
  sea-level density within 5e-4 for dry air.
- Fog: Koschmieder at 550 nm, the Kim model's wavelength exponent for 905 and 1550 nm
  (TODO(verify) the q table against Kim, McArthur and Korevaar 2001).
- Rain: Marshall–Palmer, extinction `π·N0/Λ³` for drops much larger than the wavelength
  (checked against the integral). The distribution with Atlas's fall speeds carries 5 to 18 %
  more rain than it was built from, the known looseness of the fit; the design's 5 % target for
  that check is not met and the test states the measured spread.
- Dust: quartz spheres at the median diameter.
- Wet friction and lens contamination: behavioural, time constants in `Weather.hpp`.
- Not yet: consumers (sensors in step 3, rigid body's friction), the wind and fog grids
  (step 4), radar attenuation, a weather time series.
