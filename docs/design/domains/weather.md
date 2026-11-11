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
d.output({.name = "env/dust/concentration",   .type = SignalType::Scalar, .unit = units::kg_per_m3});  // the air's own dust (particle_sensor)
d.output({.name = "env/dust/median_diameter", .type = SignalType::Scalar, .unit = units::m});
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
- Sensors (`lagrange_sensors`, `WeatherEffects.hpp`), when the weather runs:
  - Lidar range by the lidar equation, `r·exp(σr) = R·(1 − c)` at its wavelength (905 nm, or
    1550 nm with the `wavelength` setting), R the clear-air max range against its reference
    target, c the window's contamination.
  - Fog clutter: the fog's backscatter (lidar ratio 18.5 sr, 5 ns pulse) seen through the
    receiver's overlap, with a log-normal detection spread and a gain set so that half the
    beams of a 30 m lidar see clutter at 60 m visibility. Uncalibrated, TODO(verify) against
    fog-chamber data (DENSE); clutter sits about 2 m out.
  - Rain clutter: Marshall–Palmer's drops of 0.5 mm and more within 2 m in a 5 mm beam: about
    1 % of beams at 2 mm/h, 5 % at 25 mm/h.
  - Camera: Koschmieder's veil from the depth image (sky all air light) and a haze for the lens.
  - Applied per beam on the CPU on every tier, after the cast: it is a few operations per beam,
    so the GPU kernel the placement table plans (`attenuate.slang`) is not needed yet.
- Fields (`lagrange_fields`, NanoVDB): a fog grid (`fog_field`, `weather:fogField`: extinction,
  1/m) adds its optical depth along each lidar beam up to the hit and each camera pixel's ray
  (once per 8×8 pixels), and its value at the sensor to the clutter; a wind grid (`wind_field`,
  `weather:windField`: m/s) is the mean wind inside its bounds for link drag and, through
  `env/wind/velocity`, the rotors. The weather publishes them as Field3D buffers
  (`env/extinction/field`, `env/wind/velocity_field`). Samples: `assets/fields/` (synthetic, by
  `lagrange_make_fields`): a 10 m-visibility bank across the warehouse aisle hides the far wall
  from 109 of the front lidar's 720 beams.
- Snow and ice (`snow_rate`, `snow_cover`; `lagrange --season winter`): extinction 1/m per mm/h of
  3.912e-3, illustrative (1 km at 1 mm/h; Rasmussen et al. 1999 find 3 to 10 times the spread between
  storms), lidar false returns as 3 times that rate of rain (illustrative), lens contamination, and
  `env/friction_scale` to 0.3 of dry on snow and 0.13 on ice (wet below freezing). Range-checked in
  `test_weather` against table values, 0.20 to 0.35 for packed snow and about 0.10 for ice; not point
  validated. Tyres, MuJoCo and Jolt take it through `env/friction_scale`; traffic agents, being
  kinematic, drive and brake more gently and never skid. Lidar returns off a snow-covered surface
  keep the surface's reflectivity.
- Not yet: radar attenuation, a weather time series, rain streaks in the camera. (Wet friction
  is applied by MuJoCo and Jolt through `env/friction_scale`, and the viewer shows the field
  overlays, since v0.5.)
- Offline CFD: `lagrange_foam_to_vdb` turns an OpenFOAM case's cell velocities (ASCII `U`, and
  the cell centres `C` from `postProcess -func writeCellCentres`) into a wind grid: the mean of
  the cells in each voxel (voxel k covers [k·h, (k + 1)·h) after `--offset`, so cells of size h
  aligned with the origin map one to one). It
  reads OpenFOAM's files only; OpenFOAM is not a dependency.
