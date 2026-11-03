# Validation report (v1.0, brought up to date 2026-11-03)

This report is the roadmap §9 deliverable "validation report per domain". It collects, for each
domain, what the model is checked against, the tolerance, the result and the test that checks it.
Sources: the "Validation" and "Status" sections of `docs/design/domains/*.md`,
`docs/design/huge-worlds.md`, the acceptance tables in `docs/release/v0.2.0.md` to `v0.7.0.md`,
and `docs/plan/progress.md`.

## What "validated" means here

A model counts as validated against a reference when a test compares it with one of:

- a **closed form** (an analytic solution, or a monolithic high-accuracy solution);
- a **published reference** (a report, standard or dataset, cited below as the docs cite it);
- **measured data** from a real robot or bench (sim-vs-real).

Each check states its tolerance and names the test. "Met" means the test passes at that
tolerance on the development machine (GitHub Actions are disabled; the tests run locally).

**Sim-vs-real against measured robot data mostly does not exist yet.** Only the battery is
compared with measured data (a published cell dataset). Every other recording that the domain
docs plan (AMR odometry, a BLDC bench, a motor heat run, a magnetometer near a known source,
Wi-Fi RSSI in a building, a drone flight log) waits for open question Q16: which hardware is
available, and whether recordings can be published under an open licence
(`docs/plan/open-questions.md`). Most domains are therefore checked against closed forms and
published curves only, which is risk R12 in `docs/plan/risks.md`.

## Summary

| Domain | Checked against | Tolerance / result | Test | Sim-vs-real (measured data) |
|---|---|---|---|---|
| Rigid body | Closed-form mechanics; backend agreement MuJoCo vs Jolt; contact force and joint reaction on both (Newton) | Energy drift < 0.1 % over 10 s; Coulomb angle within 1°; inertia 1e-9; backends within 2 %; weight held within 0.5 %, a swinging pivot's load within 1 % | `test_mujoco_validation`, `test_backends`, `lagrange_physics_conformance` | none yet (AMR square drive, Q16) |
| Electrical | Closed forms; LG HG2 cell data (Kollmeyer et al.); ngspice | 0.1–0.5 % closed forms met; battery 5.4–6.2 mV RMS (46–89 % SoC), 21.5 mV at 24 % (target < 20 mV, not met); drive cycles at 25 °C (fitted to UDDS, unseen LA92 11.7 mV, US06 20.1 mV, just over), 39–145 mV at 40 to 0 °C (no temperature dependence) | `test_models`, `test_network`, `test_coupling`, `test_battery_dataset`, `test_export` | battery: published cell data; BLDC bench: none yet (Q16) |
| Thermal | Closed forms | 0.1 %, 0.5 %, 0.5 K met; energy to 1e-9 | `test_lumped`, `test_thermal_network` | none yet (motor heat run, Q16) |
| Rotor aero | NACA Report 640; momentum theory; classical H-force; Cheeseman–Bennett; NeuroBEM's vortex-ring fit; Hsiao and Chirarattananon's ceiling balance; downwash: Bauersfeld et al. 2024, Tanner et al. 2015, Kiran et al. 2025, own OpenFOAM | CT within 8 %, CP within 6 %; H-force 1 %; ground and ceiling effect exact to their curves; stacked drones 53 % kept at 4 l against 65 % measured (**12 points short**) | `test_bem`, `test_rotor_aero`, `test_downwash`, `test_downwash_pair` | none yet (flight log, Q16) |
| Live air (lattice Boltzmann) | Momentum theory; own OpenFOAM hover cases | Momentum through a plane within 2 %; at 3 cm the centreline within 20 % and the outwash within 6 % of OpenFOAM; at 4 cm the outwash 19 % low | `test_lbm_box`, `test_lbm`, `tools/validation/lbm_hover.py` | none (OpenFOAM is a model too) |
| Aerodynamics (drag, wind) | Closed form; Dryden (MIL-F-8785C) | Terminal velocity 0.5 %; turbulence variances 5 %; periodogram check owed | `test_aerodynamics` | none yet |
| Weather | Koschmieder, Beer–Lambert, Marshall–Palmer, ICAO density | Exact; rain rate 5–18 % (target 5 %, not met; test at 20 %) | `test_weather` | none yet |
| Sensor weather effects | Lidar equation, Koschmieder veil | Trends tested; fog clutter uncalibrated | `test_weather_effects` | none yet (fog/rain chamber data, DENSE) |
| EMI | Closed forms (conductor, dipole, far field); aliasing | 1e-6 relative; 60 → 40 Hz alias exact; 8.08 µT at 9.9 m from 400 A | `test_emi` | none yet (Q16) |
| Comms | Friis, `k·T·B`, AWGN bit errors, ISO 11898-1 | Friis, noise floor, BPSK 1e-5 point, CAN transitions exact | `test_comms` | none yet (RSSI recording, Q16) |
| Sensors: lidar | Analytic scenes; Tier 0 vs Tier 1 | Tiers within 1 mm: worst 0.16 mm (Iris Xe), 0.31 mm (llvmpipe), same shape 100 % | `test_sensors`, `test_lidar_tiers` | none yet |
| Sensors: camera | Pinhole model via OpenCV; EMVA 1288 linear model | 0.066 px mean, calibration RMS 0.07 px (< 0.1 px); photon-transfer variance within 3 % | `test_raster`, `test_camera_model` | none yet |
| Sensors: radar | Radar equation; RCS of a plate | RCS within 3 %; Doppler; detection range | `test_radar` | none yet |
| Sensors: IMU | Allan deviation of a 4-hour static record | Within the estimator's confidence band (result not stated in the docs) | `test_imu_noise` | none yet |
| Sensors: GNSS | `GeoOrigin` (GeographicLib) | Exact against `GeoOrigin`; error statistics check: no evidence found | `test_geo_sensors` | none yet |
| Sensors: magnetometer | World Magnetic Model 2025, NOAA test points | Within 0.001 nT (worst 7e-4 nT) | `test_wmm`, `test_geo_sensors` | none yet |
| Sensors: barometer | ICAO standard atmosphere | ICAO table values | `test_geo_sensors` | none yet |
| Vehicle dynamics | v²/(2μg); linear bicycle model; Magic Formula shape | Braking 3.2 % (5 %); yaw rate 1.4 % (3 %); MF peak, slope, asymptote exact | `test_vehicle_dynamics`, `test_tyre` | none yet |
| Traffic and scenarios | IDM equilibrium; light cycle; sample scenario timeline | IDM, lights, determinism met; fundamental diagram owed | `test_traffic`, `test_openscenario`, `test_opendrive` | not applicable / none yet |
| Huge worlds | Same world at the origin; GeographicLib | Bit-identical poses at 500 km; rebase within 1e-9 m (MuJoCo), 1e-4 m (Jolt) | `test_robot`, `test_frames`, `test_regions`, `test_tile_selector` | not applicable |
| Radiation | — | Not built; `zero` substitute only | — | none |
| Hydrodynamics | Buoyancy, drag, a sphere's added mass (Lamb), currents and linear (Airy) waves (`lagrange_hydrodynamics`), the closed-form checks of hydrodynamics.md §9 | released sphere's rise within 3 % of g(ρV − m)/(m + ½ρV); drift with a current within 0.02 m/s; heave on long waves within 1 cm of the amplitude. Not Fossen's full model (no rotational added inertia, no Coriolis terms, no radiation) | `test_water` | none measured |

## Since v1.0 (to 2026-11-03)

The models added after the v1.0 release, each with its reference, result and test. Where the
reference is our own OpenFOAM run, that is a check between two models, not against the air.

| Model | Reference | Result | Test |
|---|---|---|---|
| Rotor downwash, far jet | Bauersfeld et al. 2024 (arXiv:2403.13321), measured multirotor jets | fitted to it; its momentum flux carries the thrust within 5 %; our OpenFOAM hover centreline about 10 % under it | `test_downwash` |
| Rotor downwash, ground outwash | Tanner et al. 2015 (NASA), 1.9 V_h at 1.7 R | fitted to it at one rotor height (1 R); between 1 R and 2 R the model's peak rises where the measurement fades (rotor_aero.md §6) | `test_downwash` |
| Drone under a drone | Kiran, Ayanian and Breuer 2025 (RSS), stacked Crazyflies at constant voltage | 76 % of the thrust kept at 4 l (measured 65 %): **11 points over** since wakes are averaged over each disc (53 %, 12 under, when sampled at the hub); 87 % at 19 l (measured 95–98 %): **8–11 under**; constant voltage adds under one point | `DownwashPair.TheLowerQuadcopterLosesThrustAsMeasured`, `DownwashPair.AtConstantVoltage…` |
| A wall beside a drone | own OpenFOAM, four gaps | gain 0.0173 fitted; its fade length 3.5 ± 2.7 D is poorly fixed (Tanabe et al. say 1.5 D) | `DownwashPair.AWallStarves…` |
| Live air box (`downwash_model: lbm`) | momentum theory; own OpenFOAM hover at 2 m and 0.5 m | momentum within 2 %; disc speed at its centre 23 % over ideal (blurred edge); at 4 cm through-disc air 15–20 % under OpenFOAM's, the low-hover fountain 5.8–7.4 m/s against 7.1 m/s | `test_lbm_box`, `DownwashPair.TheLiveAirBoxDrawsTheRotorsTogether`, rotor_aero.md §8 |
| Vortex-ring state (`bem`) | Bauersfeld et al. 2021 (NeuroBEM, RSS) eqs. 18–19, after Hoffmann et al. 2007 | the fit exactly; thrust joins hover's within 0.1 %; a step of 10 % of hover thrust into the windmill-brake state | `Bem.TheVortexRing*` |
| Ceiling effect | Hsiao and Chirarattananon 2019 (IEEE/ASME TMech) momentum balance | exact to it; their α0 = 1.6 is for a 23 mm propeller (TODO(verify) for larger ones) | `RotorAero.Ceiling*`, `RotorAero.UnderAFixedBox*` |
| Contact force and joint reaction, Jolt | Newton's laws; MuJoCo | box weight within 0.5 %; swinging pivot 11.348 N against 11.349 N (energy) and MuJoCo's 11.349 N | `lagrange_physics_conformance` |
| Wind tunnel (`lagrange tunnel`) | shear-wave decay; Schiller–Naumann sphere; Hoerner plate and cube; Ahmed body (SAE 840300) | shear wave 1 %; sphere 25 % over; plate 0.97 and cube 1.03 against 1.17 and 1.05; Ahmed body **over four times too high** (Cd 1.26 on 8M cells against 0.29) | `test_lbm`, cfd.md §5–6 |
| OpenFOAM cases (`lagrange foam`) | Ahmed body at 25° | Cd 0.266–0.288 against 0.2875–0.298, but 8 % apart between two meshes; 35° not trustworthy yet | cfd.md §5 |
| Components between buses (RFC 0010) | formulas; ngspice | Lagrange within 0.5 % of the formula, ngspice within 0.1 %, on random circuits | `tools/ci/check_electrical.py`, `test_electrical_check` |
| Walking humans | social force (Helbing and Molnár 1995) as traffic uses it | behaviour tested (they walk, steer, stop); not against measured crowds | `test_humans` |
| Battery on drive cycles | LG HG2 UDDS, US06, LA92 (Kollmeyer et al., CC BY 4.0) | fitted to UDDS: LA92 11.7 mV, US06 20.1 mV RMS (target 20 mV; US06 not met by 0.1 mV); at 0 °C 145 mV | `BatteryDataset.TheEquivalentCircuitFollowsMeasuredDriveCycles` |
| Planar pushing (MuJoCo, Jolt) | Yu et al. 2016 (IROS), 2000 measured pushes of rect1 on four surfaces; the quasi-static model they compare with (Lynch et al. 1992) | MuJoCo within 6 mm and 4–7° of the quasi-static model; 3–41 mm and 10–17° from the measured means (as the model is). **Jolt does not turn the block** (2° against 75°): its linear and torsional friction have separate budgets | `Conformance.APushedBlockMovesAsTheMcubeMeasurements` |
| Quadrotor rotors in flight | NeuroBEM flights (Bauersfeld et al. 2021), 13 held-out segments, measured motor speeds and motion | `coefficients` F 1.46 N RMS (their quadratic fit 1.49); `bem` with their blade geometry and 2° fitted 1.12 N (their BEM 0.98, 14 % better; Mxy 41 % better: blade flapping) | `tools/validation/neurobem.sh` (downloads 627 MB), rotor_aero.md §9 |
| Thermal camera | Incropera et al. Table 12.1 black-body fractions; Planck integrated directly; Stefan–Boltzmann; Swinbank 1963 clear sky | fractions within 1e-5, band radiance within 1e-9; a 350 K node of ε 0.95 read at its apparent temperature within the 50 mK noise; scene temperatures are assumptions (no sun, no atmosphere) | `test_thermal_camera` |

### Models with no reference yet

Said plainly, so nobody reads more into these than is there:

- **Dust lifted by outwash**: the threshold is Bagnold's, but how much dust rises (the
  concentration curve) is our own choice, not checked against anything.
- **The live box's unsteadiness**: the rotor speeds move about 1 % in hover with it; no measured
  spectrum to compare.
- **Fountain lift on the frame** in the fitted model (rotor_aero.md §2.8): fitted to our own
  OpenFOAM runs at five heights only (within 0.23 N), no measurement.
- **Hydrodynamics beyond a sphere's added mass**: other shapes' added mass, rotational added
  inertia, wave radiation (Fossen's full model is planned, hydrodynamics.md).
- **Lidar intensity**, **GNSS error statistics**, **sensor timing histograms**: no evidence found
  (Sensors table below).
- **Ultrasonic cone** (the `lagrange_tof` plugin): a geometric cone and dead zone, no datasheet
  comparison.
- **Realistic camera look**: a picture, not a radiometric model; only the pinhole geometry and
  noise are checked.
- **Thermal camera scenes**: the radiometry is checked, but what a surface's temperature is
  (ambient for anything without a thermal node, no sun, no sky reflection, no atmosphere) and a
  person's skin temperature are our assumptions; no thermal image of a real scene to compare.
- **Radiation**: not built.
- **Jolt's contact force on a group of fixed bodies** goes to the group's root body; which body
  of the group was touched is not known.

## Rigid body

Source: `domains/rigid_body.md` §7 (the doc has no status section; tests confirmed in
`modules/rigid_body/mujoco_backend/test/test_mujoco_validation.cpp` and
`server/test/test_backends.cpp`).

| Check | Reference | Tolerance | Test |
|---|---|---|---|
| Free fall, pendulum, double pendulum energy drift | closed form or high-order integration | drift < 0.1 % over 10 s at 1 ms | `FreeFall.*`, `Pendulum.EnergyDriftAndLargeAnglePeriod`, `DoublePendulum.Rk4EnergyDrift` |
| Sliding block on an incline | Coulomb, closed form | stick/slip angle within 1° | `Incline.EllipticConeSlipsAtTheCoulombAngle` (the pyramidal cone slips about 1.5° early, so the backend uses the elliptic cone) |
| `effective_inertia` | analytic two-link arm inertia matrix | 1e-9 relative | `Inertia.TwoLinkArmMatchesTheAnalyticMatrix` |
| Determinism | two runs, same seed | bit-identical state hash (MuJoCo) | `Determinism.TwoRunsAreBitIdentical`; v0.2 criterion 5: 601 identical hashes over 60 s |
| Backend agreement, MuJoCo vs Jolt | the same scene on both | arm6 joint-space inertia within 2 %, its fall over 0.3 s within 2 % (+5 mrad), with armature within 5 % (+15 mrad); the sample AMR within 0.02 m/s and 0.02 rad/s | `Backends.*` in `test_backends` |
| Height fields (v0.7) | the same grid read by both backends | balls over a raised quarter rest within 1 cm of it | `Backends.AHeightFieldIsTheSameWayRoundOnBoth` |

Reported difference: on Jolt's rigid contacts the AMR's drive wheels slip while the drive ramps
up, about 0.4 m behind after 6 s. PhysX is not built (the owner's v0.6 decision), so the
three-backend comparison in §7 covers MuJoCo and Jolt only.

**Owed:** the AMR drive test, sim-vs-real: recorded wheel odometry of a real differential-drive
robot driving a square; "dataset to be recorded or sourced (open question Q16)". Tolerance:
position error after 10 m within the spread of repeated real runs.

## Electrical

Source: `domains/electrical.md` §8–§9, `v0.3.0.md`, `v0.4.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| RL step response, d and q | closed form | 0.1 % | met (exact up to rounding) | `RlStepResponseMatchesTheClosedForm` |
| DC motor no-load speed, stall current, torque-speed line | closed form from R, Kt, V | 0.5 % | met | `BrushedDcTorqueSpeedLineMatchesTheClosedForm` |
| Coupled motor + inertia | monolithic 1 µs solution | 2 % RMS | met for inertias 6e-7 to 1e-2 kg·m², no growth; the quadcopter's drive at 50 µs on 1e-5 and 9e-5 kg·m² rotors | `test_coupling` (`Coupling`, `DroneCoupling`) |
| Battery ECM pulse | closed form; LG HG2 cell, Kollmeyer et al., doi:10.17632/b5mj79w5w9.2, CC BY 4.0 (HPPC and C/20 at 25 °C) | < 20 mV RMS per cell | closed form exact; 5.4 to 6.2 mV RMS on held-out blocks from 46 to 89 % SoC; 21.5 mV at 24 % (**not met**) | `PulseDischargeMatchesTheClosedForm`, `test_battery_dataset` |
| Energy balance over a drive cycle | stored + work + losses | 0.5 % | met | `EnergyBalancesOverADriveCycle` |
| Netlist round trip | ngspice (averaged sources) | bus voltage 1 % | met: within 0.05 % (sag within 0.16 % RMS) | `test_export` (needs ngspice) |
| Brownout scenario | scripted stall on a low battery | trips and recovers | met | `AStalledAmrOnALowBatteryBrownsOutAndRecovers` |

Below about 35 % state of charge r0 and the OCV slope rise; a state-of-charge-dependent r0 is the
next step. The battery is checked at 25 °C only. The Modelica export is checked for structure
only (no OpenModelica on the machine).

**Owed:** sim-vs-real against a bench recording of a real BLDC drive (phase current, bus voltage,
speed; "to be recorded; Q16"); the battery below 35 % state of charge; the drive-design review of
the motor constants (RFC 0002 resolution 2).

## Thermal

Source: `domains/thermal.md` §7–§8, `v0.3.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Single node cooling | closed-form exponential | 0.1 % | met | `ASingleNodeCoolsExponentially` |
| Two-node network step | closed-form two-pole response | 0.5 % | met | `TwoNodesFollowTheTwoPoleStepResponse` |
| Radiative equilibrium of a sunlit plate | closed form | 0.5 K | met | `ASunlitPlateReachesItsRadiativeEquilibrium` |
| Energy conservation | stored energy and flows | 0.1 % | met, every step to 1e-9 | `EnergyIsConserved` |
| Motor heat run | closed-form steady state of a motor linked to the chassis | — | met | `test_thermal_network` |

**Owed:** the comparison with "a published motor thermal test with winding temperature rise at
constant load, or a recorded bench test (Q16)", within 3 K at steady state.

## Rotor aero and aerodynamics

Source: `domains/aerodynamics.md` §7–§8, `v0.4.0.md`, `v0.5.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| `bem` against measured propellers | NACA Report 640 (1938, public domain), propeller 5868-9, two blades, Clark Y; section data from NACA Report 502 | CT 8 %, CP 6 % (or 0.003) over the attached-flow range | 15°, J 0.3–0.7: CT 2–4 %, CP 2–5 % (7 % at J = 0.2); 25°, J 0.5–1.1: CT 0–7 %, CP 0–5 %; zero thrust at J = 0.82 and 1.29, as measured | `NacaReport640.*` in `test_bem` |
| `bem` against momentum theory | ideal rotor, figure of merit 1 | 3–4 % | met | `Bem.IdealRotorMeetsMomentumTheory` |
| `bem` edgewise H-force | `C_H = σ·c_d0·μ/4` (Johnson 1980 §5.4; Leishman 2006 §3.4), μ = 0.1 | 1 % | met | `Bem.EdgewiseHForceOfDraggingBladesIsTheClassicalOne` |
| `bem` in wind | moving through still air = standing in the wind; quadcopter in 8 m/s with gusts | 1e-12; within 0.1 m | holds within 9 cm (6 cm in height) | `RotorAeroBem.FlyingThroughStillAirIsBeingStillInTheWind` |
| Static thrust and torque | closed form; published propeller data | 5 % | closed form exact; published data owed | `test_rotor_aero` |
| Ground effect | Cheeseman–Bennett; published small-rotor measurements | 10 % for z/R 0.5–3 | Cheeseman–Bennett exact for z/R 0.5–3; measurements owed | `test_rotor_aero` |
| Terminal velocity | closed form | 0.5 % | sphere within 0.5 % | `test_aerodynamics` |
| Turbulence spectra | Dryden / von Kármán PSDs | 95 % band of the periodogram | variances within 5 % of σ² over 4000 correlation times, longitudinal autocorrelation within 0.03 of e^(−τV/L); periodogram test owed | `test_aerodynamics` |
| Wind grids | the grid's own value | exact | met | `Aerodynamics.InsideAWindGridTheGridIsTheWind` |

Geometry and NACA 640 measurements were read off the report's charts (chord about ±1.5 %, blade
angle ±0.3°, CT and CP about ±0.001); nothing was fitted to the propeller data. Below J ≈ 0.5 at
25° the inner blade stalls and the model has no stall-delay correction, so that range is not
tested.

**Owed:** small-propeller test data (the UIUC database states no licence); measured ground
effect; stall delay; the Dryden periodogram check; hover power and step response against "a
flight log of a comparable real vehicle (to be sourced; Q16)".

## Weather

Source: `domains/weather.md` §7–§8, `v0.5.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Koschmieder and Beer–Lambert | closed form | exact to rounding | met | `Extinction.KoschmiederAndBeerLambert` |
| Moist air density | ICAO sea-level density (dry air); Alduchov and Eskridge (1996) | — | within 5e-4 (as the source states it) | `Atmosphere.DryAirAtSeaLevelIsTheStandardAtmosphere` |
| Rain extinction | Marshall–Palmer integral | — | met | `Extinction.RainIsTheMarshallPalmerIntegral` |
| Rain rate from the drop distribution | Marshall–Palmer with Atlas's fall speeds | 5 % | **not met**: 5 to 18 % more rain; the test checks 20 % | `Extinction.MarshallPalmerCarriesItsRainRate` |
| Lidar detection range in weather | lidar equation | — | tested | `LidarWeather.DetectionRangeSolvesTheLidarEquation` |
| Lidar clutter | fog backscatter; Marshall–Palmer drops | trends | tested: rain clutter about 1 % of beams at 2 mm/h, 5 % at 25 mm/h | `test_weather_effects` |
| Camera contrast in fog | Koschmieder | 5 % | veil tested | `CameraWeather.TheVeilIsKoschmieders` |
| Fields | analytic fields, OpenFOAM-format round trip | — | trilinear exact; linear wind within 1e-5 | `test_fields` |

The Kim model's wavelength table is TODO(verify) against Kim, McArthur and Korevaar 2001. The
fog clutter gain is uncalibrated.

**Owed:** lidar range in fog against "published lidar-in-fog chamber measurements (candidate:
results from the European DENSE project)", within 20 %; rain false returns against published
rain-chamber statistics; the wet-friction literature range check; files from a real OpenFOAM
case.

## EMI

Source: `domains/emi.md` §7–§8, `v0.5.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Straight conductor and dipole fields | closed form | 1e-6 relative | met | `EmiFields.StraightConductorByTheRightHandRule`, `EmiFields.DipoleOnItsAxisAndAtItsEquator` |
| Far field of a transmitter | closed form (Friis) | 1e-6 relative | met | `EmiFields.FarFieldMatchesFriis` |
| Mains aliasing | 60 Hz sampled at 100 Hz gives 40 Hz | peak location exact | met | `Emi.AMainsFieldAliasesAtTheMagnetometersRate` |
| Sample scene | `μ0·I/(2πd)` | — | 8.08 µT RMS at 9.9 m from 400 A at 60 Hz | `quad_x500_power_line.yaml` (end to end in the server) |

The grid frequency is a site parameter with no fallback; the sample states 60 Hz in the scene.

**Owed:** field strengths near real lines against published measurements and guideline documents
("for example ICNIRP reference material"); the end-to-end recording from a real robot near a
known source (Q16), target heading-error amplitude within 20 %.

## Comms

Source: `domains/comms.md` §7–§8, `v0.5.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Free-space path loss | Friis equation | 1e-9 relative | met | `CommsBudget.FriisAndTheNoiseFloor` |
| Thermal noise floor | `k·T·B` | exact | met | `CommsBudget.FriisAndTheNoiseFloor` |
| Packet error rate curves | AWGN bit-error expressions | 1 % | BPSK 1e-5 point and Q(3) met | `CommsBudget.BitErrorCurves` |
| CAN state machine | ISO 11898-1 fault confinement | exact | met (transmit error counter only) | `CommsCan.FaultConfinementTransitions`, `Comms.ANoisyCanBusGoesBusOffAndRecovers` |
| Walls and shadowing | wall counting; Gudmundson shadowing | — | wall count and spread met | `CommsWalls.CountsTheWallsBetween`, `Comms.ShadowingHasItsSpread` |

**Owed:** indoor parameters traced to ITU-R P.1238 (TODO(verify)); the end-to-end RSSI recording
of an AMR driving away from an access point in a real building (Q16); the MAVLink bridge over
links.

## Sensors

Source: `domains/sensors.md` §2, §7, §8, `v0.2.0.md`, `v0.3.0.md`, `v0.4.0.md`, `v0.6.0.md`.

| Sensor | Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|---|
| Lidar | geometry | analytic scenes (plane, sphere, corner) | < 1 mm before noise | casts tested per shape | `Raycast.*`, `Lidar.*` in `test_sensors`; `test_shape_caster` |
| Lidar | height fields (v0.7) | the field's own surface (`heightAt`) | 1e-9 m | met, vertical and slanted rays | `Raycast.HeightFieldsAreHitWhereTheirSurfaceIs` |
| Lidar | tier agreement | Tier 0 vs Tier 1 | same hit > 99.9 %; 1 mm | worst 0.16 mm (Iris Xe), 0.31 mm (llvmpipe); same shape 100 % | `test_lidar_tiers` |
| Lidar | intensity | Lambertian target | 2 % | no evidence found | — |
| Camera | projection | checkerboard, OpenCV calibration | < 0.1 px | 0.066 px mean; intrinsics RMS 0.07 px | `Raster.CheckerboardCornersLieWhereThePinholeModelPutsThem` |
| Camera | noise | photon-transfer curve (EMVA 1288 linear model) | 3 % | met | `Noise.FollowsThePhotonTransferCurve` |
| Camera | lens, vignetting, rolling shutter, exposure | OpenCV's lens formulas; cos⁴θ; skew of a vertical line under a pan | rolling shutter 1 px | tested | `test_camera_model` |
| Radar | RCS, Doppler, range | σ = ρ·A; radar equation | — | plate RCS within 3 %; 10 m/s Doppler; range | `test_radar` |
| IMU | noise | Allan deviation of a 4-hour static record | estimator's confidence band | test present; result not stated in the docs | `ImuNoise.AllanDeviationOfAStaticRecordMatchesTheDensityAndRandomWalk` |
| GNSS | position | `GeoOrigin` | exact | met | `GeoSensors.GnssPlacesTheAntennaThroughTheGeoOrigin` |
| GNSS | error statistics and correlation time | — | 5 % | no evidence found | — |
| Magnetometer | main field | World Magnetic Model 2025 (`data/wmm2025`, public domain), NOAA's 100 test points | 0.001 nT | worst 7e-4 nT | `Wmm.MatchesNoaaTestValues` |
| Barometer | pressure, temperature | ICAO standard atmosphere (troposphere) | table values | met | `GeoSensors.TheBarometerFollowsTheIcaoStandardAtmosphere` |
| All | timing | rate, latency, jitter histograms | 1 base tick | no evidence found | — |

Radar and the camera model run on the CPU on every tier; hardware ray-query versions are owed.

**Owed:** sim-vs-real per release against "public datasets with ground truth and sensor
specifications (candidates: an indoor 2D-lidar AMR dataset for v0.2; an automotive dataset for
v0.6; choose and check licenses)"; a published RCS reference; radar rain attenuation and
multipath; camera motion blur, a light model and demosaicing; GNSS outages and multipath; crustal
field and hard- and soft-iron effects.

## Vehicle dynamics

Source: `domains/vehicle_dynamics.md` §7–§8, `v0.6.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Braking distance from 20 m/s | `v²/(2·μ·g)` | 5 % | 3.2 % long | `BrakingDistanceIsVSquaredOverTwoMuG` |
| Steady-state cornering | linear bicycle model, understeer gradient | 3 % | 1.4 % | `SteadyCorneringMatchesTheBicycleModel` |
| Magic Formula | its own peak, slope and asymptote | exact | met | `MagicFormula.PeakSlopeAndAsymptote` |
| Ride height, static loads; gear shifting; ideal substitute | — | — | met | `test_vehicle_dynamics` |

The sample tyre `assets/tyres/generic_205_55R16.tir` is illustrative.

**Owed:** a comparison with a published MF parameter set ("no openly licensed `.tir` was found");
the ISO 4138, ISO 3888-2 and ISO 7401 manoeuvres as scenarios; the drivetrain energy balance
(1 %, no evidence found); sloped ground; the kinematic suspension; sim-vs-real against "a public
vehicle-dynamics dataset with IMU, wheel speeds and steering (to be selected; license check)".

## Traffic, OpenDRIVE and OpenSCENARIO

Source: `domains/traffic.md` §7–§8.1, `v0.6.0.md`.

| Check | Reference | Result | Test |
|---|---|---|---|
| IDM equilibrium gap | Treiber, Hennecke and Helbing 2000 | met | `Idm.EquilibriumAndFreeRoad` |
| Light cycle; no entry on red over three minutes | the configured cycle | met | `Lights.TakeTurnsGreenYellowRed`, `CityBlock.NoVehicleEntersTheJunctionOnRed` |
| Determinism (`builtin`) | same seed | identical tracks | `CityBlock.TheSameSeedGivesTheSameTracks` |
| Cut-in scenario | its own triggers | lane change at 3.5 s, halfway at 5 s, in lane at 6.6 s, 8 m/s at 7.5 s, ended at 12 s | `CutIn.RunsToItsEndWithEventsWhereTheTriggersSay` |
| OpenDRIVE geometry | closed forms per road type; lanes meeting end to start | met | `test_opendrive` |

Pedestrian walking speed 1.34 m/s (Weidmann 1993); social force after Helbing and Molnár 1995.
SUMO and esmini are not built (the owner's v0.6 decision), so the TraCI and SUMO alignment checks
do not apply.

**Owed:** the pedestrian fundamental diagram against the literature; scenario conformance on the
ASAM OpenSCENARIO example scenarios and a comparison with esmini; catalogs; OpenDRIVE
superelevation, lane heights, objects and signals.

## Huge worlds and the floating origin

Source: `docs/design/huge-worlds.md` §1.3, §1.4, §6, `v0.2.0.md`, `v0.7.0.md`.

| Check | Reference | Tolerance | Result | Test |
|---|---|---|---|---|
| Geodetic, ECEF and ENU frames | GeographicLib | round trip < 1e-6 m within 100 km | met | `Frames.*` in `test_frames` |
| A world 500 km out | the same world at the origin | bit-identical | poses bit-identical, lidar within 1.2e-10 m after 10 s (without the floating origin: 5e-7 m in pose, 8e-5 m in range) | `FloatingOrigin.AWorld500KmOutSimulatesLikeOneAtTheOrigin` |
| Rebase during a run | the run never rebased | — | within 1e-9 m on MuJoCo, 1e-4 m on Jolt | `FloatingOrigin.ARebaseDuringTheRunKeepsTheTrajectoryOn{Mujoco,Jolt}` |
| Robots 10 km apart | one robot alone at the origin | exact | met | `Regions.RobotsTenKilometresApartRunInTheirOwnRegions` |
| Tile streaming | deterministic selection; no entry into an unloaded tile | equal state hashes | met | `Streaming.TheTilesAroundTheRobotAreLoadedAndTheRunIsDeterministic`, `TileSelector.ARobotMayNotEnterATileThatIsNotLoaded` |
| Terrain from an elevation model | the raster's height at each vertex's geodetic point | 1e-3 m | met | `WorldBuilder.TerrainFollowsTheElevationModelInTheEnuFrame` |

**Owed:** a drone flight crossing two rebases with continuous MAVLink GNSS (huge-worlds §6, v0.4;
no evidence found); memory budgets from profiles; the viewer's frame time while tiles load;
robots moving between regions while running.

## Radiation

Source: `domains/radiation.md`. Not built: the release is after v0.7 (open question Q2) and the
owner placed it after 1.0. The domain exists as a contract with its `zero` substitute only. The
planned checks (§7) are the inverse-square law, slab shielding with NIST XCOM coefficients,
build-up factors (ANSI/ANS-6.4.3 values as reproduced in open references), field sampling, SEU
statistics and a Geant4 comparison within a factor of 2. None is implemented. Sim-vs-real: none.

## Hydrodynamics

Source: `domains/hydrodynamics.md`. Post-1.0; only the contract and the `zero` substitute exist.
The planned checks (§7, "when built") are hydrostatic equilibrium of a box, terminal velocity
under quadratic drag, turning circle and zig-zag of a documented vessel, and wave response from
RAO tables. None is implemented. Sim-vs-real: none.

## Open items across domains

- **Q16 sim-vs-real data:** hardware and publication terms still to be named. Until then the
  battery is the only model compared with measured data.
- **Published datasets owed:** small-propeller data (UIUC, no licence stated), small-rotor ground
  effect, fog- and rain-chamber lidar data (DENSE), field measurements near power lines, ITU-R
  P.1238 parameters, an openly licensed MF tyre set, a published RCS reference, a pedestrian
  fundamental diagram.
- **Checks in a design table with no test found:** lidar intensity, GNSS error statistics,
  sensor timing, vehicle energy balance.
