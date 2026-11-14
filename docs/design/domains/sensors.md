# Domain: sensors

## 1. Scope and release

Lidar, camera, radar, IMU, GNSS, magnetometer, joint encoders, with timing models (rate, latency,
jitter). Sensors read the world state and the environment signals and write measurements that the
bridges publish.

| Sensor | Release | Notes |
|---|---|---|
| IMU, joint encoders, 2D/3D lidar (Tier 0 and Tier 1) | v0.2 | needed for Nav2 |
| RGB and depth camera | v0.3 | arms |
| GNSS, magnetometer, barometer | v0.4 | drones |
| Weather effects in lidar and camera | v0.5 | |
| Radar, hardware-RT lidar (Tier 2), rolling shutter, lens model | v0.6 | cars |

## 2. Models per fidelity level

Each sensor has `ground_truth` (the substitute) and one or more physical levels.

**Lidar**

| Level | Content |
|---|---|
| `raycast` | One ray per beam sample; range to the first hit; intensity from `SensorMaterialAPI` reflectivity at the sensor wavelength, incidence angle and `1/r²`; range noise; drop-out below a detection threshold; min/max range |
| `raycast_motion` | Adds per-ray timestamps within the scan so that sensor motion distorts the cloud (rotating and solid-state patterns) |
| `beam` | Adds beam divergence (several sub-rays per beam), multiple returns (strongest, last, dual), retro-reflector saturation, weather attenuation and backscatter from `env/extinction/*` |

**Camera**

| Level | Content |
|---|---|
| `raster` | Rasterised colour, depth, instance segmentation through the shared renderer (`gpu/kernels/camera/`). Implemented (v0.3): `lagrange_gpu_raster` draws colour multisampled (up to 8 samples, averaged: a pixel integrates its area) and depth and instance id at one sample; outputs `sensor/<n>/image` (rgba8), `depth` (r32f, m along the optical axis) and `instance` (r32u, scene prim index). Periods are rounded to whole milliseconds |
| `sensor_model` | Adds exposure (scene radiance → electrons with exposure time and gain), photon shot noise and read noise, quantisation to bit depth, lens distortion (plumb-bob or equidistant), vignetting, motion blur from per-pixel velocity, rolling shutter by rendering row bands at different times |
| `rt` (Tier 2) | Ray-traced shadows and reflections for the above |

**Colour look.** On a GPU a camera's colour comes from the viewport's realistic renderer: sun shadows,
sky, materials, photo textures, water, and the season (`--camera-look realistic|simple`,
`--camera-season`). A software device keeps the simple shading. Depth and instance ids are the same
either way (`RasterLook` test in `test_raster`). It is drawn by eye, not calibrated: exposure and
colour response are not those of any real camera, and rain and snow are in the picture only when the
season says so, separately from the `--weather` fog veil. With the sedan and the full town on an Iris
Xe the run went at about 1.3× real time, against 1.4× with the simple look.

Status of `sensor_model` in v0.6 (`modules/sensors/include/lagrange/sensors/CameraModel.hpp`, CPU
after the raster render, every tier with a Vulkan device): each part runs when
`CameraSensorAPI_1` asks for it. Lens distortion (plumb_bob, equidistant) by remapping a wider
pinhole render (at most twice the size); vignetting as a fraction of cos⁴θ; a rotational
rolling shutter (each row from the camera's orientation at its exposure, the angular velocity
held over the frame; translation left out); exposure as e = L·electronRate·t from the render's
linear value, Poisson shot and Gaussian read noise, full well, gain and bit depth (EMVA 1288's
linear model), then display encoding again; auto exposure aims the mean at 18 % of full scale
(`sensor/<n>/exposure_time`). camera_info carries the distortion. Checked in
`test_camera_model`: OpenCV's lens formulas and their inverse, the photon-transfer variance
within 3 %, cos⁴θ, the skew of a vertical line under a pan, exposure and auto exposure. The sample
sedan's front camera uses all of it. Owed: motion blur, a light model (absolute radiance),
colour filter array and demosaicing, translation in the rolling shutter, importing SDFormat
`<distortion>` and `<noise>`.

Since 1.2.x the per-pixel work runs on the GPU right after the render
(`gpu/kernels/camera/camera_model.slang`, in the same submission, so only the output's size is
read back): the same rays, vignetting and tables as `CameraModel::apply`, which stays the CPU
reference (`LAGRANGE_CAMERA_MODEL=cpu` keeps it in a run). The noise draws the reference's
Philox bits, six blocks per pixel as the per-row CPU stream takes them; Box-Muller is in float on
the GPU, double on the CPU, so a code differs only where the electrons sit on a rounding edge.
`test_camera` (`GpuCameraModel`) checks all but 0.1 % of the channels within 2 codes (on the RTX
4060 all but about 10 of 307 200 are the same code), the same ids, depth within 10⁻⁴ and the same
auto exposure. A thermal camera draws no colour (`RasterCamera::setColour`): it paints its own
from the ids.

The render does not hold up the run (since 1.2.x): the camera is a split step (architecture.md
§6): its `step()` submits the render (`RasterCamera::begin`), the scheduler runs the tick's
other modules (the lidar, the radar, ...) while the GPU draws, and the camera waits for it
(`RasterCamera::finish`) right before the first module that reads its images, or at the end of
the tick. The images are those of the step's state, as before (`test_raster`,
`FramesInFlightGiveWhatRenderGives`: two frames in flight give what `render()` gives, byte for
byte). With the model on the GPU the render's images stay in device memory for it; only the
model's output crosses to the host. A fisheye camera, and a camera with faults, still draw in
`step()`.

**Thermal camera** (after v1.0, `ThermalCameraAPI_1` on a camera,
`modules/sensors/include/lagrange/sensors/ThermalCamera.hpp`). A camera with it draws the same
raster frame, and then reads each pixel's instance id as a surface of known temperature and
emissivity:

- a prim under a `ThermalAPI_1` node (the longest path that holds it), else a node on the same
  body, is at that node's `thermal/<n>/temperature`, with its `thermal:emissivity`;
- a person (`HumanAPI_1`) is at Fanger's mean skin temperature in comfort while walking slowly,
  t_sk = 35.7 − 0.0275·M °C at M = 116 W/m² (Thermal Comfort, 1970): 32.5 °C, emissivity 0.98
  (Steketee 1973). Clothing, cooler than skin, is not told apart;
- everything else is at the site's ambient temperature, and so reads it whatever its emissivity;
- where nothing is hit, the clear sky: T_sky = 0.0552·T_air^1.5 (Swinbank 1963), 16 K below 20 °C
  air.

What a surface sends is ε·B(T) + (1 − ε)·B(T_amb), with B Planck's radiance over the camera's band
(`sensor:thermal:bandMin`/`bandMax`, 8–14 µm by default; the exact series of Widger and Woodall
1976), and the camera reports the temperature a black body would need for it, as radiometric
cameras do. Each pixel then gets Gaussian noise of `sensor:thermal:netd` (1 σ, Philox). Outputs:
`sensor/<n>/temperature` (r32f, K) and `image`, the temperatures in a palette
(`sensor:thermal:palette` white_hot, black_hot or ironbow, over `displayMin`/`displayMax` or each
frame's range, at least 4 K); depth and instance are as for any camera. ROS: `/<name>/temperature/image_raw`
(32FC1, K) beside `/<name>/image_raw`. URDF `<sensor type="thermal">` or `"thermal_camera"`, and
SDFormat's `thermal_camera`, give a thermal camera with the defaults. It needs the site's
temperature (`--ambient-temperature`, `--weather temperature=` or the profile's
`thermal.ambient_temperature`) and refuses to start without it. Sample:
`assets/robots/amr_diffdrive/amr_diffdrive_thermal.usda`.

Checked in `test_thermal_camera`: the black-body fractions of Incropera et al. (Fundamentals of
Heat and Mass Transfer, Table 12.1) at four λT within 1·10⁻⁵ (the table's C₂ is 14 388 µm·K,
CODATA's 14 387.77), the band radiance against a direct integration of Planck's law within 10⁻⁹
and Stefan–Boltzmann over all wavelengths within 10⁻⁶, the apparent temperature of a grey
surface, Swinbank's sky, and a run that sees a 350 K node with ε 0.95 at its apparent temperature
(about 2.6 K low), the air and the sky, with the noise at its NETD. Not modelled: the atmosphere's
own absorption and emission (small over tens of metres in clear air, not in fog; the visible fog
veil is left off for thermal cameras), the sun's heating of surfaces and the ground, the sky
reflected in upward faces (surfaces reflect the ambient air), clouds, the optics' blur and the
detector's fixed-pattern noise and drift. There is no measured thermal image to compare with.

Since 1.2.x the per-pixel part runs on the GPU right after the render
(`gpu/kernels/camera/thermal_camera.slang`): each pixel's surface temperature from a table of the
frame's apparent temperatures by prim (worked out on the CPU, a few per frame), the noise from the
reference's Philox bits (pixel u of a row takes block u of the row's stream), the frame's range
by atomics, then the palette. `Camera::stepThermal` stays the CPU reference
(`LAGRANGE_THERMAL_CAMERA=cpu`) and is used with a sensor model or a fisheye. The sums are in float
on the GPU: `test_thermal_camera` (`TheGpuImageAgreesWithTheCpuReference`, every palette, each
frame's range and a fixed one) checks the temperatures within 10⁻³ K (3·10⁻⁵ K on the RTX 4060 and
on llvmpipe), the same ids and every colour within one code (all the same there).

![The sample AMR's thermal camera on open ground: a person at skin temperature, 32.5 °C (white),
the ground at the 20 °C air (red), the clear sky at 3.7 °C (black), ironbow
palette](../../imgs/thermal-camera-person.png)

**Radar**: ray-launched returns with radar cross-section from `sensmat:radarCrossSectionDensity`,
radial velocity from relative motion, a range–Doppler–azimuth detection list after a
constant-false-alarm threshold; rain attenuation from weather. Raw ADC cubes are not produced.

Status in v0.6 (`modules/sensors/include/lagrange/sensors/Radar.hpp`, `RadarAPI_1`): rays at
half the azimuth resolution over the field of view, each echo σ = ρ·r²·ΔΩ·cos θ (Lambertian
back-scatter of the body's `sensmat:radarCrossSectionDensity`, 0.1 without one); echoes of one
object in one range cell are one detection (power-weighted range, angles, radial velocity from
the bodies' twists); SNR by the radar equation against the effective noise floor, a fixed
threshold standing in for CFAR, noise of resolution/√(2·SNR); at most 64 detections, strongest
first. Ground-plane hits are dropped as clutter. Checked in `test_radar`: a plate's σ = ρ·A
within 3 %, a 10 m/s Doppler, the detection range from the radar equation. ROS:
`/<name>/detections` PointCloud2 (x, y, z, velocity, rcs, snr). The sample sedan has a front
radar. Owed: rain attenuation, multipath and ghosts, a published RCS reference.

Viewer: a viewer that subscribes to a radar's prim (`SubscribeSensor`; a `RadarAPI_1` radar or an
`imaging_radar`) gets each new scan as a `SensorFrame` of points with fields x, y, z (sensor frame),
velocity, rcs and snr (no protocol change: `PointCloudData` names its fields). The viewport draws
them under Overlays → Radar detections (`--overlays radar`): boxes sized by RCS (−20 to +20 dBsm),
coloured by radial velocity on the design's diverging map (`viewer/client` `RadarScale.hpp`), a
Doppler tick along the line of sight, and the field of view as a faint wedge to its range from the
prim's attributes; the Sensors workspace has a Radar view (fan and range–azimuth).

Placement: on the CPU the rays and their echoes run in parallel (TBB) and are summed in ray order,
so the deterministic reference keeps its numbers bit for bit; with `--lidar gpu` the rays go to the
lidar's kernel (`bvh_traverse.slang`, float ranges, Tier 1, not deterministic) and the echoes stay
on the CPU. `test_radar` checks the GPU finds the CPU's detections within 5 mm, 1 mrad, 1 cm/s and
1 % of cross-section.

### 4D imaging radar (as built)

Kind `imaging_radar` (a built-in custom kind, §9; URDF `<sensor type="imaging_radar">` with its
settings as leaf elements, USD `CustomSensorAPI_1` with `sensor:custom:kind = "imaging_radar"`
and `sensor:custom:param:<name>`; `RadarAPI_1` is unchanged) is a cascaded-MIMO automotive radar
of the kind that reports range, azimuth, elevation and Doppler per detection. It shares the
radar's rays and echoes (`Radar.cpp`, `sensors::createImagingRadar`): the same settings as
`RadarAPI_1` (`max_range`, `azimuth_fov`, `elevation_fov`, `tx_power`, `antenna_gain`,
`noise_floor`, `detection_threshold`, `carrier_frequency`, `range_resolution`,
`velocity_resolution`) plus:

| Setting | Meaning | Default |
|---|---|---|
| `azimuth_elements`, `elevation_elements` | virtual channels (N_TX × N_RX) along each axis | none: `azimuth_resolution`, `elevation_resolution` (rad) |
| `azimuth_resolution`, `elevation_resolution` | angle cells at boresight (rad) without elements | 0.035, `azimuth_resolution` |
| `element_spacing` | d, in wavelengths | 0.5 |
| `bandwidth` | sweep B (Hz): `range_resolution` = c/(2B) unless given | none |
| `chirp_period`, `chirps` | T_c (s) and N_c: unambiguous velocity ±λ/(4·T_c), `velocity_resolution` = λ/(2·N_c·T_c) unless given | none |
| `max_velocity` | the unambiguous velocity (m/s), directly | λ/(4·T_c), else no folding |
| `unambiguous_range` | ranges beyond it fold back, r mod R_u (m) | no folding |
| `ghost_reflection` | the ground's power reflection \|Γ\|² for multipath ghosts | 0: none |
| `peak_grouping` | 1: cells merge into their local peak; 0: every cell over the threshold is a detection | 1 |
| `max_detections` | detections per scan (1..4096) | 256 |
| `ray_spacing` | rad between rays | half the finer angle cell at boresight |

The radar's resolution cells: range cells of `range_resolution`, Doppler cells of
`velocity_resolution` after folding into [−v_max, v_max), and angle cells uniform in direction
sine (u = cos el · sin az, w = sin el) of width 1/(N·d) for N channels d wavelengths apart, the
Rayleigh limit λ/(N·d) at boresight that widens as 1/cos θ off it (S. Rao,
"Introduction to mmWave Sensing: FMCW Radars", Texas Instruments mmWave training, 2017
(TODO(verify): its TI literature number): angle resolution θ_res = λ/(N·d·cos θ); range
resolution c/2B, v_max = λ/4T_c and v_res = λ/2T_f there too). Every echo (σ as the radar's,
`sensmat:radarCrossSectionDensity`) falls in its cell; with peak grouping each cell climbs to its
strongest neighbour (±1 cell in each of the four dimensions) and a local peak takes in every
cell that climbs to it, so one target spread over neighbouring cells by the grid is one
detection, and two targets nearer than about a cell merge (two with an emptier cell between them
stay two). A peak whose summed signal Σσ/R⁴ (at the true ranges) beats the threshold by the
radar equation is a detection: range, angles, folded velocity as signal-weighted means, σ the
sum. Unlike the `RadarAPI_1` radar, which makes one detection per object and range cell, an
object larger than a cell gives several, and objects in one cell share one. Ghosts: each echo
also returns by the fixed ground plane both ways, from the target's mirror image below the
ground (range to the image, its Doppler with the mirrored velocity, power × |Γ|⁴), which is what
puts imaging radars' ghost points under the road; the two one-bounce paths, which add coherently
to the direct return, and occlusion of the bounce are not modelled. Noise: range, velocity and
the angle cells over √(2·SNR), as the radar's. Writes `sensor/<n>/detections`
(`max_detections` × 6: range, azimuth, elevation, velocity, rcs, snr; NaN beyond the count) and
`sensor/<n>/count`; ROS `/<name>/detections` is the radar's PointCloud2 (x, y, z from range and
both angles, velocity, rcs, snr). CPU (deterministic) and GPU (`--lidar gpu`) rays as the radar's.
Sample: `assets/robots/sedan/sedan_4d_radar.usda` (64 × 16 channels, 1 GHz, 256 chirps of 20 µs;
illustrative, not a product's datasheet).

Checked in `test_kinds_imaging_radar`: a plate's σ = ρ·A and its SNR by the radar equation, the
detection range the equation gives; the angle cell λ/(N·d); two targets 0.08 apart in sine merge
with 16 channels (cells of 0.125) and part with 64, two 0.3 apart part with 16; two targets
0.23 rad apart in elevation part with 16 elevation channels at their heights and merge with one;
a plate receding at 10 m/s reads −2 m/s at v_max = 6 m/s (from T_c = λ/24 or given); Doppler
alone parts two targets in one range and angle cell; a 20 m plate at 5 m with R_u = 15 m; the
ghost below the ground at |Γ|⁴ of the plate's σ and at its mirror range; the CPU reference
repeats bit for bit; GPU against CPU (counts within 10 %, σ totals within 1 %, the strongest
detections within 5 cm, 5 mrad, 0.1 m/s). `test_radar` pins the `RadarAPI_1` radar's CPU
numbers bit for bit. Owed: speckle (a coherent sum per cell, which makes real point clouds of
walls dense), sidelobes and the array's grating lobes, the one-bounce multipath paths, guard-rail
ghosts, CA/OS-CFAR against the clutter around a cell (the threshold is fixed over the noise
floor), rain attenuation, a published 4D-radar dataset to check against.

**IMU**: specific force and angular rate from `body/*/acceleration` and `body/*/twist` at the
sensor frame, plus white noise (noise density), bias instability (first-order Gauss–Markov),
rate random walk, scale-factor and misalignment errors, saturation at the range limits,
quantisation. Parameters map directly to Allan-variance datasheet values.

Status in v0.2 (`modules/sensors/include/lagrange/sensors/ImuNoise.hpp`): white noise (per
sample, or as a density N), turn-on bias, Gauss-Markov bias instability, rate random walk K,
saturation and quantisation, per channel; the names follow SDFormat's `<noise>` elements, which
the URDF loader reads. Scale-factor and misalignment errors are not modelled yet. The bias is
saved with the module state and draws from its own Philox streams, so it leaves the white noise
of a run unchanged. `test_imu_noise` checks the Allan deviation of a 4-hour static record
against N²/τ + K²τ/3 (IEEE Std 952): within 1 % up to τ = 100 s and 4.3 % at 1000 s.
`<dynamic_bias_stddev>` σ_b is the deviation of the noise that drives the Gauss-Markov bias
("the standard deviation of the noise used to drive a process to model slow variations in a sensor
bias", SDFormat 1.12 `noise.sdf`), as Gazebo integrates it (gz-sensors `GaussianNoiseModel.cc`:
b ← e^(−dt/τ)·b + N(0, σ_b²·τ/2·(1 − e^(−2dt/τ)))), so the bias's stationary deviation is
σ_b·√(τ/2): 1e-3 with τ = 3600 s gives 0.042. Until 2026-11-12 Lagrange took σ_b as the
stationary deviation itself.

**GNSS**: position from the body pose converted through the local frame to WGS84
(`core/world/coords/`), Gaussian horizontal and vertical error with first-order time
correlation, velocity error, fix-loss when sky view is blocked (ray casts to a fixed set of
sky directions as a stand-in for satellites), optional `urban` multipath bias.

**Magnetometer**: Earth field (constant vector given by the scene, or a world magnetic model
evaluated at the geo-origin; the model coefficients are a data file with an epoch), rotated into
the body frame, soft- and hard-iron distortion, noise, plus `emi/{p}/b_field`.

**Timing** (all sensors, from `SensorAPI`), as built: a sample is taken on the scheduler at
`k / rate`, and its ROS message goes out `latency` plus jitter later (Gaussian, cut at 3σ, never
out of order, on the simulation's clock), still stamped with the sampling time. Set it with
`sensor:latency` and `sensor:jitter`, `<latency>` and `<jitter>` in a URDF's `<gazebo>` sensor,
`--sensor-timing NAME=LATENCY[,JITTER]` or the run file's `sensor_timing:`. The sensor's own
`sensor/<n>/<q>` stay on time as the ground truth, as do `/ground_truth` and `/tf`; every other
consumer can have what the robot gets: `lagrange_sensors`' delay line (`sensors::DelayLine`, only
for sensors with a delay) writes `sensor/<n>/delayed/<q>` for each of its numbers and images, and
`sensor/<n>/delayed/stamp` (s), the sampling time of what is there now, -1 before the first
arrives. Plugins read those like any signal, recordings and Python get them, and the viewer's
lidar points and camera image are the delayed ones. The delay line ticks at 1 kHz, so a delay
rounds up to the next millisecond, and a signal commits only when a value arrives (`SignalBus::hold`).
Each sample's delay is a function of the run seed, the sensor and
the sampling time, so runs and restored checkpoints repeat; the ROS bridge draws its own jitter
(same latency and spread, other numbers) and goes out on its publish tick. Sampling instants do
not jitter.

## 3. Signals

```cpp
// modules/sensors — declare()
for (auto& s : world.instancesOf("SensorAPI_1")) {
    auto n = name(s);  auto b = bodyOf(s);
    d.input({.name = fmt("body/{}/pose", b),         .type = SignalType::Pose,   .unit = units::m});
    d.input({.name = fmt("body/{}/twist", b),        .type = SignalType::Wrench, .unit = units::m_per_s});
    d.input({.name = fmt("sensor/{}/contamination", n), .type = SignalType::Scalar, .unit = units::one,
             .optional = true, .defaultValue = 0.0});                       // weather
    if (auto load = powerLoadOf(s))
        d.input({.name = fmt("load/{}/powered", *load), .type = SignalType::Bool, .unit = units::one,
                 .optional = true, .defaultValue = 1.0});                   // electrical: brownout turns sensors off
    d.output({.name = fmt("sensor/{}/meta", n), .type = SignalType::Bytes, .unit = units::one});   // stamp, sequence, status

    if (has(s, "LidarAPI_1")) {
        d.input ({.name = "env/extinction/nir_905", .type = SignalType::Scalar, .unit = units::per_m,
                  .optional = true, .defaultValue = 0.0});
        d.input ({.name = "env/rain_rate", .type = SignalType::Scalar, .unit = units::m_per_s,
                  .optional = true, .defaultValue = 0.0});
        d.output({.name = fmt("sensor/{}/points", n), .type = SignalType::PointCloud, .unit = units::m});
    }
    if (has(s, "CameraSensorAPI_1")) {
        d.input ({.name = "env/extinction/visible", .type = SignalType::Scalar, .unit = units::per_m,
                  .optional = true, .defaultValue = 0.0});
        d.input ({.name = "env/solar_irradiance", .type = SignalType::Scalar, .unit = units::W_per_m2, .optional = true});
        d.input ({.name = "env/sun_direction", .type = SignalType::Vec3, .unit = units::one, .optional = true});
        d.output({.name = fmt("sensor/{}/image", n), .type = SignalType::Image, .unit = units::one});
        d.output({.name = fmt("sensor/{}/depth", n), .type = SignalType::Image, .unit = units::m});
    }
    if (has(s, "RadarAPI_1")) {
        d.input ({.name = "env/radar_attenuation_77ghz", .type = SignalType::Scalar, .unit = units::per_m,
                  .optional = true, .defaultValue = 0.0});
        d.output({.name = fmt("sensor/{}/detections", n), .type = SignalType::PointCloud, .unit = units::m});
    }
    if (has(s, "ImuAPI_1")) {
        d.input ({.name = fmt("body/{}/acceleration", b), .type = SignalType::Wrench, .unit = units::m_per_s2});
        d.input ({.name = fmt("emi/{}/e_field_rms", n), .type = SignalType::Scalar, .unit = units::V_per_m,
                  .optional = true, .defaultValue = 0.0});
        d.output({.name = fmt("sensor/{}/specific_force", n), .type = SignalType::Vec3, .unit = units::m_per_s2, .frame = "sensor"});
        d.output({.name = fmt("sensor/{}/angular_rate", n),   .type = SignalType::Vec3, .unit = units::rad_per_s, .frame = "sensor"});
    }
    if (has(s, "GnssAPI_1")) {
        d.output({.name = fmt("sensor/{}/geodetic", n), .type = SignalType::ScalarArray, .unit = units::one,
                  .arrayLength = 3});                    // latitude (rad), longitude (rad), height (m): see note
        d.output({.name = fmt("sensor/{}/velocity_enu", n), .type = SignalType::Vec3, .unit = units::m_per_s});
        d.output({.name = fmt("sensor/{}/fix", n), .type = SignalType::Int, .unit = units::one});
    }
    if (has(s, "MagnetometerAPI_1")) {
        d.input ({.name = fmt("emi/{}/b_field", n), .type = SignalType::Vec3, .unit = units::T,
                  .optional = true, .frame = "body"});
        d.output({.name = fmt("sensor/{}/magnetic_field", n), .type = SignalType::Vec3, .unit = units::T, .frame = "sensor"});
    }
}
```

Implementation note (v0.2): a lidar writes `sensor/{n}/ranges` (ScalarArray, one range per
beam, m) instead of a `PointCloud` handle, because the bus's buffer table for handle types does not
exist yet. Beams are rings of columns (`LidarLayout` in `core/world`: SDFormat's horizontal and
vertical samples and angles, ring after ring from the lowest); a single ring is a 2D scanner. The
server turns ranges into the viewer's point frames, and the ROS bridge into `sensor_msgs/LaserScan`
for 2D lidars or an organised `sensor_msgs/PointCloud2` (x, y, z, intensity, ring) for 3D ones.
On Tier 0, rays are cast on the CPU with the exact closed-form tests in double, and Embree's
bounding-volume hierarchy (user geometry: a padded bounding sphere per shape; the shapes that
have never moved in a hierarchy built once, the ones that have in a second one rebuilt when they
move) only skips the shapes a ray cannot reach; planes are tested directly. The result
is bit-identical to testing every shape (`ShapeCaster`, `test_shape_caster`), beams run in
parallel with TBB, and the run stays deterministic. `--lidar auto` uses it: it outruns the GPU
round trip for scans of this size (docs/design/benchmarks.md). On Tier 1 (`--lidar gpu`), rays
are cast by `gpu/kernels/lidar/bvh_traverse.slang`: the same closed-form tests in float, one
thread per beam, through a bounding-volume hierarchy built on the CPU (median split, uploaded)
and refitted to each scan's shapes, rebuilt when refitting has grown its inner boxes' area by a
quarter or after 30 scans. Shapes are taken relative to the sensor and combined with its orientation in double
before the conversion to float, and spheres and cylinders use the distance-to-line form of the
discriminant (Ray Tracing Gems, ch. 7), so grazing rays keep their precision. Range limits and
Philox noise are applied on the CPU after the cast, keyed by the same module name, so both tiers
add the same noise. The GPU lidar declares itself not deterministic; the server uses it only when
the run takes ROS input, or with `--lidar gpu` (acceptance criterion 6 is
`modules/sensors/test/test_lidar_tiers.cpp`: 100 000 random beams in 400 random shapes give the
same shape for every beam and a worst range error of 0.16 mm on an Intel Iris Xe and 0.31 mm on
llvmpipe, within 300 m). The IMU also writes `sensor/{n}/orientation` (ground truth) for
`sensor_msgs/Imu`. Sensors come from Gazebo `<sensor>` blocks of a URDF until `SensorAPI_1` exists.

Note on GNSS: a three-element array with mixed units (rad, rad, m) cannot carry one `Unit`. The
design options are three scalar signals (`latitude`, `longitude`, `height`) or a dedicated
`Geodetic` signal type. The listing shows the problem on purpose; RFC 0001 unresolved question 2
recommends three scalar signals.

## 4. Step size

The sensors module is scheduled at the smallest period any sensor needs (default 1 ms when an
IMU is present, otherwise `1 / max(rate)`), and each sensor fires on its own schedule within it.
Rendering sensors (camera, lidar, radar) submit GPU work and collect results one or more ticks
later; the latency model absorbs this, and a sensor whose configured latency is shorter than the
measured pipeline delay raises a warning in real-time mode. In `deterministic` mode the scheduler
waits for the result so that delivery ticks do not depend on GPU timing.

## 5. Substitutes and contract

Contract: every `sensor/{s}/*` output above.

| Profile token | Kind | Behaviour |
|---|---|---|
| `ground_truth` | ideal | No rendering. IMU, GNSS, magnetometer and encoders publish exact values with correct timing. Lidar and camera outputs are published as empty buffers with valid metadata, and `body/*/pose` is published to ROS as ground-truth odometry and TF. This is the high-speed planner-testing mode in CLAUDE.md §11.2 |
| `replay` | replay | Sensor outputs from MCAP |

The profile key `sensors.only` enables physical simulation for the named sensors and ground
truth for the rest, which is the common case of "simulate the lidar, skip the cameras".

## 6. Placement

| Sensor | Tier 0 | Tier 1 | Tier 2 |
|---|---|---|---|
| Lidar | Embree `rtcIntersect` packets on the worker pool | `gpu/kernels/lidar/bvh_traverse.slang` (compute, LBVH built on GPU or uploaded from CPU) | `gpu/kernels/lidar/rt_query.slang` (ray query) |
| Camera | shared raster renderer on a software Vulkan device; documented as slow | GPU raster | GPU raster + optional RT |
| Radar | Embree (shapes) with TBB | compute BVH (`--lidar gpu`) | ray query |
| IMU, GNSS, magnetometer, encoders | CPU | CPU | CPU |

Output buffers stay on the GPU when a consumer can take them: the viewer through shared memory
after one readback, ROS 2 through `rosidl::Buffer` with the CUDA backend
(`ros-lyrical-cuda-buffer`) when `bridges.ros2.gpu_buffers` is on. A Vulkan-to-CUDA hand-off
needs external-memory interop in `gpu/hal/`; that is planned for v0.6 and tracked as a risk (R11).

The same kernels serve all three lidar tiers at the level of "one ray → one hit record"; only the
traversal differs. Tests compare tier outputs on the same scene (section 7).

## 7. Validation

| Test | Reference | Tolerance |
|---|---|---|
| Lidar geometry | analytic scenes (plane, sphere, corner) | range error < 1 mm before noise |
| Tier agreement | same scene on Tier 0, 1, 2 | identical hit prim for > 99.9 % of rays; range within 1 mm |
| Lidar intensity | Lambertian target at known range and angle | 2 % |
| Camera projection and distortion | synthetic checkerboard; calibrate with OpenCV and recover the intrinsics | reprojection error < 0.1 px |
| Camera noise | photon-transfer curve (variance vs mean) | slope within 3 % of the configured gain |
| Rolling shutter | skew of a vertical edge at known speed | 1 px |
| IMU noise | Allan deviation of a 4-hour simulated static record vs configured parameters | within the estimator's confidence band |
| GNSS | error statistics and correlation time | 5 % |
| Timing | measured rate, latency and jitter histograms | 1 base tick |
| Sim-vs-real (per release) | public datasets with ground truth and sensor specifications (candidates: an indoor 2D-lidar AMR dataset for v0.2; an automotive dataset for v0.6; choose and check licenses) | reported per metric; thresholds set from the dataset |

## 8. GNSS, magnetometer and barometer (v0.4)

`GeoSensors.cpp` in `lagrange_sensors`, from `GnssAPI_1`, `MagnetometerAPI_1` and
`BarometerAPI_1` prims. They place the sensor on the Earth through the world's geo origin
(`world.geo_origin` in a profile, `--geo-origin`), a site parameter with no default: without one
they are left out and the server logs why.

| Sensor | Signals | Model | Check |
|---|---|---|---|
| GNSS | `latitude`, `longitude` (rad), `altitude` (m, ellipsoidal), `velocity` (ENU, m/s), `fix` | The antenna's pose through the geo origin, velocity with the lever arm; white noise per fix | exact against `GeoOrigin` (`test_geo_sensors`) |
| Magnetometer | `field` (T, sensor frame) | World Magnetic Model 2025 main field (`Wmm.hpp`, data in `data/wmm2025`, public domain) at the config's `decimal_year` (default 2025.0); white noise | within 0.001 nT of NOAA's 100 test points (`test_wmm`); rotation exact |
| Barometer | `pressure` (Pa), `temperature` (K) | ICAO standard atmosphere at the sensor's ellipsoidal height (troposphere); white noise | ICAO table values (`test_geo_sensors`) |

Not yet: GNSS outages, multipath and latency; crustal field, hard- and soft-iron effects; the
geoid (ellipsoidal height stands for altitude above mean sea level); the site's weather instead of
the standard day (v0.5); URDF and SDFormat `<sensor type="gps|magnetometer|altimeter">` import.

## Lidar intensity from materials (as built)

`sensor/<name>/intensities` holds each beam's intensity next to `ranges`; it reaches `/scan`'s
`intensities`, `/points`' `intensity` and the viewer. It is the reflected power relative to a white
wall head-on at 1 m, capped at 1: `ρ·((1−s)·cos θ + s·lobe(θ))·(1 m / r)²`, with ρ the reflectivity at the
lidar's wavelength (905 or 1550 nm), θ the incidence angle from the true surface normal, s the
specular fraction and `lobe` a mirror's, 2° wide (illustrative). A retroreflector gives `100·ρ·(1 m / r)²` at any angle.
A beam passes a surface with `transmission` by chance (seeded per step and beam; CPU only: the GPU tier gets the first hit).
A surface with a material gives no return when its power is under what the lidar needs at its
range limit for a target of `max_range_reflectivity` (default 0.1), so water and dark surfaces drop
out at a slant or from far. Surfaces with no material keep ρ = 0.5 and always return.

Give a surface a material with `SensorMaterialAPI_1` on a collider prim, an ancestor or its bound
material (`sensmat:preset`, then any authored attribute), or in URDF:

```xml
<gazebo reference="deck">
  <sensor_material preset="asphalt" reflectivity_905="0.12"/>
</gazebo>
```

Presets (`default`, `water`, `glass`, `mirror`, `asphalt`, `concrete`, `white_paint`, `black_rubber`,
`retroreflector`) are illustrative orders of magnitude, not measurements. The formula is tested against
its closed form; a real lidar's intensity channel has not been compared with it.

## Lidar motion distortion (as built)

A rotating lidar's columns fire one after another through the scan, so a moving robot's scan skews.
Column `c` of `n` fires at the fraction `(c + 1) / n` of the scan, from the sensor's pose at that moment,
interpolated between the last scan's end and this one's (linear position, slerp). So a motion that
changes within one scan is smoothed over it. On by default on both lidar tiers
(`sensor:lidar:motionDistortion`, or `<ray><motion_distortion>0</motion_distortion></ray>` in a URDF's
`<gazebo>` sensor; an SDFormat file cannot turn it off yet). Only the sensor's motion counts: other bodies stay where they are at the scan's end, and
every lidar sweeps once per output, whatever `sensor:lidar:rotationRate` says. The first scan of a run has
no motion to use.

So that the scan can be undone, `/scan` is stamped when its first column fired, with `time_increment`
and `scan_time`, and `/points` the same with a `time` field (float32, s after the stamp). With distortion off,
the stamp is the scan's end and those are zero (`scan_time` stays).

## Lidar scan patterns (as built)

A lidar's beams are a grid by default: `samples` columns evenly spaced in azimuth from `min_angle` to
`max_angle`, `vertical_samples` rings evenly spaced in elevation (`lagrange/world/LidarLayout.hpp`), at most 2²² beams
in a scan (a count below 1 or over that is refused).
Three opt-in patterns change where the beams go; without their settings every beam stays exactly
where it was (tested bit for bit). All three reach `/points` (`sensor_msgs/PointCloud2`) and the
viewer; `/scan` stays for 2D grids only.

**Channel table** (Velodyne style). Real multi-beam lidars space their channels unevenly (denser near
the horizon). List one elevation per channel, in radians, in any order; the rings are these sorted
from the lowest up (the `ring` field of `/points`), and `vertical_samples` and the vertical angles are
then ignored:

```xml
<ray>
  <scan>
    <horizontal><samples>1800</samples><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
    <vertical><angles>-0.4363 -0.0175 0.0262 -0.1396 0.2618 -0.0087</angles></vertical>
  </scan>
</ray>
```

Settings `vertical_angle_0`, `vertical_angle_1`, ... (rad).

**Flash lidar** (solid state). A focal-plane array fires every beam at once, so its scan has no motion
distortion (`motion_distortion` is set to 0 and `/points`' `time` is 0). Its beams are a pinhole's pixels:
evenly spaced on the image plane, so the tangent of a beam's angle, not the angle, is linear in its
column and ring; `min_angle`/`max_angle` and the vertical angles are the outer beams' angles and must lie
within ±π/2 (the run refuses to start otherwise). `/points` is organised as rings × columns.

```xml
<ray>
  <pattern>flash</pattern>
  <scan>
    <horizontal><samples>320</samples><min_angle>-0.5236</min_angle><max_angle>0.5236</max_angle></horizontal>
    <vertical><samples>240</samples><min_angle>-0.3927</min_angle><max_angle>0.3927</max_angle></vertical>
  </scan>
</ray>
```

Setting `pattern` 1.

**Rosette** (Livox Mid-40/Mid-70 style non-repetitive scan). The beam passes two wedge (Risley) prisms
turning at different, incommensurate speeds; to first order each deflects it by a fixed angle δᵢ in the
direction its wedge points, and the deflections add as vectors (Marshall, "Risley prism scan patterns",
Proc. SPIE 3787, 1999; Livox's own account: Liu, Zhang and Hong, "Low-cost retina-like robotic lidars
based on incommensurable scanning", IEEE/ASME Trans. Mechatronics 27(1), 2022):

    δ(t) = δ₁ (cos ω₁t, sin ω₁t) + δ₂ (cos ω₂t, sin ω₂t)

`|δ|` is the beam's angle off the sensor's x axis, its direction the way round it. With equal prisms the
beam traces petals through the centre (`|δ| = 2δ|cos((ω₁ − ω₂)t / 2)|`); since the speeds are
incommensurate the petals never repeat, so each scan goes on where the last stopped and the field fills
up with integration time. Each scan holds `samples` points fired one after another at
`rosette_point_rate`; scan k's point i fires at t = (k·samples + i) / rate (so set
`samples = rate / update_rate` for the real picture). The beams change every scan, so the module also
writes `sensor/<name>/directions` (ScalarArray, 3 × samples, unit vectors x y z in the sensor frame, per
beam) and `/points` and the viewer take each scan's beams from it. `/points` is one row of `samples`
points; each point's `time` is when it fired (with motion distortion on, the default, each point is cast
from the sensor's pose at that moment).

```xml
<ray>
  <pattern>rosette</pattern>
  <scan><horizontal><samples>10000</samples></horizontal></scan>
  <rosette>
    <fov>0.6702</fov>                <!-- full cone angle, rad: 38.4°, Mid-40 -->
    <point_rate>100000</point_rate>  <!-- points/s -->
    <speed_1>763.83</speed_1>        <!-- rad/s, prism 1 -->
    <speed_2>-488.42</speed_2>       <!-- rad/s, prism 2; opposite sign: counter-rotating -->
    <prism_ratio>1</prism_ratio>     <!-- δ₂/δ₁; ≠ 1 leaves a hole of |δ₁ − δ₂| at the centre -->
  </rosette>
</ray>
```

| Setting | Unit | Default |
|---|---|---|
| `pattern` | — | 2 for the rosette |
| `samples` | points per scan | 10000 (when the rosette's URDF gives none) |
| `rosette_fov` | rad, full cone (δ₁ + δ₂ = fov / 2) | 0.670206 (38.4°: Livox Mid Series User Manual v1.2, 2019.08, table 1.2.1) |
| `rosette_point_rate` | 1/s | 100000 (same table) |
| `rosette_speed_1`, `rosette_speed_2` | rad/s | 2π·7294/60, −2π·4664/60 (7294 and −4664 rpm: the prototype in Livox's paper, Liu, Zhang and Hong, arXiv:2006.11034, appendix B; Livox does not publish the Mid-40's own, so a recorded Mid-40 pattern may differ) |
| `rosette_prism_ratio` | δ₂/δ₁ | 1 |

With the defaults one 0.1 s scan hits under half of the disc's 0.5° cells and a second about all of
them; the shape is checked against the closed forms (reach δ₁ ± δ₂, the radial distribution
`P(|δ| ≤ R) = 1 − (2/π) arccos(R / 2δ)` for equal prisms, coverage growing every scan, scans not
repeating, the same scan and beam always the same direction) and both lidar tiers cast it
(`test_lidar_patterns`). Not a measured Livox pattern: the first-order model leaves out the prisms'
refraction nonlinearity (third-order Risley theory) and the Mid-40's actual speeds; the Avia's own
non-repetitive pattern (70.4° × 77.2°, six beams) is not modelled.

These patterns come from all three formats (RFC 0015): URDF (`<gazebo reference=L><sensor
type="ray|lidar">`, as above); SDFormat, whose `<lidar>` has no place for them, from Lagrange's custom
elements `<lagrange:pattern>`, `<scan><vertical><lagrange:angles>` and `<lagrange:rosette>` (or the
plain names, as in URDF; formats.md §9.3); and USD, `LidarAPI_1`'s `scanPattern` `flash` or `rosette`,
`channelElevations` and `rosette:fov`, `pointRate`, `speed1`, `speed2`, `prismRatio` (formats.md §3.8).
The rosette's USD attributes have no fallback: unauthored, the values above are assumed and the
loader says so. `lagrange_import` writes all of it, and the round trip URDF → USD → run gives the same
beams bit for bit (`OpticsImport.PatternsAndLensesSurviveTheTripThroughUsd`).

## Fisheye cameras (as built)

A camera can be a fisheye up to all round (`lagrange/world/FisheyeLens.hpp`,
`modules/sensors/src/Fisheye.cpp`), for surround-view rigs and wide-angle navigation cameras. The
projection is Kannala–Brandt as OpenCV's fisheye module and ROS write it: a ray θ off the optical axis
lands `f·θ_d` from the principal point, `θ_d = θ(1 + k₁θ² + k₂θ⁴ + k₃θ⁶ + k₄θ⁸)` (Kannala and Brandt,
"A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses",
IEEE TPAMI 28(8), 2006); all k = 0 is the ideal equidistant lens, r = fθ. `horizontal_fov` is then the
lens's angle across the image's width (`f = (width/2) / θ_d(fov/2)`, as Gazebo's wide-angle camera with
`scale_to_hfov`); the lens sees nothing beyond `cutoff_angle` (half-angle, default the larger of fov/2
and 90°, as Gazebo), where pixels are black, at infinite range, with no instance. Up to 2π.

```xml
<sensor name="fisheye" type="camera">
  <camera>
    <horizontal_fov>3.4907</horizontal_fov>   <!-- 200° across the width -->
    <image><width>1024</width><height>1024</height></image>
    <lens>
      <type>equidistant</type>                <!-- or kannala_brandt; gnomonical is the pinhole -->
      <cutoff_angle>1.7453</cutoff_angle>     <!-- rad -->
      <k1>-0.013</k1><k2>0.0042</k2><k3>-0.0011</k3><k4>0.0001</k4>  <!-- Lagrange's: OpenCV fisheye -->
    </lens>
  </camera>
</sensor>
```

Settings `projection` 1, `distortion_model` 2, `distortion_k1`..`distortion_k4`, `fisheye_cutoff` (rad).
A lens whose θ_d turns back before the cutoff is refused.

How it is drawn: a pinhole render cannot reach 180°, so the camera renders the faces of a cube around
itself (front, right, left, down, up, back: only those some pixel looks through, and of each only the
window those pixels need, at the fisheye's focal length so the centre keeps its resolution), then each
output pixel takes the face its ray crosses: colour bilinear in linear light, depth and instance id from
the nearest render pixel. Depth is the **range along the pixel's ray** (m), not along the optical axis,
which means nothing at 90° and beyond. `/<name>/camera_info` is `distortion_model: equidistant`,
`D = [k1, k2, k3, k4]`, `K` with that f (the ROS `camera_calibration` fisheye convention), so
`cv::fisheye::undistortImage` takes it as is. A 200° fisheye costs five renders instead of one.

Tested: projection and its inverse round trip within 1 µrad up to 150° for realistic k
(`test_fisheye`), the face choice and the range from synthetic renders, and a 240° camera drawn on the
Vulkan device (llvmpipe in CI) sees a cube beside it at 90° and one behind it at 112° where the formula
puts them, at their ranges.

Not yet: vignetting, rolling shutter and exposure together with a fisheye (refused at start); a fog
field's veil and the `detections`/`stereo_camera`/`event_camera` kinds, which still take the camera as a
pinhole; other lens types (stereographic, equisolid, orthographic).

From SDFormat (RFC 0015): `<lens><type>equidistant</type>` and `<cutoff_angle>` as SDFormat defines
them, k1..k4 from `<lens><lagrange:k1>`..`<lagrange:k4>` (or plain `<k1>`.., as in URDF; formats.md
§9.3). From USD: `CameraSensorAPI_1`'s `lensType = "equidistant"` selects this fisheye, with k1..k4 from
`distortionModel = "equidistant"` and `distortionCoeffs` and the cutoff from `cutoffAngle`; the angle
across the width comes from `UsdGeomCamera`'s focal length f and horizontal aperture w through the lens,
`2 θ_d⁻¹(w / 2f)`, so a 200° lens has a positive focal length (formats.md §3.8). Without `lensType`, an
`equidistant` `distortionModel` stays what it was: the same distortion on the pinhole render, below
about 160°. `lagrange_import` writes the lens, and the round trip gives the same k and cutoff exactly
and the same angle to the float precision of `focalLength`.

## Sensor faults (as built)

To test how a robot stack copes with a failing sensor, any sensor can be given faults: the built-in
types, the built-in custom kinds of §9 and a plugin's custom sensors alike. They are off unless asked
for, and change what every consumer gets: ROS, plugins, the delay line, recordings, Python and the
viewer all read the faulted `sensor/<n>/<q>`. Ground truth stays where it always is, in the bodies'
and joints' signals (`/ground_truth/odom`, `/tf`).

| Mode | What the sensor delivers | Keys (besides `at`, `for`) |
|---|---|---|
| `dropout` | nothing for each lost sample (no new value: its stamp stays, so no ROS message goes out), or NaN with `nan=1` | `probability` (per sample, default 1), `nan` |
| `stuck` | the last value it delivered before the window, again at every sample | `component` |
| `bias` | its value plus `value` (a step at `at`) | `value`, `component` |
| `drift` | its value plus `rate · (t − at)` | `rate`, `component` |
| `noise` | its value plus Gaussian noise of standard deviation `sigma` (a burst, with `for`) | `sigma`, `component` |
| `disconnect` | nothing (or NaN) from `at` for good | `nan`; no `for` |

`at` (s, default 0) and `for` (s, default for good) set the window, compared with each sample's time
(the end of the step that takes it), in whole nanoseconds. `value`, `rate` and `sigma` are in the
quantity's own unit (rad/s for an IMU's `angular_rate`, m for a lidar's `ranges`); they apply to every
component, or to one with `component` (0 = x). A fault names one quantity, or all of the sensor's
(`*`, or no quantity): `dropout`, `stuck` and `disconnect` can, `bias`, `drift` and `noise` need one.
Images and point buffers take `dropout`, `stuck` and `disconnect` only; for them `nan` means nothing.
A sensor's quantities in one fault go together: with `imu=dropout` the whole IMU message is lost,
while `imu:angular_rate=dropout` keeps the message, which then carries the last angular rate (the ROS
bridge sends an IMU message when `specific_force` is new).

Set them with `--sensor-fault NAME[:QUANTITY]=MODE[,key=value...]` (repeatable), the run file's
`sensor_faults:`, a profile's `sensor_faults:` (which replaces the command line's when given), or
Python's `Simulation(..., sensor_faults=[...])`:

```bash
lagrange_server --sensor-fault imu:angular_rate=bias,value=0.02,at=5,component=2 \
                --sensor-fault front_lidar=dropout,at=10,for=2,probability=0.3
```

```yaml
sensor_faults:      # profile or run file
  - {sensor: imu, quantity: angular_rate, mode: drift, rate: 0.001, at: 30}
  - {sensor: gps, mode: disconnect, at: 60}
```

Each faulted sensor also writes `sensor/<n>/fault/active` (Bool: a fault's window is open) and
`sensor/<n>/fault/dropped` (Int: samples lost so far), which `--ros-topics` can publish, e.g.
`{signal: sensor/imu/fault/active, topic: /imu/fault, type: std_msgs/Bool}`. A fault that names
no sensor, or a quantity the sensor does not have, stops the start and lists what it has.

How: `sensors::withFaults` (`modules/sensors/src/Faults.cpp`) wraps each sensor module the server
adds, the built-in ones and plugins' own-domain modules, and applies the faults to its outputs after
its step and before they commit: a dropped sample is held (`SignalBus::hold`, so readers and its
stamp stay as they were), the others are written into the pending value. The wrapper keeps what the
sensor wrote last without the faults and puts it back before each step, so a module that does not
write every step never gets a fault twice. With no faults, nothing is wrapped and nothing costs; with
faults, every sensor module goes through one more call per step, and each faulted number is copied
twice per step. The wrapped module keeps its name, seed and order, so its own noise is unchanged.

Random draws (which samples a `probability` loses, a `noise` burst) come from
`random::Stream(seed, step, 0xFA170000 + 2f [+ 1])` of the sensor module's own seed and step index
for fault `f`: a run and a restored checkpoint repeat them, and another run seed draws others. The
wrapper's state (the stuck values, the clean values, the counts) is saved with the module's.

The ground-truth substitute (`sensors: {enabled: false, substitute: ground_truth}`) is faulted too.
A fault is in the hardware between the sensor and the robot (a cable, a supply, a mount), not in the
sensor's model, which is what the substitute replaces: a run that tests a planner at ground truth
speed against a lost GPS still loses it. To have clean ground truth, give no faults; the bodies'
signals are never faulted. A replay (`--replay`) shows what was recorded, faults included, and adds
none. A plugin that substitutes the sensors domain itself is not wrapped.

Faults are set when the run starts; `at` and `for` schedule them within it. Applying a profile with
other `sensor_faults` changes them, restarting the run at t = 0 as any profile does. Tests:
`modules/sensors/test/test_faults.cpp` (every mode on a lidar, an IMU and a buffer, determinism,
saved states, the substitute), `server/test/test_sensor_faults.cpp`, `Plugins.APluginSensorCanBeFaulted`.

## 9. Built-in custom sensor kinds

A custom sensor (`SensorType::Custom`: a URDF/SDFormat `<sensor type="KIND">` Lagrange has no
type for, or USD `CustomSensorAPI_1` with `sensor:custom:kind`) whose kind is listed here is
simulated by `lagrange_sensors` itself, with no plugin (`sensors::isBuiltinKind`,
`modules/sensors/src/kinds/`). Its numbers are `World::Sensors::settings` (SI units) and its text
`World::Sensors::text`, by the parameter's path (`<range><max>` is `range/max`). Every kind runs at
the sensor's update rate (above 0 Hz: Gazebo's 0, as fast as possible, is refused; a period of
whole nanoseconds is kept, 400 Hz runs every 2.5 ms, and any other goes to whole milliseconds,
or whole 10 µs below 1 ms, as the cameras' do: 30 Hz runs every 33 ms, so that its steps line up
with the other modules'; the built-in lidar, IMU, radar, GNSS, magnetometer and barometer
likewise), writes `sensor/<name>/...`, draws its noise from the run's seed (the
CPU reference stays bit-reproducible), and under the sensors domain's `ground_truth` substitute
reads exactly. On a robot added to a run (spawned, `--add-robot`), whose bodies, joints,
sensors and components are renamed `<robot>_<name>`, the names a kind's text gives (joints,
bodies, batteries, tethers, cameras, signals) are its own robot's: `left_wheel_joint` on a copy
added as `copy` is `copy_left_wheel_joint`; a kind that takes the stage's only battery or
tether takes its own robot's when the run has several. No schema changes: these are additions
under RFC 0007. The sample plugins' kinds
(`tof`, `ultrasonic`, `thermistor`, `ir_thermometer`) stay plugins; a plugin that implements one
of the kinds below as well would write the same signals, and the run refuses to start.

Defaults. A default taken from a part's data sheet cites the document (part number, document
number and revision, where it says so); the checks of 2026-11-12 against the documents are listed
in `docs/plan/sensor-defaults-verification-2026-11-12.md`. A default marked *illustrative* has no
primary source because it is a property of the installation or the scene, not of a part (a
shunt's resistance, a tag's size, a mast's height, a threshold the integrator picks), or because
the maker publishes no figure for it (the noise of an optical flow chip, the prisms' speeds of a
Livox lidar): give your part's or your log's figure.

### ROS topics of the built-in kinds

Every custom sensor's numbers are on ROS 2 from the first run, with no mapping file, as the
cameras, lidars and IMU are (`bridges/ros2/src/KindTopics.cpp`, `ros2::kindTopics`): each on
`/<sensor>/<quantity>`, stamped when it was sampled, through the sensor's `sensor:latency` and
`jitter` like any sensor topic, faulted as the signals are (sensor faults), and built only while
someone subscribes. A kind whose device has a standard ROS message publishes that message; every
other number the sensor writes goes out by its signal type and unit:

| Kind | Topic | Message | Notes |
|---|---|---|---|
| `rangefinder` | `/<n>/range` | `sensor_msgs/Range` | INFRARED (light), `field_of_view` 0 (one ray), `min_range`/`max_range` the kind's `range/min`, `range/max`; +inf: nothing in range (REP 117) |
| `cliff` | `/<n>/range` | `sensor_msgs/Range` | INFRARED, `field_of_view` 2 × `beam_angle` with `rays` > 1; `/<n>/cliff` Bool |
| `proximity` | `/<n>/range` | `sensor_msgs/Range` | `min_distance`, `field_of_view` the `fov`; `/<n>/distances` Float64MultiArray rows × columns |
| `sonar`, one beam | `/<n>/range` | `sensor_msgs/Range` | ULTRASOUND, `field_of_view` the `beam_width` |
| `sonar`, a fan | `/<n>/scan` | `sensor_msgs/LaserScan` | `ranges` and `intensities` (dB), beam 0 the left one: `angle_min` +fov/2, `angle_increment` negative; `/<n>/image` Float64MultiArray beams × bins |
| `safety_scanner` | `/<n>/scan` | `sensor_msgs/LaserScan` | from −fov/2, `scan_time` 1/rate; `/<n>/protective`, `/<n>/warning` Bool, `/<n>/min_distance` Float64 |
| `wheel_odometry` | `/<n>/odom` | `nav_msgs/Odometry` | `odom` → the sensor's body, twist in the body frame; covariance diagonals (x y z roll pitch yaw) from the text parameters `pose_covariance_diagonal` and `twist_covariance_diagonal` (diff_drive_controller's names), else 1e-3 m², 1e-3 m², 1e6, 1e6, 1e6, 1e-2 rad² (illustrative; 1e6: not measured) |
| `battery_monitor` | `/<n>/battery_state` | `sensor_msgs/BatteryState` | ROS's signs and units: current negative discharging, `charge` and `capacity` in Ah (`capacity` the parameter, else charge / SOC), `percentage` 0 to 1, °C; `present` the `valid` signal; `location` the battery's name |
| `joint_torque` | `/<n>/joint_state` | `sensor_msgs/JointState` | the joint's name and its `effort` (N·m, or N on a slide), as Franka's and KUKA's drivers report link-side torque; `/<n>/saturated` Bool |
| `force_torque` | `/<n>/wrench` | `geometry_msgs/WrenchStamped` | in the signal's frame; `force` and `torque` are in it, not on topics of their own |
| `optical_flow` | `/<n>/flow` | `geometry_msgs/Vector3Stamped` | x, y the integrated flow (rad) about `<n>_optical`'s axes, PX4's sign (`SensorOpticalFlow.pixel_flow`), z 0; no standard ROS message carries optical flow (px4_msgs and mavros_msgs' `OpticalFlowRad` are not Lyrical dependencies of Lagrange) |
| `detections` | `/<n>/detections`, `/<n>/detections_3d` | `vision_msgs/Detection2DArray`, `Detection3DArray` | in the camera's `<camera>_optical` frame; `id` the object id, one hypothesis: `class_id` the class id as text, score 1; 2D box centre between the first and last pixel, size across them; 3D box KITTI's: turned by `rotation_y` about the optical y, `size` x along the object, y its height, z its width (upright objects under a level camera) |
| `event_camera`, `stereo_camera` and any image | `/<n>/<q>/image_raw`, `/<n>/events` | `sensor_msgs/Image`, `PointCloud2` | "Images of kinds" above |

Every other number, any kind (plugins' kinds too):

| Signal | Message |
|---|---|
| Bool | `std_msgs/Bool` |
| Int | `std_msgs/Int64` |
| Scalar in K | `sensor_msgs/Temperature` (°C) |
| Scalar in Pa | `sensor_msgs/FluidPressure` |
| Scalar `relative_humidity` | `sensor_msgs/RelativeHumidity` (0 to 1) |
| Scalar `illuminance` | `sensor_msgs/Illuminance` |
| any other Scalar | `std_msgs/Float64` (SI, the signal's unit) |
| ScalarArray | `std_msgs/Float64MultiArray`; rows of fields keep two dimensions (`rows`, `columns`): `adsb_receiver` `aircraft` (10), `v2x` `bsm` (10) and `spat` (7), `proximity` `distances`, `sonar` `image` |
| Vec3 | `geometry_msgs/Vector3Stamped`, in the signal's frame (`<n>`, `<n>_optical`, the body, or `world` with the floating origin's offset for a position) |
| Pose | `geometry_msgs/PoseStamped` |
| Wrench in m/s | `geometry_msgs/TwistStamped`; else `WrenchStamped` |

So a `humidity_sensor` publishes `/<n>/relative_humidity` and `/<n>/temperature`, an `airspeed`
`/<n>/differential_pressure` (FluidPressure), `/<n>/airspeed`, `/<n>/indicated_airspeed`,
`/<n>/temperature`, an `anemometer` `/<n>/wind_velocity` (Vector3Stamped in `<n>`),
`/<n>/wind_speed`, `/<n>/wind_direction`, a `uwb` `/<n>/ranges` (no standard ROS message
carries UWB ranges), a sensor with faults `/<n>/fault/active` and `/<n>/fault/dropped`, and every
kind its `/<n>/valid`. Not published: `sensor/<n>/delayed/...` (the topics are delayed
themselves), an image's `intrinsics` and `position` (its `camera_info`).

**A map wins.** A `--ros-topics` publish entry replaces a default topic when it uses the same
topic name, or when it publishes a signal the default topic carries (the whole default goes:
publishing `sensor/alt/range` as Float64 on `/alt` leaves no `/alt/range`). The sensor's other
defaults stay. `kind_topics: false` at the top of the map file turns every default off. With
`--no-ros` there is no bridge and nothing is published. Tests: `test_kind_topics` (bridges/ros2:
each message family from fake signals, laziness, a map's precedence; lagrange_server: the safety
AMR's topics end to end).

### optical_flow

A downward optical flow sensor such as the PixArt PMW3901 or PX4Flow, computed from geometry
rather than by matching images: the sensor's own velocity and angular velocity (its body's
`twist`, with the lever arm to the mount) and the depth to what it sees, from a 3 × 3 grid of
rays over its field of view cast against every collider except its own body's (CPU
`ShapeCaster`, deterministic; no GPU). Each ray that hits within range contributes the
Longuet-Higgins and Prazdny (1980) motion field at its image point; the flow is their mean. On
the optical axis that is `ω + (ẑ × v)/Z`; over a plane parallel to the image at height h, a
sensor moving at v reads exactly v/h.

The sensor looks along its x like a camera, so a downward sensor is mounted pitched by +90°
(`rpy="0 1.5708 0"`). Its flow is about the optical frame's x and y (z along the view, x right,
y down in the image), in the sign of PX4's `SensorOpticalFlow.pixel_flow`: a right-handed
rotation of the sensor about an axis reads positive about it, so a pure rotation reads as the
gyro's delta angle and `flow − delta_angle` is the translational part (PX4-Autopilot
`msg/SensorOpticalFlow.msg`; ekf2 negates both, `EKF2.cpp`). With a wide field of view a pure
rotation reads slightly more than the gyro (the `1 + y²` term of the motion field: about 4 % at
42°), as a real perspective sensor does.

```xml
<gazebo reference="base_link">
  <sensor name="flow" type="optical_flow">
    <pose>0 0 -0.05 0 1.5708 0</pose>
    <update_rate>100</update_rate>
    <optical_flow><fov>0.733</fov><min_range>0.08</min_range><max_range>30</max_range>
      <max_flow_rate>7.4</max_flow_rate><noise>0.05</noise></optical_flow>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `fov` | rad | 0.733 (42°, PMW3901MB-TXQT datasheet v1.00, table 3) | field of view the rays span |
| `min_range` | m | 0.08 (PMW3901 datasheet) | nearer ground: quality 0 |
| `max_range` | m | 30 (illustrative: PixArt gives no limit, "80 mm to infinity") | farther ground: quality 0 |
| `max_flow_rate` | rad/s | 7.4 (PMW3901 datasheet) | faster flow: quality 0, it loses the texture |
| `noise` | rad/s | 0.05 (illustrative: the datasheet gives no noise figure) | white noise on each axis's rate, 1σ |

The period is rounded to whole milliseconds. Signals: `sensor/<n>/flow` (ScalarArray of 2, rad,
integrated over the sample: rate × period), `quality` (0–255 as PX4: the share of rays in range
times the contrast through fog `exp(−σ d)` with σ the optional `env/extinction/visible`, 0 when
nothing is in range or the flow exceeds `max_flow_rate`; flow then reads 0), `distance` (m, the
optical axis's ray, +inf out of range) and `integration_time` (s). Under `ground_truth` there is
no noise. Not modelled: the surface's texture and the light level (owed: a low-texture or dark
floor does not lower the quality yet). Tests: `test_kinds_optical_flow` (v/h over a plane,
rotation = gyro, noise σ, range and rate limits).

### event_camera

A dynamic vision sensor (iniVation DAVIS346, Prophesee) from successive rendered frames, as ESIM
(Rebecq, Gehrig and Scaramuzza, "ESIM: an Open Event Camera Simulator", CoRL 2018) and v2e (Hu,
Liu and Delbruck, "v2e: From Video Frames to Realistic DVS Events", CVPR Workshops 2021) make
events. Each step renders the camera scene (as the camera does, on the GPU or the software
device) and turns each pixel into log intensity (linear luminance in digital numbers, `ln` above
20 DN and linear below, v2e's lin-log). Between two frames a pixel's log intensity is linear in
time; each time it crosses the pixel's memorised level by its threshold an event fires at the
crossing time and the level moves by the threshold. A crossing within the refractory period of
the pixel's last event is held to the next frame. Thresholds differ per pixel by
`threshold_sigma` (drawn once per run from its seed), and background activity fires at
`noise_rate` per pixel with random polarity. The render rate is the sensor's update rate: events
between frames are interpolated, so a faster rate follows faster motion.

```xml
<sensor name="dvs" type="event_camera">
  <update_rate>200</update_rate>
  <event_camera><width>346</width><height>260</height><horizontal_fov>1.2357</horizontal_fov>
    <threshold_on>0.2</threshold_on><threshold_off>0.2</threshold_off>
    <threshold_sigma>0.03</threshold_sigma><refractory_period>0.0005</refractory_period>
    <noise_rate>0.1</noise_rate><max_events>20000</max_events></event_camera>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `width`, `height` | px | 346, 260 (DAVIS346) | the pixel array (also `<camera><image><width>`) |
| `horizontal_fov` | rad | 1.2357 (70.8°: the DAVIS346 sheet's, 4.5 mm lens) | (also `<camera><horizontal_fov>`) |
| `near_clip` | m | 0.05 | |
| `threshold_on`, `threshold_off` | ln | 0.2 (v2e's default; its paper calls 0.3 typical) | contrast thresholds |
| `threshold_sigma` | ln | 0.03 (v2e) | per-pixel mismatch, 1σ |
| `refractory_period` | s | 5e-4 (v2e's default; iniVation publishes none) | |
| `noise_rate` | Hz | 0.1 (the v2e paper's typical leak rate; v2e's own default is 0.01) | background activity per pixel |
| `max_events` | | 20000 | events per step kept in `events` |

Signals: `sensor/<n>/events` (ScalarArray of 4 × `max_events`: x, y, t in s of sim time,
polarity ±1, sorted by time, NaN after the last), `count` (Int, events in this step's packet),
`dropped` (Int, events over `max_events`), and `image` (Image rgba8: this step's events per
pixel, grey 128 plus 64 per ON and minus 64 per OFF) with `image/intrinsics` and
`image/position`. Rendering makes it a GPU path, not
deterministic (invariant 7); under `ground_truth` it renders nothing and fires nothing. Without a
Vulkan device the run refuses to start, as with a camera. Tests: `test_kinds_event_camera` (a
ramp fires one event per threshold at the crossing times, the refractory hold, threshold and
noise statistics, lin-log; rendered: a still scene is silent, a moving one fires).

### stereo_camera

An active stereo depth camera like the Intel RealSense D435: left and right infrared imagers a
baseline apart and the left imager's depth. It renders the camera scene from both imagers (the
right one `baseline` to the right in the image) and gives the left's rendered depth the error a
stereo matcher makes, without matching: Gaussian noise of `σ_z = z² σ_d / (f b)` with σ_d the
sub-pixel disparity error, f the focal length in pixels and b the baseline (Grunnet-Jepsen et al.,
"Tuning depth cameras for best performance", Intel RealSense white paper), quantised to
`depth_unit`. Invalid pixels follow from the geometry: a point the right imager does not see
(outside its image, the left band whose width is the disparity f b / z, or behind something
nearer in its depth, an occlusion shadow) is NaN; nearer than min-Z = f b / `max_disparity` is
−inf; beyond `max_range` or nothing is +inf (ROS REP 118).

```xml
<sensor name="d435" type="stereo_camera">
  <update_rate>30</update_rate>
  <stereo_camera><width>848</width><height>480</height><horizontal_fov>1.518</horizontal_fov>
    <baseline>0.05</baseline><subpixel>0.08</subpixel><max_disparity>126</max_disparity>
    <max_range>10</max_range><depth_unit>0.001</depth_unit></stereo_camera>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `width`, `height` | px | 848, 480 (D435 depth) | |
| `horizontal_fov` | rad | 1.518 (87°, D435 depth field of view, D400 datasheet 337029 table 3-49) | |
| `near_clip` | m | 0.05 | |
| `baseline` | m | 0.050 (D435) | |
| `subpixel` | px | 0.08 (Intel white paper) | σ_d |
| `max_disparity` | px | 126 (D400 search range, the white paper's min-Z formula) | sets min-Z |
| `max_range` | m | 10 (illustrative: the D400 datasheet gives "0.2 m to over 3 m (varies with lighting conditions)", the white paper up to about 10 m on a white wall) | |
| `depth_unit` | m | 0.001 (D400) | output quantum; 0 for none |

Signals: `sensor/<n>/depth` (Image r32f, m along the optical axis, NaN/±inf as above), `left`
and `right` (Image rgba8, the rendered luminance as grey: the IR imagers without their emitter's
dot pattern), each with its `intrinsics` and `position` (the right imager at +`baseline`; see
"Images of kinds"). Rendering makes it a GPU path, not deterministic; under `ground_truth` it
publishes empty images that keep their size. Tests: `test_kinds_stereo_camera` (σ_z of a noisy wall at 1,
2 and 4 m, the left band, REP 118 limits, occlusion, the 1 mm quantum; rendered: a wall ahead).

#### Getting these signals out

The scalar and array signals are on ROS by default ("ROS topics of the built-in kinds" above:
`/flow/flow` Vector3Stamped, `/flow/quality`, `/flow/distance`, `/dvs/count`, ...); a
`--ros-topics` map can put them elsewhere or on other messages:

```yaml
publish:
  - {signal: sensor/flow/flow, topic: /flow/flow, type: std_msgs/msg/Float64MultiArray}
```

and Python reads them with `sim.read("sensor/flow/flow")`. Images and events need no mapping:
see the next section.

### Images of kinds

Any Image signal a custom sensor writes (a built-in kind's or a plugin's) reaches every user by
its name, with nothing written per kind; a new kind that writes `sensor/<n>/<q>` as an Image gets
all of this for free.

- **Calibration on the bus.** A kind whose image is a pinhole writes beside it
  `sensor/<n>/<q>/intrinsics` (ScalarArray of 4, px: fx, fy, cx, cy, OpenCV convention, origin at
  the first pixel's centre) and `sensor/<n>/<q>/position` (Vec3, m, frame `optical`: where that
  image's camera centre is in the sensor's optical frame). `stereo_camera` writes them for
  `left`, `right` (at +`baseline` along x) and `depth`; `event_camera` for `image`.
- **ROS 2** (`Bridge::attach`, any `SensorType::Custom`): each Image signal as
  `sensor_msgs/Image` on `/<n>/<q>/image_raw`, encoding from the buffer (rgba8 → `rgba8`, rgb8 →
  `rgb8`, r8 → `mono8`, r16u → `mono16`, r32f → `32FC1`, r32u → `32SC1`; `ros2::rosImageEncoding`),
  stamped when it was sampled, in `<n>_optical` for an image in the optical frame (the bridge then
  publishes the static `<n>` → `<n>_optical` transform) and `<n>` otherwise; with the calibration,
  `sensor_msgs/CameraInfo` on `/<n>/<q>/camera_info` (no distortion, K from the intrinsics, P with
  `Tx = −fx·x` and `Ty = −fy·y` from the position: a stereo pair's right image has `Tx = −fx·b`,
  as REP 104 and CameraInfo.msg give it for the second camera of a horizontal pair; both images
  in the left's optical frame). Packed events (`sensor/<n>/events`, 4 numbers per event, with
  `count`) go as `sensor_msgs/PointCloud2` on `/<n>/events`: `x`, `y` (uint16, px), `polarity`
  (int8, ±1), `t` (float64, s of sim time), one point per event, in `<n>_optical`. No new message
  package: `event_camera_msgs` is not a Lyrical rosdep key, and Image, CameraInfo and PointCloud2
  are in `sensor_msgs`. Sensor timing (`<latency>`, `<jitter>`) delays them as it does a camera's
  topics; radio links (`--link-topics`) gate them by topic. For the D435 example: `/d435/left/`,
  `/d435/right/` and `/d435/depth/image_raw` with their `camera_info`, ready for
  `stereo_image_proc` or `depth_image_proc`. Test: `test_bridge`
  (`KindImageBridgeTest.PublishesACustomSensorsImagesWithTheirCalibration`).
- **Python**: `sim.image("sensor/d435/depth")` returns the latest frame as a numpy array: rgba8
  `(height, width, 4)` uint8, rgb8 `(h, w, 3)`, r8 `(h, w)` uint8, r16u uint16, r32f `(h, w)`
  float32 (m for a depth, K for a temperature), r32u uint32; None before the first frame (the
  `ground_truth` substitute renders nothing). It reads a camera's `image`, `depth` and
  `instance` too. `sim.read` of an image raises TypeError and `image` of a number signal
  likewise; the events read as numbers, `numpy.asarray(sim.read("sensor/dvs/events"))
  .reshape(-1, 4)` (`Server::readImage`; test `python/test/test_images.py`).
- **Viewer**: the existing `SubscribeSensor` command and `SensorFrame` message carry an image by
  signal name as well as a sensor by prim path: a viewer that subscribes to `sensor/<n>/<q>` gets
  that image's frames as `SensorFrame{sensor: "sensor/<n>/<q>", image}` (the delayed image when
  the sensor has sensor timing), with its own encoding. No protocol change: the field was a
  string already, and an older server ignores a name it does not know. The viewer keeps image
  signals in its catalog (they have no samples) and lists custom sensors with images in the
  Sensors workspace; the Camera panel's modes are then the sensor's images (`left`, `right`,
  `depth`; `image` for an event camera), and for a camera `depth` and `labels` now show its depth
  and instance images. r32f draws as grey from its nearest (white) to its farthest, r32u as a
  colour per id. Tests: `test_viewer_client` (`AnImageSignalDoesNotEndTheCatalog`,
  `AnImageSignalIsWatchedByNameAndDrawnAsRgba`, `ImageToRgba.*`), `test_server`
  (`ArmServerTest.AnImageSignalIsSentByItsName`).

The power kinds below (`battery_monitor`, `current_sensor`, `voltage_sensor`, `esc_telemetry`,
`tension_sensor`) share one measuring chain (`kinds/Converter.hpp`): the truth times
(1 + `gain_error`), plus `offset`, plus white noise of `noise` (1 σ) per conversion, each
conversion rounded to the step and clamped to the range, `samples` conversions averaged and the
mean rounded again. They read other domains' signals as optional inputs: while the domain that
writes one is off (or the stage has nothing that writes it), that reading is NaN and
`sensor/<name>/valid` is 0 when the main one is missing. Under `ground_truth` every reading is
the input itself, unrounded.

### battery_monitor

A smart battery's fuel gauge (a TI BQ40Z50 class part) or a power module, on one battery of the
electrical domain. Its default steps are the Smart Battery Data Specification Rev 1.1 (SBS-IF,
1998) §5.1 registers: Voltage in mV, Current in mA, Temperature in 0.1 K,
RelativeStateOfCharge in %, RemainingCapacity in mAh.

```xml
<sensor name="fuel_gauge" type="battery_monitor">
  <update_rate>10</update_rate>
  <battery_monitor>
    <battery>battery</battery>
    <voltage><noise>0.002</noise></voltage>
    <current><bits>16</bits><full_scale>60</full_scale></current>
  </battery_monitor>
</sensor>
```

| Parameter | Default | Meaning |
|---|---|---|
| `battery` | the stage's only `BatteryAPI_1` prim | which battery (its prim name); the run refuses to start when there are none or several and none is named |
| `capacity` | `elec:battery:capacity` | the pack's rated charge in **A·h** (5 for a 5000 mAh pack), only when the stage has no such battery. The one parameter of §9 not in SI units: it is kept in A·h as written on packs and in ROS's `BatteryState` (1.x is frozen, RFC 0007); the signal `remaining_capacity` is in C (1 A·h = 3600 C) |
| `<q>/resolution` | SBS steps: 1 mV, 1 mA, 0.1 K, 0.01, 3.6 C | the step of quantity `<q>` (`voltage`, `current`, `temperature`, `soc`, `remaining_capacity`) |
| `<q>/bits`, `<q>/full_scale` | none | an ADC instead: step = full_scale / 2^bits over [0, full_scale] (current: ±full_scale, step full_scale / 2^(bits−1)) |
| `<q>/noise`, `<q>/offset`, `<q>/gain_error`, `<q>/samples` | 0, 0, 0, 1 | the chain above, in `<q>`'s unit |

| Signal | Unit | |
|---|---|---|
| `sensor/<n>/voltage` | V | from `battery/<b>/terminal_voltage` |
| `sensor/<n>/current` | A | `battery/<b>/current`, positive discharging (Lagrange's sign, not SBS's) |
| `sensor/<n>/soc` | 1 | `battery/<b>/soc`, clamped to [0, 1] |
| `sensor/<n>/temperature` | K | `thermal/<b>/temperature`; NaN while the battery has no thermal node |
| `sensor/<n>/remaining_capacity` | C | SOC × capacity (1 mAh = 3.6 C) |
| `sensor/<n>/valid` | 1 | 0 while the electrical domain is off |

Test: `test_kinds_battery_monitor` (register steps, step = full scale / 2^bits, noise σ and
reproducibility, NaN with electrical off, battery naming, ground truth).

### current_sensor and voltage_sensor

One current (A) or voltage (V) signal of the run, named by `signal`: a bus voltage
(`bus/<b>/voltage`), a motor's `motor/<m>/bus_current`, a branch's `branch/<b>/current`, a
battery's `battery/<b>/terminal_voltage`, or a plugin's. A signal of the other unit is refused at
wiring (invariant 2).

```xml
<sensor name="motor1_current" type="current_sensor">
  <current_sensor>
    <signal>motor/rotor_1_motor/bus_current</signal>
    <model>ina226</model>
    <shunt_resistance>0.0005</shunt_resistance>
    <averages>16</averages>
    <conversion_time>0.000588</conversion_time>
  </current_sensor>
</sensor>
```

Models:

- `ina226` (default, both kinds): TI INA226, datasheet SBOS547C §5.5 and Tables 7-3 to 7-5. Shunt
  voltage step 2.5 µV over ±81.92 mV; bus voltage step 1.25 mV over 0 to 40.96 V (inputs only up
  to 36 V); `conversion_time` one of 140, 204, 332, 588, 1100 (default), 2116, 4156 or 8244 µs;
  `averages` one of 1 (default), 4, 16, 64, 128, 256, 512, 1024; `channels` 2 (default: shunt and
  bus, continuous, the power-on mode) or 1. The INA226 integrates, so it reads the signal's mean
  over its window (`Hold::WindowAverage`), and a new reading comes every conversion_time ×
  channels × averages: that is the module's step, whatever `update_rate` says, on the kinds'
  whole milliseconds (2.2 ms by default runs every 2 ms: the exact 2.2 ms beside a 1 Hz sensor
  would repeat only after 11 s, and the scheduler would refuse the run). For
  currents `shunt_resistance` (Ω, default 2 mΩ, illustrative); `noise` and `offset` are in volts
  at the ADC's input (shunt or bus). The datasheet bounds them: shunt offset ±2.5 µV typical,
  ±10 µV maximum; bus offset ±1.25 mV typical, ±7.5 mV maximum; gain error 0.02% typical, 0.1%
  maximum. The defaults are 0, so give the part's figures. Each step averages `averages` noisy
  conversions, so noise falls as 1/√N.
- `hall` (current only): an Allegro ACS712 read by a microcontroller's ADC. Its output is
  `zero_output` (default `supply` / 2, `supply` 5 V) + `sensitivity` × I. The defaults are the
  ACS712ELCTR-05B (ACS712-DS Rev. 22, 2024): 0.185 V/A, and noise 21 mV peak to peak taken as
  6 σ (3.5 mV rms; the 20 A part is 0.100 V/A and 11 mV, the 30 A part 0.066 V/A and 7 mV). Then
  `adc/bits` (12) over 0 to `adc/reference` (default `supply`), and `averages`.
- `divider` (voltage only): a divider of `ratio` (output over input, default 1/11, illustrative)
  into `adc/bits` (12) over 0 to `adc/reference` (3.3 V), as a flight controller's power module
  reads its pack.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `signal` (text) | | none: required | the current (A) or voltage (V) signal it measures |
| `model` (text) | | `ina226` | `ina226`, `hall` (current only) or `divider` (voltage only) |
| `shunt_resistance` | Ω | 0.002 (illustrative) | `ina226`, current only |
| `conversion_time` | s | 0.0011 | `ina226`: the nearest listed value is taken |
| `averages` | | 1 | conversions averaged (`ina226`: the nearest listed value) |
| `channels` | | 2 | `ina226`: 2 (shunt and bus) or 1 |
| `noise`, `offset` | V | 0, 0 | at the ADC's input, 1σ and fixed (`hall`: noise 0.0035 V) |
| `gain_error` | | 0 | reads (1 + gain_error) × the truth |
| `supply` | V | 5 | `hall`: the ACS712's supply |
| `sensitivity` | V/A | 0.185 | `hall` (ACS712ELCTR-05B) |
| `zero_output` | V | `supply` / 2 | `hall`: the output at 0 A |
| `ratio` | | 1/11 (illustrative) | `divider`: output over input |
| `adc/bits` | | 12 | `hall`, `divider` |
| `adc/reference` | V | `supply` (`hall`), 3.3 (`divider`) | the ADC's range is 0 to it |

| Signal | Unit | |
|---|---|---|
| `sensor/<n>/current` or `sensor/<n>/voltage` | A or V | the reading |
| `sensor/<n>/valid` | 1 | 0 while nothing writes `signal` |

Tests: `test_kinds_current_sensor` (2.5 µV steps and the ±81.92 mV range, σ ratio 4 from 1 to
16 averages, the window mean, an ACS712 through 12 bits with its default noise, NaN and unit
checks, ground truth); `test_kinds_voltage_sensor` (1.25 mV steps, 40.96 V full scale, the
divider and its range).

### esc_telemetry

What a drone's ESC reports of the motor on `joint` (BLHeli_32, AM32). The motor is the
`ElectricalMotorAPI_1` whose `elec:joint` is that joint: its pole pairs (`elec:polePairs`), its
gear (`elec:gearRatio`, motor turns per joint turn), its bus (found through its ports, as the
electrical network finds it) and its thermal node (`thermal/<motor>/temperature`). eRPM =
mechanical RPM × pole pairs. `speed` is decoded back to the motor shaft, so RPM = speed × 60/2π.

```xml
<sensor name="esc_1" type="esc_telemetry">
  <update_rate>50</update_rate>
  <esc_telemetry><joint>rotor_1_joint</joint><protocol>kiss</protocol><escs>4</escs></esc_telemetry>
</sensor>
```

- `kiss` (default): KISS serial telemetry, a 10-byte frame at 115200 baud per request, the flight
  controller asking each ESC in turn on a shared wire (Betaflight `src/main/sensors/esc_sensor.c`).
  Temperature in °C, voltage in 0.01 V, current in 0.01 A, consumption in mAh, speed in 100 eRPM,
  all unsigned. The rate is `update_rate` but at most `baud` / (100 bits × `escs`) (default
  115200 baud and 4 ESCs: 288 Hz). `consumption` counts the motor's `bus_current` from the start.
- `dshot`: bidirectional DShot. Each reply carries the eRPM as its period in whole µs, a 9-bit
  mantissa shifted by a 3-bit exponent (period = m << e). Below 60·10⁶ / (511 << 7) ≈ 917 eRPM it
  reads 0. Extended DShot Telemetry v2.1.1 (github.com/bird-sanctuary/extended-dshot-telemetry)
  sends temperature in 1 °C (0 to 255), voltage in 0.25 V (0 to 63.75) and current in 1 A (0 to
  255) at `extended_rate` (default 1 Hz, the spec's example schedule). It has no consumption
  (NaN).

Other parameters: `pole_pairs` and `gear_ratio` (needed when no motor prim drives the joint),
`bus`, `temperature_node`, and `current/…`, `voltage/…`, `temperature/…` noise, offset and
gain_error (`temperature/` in °C). Telemetry periods are rounded up to whole milliseconds, as every kind's.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `joint` (text) | | none: required | the motor's joint |
| `protocol` (text) | | `kiss` | `kiss` or `dshot` |
| `escs` | | 4 | `kiss`: ESCs asked in turn on the wire |
| `baud` | bit/s | 115200 | `kiss`: the wire's rate |
| `extended_rate` | Hz | 1 | `dshot`: extended frames a second |
| `pole_pairs`, `gear_ratio` | | the motor's `elec:polePairs`, `elec:gearRatio` (else 1) | needed when no motor prim drives the joint |
| `bus`, `temperature_node` (text) | | the motor's bus and thermal node | |
| `current/resolution`, `voltage/resolution` | A, V | 0.01, 0.01 (`dshot`: 1 A, 0.25 V) | the frame's steps |
| `temperature/resolution` | °C | 1 | |
| `<q>/bits`, `<q>/full_scale` | | none | an ADC instead, as the battery monitor's |
| `<q>/noise`, `<q>/offset`, `<q>/gain_error`, `<q>/samples` | | 0, 0, 0, 1 | the chain above, in `<q>`'s unit (`temperature/` in °C) |

| Signal | Unit | |
|---|---|---|
| `sensor/<n>/speed` | rad/s | motor shaft, from the protocol's eRPM |
| `sensor/<n>/current` | A | `motor/<m>/bus_current`, its mean over the frame (negative reads 0) |
| `sensor/<n>/voltage` | V | `bus/<bus>/voltage` |
| `sensor/<n>/temperature` | K | `thermal/<motor>/temperature` (the motor's node stands in for the ESC's), NaN without one |
| `sensor/<n>/consumption` | C | kiss only (1 mAh = 3.6 C) |
| `sensor/<n>/valid` | 1 | 0 while physics writes no joint velocity |

Test: `test_kinds_esc_telemetry` (KISS steps, whole-mAh consumption, the wire's rate limit,
DShot's 898 µs = 449 << 1 and the extended frames once a second, the stop threshold, missing
domains, ground truth).

### tension_sensor

A strain-gauge load cell where a tether meets its body (`end` `attach`, default:
`tether/<t>/tension_attach`) or its station (`anchor`: `tension_anchor`), read by a bridge ADC.
The cell gives `sensitivity` × `excitation` × F / `capacity` volts. Its error is a bow of
`nonlinearity` × capacity × 4x(1 − x) (x = |F| / capacity: largest at half load, none at 0 and
full load), and `hysteresis` × capacity between loading and unloading at half load. The loading
branch reads low and the unloading branch high, by half each. The ADC defaults to an Avia HX711
channel A: 24-bit two's complement over ±0.5 × `excitation` / `adc/gain` (gain 128: ±19.5 mV at
5 V), input noise 50 nV rms (gain 128, 10 SPS), all from its datasheet. Run it at 10 or 80 Hz
`update_rate`, the HX711's two rates.

```xml
<sensor name="tether_load" type="tension_sensor">
  <update_rate>80</update_rate>
  <tension_sensor>
    <tether>tether</tether><end>attach</end>
    <capacity>200</capacity><sensitivity>0.002</sensitivity><excitation>5</excitation>
    <nonlinearity>0.0002</nonlinearity><hysteresis>0.0002</hysteresis>
  </tension_sensor>
</sensor>
```

| Parameter | Default | Meaning |
|---|---|---|
| `tether` | the stage's only `TetherAPI_1` prim | which tether |
| `end` | `attach` | `attach` or `anchor` |
| `capacity` | 500 N | rated load (illustrative: give your cell's) |
| `sensitivity` | 0.002 V/V | rated output at capacity (2 mV/V, a common figure) |
| `excitation` | 5 V | also the ADC's AVDD |
| `nonlinearity`, `hysteresis` | 0 | fractions of capacity |
| `noise`, `offset`, `gain_error` | 0 | at the cell, in N (offset: zero balance) |
| `adc/bits`, `adc/gain`, `adc/full_scale`, `adc/noise`, `adc/samples` | 24, 128, ±0.5·excitation/gain, 50 nV, 1 | the converter, in V at the bridge |
| `rated_strain` | 0.001 | strain at the gauges at capacity, for `strain` |

| Signal | Unit | |
|---|---|---|
| `sensor/<n>/force` | N | the reading |
| `sensor/<n>/strain` | 1 | force / capacity × rated_strain |
| `sensor/<n>/valid` | 1 | 0 while the tether domain is off |

Test: `test_kinds_tension_sensor` (the ADC's step, HX711 noise in newtons, the bow and the
hysteresis loop at half load, the anchor end, tether off, ground truth).

On ROS by default: the fuel gauge as `/<n>/battery_state` (sensor_msgs/BatteryState), the
others' numbers on `/<n>/<quantity>` ("ROS topics of the built-in kinds" above). A
`--ros-topics FILE.yaml` map (`bridges/ros2/src/SignalTopics.cpp`) renames or retypes them:

```yaml
publish:
  - {signal: sensor/fuel_gauge/voltage, topic: /battery/voltage, type: std_msgs/msg/Float64}
  - {signal: sensor/fuel_gauge/temperature, topic: /battery/temperature, type: sensor_msgs/msg/Temperature, frame: base_link}
  - {signal: sensor/esc_1/speed, topic: /esc_1/speed, type: std_msgs/msg/Float64}
  - {signal: sensor/tether_load/force, topic: /tether/force, type: std_msgs/msg/Float64}
```

A sample: `assets/robots/quad_x500/quad_x500_telemetry.usda` adds to the sample quadcopter
KISS telemetry from its first rotor's ESC, the pack's fuel gauge and an INA226 on its bus
(`CustomSensorAPI_1`, `sensor:custom:param:<path>`).

The environment kinds below read other domains' signals as optional inputs: while the domain
that writes one is off (no weather, no aerodynamics), the outputs that need it are NaN and
`sensor/<name>/valid` is 0. Their closed forms are in `lagrange/sensors/EnvironmentSensors.hpp`;
the shared code (the body's motion, a collider scene for a ray or two, the sun) in
`src/kinds/EnvCommon.hpp` and `SunCommon.hpp`. Datasheet "±" accuracies are taken as 2σ. Noise,
offsets and lags are off under `ground_truth`, which reads the environment exactly.

### anemometer

Ultrasonic (2D or 3D, after the Gill WindSonic) or cup and vane (after the R. M. Young 05103).

```xml
<gazebo reference="mast">
  <sensor name="wind" type="anemometer">
    <update_rate>4</update_rate>
    <anemometer><type>ultrasonic</type><axes>2</axes></anemometer>
  </sensor>
</gazebo>
```

| Parameter | Default | Meaning |
|---|---|---|
| `type` (text) | `ultrasonic` | or `cup` (cups and a vane, horizontal only) |
| `axes` | 2 | 2 or 3 (ultrasonic) |
| `noise/fraction` | 0.02 (cup 0) | speed noise, 1σ, of the speed (the WindSonic's 2 % RMSE) |
| `noise/speed` | 0.01 m/s (cup 0) | speed noise, 1σ, the least |
| `noise/direction` | 1° (cup 0) | rad, 1σ (the WindSonic's ±2° as 2σ) |
| `resolution/speed`, `resolution/direction` | 0.01 m/s, 1° | |
| `threshold/speed` | 0.01 m/s (cup 1.0 m/s) | below it the speed reads 0 |
| `threshold/direction` | 0.01 m/s (cup 1.1 m/s) | below it the vane holds |
| `distance_constant` | 0 (cup 2.7 m) | cup lag: first order, τ = L / V |
| `air_rotor` (text) | | read `rotor/<j>/air_velocity` instead of `env/wind/velocity`; a name that is no `RotorAPI_1` joint is refused |

Signals: `wind_velocity` (Vec3, m/s, sensor frame: where the air goes, relative to the sensor),
`wind_speed` (m/s), `wind_direction` (rad, [0, 2π): where it blows from, clockwise from the
sensor's +x seen from above, the meteorological convention when +x points north), `valid`.

Model: the air is the aerodynamics domain's `env/wind/velocity` (mean wind and gust at the first
link's height; or with `air_rotor` a rotor's air, the wind plus other robots' downwash at its hub;
a robot's own downwash at the sensor is not included), less the sensor's own velocity
(v + ω × r of its body), in the sensor frame; 2D drops the vertical part. The noise is a gain
error on the speed and an angle error on the direction. Without the aerodynamics domain (no
`AeroAPI_1` link or rotors, or no site air density) it is not valid. Check: `test_kinds_anemometer`
(relative air and heading exact, the direction convention, WindSonic accuracy, the cup's threshold
and its lag against y = V·(1 − e^(−t·V/L))). References: Gill WindSonic User Manual 1405-PS-0019
Issue 30 (2022), appendix A (speed "± 2% RMSE (at 12m/s)", 0.01 m/s resolution; direction ±2° at
12 m/s, ±3° in the 2009 datasheet, 1°; threshold 0.01 m/s in the datasheets), R. M. Young 05103
Wind Monitor manual 05103-90 rev. N062819 (propeller threshold 1.0 m/s, vane 1.1 m/s at 10°
displacement, distance constant 2.7 m for 63 % recovery).

### airspeed

A pitot-static probe along the sensor's +x on a differential pressure sensor (after the TE
MS4525DO-DS5AI001DP that PX4 and ArduPilot use).

```xml
<sensor name="pitot" type="airspeed">
  <update_rate>50</update_rate>
  <airspeed><noise><stddev>2.0</stddev></noise></airspeed>
</sensor>
```

| Parameter | Default | Meaning |
|---|---|---|
| `noise/stddev` | 0 Pa | 1σ |
| `offset` | 0 Pa | zero offset |
| `range` | 6894.757 Pa (±1 psi) | the reading clips there |
| `resolution` | 1.052 Pa | one count: 2·1 psi / (0.8·16383) |
| `temperature_resolution` | 0.098 K | 11 bits over −50 … 150 °C |
| `air_density` | | kg/m³, only when neither the weather nor the site gives one |
| `air_rotor` (text) | | as the anemometer's |

Signals: `differential_pressure` (Pa), `airspeed` (m/s, true: from the measured pressure at the
air's density), `indicated_airspeed` (m/s, at the ICAO sea-level 1.225 kg/m³), `temperature` (K:
the weather's `env/air_temperature`, else the site's ambient temperature, else NaN), `valid`.

Model: q = ½·ρ·V·|V|, V the air relative to the probe along its axis (the wind as the anemometer;
without the aerodynamics domain, still air, so the probe still sees its own motion). ρ is the
weather's `env/air_density` when it runs, else the site's (`--air-density`, the profile's
`rotor_aero.air_density` or `aerodynamics.air_density`, handed to the sensors domain as config
`air_density`), else the parameter; with none it is not valid. Incompressible: under 1 % low below
Mach 0.3. Check: `test_kinds_airspeed` (½ρV², a headwind, the weather's density first, the count
grid). The MS4525DO's total error band, ±1 % of span (datasheet MS4525DO 04/2019, p. 4: about
±138 Pa on ±1 psi, over −10 to 85 °C), is not modelled by default: give it as `offset`.

### rangefinder

One laser beam along the sensor's +x (after the Benewake TFmini-S; set the range for a LightWare
SF11/C, 0.2 to 100 m). Kind `tof`, the sample plugin, is a different sensor (a cone of rays).

```xml
<sensor name="altimeter" type="rangefinder">
  <pose>0 0 -0.05 0 1.5708 0</pose>
  <update_rate>100</update_rate>
  <rangefinder><range><min>0.1</min><max>12</max></range></rangefinder>
</sensor>
```

| Parameter | Default | Meaning |
|---|---|---|
| `range/min`, `range/max` | 0.1 m, 12 m | |
| `noise/stddev` | 0.03 m | 1σ, the least |
| `noise/fraction` | 0.005 | 1σ of the range |
| `resolution` | 0.01 m | |

Signals: `range` (m; +inf when nothing is within reach, −inf when something is closer than
`range/min`, REP 117), `valid`.

Model: the exact CPU ray cast against every collider at its body's pose (`ShapeCaster`, on every
tier), its own body's colliders skipped. Noise σ = max(stddev, fraction·r) (TFmini-S: ±6 cm to
6 m, ±1 % to 12 m). With the weather its reach shrinks as a lidar's does
(`lidarDetectionRange`: `env/extinction/nir_905` and its `sensor/<name>/contamination`); the
target's reflectivity is not used. Check: `test_kinds_rangefinder`.

ROS: `/altimeter/range` (`sensor_msgs/Range`, INFRARED, the kind's limits) and `/altimeter/valid`
by default; the same by hand: `{signal: sensor/altimeter/range, topic: /altimeter/range, type:
sensor_msgs/msg/Range, frame: altimeter, params: {radiation_type: 1, min_range: 0.1, max_range:
12.0}}`.

### light_sensor

An ambient light sensor whose face looks along the sensor's +x (after the ROHM BH1750FVI).

```xml
<sensor name="lux" type="light_sensor">
  <pose>0 0 0.2 0 -1.5708 0</pose>   <!-- +x up -->
  <update_rate>8</update_rate>
  <light_sensor><start_utc>2026-06-21T03:00:00Z</start_utc></light_sensor>
</sensor>
```

| Parameter | Default | Meaning |
|---|---|---|
| `start_utc` (text) | | UTC date and time at t = 0, `YYYY-MM-DDTHH:MM:SSZ` |
| `sky` (text) | `clear` | or `overcast` |
| `fog_depth` | 100 m | illustrative depth of fog the beam crosses per air mass |
| `gain` | 1 | the BH1750's part-to-part ratio, 0.8 to 1.2 |
| `noise/fraction` | 0 | 1σ, of the reading |
| `resolution` | 1/1.2 lx | one count in H-resolution mode (BH1750FVI datasheet Rev.D, p. 7: lx = counts / 1.2) |
| `max_illuminance` | 54 612 lx | 65535 counts / 1.2 |
| `ground_reflectance` | 0 | light from the ground |

Signals: `illuminance` (lx, as cd·sr/m² with no quantity kind: `lagrange::units` has no lux),
`valid`. ROS: `sensor_msgs/msg/Illuminance`.

Model: the sun (below) and the daylight model of Gillette, Pierpoint and Treado (NBS, 1982), as
the IES daylight availability practice (J. IES 13(4), 1984; not read) took it up: direct normal
E_dn = 127.5 klx·(1 + 0.033·cos(2π·day/365))·exp(−0.21·m), m the Kasten–Young air mass (the
source has 1/sin α), and diffuse horizontal (0.8 + 15.5·√sin α) klx on a clear sky,
(0.3 + 21·sin α) klx and no beam on an overcast one (Robbins and Hunter, SERI/TR-254-1703, 1983,
eqs. 2-16, 2-17 and 3-6, which quote it). The clear-sky extinction 0.21 per air mass is the
rural design value of LBL-17727 ("Analysis of atmospheric turbidity for daylight calculations",
Lawrence Berkeley Laboratory, 1984: 0.21, 0.32 and 0.53 for rural, urban and industrial air). On the face: the beam ×
cos of incidence, zero when one ray toward the sun meets a collider (its own body skipped), plus
the sky by the isotropic model (1 + n_z)/2 and the ground's ρ_g·E_gh·(1 − n_z)/2. The weather:
rain or snow (`env/rain_rate`, `env/snow_rate`) make the sky overcast; fog (`env/extinction/visible`)
dims the beam by exp(−σ·fog_depth·m). Night is zero (no twilight, moon or lamps; cloud cover is
not a weather signal yet). The cosine and spectral response are ideal. Full sun saturates the
BH1750 (65535 counts, 54 612 lx); set `max_illuminance` for another part. Check: `test_kinds_light_sensor`
(Meeus' Julian day example, the NREL SPA example within 0.02°, Kasten–Young at the horizon, the IES
closed forms, shade and overcast).

### sun_sensor

A two-axis sun sensor looking along +x (after the Solar MEMS nanoSSOC-A60, product sheet
nanoSSOC-A60, solar-mems.com, 2024: field of view ±60°, accuracy < 0.5° (3σ), precision
< 0.1°). The noise is the accuracy over 3, which holds its bias too; for the noise alone give the
precision.

| Parameter | Default | Meaning |
|---|---|---|
| `start_utc`, `sky`, `fog_depth` | | as the light sensor's |
| `fov` | 120° | full cone angle |
| `noise/stddev` | 0.167° | rad, 1σ per axis |
| `min_illuminance` | 1000 lx | illustrative (Solar MEMS gives no threshold): the least direct beam it finds the sun in |

Signals: `sun_direction` (Vec3, unit, sensor frame; NaN when the sun is not seen), `sun_visible`
(the sun is in the field of view, not shaded and its beam is strong enough), `valid`.

The sun, for both kinds: the NOAA solar position algorithm (NOAA GML Solar Calculator, after
Meeus 1998; about 0.01°, with NOAA's refraction) at the sensor's geodetic position (the site's geo
origin, `--geo-origin`) and the time `start_utc` + t, in world ENU. Without a geo origin or
`start_utc` it is the weather's `env/sun_direction` (the `WeatherAPI_1` prim's, day 80); without
that, not valid. Check: `test_kinds_sun_sensor` and the solar checks above (Reda and Andreas,
NREL/TP-560-34302, example: zenith 50.11162°, azimuth 194.34024°).

### rain_sensor

An optical rain gauge (after the Hydreon RG-15: Rain Gauge Model RG-15 Instructions, SW rev 1.000,
2022.07.18: ±10 % "under controlled conditions, accuracy may vary near/below freezing", 0.2 or
0.02 mm).

| Parameter | Default | Meaning |
|---|---|---|
| `resolution` | 0.2 mm (0.2e-3 m) | the accumulation's step (high resolution 0.02 mm) |
| `noise/gain` | 0.05 | 1σ of a gain error drawn once per run (RG-15 ±10 %) |
| `threshold` | 0.1 mm/h | illustrative: `raining` above it (Hydreon defines no rate; its rain event lasts from the first drop to 60 min after the last) |
| `time_constant` | 0 s | first-order lag of the rate |
| `snow` | 0 | 1: count snow too (melted water) |

Signals: `rain_rate` (m/s of water), `accumulation` (m, since the start, in whole steps),
`raining`, `valid`. ROS: `std_msgs/msg/Float64` and `Bool`. From the weather's `env/rain_rate`
(and `env/snow_rate`). Check: `test_kinds_rain_sensor` (∫R dt).

### humidity_sensor

Relative humidity and temperature (after the Sensirion SHT40).

| Parameter | Default | Meaning |
|---|---|---|
| `time_constant/humidity`, `time_constant/temperature` | 4 s, 2 s | τ63 |
| `noise/humidity`, `noise/temperature` | 0.000267, 0.0133 K | 1σ: the SHT4x's high-precision repeatability, 0.08 %RH and 0.04 °C, is 3σ (datasheet V7.3, tables 1–2) |
| `offset/humidity`, `offset/temperature` | 0 | its accuracy (SHT40: ±0.018, ±0.2 K typical) |
| `resolution/humidity`, `resolution/temperature` | 0.0001, 0.01 K | |

Signals: `relative_humidity` (0 to 1; ROS `sensor_msgs/msg/RelativeHumidity` takes 0 to 1 too),
`temperature` (K), `valid`. From the weather's `env/relative_humidity` and `env/air_temperature`
through a first-order lag, y += (1 − e^(−Δt/τ))·(u − y), exact for an input held over a step.
Without the weather the humidity is NaN (`valid` 0) and the temperature the site's ambient one,
if given. Check: `test_kinds_humidity_sensor` (the lag's closed form, a step through τ).

### visibility_sensor

A forward-scatter visibility sensor (after the Vaisala PWD22, datasheet B210385EN-F: 10 to 20 000 m,
±10 % to 10 km, ±15 % from 10 to 20 km).

| Parameter | Default | Meaning |
|---|---|---|
| `range/min`, `range/max` | 10 m, 20 000 m | |
| `noise/fraction` | 0.05 | 1σ, of the MOR (±10 % as 2σ) |
| `noise/fraction_far` | 1.5 × `noise/fraction` | 1σ beyond 10 km (±15 %) |
| `time_constant` | 60 s | first-order lag of the extinction: PWD22 User's Guide M210543EN-B, table 24, "Time constant 60 seconds" (its 1 min output is a running mean of 15 s values) |

Signals: `visibility` (m: the meteorological optical range), `extinction` (1/m, what it measured),
`valid`. Model: MOR = −ln 0.05 / σ ≈ 3/σ (Koschmieder with WMO's 5 % threshold, WMO-No. 8 Vol. I
ch. 9), σ the weather's `env/extinction/visible` (fog, rain, snow and dust). The weather's
`visibility` is at a 2 % threshold (σ = 3.912/V), so a fog of V reads an MOR of 0.766·V, as a
real sensor would. Check: `test_kinds_visibility_sensor`.

They are on ROS by default ("ROS topics of the built-in kinds" above); a `--ros-topics` map
puts them elsewhere, which replaces the defaults it covers:

```yaml
publish:
  - {signal: sensor/wind/wind_velocity, topic: /wind, type: geometry_msgs/msg/Vector3Stamped,
     frame: wind}
  - {signal: sensor/pitot/differential_pressure, topic: /pitot/pressure,
     type: sensor_msgs/msg/FluidPressure, frame: pitot}
  - {signal: sensor/lux/illuminance, topic: /lux, type: sensor_msgs/msg/Illuminance, frame: lux}
  - {signal: sensor/sht/relative_humidity, topic: /sht/humidity,
     type: sensor_msgs/msg/RelativeHumidity, frame: sht}
  - {signal: sensor/sht/temperature, topic: /sht/temperature, type: sensor_msgs/msg/Temperature,
     frame: sht}
  - {signal: sensor/pwd/visibility, topic: /visibility, type: std_msgs/msg/Float64}
```

The contact, ray and joint kinds below (`bumper`, `cliff`, `wheel_odometry`, `safety_scanner`,
`force_torque`, `joint_torque`) read only declared inputs (`body/<b>/pose`, `body/<b>/twist`,
`body/<b>/contact_force`, `joint/<j>/position`, `joint/<j>/reaction`); `cliff` and
`safety_scanner` cast against every collider at its body's pose on the CPU (`ShapeCaster`, the
Tier 0 lidar's exact tests; `kinds/KindScene.hpp`), skipping their own body's colliders, so they
run the same on every tier. They publish by default ("ROS topics of the built-in kinds" above);
a `--ros-topics` map renames them (example at the end).

### bumper

A contact switch: pressed when what touches the sensor's body pushes it hard enough from the
side it watches. It reads `body/<b>/contact_force` (the sum of every contact on the body, world
frame, rigid_body.md "Contact force"), averaged over the sensor's period, turns it into the
sensor frame and takes its xy part (the floor's support is vertical). The contact sits opposite
the push, at angle `atan2(−F_y, −F_x)` (0 straight ahead); the bumper is pressed when the push is
at least `threshold` and that angle is in `[sector_min, sector_max]`. It needs physics; on a body
that never moves the run refuses to start.

```xml
<gazebo reference="base_link">
  <sensor name="front_bumper" type="bumper">
    <pose>0.3 0 0.05 0 0 0</pose><update_rate>100</update_rate>
    <bumper><threshold>5</threshold><sector_min>-1.05</sector_min><sector_max>1.05</sector_max></bumper>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `threshold` | N | 5 (illustrative) | push that closes the switch |
| `sector_min`, `sector_max` | rad | −π, π | angles of the contact it reports, sensor frame |
| `vertical` | 0/1 | 0 | 1: the whole force counts, the vertical part too |

Signals: `sensor/<n>/pressed` (Bool), `force` (N, the push's size), `direction` (rad, where the
contact is). The body is all one switch: two bumpers on one body see the same contact sum, told
apart by their sectors. A floor that drags the body sideways (a sliding caster) pushes it too;
the threshold is set above that. Test `test_kinds_bumper`: a 10 kg box pushed by 30 N into a
wall reads 30 N ±2 % straight ahead and closes the front bumper, not the rear one; standing alone
it closes neither (the `vertical` one reads m·g ±1 %).

### cliff

A robot vacuum's cliff sensor: an infrared range finder looking down the sensor's −z. One ray, or
`rays` rays (one straight down and the others on a ring at `beam_angle`) whose nearest return
wins, as an IR spot's strongest return does. The range is clamped up to `min_range` (the floor is
there), +inf beyond `max_range`; `cliff` is 1 when the range is +inf or above `threshold`.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `min_range`, `max_range` | m | 0.005, 0.15 (illustrative) | what it can measure |
| `threshold` | m | 0.06 (illustrative) | farther than this is a drop |
| `rays` | 1 | 1 | rays, 1 to 64 |
| `beam_angle` | rad | 0 | half angle of the ring |
| `range_noise` | m | 0 | 1 sigma, per reading |
| `ignore_own_body` | 0/1 | 1 | rays pass the sensor's own body's colliders |

Signals: `sensor/<n>/range` (m, +inf for none), `cliff` (Bool). Floor material and colour do not
change the reading (an IR cliff sensor's real weakness on black carpet is not modelled). Test
`test_kinds_cliff`: over a table it reads its height to 1e-12 m, past the edge (floor 0.5 m down,
out of range) and held 9 cm high (threshold 6 cm) it reports a cliff; a cone 1 cm past the edge
still finds the table at 0.04/cos(0.3) m; the noise has its sigma within 5 % over 2000 readings;
ground truth reads exactly.

### wheel_odometry

Dead reckoning of a differential drive, as its base controller computes it. Each step it reads
the wheels' `joint/<j>/position`, quantised to whole counts when `encoder_resolution` (counts per
revolution) is given, turns the count change into distance with the radius it believes, and
integrates the unicycle model exactly over the step (an arc of `ds = (s_l + s_r)/2`,
`dψ = (s_r − s_l)/b`). Nothing models slip: when the physics lets a wheel spin or skid, the wheel
turns no longer match the body's motion, and the odometry drifts as a real robot's does. The
odometry frame is the body's pose at its first step, flattened to x, y, yaw. Without
`left_joint`/`right_joint` it finds the two driven hinges hanging off its body that carry a
cylinder or sphere (the one at +y is left; on a passive robot, one spawned or added to a run,
whose joints nothing drives, the two such hinges); without `wheel_radius`/`wheel_separation` it takes
the wheel collider's radius and the wheels' distance apart, as the server's diff-drive controller
does. Each wheel's forward sense comes from its axis in the body frame.

```xml
<gazebo reference="base_link">
  <sensor name="wheel_odom" type="wheel_odometry">
    <update_rate>50</update_rate>
    <wheel_odometry>
      <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
      <encoder_resolution>4096</encoder_resolution><radius_error>0.01</radius_error>
    </wheel_odometry>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `left_joint`, `right_joint` (text) | | detected | the wheel joints |
| `wheel_radius` | m | the wheel collider's | radius it believes |
| `wheel_separation` (or `track_width`) | m | the wheels' distance apart | track it believes |
| `radius_error` | 1 | 0 | relative error of both radii (0.01: 1 % large) |
| `radius_error_left`, `radius_error_right` | 1 | `radius_error` | per wheel |
| `separation_error` | 1 | 0 | relative error of the track |
| `encoder_resolution` | counts/rev | 0 (exact angle) | encoder counts per wheel turn |

Signals: `sensor/<n>/pose` (Pose, frame `odom`, z = 0), `yaw` (rad), `twist` (Wrench of m/s:
forward speed, 0, 0, then 0, 0, turn rate, body frame). Ground truth: the body's own pose in the
odometry frame and its speeds, exactly. Test `test_kinds_wheel_odometry` (MuJoCo, two wheels and
two caster balls): driving straight 3 s on μ = 1 it is 0.13 % long against the ground truth
(2.0376 m for 2.0351 m), turning on the spot 2 s 1.2 % (0.3033 rad for 0.2997 rad); a radius 5 %
large reads 5 % long; a 1024-count encoder stays within a count of the exact angle; on μ = 0.05
with 40 N of wheel force asked for, the spinning wheels read 70.9 m while the body moved 0.74 m.

### safety_scanner

A safety laser scanner in the manner of SICK's nanoScan3 and microScan3: a scan in the sensor's
xy plane over `fov`, each return checked against a protective field and a warning field in the
sensor frame. A field is a polygon (`<field>/polygon`, text `"x,y x,y x,y ..."`, even-odd rule), a
sector (`<field>/radius`, within the aperture) or a rectangle (`<field>/x_min` (default 0),
`x_max`, `y_min`, `y_max`). The outputs switch `response_time` after the scan that saw the
change, rounded up to whole scans of the sensor's period (a ring of scan states): `protective` is
what turns the OSSDs off. Defaults are the nanoScan3 Core I/O's, SICK data sheet NANS3-AAAZ30AN1
(part 1100333): scanning angle 275°, angular resolution 0.17°, response time 70 ms, distance
measuring range 40 m (protective fields up to 3 m and warning fields up to 10 m on that part; the
fields themselves are the user's, with no default, and a protective field is required). The data
sheet gives no scan cycle time; the operating instructions do: 30 ms (nanoScan3 I/O operating
instructions 8024602/1W27/2026-05-26, Japanese edition, §14.2 table 33 "scan cycle time 30 ms";
§14.3: response time t_R = n × 30 ms + 10 ms, n the multiple sampling, 2 by default, hence the
70 ms). The sensor's update rate is its scan cycle: give `<update_rate>` 33.333 (a 30 ms period)
for a nanoScan3. Not modelled: the 65 mm protective field supplement (add it to the field), object
resolution (20–200 mm), remission limits (1.8 % up), contour detection, monitoring cases switched
by inputs. Two speed-dependent monitoring cases are: with `switch_speed` (m/s), at or above that
speed along the sensor's x (from `body/<b>/twist`) the `protective_fast` and `warning_fast` fields
watch (`warning_fast` defaults to `warning`), and `sensor/<n>/case` reads 1.

```xml
<gazebo reference="base_link">
  <sensor name="safety" type="safety_scanner">
    <pose>0.3 0 0.15 0 0 0</pose><update_rate>33.333</update_rate>
    <safety_scanner>
      <protective><x_max>0.6</x_max><y_min>-0.35</y_min><y_max>0.35</y_max></protective>
      <warning><polygon>0,-0.6 1.5,-0.6 1.5,0.6 0,0.6</polygon></warning>
    </safety_scanner>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `fov` | rad | 4.7997 (275°) | aperture, centred on +x |
| `angular_resolution` | rad | 0.002967 (0.17°) | between beams |
| `min_range`, `max_range` | m | 0, 40 | returns kept |
| `range_noise` | m | 0 | 1 sigma, per beam |
| `response_time` | s | 0.070 | delay of the outputs, 0 to 60 s |
| `protective/...`, `warning/...` | m | none | the fields, above |
| `switch_speed` | m/s | 0 (off) | forward speed of the fast case |
| `protective_fast/...`, `warning_fast/...` | m | none, `warning` | the fast case's fields |

Signals: `sensor/<n>/protective`, `warning` (Bool, delayed), `min_distance` (m, nearest return
of the latest scan, +inf for none), `ranges` (ScalarArray, m, +inf for none; a LaserScan with
`angle_min = −fov/2`). Ground truth: no noise and no delay. Test `test_kinds_safety_scanner`: a
plate driving at 1 m/s enters the 2 m warning polygon at 1 s and the 1 m protective rectangle at
2 s; at 25 Hz each output is still off one scan after and on by the third scan (70 ms → 2 scans),
at once under ground truth; `min_distance` is the plate's face; driving at 1 m/s at a wall, a
scanner fast above 0.5 m/s with a 1.5 m fast field trips a metre before one with a fixed 0.5 m
field; no protective field refuses to start.

### force_torque

A six-axis force-torque sensor (a wrist sensor) from `joint/<j>/reaction`: what the parent does
to the child at the child's origin, world frame, including the motor's torque (rigid_body.md
"Contact force and force-torque"). The joint is `joint` (text), else the one whose child is the
sensor's body. The wrench is moved to the sensor's origin (`τ_s = τ_j + (p_j − p_s) × F`) and
turned into the sensor frame; `measure_direction` `child_to_parent` reverses it. Then per axis:
offset, noise, a first-order low-pass at `cutoff_frequency` (`α = 1 − exp(−2π·f_c·dt)`), range
(clipped, `saturated` = 1), resolution (rounded to whole steps). Preset `mini45_si_580_20` is ATI's
Mini45 with calibration SI-580-20 (ati-ia.com, F/T sensor models, Mini45, table "Metric
Calibrations (SI)" for a 16-bit DAQ, ECAT, Net, TWE or WNet F/T system): ranges Fx, Fy 580 N, Fz 1160 N, Tx, Ty, Tz 20 N·m; resolutions Fx, Fy, Fz 1/4 N, Tx, Ty
1/188 N·m, Tz 1/376 N·m. ATI quotes those resolutions after dropping four counts of noise; the
preset's noise of one resolution step (1 sigma) per axis is our assumption. Temperature drift is
not modelled.

```xml
<gazebo reference="tool0">
  <sensor name="wrist_ft" type="force_torque">
    <update_rate>1000</update_rate>
    <force_torque><joint>wrist_3_joint</joint><preset>mini45_si_580_20</preset>
      <cutoff_frequency>200</cutoff_frequency></force_torque>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `joint`, `measure_direction`, `preset` (text) | | the body's joint, `parent_to_child`, none | above |
| `force_range`, `torque_range`; per axis `fx_range` ... `tz_range` | N, N·m | ∞ (or the preset's) | clipping |
| `force_resolution`, `torque_resolution`; `fx_resolution` ... | N, N·m | 0 (or the preset's) | step |
| `force_noise`, `torque_noise`; `fx_noise` ... | N, N·m | 0 (or a step) | 1 sigma |
| `force_offset`, `torque_offset`; `fx_offset` ... | N, N·m | 0 | bias |
| `cutoff_frequency` | Hz | 0 (none) | first-order low-pass |

Signals (frame `sensor:<n>`): `sensor/<n>/wrench` (Wrench, N then N·m), `force` (Vec3, N),
`torque` (Vec3, N·m), `saturated` (Bool). Ground truth: the moved and turned wrench exactly (no
offset, noise, filter, range or step).

### joint_torque

A link-side joint torque sensor, as in each joint of a Franka Panda or a KUKA LBR iiwa: the
reaction's moment about the joint axis (its force along the axis for a slide), so at rest it reads
what the motor holds. Parameters `joint` (text, default the body's joint), `range`, `resolution`,
`noise` (1 sigma), `offset` (N·m or N) and `cutoff_frequency` (Hz), all ideal by default: give the
part's figures. Franka publishes none for the FR3 (datasheet R02212 2.6, July 2026: "link-side
torque sensor in all 7 axes"); KUKA gives "axis-specific torque accuracy ±2 % of maximum torque"
for the LBR iiwa 7 R800 and 14 R820 (spec sheet), no resolution or noise. Signals:
`sensor/<n>/torque` (N·m; `force`, N, on a slide), `saturated` (Bool).

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `joint` (text) | | the joint whose child is the sensor's body | |
| `range` | N·m (N) | ∞ | clipping; `saturated` reads 1 |
| `resolution` | N·m (N) | 0: none | step |
| `noise`, `offset` | N·m (N) | 0, 0 | 1σ, bias |
| `cutoff_frequency` | Hz | 0: none | first-order low-pass |

Test `test_kinds_force_torque` (both kinds): a 2 kg arm on a hinge held horizontal by its motor
(m·g·d = 9.80665 N·m): at the joint F = (0, 0, m·g) and τ = (0, −m·g·d, 0) within 0.5 %; at its
centre of mass no moment; `child_to_parent` the opposite; turned 90° about z the moment is about
the sensor's x; the joint torque sensor reads −m·g·d; the Mini45 clips Fz at a 10 N range and its
readings are whole 1/4 N and 1/188 N·m steps; the 5 Hz low-pass follows 1 − exp(−2π·5·t) within
2 % of the weight at 20, 50 and 100 ms; ground truth is exact.

### ROS topics for these kinds

By default `/front_bumper/pressed` (Bool), `/cliff_fl/range` (Range), `/wheel_odom/odom`
(Odometry), `/safety/scan` (LaserScan), `/safety/protective`, `/wrist_ft/wrench`
(WrenchStamped), `/<joint torque sensor>/joint_state` (JointState). A map that wants other names:

```yaml
publish:
  - {signal: sensor/front_bumper/pressed, topic: /bumper/front, type: std_msgs/msg/Bool}
  - {signal: sensor/cliff_fl/range, topic: /cliff/front_left, type: sensor_msgs/msg/Range,
     frame: cliff_fl, params: {min_range: 0.005, max_range: 0.15, field_of_view: 0.1}}
  - {signal: sensor/wheel_odom/pose, topic: /wheel_odom/pose, type: geometry_msgs/msg/PoseStamped,
     frame: odom}
  - {signal: sensor/wheel_odom/twist, topic: /wheel_odom/twist, type: geometry_msgs/msg/TwistStamped,
     frame: base_link}
  - {signal: sensor/safety/protective, topic: /safety/protective, type: std_msgs/msg/Bool}
  - {signal: sensor/safety/ranges, topic: /safety/scan, type: sensor_msgs/msg/LaserScan,
     frame: safety, params: {angle_min: -2.39983, angle_max: 2.39983, range_min: 0.0,
     range_max: 40.0}}
  - {signal: sensor/wrist_ft/wrench, topic: /wrist_ft, type: geometry_msgs/msg/WrenchStamped,
     frame: wrist_ft}
```

Kinds that find things by name (`detections`, `landing_target`, `line_sensor`, `tag_reader`)
search the model's scene prims (`Model::scene`, every prim of the robot and the world, with the
box of its meshes and its children's meshes on the same body); run without a scene (an old
caller of `createSensorModules(const World&)`) they see only `World::prims`. Names match a whole
prim name, a whole path or the end of a path (`landing_pad`, `/open_field/landing_pad`), or, where
said, whole words of a name split at `_ - . /` and spaces. The URDF examples below go inside a
`<gazebo reference="LINK">` block, as every URDF sensor does. Their ray checks (occlusion, the floor
under a sensor) are exact casts against every collider at its body's pose (`ShapeCaster`, the
CPU reference) that look through the colliders of the sensor's own body: a camera looks out of
its housing, an array out of its chassis.

### detections

Ground-truth annotations for machine-learning datasets, attached to a camera of the same robot:
from that camera's instance image (`sensor/<camera>/instance`, the scene prim index per pixel)
and the scene, every visible object's 2D box, visible pixels, occlusion and truncation, class and
3D box, and a semantic segmentation image. It runs in step with the camera, at the camera's
rate (its own `update_rate` is not used), and reads the frame the camera drew in the same tick.

```xml
<sensor name="front_labels" type="detections">
  <camera>front_camera</camera>
  <semantic>crate,pallet=1;forklift=2;human,person=3</semantic>
  <max_objects>32</max_objects>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `camera` (text) | | required | the camera sensor whose images it labels |
| `semantic` (text) | | the table below | `word,word=ID;word=ID`: the first rule with a whole word of the object's name gives its class |
| `max_objects` | | 64 | rows of `detections`, the most pixels first |
| `min_pixels` | px | 1 | fewer visible pixels: not listed (still in the semantic image) |
| `occlusion` | | 1 | 0: no occlusion renders (field 7 NaN) |

**Objects.** A robot is one object (every body below one root body, drawn parts and all), a
walking human (RFC 0009, one robot per person) is one object, and every static scene prim with a
mesh is one. The object's id (field 0) is the lowest scene prim index of its parts, the number
its pixels carry in `sensor/<camera>/instance`.

**Classes.** The class comes from the words of the root body's name, then the body's, then each
part's prim path; a moving body nothing names is 9 (robot); anything else is 0 (unlabelled).
Without `semantic`:

| Id | Words |
|---|---|
| 1 ground | ground, floor, terrain |
| 2 road | road, lane, sidewalk, crossing, marking |
| 3 building | building, wall, house, shed, roof |
| 4 vehicle | car, sedan, vehicle, truck, bus, van |
| 5 person | human, person, pedestrian, walker, actor |
| 6 vegetation | tree, bush, hedge, plant, vegetation |
| 7 pole, sign | pole, sign, light, signal |
| 8 goods | box, crate, pallet, rack, shelf, barrel, cone |
| 9 robot | any other moving body |

**Signals.**

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/detections` | ScalarArray, `max_objects` × 16 | mixed (below) | one row per object, NaN rows after the last |
| `sensor/<n>/count` | Int | 1 | rows filled |
| `sensor/<n>/semantic` | Image `r32u` | | the class id per pixel, 0 for nothing or unlabelled, the camera's size |

The 16 fields of a row:

| # | Field | Unit |
|---|---|---|
| 0 | object id (scene prim index) | |
| 1 | class id | |
| 2–5 | 2D box `u_min, v_min, u_max, v_max`: the first and last visible pixel columns and rows (inclusive; pixel (0, 0) is the top left, u right, v down) | px |
| 6 | visible pixels | px |
| 7 | occlusion: 1 − visible / pixels it covers drawn alone (NaN without a GPU device, with `occlusion` 0 or on a fisheye camera) | 1 |
| 8 | truncation: the share of its projected 3D box outside the image (NaN when the box is partly behind the camera, and on a fisheye camera) | 1 |
| 9–11 | 3D box centre in the camera's optical frame (x right, y down, z forward) | m |
| 12–14 | 3D box length, width, height: along the object's own x, y, z (the root body's frame; a static prim's own frame) | m |
| 15 | yaw: KITTI's `rotation_y`, the object's x axis about the optical y axis, 0 along the optical x, −π/2 facing away | rad |

**Model.** The 2D boxes, pixel counts and the semantic image come straight from the instance
image, so they are exact for what the camera drew, lens distortion and rolling shutter included.
The occlusion draws each listed object alone with the same pinhole (no lens model) on the
camera's device: exact without lens distortion, approximate with it. That is one more render
per listed object and frame (`max_objects`, `occlusion` 0 to skip them in a busy scene). The 3D box is the
axis-aligned box of the object's meshes in its frame, at this step's body poses (the box of the
mesh vertices, not the tightest box). The conventions follow the KITTI object benchmark (Geiger,
Lenz, Urtasun 2012, "Are we ready for autonomous driving?", CVPR; its `devkit_object` readme
defines the box, `truncated`, `occluded` and `rotation_y`); KITTI's occlusion is a level
(0–3) and its 3D box centre is the bottom face's, while here occlusion is a fraction and the
centre is the box's middle. A GPU path (invariant 7): not deterministic, like the camera. With
the cameras as ground truth (empty images) it lists nothing. `test_kinds_detections`: three boxes
at known poses give their projected 2D boxes within a pixel, the occlusion of the one behind
within 0.02 of the closed form, their 3D boxes and classes (llvmpipe or a GPU).

The rows are on ROS by default as `vision_msgs/Detection2DArray` on `/<n>/detections` and
`Detection3DArray` on `/<n>/detections_3d`, the semantic image as `/<n>/semantic/image_raw`
("ROS topics of the built-in kinds", "Images of kinds"). The raw rows as a
`std_msgs/msg/Float64MultiArray` take a `--ros-topics` map (which replaces both defaults):

```yaml
publish:
  - {signal: sensor/front_labels/detections, topic: /front_labels/detections,
     type: std_msgs/msg/Float64MultiArray}
  - {signal: sensor/front_labels/count, topic: /front_labels/count, type: std_msgs/msg/Int64}
```

### landing_target

A vision landing-target detector (an AprilTag or ArUco pad seen by a downward camera, IR-LOCK
style beacons) at the detector level, with PX4's meaning of MAVLink `LANDING_TARGET`
(common.xml: `angle_x`, `angle_y`, `distance`, `size_x`, `size_y`, position). No fiducial texture
is drawn or decoded: the target is a scene prim, seen when it is wholly in the field of view, in
range, facing the camera, not behind a collider, and large enough in pixels to decode.

```xml
<sensor name="pad_finder" type="landing_target">
  <pose>0 0 -0.05 0 1.5708 0</pose>   <!-- pitched 90°: x down, a downward camera -->
  <update_rate>20</update_rate>
  <target>landing_pad</target>
  <horizontal_fov>1.047</horizontal_fov>
  <max_range>20</max_range>
</sensor>
```

The open field's pad (`assets/worlds/open_field`, prim `landing_pad`, 2 m × 2 m) is the default
target.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `target` (text) | | `landing_pad` | the prim (name or path); its box's top face centre is the target point, its +z its normal |
| `target_size` | m | its box's x and y extents, else 1 | side of the marker |
| `horizontal_fov`, `vertical_fov` | rad | 1.047; from `width` × `height` | the camera's view (sensor x forward) |
| `width`, `height` | px | 640, 480 | the image, for pixels across the marker |
| `min_range`, `max_range` | m | 0.05, 30 | |
| `min_pixels` | px | 12 (illustrative) | pixels across the marker's smaller side under which it is never decoded |
| `max_incidence` | rad | 1.2217 (70°, illustrative) | the most the line of sight may be off the marker's normal |
| `pixel_noise` | px | 0.5 (illustrative) | 1 sigma of the corners' position |
| `occlusion_margin` | m | 0.05 | a collider this close in front of the target does not hide it |

**Signals:** `sensor/<n>/detected` (Bool), `angle_x`, `angle_y` (rad: the target's direction in
the optical frame, `atan2(x, z)` and `atan2(y, z)`, x right, y down), `distance` (m),
`position` (Vec3, m, in the sensor frame: x forward, y left, z up), `size_x`, `size_y` (rad, the
angular size of the marker's sides). Not detected: NaN in all but `detected`.

**Model.** With f = `width` / 2 / tan(`horizontal_fov` / 2) and the marker `pixels` = f · s ·
√cos θ / d across (s its smaller side, θ the angle between the line of sight and its normal, d the
distance), the chance of a detection per frame is (pixels − `min_pixels`) / `min_pixels`, clamped
to 0..1: none below `min_pixels`, always from twice that. Each angle gets Gaussian noise of
`pixel_noise` / f, and the distance, taken from the apparent size, of d · `pixel_noise` / pixels:
it grows with d² as a fiducial's range does. Occlusion is one exact ray (`ShapeCaster`) from the
target point towards the sensor; a collider within `occlusion_margin` of either end, or on the
sensor's own body, does not hide it. Deterministic (CPU, Philox). Under `ground_truth`: exact, detected
whenever the chance is above 0. `test_kinds_landing_target`: a pad at a known offset gives the
angles and distance of the geometry, the noise has the spread above within 5 %, the detection rate
the chance above, and a collider, the field of view and the range hide it.

On ROS by default as `/<n>/<quantity>` ("ROS topics of the built-in kinds"); a map for other
names:

```yaml
publish:
  - {signal: sensor/pad_finder/angle_x, topic: /landing_target/angle_x, type: std_msgs/msg/Float64}
  - {signal: sensor/pad_finder/position, topic: /landing_target/position,
     type: geometry_msgs/msg/Vector3Stamped, frame: pad_finder}
```

### line_sensor

An AGV guide sensor, a row of channels across the sensor's y that look down (the sensor's −z) for
tape on the floor: a reflective line-follower array or a magnetic tape sensor, at the detector
level (no light or magnetic field is modelled, only where tape is). Channel 0 is the leftmost
(+y).

```xml
<sensor name="guide" type="line_sensor">
  <pose>0.3 0 0.04 0 0 0</pose>
  <update_rate>100</update_rate>
  <channels>16</channels>
  <spacing>0.01</spacing>
</sensor>
```

Tape is either a scene prim with the word `tape` or `line` in its name (parameter `pattern`; its
mesh box: a thin box or quad on the floor; e.g. a URDF link `tape_aisle_1` with a
`<box size="10 0.05 0.001"/>` visual fixed to the world, or a USD `Xform "guide_tape"` with a
mesh), or a collider whose surface material (`SensorMaterialAPI_1`, URDF `<sensor_material>`) has
a 905 nm reflectivity of at least `tape_reflectivity` (at most, with `dark_tape`).

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `channels` | | 16 | |
| `spacing` | m | 0.01 (illustrative) | between channel centres; each channel is that wide |
| `samples_per_channel` | | 5 | rays across a channel |
| `max_range` | m | 0.1 | tape further below is not seen |
| `noise` | 1 | 0.02 (illustrative) | 1 sigma on each channel's activation |
| `threshold` | 1 | 0.2 | activation a channel counts from in `offset` and `width` |
| `pattern` (text) | | `tape\|line` | words that make a prim tape |
| `tape_reflectivity` | 1 | 0.8 | the collider route |
| `dark_tape` | | 0 | 1: tape darker than `tape_reflectivity` |

**Signals:** `sensor/<n>/channels` (ScalarArray of `channels`, 1: the share of each channel over
tape, 0..1), `offset` (m, the line's centre to the sensor's left, the activation-weighted mean of
the channels at or over `threshold`; NaN without tape), `width` (m, the tape under the array,
Σ activation × spacing: wider at a junction or a marker), `detected` (Bool).

**Model.** Each of a channel's rays counts when the first thing below it within `max_range`
(its own body's colliders aside) is tape (a tape prim's box no further than 1 mm past the floor, or a tape-bright collider). The
offset resolves a fraction of a channel: within one ray spacing (`spacing` /
`samples_per_channel`) of the tape's centre without noise (`test_kinds_line_sensor`, named tape
and bright collider). Deterministic (CPU, Philox); `ground_truth` drops the noise. Real arrays
reach about 1 mm: the Roboteq MGS1600GY magnetic guide sensor (datasheet v1.3, 2018, table 11)
gives 160 mm of capture width, 1 mm resolution (2 mm at most) from 16 sensors, at 100 Hz.

### tag_reader

A warehouse localisation reader for tags on the floor at the detector level: `mode` `rfid` reads
the nearest tag within a radius of the antenna; `mode` `qr` (QR or Data Matrix under a downward
camera) reads the tag nearest the optical axis in the camera's footprint (along the sensor's −z),
facing it and not hidden, with its offset and yaw.

```xml
<sensor name="floor_reader" type="tag_reader">
  <pose>0 0 0.05 0 0 0</pose>
  <update_rate>30</update_rate>
  <mode>qr</mode>
  <max_range>0.3</max_range>
</sensor>
```

Tags are scene prims whose name starts with `rfid_` or `qr_` (parameter `prefix`); a tag's point
is its box's top face centre (its origin without a mesh), its +z must face the reader, its x axis
gives the yaw. The number a name ends with is its `code` (`qr_0042` → 42). Tags on the floor of a
USD world:

```usda
def Xform "qr_0001" {
    double3 xformOp:translate = (2.0, 0.5, 0.0005)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    def Cube "visual" { double size = 1  float3 xformOp:scale = (0.05, 0.05, 0.001)
                        uniform token[] xformOpOrder = ["xformOp:scale"] }
}
def Xform "rfid_17" { double3 xformOp:translate = (4.0, 0.5, 0.0)
                      uniform token[] xformOpOrder = ["xformOp:translate"] }
```

or a URDF link `qr_0001` with a `<box size="0.05 0.05 0.001"/>` visual fixed to the world. The
sample floor `assets/worlds/tag_floor/tag_floor.urdf` has an 8 m guide tape along x with QR tags
`qr_0001`..`qr_0004` on it every 2 m from x = 1.5 m and RFID tags `rfid_1`..`rfid_4` between
them: `lagrange my_agv.urdf assets/worlds/tag_floor/tag_floor.urdf`.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `mode` (text) | | `rfid` | `rfid` or `qr` |
| `prefix` (text) | | `rfid_` or `qr_` | |
| `range` (rfid) | m | 0.1 (illustrative, an HF floor tag) | from the sensor's origin to the tag point |
| `min_range`, `max_range` (qr) | m | 0.01, 0.5 | depth below the reader |
| `horizontal_fov`, `vertical_fov` (qr) | rad | 0.8, as horizontal | the footprint across the sensor's x and y |
| `max_incidence` (qr) | rad | 0.79 (45°) | |
| `position_noise` (qr) | m | 5e-4 (illustrative) | 1 sigma on the offset's x and y |
| `yaw_noise` (qr) | rad | 0.005 (illustrative) | 1 sigma |

**Signals:** `sensor/<n>/detected` (Bool), `tag` (Int: the index in the tags of this mode sorted
by path, −1 for none), `code` (Int, −1 without a number), `tag_position` (Vec3, m, world_local:
the tag's surveyed position, for a localisation filter), `offset` (Vec3, m, the tag point in the
sensor frame), `yaw` (rad: the tag's x axis in the sensor's xy plane; NaN for RFID).

**Model.** RFID: the nearest tag in range, read every step (no anti-collision protocol or
missed reads; radio passes through colliders). QR: the footprint, the facing test and one
occlusion ray (as `landing_target`'s, 1 cm margin); Gaussian noise on the
offset and yaw. Deterministic (CPU, Philox); `ground_truth` drops the noise.
`test_kinds_tag_reader`: a QR tag at a known pose gives its offset and yaw exactly, its noise the
spread set, RFID reads within and not beyond its range, and a reader in its own housing reads while another
body's plate hides the tag.

On ROS by default as `/<n>/<quantity>` ("ROS topics of the built-in kinds"); a map for other
names:

```yaml
publish:
  - {signal: sensor/floor_reader/code, topic: /floor_reader/code, type: std_msgs/msg/Int64}
  - {signal: sensor/floor_reader/offset, topic: /floor_reader/offset,
     type: geometry_msgs/msg/Vector3Stamped, frame: floor_reader}
```

### wifi_scanner

What a Wi-Fi scan reports: the received signal strength (RSSI) of every access point the robot
hears (`modules/sensors/src/kinds/WifiScanner.cpp`, `test_kinds_wifi_scanner`).

```xml
<gazebo reference="base_link">
  <sensor name="wifi" type="wifi_scanner">
    <pose>0 0 0.4 0 0 0</pose>
    <update_rate>10</update_rate>
    <wifi_scanner>
      <access_points>0 0 2.5; 30 12 2.5</access_points>
      <scan_time>2</scan_time>
      <sensitivity>-82</sensitivity>
    </wifi_scanner>
  </sensor>
</gazebo>
```

Access points, in the order of the outputs:

1. every `RadioLinkAPI_1` whose receiver prim is on the sensor's body (its transmitter is the
   access point). It reads `link/<rx>/rx_power` from the comms domain, so walls and shadowing are
   the link's (comms.md). With the comms domain off (the input unconnected, or the ideal
   substitute's 1 W sentinel) the link's own attributes go through the model below;
2. the points of `access_points` (`x y z; x y z; ...`, m, in the region frame the run started in;
   they move with a floating-origin rebase as the scenery does);
3. every world prim or body whose name starts with `wifi_ap`, in load order.

The model for 2 and 3 (and 1 without comms) is comms.md's log-distance budget: Friis free space
to `reference_distance`, then `(d/d0)^path_loss_exponent`, and `wall_loss_db` per wall the
straight line crosses (colliders of bodies that never move; the ground, the scanner's body and
the access points' bodies left out).

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `access_points` | m | none | `x y z; ...` region-local |
| `scan_time` | s | 2 | a new scan every so often; between scans the outputs hold. Illustrative: a scan dwells some 60 ms on each channel when active and 111 ms when passive (Linux mac80211 `scan.c`, IEEE80211_PROBE_DELAY + IEEE80211_CHANNEL_TIME and IEEE80211_PASSIVE_CHANNEL_TIME; firmware scans set their own) over 13 channels at 2.4 GHz and some 25 at 5 GHz: one to four seconds |
| `sensitivity` | dBm | −82 | weaker access points are not listed: the OFDM PHY's minimum input sensitivity at 6 Mb/s, 20 MHz (IEEE 802.11-2020 Table 17-18) |
| `noise` | dB | 0 | 1σ per reading, before rounding (the comms domain adds its own shadowing) |
| `tx_power` | W | 0.1 | 20 dBm |
| `frequency` | Hz | 2.437e9 | channel 6 |
| `path_loss_exponent` | 1 | 3 | as the sample link |
| `reference_distance` | m | 1 | |
| `wall_loss_db` | dB | 10 | per wall, illustrative (as the sample link) |

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/rssi` | ScalarArray(N) | dBm (unit `one`) | per access point, rounded to whole dBm as scans report it; NaN when not heard |
| `sensor/<n>/power` | ScalarArray(N) | W | the same as power, `1 mW · 10^(rssi/10)` |
| `sensor/<n>/count` | Int | 1 | access points heard |
| `sensor/<n>/valid` | Bool | 1 | at least one heard |

N is the number of access points (at least 1). Ground truth: the model's exact level, no noise,
no rounding, no sensitivity cut. Checked against the closed form at 10 m (−50 dBm), with a wall,
the sensitivity cut, a link's −40 dBm and the scan interval. On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/wifi/rssi, topic: /wifi/rssi, type: std_msgs/msg/Float64MultiArray}
```

### uwb

Ultra-wideband two-way ranging from a tag on the robot to fixed anchors, as a Qorvo (Decawave)
DW1000 or DW3000 gives it (`modules/sensors/src/kinds/Uwb.cpp`, `test_kinds_uwb`).

```xml
<gazebo reference="base_link">
  <sensor name="tag" type="uwb">
    <pose>0 0 0.3 0 0 0</pose>
    <update_rate>10</update_rate>
    <uwb>
      <anchors>0 0 2.5; 20 0 2.5; 20 15 2.5; 0 15 2.5</anchors>
      <range_noise>0.1</range_noise>
    </uwb>
  </sensor>
</gazebo>
```

Anchors, in the order of the outputs: the points of `anchors` (`x y z; ...`, m, in the region frame
the run started in; they move with a floating-origin rebase as the scenery does),
then every world prim or body whose name starts with `uwb_anchor` (in prim order, then body
order; a body is taken at its origin). A run with no anchor is refused.

Each range is the straight distance from the tag (the sensor's mount) to the anchor. When that
line passes through colliders (all of them but the tag's body and the anchors' own bodies), the
first path is slower inside them: the range grows by `(√εr − 1)` times the length inside, the
excess delay of a pulse through a dielectric slab, and the flag `los` is 0; such a range also gets
`nlos_noise`. Material shorter than 1 cm along the line (an anchor on a wall) does not count.
Beyond `max_range` the anchor drops out.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `anchors` | m | none | `x y z; ...` region-local |
| `range_noise` | m | 0.1 | 1σ with line of sight: "a precision of 10 cm" (DW1000 Datasheet Version 2.10, p. 1; DW3000 Datasheet Version 1.1: "an accuracy of 10 cm"), on the safe side: the DW1000's figure 11 spreads about ±8 cm |
| `nlos_permittivity` | 1 | 5.24 | εr of what blocks the line: concrete, 1–100 GHz (ITU-R P.2040-1 Table 3; UWB channels 5 and 9 are 6.5 and 8 GHz) |
| `nlos_noise` | m | 0.3 | extra 1σ without line of sight, illustrative |
| `max_range` | m | 50 | illustrative for indoor anchors at 6.8 Mb/s (DW1000 datasheet figure 31: some 50–110 m in open line of sight) |

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/ranges` | ScalarArray(N) | m | per anchor; NaN beyond range |
| `sensor/<n>/los` | ScalarArray(N) | 1 | 1 line of sight, 0 through material, NaN beyond range |
| `sensor/<n>/count` | Int | 1 | anchors in range |
| `sensor/<n>/valid` | Bool | 1 | at least one |

Ground truth: the straight distance, no delay, no noise (`los` still tells whether the line is
clear). Checked: exact distances, σ = 0.1 m over 2000 samples, 0.2 m of concrete adding
`(√5.24 − 1)·0.2` m, dropouts and anchors named in the world. On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/tag/ranges, topic: /uwb/ranges, type: std_msgs/msg/Float64MultiArray}
```

### particle_sensor

An optical particulate-matter sensor such as the Sensirion SPS30, reading the dust the weather
domain puts in the air (`modules/sensors/src/kinds/ParticleSensor.cpp`,
`test_kinds_particle_sensor`). A gas sensor has no counterpart yet: no domain carries gas
concentrations.

```xml
<gazebo reference="base_link">
  <sensor name="pm" type="particle_sensor">
    <update_rate>1</update_rate>
  </sensor>
</gazebo>
```

The air holds two populations of grains, each of one size: the weather's own dust
(`env/dust/concentration` and `env/dust/median_diameter`, written by the weather domain from its
`dust_concentration` and `dust_median_diameter`) and the dust rotor outwash lifts off loose ground
(`env/dust/downwash`, of `downwash_diameter`). PMx counts the grains between 0.3 µm and x. The
reading follows the air with a first-order lag, `y += (u − y)(1 − e^(−dt/τ))` (the first sample reads the air as it is), then gets the
datasheet's precision as Gaussian noise and is held within 0–1000 µg/m³. Without the weather
domain (its inputs unconnected) every class is NaN and `valid` 0; without lifted dust that part
is 0.

Sensirion SPS30 datasheet (Version 2.0 – D1 – June 2023, table 1): sizes 0.3–10 µm, range
0–1000 µg/m³, sampling interval 1 s; precision PM1 and PM2.5 ±(5 µg/m³ + 5 % of the reading) up
to 100 µg/m³ (±10 µg/m³ in v1.0 of March 2020) and ±10 % above, PM4 and PM10 ±25 µg/m³ and
±25 %, taken here as 1σ of each reading. Sensirion's precision is the variation between devices
on averages of at least 150 s, so this is a stand-in: a real part's error is more an offset that
stays than noise.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `time_constant` | s | 8 | the lag: the SPS30's typical start-up time at 200–3000 particles/cm³ (table 1); illustrative as a lag, since Sensirion gives no step response |
| `noise_scale` | 1 | 1 | times the datasheet precision (0: no noise) |
| `downwash_diameter` | m | 1e-4 | grains lifted by outwash (the weather's `ground_dust_diameter` default) |

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/pm1`, `pm2_5`, `pm4`, `pm10` | Scalar | kg/m³ | mass concentration; µg/m³ = kg/m³ × 1e9 |
| `sensor/<n>/valid` | Bool | 1 | the weather domain is there |

Ground truth: the air's PMx exactly, no lag, no noise, no limit. Checked: the classes a grain
size falls in, a step against `1 − e^(−t/τ)`, the precision at 50 µg/m³ over 3000 samples, the
1000 µg/m³ limit, NaN without weather. Run with `--weather ...,dust_concentration=2e-7` (200
µg/m³). On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/pm/pm2_5, topic: /pm/pm2_5, type: std_msgs/msg/Float64}
```

### proximity

A proximity skin of a collaborative robot: an array of short-range time-of-flight elements over
a field of view (`modules/sensors/src/kinds/Proximity.cpp`, `test_kinds_proximity`). Each element
casts one ray along its direction against every collider except the sensor's own body and reads
the distance to the first. A different kind from the `tof` and `ultrasonic` plugins: many
elements, and the nearest of them as one signal for a speed-and-separation monitor.

```xml
<gazebo reference="forearm_link">
  <sensor name="skin" type="proximity">
    <pose>0.1 0 0.05 0 0 0</pose>
    <update_rate>50</update_rate>
    <proximity>
      <rows>2</rows><columns>4</columns>
      <fov>1.2</fov><vertical_fov>0.4</vertical_fov>
      <max_range>0.3</max_range>
    </proximity>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `rows`, `columns` | 1 | 1, 1 | elements, at most 4096 |
| `fov` | rad | 0.44 | spread of the columns about the sensor's +x (25°: ST VL53L0X DocID029104 Rev 2, §5.1) |
| `vertical_fov` | rad | `fov` | spread of the rows |
| `min_range`, `max_range` | m | 0, 0.3 | outside them an element reads +inf |
| `noise` | m | 0.012 | 1σ per element: the VL53L0X's 4 % standard deviation (table 12, 33 ms, white target indoors) at the 0.3 m reach |

Element (row r, column c) looks at azimuth `−fov/2 + (c + ½)·fov/columns` (columns from the
sensor's right to its left) and elevation `−vertical_fov/2 + (r + ½)·vertical_fov/rows` (rows from
bottom to top), row-major in the outputs.

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/distances` | ScalarArray(rows·columns) | m | per element; +inf when nothing in range |
| `sensor/<n>/min_distance` | Scalar | m | the nearest; +inf for none |
| `sensor/<n>/detected` | Bool | 1 | something in range |

Ground truth: the ray distances exactly. Checked against `d / (cos el · cos az)` to a wall, the
range cut and σ = 1 mm. On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/skin/min_distance, topic: /skin/min_distance, type: std_msgs/msg/Float64}
```

### gripper_sensor

What a gripper's controller knows about its grasp: the opening, the motor current, whether it
holds something and whether that slips (`modules/sensors/src/kinds/GripperSensor.cpp`,
`test_kinds_gripper_sensor`). Mount it on the hand; the fingers are joints.

```xml
<gazebo reference="hand">
  <sensor name="grip" type="gripper_sensor">
    <update_rate>100</update_rate>
    <gripper_sensor>
      <fingers>finger_l_joint finger_r_joint</fingers>
      <width_scale>1</width_scale>
      <torque_constant>10</torque_constant>
    </gripper_sensor>
  </sensor>
</gazebo>
```

The model:

- **Opening**: `width_offset + width_scale · Σ joint/<f>/position` over the finger joints (two
  prismatic fingers moving apart from 0: scale 1, offset 0; revolute fingers: scale ≈ finger
  length).
- **Current**: Σ |`motor/<m>/current_q`| over the motors `motors` names (the electrical
  domain's torque current); without `motors`, or with the electrical domain off, the proxy
  Σ |`joint/<f>/torque`| / `torque_constant` (0 for a joint nothing drives).
- **Contact**: a finger touches when |`body/<finger>/contact_force`| (its joint's child body)
  exceeds `contact_threshold`. An object is held when every finger touches and the opening is
  above `closed_width` (fingers pressing on each other are not an object). `grip_force` is the
  smallest finger's contact force while all touch, else 0.
- **Slip**: the object is the body `object` names, else a moving body outside the gripper's body
  tree within a finger's colliders' reach plus `contact_margin`. Its velocity at the finger less
  the finger's, with the part along the contact force removed, is the slip speed; `slip` is 1
  while an object is held and that exceeds `slip_threshold`.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `fingers` | text | required | finger joint names |
| `motors` | text | none | motor names for `motor/<m>/current_q` |
| `width_scale`, `width_offset` | m per joint unit, m | 1, 0 | |
| `torque_constant` | N·m/A or N/A | 1 | for the current proxy |
| `contact_threshold` | N | 0.5 | illustrative |
| `closed_width` | m | 0.001 | |
| `slip_threshold` | m/s | 0.005 | illustrative |
| `contact_margin` | m | 0.005 | |
| `object` | text | none | the held body by name |
| `width_noise`, `current_noise` | m, A | 0, 0 | 1σ |

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/width` | Scalar | m | opening |
| `sensor/<n>/current` | Scalar | A | motor current, or its proxy |
| `sensor/<n>/grip_force` | Scalar | N | |
| `sensor/<n>/object_detected` | Bool | 1 | |
| `sensor/<n>/slip` | Bool | 1 | |
| `sensor/<n>/slip_speed` | Scalar | m/s | 0 when nothing is held |

Ground truth: no noise. Checked: a held 4 cm cube (opening, current, grip), slip across the force
and none along it, closed or open fingers holding nothing, the motors' currents. On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/grip/object_detected, topic: /gripper/object_detected, type: std_msgs/msg/Bool}
```

### tpms

A tyre-pressure monitoring system: a pressure and temperature sensor in every tyre of a vehicle
(`modules/sensors/src/kinds/Tpms.cpp`, `test_kinds_tpms`). Its wheels are the `WheelAPI_1`
joints in the sensor's body tree (or those `wheels` names), in load order.

```xml
<gazebo reference="base_link">
  <sensor name="tpms" type="tpms">
    <update_rate>1</update_rate>
    <tpms><fill_pressure>230000</fill_pressure></tpms>
  </sensor>
</gazebo>
```

The tyre's air is an ideal gas at nearly constant volume (Gay-Lussac): its absolute pressure is
the cold fill's, `fill_pressure + 101325 Pa` at `fill_temperature`, times `T / T_fill`, with `T`
the ambient air's (`env/air_temperature` from the weather domain, else the parameter
`temperature`; with neither every reading is NaN and `valid` 0). The vehicle_dynamics domain has
no tyre temperature, so the heating of a running tyre is **not** modelled: the readings follow
the weather only (a cold morning's drop, a leak). A leak lowers the fill by `leak_rate`. The
reading is gauge pressure (absolute less `env/pressure`, else 101325 Pa), and `warning` comes on
as FMVSS No. 138 (49 CFR 571.138 S4.2) asks, when a tyre is 25 % or more below the placard.

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `fill_pressure` | Pa (gauge) | 230000 | the placard cold pressure, a typical car |
| `fill_temperature` | K | 293.15 | where it was filled |
| `temperature` | K | none | the air's when the weather domain is off |
| `leak_rate` | Pa/s | 0 | every tyre |
| `noise` | Pa | 0 | 1σ |
| `wheels` | text | all | wheel joint names |

| Signal | Type | Unit | Meaning |
|---|---|---|---|
| `sensor/<n>/pressure` | ScalarArray(N) | Pa | gauge, per wheel |
| `sensor/<n>/temperature` | ScalarArray(N) | K | per wheel |
| `sensor/<n>/warning` | Bool | 1 | FMVSS No. 138 low pressure |
| `sensor/<n>/valid` | Bool | 1 | an air temperature is known |

Ground truth: no noise. Checked: 230 kPa filled at 20 °C reads 207.4 kPa at 0 °C, a 1 kPa/s leak
warns after 57.5 s, NaN without a temperature. On ROS by default as `/<n>/<quantity>` ("ROS topics of the
built-in kinds"); a map for other names:

```yaml
publish:
  - {signal: sensor/tpms/pressure, topic: /tpms/pressure, type: std_msgs/msg/Float64MultiArray}
```


### sonar

An underwater sonar: by default a multibeam imaging sonar's fan, after the Blueprint Subsea
Oculus M750d in its 750 kHz mode (Blueprint Subsea, Oculus M-series specifications page,
blueprintsubsea.com/oculus/oculus-m-series, read 2026-11-12: 0.1 to 120 m, 130° × 20°, 512
beams, 4 mm range resolution, up to 40 Hz); with `beams` 1 a single-beam echosounder or altimeter
such as the Blue Robotics Ping2 (store page "Ping Sonar Altimeter and Echosounder", technical
details, read 2026-11-12: 115 kHz, a 25° beam at −3 dB, 0.3 m to 100 m typical, range resolution
0.5 % of range). The defaults are the Oculus's: for a Ping2 give `beams` 1, `frequency` 115000
and `range` 0.3 to 100 m, as the example does. It hears only under the surface of a body of water
(`WaterAPI_1`, as the hydrodynamics domain and the lidars read it).

```xml
<sensor name="fls" type="sonar">            <!-- forward-looking, +x ahead, z up -->
  <pose>0.3 0 -0.1 0 0.35 0</pose>
  <update_rate>15</update_rate>
  <sonar><beams>256</beams><image_bins>200</image_bins><water_temperature>12</water_temperature></sonar>
</sensor>
<sensor name="altimeter" type="sonar">      <!-- Ping2 looking down -->
  <pose>0 0 -0.15 0 1.5708 0</pose>
  <update_rate>10</update_rate>
  <sonar><beams>1</beams><beam_width>0.436</beam_width><frequency>115000</frequency><range><min>0.3</min><max>100</max></range></sonar>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `beams` | | 512 | across `horizontal_fov`, beam 0 on the left (+y); 1: a single beam |
| `horizontal_fov`, `vertical_fov` | rad | 2.269 (130°), 0.349 (20°) | the fan |
| `beam_width` | rad | 0.436 (25°) | full cone of a single beam |
| `vertical_rays` | | 5 | rays across `vertical_fov` per beam (a single beam: a grid of n × n in its cone) |
| `range/min`, `range/max` | m | 0.1, 120 | |
| `resolution` | m | 0.004 | range quantisation |
| `noise/stddev` | m | `resolution` | 1σ on the range |
| `sound_speed` | m/s | 1500 | what the device converts time of flight with (Urick, *Principles of Underwater Sound*, 1983: the conventional round value) |
| `water_temperature`, `salinity`, `ph` | °C, PSU, 1 | 10, 35, 8 (illustrative) | the water: absorption, and with a temperature its own sound speed; −2 to 40 °C (not K) and 0 to 45 PSU, else refused |
| `frequency` | Hz | 750e3 | for the absorption |
| `backscatter` | dB | −27 | Lambert's 10·log10 μ (Mackenzie, JASA 33(11), 1961), every surface |
| `intensity_noise` | dB | 1 (illustrative) | 1σ on the echo level |
| `min_level` | dB | −180 (illustrative) | weaker echoes are not detected |
| `image_bins` | | 0 | range bins of the polar image; 0: no image |

Signals: `ranges` (ScalarArray[beams], m; +inf without an echo), `intensities` (ScalarArray[beams],
dB re a perfect reflector at 1 m; NaN without an echo), `image` (ScalarArray[beams × image_bins],
row per beam, dB above `min_level` at each echo's bin, 0 elsewhere; only with `image_bins`),
`depth` (m below the surface; NaN in air), `sound_speed` (m/s, the water's), `ping_interval` (s),
`valid` (Bool: under water).

Model: each beam's first echo among its rays: the colliders at their bodies' poses (the CPU
`ShapeCaster`, the sensor's own body skipped), the water's flat surface (its mean level: waves
are not seen) and the box's bottom; sound leaving through a side of the box is lost. The water's
sound speed is Mackenzie's nine-term equation (JASA 70(3), 1981) when `water_temperature` is
given, else `sound_speed`; the device reads range = true range × `sound_speed` / c, so a sonar set
to 1500 m/s in 1490 m/s water reads 0.7 % long, as real ones do. A ping waits for the farthest
echo: the kind steps no faster than 1 / (2·range/max / sound_speed) (0.16 s at 120 m). Echo
level −40·log10 R − 2·α·R + 10·log10(μ·cos²θ): two-way spherical spreading, absorption α by
Ainslie and McColm (JASA 103(3), 1998; ≈ 0.24 dB/m at 750 kHz, 10 °C) and Lambert backscatter at
incidence θ (from two more rays 1 mrad apart on a collider). No beam pattern, side lobes,
multipath, speckle or acoustic shadow texture; the insonified area is not in the level. Above water
everything reads nothing and `valid` 0. Check: `test_kinds_sonar` (Mackenzie's value, ≈ 1 dB/km at
10 kHz, the bottom and the surface at their depths, the sound speed's bias, a fan on a wall at
45°: √2 in range and 12.04 + 3.01 dB weaker, the image bin, the noise and 4 mm steps).

ROS by default: a fan as `/fls/scan` (`sensor_msgs/LaserScan` with the intensities, beam 0 the
left one, so the angles run from +65° to −65°), one beam as `/<n>/range` (`sensor_msgs/Range`,
ULTRASOUND), and the rest on `/<n>/<quantity>`. By hand: `{signal: sensor/fls/ranges, topic:
/fls/scan, type: sensor_msgs/msg/LaserScan, frame: fls, params: {angle_min: 1.1345, angle_max:
-1.1345, range_min: 0.1, range_max: 120.0}}`.

### adsb_receiver

A 1090 MHz ADS-B receiver (1090ES, RTCA DO-260B) that hears the other aircraft of the run: a
detect-and-avoid input for a drone, or a ground station's traffic display. An aircraft is another
robot with `RotorAPI_1` rotors (Lagrange has no fixed-wing aircraft yet), or with `aircraft: all`
every other robot whose root body is not kinematic (traffic agents and lamps never count). It reads
their root bodies' `body/<b>/pose` and `twist` from the bus, nothing else.

```xml
<sensor name="adsb" type="adsb_receiver">
  <update_rate>10</update_rate>
  <adsb_receiver><range><max>50000</max></range></adsb_receiver>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `aircraft` (text) | | `rotors` | or `all` |
| `range/max` | m | 100e3 (illustrative) | |
| `horizon` | | 1 | 0: no radio horizon |
| `antenna_height` | m | 2 (illustrative) | the receiver's antenna above its mount, for the horizon |
| `position_noise` | m | 3 (illustrative) | 1σ per axis of the sender's GNSS position |

Signals: `aircraft` (ScalarArray, 10 per aircraft in robot order: robot index (the ICAO address
stands in), heard (1/0), latitude (rad), longitude (rad), height (m, ellipsoidal) when the site's
geo origin is known (`geodetic` 1), else x, y, z in world ENU (m); east, north, up velocity (m/s);
age of the last message (s); distance (m)); `count` (Int, aircraft heard); `geodetic` (Bool).

Model: each aircraft squitters its airborne position and velocity every 0.4 to 0.6 s at random
(ICAO Annex 10 Vol. IV §3.1.2.8.6.4.2 and .5, which DO-260B implements; about 2 Hz; the receiver should run at 10 Hz or more to see them); a message
is heard while the aircraft is within `range/max` and the straight line is above the radio horizon
√(2kR·h₁) + √(2kR·h₂), k = 4/3 (Bean and Dutton, *Radio Meteorology*, 1966), heights the ENU up
coordinates (h₁ plus `antenna_height`). The message carries the sender's noisy position, height
in 25 ft steps, east and north velocity in 1 kt steps and vertical rate in 64 ft/min steps (the Q-bit
altitude of Annex 10 Vol. IV §3.1.2.6.5.4 and the subtype 1 velocity, as J. Sun's *The 1090
Megahertz Riddle*, 2nd ed., gives it); latitude and longitude are not CPR-encoded (their
≈ 5 m step is below the noise). Out of reach the row reads NaN and heard 0. No garbling, Mode S
interference, barometric altitude, identification or NIC/NACp fields. `ground_truth`: every step,
exact. Check: `test_kinds_adsb_receiver` (which robots are aircraft, exact local and geodetic
positions, range and horizon at 30 and 50 km, 17 to 25 messages in 10 s, the quantisation).

ROS by default: `/adsb/aircraft` (`std_msgs/Float64MultiArray`, dimensions `rows` × `columns` of
10; no standard ROS message carries ADS-B), `/adsb/count`, `/adsb/geodetic`.

### v2x

A V2X on-board unit (DSRC IEEE 802.11p or C-V2X PC5) receiving the SAE J2735 messages of the
traffic domain's world: each traffic agent's Basic Safety Message and each signalised approach's
Signal Phase and Timing, sent at 10 Hz (SAE J2945/1) by a roadside unit at its signal head.

```xml
<sensor name="obu" type="v2x">
  <update_rate>10</update_rate>
  <v2x><loss_distance>300</loss_distance></v2x>
</sensor>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `loss_distance` | m | 300 (illustrative) | where half the messages are lost |
| `loss_width` | m | 50 (illustrative) | how fast the loss grows |
| `range/max` | m | 1000 | nothing beyond |
| `timeout` | s | 0.5 | a sender counts as heard while its last message is this recent |

Signals: `bsm` (ScalarArray, 10 per traffic agent in `traffic/agents` order: index, valid, class
(0 car, 1 pedestrian), x, y (m, world_local), yaw (rad), speed (m/s), length, width (m), age (s)),
`spat` (ScalarArray, 7 per light in `traffic/lights` order: index, valid, road id, state (0 red,
1 yellow, 2 green), x, y of the signal head (m), age (s)), `bsm_count`, `spat_count` (Int: valid
rows), `valid` (Bool: the traffic domain's signals are there).

Model: inputs `traffic/agents` and `traffic/lights` only (optional: without the traffic domain
everything reads NaN and `valid` 0). Their lengths are found from the world as the traffic domain
finds them (one agent per kinematic body `traffic_vehicle_<i>`/`traffic_pedestrian_<i>`, one light
per signal head, the kinematic lamp bodies `traffic_lamp_<lane>_red` in lane order), so a
different traffic layout would stop the run at wiring, not misread. Each step every sender in range
is received with probability 1/(1 + exp((d − loss_distance)/loss_width)): illustrative, of the shape
of measured DSRC packet delivery ratios, not fitted (Meireles et al., IEEE VNC 2010: over 80 %
at 400 m with line of sight in their suburban run, to zero at 500 m with vehicles in the way; the
defaults, 12 % at 400 m, sit near the obstructed case);
no walls, no channel congestion, no latency. A row keeps its last message; `age` grows until the
next. Run it at 10 Hz (one draw per step). `ground_truth`: every message in range. Check:
`test_kinds_v2x` (exact copies and the heads' positions, ½ received at `loss_distance` over 2000
steps, range, no traffic).

ROS by default: `/<n>/spat` and `/<n>/bsm` (`std_msgs/Float64MultiArray`, rows of 7 and 10),
the counts and `valid`; a map renames them, e.g. `{signal: sensor/obu/spat, topic: /v2x/spat,
type: std_msgs/msg/Float64MultiArray}`.

### vehicle_can

What a production car's own sensors put on its CAN bus, for a `VehicleAPI_1` robot (the sensor's
robot must have a `VehicleAPI_1` chassis, or the run refuses to start): ABS wheel speeds, the
steering wheel angle, the ESP cluster's yaw rate and accelerations, and the gear.

```xml
<gazebo reference="base_link">
  <sensor name="can" type="vehicle_can">
    <update_rate>100</update_rate>
    <vehicle_can><teeth>48</teeth><steering_ratio>15.5</steering_ratio></vehicle_can>
  </sensor>
</gazebo>
```

| Parameter | Unit | Default | Meaning |
|---|---|---|---|
| `teeth` | | 48 (illustrative; 43 to 100 are common) | teeth of each ABS tone ring |
| `steering_ratio` | | 15.5 (illustrative) | steering wheel to road wheel |
| `steering_resolution` | rad | 0.00175 (0.1°, illustrative) | |
| `yaw_rate_noise` | rad/s | 0.00175 (0.1°/s, illustrative) | 1σ |
| `acceleration_noise` | m/s² | 0.03 (illustrative) | 1σ |

Signals: `wheel_speeds` (ScalarArray, m/s, unsigned, one per `WheelAPI_1` wheel in schema order),
`speed` (m/s, their mean), `steering_wheel_angle` (rad, positive left), `yaw_rate` (rad/s),
`longitudinal_acceleration`, `lateral_acceleration` (m/s², the specific force in the sensor's
frame: gravity shows when the car pitches or rolls, as it does on a real ESP sensor), `gear`
(`vehicle/<v>/gear`; NaN without the vehicle dynamics domain), `valid`.

Model: wheel speed from each wheel joint's `joint/<j>/position` as an ABS controller measures
it, by the time between tooth edges: the edges passed in a step (each edge's moment interpolated
within the step) × (2π/teeth)·r over the time since the edge before them; with no edge in a step
the estimate falls as one tooth over the time since the last, so a stopped wheel reads a speed
that decays toward zero (no direction, as passive sensors give none). `ground_truth` uses
|ω|·r. Steering: each steered wheel's angle (its `wheel:steerJoint`
position with its sign, else `wheel/<j>/steer_angle`) turned back through Ackermann's
tan δ = L·tan δw / (L + y·tan δw) (the inverse of vehicle_dynamics' `ackermannAngle`), averaged,
times the ratio, quantised. Yaw rate: the chassis's angular velocity about the sensor's z.
Accelerations: the sensor point's velocity differenced over the step, less gravity, in the
sensor's frame (the first step reads as at rest). No CAN timing, message counters or checksums.
Check: `test_kinds_vehicle_can` (10 m/s straight, v·ω = 5 m/s² on a circle, the steering wheel
angle of a 0.1 rad bicycle angle, edge timing exact at a steady 33 rad/s and under one tooth per
second a second after the wheel stops, the gear, refused without `VehicleAPI_1`).

ROS by default: `/can/yaw_rate`, `/can/speed`, ... (`std_msgs/Float64`) and `/can/wheel_speeds`
(`std_msgs/Float64MultiArray`), with the sensor named `can`.
