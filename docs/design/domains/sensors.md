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
| `raster` | Rasterised colour, depth, instance segmentation through the shared renderer (`gpu/kernels/camera/`) |
| `sensor_model` | Adds exposure (scene radiance → electrons with exposure time and gain), photon shot noise and read noise, quantisation to bit depth, lens distortion (plumb-bob or equidistant), vignetting, motion blur from per-pixel velocity, rolling shutter by rendering row bands at different times |
| `rt` (Tier 2) | Ray-traced shadows and reflections for the above |

**Radar**: ray-launched returns with radar cross-section from `sensmat:radarCrossSectionDensity`,
radial velocity from relative motion, a range–Doppler–azimuth detection list after a
constant-false-alarm threshold; rain attenuation from weather. Raw ADC cubes are not produced.

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
against N²/τ + K²τ/3 (IEEE Std 952): within 1 % up to τ = 100 s and 4.3 % at 1000 s. SDFormat
does not define whether `<dynamic_bias_stddev>` is the stationary deviation or the driving
noise; here it is the stationary deviation. TODO(verify): Gazebo's reading of it.

**GNSS**: position from the body pose converted through the local frame to WGS84
(`core/world/coords/`), Gaussian horizontal and vertical error with first-order time
correlation, velocity error, fix-loss when sky view is blocked (ray casts to a fixed set of
sky directions as a stand-in for satellites), optional `urban` multipath bias.

**Magnetometer**: Earth field (constant vector given by the scene, or a world magnetic model
evaluated at the geo-origin; the model coefficients are a data file with an epoch), rotated into
the body frame, soft- and hard-iron distortion, noise, plus `emi/{p}/b_field`.

**Timing** (all sensors, from `SensorAPI`): a sample is taken at `k / rate` plus jitter
(Gaussian, truncated), and delivered `latency` later. The message timestamp is the sampling
time. Sampling instants are aligned to the scheduler tick that follows them; the alignment error
is at most one base tick and is reported in the sensor's metadata signal.

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
Rays are cast exactly against every collision shape on the CPU (Tier 0 without Embree yet), or
on Tier 1 by `gpu/kernels/lidar/bvh_traverse.slang`: the same closed-form tests in float, one
thread per beam, through a bounding-volume hierarchy built on the CPU each scan (median split,
uploaded). Shapes are taken relative to the sensor and combined with its orientation in double
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
| Radar | Embree | compute BVH | ray query |
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
