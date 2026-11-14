# Sensors

Every sensor Lagrange simulates, how to put one on your robot, how to read it, and how to make it
fail. The models themselves (equations, parameters with units and defaults, signals, references
and checks) are in the design notes: [sensors.md](../design/domains/sensors.md), and for users
[the sensors physics page](../physics/sensors.md).

There are two families:

- **Built-in types**: IMU, lidar, camera, thermal camera, radar, GNSS, magnetometer and barometer.
  Each has its own USD schema (`ImuAPI_1`, `GnssAPI_1`, ...) and its own ROS 2 topics.
- **Built-in kinds**: 37 more sensors, from bumpers to sonar, simulated by Lagrange itself with no
  plugin. A kind is a custom sensor (`<sensor type="KIND">` in URDF or SDFormat, USD
  `CustomSensorAPI_1`) whose kind Lagrange knows. Each writes `sensor/<name>/<quantity>` signals.
  Its parameters, signals and model are in [sensors.md §9](../design/domains/sensors.md#9-built-in-custom-sensor-kinds),
  one section per kind (linked below).

A `<sensor type>` that is neither (`gas`, say) is a custom sensor for a [plugin](writing-a-plugin.md)
to implement, like the sample `tof` and `ultrasonic` (`lagrange_tof`) and `thermistor` and
`ir_thermometer` (`lagrange_thermal_probe`) plugins.

## Every sensor, by robot

"URDF type" is what goes in `<sensor type="...">` inside a `<gazebo reference="LINK">` block. A
sensor can sit on any robot; the grouping only says where each is usually found.

### On every robot

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| IMU | specific force and angular rate, with noise, bias and random walk | `imu` | [physics](../physics/sensors.md#imu) |
| Lidar, 2D or 3D | ranges and intensities; channel tables, flash and Livox-style rosette patterns; motion distortion | `ray`, `gpu_ray`, `lidar`, `gpu_lidar` | [physics](../physics/sensors.md#lidar), [patterns](../design/domains/sensors.md#lidar-scan-patterns-as-built) |
| Camera | colour, depth and instance images; lens, noise, exposure, rolling shutter; fisheye up to 360° | `camera`, `depth`, `depth_camera`, `rgbd_camera` | [physics](../physics/sensors.md#camera), [fisheye](../design/domains/sensors.md#fisheye-cameras-as-built) |
| `battery_monitor` | a smart battery's fuel gauge: voltage, current, charge, temperature | `battery_monitor` | [§9](../design/domains/sensors.md#battery_monitor) |
| `current_sensor`, `voltage_sensor` | any current or voltage signal of the run, through an ADC | `current_sensor`, `voltage_sensor` | [§9](../design/domains/sensors.md#current_sensor-and-voltage_sensor) |
| `rangefinder` | one laser beam's range (TFmini-S style) | `rangefinder` | [§9](../design/domains/sensors.md#rangefinder) |
| `wifi_scanner` | the RSSI of every access point heard | `wifi_scanner` | [§9](../design/domains/sensors.md#wifi_scanner) |
| `uwb` | ultra-wideband ranges to fixed anchors, with line of sight | `uwb` | [§9](../design/domains/sensors.md#uwb) |

### Drones

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| GNSS | latitude, longitude, height and velocity through the site's geo origin | USD `GnssAPI_1` or an overlay | [physics](../physics/sensors.md#gnss-magnetometer-barometer) |
| Magnetometer | the Earth's field (World Magnetic Model 2025) plus EMI | USD `MagnetometerAPI_1` or an overlay | [physics](../physics/sensors.md#gnss-magnetometer-barometer) |
| Barometer | static pressure and temperature (ICAO standard atmosphere) | USD `BarometerAPI_1` or an overlay | [physics](../physics/sensors.md#gnss-magnetometer-barometer) |
| `optical_flow` | angular flow of the ground under it (PMW3901, PX4Flow) | `optical_flow` | [§9](../design/domains/sensors.md#optical_flow) |
| `airspeed` | pitot-static differential pressure and airspeed | `airspeed` | [§9](../design/domains/sensors.md#airspeed) |
| `esc_telemetry` | each ESC's motor speed, current, voltage and temperature | `esc_telemetry` | [§9](../design/domains/sensors.md#esc_telemetry) |
| `landing_target` | a landing pad's direction and distance, as MAVLink `LANDING_TARGET` | `landing_target` | [§9](../design/domains/sensors.md#landing_target) |
| `adsb_receiver` | other aircraft of the run, as ADS-B reports | `adsb_receiver` | [§9](../design/domains/sensors.md#adsb_receiver) |
| `tension_sensor` | a tether's tension through a load cell | `tension_sensor` | [§9](../design/domains/sensors.md#tension_sensor) |

### Cars

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| Radar | detections with range, Doppler velocity, RCS and SNR | USD `RadarAPI_1` | [physics](../physics/sensors.md#radar) |
| `imaging_radar` | a 4D imaging radar: range, azimuth, elevation and Doppler per detection, angle cells from its virtual array, Doppler folding, ground ghosts | `imaging_radar` | [§2](../design/domains/sensors.md#4d-imaging-radar-as-built) |
| `vehicle_can` | the car's own CAN bus: wheel speeds, steering angle, yaw rate, gear | `vehicle_can` | [§9](../design/domains/sensors.md#vehicle_can) |
| `tpms` | pressure and temperature in every tyre, with the low-pressure warning | `tpms` | [§9](../design/domains/sensors.md#tpms) |
| `v2x` | traffic agents' Basic Safety Messages and the lights' signal phase and timing | `v2x` | [§9](../design/domains/sensors.md#v2x) |

### Mobile robots and AMRs

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| `bumper` | a contact switch: pressed, how hard, from where | `bumper` | [§9](../design/domains/sensors.md#bumper) |
| `cliff` | the range to the floor below; a drop | `cliff` | [§9](../design/domains/sensors.md#cliff) |
| `wheel_odometry` | dead-reckoned pose and twist from the wheel encoders | `wheel_odometry` | [§9](../design/domains/sensors.md#wheel_odometry) |
| `safety_scanner` | a safety laser scanner's protective and warning fields | `safety_scanner` | [§9](../design/domains/sensors.md#safety_scanner) |
| `line_sensor` | where guide tape is under a row of channels | `line_sensor` | [§9](../design/domains/sensors.md#line_sensor) |
| `tag_reader` | floor tags read by RFID or a downward QR camera | `tag_reader` | [§9](../design/domains/sensors.md#tag_reader) |

### Arms

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| `force_torque` | a six-axis wrench at a wrist | `force_torque` | [§9](../design/domains/sensors.md#force_torque) |
| `joint_torque` | link-side torque in a joint | `joint_torque` | [§9](../design/domains/sensors.md#joint_torque) |
| `gripper_sensor` | a gripper's opening, current, grip force, and whether it holds or slips | `gripper_sensor` | [§9](../design/domains/sensors.md#gripper_sensor) |
| `proximity` | a proximity skin: distances from an array of short-range elements | `proximity` | [§9](../design/domains/sensors.md#proximity) |

### The environment

These read the weather domain's air (`--weather`, a `WeatherAPI_1` prim or a profile) and the sun.

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| `anemometer` | wind velocity, ultrasonic or cup and vane | `anemometer` | [§9](../design/domains/sensors.md#anemometer) |
| `humidity_sensor` | relative humidity and air temperature | `humidity_sensor` | [§9](../design/domains/sensors.md#humidity_sensor) |
| `rain_sensor` | rain rate and accumulation | `rain_sensor` | [§9](../design/domains/sensors.md#rain_sensor) |
| `visibility_sensor` | visibility (meteorological optical range) and extinction | `visibility_sensor` | [§9](../design/domains/sensors.md#visibility_sensor) |
| `particle_sensor` | PM1, PM2.5, PM4 and PM10 dust | `particle_sensor` | [§9](../design/domains/sensors.md#particle_sensor) |
| `light_sensor` | illuminance on its face | `light_sensor` | [§9](../design/domains/sensors.md#light_sensor) |
| `sun_sensor` | the sun's direction | `sun_sensor` | [§9](../design/domains/sensors.md#sun_sensor) |

### Underwater

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| `sonar` | a multibeam imaging sonar's fan, or a single-beam echosounder | `sonar` | [§9](../design/domains/sensors.md#sonar) |

### Vision and machine learning

`event_camera` and `stereo_camera` draw the scene, so they need a Vulkan device (a GPU, or the
software device on a machine without one).

| Sensor | What it measures | URDF type | Model |
|---|---|---|---|
| Thermal camera | each pixel's apparent temperature (Planck band) | `thermal`, `thermal_camera` | [physics](../physics/sensors.md#thermal-camera) |
| `stereo_camera` | left and right images and a stereo depth with its error (RealSense D435 style) | `stereo_camera` | [§9](../design/domains/sensors.md#stereo_camera) |
| `event_camera` | brightness-change events (DVS) | `event_camera` | [§9](../design/domains/sensors.md#event_camera) |
| `detections` | ground-truth labels for a camera: 2D and 3D boxes, classes, a segmentation image | `detections` | [§9](../design/domains/sensors.md#detections) |

## Adding one to your robot

In URDF, a sensor goes in a `<gazebo reference="LINK">` block: the link it is fixed to, its
`<pose>` on that link, its `<update_rate>` in Hz, and its parameters inside an element named as its
type. Every leaf element is a parameter by its path: `<rangefinder><range><max>12</max></range>`
is `range/max = 12` (the outer element named as the type is left out). Numbers are in SI units.

```xml
<gazebo reference="base_link">
  <sensor name="altimeter" type="rangefinder">
    <pose>0 0 -0.05 0 1.5708 0</pose>       <!-- looking down: +x turned to -z -->
    <update_rate>100</update_rate>
    <rangefinder><range><min>0.1</min><max>12</max></range></rangefinder>
  </sensor>
  <sensor name="front_bumper" type="bumper">
    <pose>0.30 0 0.03 0 0 0</pose>
    <update_rate>100</update_rate>
    <bumper><threshold>10</threshold></bumper>
  </sensor>
</gazebo>
```

Parameters left out take the defaults of the kind's section in §9. The sample AMR with bumpers,
cliff sensors, wheel odometry, a safety scanner and wheel torque sensors is
`assets/robots/amr_diffdrive/amr_diffdrive_safety.urdf`; the example
[`lagrange_example_sensors`](../../examples/lagrange_example_sensors/README.md) runs it.

In USD, a sensor is a prim with `SensorAPI_1` and `CustomSensorAPI_1`; a parameter's path takes
`:` for `/`:

```usda
def Xform "altimeter" (
    prepend apiSchemas = ["SensorAPI_1", "CustomSensorAPI_1"]
)
{
    string sensor:custom:kind = "rangefinder"
    double sensor:custom:param:range:max = 12
    double sensor:rate = 100
}
```

SDFormat takes the same `<sensor type="KIND">` as URDF. In an overlay file (`--overlay`,
[formats.md §9](../design/formats.md)) the same schemas go on a `create_child` prim.

The built-in types: an IMU, lidar, camera or thermal camera from URDF or SDFormat as in Gazebo; GNSS,
magnetometer, barometer and radar through their USD schemas (`GnssAPI_1`, `MagnetometerAPI_1`,
`BarometerAPI_1`, `RadarAPI_1`), as the sample quadcopter's overlay
(`assets/robots/quad_x500/quad_x500.overlay.yaml`) and the sample sedan (`assets/robots/sedan/sedan.usda`) do. GNSS, magnetometer and barometer run only when the site's
geo origin is given (`--geo-origin LAT,LON,HEIGHT` or the profile's `world.geo_origin`).

Sensor timing (`<latency>`, `<jitter>`) and faults (below) apply to every sensor, built-in or kind.

## Reading it

Every sensor writes signals named `sensor/<name>/<quantity>`, in SI units: a bumper named
`front_bumper` writes `sensor/front_bumper/pressed`, `force` and `direction`. `lagrange graph`
lists every signal of a run; each kind's section in §9 lists its own.

**Python**, headless and without ROS:

```python
import lagrange

sim = lagrange.Simulation(
    "assets/robots/amr_diffdrive/amr_diffdrive_safety.urdf", world=lagrange.sample_world()
)
sim.step_until(2.0)
print(sim.read("sensor/cliff_front_left/range"))  # m
print(sim.read("sensor/safety/min_distance"))  # m
print(sim.read("sensor/wheel_odom/pose"))  # x, y, z, then the quaternion
```

`sim.read` returns a signal's components as a list of numbers (a Bool reads 0 or 1);
`sim.image("sensor/<name>/<quantity>")`
returns an image (a camera's `image`, `depth` and `instance`, a stereo camera's `left`, `right` and
`depth`, an event camera's `image`, `detections`' `semantic`) as a numpy array, or None before the
first frame. `sim.signals()` lists the names.

**ROS 2**: the built-in types publish their own topics (`/scan`, `/points`, `/imu`,
`/<name>/image_raw`, `/<name>/fix`, ...; see [the command line](cli.md)). A kind's images go out
as `sensor_msgs/Image` on `/<name>/<quantity>/image_raw` with their `camera_info`, and a kind's
numbers go out by default on standard messages on `/<name>/<quantity>`: `sensor_msgs/Range` for
rangefinders and cliff sensors, `LaserScan` for safety scanners and sonars, `nav_msgs/Odometry` for
wheel odometry, `BatteryState`, `WrenchStamped`, `vision_msgs/Detection2DArray`, ... (the table is
[ROS topics of the built-in kinds](../design/domains/sensors.md#ros-topics-of-the-built-in-kinds)).
A topic mapping (`--ros-topics FILE.yaml`) renames or replaces them; an entry for the same topic or
signal wins over the default, and `kind_topics: false` in the file turns the defaults off:

```yaml
publish:
  - {signal: sensor/front_bumper/pressed, topic: /bumper/front, type: std_msgs/msg/Bool}
```

```bash
lagrange my_robot.urdf --ros-topics topics.yaml
```

The message types a mapping can use are listed under
[Plugin outputs on ROS 2](cli.md#plugin-outputs-on-ros-2).

**Viewer**: the Scopes plot any number signal (search for `sensor/`); the Sensors workspace shows
each camera and each kind with images (the Camera panel's modes are then its images: `left`,
`right`, `depth`); lidars draw their points in the viewport.

**Recordings**: `--record run.mcap` keeps the sensor signals (faults included; arrays over 1024
components are left out).

## Breaking it: sensor faults

To test how your software copes when a sensor fails, give the run faults. They work on every
sensor, built-in or kind:

```bash
# the IMU's z rate jumps by 0.02 rad/s at 5 s; the front bumper stops reporting at 20 s
lagrange amr --sensor-fault imu:angular_rate=bias,value=0.02,at=5,component=2 \
             --sensor-fault front_bumper=disconnect,at=20
```

The modes are `dropout`, `stuck`, `bias`, `drift`, `noise` and `disconnect`, each in a time window;
ROS, Python, plugins, recordings and the viewer all see the faulted values, while the ground truth
(`/ground_truth/odom`, `/tf`, the bodies' signals) stays clean. The keys, run-file and profile
forms are in [Sensor faults](cli.md#sensor-faults); the model in
[sensors.md](../design/domains/sensors.md#sensor-faults-as-built). In Python:
`lagrange.Simulation(robot, sensor_faults=["imu=stuck,at=2,for=1"])`.

For perfect sensors instead, switch the domain to its substitute: `--domain sensors=off` (or the
profile's `sensors: {enabled: false, substitute: ground_truth}`) gives exact values at the right
times, and draws no images.

## See them run

Short clips of real runs, recorded by `tools/videos/make_videos.py` (all the clips:
[feature videos](../videos/README.md)):

- [radar_4d.mp4](../videos/radar_4d.mp4): the sedan's 4D imaging radar in the town, its detections
  in the 3D view (Overlays → Radar detections) and in the Sensors workspace's radar view, coloured
  by radial velocity.
- [sensor_faults.mp4](../videos/sensor_faults.mp4): the AMR's gyro biased, then stuck, against the
  true yaw rate.
- [lidar_patterns.mp4](../videos/lidar_patterns.mp4): a channel table, a flash lidar and a
  Livox-style rosette in the same room.
- [fisheye.mp4](../videos/fisheye.mp4): a 90° pinhole and a 220° equidistant fisheye on the AMR.
- [event_camera.mp4](../videos/event_camera.mp4): a camera and an event camera with the same
  pixels.
- [amr_sensor_kinds.mp4](../videos/amr_sensor_kinds.mp4): the safety AMR's scanner entering its
  warning field at a rack, and a wheel's torque.

## What is not modelled

Each kind's section in §9 and the [physics page](../physics/sensors.md) list their own limits.
Across them:

- **Kinds that find things are detectors, not image processing.** `detections`, `landing_target`,
  `tag_reader` and `line_sensor` decide what is seen from the scene's prims and geometry; no
  fiducial or tape texture is drawn or decoded. `optical_flow` computes flow from motion and depth,
  not by matching images.
- **No real recordings yet.** The models are checked against closed forms and data sheets, not
  against logs of the real parts. The data-sheet defaults were checked against the documents on
  2026-11-12 (`docs/plan/sensor-defaults-verification-2026-11-12.md`); the defaults marked
  illustrative in §9 have no primary source: give your part's figures.
- **Lidar**: no beam divergence or multiple returns. Scan patterns and fisheye lenses come from
  URDF, SDFormat (Lagrange's `lagrange:` elements, [formats.md §9.3](../design/formats.md)) and USD
  (`LidarAPI_1`'s `scanPattern`, `channelElevations`, `rosette:*`; `CameraSensorAPI_1`'s `lensType`,
  `cutoffAngle`; [formats.md §3.8](../design/formats.md)), and `lagrange_import` keeps them; a USD
  file that uses them runs as a grid lidar and a pinhole camera in Lagrange 1.2 or older.
- **Camera**: no motion blur, absolute radiance or colour filter array; a fisheye has no vignetting,
  rolling shutter or exposure model, and `detections`, `stereo_camera` and `event_camera` take
  their camera as a pinhole.
- **GNSS**: no outages, multipath or latency model; **magnetometer**: no crustal field and no hard-
  or soft-iron effects; **radar**: no rain attenuation, multipath or ghosts.
- **No gas sensors**: no domain carries gas concentrations.
