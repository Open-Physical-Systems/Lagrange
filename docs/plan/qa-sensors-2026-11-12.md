# Sensors QA, 2026-11-12: the built-in kinds, faults, images

An independent, adversarial test of what the overnight run added to `lagrange_sensors`: the 37
built-in custom sensor kinds (`modules/sensors/src/kinds/`, sensors.md §9), sensor fault injection
(`Faults.cpp`), the fisheye camera and the images of kinds. Branch `night/qa-sensors`, from
`night/integration` at `1f3cae4`. Each finding has a minimal reproduction; the fixed ones have a
regression test that fails without the fix.

## Findings

| # | Severity | Finding | Where | State |
|---|---|---|---|---|
| 1 | High | Every kind at a rate whose period is not a whole number of ns beside 1 ms (30, 7, 3 Hz) refused to start: "base tick 1 ns is below 1 µs" | `kinds/KindModule.hpp` `kindPeriod` | Fixed `1edd420` |
| 2 | High | A fisheye camera on a Vulkan device segfaulted the server at init (`renderer_` null: the GPU camera model merge called `setColour` on it); `test_fisheye` crashed in `night/integration` | `Sensors.cpp` `Camera::init` | Fixed `8bdfb32` |
| 3 | Medium | `current_sensor`/`voltage_sensor` (INA226) step every conversion × channels × averages (2.2 ms by default): beside any 1 Hz sensor the steps repeat after 11 s and the run is refused (the sample `quad_x500_telemetry.usda` plus a 1 Hz humidity sensor); the sonar's 2R/c ping interval likewise | `PowerMonitor.cpp`, `Sonar.cpp`, `EscTelemetry.cpp` | Fixed `1edd420` |
| 4 | Medium | `adsb_receiver`, `v2x` and `vehicle_can` kept state outside `saveState`: after `restore_state` (and after every floating-origin rebase, which restores) `vehicle_can` read 57 and −98 m/s² of acceleration, `adsb_receiver` a negative message age and another squitter clock | the three kinds | Fixed `4d04bc5` |
| 5 | Medium | A NaN parameter (`<max>nan</max>`; `strtod` reads it) passes every range check (comparisons with NaN are false): a rangefinder that never reads, arrays sized from an undefined cast (`proximity` rows, `line_sensor` channels, the converters' `samples`) | `Kinds.cpp` `createBuiltinKind` | Fixed `cf643f9` (all kinds) |
| 6 | Medium | Spawning a second copy of a robot with `wheel_odometry` was refused: spawned robots are passive (`prefixModel` clears `actuated`) and the kind found only driven wheels | `WheelOdometry.cpp` `detect` | Fixed `e62de6c` |
| 7 | Medium | `airspeed`/`anemometer` with a misspelt `air_rotor` read NaN air for ever, taken as still air: the probe stayed `valid` and read only the craft's own motion | `Airspeed.cpp`, `Anemometer.cpp` | Fixed `cb613e7` |
| 8 | Medium | Any spawn or delete, even a refused spawn, rebuilds the run and every kind starts afresh: the first robot's `wheel_odometry` jumps back to (0, 0), filters and accumulators restart (`tpms` leak, `rain_sensor` accumulation) | `Server::spawnRobot`/`removeRobot` rebuild | Fixed `5a0a11b` (a module of the same label and signals carries its state over the rebuild; a refused spawn restores the run it puts back) |
| 9 | Medium | After `restore_state`, image signals read `None` until the next frame, and `state_hash` differs from the uninterrupted run whenever a run publishes images (camera, event, stereo), though every numeric signal matches: buffer handles and their sequence are not part of the saved state | `core/signals` `SignalBus`, `Scheduler::saveState` | Fixed `7a26322` (buffers and sequences in the saved state; the hash leaves pixels out) |
| 10 | Low | `safety_scanner` `response_time` 1e9 s: its delay ring asked for 3e10 bytes, `std::bad_alloc` at start | `SafetyScanner.cpp` | Fixed `428b1fe` (over 60 s refused) |
| 11 | Low | `wheel_odometry` `separation_error` −1 believed a 0 m track: NaN pose for the rest of the run (`radius_error` −1 likewise) | `WheelOdometry.cpp` | Fixed `51052a7` |
| 12 | Low | `<update_rate>0</update_rate>` (Gazebo's "as fast as possible") or a negative rate: a 1000 s period, refused with a message about the profile's dt values | `Kinds.cpp` | Fixed `4a7d8e8` (refused by name); built-in sensors at 0 Hz still behave the old way |
| 13 | Low | A misspelt `signal` on `current_sensor`/`voltage_sensor`, or `battery` on `battery_monitor`, reads NaN with `valid` 0 for ever, never reported (the inputs are optional so that the domain may be off) | `PowerMonitor.cpp`, `BatteryMonitor.cpp` | Fixed `add82ee` (warned at start; the domain being off is named as the other cause) |
| 14 | Low | A non-ASCII sensor name: the wiring error quotes one byte of a multi-byte character, the message is not UTF-8 and Python raises `UnicodeDecodeError` instead of the start error | `core/signals` name check | Fixed `bbe04d3` |
| 15 | Low | A sensor named `fault` could not take a fault ("no sensor fault writes anything"): the `/fault/` segment test also matched `sensor/fault/...`. A name with `/` (`a/b`) is still accepted and its signals read as sensor `a`'s | `Faults.cpp` | Fixed `f24a574` (the name); `/` open |
| 16 | Low | A custom kind that is neither built in nor implemented by a loaded plugin (`tof` without `--plugin lagrange_tof`) does nothing, silently | `createModules` | Fixed `685c2c8` (warned at load with the built-in kinds) |
| 17 | Low | `uwb` `anchors` and `wifi_scanner` `access_points` are region-local text: after a floating-origin rebase they are read again unshifted, off by the rebase | `Uwb.cpp`, `WifiScanner.cpp` | Fixed `a753155` (`shiftOrigin`) |
| 18 | Low | `battery_monitor` `capacity` is in A·h, where §9 says every number is SI | sensors.md, `BatteryMonitor.cpp` | Fixed `f19f936` (docs: the unit stays A·h, 1.x is frozen) |
| 19 | Info | The sedan's restored run hashes differently (no kind involved): finding 9, its camera | | Fixed with 9 |
| 20 | Medium | `detections` on a fisheye camera drew its occlusion renders and projected its truncation through a pinhole of the camera's `horizontal_fov` (over 180°: a negative focal length): a crate half hidden by a pole read occlusion 0 and truncation 0 | `Detections.cpp` | Fixed `5107e02` (NaN on a fisheye) |
| 21 | Medium | A lidar's `samples` −1 became 4294967295 columns, 1e8 columns asked for gigabytes: `std::bad_alloc` at start (a near-OOM on a shared machine); pre-existing, kept by the scan patterns | `Sensors.cpp` Lidar, `LidarLayout.hpp` | Fixed `6480e47` (below 1 or over 2²² beams refused) |
| 22 | High | On a robot added to a run (spawned, `--add-robot`) every name is prefixed `<robot>_` but the sensors' text parameters are not: a copy's `wheel_odometry` with `left_joint`/`right_joint` read the first robot's wheels (0.95 m of travel standing still), its `current_sensor` the first robot's bus; a second `quad_x500_telemetry.usda` could not be spawned ("the stage has several batteries") | every kind that names a joint, body, component, camera or signal | Fixed `dce4184` (names resolve to the sensor's own robot first; the only battery or tether is its own robot's) |
| 23 | Medium | Spawning a second quadcopter stops the first: with the sample quadcopter flying, `sim.spawn(sample_quadcopter(), "q2", ...)` leaves both on the ground, rotors at 0 rad/s (spawning an AMR instead leaves it flying); not a sensor, found on the way | `server/src/Multicopter.cpp` `canFly`: it asks that every `RotorAPI_1` joint of the world hang from one body, so a second multirotor anywhere in the run leaves the first without its controller | Fixed `537dc88` (the first robot's rotors and mass; later robots passive) |
| 24 | Low | `sonar` `water_temperature` is in °C where §9 says every number is SI: 283.15 (10 °C in K) gave a sound speed of 3886 m/s and every range 2.6 times short | `Sonar.cpp` | Fixed `c432f5a` (outside −2 to 40 °C refused) |
| 25 | Low | `rangefinder` read +inf for a target closer than `range/min` where REP 117, which its docs cite, gives −inf: a landed drone's altimeter said "nothing below" | `Rangefinder.cpp` | Fixed `35fd575` |
| 26 | High | A built-in lidar, IMU, radar, GNSS, magnetometer or barometer at a rate without a whole-ns period (15 or 30 Hz) refused to start ("base tick 1 ns"): the same exact 1e9/rate period as finding 1; a 15 Hz lidar is common; pre-existing | `Sensors.cpp` `periodOf`, `GeoSensors.cpp`, `Radar.cpp` | Fixed `312c726` |
| 27 | Medium | `current_sensor`/`voltage_sensor` took any parameter: a shunt of 0 Ω read NaN, a Hall sensitivity of 0 read inf, a divider ratio of 0 NaN, a negative shunt the wrong sign, all with `valid` 1; a negative `adc/reference` broke `std::clamp`'s precondition (UB); `esc_telemetry` `gear_ratio` 0 read a speed of 0 | `PowerMonitor.cpp`, `EscTelemetry.cpp` | Fixed `5b5fd7d` |

Severities: High, a run cannot start or crashes for an ordinary input; Medium, wrong or lost data,
or a refusal, in a plausible use; Low, an edge case or a misleading message.

## What was checked and held

- **Every kind with its defaults** (one per run on the sample AMR, none and warehouse worlds; the
  quadcopter's power kinds; the sedan's car kinds in the town with traffic): each starts or is
  refused with a message naming the sensor and what is missing; no crash, no hang. Outputs of a
  kind whose domain is off are NaN with `valid` 0, as documented.
- **Parameters** (`qa/fuzz_all.py`, 22 kinds, 120 keys, −1, 0 and 1e9, three runs in parallel;
  `qa/fuzz_quad.py` and `qa/fuzz_sedan.py` for the power and car kinds; NaN and inf separately): no crash, no timeout, no allocation blow-up after fix 10; sizes are capped
  (`proximity` 4096 elements, `sonar` 4096 beams, `event_camera` 1e6 events, image sizes).
- **Save and restore** (`qa/restore_check.py`: save at t1, run to t2 reading every `sensor/`
  signal at 10 to 100 ms, restore, run again, compare): the AMR set (odometry, line, proximity,
  rangefinder, cliff, bumper, optical flow, sonar, tag reader, Wi-Fi, ADS-B, V2X), the quadcopter
  set (airspeed, anemometer, KISS and DShot ESCs, fuel gauge, INA226 bus and current monitors)
  and the sedan set (CAN, TPMS, V2X, ADS-B with a spawned aircraft) all repeat exactly after the
  fixes, also with faults (dropout, noise, stuck, drift) applied. A code audit of every kind's
  members written in `step` found no other unsaved state.
- **Determinism**: two fresh runs give equal `state_hash` (AMR set with `lidar="cpu"` and
  `"gpu"`, quadcopter set, sedan in the town with traffic); no kind uses the GPU ray context, so
  `--lidar gpu` leaves them on the CPU reference. The rendering kinds (`event_camera`,
  `stereo_camera`, `detections`) say they are not deterministic on a GPU, as documented.
- **Ground-truth substitute**: the AMR set under `sensors: {enabled: false, substitute:
  ground_truth}` runs, reads exactly (no quantisation, no noise), renders nothing, deterministic.
- **Placement**: kinds on a link fixed to the world (a pole in a world URDF) run; `bumper` and
  `wheel_odometry` there are refused by name. Two kinds of one name, or one kind twice, are
  refused as duplicate writers. Spawn and delete with kinds on both robots work after fixes 6 and 22.
- **Faults with kinds**: stuck, dropout (with NaN), noise, drift, bias, disconnect on arrays,
  numbers and an event camera's image; numeric modes on an image are refused; bad fault texts
  (`at=nan`, `probability=2`, `sigma=-1`, `rate=inf`, unknown sensor, unknown quantity, component
  out of range) are refused with clear messages.
- **Physics spot checks** (CPU, warehouse or a wall world): `rangefinder` and `proximity` read the
  wall at (5 − x)/cos ψ to 0.2 mm; `cliff` and `optical_flow` read their height (0.0998 m,
  0.1998 m); optical flow per step is v·dt/h (0.015 rad at 0.3 m/s, 100 Hz, 0.2 m); the pitot
  reads ½ρV² at the wind's 5.87 m/s (q −22.1 Pa for −21.1, tail wind, within a count and the
  noise) and the anemometer 5.94 m/s from π (wind from the west, sensor +x east); the fuel gauge
  and INA226 read the battery's 16.478 V and 13.744 A, remaining capacity 0.9962 × 18000 C;
  `light_sensor` at Zürich on 21 June at 11:30 UTC reads 105 218 lx against 105 203 lx of the
  IES clear-sky model, 19 491 lx overcast against 19 490, 0 at night, 65 535 lx saturated by
  default; the sun sensor's elevation is 66° (90° − 47.4° + 23.44°).
- **Sensor timing and ROS**: `<latency>`/`<jitter>` on kinds give `delayed/<q>` and
  `delayed/stamp` matching the live value at the stamp, for numbers, arrays and images;
  `--ros-topics` publishes a kind's Range, Bool and Float64MultiArray; an event camera's
  `image_raw`, `camera_info` (frame `ev_optical`, K) and `events` appear on ROS.
- **A plugin implementing a built-in kind** (the sample `tof` plugin rebuilt for kind
  `rangefinder`): refused as duplicate writers, as §9 says.

## What the fixes change

- Fixes 1 and 26 keep a period of whole nanoseconds exactly (10, 128, 400 Hz: as before) and put
  any other on whole milliseconds (whole 10 µs below 1 ms), as the cameras' already were: 30 Hz
  runs every 33 ms (30.3 Hz), 15 Hz every 67 ms. A rate between 500 and 1000 Hz without a whole
  ns period (600 Hz) rounds to 1 or 2 ms, up to 30 % off; a finer grain would bring back runs
  refused beside a 1 Hz sensor (hyperperiod over 10 s). The INA226 kinds read every 2 ms
  instead of 2.2 ms by default (their window average still covers the whole period), the KISS
  ESC every 4 ms instead of 3.5 ms (never faster than the wire). Every run that started before
  keeps its steps: only periods that were refused change.
- Fix 22 resolves a name to `<robot>_<name>` only when the sensor's robot is a prefixed one and
  such a name exists; a single-robot run reads every name as before.

- Fixes 8, 9, 13, 14, 16, 17, 18 and 23 (branch `night/qa-fixes`): the scheduler's saved state
  holds the bus's published buffers (state version 2: an older saved state is refused by name);
  `state_hash` covers each buffer's handle and sequence, not its bytes, so two GPU runs still
  hash alike. A spawn or delete restores each module whose label and signals (name, step,
  inputs and outputs with units) are unchanged; physics and any module the new robot joins
  start again, as before. The built-in multicopter controller flies the robot of the first body.

## Not done

- The floating-origin rebase was not driven end to end (it needs 1 km of travel); it saves and
  restores state as `restore_state` does, so the restore checks cover the kinds' state, and the
  v2x rows now shift.
- `gripper_sensor`, `landing_target`, `tag_reader` with tags, `uwb`, `detections`,
  `particle_sensor` and the weather kinds with the weather on were run only through their own
  tests and the parameter sweep, not in a scenario.

## How to repeat

Scripts in the worktree's `qa/` (not committed): `probe_kinds.py` (each kind with defaults),
`fuzz_all.py` (parameters), `restore_check.py` and `restore_sedan.py` (save and restore),
`faults_kinds.py`, `gt_run.py`, `textfuzz.py`. The regression tests are in `lagrange_sensors`:
`colcon test --packages-select lagrange_sensors` (358 tests), and `lagrange_server` (246) and
`lagrange_python` (46), all passing on this branch.
