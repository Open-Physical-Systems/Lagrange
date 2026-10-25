# Tutorial: weather, interference and radio links

v0.5 adds the world around the robot: the weather, which blinds lidars and cameras; volumetric
fields of fog and wind, including wind from a CFD case; electromagnetic interference, which
disturbs magnetometers and IMUs; and radio links, which lose and delay messages. In this
tutorial you switch each on for the samples and read what it does. It takes about 20 minutes.

`tools/ci/tutorial_environment.sh` runs every step below without the viewer and checks the
results, so the commands here are known to work. The numbers quoted are what that script
printed on the development machine.

## Before you start

Build Lagrange as in the [README](../../README.md#build). In every terminal, from the source
tree:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/lagrange_ws/src/lagrange && source ../../install/setup.bash
alias lagrange='ros2 run lagrange_server lagrange'
```

## 1. The site comes first

The weather depends on where and when the robot runs, so Lagrange has no defaults for it: the
air temperature, pressure and relative humidity are the site's, given by the profile, by
`--weather temperature=K,pressure=PA,relative_humidity=X` or by a `WeatherAPI_1` prim. Without
them the weather domain does not run. The same holds for a power line's grid frequency (50 or
60 Hz, by region), which the scene states.

```bash
lagrange validate profiles/examples/amr_fog.yaml profiles/examples/quad_x500_power_line.yaml
```

`amr_fog.yaml` puts the sample AMR in 60 m of fog at 10 °C, 1000 hPa and 95 % humidity.

## 2. Fog and rain on the lidars

```bash
lagrange run profiles/examples/amr_fog.yaml
```

With the viewer connected (`ros2 run lagrange_viewer lagrange_viewer`), the Environment
workspace shows the weather. Some of the lidar's points jump to false returns close to the
sensor.

What happens per beam (domains/weather.md §8):

- **Range:** a target is seen only if the lidar equation still finds it through the fog's
  extinction, both ways.
- **Clutter:** the fog's backscatter makes false returns about 2 m out.
- **Rain:** drops in the beam give a few false returns, about 1 % of beams at 2 mm/h.
- **Camera:** it gets a veil of air light.

In the small warehouse every wall is within 20 m, so 60 m of fog costs only a few of the front
lidar's 720 returns (716 in the script). Try `visibility: 15` in a copy of the profile to see
the range collapse.

## 3. A fog bank from a field

Fog is rarely uniform. A NanoVDB grid of extinction (1/m) describes a bank of it:
`assets/fields/warehouse_fog_bank.nvdb` is 10 m visibility across the aisle between x = 2 and
6 m, made by `ros2 run lagrange_fields lagrange_make_fields fog_bank OUT.nvdb`. Name it as the
weather's `fog_field` (in the profile, or `--weather ...,fog_field=FILE.nvdb`) and the lidar
integrates it along every beam: 109 of the front lidar's beams, the ones looking down the aisle,
lose their target. In the viewer, subscribe to `weather/fog_density` in the Slice 2D panel to
see the bank around the robot.

## 4. Wind from a CFD case

A wind field around buildings comes from a CFD solver. Lagrange reads an OpenFOAM case's
results without depending on OpenFOAM: write the cell centres and the velocity as ASCII, then
convert them.

```bash
# In the OpenFOAM case, at the time you want:
postProcess -func writeCellCentres -time 500
# Then, here:
ros2 run lagrange_fields lagrange_foam_to_vdb CASE/500/C CASE/500/U wind.nvdb --voxel 1 \
    --offset 0,0,0
```

Each voxel holds the mean velocity of the cells inside it; cells of the voxel size, aligned with
the origin, map one to one. `--offset` moves the case's coordinates into the scene's. Name the
result as the weather's `wind_field`: the drag on the robot's links and the rotors' inflow use
it inside its bounds and the mean wind outside. The script writes a small stand-in case (4000
cells of 1 m) and converts it.

## 5. A power line and a magnetometer

`assets/worlds/power_line/power_line.usda` adds an overhead line to the open field: one conductor
10 m up, carrying 400 A at 60 Hz (`EmiSourceAPI_1`). `quad_x500_emi.usda` marks the quadcopter's
magnetometer and IMU as victims (`EmiSusceptibilityAPI_1`).

```bash
lagrange run profiles/examples/quad_x500_power_line.yaml \
    --robot assets/robots/quad_x500/quad_x500_emi.usda \
    --world assets/worlds/power_line/power_line.usda
```

Hovering 2 m up, 9.9 m from the conductor, the magnetometer sees the line's field on top of the
Earth's: 8.1 µT RMS, which is `μ0·I/(2πd)`. It samples at 100 Hz, so the 60 Hz field shows at
40 Hz, as it would on a real sensor. That is the aliasing a heading filter has to cope with. In
the viewer, the Environment workspace lists the source and the Slice 2D panel shows |B| around
the drone (`emi/flux_density`). To turn the disturbance off, add `--domain emi=off`; the zero
substitute takes its place.

## 6. Wi-Fi through a wall

`amr_diffdrive_wifi.usda` gives the AMR a 2.4 GHz radio (`RadioLinkAPI_1`) and a CAN bus
(`CanBusAPI_1`). The access point stands outside the warehouse's east wall.

```bash
ros2 run lagrange_server lagrange_server \
    --robot assets/robots/amr_diffdrive/amr_diffdrive_wifi.usda \
    --world assets/worlds/warehouse_small/warehouse_small.usda \
    --link-topics assets/robots/amr_diffdrive/amr_diffdrive_wifi.links.yaml
```

Every 10 ms the comms domain works out the link from:

- the path loss over the distance, with exponent 3 indoors;
- 10 dB for the wall the straight line crosses;
- slow fading;
- the receiver's noise, plus any EMI in its band.

At the start the robot receives about −61 dBm at 34 dB SNR, and its 16-QAM packets arrive.
`link/wifi/*` holds the numbers (received power, SNR, packet loss after retries, latency,
throughput, up), and `can/can0/state` the CAN bus's fault-confinement state.

`--link-topics` makes it matter to your software: `/scan`, `/odom` and `/cmd_vel` ride on the
link (`amr_diffdrive_wifi.links.yaml`). A message is lost with the link's packet loss and
otherwise arrives after its latency.

This sample's link has margin to spare anywhere in the small warehouse. To watch it fail, copy
`amr_diffdrive_wifi.usda` and set `comms:txPower = 0.01` (10 mW). At the start that gives about
24 dB of SNR, enough for every packet. At the west wall it gives about 17 dB, where 16-QAM's
1500-byte packets are nearly all lost even after three retries. The 4 dB of shadowing moves both
figures around. Drive the robot west with `teleop_twist_keyboard` and watch `ros2 topic hz /scan` fall.
To compare against a perfect link, use `--domain comms=off`.

## Where to go next

- The models and their limits: [weather](../design/domains/weather.md),
  [EMI](../design/domains/emi.md), [comms](../design/domains/comms.md), each in §8.
- Your own scenes: the schemas are in `schemas/weather`, `schemas/emi` and `schemas/comms`
  ([formats](../design/formats.md) §3).
