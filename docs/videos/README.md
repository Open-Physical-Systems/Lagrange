# Feature videos

Each clip is a real run, seen through the headless viewer: `tools/videos/make_videos.py` starts the
simulator, sends the commands and puts each under the picture as it is sent. Remake them with
`python3 tools/videos/make_videos.py [CLIP...]` after sourcing the workspace (real time on an Intel
Iris Xe, about 12 minutes for all).

| Video | Shows | Command in the clip |
|---|---|---|
| [realistic_water.mp4](realistic_water.mp4) | The sea: waves, sun glitter, reflections, foam round a floating crate | `--water` |
| [realistic_water_wind.mp4](realistic_water_wind.mp4) | The same sea getting rough as the wind rises | `lagrange set wind_speed=14` |
| [realistic_materials.mp4](realistic_materials.mp4) | Chrome to rubber, in sun and shadow, with a pane of glass | `<visual_material preset="chrome"/>` |
| [town_junction.mp4](town_junction.mp4) | The town: lights change, people press a button and cross, cars stop and go | `lagrange town` |
| [town_street.mp4](town_street.mp4) | Street level at a lone crossing with its signal | `lagrange town` |
| [town_summer.mp4](town_summer.mp4) | Summer | `lagrange town --season summer` |
| [town_fall.mp4](town_fall.mp4) | Fall | `lagrange town --season fall` |
| [town_winter.mp4](town_winter.mp4) | Winter | `lagrange town --season winter` |
| [town_rainy.mp4](town_rainy.mp4) | The rainy season | `lagrange town --season rainy` |
| [realistic_warehouse.mp4](realistic_warehouse.mp4) | A robot in the sample warehouse | `lagrange ROBOT WORLD` |
| [wind_tunnel_wake.mp4](wind_tunnel_wake.mp4) | Speed in a slice along an Ahmed body, moving, over pressure seen from above | `lagrange tunnel --frames 200` |
| [wind_tunnel_lexus.mp4](wind_tunnel_lexus.mp4) | Speed along the Lexus RX 450h's body mesh, moving, over pressure from above. Made by hand, see cfd.md §6.2 | `lagrange tunnel --frames 200` |
| [wind_tunnel_flow.mp4](wind_tunnel_flow.mp4) | The mean flow as a world: streamlines, a slice, surface pressure, colour scale | `lagrange --robot none --world DIR/flow_view.usda` |
| [drone_low_altitude.mp4](drone_low_altitude.mp4) | The quadcopter coming down to half a metre: the outwash ring spreads wide | `/cmd_vel` down |
| [drone_building.mp4](drone_building.mp4) | Beside a 12 m building and over its roof: the wake hugs the wall, spreads on the roof; the rotors by the wall make less thrust | `lagrange run quad_x500_downwash.yaml`, `drone_building.urdf` |
| [drone_downwash.mp4](drone_downwash.mp4) | The air under a hovering quadcopter: particles in 3D and its speed through the drone, then leaning in wind | `--overlays airflow`, `lagrange set wind_speed=4` |
| [gravity.mp4](gravity.mp4) | A pendulum under Earth, Moon and Jupiter gravity, changed while it swings | `lagrange set gravity=1.62` |
| [wind.mp4](wind.mp4) | A hovering quadcopter leaning into 6 and 14 m/s of wind, and calm again | `lagrange set wind_speed=6 wind_from=270` |
| [plugin.mp4](plugin.mp4) | A time-of-flight sensor from a shared library, its range on the scope | `--plugin lagrange_tof` |
| [sensor_delay.mp4](sensor_delay.mp4) | Yaw rate on time and 0.4 s late, lidar returns as they arrive | `--sensor-timing imu=0.4,0.02` |
| [water.mp4](water.mp4) | A wooden crate bobs at its draft, a steel one sinks | `--water 0,0,1,12,8,1` |
| [lidar_materials.mp4](lidar_materials.mp4) | Retroreflector, white paint, asphalt and black rubber walls to a lidar | `<sensor_material preset="..."/>` |
| [lidar_water.mp4](lidar_water.mp4) | No returns from a pond at a slant, also not from the ground under it | `--water` |
| [lidar_motion_distortion.mp4](lidar_motion_distortion.mp4) | A spinning lidar in a square room, distortion on and off side by side | `<motion_distortion>0` |
| [joint_control.mp4](joint_control.mp4) | An arm by position and by velocity per joint | `/joint_commands` |
| [motor_modes.mp4](motor_modes.mp4) | A DC motor at 6 V, 12 V and 0 V: speed and current | `elec:controlMode: voltage` |
| [spawn.mp4](spawn.mp4) | Robots and a person added to and removed from a running simulation | `lagrange spawn`, `lagrange delete` |
| [run_file.mp4](run_file.mp4) | One file starting a whole run | `lagrange run.yaml` |

In the lidar materials clip the viewer's lidar views draw two tones, strong and attenuated, not a
colour scale. The presets' reflectivities are illustrative, not measured.

The wind tunnel clips show where the air goes on a coarse grid. Their Cd, 1.35 at 2M cells, is far from the Ahmed body's measured 0.29 to 0.30 (`docs/design/domains/aerodynamics.md` §10). The stripes in its pressure panel are a known defect of the flow, not of the video ([cfd.md](../design/domains/cfd.md)).
