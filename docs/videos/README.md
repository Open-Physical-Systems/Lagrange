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
| [realistic_warehouse.mp4](realistic_warehouse.mp4) | A robot in the sample warehouse | `lagrange ROBOT WORLD` |
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
