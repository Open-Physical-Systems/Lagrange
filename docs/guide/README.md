# User guide

| Page | What it covers |
|---|---|
| [Tutorial: an AMR in a warehouse](tutorial-amr.md) | Run, watch, drive over ROS 2, record, substitute a domain, add a module, Nav2. Start here |
| [Tutorial: a quadcopter, from hover to a PX4 mission](tutorial-drone.md) | The sample quadcopter's files and site, hover, gusty wind, PX4 SITL in lockstep, a low battery |
| [Tutorial: test your own drone, step by step](tutorial-custom-drone.md) | The Holybro X500 V2 from a hover to live computed air: overlay, teleop, blade elements, downwash, a building, checks |
| [Tutorial: a robot's power system from a schematic](tutorial-power.md) | A SPICE netlist named from your URDF, seen in the viewer and checked by hand |
| [Tutorial: weather, interference and radio links](tutorial-environment.md) | Fog and rain on the lidars, fog and wind fields (and wind from OpenFOAM), a power line next to a magnetometer, Wi-Fi through a wall |
| [Tutorial: huge worlds](tutorial-huge-worlds.md) | A world streamed in tiles, the floating origin, robots far apart, a world built from an elevation model and OpenStreetMap |
| [Tutorial: the sample arm with MoveIt 2](tutorial-arm.md) | A 6-DoF arm on a 48 V bus, MoveIt 2 pick-and-place, motor heating and derating |
| [Tutorial: a car in a city block](tutorial-cars.md) | The sedan's layers, OpenDRIVE roads, traffic, radar and camera, an OpenSCENARIO cut-in, driving over `/ackermann_cmd` |
| [Tutorial: people around your robot](tutorial-humans.md) | A crowd, spawning a person, your own person with waypoints, how close a robot came, and what people do not do |
| [Autoware with a simulated car](autoware.md) | The vehicle interface (`lagrange_autoware`), building it with the Autoware messages, the sedan launch, and what to do while Autoware is not on Lyrical |
| [The run file](run-file.md) | One YAML file for a whole run: robot, world, what to simulate, the site. Start here for your own robot |
| [Concepts](concepts.md) | Domains and substitutes, signals and units, the multirate scheduler, capability tiers, determinism, coordinates |
| [Profiles](profiles.md) | The YAML that says what a run simulates; what v0.2 applies; the errors |
| [Writing a module](writing-a-module.md) | The module interface, worked through with a wheel odometer |
| [Writing a plugin](writing-a-plugin.md) | A plugin from the template: the C ABI, seeing the scene, a custom time-of-flight sensor, ROS topics, a physics engine and its conformance suite |
| [The `lagrange` command](cli.md) | Run the simulator and viewer, spawn robots, load plugins; `run`, `validate`, `graph`, `info` |

The tutorial runs end to end in `tools/ci/tutorial_amr.sh`, and the module chapter's code is
built and tested in `tests/tutorial/`, so both stay in step with the code. The plugin chapter's
template is built and tested in a scratch workspace by `tools/ci/check_plugin_template.sh`.
