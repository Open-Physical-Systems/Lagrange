# User guide

| Page | What it covers |
|---|---|
| [Tutorial: an AMR in a warehouse](tutorial-amr.md) | Run, watch, drive over ROS 2, record, substitute a domain, add a module, Nav2. Start here |
| [Tutorial: a quadcopter, from hover to a PX4 mission](tutorial-drone.md) | The sample quadcopter's files and site, hover, gusty wind, PX4 SITL in lockstep, a low battery |
| [Tutorial: weather, interference and radio links](tutorial-environment.md) | Fog and rain on the lidars, fog and wind fields (and wind from OpenFOAM), a power line next to a magnetometer, Wi-Fi through a wall |
| [Tutorial: huge worlds](tutorial-huge-worlds.md) | A world streamed in tiles, the floating origin, robots far apart, a world built from an elevation model and OpenStreetMap |
| [Tutorial: the sample arm with MoveIt 2](tutorial-arm.md) | A 6-DoF arm on a 48 V bus, MoveIt 2 pick-and-place, motor heating and derating |
| [Tutorial: a car in a city block](tutorial-cars.md) | The sedan's layers, OpenDRIVE roads, traffic, radar and camera, an OpenSCENARIO cut-in, driving over `/ackermann_cmd` |
| [Autoware with a simulated car](autoware.md) | The vehicle interface (`lagrange_autoware`), building it with the Autoware messages, the sedan launch, and what to do while Autoware is not on Lyrical |
| [Concepts](concepts.md) | Domains and substitutes, signals and units, the multirate scheduler, capability tiers, determinism, coordinates |
| [Profiles](profiles.md) | The YAML that says what a run simulates; what v0.2 applies; the errors |
| [Writing a module](writing-a-module.md) | The module interface, worked through with a wheel odometer |
| [The `lagrange` command](cli.md) | `run`, `validate`, `graph`, `info` |

The tutorial runs end to end in `tools/ci/tutorial_amr.sh`, and the module chapter's code is
built and tested in `tests/tutorial/`, so both stay in step with the code.
