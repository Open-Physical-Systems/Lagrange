# User guide

| Page | What it covers |
|---|---|
| [Tutorial: an AMR in a warehouse](tutorial-amr.md) | Run, watch, drive over ROS 2, record, substitute a domain, add a module, Nav2. Start here |
| [Tutorial: a quadcopter, from hover to a PX4 mission](tutorial-drone.md) | The sample quadcopter's files and site, hover, gusty wind, PX4 SITL in lockstep, a low battery |
| [Tutorial: the sample arm with MoveIt 2](tutorial-arm.md) | A 6-DoF arm on a 48 V bus, MoveIt 2 pick-and-place, motor heating and derating |
| [Concepts](concepts.md) | Domains and substitutes, signals and units, the multirate scheduler, capability tiers, determinism, coordinates |
| [Profiles](profiles.md) | The YAML that says what a run simulates; what v0.2 applies; the errors |
| [Writing a module](writing-a-module.md) | The module interface, worked through with a wheel odometer |
| [The `lagrange` command](cli.md) | `run`, `validate`, `graph`, `info` |

The tutorial runs end to end in `tools/ci/tutorial_amr.sh`, and the module chapter's code is
built and tested in `tests/tutorial/`, so both stay in step with the code.
