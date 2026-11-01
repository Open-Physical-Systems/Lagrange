# What to develop next: an evaluation, 2026-11-01

Asked for by the owner. It rests on reading the code and plan documents and on the measurements below,
on this machine (20 threads, RTX 4060 laptop GPU), one run each. Rankings are judgement, not data.

## Speed today

Headless, from Python, CPU lidar, 5 s of simulated time:

| Run | Real-time factor |
|---|---|
| Sample AMR in the warehouse, lidar on | 41× |
| Sample arm in its workcell | 29× |
| Quadcopter | 28× |
| Sedan in the city block | 3.3× |
| Sedan in the town with 24 cars and 36 people | 2.1× |

So one AMR is fast enough to train on with processes in parallel, and a car in the town runs at about
twice real time on one process. There is no batched path: nothing runs thousands of copies on one GPU.

`lagrange.envs.ParallelEnvs`, steps of 20 ms per second in total:

| Processes | Arm with no sensors | Sample arm with its wrist camera |
|---|---|---|
| 1 | 14,300 | 1,400 |
| 2 | 25,500 | 2,000 |
| 4 | 45,100 | 2,300 |
| 8 | 28,600 | 2,400 |
| 16 | 34,500 | 2,500 |

Past 4 processes it does not rise: this laptop's other cores are slower and everything waits for the
slowest each step. With a camera, rendering sets the limit however many processes there are.

## Where it is strong

Multi-domain coupling is the wedge and it is real: electrical, thermal, weather, EMI and comms reach
the sensors and the robot through checked signals, each with a substitute. A robot file loads directly.
A schematic can now be the power system, checked against hand formulas and ngspice. The ten example
packages run through launch files, and the C plugin path works without linking Lagrange.

## Where it is weak, most important first

| # | Gap | What exists | Size |
|---|---|---|---|
| 1 | **Nothing tests the viewer's mouse.** Buttons, dragging and forms ship checked by screenshot only | Headless screenshots, no input | Small |
| 2 | **No reinforcement-learning path.** Isaac Lab, MuJoCo Playground and others start here | One `Simulation`; no Gymnasium wrapper, no parallel environments, no domain randomization; the profile's `batch` key does nothing | Small for a wrapper over processes; very large for GPU batches |
| 3 | **Concave collision meshes are hulls at import.** A hollow link blocks what should pass through it; `collision_bake` splits them only in USD, and the import had already made them hulls | CoACD bake tool | Medium |
| 4 | **Most models have no measured reference.** CLAUDE.md §10 asks for one "where possible"; `docs/validation/report.md` says sim against real "mostly does not exist" | Battery (LG HG2), rotor (NACA 640), WMM, ICAO; the rest closed-form or between engines | Large, needs recordings (Q16) |
| 5 | **Common sensors missing.** No event or thermal camera; force-torque and contact only in MuJoCo (`joint/<j>/reaction`, `body/<b>/contact_force`); ultrasonic through the `lagrange_tof` plugin | IMU, lidar, camera, GNSS, magnetometer, barometer, radar, Custom by plugin | Small each, schema change each |
| 6 | **Legged robots have no controller.** A class users ask for first after arms | `lagrange_example_quadruped` trots open loop; no balance control, no terrain or contact sensing | Medium |
| 7 | **Electrical elements between buses.** A fuse, a relay or a real harness cannot be read from a schematic, and most real schematics have them | Averaged network of buses | Medium; drafted as `rfcs/0010-electrical-nodes.md`, waiting on the owner |
| 8 | **Tier 2 is thin.** Hardware ray-traced lidar is detected and not used; batched physics absent | Tier 1 BVH lidar, raster camera | Large |
| 9 | **Regions are headless only.** No ROS or viewer, no robot moving between regions | `RegionHost` in one process | Large |
| 10 | **Marine is buoyancy and drag.** No added mass, waves or currents; not Fossen as the plan says | `lagrange_hydrodynamics` | Medium |
| 11 | **Radiation has no module.** The zero substitute only, though it is a named domain | Design document | Medium |
| 12 | **Release gaps.** Binary packages are built locally, publishing and signing are the owner's | `packaging/` | The owner's |

Smaller things seen on the way: invariant 12 still lists PhysX, dropped by the owner at v0.6; under
ASan a controller takes over 10 s to load, so the spawner gives up and ros2_control is not cleared by
the sanitizers; `test_server`'s wrist camera test does not finish under ASan on software Vulkan;
Export Modelica needs a protocol command; `lagrange export spice` drives do not read back into the
importer.

## Recommendation

1, 2 (the wrapper over processes) and 3 were done the same night (#232, #233, #234), and 6 has a
first example (#235). Next, 5 widens what the simulator can be used for. The owner has no hardware, so 4 stays formulas, ngspice and published data. and about 7, whose draft RFC has questions for them. Leave GPU batched physics until someone needs
thousands of environments: integrating MuJoCo Warp or similar is a project of its own, and its status
has to be verified first (`TODO(verify)`).
