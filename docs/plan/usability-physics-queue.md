# Usability and sensor physics queue

Asked for by the owner on 2026-10-31. Started: 2026-10-31 22:06 JST. Budget: 9 hours; items first, then a code
check and optimizations with what is left. One PR per item into `dev`, tested locally.

Out of scope, by the owner: Nav2 work (the simulator is low level and is not tied to a stack), and
electrical schematics in the run file (later).

| # | Item | State |
|---|---|---|
| 1 | **One run file.** `lagrange run.yaml`: robot (URDF or USD), optional world, and what to simulate. Robot-only needs no domain list; a robot or world on the command line needs no file at all | done |
| 2 | **Any robot, joint by joint.** Low-level joint commands (position, velocity, effort) over ROS for any robot, with no controller or demo driver needed | done |
| 3 | **Runtime site parameters.** Gravity, wind speed, temperature and the like, easy to change while running (viewer, command line, ROS) | done |
| 4 | **Lidar intensity from materials.** Reflectivity, incidence angle and range, with parameters users can set | done |
| 5 | **Water.** Water in a world: drawn, seen by sensors as water is (a lidar's return off water), and floating or dragging what enters it | done |
| 6 | **Lidar motion distortion.** A moving robot's scan skews | done |
| 7 | **Sensor timing.** Latency and jitter | done |
| 8 | **Importer audit.** URDF damping and friction, mimic joints, ellipsoid colliders, SDF joints between models, MJCF types | todo |
| 9 | **Motor models.** Velocity and voltage control, motors on prismatic joints | done |
| 10 | **Code check**, if time is left: check everything, make the optimizations found | done |

Code check, as run on 2026-11-01. Fixed in #202 to #205. Sanitizers: ASan and UBSan report nothing in
Lagrange's code. TSan reports only inside ROS 2 control, Fast CDR and Vulkan's validation layer; its
reports through TBB and OpenUSD, which are not instrumented here, were set aside unread, and
`test_server` did not finish under it, so it does not clear those paths.

Looked at and left, with the reason: camera frames are copied a few times per frame, small next to
rendering, and readers hold them by shared pointer; bus commits copy large arrays, a change to the core
of the bus for a gain nobody has measured. No profiler runs on this machine (perf and ptrace are
blocked), so the performance work rests on reading the code and on before-and-after speeds.
