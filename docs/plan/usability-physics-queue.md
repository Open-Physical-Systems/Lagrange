# Usability and sensor physics queue

Asked for by the owner on 2026-10-31. Started: 2026-10-31 22:06 JST. Budget: 9 hours; items first, then a code
check and optimizations with what is left. One PR per item into `dev`, tested locally.

Out of scope, by the owner: Nav2 work (the simulator is low level and is not tied to a stack), and
electrical schematics in the run file (later).

| # | Item | State |
|---|---|---|
| 1 | **One run file.** `lagrange run.yaml`: robot (URDF or USD), optional world, and what to simulate. Robot-only needs no domain list; a robot or world on the command line needs no file at all | todo |
| 2 | **Any robot, joint by joint.** Low-level joint commands (position, velocity, effort) over ROS for any robot, with no controller or demo driver needed | todo |
| 3 | **Runtime site parameters.** Gravity, wind speed, temperature and the like, easy to change while running (viewer, command line, ROS) | todo |
| 4 | **Lidar intensity from materials.** Reflectivity, incidence angle and range, with parameters users can set | todo |
| 5 | **Water.** Water in a world: drawn, seen by sensors as water is (a lidar's return off water), and floating or dragging what enters it | todo |
| 6 | **Lidar motion distortion.** A moving robot's scan skews | todo |
| 7 | **Sensor timing.** Latency and jitter | todo |
| 8 | **Importer audit.** URDF damping and friction, mimic joints, ellipsoid colliders, SDF joints between models, MJCF types | todo |
| 9 | **Motor models.** Velocity and voltage control, motors on prismatic joints | todo |
| 10 | **Code check**, if time is left: check everything, make the optimizations found | todo |
