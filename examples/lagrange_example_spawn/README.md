# Adding and removing robots

An empty run with people walking. A node adds a robot and takes it out with the standard `simulation_interfaces` services.

```bash
ros2 launch lagrange_example_spawn spawn.launch.py                         # with the viewer
ros2 launch lagrange_example_spawn spawn.launch.py viewer:=false once:=true  # without: stops at its result
```

The result it logs, on the machine it was checked on:

```
RESULT: entities 4 -> 5 with /visitor -> 4 after deleting it
```

Where to look: `launch/spawn.launch.py`, `scripts/spawn_robot.py`.

What it shows: `--robot none`, `--humans`, `/spawn_entity`, `/get_entities`, `/delete_entity`.
