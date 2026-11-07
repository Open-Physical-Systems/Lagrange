# Your own drone, by keyboard

The Holybro X500 V2 (PX4's model, with its motors, 4S battery and APC 10x4.7 rotors from
`holybro_x500_v2.overlay.yaml`) on the open field's landing pad. It waits on the ground until
you take off: `teleop_twist_keyboard` opens in a window of its own, `t` climbs, and the other
keys fly it over `/cmd_vel`; `b` brings it down, and on the ground its motors stop. The viewer
follows it, and a node reports its height, speed, climb and heading once a second.

Once, to fetch the meshes from PX4's repository (network):

```bash
ros2 run lagrange_assets fetch_assets holybro_x500_v2
```

Then:

```bash
ros2 launch lagrange_example_holybro teleop.launch.py                          # viewer + teleop window
ros2 launch lagrange_example_holybro teleop.launch.py teleop:=false            # teleop in your own terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
ros2 launch lagrange_example_holybro teleop.launch.py viewer:=false once:=true  # headless: stops at its result
```

## The downwash

The viewer shows the air the rotors push down from the start: particles in the 3D view, coloured
by speed, and a vertical slice of the air speed through the drone in the **Environment**
workspace (top bar → Environment). Two ways to compute it:

```bash
ros2 launch lagrange_example_holybro teleop.launch.py              # air:=fitted, the default
ros2 launch lagrange_example_holybro teleop.launch.py air:=live    # computed live, RTX GPU
ros2 launch lagrange_example_holybro teleop.launch.py airflow:=false   # don't draw it
```

| `air:=` | What computes the air | Speed |
|---|---|---|
| `fitted` | the wake model fitted to measurements (jet, ground outwash, walls), from the rotors' thrust | real time, any machine |
| `live` | a lattice Boltzmann box 3 m across that follows the drone, with blade-element rotors (`quad_x500_live_air.yaml`, a 10 inch propeller) | about 0.4–0.75× real time on an RTX 4060 Laptop; the fitted model without a hardware GPU |

Fly low (`b`) to see the outwash ring spread on the ground, and forward (`i`) to see the wake
lean back.

What the particles are: tracers the viewer moves through the air the server computes, 20 times a
second, in a 5.4 m box round the drone. They show the field; the field is what pushes other
robots and what other drones' rotors fly in. A drone's own rotors don't fly in its own fitted
wake: their thrust model already describes an isolated rotor.

In forward flight the wake trails behind and below at a slant: the air the rotors push down
keeps going down while the drone moves on (Leishman's wake skew, tan χ = μ/λ). That is the
physics, not lag. What the fitted model leaves out is memory: its wake is worked out afresh each
step from the rotors' thrust and the drone's speed, so the air the drone left behind stops at
once instead of drifting down for a few seconds, and a sudden stop or turn moves the whole wake
at once. `air:=live` has that memory: the air in its box is a fluid that carries on moving. How each is computed: [Downwash](../../docs/physics/downwash.md).

## Keys

Keys, with the teleop window focused. Take off with `t`:

| Key | What it does | `/cmd_vel` |
|---|---|---|
| `i` / `,` | forward / back | `linear.x` (m/s, in its heading) |
| `J` / `L` (shift) | left / right | `linear.y` |
| `t` / `b` | up (take off) / down (land, on the ground) | `linear.z` |
| `j` / `l` | turn left / right | `angular.z` (rad/s) |
| `k` or any other key | stop: hold where it is | all zero |
| `q` / `z` | all speeds 10 % faster / slower | |

teleop sends a command only when a key is pressed, and the drone holds where it is when none
comes for 0.5 s. So **hold the key down** to keep moving (the keyboard's repeat sends it on);
one tap moves it for half a second. teleop's default speed is 0.5 m/s; press `q` a few times to
go faster.

With `once:=true` the node takes off itself (1 m/s up). The result it logs, on the machine it
was checked on:

```
RESULT: took off on /cmd_vel and climbed to 1.50 m above the start
```

Then once a second, while you fly:

```
t    9.1 s  height  1.99 m  speed 2.04 m/s  climb +0.01 m/s  heading  +67.7 deg
```

Where to look: `launch/teleop.launch.py` (the server's options, the viewer's follow camera, the
teleop window: ptyxis, gnome-terminal, konsole or xterm, whichever is installed),
`scripts/watch.py`.

What it shows: `--robot` with `--overlay`, `--no-demo` (the drone waits for you), `lagrange run PROFILE` (air:=live), `--overlays airflow`, `--slice`, `lagrange-asset://` meshes, `--air-density`,
`--geo-origin`, `--follow`, `teleop_twist_keyboard` with `stamped:=true`.

For your own drone, change `ROBOT` and the overlay in the launch file. The
[tutorial for your own drone](../../docs/guide/tutorial-custom-drone.md) goes further: blade
elements, downwash, a building, live air.
