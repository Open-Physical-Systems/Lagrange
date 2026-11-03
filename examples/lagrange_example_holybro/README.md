# Your own drone, by keyboard

The Holybro X500 V2 (PX4's model, with its motors, 4S battery and APC 10x4.7 rotors from
`holybro_x500_v2.overlay.yaml`) over the open field. It takes off by itself and hovers 2 m up.
`teleop_twist_keyboard` opens in a window of its own and flies it over `/cmd_vel`. The viewer
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

Keys, with the teleop window focused, once the drone hovers:

| Key | What it does | `/cmd_vel` |
|---|---|---|
| `i` / `,` | forward / back | `linear.x` (m/s, in its heading) |
| `J` / `L` (shift) | left / right | `linear.y` |
| `t` / `b` | up / down | `linear.z` |
| `j` / `l` | turn left / right | `angular.z` (rad/s) |
| `k` or any other key | stop: hold where it is | all zero |
| `q` / `z` | all speeds 10 % faster / slower | |

teleop sends a command only when a key is pressed, and the drone holds where it is when none
comes for 0.5 s. So **hold the key down** to keep moving (the keyboard's repeat sends it on);
one tap moves it for half a second. teleop's default speed is 0.5 m/s; press `q` a few times to
go faster.

The result it logs, on the machine it was checked on:

```
RESULT: hovering 1.98 m above the start at x 0.00, y -0.00; fly it with teleop (i forward, t up, b down, j/l turn)
```

Then once a second, while you fly:

```
t    9.1 s  height  1.99 m  speed 2.04 m/s  climb +0.01 m/s  heading  +67.7 deg
```

Where to look: `launch/teleop.launch.py` (the server's options, the viewer's follow camera, the
teleop window: ptyxis, gnome-terminal, konsole or xterm, whichever is installed),
`scripts/watch.py`.

What it shows: `--robot` with `--overlay`, `lagrange-asset://` meshes, `--air-density`,
`--geo-origin`, `--follow`, `teleop_twist_keyboard` with `stamped:=true`.

For your own drone, change `ROBOT` and the overlay in the launch file. The
[tutorial for your own drone](../../docs/guide/tutorial-custom-drone.md) goes further: blade
elements, downwash, a building, live air.
