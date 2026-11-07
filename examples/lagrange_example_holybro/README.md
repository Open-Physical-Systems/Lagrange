# Your own drone, by remote

The Holybro X500 V2 (PX4's model, with its motors, 4S battery and APC 10x4.7 rotors from
`holybro_x500_v2.overlay.yaml`) on the open field's landing pad. It waits on the ground until
you take off. The TX16S-style remote ([`lagrange_tx16s`](../../utils/lagrange_tx16s/README.md))
opens in a window of its own: a real RadioMaster TX16S over USB, or the keyboard. Arm it, push
the throttle up and it climbs; the sticks fly it over `/cmd_vel`; bring it down and on the ground
its motors stop. The viewer follows it, the remote's screen shows its height and speed, and a
node reports its height, speed, climb and heading once a second.

Once, to fetch the meshes from PX4's repository (network):

```bash
ros2 run lagrange_assets fetch_assets holybro_x500_v2
```

Then:

```bash
ros2 launch lagrange_example_holybro teleop.launch.py                          # viewer + the remote
ros2 launch lagrange_example_holybro teleop.launch.py teleop:=keyboard         # teleop_twist_keyboard instead
ros2 launch lagrange_example_holybro teleop.launch.py teleop:=false            # your own
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

Fly low to see the outwash ring spread on the ground, and forward to see the wake lean back.

What the particles are: tracers the viewer moves through the air the server computes, 20 times a
second, in a 5.4 m box round the drone. They show the field; the field is what pushes other
robots and what other drones' rotors fly in. A drone's own rotors don't fly in its own fitted
wake: their thrust model already describes an isolated rotor.

In forward flight the wake trails behind and below at a slant: the air the rotors push down
keeps going down while the drone moves on (Leishman's wake skew, tan χ = μ/λ). That is the
physics, not lag. What the fitted model leaves out is memory: its wake is worked out afresh each
step from the rotors' thrust and the drone's speed, so the air the drone left behind stops at
once instead of drifting down for a few seconds, and a sudden stop or turn moves the whole wake
at once. `air:=live` has that memory: the air in its box is a fluid that carries on moving.
How each is computed: [Downwash](../../docs/physics/downwash.md).

## Flying it

With the remote (the default), click its window so it has the keyboard, then:

| On a TX16S (Mode 2) | Keyboard | Does |
|---|---|---|
| SF up | `Space` | arm: the remote sends `/cmd_vel` only while armed |
| left stick up / down | `W` / `S` | climb (take off) / descend (land, on the ground) |
| left stick left / right | `A` / `D` | turn left / right |
| right stick up / down | `↑` / `↓` or `I` / `K` | forward / back |
| right stick left / right | `←` / `→` or `J` / `L` | left / right |
| SA | `R` | rate: 30 %, 60 %, 100 % of full stick |
| | `Esc` | disarm at once |

The keyboard's sticks spring back to the centre when you let go, the throttle too: the centre
holds the height. A real TX16S's throttle stays where you leave it, so there its centre is the
hover. Speeds at full stick are 5 m/s, 2 m/s up and 1.5 rad/s (the remote's panel changes them),
times the rate. The remote's README has the USB setup and the joystick map:
[`lagrange_tx16s`](../../utils/lagrange_tx16s/README.md).

With `teleop:=keyboard`, `teleop_twist_keyboard` opens in a terminal instead:

| Key | What it does | `/cmd_vel` |
|---|---|---|
| `i` / `,` | forward / back | `linear.x` (m/s, in its heading) |
| `J` / `L` (shift) | left / right | `linear.y` |
| `t` / `b` | up (take off) / down (land, on the ground) | `linear.z` |
| `j` / `l` | turn left / right | `angular.z` (rad/s) |
| `k` or any other key | stop: hold where it is | all zero |
| `q` / `z` | all speeds 10 % faster / slower | |

It sends a command only when a key is pressed, and the drone holds where it is when none comes
for 0.5 s, so hold the key down to keep moving.

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
remote, or the teleop terminal: ptyxis, gnome-terminal, konsole or xterm, whichever is
installed), `scripts/watch.py`.

What it shows: `--robot` with `--overlay`, `--no-demo` (the drone waits for you), `lagrange run PROFILE` (air:=live), `--overlays airflow`, `--slice`, `lagrange-asset://` meshes, `--air-density`,
`--geo-origin`, `--follow`, `lagrange_tx16s`, `teleop_twist_keyboard` with `stamped:=true`.

For your own drone, change `ROBOT` and the overlay in the launch file. The
[tutorial for your own drone](../../docs/guide/tutorial-custom-drone.md) goes further: blade
elements, downwash, a building, live air.
