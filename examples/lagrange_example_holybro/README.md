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

## Wind and weather, from the viewer

In the viewer, open the **Environment** workspace (top bar). Type a value and press Enter (or
click away): it applies at once, and the run goes on.

- **Wind**: the **mean** row has two fields, the speed in m/s and where the wind comes from in
  degrees (0 north, 90 east, 180 south, 270 west; the world's +y is north). An east wind pushes
  the drone west. Lagrange's controller holds its position, so it leans into the wind rather
  than drifts: at 5 m/s the Holybro sits about 1.5 cm downwind.
- **Weather**: it starts once the site has its air. Give the **ambient** temperature (Time and
  sun), the **pressure** (about 1013 hPa at sea level) and the **humidity**, then press **Start
  weather**. Then **rain rate** (mm/h) and **fog visibility** (m; 10000 or more is clear air)
  can be typed in. Rain and fog act on the sensors (lidars, cameras) and the surface's friction.

What the weather does to the drone:

- **The air's density** comes from the temperature, pressure and humidity, and the rotors fly
  in it: in hot, thin air they spin faster for the same lift (about 5690 rpm at 40 °C and 950
  hPa, 4990 rpm at −10 °C and 1030 hPa, for the sample X500) and the battery drains faster.
- **Rain and snow** push it with the drops that hit it. That is real but small: heavy rain (25
  mm/h) pushes the X500 down about 4 mN against its 20 N weight. The wind is what you will feel.
- **Wind and gusts** as above. In the thunderstorm preset's 18 m/s (at 10 m; about 13 m/s at
  2 m) the simulated controller still holds within about 0.2 m, leaning hard into it.

In the realistic view (the default on a GPU) rain and snow fall through the scene, carried by
the wind, and the ground turns wet or white with them.

Or one click: the **Presets** tab beside Environment has whole weathers (clear and calm,
breezy, strong wind, light rain, heavy rain, thunderstorm, fog, snow). Each sets the
temperature, the air, rain, fog, snow and the wind's speed, and starts the weather if it was
off; the wind keeps the direction you gave it. You can change any value after in the
Environment tab.

The viewer shows the pressure and humidity it sent (the server does not report them back), so
a viewer opened later shows them empty.

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
