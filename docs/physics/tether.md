# Tethers

A cable from an anchor to a body (`TetherAPI_1`, RFC 0011), such as a power tether from a ground
station's winch to a drone. `lagrange_tether` runs every 1 ms. The cable sags, swings, goes taut
and drags in the wind. It lies on the ground, catches on edges and stays caught, and wraps and
tangles on itself. It hits the drone and any other body, and pulls on both ends. Each body it
touches or is tied to gets one wrench, `body/<b>/wrench/tether`.

```mermaid
flowchart LR
    RB["every collider at its body's pose<br/>(rigid body)"] --> C
    WIND["env/wind/velocity<br/>(aerodynamics)"] --> C
    DW["tether/n/downwash<br/>(aerodynamics, with rotors)"] --> C
    C["the cable: masses a segment apart<br/>20 substeps a step"] --> T["tether/n/tension_attach,<br/>tension_anchor, length, nodes ..."]
    C --> W["body/b/wrench/tether<br/>(pull at the ends, pushes at contacts)"]
    WINCH["winch: fixed, tension, free"] --> C
```

## Using it

On any robot, from an overlay (an overlay's new prims have no pose, so `anchorPoint` places the
station, here in world coordinates):

```yaml
schema_version: 1
apply:
  - create: ground_station
    schemas:
      TetherAPI_1:
        tether:anchorPoint: [2.0, 0.0, 0.3]
        tether:attach: base_link
        tether:attachPoint: [0.0, 0.0, -0.1]
        tether:length: 2.0           # paid out at the start
        tether:maxLength: 50.0       # on the drum
        tether:diameter: 0.006
        tether:linearDensity: 0.03
        tether:winch: tension
        tether:winchTension: 4.0
```

Or with no file: `lagrange_server ... --tether X,Y,Z,LINK[,LENGTH]` ties LINK to the world point
X,Y,Z, its length the distance plus 10 % unless given. Every attribute and its fallback is in
[formats.md](../design/formats.md) §3.12. The domain is `tether`; `--domain tether=off` (or the
profile's `domains.tether`) is the zero substitute: tensions 0, no cable, no force. Air drag needs
the site's air density (`--air-density`); without it the cable hangs in still air with no drag.

## The equations

**Masses and segments.** The cable is point masses about `segmentLength` apart, each carrying
half of the two segments beside it, $m_i = \mu (\ell_{i-1} + \ell_i) / 2$, with $\mu$ the mass
per metre. The first is held at the anchor, the last at the attach point.

**Extended position-based dynamics with small steps** (Macklin et al. 2019). Each 1 ms step is
split into $n$ substeps of $h$ (default 20). In each, every free mass is predicted under gravity
and drag, $v \leftarrow v + h\,(g + F/m)$, $p = x + h v$. Then each segment's length constraint
$C = |p_k - p_i| - \ell$ is corrected once, forward along the cable and back:

$$
\Delta\lambda = \frac{-C - \tilde\alpha\,\lambda - \gamma\,\nabla C\cdot(p - x)}{(1+\gamma)(w_i + w_k) + \tilde\alpha},
\qquad \tilde\alpha = \frac{\ell}{EA\,h^2},\quad \gamma = \frac{\ell}{EA}\frac{\beta}{h},
$$

where $w$ are the inverse masses and $\beta = 2\zeta\sqrt{k m}$ is the segment's axial damping
($k = EA/\ell$, $\zeta$ the damping ratio). A cable cannot push: $\lambda$ is kept at or below
zero, so a slack segment does nothing. The segment's tension is $T = -\lambda / h^2$. Then come
the bend term (when `bendStiffness` is set, a pinned beam's $48EI/s^3$ across each pair of
segments), the contacts, and the velocities $v = (p - x)/h$.

**Air drag.** Each segment of length $L$ and diameter $d$ is a cylinder in the air moving at
$u$ relative to it, split into $u_n$ across and $u_t$ along:
$F = \tfrac12 \rho\, C_d\, d\, L\, |u_n|\, u_n + \tfrac12 \rho\, \tfrac{C_d}{50}\, d\, L\, |u_t|\, u_t$,
half to each end. The air is the site's wind (ground truth and gusts, from the aerodynamics
domain) plus, when the run has rotors, their downwash at the segment: the mean of what the
aerodynamics domain writes at its two masses.

**Downwash.** The aerodynamics domain builds every multirotor's wake (near wake of each rotor,
merged jet, outwash along the ground; [rotor_aero.md](../design/domains/rotor_aero.md) §2). Each
step it reads `tether/<n>/nodes`, evaluates every robot's wake at each mass (the drone's own
too: the cable is not part of its airframe) and writes `tether/<n>/downwash`, one velocity per
mass in the same layout. The tether module reads it a step late (1 ms, a feedback input), at
last step's masses; it matches them from the attach end, so a mass the winch adds or takes at
the anchor leaves the others where they were. A cable crossing a wake of speed $w$ at right
angles feels $\tfrac12\rho C_d d\,w^2$ per metre; one hanging along it, only the tangential
fiftieth. Without rotors, with `aerodynamics.downwash: false` or with the domain off, the signal
is not there and the drag is exactly as before.

**Contacts.** Every mass is a sphere of the cable's radius. Against each collider near the cable
(every shape type, at its body's pose this step) the module finds the signed distance, normal and
nearest surface point:
- **Out.** A mass inside is moved out along the normal. Beyond undoing its own move in that
  substep, it comes out at most 1 m/s, so a cable laid through something is not thrown out of it.
- **Friction.** Coulomb, as a limit on the sliding over the substep (Macklin et al. 2014): within
  μ times the push-out it sticks (`caught`), beyond it slides.
- **Equal and opposite.** On a moving body, the mass's move times its mass over $h^2$ is the
  force on the body at the contact point, so the cable pushes the drone it hits and the drone
  pushes back.
- **Along each segment.** Where the cable bends over an edge or a corner, the straight segment
  between two masses outside a shape can still cut into it. For each segment that could reach a
  shape, the point along it that goes deepest is found (golden-section search: the distance
  into a convex shape along a line is convex) and pushed out, the two masses sharing the move,
  with friction and the reaction on a moving body as for a mass.
- **Itself.** Segments that are not neighbours keep a diameter apart (closest points of two
  capsules). That is how it wraps and knots.

Within 0.5 m of either end the cable does not touch the robot that end is tied to, nor a shape
that holds the end, because it starts inside them (the drone's hull around its attach point).
Moving bodies are met first and scenery last, and a body feels only how far a mass ends up moved
away from it: a cable squeezed between a skid and the ground carries nothing between them.

**Laying it out.** On its first step each cable is laid between its ends where the bodies are
then, in an arch of its length above them in the vertical plane through them, and falls into
place. (A sag below them would start the slack inside the ground, slab or pad it should lie on;
a mass inside a thin box resting on another can be pushed between the two and never get out.)
Where the arch goes through something, it comes out at 1 m/s.

**The pull on each end.** The end segment's tension along it, plus the weight and drag of the
half segment the end carries, at the attach point; with the contact pushes, the body's wrench.

**The winch.** It pays cable out at the anchor or reels it in, at most `winchSpeed`. A new mass
appears on the first segment each time it grows past 1.5 segments, and one goes back on the drum
below 0.5. `tension` holds `winchTension` at the anchor (proportional and integral on the
anchor's tension, filtered over 50 ms). `free` pays out whatever pulls over 0.2 N and reels
nothing in. `fixed` holds the length. Paying out stops at `maxLength`.

**Commanding the winch while it runs.** Three inputs, each optional and ignored at 0 or below,
let a plugin, or ROS through `--ros-topics`, run the winch like an operator or a feeder:
`tether/<n>/length_command` (m) pays out or reels in to that length; else
`tether/<n>/tension_command` (N) holds that tension at the anchor, as `tension` does; else it does
what `tether:winch` says. `tether/<n>/winch_speed_command` (m/s) replaces `winchSpeed` as the
most it reels at. For example, to hold a tension sent on a topic:

```yaml
# --ros-topics winch.yaml
subscribe:
  - {signal: tether/tether_0/tension_command, topic: /winch/tension, type: std_msgs/msg/Float64}
  - {signal: tether/tether_0/winch_speed_command, topic: /winch/speed, type: std_msgs/msg/Float64}
```

Until a message comes the input reads 0, so the winch does what its parameters say. A winch
slower than the drone flies away jerks it, so give it at least the drone's top speed.

## What it writes

| Signal | Unit | |
|---|---|---|
| `tether/<n>/tension_attach`, `tension_anchor` | N | at each end |
| `tether/<n>/length` | m | paid out |
| `tether/<n>/winch_speed` | m/s | positive paying out |
| `tether/<n>/contacts` | | masses touching anything this step |
| `tether/<n>/caught` | | held somewhere by static friction |
| `tether/<n>/nodes` | m | x, y, z of each mass, world; unused entries NaN |
| `tether/<n>/downwash` | m/s | read, written by the aerodynamics domain: the rotors' air at each mass, world, same layout |
| `body/<b>/wrench/tether` | N, N·m | for each body it is tied to or touches |
| `tether/<n>/length_command`, `tension_command`, `winch_speed_command` | m, N, m/s | read, optional: the winch's commands (≤ 0: none) |

## Seeing it

The viewer draws every cable as an object in the viewport, always: its centre line and four lines
at its radius, a dark rubber colour, depth-tested like the meshes. The server sends each cable in
every scene delta (`Overlay` `KIND_CABLE`: radius, then the points), so a viewer that predates it
connects and shows no cable.

## How far to trust it

| Check | Against | Result | Test |
|---|---|---|---|
| A 12 m cable hung over 10 m | the catenary | sag 0.03 %, end tension 0.5 % | `Cable.HangsAsACatenary` |
| A taut string plucked | its fundamental, $\sqrt{T/\mu}/2L$ | 0.3 % | `Cable.ATautStringRingsAtItsFundamental` |
| Sliding over a fixed drum, μ 0.3, half a turn | the capstan equation, $e^{\mu\theta}$ | 0.35 % | `Cable.SlidesOverACylinderAsTheCapstanEquationSays` |
| Draped over a body's box | the weight held by the box and both ends | 0.7 % | `Cable.PushesABodyAsMuchAsTheBodyHoldsItUp` |
| 6 m piled on the ground | no segment within a diameter of another, none in the ground | holds | `Cable.PilesUpWithoutPassingThroughItself` |
| A tension winch, drone climbing | its set point, on average | 2.00 N of 2.00 | `Cable.ATensionWinchHoldsItsTension` |
| The sample quad hovering on 3 m of tether | the weight held by the station and the drone | 0.5 % over it (its downwash included) | `TetherRun.AQuadcopterHoversOnItsTether` |
| A 2 m taut cable across air blowing down at 10 m/s | $\tfrac12\rho C_d d v^2 L$; on the attach half, half | 0.04 %, 0.05 % | `Cable.ADownwashAcrossItPushesWithItsDynamicPressure` |
| The sample quad's tether 0.32 m under rotor 3, across its slipstream | the actuator disc's closed form (rotor_aero.md §2.9) | 0.0529 N held up for 0.0537 N, 1.5 % | `TetherRun.TheDronesDownwashPushesItsTetherDown` |
| Laid out through a slab | comes out at 1 m/s | 1.26 m/s, at most 14 N | `Cable.LaidThroughABoxItComesOutSlowly` |
| Over a slab's edge, out, down and back in under it | the centre line stays out of the slab | at worst 1.3 mm in, within its 2.5 mm radius (centimetres with contacts at the masses alone) | `Cable.BendsOverAnEdgeWithoutCuttingIntoIt` |
| A fixed winch commanded to 5 m at 2 m/s, then 3 m, then a 3 N tension | each length and its speed, then the tension | 5.00 m and 3.00 m at 2 and 1 m/s, 3.00 N | `Cable.TheWinchFollowsCommandsWhileItRuns` |
| The sample quad flying sideways at 2.5 m/s and back, tied 0.15 m off centre to a 4 N tension winch | turned by the cable but flying (built-in controller) | flying, upright within 33°; the controller before 1.4 turned over and fell | `Multicopter.TurnedByItsTetherItStillFlies` |
| Lifted off a pad on a slab by a drone, tension winch at 2.5 N | pays out, tension near its set point | 2.6 m out, 2.9 N mean | `Cable.LiftedOffAPadTheWinchPaysOut` |

**Limits.**

- A cable tied off the drone's centre steers its yaw, and the propellers answer yaw only weakly
  (a few tenths of a N·m on a 500 mm quad). The built-in controller then gives up heading and
  keeps lift and attitude, but past about two and a half times what the propellers can answer
  (on the sample quad, 5 N pulled 0.15 m off centre) it sinks up to 1.8 m before it recovers.
  Tie the cable near the centre, or through a joint that lets the frame turn under it.
- The downwash is taken at the masses. A wake column narrower than a segment (a rotor's near
  wake is 0.18 m across under the sample quad; the default segment is 0.2 m) pushes only the
  masses that sit in it, so more or less than it should depending on where they fall (right
  over many placements, not in one): use a shorter `segmentLength`, 5 cm in the check above,
  for a cable that crosses one. The air the cable pushes back on the rotors is
  not modelled.
- Twist and bending stiffness beyond the light bend term are not modelled: the model is for limp
  cables.
- Turns of the cable do not rub on each other (no friction between them).
- Two tethers do not touch each other.
- A convex hull's distance near an edge is its farthest face plane's.
- A nearly straight tether is stiff along itself, so each payout of a tension winch sends a pulse
  of tension along it; the mean holds the set point, a single step may not.
- No breaking, and no conductor heating yet (`tether:resistancePerLength` is read but not used).
