# Rotors

Each joint with `RotorAPI_1` is a propeller. Every 1 ms the rotor domain (`lagrange_rotor_aero`)
reads the joint's speed and the hub's motion, works out its thrust and its drag torque, and
puts them on the body and the joint. Its fidelity is set by the profile's
`rotor_aero.fidelity`: `coefficients` or `bem`.

```mermaid
flowchart LR
    W["joint speed ω<br/>(rigid body)"] --> F{"fidelity"}
    H["hub velocity, air at the disc<br/>(rigid body, aerodynamics)"] --> F
    F -->|coefficients| C["T = CT·ρ·n²·D⁴<br/>Q = CP·ρ·n²·D⁵ / 2π"]
    F -->|bem| B["blade elements on 40 annuli<br/>from the propeller file"]
    C --> G["× ground effect<br/>× ceiling effect"]
    B --> G
    G --> O["thrust and drag torque<br/>on the propeller"]
    G --> S["rotor/&lt;j&gt;/thrust,<br/>induced_velocity, thrust_axis"]
```

## Coefficients

With $n$ the speed in revolutions per second, $D$ the diameter, $\rho$ the air density and
$C_T$, $C_P$ the rotor's `aero:rotor:thrustCoeff` and `powerCoeff`:

$$
T = C_T\,\rho\,n^2 D^4, \qquad Q = \frac{C_P\,\rho\,n^2 D^5}{2\pi}
$$

The thrust points along the joint axis when the rotor turns its own way
(`aero:rotor:direction`). The drag torque always opposes the rotation, which is what yaws a
multirotor when its cw and ccw rotors turn at different speeds. The air applies it to the
propeller alone; the motor drives the propeller against it, and the motor's reaction on the
frame is what turns the drone.

$\rho$ is the site's air density (`--air-density`), or with the weather running its own, from
the temperature, pressure and humidity, read every step: on a hot day at altitude the rotors
turn faster for the same thrust ($\omega \propto \rho^{-1/2}$), and draw more power. The
built-in controller follows the same density. Rain pushes the frame down a few millinewtons
([aerodynamics.md §3.1](../design/domains/aerodynamics.md)).

These are static coefficients: thrust doesn't change with climb, descent or forward speed. A
profile's `rotor_aero.inflow_time_constant` (s) can make the thrust follow its target with a
first-order lag. The induced velocity that the downwash uses comes from momentum theory,
$v_i = \sqrt{T / (2\rho A)}$.

The Holybro sample uses $C_T = 0.125$ and $C_P = 0.0507$, the UIUC database's static values for
an APC 10x4.7 SF.

## Blade-element momentum (`bem`)

The propeller file (`aero:rotor:propeller`, [formats.md](../design/formats.md) §3.3.1) gives
the blade's chord and twist along the radius, its section lift and drag, its blade count and
its hand. For each of 40 annuli, the solver finds the induced velocity $w$ at which the blade
elements' force equals the momentum given to the air through that ring:

$$
dT = 4\pi r\,\rho\,F\,|V + w|\,w\,dr
$$

$F$ is Prandtl's tip-loss factor and $V$ the axial speed of the air arriving at the disc. The
swirl gives the torque the same way. So thrust now changes with what the air does:

- **Climbing:** air arrives from above, the blades see a smaller angle, and thrust drops at the
  same speed.
- **Forward flight and wind:** the advancing blade sees more air than the retreating one. This
  gives translational lift, an in-plane H-force and a hub moment, averaged over 12 blade
  positions.
- **Vortex ring state:** when the rotor sinks into its own wake, momentum theory has no
  solution. Between $0 < V_d/v_h < 2$ each annulus takes NeuroBEM's empirical fit (Bauersfeld
  et al. 2021, eqs. 18–19):
  $\tilde v / v_h = 1 + 1.125x - 1.372x^2 + 1.718x^3 - 0.655x^4$, with $x = V_d / v_h$.
- **Wrong way round:** sections are defined over the full circle of angles, so a propeller on
  backwards or spun the wrong way gives little thrust for much power, as a real one does.

The propeller is solved once at load over every inflow angle, about 1 s per propeller, and each
step only looks values up. So `bem` costs little more than `coefficients` while running.

The details are in [aerodynamics.md](../design/domains/aerodynamics.md) §2.1.

## Ground and ceiling effect

Both fidelities apply these when the profile has `ground_effect: true`.

**Ground effect** (Cheeseman and Bennett, 1955), with $z$ the hub's height above the ground
plane and $R$ the radius:

$$
\frac{T}{T_\infty} = \frac{1}{1 - \left(\dfrac{R}{4z}\right)^2}, \qquad z \ge 0.5\,R
$$

It is +33 % at half a radius (the clamp), +6.7 % at one radius and +1.6 % at two.

**Ceiling effect** (Hsiao and Chirarattananon 2019): a ray up each rotor's axis finds the
nearest fixed box of the scenery at distance $d$. With $\delta = R/d$ and $\alpha_0 = 1.6$:

$$
\gamma = \tfrac12 + \tfrac12\sqrt{1 + \alpha_0\,\delta^2/8}, \qquad
\frac{T}{T_\infty} = \gamma^{2/3}
$$

That's +11 % at half a radius, +3 % at one and under 1 % beyond two. It is left out beyond
6 R. $\alpha_0$ was fitted for a 23 mm propeller, so it is unchecked for larger ones.

## How far to trust it

| Check | Result | Test |
|---|---|---|
| `bem` against NACA Report 640's measured propeller | CT within 2–7 %, CP within 0–7 % over the attached-flow range | `NacaReport640.*` in `test_bem` |
| `bem` against ideal momentum theory | within 3–4 % | `Bem.IdealRotorMeetsMomentumTheory` |
| Vortex ring state | NeuroBEM's fit exactly; thrust joins hover's within 0.1 % | `Bem.TheVortexRing*` |
| Against measured quadrotor flights (NeuroBEM, 13 held-out segments) | `coefficients` 1.46 N RMS (their own fit 1.49 N); `bem` 1.12 N (their BEM 0.98 N) | `tools/validation/neurobem.sh` |
| Ground effect | equals Cheeseman–Bennett for z/R from 0.5 to 3; **not yet compared with measurements** | `test_rotor_aero` |
| Static thrust against published propeller data | **owed** | — |

What isn't modelled: wake skew inside the disc (the fore-aft inflow gradient in forward flight),
blade flapping of rigid propellers beyond the hub moment, stall delay on rotating blades, and
Reynolds-number effects beyond what the section data cover.
