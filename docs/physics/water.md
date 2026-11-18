# Water

A box of water (`WaterAPI_1`, `--water X,Y,LEVEL,SIZE_X,SIZE_Y,DEPTH` or the run file's
`water:`) floats, sinks and drags whatever moving body enters it. `lagrange_hydrodynamics` runs
every 1 ms: it cuts each body's colliders into small cells, finds which are under the surface, and
writes one wrench per body, `body/<b>/wrench/water`, with the share of it under water,
`body/<b>/submerged`. A vehicle with its own measured coefficients (`--hydro-coefficients
FILE.yaml`) takes Fossen's manoeuvring model instead of the cells' drag.

```mermaid
flowchart LR
    RB["body pose, twist<br/>(rigid body)"] --> CELLS["cells of each collider<br/>under the moving surface?"]
    WAVES["current, Airy waves<br/>(site)"] --> CELLS
    CELLS --> BU["buoyancy<br/>ρ g V per cell"]
    CELLS --> DR["drag<br/>½ ρ Cd A |v| v per cell"]
    CELLS --> AM["added mass, water's<br/>acceleration (Morison)"]
    COEF["--hydro-coefficients<br/>(e.g. BlueROV2)"] --> FOS["Fossen: M_A, C_A, D, D_n"]
    BU --> W["body/b/wrench/water"]
    DR --> W
    AM --> W
    FOS -->|"instead of drag<br/>and added mass"| W
```

## The equations

**Buoyancy and drag.** Each collider is cut into cells (6 a side, kept when their centre is
inside it, scaled to the shape's true volume). A cell under the surface is pushed up by the
water it displaces and dragged by its speed through the water $v$:

$$
F_b = \rho\,g\,V_{cell}, \qquad F_d = -\tfrac12\,\rho\,C_d\,A\,|v|\,v
$$

Because buoyancy acts at the wet cells, an off-centre or tilted hull rights itself. $C_d$ is
illustrative (1 by default), and the frontal area is taken as a box's.

**Moving water.** `--water-current VX,VY[,VZ]` is a steady current. `--water-waves
AMPLITUDE,PERIOD[,TOWARDS_DEG]` is one train of linear (Airy) waves over the water's depth $d$:

$$
\omega^2 = g\,k\,\tanh(k d), \qquad \eta = a\,\cos(k x - \omega t)
$$

with the water under the surface moving in the orbits of linear theory (Dean and Dalrymple
1991, ch. 3). Cells count as wet under the moving surface, and the drag works on a cell's speed
relative to the water there.

**Added mass.** A body that accelerates in water drags some water along, which acts as extra
mass. A submerged body gets Morison's inertia terms at its centre of mass,

$$
F_a = \rho\,V\,a_w + m_a\,(a_w - a_b), \qquad m_a = \tfrac12\,\rho\,V
$$

with $a_w$ the water's acceleration, $a_b$ the body's, and $m_a$ the added mass of a sphere in
potential flow (Lamb, §92), along every axis.

**Fossen's model.** A body named in `--hydro-coefficients` takes Fossen's model (*Handbook of
Marine Craft Hydrodynamics and Motion Control*, 2011, ch. 6) in its body frame, on its velocity
through the water $\nu_r = (u, v, w, p, q, r)$, scaled by the share of it under water:

$$
\tau = -M_A\,\dot\nu_r - C_A(\nu_r)\,\nu_r - \big(D + D_n(|\nu_r|)\big)\,\nu_r
$$

with a diagonal added mass $M_A = \mathrm{diag}(A_t, A_r)$, its Coriolis and centripetal terms
$-C_A\nu = (\omega \times A_t u,\; u \times A_t u + \omega \times A_r \omega)$, linear damping
$D$ and quadratic damping $D_n$, all measured for the vehicle. Buoyancy stays the cells', and the
water's acceleration still pushes the displaced volume. The sample is the BlueROV2 Heavy,
`assets/vehicles/bluerov2_heavy.hydro.yaml`, from von Benzon et al. 2022 (J. Mar. Sci. Eng. 10,
1898, Table A1, CC BY 4.0): damping identified from open-loop runs in a pool, added mass by
Eidsvik's method.

## The surface under a rotor

With rotors, the downwash and the site's air density in the run, the water's surface moves under
the wakes (`lagrange_hydrodynamics`'s `surface` module, every 40 ms). The profile's
`aerodynamics.downwash_water: false` keeps it flat.

```mermaid
flowchart LR
    DW["downwash over the water<br/>(aerodynamics)"] -->|"water/w/air_pressure<br/>p = ½ρ_air u_n²"| S["linear capillary-gravity waves<br/>on a 3.84 m patch, 3 cm cells"]
    DW -->|"water/w/air_stress<br/>τ = ρ_air u*²"| S
    DW -->|"water/w/patch<br/>(follows the wake)"| S
    S -->|"water/w/surface"| B["bodies in the water"]
    S --> R["rotors' ground effect"]
    S --> V["viewer"]
```

**The equations.** The surface's height $\eta(x, y, t)$ over its still level obeys linear
potential-flow theory (Lamb, *Hydrodynamics*, 1932, §267 and §348). Each Fourier mode of
wavenumber $k$ is a damped oscillator,

$$
\ddot\eta_k + 2\gamma_k\dot\eta_k + \omega_k^2\,\eta_k
  = -\frac{k\tanh(kd)}{\rho_w}\,\hat p_k - \frac{\tanh(kd)}{kd}\,\frac{i\,\mathbf k\cdot\hat{\boldsymbol\tau}_k}{\rho_w},
\qquad
\omega_k^2 = \Big(g k + \frac{\sigma k^3}{\rho_w}\Big)\tanh(kd), \qquad \gamma_k = 2\nu k^2
$$

with $\sigma$ water's surface tension (72.75 mN/m at 20 °C, Vargaftik et al. 1983), $\nu$ its
viscosity and $d$ its depth. A steady pressure gives the hydrostatic dimple
$\eta = -p/(\rho_w g)$ (surface tension only smooths what is a few centimetres wide). The
tangential stress is taken as a force spread over the depth (shallow water's
$\partial_t^2\eta = g d\,\nabla^2\eta - \nabla\cdot\boldsymbol\tau/\rho_w$ with the pressure
term's depth factor), a modelling choice: what it sets up is under 1 % of the dimple. The
loads come from the downwash ([downwash.md](downwash.md) §5a).

**Solving it.** The modes are integrated exactly over each step (the load held), so the step
has no stability limit. The patch is 128 × 128 cells of 3 cm, periodic in the transform; its
outer eighth damps the water's velocity so waves leave and do not wrap round, and cells off the
water (the bank) are damped hard. The patch follows the first wake over the water by whole cells
and starts flat and still, centred on it. It costs nothing while nothing presses on it.

**Outputs.** `water/<w>/surface` (m, per cell), `surface_origin` (the grid's corner),
`surface_min` and `surface_max`. The water's cells are under the moved surface, so floating
bodies see it, and the rotors measure their ground effect to it ([rotors.md](rotors.md)).

![The sample quadcopter 0.6 m over a pond: the water/surface layer shows its four rotors' dimples (8 mm deep), and its spray drops ride the outwash out](../imgs/drone-water-dimple.png)

**The sample quadcopter's dimple** (2.0 kg, 10-inch propellers), over still water:

| Rotor plane over the water | Jet arriving | Dimple | $\rho_{air}U^2/(2\rho_w g)$ |
|---|---|---|---|
| 0.5 m | 12.4 m/s (four jets) | 9.6 mm | 9.6 mm |
| 1 m | 10.2 m/s (merging) | 6.4 mm | 6.4 mm |
| 2 m | 6.5 m/s (merged) | 2.6 mm | 2.6 mm |

## How the water looks

The viewer's realistic view and the cameras' realistic colour shade the water from its physics
(`gpu/kernels/camera/realistic.slang`, `fsWater`; `gpu::water_optics`). Three scales of slope
make the surface: the drawn swell and the moving surface $\eta$ above, which the mesh and its
3 cm cells hold; and the capillary ripples below the grid, which the air raises.

**The ripples' slope variance.** Cox and Munk (1954) measured the total mean square slope of a
clean sea as $\sigma^2 = 0.003 + 5.12\cdot10^{-3}\,U$ and of one under an oil slick as
$0.008 + 1.56\cdot10^{-3}\,U$ ($U$ in m/s at 12.5 m). A slick damps the short capillary-gravity
waves and leaves the long ones, so the difference,
$\sigma^2_r = \max(3.56\cdot10^{-3}\,U - 0.005,\ 0)$, is taken as the short ripples' share. It
starts at $U = 1.4$ m/s, where Kahma and Donelan (1988) put the onset of ripples on clean water
($u_* \approx 5$ cm/s in theory, measured up to twice that). $U$ comes from the air's friction
velocity on the water by the neutral log law, $U(z) = (u_*/\kappa)\ln(z/z_0)$ with
$z_0 = 0.011\,u_*^2/g + 0.11\,\nu_{air}/u_*$ (Charnock; Smith 1988). The friction velocity is the
downwash's on each cell of the moving patch (`water/<w>/air_stress`, served as
`water/friction_velocity`) with the site's wind's added as a stress ($u_*$ from its speed at
10 m by the same law). **Assumed:** that the ripples are in equilibrium with the local $u_*$
whatever makes it, so a sea's slope statistics apply on a patch of metres. Centimetre ripples come
to equilibrium within tens of centimetres and a second; longer wind waves do not, which is why
only the short waves' share is taken, and why the outwash's ring roughens while the stagnant
centre under the jet stays a mirror. The slope is taken isotropic (Cox and Munk's upwind and
crosswind differ by 3.16 to 1.92).

**What a pixel resolves.** The ripples are four octaves (4.8 to 0.8 cm, as much slope variance
in each, as in the capillary saturation range), each of four capillary-gravity waves running
within ±63° of the stress, $\omega^2 = gk + \sigma k^3/\rho_w$, in packets. The octaves wider
than two pixels are drawn as waves (glitter breaks into sparkles); the rest are the facets'
spread in a Beckmann distribution of the same variance, so the total slope variance is kept
whatever the distance.

**Light.** Reflection is the exact unpolarised Fresnel of water, $n = 1.333$ (0.020 at normal
incidence, 1 at grazing), per facet: eight Beckmann facets per pixel each reflect the scene (the
mirror pass's image, found where the reflected ray meets what it reflects, at the distance the
mirror pass's depth gives) or the sky; a facet that sends the ray down meets the water again,
which reflects the sky by Fresnel at that grazing incidence and lets the water's own light
through (a second bounce); a resolved ripple's back, turned from the eye, is masked by the crest
in front of it. Tilted facets see higher, darker sky at a lower Fresnel, so
the rough ring is darker and duller than still water, and the drone's and the sky's reflections
blur there. The sun's glitter is $F\,D/(4\,\mathbf n\cdot\mathbf v)\,E_{sun}$ with $D$ the
Beckmann distribution of the unresolved facets widened by the sun's 0.53° disc; on still water it
is the sun's own image. Under the surface, light is absorbed by pure water (Pope and Fry 1997:
0.27, 0.0565, 0.0102 /m at 620, 550, 465 nm) along the sun's refracted path down and the view's
refracted path (Snell's law to the bottom), so the bottom shows through clear shallow water and
goes blue-green with depth; the water's own light is a deep column's reflectance
$R_\infty = 0.33\,b_b/(a + b_b)$ (Gordon et al. 1975; Morel and Prieur 1977) with pure water's
backscatter (Morel 1974), and what leaves the surface is divided by $n^2$. Lakes and rivers
also hold coloured dissolved organic matter (CDOM), whose absorption is
$a(\lambda) = a(440)\,e^{-0.014(\lambda - 440)}$ (Bricaud, Morel and Prieur 1981): the
viewer's `--water-cdom A440` (1/m; 0, pure water, by default; natural waters 0.1 to 10,
TODO(verify) the range against Kirk, *Light and Photosynthesis in Aquatic Ecosystems*) adds it,
which darkens the water and turns it brown-green and hides the bottom. Over dark water a
roughened patch reads darker than the mirror round it at low angles (its facets see higher,
darker sky at lower Fresnel); over a bright, shallow, clear bottom it reads lighter, because
the light the roughness does not reflect shows the bottom.

## Solving it

Added mass makes the force depend on the body's own acceleration, which an explicit exchange
with the physics engine handles badly: an added mass larger than the body's own would diverge.
So $a_b$ (and $\dot\nu_r$ per axis) is last step's acceleration smoothed with gain
$1/(1 + m_a/m)$; with this gain a steady push settles in one step on the exact $F/(m + m_a)$.

## How far to trust it

| Check | Result | Test |
|---|---|---|
| A floating box's draft; an off-centre hull floats level | the closed form | `Water.ABoxFloatsAtItsDraft`, `Water.AnOffCentreHullFloatsLevel` |
| A sinking box's terminal speed under quadratic drag | the closed form | `Water.ASinkingBoxReachesItsTerminalSpeed` |
| A sphere half as dense as water released under it | starts rising at $g(\rho V - m)/(m + \tfrac12\rho V) = g/2$ within 3 %, MuJoCo and Jolt | `Water.AReleasedSphereRisesWithItsAddedMass` |
| A floating box in a 0.5 m/s current | drifts with it within 0.02 m/s | `Water.AFloatingBoxDriftsWithTheCurrent` |
| A 0.4 m float on 0.1 m, 4 s waves | heaves 0.1 m within 1 cm | `Water.ASmallFloatRidesLongWaves` |
| BlueROV2 Heavy at full thrust (Fossen) | top speeds 0.729 / 0.626 / 0.713 m/s (surge, sway, heave) against the published model's 0.72 / 0.63 / 0.71; start acceleration within 3 % of $F/(m + m_a)$ | `Water.TheBlueRov2ReachesItsTopSpeedsWithFossensModel` |
| Against a real vehicle in a pool | through the published model only: it is within about 10 % of their pool runs | — |
| The dimple under a steady jet | $\rho_{air}U^2/(2\rho_w g)$ within 2 % for 5.7 to 16.6 m/s (2 to 17 mm), and within 0.5 % for the sample quadcopter at 0.5, 1 and 2 m; **no measurement of a rotor's dimple** | `SurfacePatch.TheDimpleUnderASteadyJetIsHydrostatic`, `DownwashOnWater.TheSampleQuadsDimpleIsItsJetsStagnationPressure` |
| Lidars and cameras on the moving surface | a vertical ray hits the level plus $\eta$ within 1e-9 m; a slanted ray's hit is its first crossing within a hundredth of a cell (0.3 mm); a lidar over a 25 mm dimple reads it within 1e-9 m on the CPU reference (bit for bit run to run) and within 1 mm on the GPU path | `MovingWater.*` (test_moving_water) |
| How the water looks: Fresnel | the exact unpolarised reflectance, $(n-1)^2/(n+1)^2 = 0.020$ at normal incidence, 0.060 at 60°, to 1 at grazing | `WaterOptics.FresnelOfWater` |
| How the water looks: ripples | none below the onset (Cox and Munk's clean and slick fits meet at $U_{12.5} = 1.40$ m/s, $u_*$ 4.5 cm/s; Kahma and Donelan: ~5 cm/s), rising with $u_*$; at a 10 m/s wind ($u_*$ 0.36 m/s, Charnock) under the whole clean sea's 0.055; a dimple changes the picture only where it lies; **no photograph or measurement of the water under a rotor compared** | `WaterOptics.RippleSlopeVarianceFollowsTheWind`, `RealisticWater.TheMovingSurfaceShadesTheWaterWhereItLies` |
| Ripples | group velocity within 3 % of $d\omega/dk$ for gravity waves (λ 0.63 m, $c_g = c/2$), the slowest ripple (λ 1.7 cm, 0.23 m/s) and capillary waves (λ 2 mm, $c_g = 3c/2$); viscous decay $e^{-2\nu k^2 t}$ within 5 % | `SurfacePatch.Ripples*`, `SurfacePatch.ViscosityDamps*` |

The BlueROV2 check shows the model is implemented as published; it is as close to the real
vehicle as von Benzon et al.'s model is, for this vehicle only. A body without its own
coefficients takes a sphere's added mass and the cells' drag, which no measurement backs.

Not modelled at the surface under a rotor: anything non-linear (the dimple is millimetres, so
linear theory holds; a jet that would dig a cavity, or breaking waves, are out of it); the
jet's turbulence, which makes a real dimple flicker (the load is the mean flow's); the waves'
own push back on the air; the surface's cells under 3 cm (capillary ripples are only partly
resolved). Lidars and cameras see the moved surface: their casts meet the water's box at the
still level, and each hit is then moved onto $\eta$ (or onto a crest in front of what it hit),
in double, on every tier (`sensors::rayOntoWater`); a lidar's return takes the surface's slope as
its normal. The camera's realistic colour draws it too; its depth is moved on the undistorted
pinhole ray (a distorted lens's depth is off by its distortion there). The viewer's realistic
water is lifted and shaded by it (viewer.md). Not modelled in how it looks: Cox and Munk's
upwind-crosswind anisotropy and skewness; waves of 6 to 30 cm, which neither the grid's physics
(no wind-wave growth) nor the ripples hold; particles in the water (only pure water and,
with `--water-cdom`, its dissolved matter: real lakes are also turbid); light scattered more than once
under the surface; ripples on the camera sensors' water from the downwash (they see the wind's
only: the sensors read no stress).

Not modelled: added mass for shapes other than a sphere, and added inertia in rotation, without
coefficients; off-diagonal added mass and damping; wave radiation and diffraction (the waves pass
through bodies); thrusters as rotors in water; friction on what drives through water. Water has
no collider; away from the moving patch the viewer and the lidars see a flat surface (lidars as
a mirror, with no return at a slant or from the bottom).

The full model is in [hydrodynamics.md](../design/domains/hydrodynamics.md) §9.
