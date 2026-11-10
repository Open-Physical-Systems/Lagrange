# RFC 0013: Magnetic interference from a robot's own power wiring

| | |
|---|---|
| Status | Accepted (2026-11-11) by the owner, with the proposed answer to each open question; gated merges of its steps pre-approved |
| Created | 2026-11-11 |
| Affects | `schemas/emi/` (additions), `modules/emi/`, `modules/sensors/` (magnetometer outputs), `server/` (motor leads), `profiles/schema.json` (addition), `viewer/` (EMI effects), `docs/` |
| Builds on | RFC 0002 (schemas), RFC 0005 (profiles), RFC 0007 (the 1.x freeze) |

## Summary

The current a robot's motors draw flows through its battery leads and motor wires, and that
current's magnetic field is what disturbs its compass most: ArduPilot and PX4 both calibrate it
out as a field proportional to the current ("compassmot"), and aeromagnetic surveys measure it
the same way. Lagrange's EMI domain has sources fixed in the scene (power lines, transmitters)
and motor dipoles scaled by a motor's current, but no wires. This RFC adds a `wire` source whose
field is the exact Biot–Savart sum over its segments, driven by any current signal; a server
option that lays one from the battery to each motor; and two magnetometer outputs that say what
the disturbance does to a heading. All additions.

## Design

### 1. Schema additions (`EmiSourceAPI_1`)

| Attribute | Type | Unit | Fallback | Meaning |
|---|---|---|---|---|
| `emi:sourceType` | token | — | — | appended: `wire` |
| `emi:path` | double[] | m | `[]` | the wire's route, x, y, z of each point in the prim's frame (at least two points) |
| `emi:returnOffset` | double3 | m | `(0, 0, 0)` | the return conductor, the same route shifted by this, carrying the current back; zero: no return in the model |
| `emi:currentSignal` | token | — | `""` | a signal in A that sets the source's current every step (any source type); over `emi:drivenBy` |

### 2. Field

Each straight segment from $a$ to $b$ carrying $I$ gives, at a point at perpendicular distance
$d$ from its line, $B = \frac{\mu_0 I}{4\pi d}(\sin\theta_2 - \sin\theta_1)$ along
$\hat{\ell} \times \hat{r}$ (Griffiths, *Introduction to Electrodynamics*, 4th ed., Example 5.5).
The source's field is the sum over its segments, minus the same sum over the return path. A
direct current has frequency 0; the field adds as a vector to the other sources'.

### 3. Motor leads

`lagrange_server --emi-motor-leads SPACING`: for every motor (`ElectricalMotorAPI_1`) a wire from
the robot's (first) battery's prim to the motor's prim, at their rest poses, carrying `motor/<m>/bus_current`,
with its return `SPACING` m above it (the body's z). And every magnetometer without an
`EmiSusceptibilityAPI_1` becomes a victim of gain 1. The geometry is the user's: SPACING has no
default, and a straight lead from battery to motor is the simplest route; a real harness is
authored as `wire` prims with its own path.

### 4. What it does to a heading

Each magnetometer gets `sensor/<n>/heading_error` (rad): the angle between the horizontal part of
the Earth's field (World Magnetic Model) and that of the Earth's field plus the disturbance, and
`sensor/<n>/interference` (1): |disturbance| / |Earth's field|, the ratio ArduPilot's compassmot
reports as a percentage (under 30 % acceptable, over 60 % move the compass; ArduPilot, "Advanced
Compass Setup").

### 5. Viewer

The EMI workspace's |B| slice includes the wires (the server's `emi/flux_density` sums every
source). The Environment panel's EMI section lists each victim with what it suffers now: |B| at
it, and for magnetometers the heading error and interference with ArduPilot's bands coloured.

## Validation

`test_emi`: the finite segment against the infinite wire (long segment), the pair's far field
$\mu_0 I s / (2\pi r^2)$, the perpendicular bisector's closed form, linearity in the current (the
property both autopilots' calibrations rely on), and the heading error of a known disturbance.

## Open questions (with a proposed answer)

1. *Where does the return run?* Offset by a vector, the same route: a pair of leads; zero for a
   wire whose return is far away (a chassis return).
2. *Motor phase wires and the motor's own stray field?* Not now: the three phases' currents sum
   to zero and their field falls off fast; the motor's stray field is the existing dipole source,
   whose moment per ampere the user gives (no published value was found).
3. *PWM noise on GNSS and radios?* Not now: the coupling from a drive's switching edges to an
   antenna depends on the layout and no general model was found; measured UAS C/N0 losses (up to
   10 dB, Stubbs and Akos, GPS World 2016) are noted in the docs.
