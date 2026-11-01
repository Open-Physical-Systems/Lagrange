# RFC 0010: Components between buses, and a schematic's parts

- Status: Draft, waiting on the owner
- Date: 2026-11-02
- Touches: `schemas/electrical`, `modules/electrical/network`, `io/stage/src/Netlist.cpp`, the viewer's Circuit tab

## Why

A real robot's schematic has parts between two nodes: a fuse from the battery to the main bus, a contactor, a relay, a diode against reverse current, a harness with resistance between two boards. The netlist importer refuses all of them, so most schematics a person draws will not load. The network today is star-shaped: each bus has its own voltage state, and only DC-DC converters join buses, by power.

## What it would do

Add one schema, `ElectricalBranchAPI_1`, for a two-terminal part between two buses:

| `elec:branch:kind` | Behaviour | Parameters |
|---|---|---|
| `resistor` | Ohm's law | `resistance` |
| `fuse` | Resistor until `I²t` reaches its rating, then open until reset | `resistance`, `i2tRating` |
| `switch` | Resistor or open, by the signal `branch/<b>/command` | `resistance`, `openResistance` |
| `diode` | Piecewise: `forwardDrop` and `resistance` one way, open the other | `forwardDrop`, `resistance` |

The network solves all connected buses together. Each step is one sparse linear system, nodal analysis with the capacitors' backward-Euler companions as today, with a few Newton or active-set passes for what is piecewise (diodes, batteries at their charge limit). Bus voltages stay states at 100 µs. Signals add `branch/<b>/current`, `/voltage`, `/state`. Disabling the domain still leaves the ideal torque source. The netlist importer reads `R` between two buses as a resistor, `D` with a `.model` as a diode, `S` as a switch, and `*lagrange fuse R_x i2t=` for a fuse.

## What it would not do

Inductors, transformers, op-amps, regulators as circuits, or switching in the loop: those stay offline in ngspice, as invariant-level design says (`electrical.md`, CLAUDE.md §11.7). A regulator stays a DC-DC converter.

## Questions for the owner

1. **Solver.** Own small dense or sparse solve in `network/`, or ngspice's library in the loop? I recommend our own: real time at 100 µs with no allocation, a few dozen nodes at most, and invariant 12 names ngspice as offline. CLAUDE.md §10 asks for a reference: `check_electrical.py` gains a circuit for each kind against ngspice.
2. **Determinism and cost.** A robot with no branches keeps today's code path, so nothing it does changes. A robot with branches costs a solve per step; I expect well under 100 µs for 20 nodes on this machine, unmeasured.
3. **Schema change.** It needs a schema addition within 1.x (RFC 0007 allows additions) with a migration note and two approvals. Should I go ahead once you accept this?

## Tested by

Hand formulas and ngspice through `tools/ci/check_electrical.py` for each kind, a fuse opening at its `I²t` to within a step, and energy balance over a drive cycle as `test_network` does today.
