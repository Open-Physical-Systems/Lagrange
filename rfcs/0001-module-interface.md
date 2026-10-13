# RFC 0001: Module interface, signals and scheduling contract

| | |
|---|---|
| Status | Draft |
| Created | 2026-10-13 |
| Affects | `core/module/`, `core/signals/`, `core/scheduler/`, `core/plugin_abi/`, every module |
| Supersedes | The illustrative interface in CLAUDE.md §11.4–§11.5 |
| Detail | `docs/design/architecture.md` §2–§8, §13 |

## Summary

Define how a simulation module declares its signals, how units are checked, how modules at
different rates exchange values, how substitutes satisfy a domain's contract, and the C ABI that
third-party modules implement.

## Motivation

The project's central promise is that any domain can be enabled, disabled or replaced
(invariants 1–4). That only works if (a) modules never reference each other, (b) every exchanged
quantity has a checked unit, and (c) the scheduler, not the modules, decides ordering and rate
transitions. The reference interface in CLAUDE.md shows the intent but has gaps that would
surface as soon as the second module is written:

1. `inputs()`/`outputs()` depend on the world (one torque signal per joint), but the world first
   arrives in `init()`. Wiring needs the list before `init()`.
2. `SignalSpec::name` is a `std::string_view` to strings that must be built at run time.
3. `void` methods leave exceptions as the only error path; exceptions cannot cross the C ABI.
4. Nothing says how a 100 µs module and a 1 ms module see each other's values.
5. There is no state save/restore, so determinism tests, checkpoints and rollback are impossible.
6. A dimension-only unit check accepts energy where torque is expected.

## Design

### Lifecycle

```
create → declare(world, config, declarer) → [scheduler wires and checks] → init(world, config, bus)
       → step(ctx, bus) × N → shutdown → destroy
```

- `declare` lists every input and output as a `SignalSpec`. It may read the world and config. It
  must not allocate large state or touch the bus.
- Between `declare` and `init` the scheduler checks units, types, writers, contracts and cycles.
  Any error stops startup with a message naming the signal, both modules and both units.
- `init` receives valid `SignalId`s (returned by the declarer) and writes initial outputs, so
  that a reader that runs first in tick 0 sees defined values.
- `step` is the hot path: no allocation, no I/O, no USD, no string lookups.
- All methods return `Status`. A non-OK status from `step` stops the run with the module's
  message; `Diverged` additionally triggers a state dump for debugging.

The C++ interface (`IModule`, `SignalDeclarer`, `StepContext`, `ModuleInfo`, `Status`) is given
in `architecture.md` §5 and is normative for this RFC.

### Signals

- `SignalSpec { name, type, unit, arrayLength, frame, hold, optional, defaultValue, feedback }`.
- One writer per signal. Readers are unlimited.
- Naming: `<class>/<instance>/<quantity>[/<qualifier>]`; robot prefix in multi-robot worlds.
- Types: `Scalar`, `Vec3`, `Quat`, `Pose`, `Wrench`, `ScalarArray`, `Bool`, `Int`, `Bytes`,
  `Field3D`, `Image`, `PointCloud`. Numeric types are stored as `double`.
- Composite types and units: the `unit` of a `Pose` is that of its position (m); of a `Wrench`,
  that of its first three components (N for force, m/s for a twist, m/s² for an acceleration
  pair), and the second three are the corresponding angular quantity (N·m, rad/s, rad/s²). The
  quaternion of a `Pose` is dimensionless.
- Batches: every signal has `batchCount` values; `batchCount == 1` for single-environment runs.

### Units

`Unit = { Dimension (8 exponents: m, kg, s, A, K, mol, cd, rad), scale, QuantityKind }`.
Wiring requires all three to be equal. There is no conversion, by design (invariant 2): a module
that wants millimetres declares metres and converts internally, or fails. `QuantityKind`
separates quantities with equal dimensions (torque and energy; frequency and angular velocity
are already separated by the angle exponent).

### Rates and holds

Time is integer nanoseconds. Each module steps at its `dt`; the base tick is the GCD. An input's
`hold` says what a reader sees when its writer runs at another rate:

| Hold | Reader sees | Typical use |
|---|---|---|
| `ZeroOrder` | last committed value | states read by slower or equal-rate modules |
| `LinearExtrapolate` | extrapolation from the last two commits to the reader's time | slow state read by a fast module (speed into the electrical module) |
| `WindowAverage` | mean of the writer's commits since the reader's previous step | fast flows read by a slow module (torque, loss power), conserving impulse and energy |

### Ordering and loops

Edges go from writer to reader. Inputs marked `feedback: true` read the writer's previous-step
value and do not create an ordering edge. After removing feedback edges the graph must be
acyclic; otherwise startup fails and prints the cycle. Ties are broken by (dt, domain, name).
This makes loop-breaking an explicit, reviewed property of each module's declaration.

### Substitutes and contracts

Each domain registers a `DomainContract` (the outputs others may rely on, expressed as rules
over schema instances) and at least one substitute. The module or substitute chosen for a domain
must provide the whole contract for the loaded world. Three substitute kinds exist: ideal,
constant, replay; the generic replay substitute works for any domain from MCAP. Profile tokens
(`zero`, `clear`, `ground_truth`, …) map to (kind, preset) pairs.

### State

Deterministic modules implement `saveState`/`restoreState` (opaque bytes, versioned by the
module). The scheduler composes them with bus contents and time into a run state and a hash.

### C ABI

`core/plugin_abi/lagrange_plugin.h` mirrors the above in C99: `lagrange_module_api` (function
table from the plugin), `lagrange_host_api` (function table from the host), every struct starting
with `struct_size`, one exported symbol `lagrange_plugin_get_module`, and
`LAGRANGE_PLUGIN_ABI_MAJOR/MINOR`. Same major required; minor additions are appended fields and
functions. The ABI is major 0 (unstable) until product v1.0.

### What the reference example keeps

`IdealTorqueSource::step` remains a loop of `bus.write(j.torqueOut, bus.read(j.torqueCommand))`,
with both members being `SignalId`s. `preferredStep()`, `inputs()` and `outputs()` remain.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Keep the reference interface unchanged | Cannot wire before `init`; no error path over the C ABI; no state |
| Compile-time units only (mp-units, packaged for Lyrical as 2.5.0) | Plugins cross a C ABI and signals are created from data at run time; a runtime representation is needed anyway. Modules may still use mp-units internally |
| Automatic unit conversion at wiring | Hides mistakes (mm vs m, rpm vs rad/s); violates invariant 2 |
| FMI 3.0 as the module interface itself | FMI variables are scalar and static; our signals are world-dependent, typed (fields, images) and batched. FMUs are hosted by one module instead |
| Publish/subscribe with queues (ROS-like) between modules | Queues add latency and nondeterminism; co-simulation needs defined sampling instants |
| Scheduler breaks loops automatically | Silent one-step delays in stiff loops cause instability that nobody chose |
| Direct module-to-module "fast path" for stiff pairs | Violates invariant 1. Stiff pairs are handled by subcycling, averaging and prediction (`architecture.md` §8) |
| Variable-step co-simulation with rollback | More accurate for stiff coupling, but needs every module to support rollback and makes real-time pacing hard. `saveState` keeps the door open |

## Unresolved questions

1. **Jacobian exchange.** Should the interface let a module publish partial derivatives of
   outputs with respect to inputs (FMI-style directional derivatives) for implicit coupling?
   Proposed: not in v1; `joint/*/effective_inertia` covers the motor case. Revisit after the
   v0.3 coupling tests.
2. **Geodetic and other mixed-unit arrays.** Proposed: no `Geodetic` type; use three scalar
   signals. Confirm.
3. **Parameter changes at run time.** When the inspector edits a schema attribute, how is the
   module told: a `reconfigure(WorldView)` call between ticks, or full re-`init`? Proposed: an
   optional `onParametersChanged(const WorldView&, std::span<const PrimId>)`; modules without it
   are re-initialised with state preserved through `saveState`/`restoreState`.
4. **Events.** Discrete events (contact begin, fault trip, SEU) are currently `Bytes` or `Bool`
   signals. Is a first-class event type with exact timestamps needed? Proposed: decide with the
   first consumer (v0.3 fault handling).
5. **Topology changes.** Spawning a robot changes the signal set. Proposed: allowed only between
   ticks through a re-declare and re-wire of affected modules, with a new catalog generation.
   Cost and API to be designed in v0.2 (`simulation_interfaces` spawn needs it).
6. **GPU modules.** How does a module that keeps its state on the GPU expose batch signals
   without a readback each tick? Proposed: `BufferHandle`-typed batch signals with lazy
   readback; details in a later RFC before Tier 2 batched physics.
7. **`std::expected`** is C++23. Choose `tl::expected` (license CC0; TODO(verify)) or an in-tree
   type.
8. **Thresholds of the coupling stability check** are provisional until validated.
