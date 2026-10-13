# Domain design documents

One file per domain. Each has the same sections:

1. Scope and release
2. Models per fidelity level
3. Signals: `declare()` as code (inputs, outputs, units)
4. Step size
5. Substitutes and the domain contract
6. GPU and CPU placement per tier
7. Validation method and reference data

Conventions used in the signal listings:

- Code is the body of `IModule::declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d)`
  from `docs/design/architecture.md` §5. `SignalSpec` fields use C++20 designated initialisers.
- `{j}` is a joint instance name, `{b}` a body, `{p}` the prim that carries the schema, `{bus}` a
  bus prim, `{s}` a sensor prim. Names follow architecture §4.
- Units are constants from `simcore::units`. Constants not yet listed in architecture §3
  (`kg_m2`, `W_per_K`, `Hz`, `m_per_s2`, `rad_per_s2`, `N_s_per_m`, `one_per_s`, `bit_per_s`, `s`)
  are added to `core/signals/Unit.hpp` with the first module that needs them.
- "Contract" lists the outputs that other domains may depend on. Every substitute must provide
  them (architecture §7).
- Reference data listed under validation is a plan. Each dataset or paper must be obtained and
  its license checked before it is used in tests; none was downloaded in the planning task.

| Domain | File | First release | Wedge? |
|---|---|---|---|
| Rigid body | `rigid_body.md` | v0.2 | no (integrate engines) |
| Electrical | `electrical.md` | v0.3 (thin slice in v0.2 proposed, Q3) | yes |
| Thermal | `thermal.md` | v0.3 | yes (with electrical) |
| Aerodynamics and rotor aero | `aerodynamics.md` | v0.4 | partly |
| Weather | `weather.md` | v0.5 | yes (sensor effects) |
| EMI | `emi.md` | v0.5 | yes |
| Radiation | `radiation.md` | after v0.7 (see roadmap) | yes |
| Sensors | `sensors.md` | v0.2 (lidar, IMU), then per release | partly |
| Vehicle dynamics | `vehicle_dynamics.md` | v0.6 | no |
| Hydrodynamics | `hydrodynamics.md` | post-1.0 | no |
| Traffic | `traffic.md` | v0.6 | no (integrate SUMO) |
| Comms | `comms.md` | v0.5 | partly |
