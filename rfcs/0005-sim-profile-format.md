# RFC 0005: Simulation profile format

| | |
|---|---|
| Status | Accepted (2026-10-15), with the resolutions below |
| Created | 2026-10-13 |
| Affects | `profiles/`, `server/`, `mock_server/`, the viewer's domain panel, `core/module/ModuleConfig` |
| Extends | The example profiles in CLAUDE.md §11.1 |
| Detail | `docs/design/formats.md` §5 |

## Summary

A simulation profile is a YAML file that says which stage to load, which domains are enabled,
with which backend, fidelity and step, what replaces each disabled domain, and what to record. It
is validated by `profiles/schema.json` (JSON Schema 2020-12) and by additional loader checks.

## Motivation

Toggling domains is the product's defining feature. The profile is where a user expresses it, the
viewer's domain panel writes it, CI validates it, and recordings embed it. It needs a stable,
versioned, machine-validated format before the viewer's domain panel is built in v0.1.

The reference examples establish the shape. This RFC keeps that shape and closes five gaps:

1. No format version, so files cannot be migrated.
2. No seed, determinism or tier settings.
3. `dt` as a binary floating-point number cannot be compared exactly.
4. Substitute tokens are not tied to the three substitute kinds.
5. Domains that do not apply to a robot class have no defined treatment.

## Design

### Top-level keys

| Key | Type | Required | Meaning |
|---|---|---|---|
| `schema_version` | integer, const 1 | yes | Format version |
| `stage` | string | yes | USD file with robots and world |
| `environment` | string | no | USD file with the environment prim (`WeatherAPI`) and environment sources |
| `seed` | integer ≥ 0 | no (0) | Run seed for all counter-based random numbers |
| `deterministic` | boolean | no (false) | Deterministic CPU reference mode |
| `tier` | `auto`, 0, 1, 2 | no (`auto`) | Force a capability tier at or below the detected one |
| `batch` | integer ≥ 1 | no (1) | Parallel environments |
| `time` | object | no | `real_time_factor` (0 = unlimited), `end` (s) |
| `domains` | object | yes | One entry per domain (below) |
| `world` | object | no | `geo_origin`, `streaming` |
| `bridges` | object | no | `ros2`, `ros2_control`, `mavlink`, `autoware` |
| `scenario` | object | no | `opendrive`, `openscenario` |
| `views` | array of tokens | no | Initial viewer layout hint; does not affect simulation |
| `record` | object | no | `format: mcap`, `path`, `signals`, `compression` |

Unknown keys are errors (`additionalProperties: false`), so typos are caught.

### Domains

Required keys in `domains`: `rigid_body`, `electrical`, `thermal`, `aerodynamics`, `weather`,
`emi`, `radiation`, `sensors`. Optional keys, used when the stage contains matching prims:
`vehicle_dynamics`, `rotor_aero`, `hydrodynamics`, `traffic`, `comms`. An optional domain that is
absent from the profile but needed by the stage (rotor prims without `rotor_aero`) is a loader
error that names the prims; the reverse (domain enabled, no matching prims) is a warning and the
domain is skipped.

Every domain entry has:

| Key | Meaning |
|---|---|
| `enabled` (required) | boolean |
| `dt` | step in seconds; optional (module's preferred step) |
| `substitute` | required when `enabled: false` |
| `source` | MCAP path, required when the substitute is `replay` |
| domain-specific keys | `backend`, `fidelity`, `coupling`, `effects`, `only`, … as listed in the schema |

### Substitutes

`substitute` is either a short token or an object.

| Token | Kind | Preset | Valid for |
|---|---|---|---|
| `ideal` | ideal | domain default (`ideal_torque` for electrical) | electrical, rotor_aero, vehicle_dynamics, comms |
| `ground_truth` | ideal | `ground_truth` | sensors |
| `load_profile` | ideal | `load_profile` | rigid_body |
| `zero` | constant | `zero` | aerodynamics, emi, radiation, hydrodynamics, traffic |
| `clear` | constant | `clear` | weather |
| `ambient` | constant | `ambient` | thermal |
| `fixed` | constant | `fixed` | rigid_body |
| `replay` | replay | — | any |

Object form: `{ kind: ideal|constant|replay, preset: <string>, source: <path>, value: <number> }`
for substitutes registered by plugins. A token that is not valid for the domain is a loader
error listing the valid ones (the valid set comes from the substitute registry at run time, so a
third-party domain can add its own).

### Time steps

- `dt` is written in seconds, as in the reference (`dt: 0.0001`).
- The loader reads the scalar's source text and converts it as a decimal to integer nanoseconds.
  It rejects values that are not a whole number of nanoseconds.
- Loader checks across domains: base tick (GCD) ≥ 1 µs; hyperperiod (LCM) ≤ 10 s. Violations
  name the offending pair.
- YAML parsers that only expose a `double` are not sufficient; the loader needs the scalar text.
  Both yaml-cpp and Python YAML libraries can provide it (TODO(verify) for the Python library
  chosen).

### Paths

Relative paths resolve against the profile file's directory, then against the search path
`LAGRANGE_ASSET_PATH`, then against installed share directories of ROS packages using
`package://<pkg>/…` URIs. Recording paths resolve against the working directory. `record.path`
may contain `{date}`, `{time}` and `{seq}` placeholders (proposed).

### Validation layers

1. **JSON Schema** (`profiles/schema.json`, draft 2020-12; full draft in `formats.md` §5.2):
   structure, types, enumerations, "disabled ⇒ substitute", "replay ⇒ source".
2. **Loader checks**: nanosecond steps, commensurability, file existence, substitute validity
   per domain, replay source contains the contract's channels with matching units, tier
   available, stage/domain consistency.
3. **Wiring checks** (RFC 0001): units, contracts, cycles, coupling stability.

Layer 1 runs in the viewer (immediately as the user edits), in pre-commit and in CI. Layers 2 and
3 run in the server and are reachable from the viewer through `ValidateProfile`, which returns
issues with JSON pointers (`/domains/electrical/dt`).

### Canonical form

The server and the viewer emit a canonical YAML: fixed key order (as in the tables above), flow
style for domain entries, no anchors or aliases, UTF-8, one trailing newline. Recordings embed
the canonical text. The viewer's "Save" writes it.

### Versioning

`schema_version` increases on any incompatible change. `tools/migrate_profile` upgrades files one
version at a time. The JSON Schema `$id` contains the version. Before product v1.0 the format is
version 1 and may still change; changes are announced in the changelog.

### Compatibility with the reference examples

`bridge_rain_electrical.yaml` and `bench_electrical_replay.yaml` from CLAUDE.md §11.1 validate
after adding `schema_version: 1`. Their `backend: physx` stays valid; new examples default to
`mujoco` (open question Q9). Six further examples, one per robot class and one for a huge world,
are in `formats.md` §5.3.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Profile inside the USD stage (as stage metadata or a prim) | A profile is a run configuration, not part of the robot; the same stage runs under many profiles; YAML is easier to diff, review and generate from a UI |
| ROS 2 launch files and parameters as the only configuration | Launch is Python code and cannot be validated statically; the server must run without ROS; a launch file can still point at a profile |
| TOML or JSON | YAML is what the ROS community edits daily; JSON Schema validates YAML documents equally well |
| `dt` as integer microseconds or as a string with unit (`"100us"`) | Clearer for machines, but changes the agreed examples and is less familiar than seconds. Exact decimal parsing gives the same safety. Could be accepted as an additional form later |
| Free-form substitute names only | Users could not see which of the three kinds a name means; plugins could collide |
| All thirteen domains required in every profile | Forces an arm profile to mention hydrodynamics; noise that hides the meaningful toggles |
| Implicit substitutes (a default when the key is missing) | The reference rule "every disabled domain names a substitute" is deliberate: what replaces a domain changes results and must be visible in the file |
| A domain-specific language or Python config | Not statically validatable; harder for the viewer to round-trip |

## Resolutions on acceptance (2026-10-15)

The owner accepted this RFC. Each question below takes its proposed answer, or is deferred:

1. Per-robot overrides: not in version 1; a later `overrides:` list keyed by robot prim path.
2. Every change under `domains` restarts the run in v0.2; real-time factor, recording and the
   sensor `only` list may apply live. Live re-wiring waits for RFC 0001 question 5.
3. No `extends:` composition in version 1.
4. `views` stays in the profile as a hint to the viewer.
5. No general `environment_overrides:` in version 1; site constants stay in the environment USD,
   and `thermal.ambient_temperature` remains for the substitute.
6. Machine-specific settings (ROS domain id, endpoints) come from the command line and the
   environment, never the profile.
7. The C++ JSON Schema validator is jsoncons (BSL-1.0, header-only, Ubuntu `libjsoncons-dev`),
   chosen with roadmap 0.2-E5 because it supports JSON Schema 2020-12 `unevaluatedProperties`;
   recorded in `dependencies.md`.
8. The published `$id` URL waits for the project's domain; until then it stays as it is.

## Unresolved questions (as proposed)

1. **Per-robot overrides.** In a multi-robot stage, can one robot have electrical enabled and
   another substituted? Proposed for a later version: an `overrides:` list keyed by robot prim
   path. Version 1 is world-wide per domain.
2. **Changing a profile at run time.** Which changes apply live (real-time factor, recording,
   sensor `only` list) and which need a restart (enable/disable, `dt`, backend)? Proposed:
   everything in `domains` restarts in v0.2; live re-wiring is a later feature that depends on
   RFC 0001 unresolved question 5.
3. **Composition.** `extends: base.yaml` to share settings between profiles? Useful, but it
   complicates validation and the viewer's round trip. Proposed: not in version 1.
4. **`views`** belongs to the viewer rather than the simulation. Keep it in the profile (as in
   the reference) or move it to layout presets? Proposed: keep as a hint.
5. **Scenario parameters.** Site constants live in the environment USD. Should the profile be
   able to override a few (ambient temperature, wind) without editing USD? `thermal.
   ambient_temperature` exists for the substitute case; a general `environment_overrides:` map is
   a possible addition.
6. **Secrets and machine-specific settings** (ROS domain id, endpoints): profile, environment
   variables, or command line? Proposed: command line and environment, not the profile, so
   profiles stay shareable.
7. **JSON Schema validator for C++** is not yet chosen (`dependencies.md` §2).
8. **Published `$id` URL** needs the final project name and a domain.
