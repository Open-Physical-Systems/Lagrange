# RFC 0007: The v1.0 freeze

| | |
|---|---|
| Status | Accepted (2026-10-29), owner-delegated: the owner pre-approved accepting the freeze RFC for v1.0 on 2026-10-29 (`/loop finish until last version`); listed in `docs/plan/progress.md` for the owner's review |
| Created | 2026-10-29 |
| Affects | `schemas/`, `core/plugin_abi/`, `proto/`, `profiles/schema.json`, recordings (MCAP), `tools/migrate/`, `GOVERNANCE.md` |
| Decides | What is stable in 1.x, how it may change, how old files are carried forward, how long 1.x is supported |

## Summary

From v1.0.0 these five surfaces are stable for the whole 1.x line: the USD schemas, the plugin C
ABI, the viewer protocol, the profile format and the recording format. Within 1.x they change
only by addition. Anything else needs a new version of that surface (a `_2` schema, ABI major 2,
`lagrange.viewer.v2`, `schema_version: 2`, `format_version: 2`) with a migration in
`tools/migrate/`. The 1.x line is supported on `release/1.x` for as long as ROS 2 Lyrical is,
until May 2031.

## Motivation

The roadmap's v1.0 is "stable schemas + plugin ABI" (§9): users writing robots, plugins,
viewers and scripts against Lagrange need to know which files and binaries keep working across
updates. Until v0.7 every surface could change in any release (RFC 0002 "until product v1.0
every schema is version 1 and may change without a migration"). This RFC ends that period.

## What is frozen, and the rules

| Surface | Version at 1.0 | Allowed within 1.x | Needs a new version |
|---|---|---|---|
| USD schemas (`schemas/`, RFC 0002) | every `*API_1` as of v1.0.0 | new optional attributes with a fallback that keeps the old behaviour; new schemas; new allowed tokens at the end | removing or renaming an attribute, changing a fallback, a type or a unit, or what a value means: `*API_2` and a `migrate_schema` step |
| Plugin C ABI (`core/plugin_abi/`) | `LAGRANGE_PLUGIN_ABI_MAJOR 1` | appending fields at the end of a struct (callers check `struct_size`), appending functions at the end of a table (MINOR + 1) | anything else: MAJOR 2; hosts load only their own MAJOR |
| Viewer protocol (`proto/`, RFC 0003) | `lagrange.viewer.v1` | new fields with new numbers, new messages, new `oneof` members; `buf breaking` against the release (`FILE` rules) | removing, renumbering or retyping a field: package `v2`, negotiated in `Hello` |
| Profiles (`profiles/schema.json`, RFC 0005) | `schema_version: 1` | new optional keys with a default that keeps the old behaviour | removing or renaming a key or changing a default: `schema_version: 2` and a `migrate_profile` step |
| Recordings (MCAP, formats.md §7) | `RunMeta.format_version 1` | new channels, new fields in their messages | removing a channel or changing what a field means: `format_version 2` and a `migrate_recording` step |

Everything not in the table (C++ headers, the signal names of modules, the CLI's output text, the
Python API) follows semantic versioning of the packages but is not frozen by this RFC; the
signal names that ROS topics and recordings expose are documented per domain and change only
with a changelog entry.

## How the freeze is checked

- Schemas: `schemas/test/test_schemas.py` holds the list of every attribute with its type,
  fallback and unit at 1.0 (`schemas/test/frozen_1_0.json`); the test fails when an existing
  entry changes or disappears.
- Plugin ABI: `core/plugin_abi/test/test_abi_layout.c` pins every struct's size and every field's
  offset of ABI 1.0 with static assertions; `tools/ci/abi_check.sh` runs `abidiff` against the
  1.0 baseline when libabigail is installed.
- Protocol: `buf breaking --against '.git#tag=v1.0.0'` in CI (`buf` is not installed on the
  development machine yet: TODO(verify) its version when it is).
- Profiles and recordings: `tools/migrate` tests migrate every sample and fixture written by
  earlier releases (back to v0.5) and check the result validates.

## Long-term support

- `release/1.x` branches from `main` at v1.0.0 and is supported until ROS 2 Lyrical's end of
  life (May 2031).
- Fixes land on `dev` first and are cherry-picked to `release/1.x` when they fix a bug, a
  security issue or a build break on Ubuntu 26.04; features are not backported.
- 1.x patch releases (`v1.0.1`, ...) are tagged on `release/1.x`; minor releases (`v1.1.0`) come
  from `dev` through `main` as before and are merged into `release/1.x`.

## Alternatives considered

- Freezing only the schemas and the ABI (the roadmap's wording): the protocol, profiles and
  recordings are files users keep and programs they write, so they are frozen too.
- Version 1 of each schema renamed to `_1_0`: no; `_1` already says the major version, and the
  minor additions are tracked in `schemas/CHANGELOG.md`.

## Unresolved

- Viewer protocol `v2` negotiation is designed in RFC 0003 but not built; it is not needed until
  a breaking change.
