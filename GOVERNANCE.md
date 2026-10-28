# Governance

Lagrange is a project of Open Physical Systems.

## Maintainers

| Maintainer | GitHub | Areas |
|---|---|---|
| Danendra | [@Danendra10](https://github.com/Danendra10) | all |

## Decisions

- Day-to-day changes are decided in pull requests by review.
- Changes to stable surfaces (USD schemas, the plugin C ABI, the viewer protocol, the profile
  format) and changes to an architecture invariant need an accepted RFC in `rfcs/`.
- Recorded decisions and open questions are in `docs/plan/open-questions.md`.

## Releases

`main` holds releases. A release is a pull request from `dev` to `main`, tagged `vX.Y.Z`.
Versions follow semantic versioning from v1.0.0.

## Stability and long-term support (RFC 0007)

From v1.0.0 the USD schemas, the plugin C ABI (major 1), the viewer protocol (`v1`), the profile
format (`schema_version: 1`) and the recording format (`format_version: 1`) are frozen for the
whole 1.x line: they change only by addition. Anything else is a new version of that surface
with a migration in `tools/migrate/`, and a new major release.

| Line | Branch | Supported until | Gets |
|---|---|---|---|
| 1.x | `release/1.x` (from `main` at v1.0.0) | ROS 2 Lyrical's end of life, May 2031 | bug fixes, security fixes, build fixes for Ubuntu 26.04 |
| next minor (1.1, ...) | `dev`, released through `main` | until the following minor | features within the RFC 0007 rules |

Backports:

- A fix lands on `dev` first, by pull request, then is cherry-picked to `release/1.x` in a
  pull request titled `fix(...): ... (backport #N)`.
- Backported: bugs that give wrong results or crash, security issues (`SECURITY.md`), build or
  packaging failures on Ubuntu 26.04 and ROS Lyrical. Not backported: features, refactors,
  behaviour changes users would notice other than the fix.
- Patch releases (`v1.0.1`, ...) are tagged on `release/1.x`; minor releases (`v1.1.0`) come from
  `dev` through `main` and are merged into `release/1.x` so that the branch stays the newest
  1.x.
