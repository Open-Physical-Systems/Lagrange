# Changelog

Notable changes to Lagrange are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- Project plan, design documents and RFC drafts 0001 to 0006.
- Repository tooling: license, formatter and linter configuration, pre-commit hooks, GitHub
  templates, labels and pull-request checks.
- `lagrange_core` package with simulation time types.
- `lagrange_proto` package: viewer protocol messages (`lagrange.viewer.v1`) with generated C++
  and Python code. RFC 0003 accepted.
- Protocol framing: `FrameReader` and `appendFrame` for length-prefixed frames, with a fuzz
  target.
- `lagrange_cmake` package with the shared compile options.
- Protocol transport: non-blocking `Connection` and `Listener` over Unix sockets and TCP,
  file-descriptor passing, `Endpoint` parsing and reconnect `Backoff`.
- Shared-memory fast path: `ShmArena` (sealed memfd ring) and `ShmReader` with generation and
  content-hash checks.

### Changed

- CI builds only when code changed, runs the protocol checks only when the protocol changed,
  and fuzzes nightly instead of on every pull request.
