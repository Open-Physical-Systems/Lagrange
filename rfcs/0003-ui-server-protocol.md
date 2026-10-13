# RFC 0003: UI↔server protocol

| | |
|---|---|
| Status | Draft |
| Created | 2026-10-13 |
| Affects | `proto/`, `viewer/client/`, `server/`, `mock_server/`, `io/recording/` |
| Decides | The open choices in CLAUDE.md §4: Protobuf or FlatBuffers; Zenoh or TCP/Unix socket |
| Detail | `docs/design/protocol.md` |

## Summary

The viewer and any simulation server (real, mock, replay) communicate through length-prefixed
Protobuf messages over a Unix domain socket or TCP, with large payloads optionally passed through
shared memory negotiated in the handshake. The same message types are used in MCAP recordings.

## Motivation

Invariant 14 makes the protocol the only link between the viewer and a simulation, even on one
machine. v0.1 builds the viewer against it with a mock server, and v0.2 puts the real server
behind it. The protocol therefore has to be designed for the real server's needs from the start:
world snapshots and deltas with double-precision positions, a unit-aware signal catalog that
changes when domains are toggled, 10 kHz signal streams, commands, profile state, and large
buffers.

Requirements:

| # | Requirement | Source |
|---|---|---|
| 1 | 16+ signals at 10 kHz with no dropped samples; drops must be detectable | CLAUDE.md §13.4 |
| 2 | Scene with 100 000 prims; deltas at 60 Hz | scene-tree budget |
| 3 | Meshes, images and point clouds without copies through a socket when local | invariant 14 |
| 4 | Works across machines | CLAUDE.md §11.6 |
| 5 | Schema evolution with machine-checked breaking-change rules | CLAUDE.md §7 (`buf breaking`) |
| 6 | Implementable in C++ and Python with packages available on Ubuntu 26.04 | §1a |
| 7 | Not WebSocket or WebRTC | CLAUDE.md §4 |

## Design

### Encoding: Protobuf (proto3)

- Messages are defined in `proto/lagrange/viewer/v1/` (drafts in `protocol.md` §4).
- Bulk numeric data (signal samples, transform arrays, vertex data) is carried as raw
  little-endian arrays in `bytes` fields, described by sibling fields (count, components,
  encoding). Protobuf frames and describes; it does not encode each number.
- Available on the platform: `libprotobuf-dev` and `protobuf-compiler` 3.21.12 with a rosdep key
  (`protobuf-dev`). This version predates Protobuf editions, so the files use `syntax = "proto3"`.

### Framing and transport

- Frame: `u32` little-endian length + one `Envelope` message. Maximum 64 MiB.
- Local: Unix domain stream socket at `$XDG_RUNTIME_DIR/lagrange/<instance>.sock`, mode 0600.
- Remote: TCP, default port 47610, bound to localhost unless told otherwise.
- First frames: `ClientHello` → `ServerHello` with protocol version, capabilities, and
  shared-memory regions.
- One connection carries everything; ordering between scene deltas and signals is preserved.

### Shared memory

- Offered only on Unix sockets. Regions are `memfd` file descriptors sent as `SCM_RIGHTS` with
  `ServerHello`. Payloads are referenced by `ShmRef {region, offset, size, generation}`.
- Slots carry a generation header; readers verify it before and after copying (seqlock). No
  cross-process locks.
- Everything must also work in-band (`Blob` chunks), which is how remote viewers and the tests
  without shared memory operate.

### Data model highlights

- **Scene**: `SceneSnapshot` (prims, transforms as packed f64 positions + f32 rotations, meshes
  and materials by `DataRef`, electrical nets) and `SceneDelta` with `sequence`/`base_sequence`
  for loss detection. Attributes are fetched per prim on demand.
- **Signals**: `SignalCatalog` (name, type, unit, native step, writer, substituted flag) with a
  `generation`; `Subscribe` with decimation mode; `SignalBlock` containing `SignalRun`s with
  `first_index` so that gaps are visible.
- **Commands**: one `Command` oneof with `request_id`, answered by `CommandResult`.
- **Profile**: YAML text plus a structured `DomainState` list (availability, substitutes,
  measured cost) and the server's JSON Schema, so the viewer validates with the same schema.
- **Coordinates**: positions are double; `LocalFrame` gives the ECEF origin and rebase count.
- **ROS graph**: delivered by the server; the viewer does not link ROS.

### Versioning

Package `lagrange.viewer.v1`; handshake `(major, minor)`; major mismatch is refused with a
readable reason; minor differences rely on proto3 unknown-field handling. `buf lint` (STANDARD)
and `buf breaking` (WIRE_JSON before 1.0, FILE after) run in CI. Breaking changes before 1.0
need the `proto-break` label and two approvals.

## Alternatives considered

### FlatBuffers instead of Protobuf

| Aspect | Protobuf | FlatBuffers |
|---|---|---|
| Version in Ubuntu 26.04 | 3.21.12 | 23.5.26 |
| Zero-copy read | no (parse step) | yes |
| Breaking-change tooling | `buf breaking` (mature, already required by CLAUDE.md §7) | no equivalent to `buf`; manual review [Medium, from general knowledge of the ecosystems] |
| Python support | mature | available, slower and less idiomatic [Medium] |
| MCAP ecosystem | well supported (Foxglove reads Protobuf channels) | supported |
| Evolution rules | field numbers; well understood | field order/ids; easier to break by accident |

Zero-copy matters only for bulk data, and bulk data is handled by raw arrays in `bytes` fields
or shared memory in this design, so the parse cost of the Protobuf envelope is small (a 25 kB
signal block is one `memcpy`-like parse of a few fields). The tooling advantage decides for
Protobuf.

### Zenoh as the transport

Zenoh is available (`libzenohc-dev`, `libzenohcpp-dev` 1.8.0) and offers discovery, pub/sub,
queries and its own shared memory. Reasons to defer it:

- The viewer–server relation is one stateful session with strict ordering (snapshot, then
  deltas with sequence numbers; catalog generations). A single ordered stream expresses this
  directly; with pub/sub the ordering and resynchronisation logic must be rebuilt across topics.
- Passing `memfd` descriptors needs a Unix socket anyway.
- A second transport doubles the test matrix before there is a user who needs it.
- `rmw_zenoh_cpp` ships its own Zenoh (`zenoh_cpp_vendor` 0.10.6); the server process would hold
  two Zenoh versions if the ROS bridge uses that RMW (conflict potential unverified).

The `Envelope` is transport-neutral, so a Zenoh transport can be added later for
multi-viewer or cross-network discovery without changing messages.

### Other alternatives

| Alternative | Why not |
|---|---|
| gRPC streaming | gRPC 1.51.1 is old in the archive; HTTP/2 framing adds latency and complexity for a local 10 kHz stream; fd passing is not available; a control-plane gRPC API for scripting remains planned separately |
| ROS 2 topics for viewer data | The viewer would need a ROS environment and RMW; high-rate signals would pay serialisation and discovery costs; invariant 14 asks for a dedicated protocol |
| Foxglove WebSocket protocol | WebSocket is excluded (CLAUDE.md §4). Interoperability with Foxglove is kept through MCAP files and the existing `foxglove_bridge` on the ROS side |
| Cap'n Proto | `libcapnp-dev` 1.1.0 exists, but there is no `buf`-like breaking checker and less Python tooling [Medium] |
| Custom binary structs | No schema evolution, no tooling |
| Shared memory only (no socket) when local | Still needs a control channel, and a second code path; the socket + optional shm design covers both cases with one |
| POSIX `shm_open` names instead of `memfd` | Named segments leak on crashes and need cleanup; `memfd` descriptors vanish with the processes |

## Unresolved questions

1. **Compression.** zstd per frame for remote links: negotiate in the handshake (proposed) and
   measure before enabling by default.
2. **Multiple viewers.** The design allows several clients per server with independent
   subscriptions. Who may send commands: all, or one "controller" client? Proposed: all, with the
   last command winning, until a real need for arbitration appears.
3. **Authentication and TLS** for TCP before 1.0: a pre-shared token in `ClientHello` plus TLS,
   or rely on SSH tunnels and document that? Proposed: token + TLS in v0.9.
4. **Back-pressure policy.** When a viewer is slow, the server drops deltas and signal blocks and
   reports counts. Should scopes instead be able to request loss-less delivery with server-side
   buffering up to a limit? Proposed: yes, as a subscription flag with a memory cap, in v0.2.
5. **Texture formats.** PNG/JPEG decode in the viewer versus pre-transcoded KTX2 from the
   server. Proposed: PNG/JPEG in v0.1.
6. **Field volumes.** Dense grid in v0.1; NanoVDB buffers through shared memory from v0.5.
   Remote viewers need a downsampled representation. To be designed with the fields epic.
7. **Recording schema location.** Whether `proto/lagrange/record/v1` imports the viewer package
   (proposed: yes, to reuse `SignalBlock` and scene messages) or duplicates messages to decouple
   file-format stability from live-protocol stability.
8. **Default TCP port** 47610 is arbitrary and unregistered.
9. **`buf lint`** has not been run on the drafts; names may need adjustment.
