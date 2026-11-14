# RFC 0014: Protocol frame limits before the handshake

| | |
|---|---|
| Status | Accepted (2026-11-14) by the owner, with the proposed answer to each open question; gated merges of its steps pre-approved |
| Created | 2026-11-12 |
| Affects | `server/` (connection policy), `proto/framing/` and `proto/transport/` (a C++ API addition, no wire or `.proto` change), `docs/design/protocol.md` §2, `docs/design/security.md` §1, `tools/cli/` (options, additive) |
| Builds on | RFC 0003 (protocol and transport), RFC 0007 (the 1.x freeze) |
| Origin | QA finding #7, `docs/plan/qa-2026-11-12.md` |

## Summary

Before a client has said `ClientHello`, the server treats it as fully trusted with memory and
file descriptors: it accepts any number of connections, keeps each as long as the peer likes, and
will buffer a 64 MiB frame from each. This RFC proposes four server-side limits: a 64 KiB frame
cap until the handshake, at most 16 connections waiting for their handshake and 32 open in
all, at most 8 new TCP connections in a burst and 2 a second after that from one address, and a
10 s handshake timeout. It also proposes enforcing the rule `protocol.md` §2 already states: a
first frame that is not a `ClientHello` closes the connection. None of this changes a byte on
the wire or a line of a `.proto` file, and every conforming v1 client keeps working. The frame
cap needs one small C++ API addition in `proto/framing` and `proto/transport` (a reader whose
limit can be raised after the handshake); the other three limits are server code only, and a
prototype of them is on branch `night2/rfc-drafts`, awaiting acceptance of this RFC.

## Motivation

QA finding #7 (`docs/plan/qa-2026-11-12.md`):

- 8 clients on the Unix socket, each sending a header announcing 60 MiB and then 59 MiB of
  zeros, no `ClientHello`: the server's resident memory went from 85 MB to 626 MB.
- 3000 idle connections were all accepted (the process may open 524 288 files) and cost about
  half a core in `poll()`.
- The same holds on `tcp://` with `--auth-token-file`, because the token is checked only in
  `ClientHello`: on a network-facing server anyone who can reach the port can do it without the
  token.

`security.md` §1 calls the protocol untrusted when it is reachable over TCP, and lists the
64 MiB frame limit as its protection. The limit is the right size for a greeted viewer (it may
send a robot file with `ImportRobot`), and the wrong size for a stranger who has not said who it
is. TCP on loopback is reachable by every local user, not only the one who started the server.

## How it works today

All references are to `origin/dev` at 71880eb.

| Step | Where | What happens |
|---|---|---|
| Listen | `server/src/Server.cpp:6942` → `proto::Listener::listen` (`proto/transport/src/Connection.cpp:609`) | backlog 8 (`Connection.hpp:178`); Unix socket mode 0600 before `bind()`; TCP beyond loopback refused without a token (`security.md` §2) |
| Accept | `Server.cpp:6168` `Impl::accept()` | loops `listener.accept(ec, proto::kDefaultMaxFrameSize, tls)` (line 6172) until the backlog is empty; every connection becomes a `Session` with no limit on their number. `Listener::accept` calls `accept4(…, nullptr, nullptr, …)` (`Connection.cpp:711`): the peer's address is not looked at |
| Frame limit | `Connection::adopt` (`Connection.cpp:230`) | `FrameReader(maxFrameSize)` = 64 MiB (`kDefaultMaxFrameSize`, `proto/framing/include/lagrange/proto/Framing.hpp:26`), fixed for the connection's life: the reader has no way to change it |
| Receive | `Connection::receive` (`Connection.cpp:489–560`) | reads 64 KiB chunks (`kReceiveChunk`, line 28) up to 1 MiB a call (`kReceiveBudgetPerCall`, line 29) and feeds them to the reader (line 560) |
| Frame | `FrameReader::next` (`proto/framing/src/Framing.cpp:76`) | refuses a header over the limit (line 85), otherwise waits until the whole payload is buffered. `feed()` (line 68) appends without a check: the buffer grows with what the peer sends, up to the limit. It never allocates on an unchecked length, so memory costs the attacker the bytes |
| First frame | `Impl::serve` (`Server.cpp:6182`) | a payload that does not parse is skipped (`continue`, line 6196) even before the handshake, so a peer may send unparsable frames forever; a parsed first frame that is not `ClientHello` is refused (`handshake`, line 3150) |
| Handshake | `Impl::handshake` (`Server.cpp:3143`) | version check, then the token on TCP (`proto::sameSecret`, line 3166); TLS has already been negotiated inside `Connection` before any frame |
| Time | — | nothing: a connection that never sends a byte, or never finishes TLS, stays until it closes |
| Clean up | `Server::runOnce` (`Server.cpp:7149`) | closed sessions are erased; `poll()` (line 7060) watches every session's socket each loop |

So one unauthenticated connection can make the server hold 64 MiB plus the 64 KiB receive
scratch, for as long as it stays, and the number of such connections is bounded only by the
process's file limit.

## Design

### 1. A small frame cap until the handshake

Until a session is greeted, its reader accepts frames of at most **64 KiB**
(`kPreHandshakeMaxFrameSize`); a larger header closes the connection (the reader's existing
`Oversize` error). When `handshake()` greets the session, the server raises the limit to the
normal 64 MiB before it reads the next frame.

Raising it between `next()` calls is exact: the reader checks a frame's header only in
`next()`, so frames the client pipelined behind its `ClientHello` (a `Subscribe`, say) are
checked against the raised limit, and nothing is checked twice.

What this needs in `proto/` (C++ only):

```cpp
// proto/framing/include/lagrange/proto/Framing.hpp, class FrameReader
/// Changes the size limit for frames not yet returned by next(). Bytes already buffered stay.
void setMaxFrameSize(std::uint32_t maxFrameSize) { maxFrameSize_ = maxFrameSize; }

// proto/transport/include/lagrange/proto/Connection.hpp, class Connection
/// The largest frame accepted from the peer from now on; queue() keeps its own limit.
void setMaxIncomingFrameSize(std::uint32_t maxFrameSize) { reader_.setMaxFrameSize(maxFrameSize); }
```

The server then accepts with `listener.accept(ec, kPreHandshakeMaxFrameSize, tls)` and, in
`handshake()` after `session.greeted = true`, calls
`session.connection.setMaxIncomingFrameSize(proto::kDefaultMaxFrameSize)`. The outgoing limit
(`Connection::maxFrameSize_`, used by `queue()`) must stay at 64 MiB from the start, because the
server's own first frames (`ServerHello`, the catalog, the profile with its JSON Schema) are not
small: hence a setter for the incoming side only rather than a smaller `maxFrameSize` at accept.

Why 64 KiB: our viewer's `ClientHello` (`viewer/client/src/ViewerClient.cpp:1407–1411`) is a
version, `"lagrange_viewer"` and an optional token: well under 1 KiB. No field of `ClientHello`
(`proto/lagrange/viewer/v1/session.proto:11`) is meant to be large; 64 KiB leaves room for a
long token, many compression names and any later addition, and equals the receive chunk
(`kReceiveChunk`), so one read can hold a whole greeting.

With the cap, an ungreeted connection holds at most 64 KiB + 4 bytes between loops (a frame it
has not finished), plus the 64 KiB receive scratch; within one `serve()` call at most one receive
budget (1 MiB + one chunk) more, and only one connection is served at a time.

### 2. How many connections

| Limit | Default | Over it |
|---|---|---|
| `maxPending`: connections not yet greeted (TLS still negotiating, or no `ClientHello` yet) | 16 | refused at accept |
| `maxConnections`: all open connections, greeted or not | 32 | refused at accept |

The listener still accepts (the backlog must drain, or the kernel queues connections the server
never sees), then refuses (§5) and closes. The pending limit is what protects against strangers;
the total is a resource ceiling for greeted viewers, which on TCP have the token and on a Unix
socket are the same user (each may queue up to 32 MiB of output, `kMaxQueuedBytes`,
`Server.cpp:129`, so 32 viewers bound that at 1 GiB).

Why 16 and 32: a run is watched by one to a few viewers; `lagrange` itself opens one viewer and
one TCP probe (`reachable()`, `tools/cli/lagrange:218`, which connects and closes); the viewer
reconnects every 50 ms only while it has never connected (`ViewerClient.cpp:1370–1374`), one
attempt at a time. Sixteen strangers waiting at once is already an attack or a bug; with §1 they
hold about 2 MiB in all, against 1 GiB today at the same count.

### 3. How fast one address may connect (TCP)

A token bucket per peer address: **8** connections in a burst, then **2 a second**. A connection
over the rate is refused at accept. IPv4 addresses are keyed whole, IPv6 by their /64 prefix
(a single host usually holds a whole /64, so keying by the full address would let it rotate),
IPv4-mapped IPv6 as IPv4. The peer's address comes from `getpeername()` on the accepted socket,
so `Listener::accept` does not change. Unix sockets are exempt: mode 0600 makes every peer the
user who started the server, who could stop it anyway. The table of buckets is bounded (4096
addresses; past that, buckets that have refilled are forgotten, and if none has, all of them),
so the table is not itself a way in.

The bucket is not a defence against a botnet; the pending limit is the backstop. It keeps one
misbehaving script or scanner from occupying all 16 pending slots by itself.

Why 8 and 2 a second: a viewer and the `lagrange` probe make two connections at start; a test
or a person opening a few viewers by hand stays well inside 8; and a client refused for
authentication (which closes, `handshake()`), retrying with the viewer's backoff, is never
throttled by 2 a second.

### 4. Handshake timeout

A connection that is not greeted **10 s** after it was accepted is refused (§5) and closed. The
clock covers the TCP connection, the TLS handshake (run inside `Connection` before any frame)
and the `ClientHello`. The viewer sends its hello as soon as it is connected, so a real client
needs one or two round trips; 10 s covers a TLS 1.3 handshake over a link with a one-second
round trip several times over, and is short enough that the pending slots turn over.

### 5. Refusal, and the first-frame rule

- A connection over a limit or past the timeout is sent a `ServerHello` with `refusal` set to
  the reason ("connection refused: too many connections waiting for their handshake", "... no
  ClientHello within the handshake timeout", ...), and closed. `ServerHello.refusal`
  (`session.proto`, field 9) exists in v1 and every v1 client reads `ServerHello` as the first
  frame and shows a refusal (the viewer keeps it visible while it retries). It is best effort: a
  TLS connection whose handshake has not run is closed without it, and on TCP a close with
  unread input may reach the client as a reset before it reads the frame.
- A first frame whose payload does not parse is refused like one that is not a `ClientHello`
  ("the first message must be ClientHello"), instead of skipped. `protocol.md` §2 already says
  "Any other first frame closes the connection"; this makes the server do it. After the
  handshake a malformed message is still skipped, as now.
- Refusals are logged at most once a second, with the count since the start, so a flood does
  not flood the log (whose last 500 lines go to every new viewer).

### 6. Settings

`ServerOptions::connectionLimits` (`server/include/lagrange/server/Server.hpp`) holds the five
numbers with the defaults above. Proposed command-line options for `lagrange_server` (and so
`lagrange`), all additive: `--max-connections N`, `--max-pending N`, `--handshake-timeout S`,
`--peer-rate BURST,PER_SECOND`. The frame cap before the handshake is a constant, not an option:
nothing legitimate needs it larger.

### 7. The viewer side (optional, same pattern)

A viewer may connect to any server (`security.md` §1: the server is untrusted in the viewer), and
it, too, accepts a 64 MiB first frame. `ServerHello` is small but the frames the server sends
right after it are not, so the same cap applies to the `ServerHello` only: the client raises its
limit once a `ServerHello` without a refusal arrives. This is a change in `viewer/client` only
(with the same `setMaxIncomingFrameSize`), and can come later or not at all; it is listed for
completeness.

## Compatibility with the 1.x freeze (RFC 0007)

| Change | Bytes on the wire | `.proto` files | `buf breaking` | Behaviour for a conforming v1 client |
|---|---|---|---|---|
| §1 frame cap before the handshake | none | none | unaffected | none: its first frame is a `ClientHello` far under 64 KiB |
| §1 `FrameReader::setMaxFrameSize`, `Connection::setMaxIncomingFrameSize` | none | none | unaffected | none. A C++ API addition in `lagrange_proto`; RFC 0007 does not freeze C++ headers, but CLAUDE.md §9 asks an RFC (this one) and two approvals for anything under `proto/` |
| §2 connection limits, §3 rate, §4 timeout | none | none | unaffected | refused only when it is the 17th waiting, the 33rd open, the 9th in a burst from one address, or silent for 10 s |
| §5 refusal text | none: `ServerHello.refusal` exists | none | unaffected | the existing refusal path |
| §5 first-frame rule | none | none | unaffected | none: `protocol.md` §2 already requires a `ClientHello` first |

So the whole proposal is server-side policy within the v1 contract. No additive protocol change
is needed. One is possible and left open (question 3): a `uint64 refused_connections = 11` in
`ServerStats`, which RFC 0007 allows (a new field with a new number).

## Test plan

1. **Frame reader** (`proto/framing/test/test_framing.cpp`): a header one byte over a lowered
   limit is `Oversize`; a frame buffered under a lowered limit and then the limit raised returns
   frames up to the new limit; raising the limit after an `Oversize` does not revive the stream.
2. **Fuzz harness** (`proto/framing/fuzz/FrameReaderHarness.hpp`): a second mode chosen by a bit
   of the first byte starts the reader at a small limit, raises it to `kHarnessMaxFrameSize`
   after the first frame (as the server does after the hello), and checks that no frame over the
   limit in force when it was returned ever comes out and that `bufferedBytes()` never exceeds
   one fed chunk plus the limit plus the header. `test_framing` runs it on its pseudo-random
   streams; `fuzz_frame_reader` under libFuzzer (Clang, `-DLAGRANGE_BUILD_FUZZERS=ON`) runs it
   for long campaigns (`security.md` §3: `TODO(verify)` a nightly hour under ASan once CI is
   back).
3. **Server** (`server/test/test_server.cpp`, `ConnectionLimitsTest.*`, in the prototype):
   silent connections over `maxPending` are refused with the reason, those waiting are refused
   after the timeout, and a viewer is greeted afterwards; greeted viewers count towards
   `maxConnections`, and one leaving lets the next in; an unparsable first frame is refused; a
   TCP address opens its burst, is refused, and gets in again after the rate refills.
4. **The QA repro as a test** (with §1): 8 clients each send a header announcing 60 MiB and
   1 MiB of zeros without a `ClientHello`; every one is closed at its header, and the server's
   `ServerStats.memory_bytes` stays within 10 MiB of where it was. Then 3000 connections in a
   loop (Unix socket, then TCP with a token): at most `maxPending` are ever open, the rest are
   refused, and a real viewer is greeted at the end. Run as a separate test binary with a 60 s
   timeout so that `test_server` does not grow.
5. **Nothing else breaks**: the full `test_server`, `test_transport` (`Tls.*`), the viewer's
   headless tests against a server (`colcon test --packages-select lagrange_viewer`), the
   examples (`tools/ci/check_examples.py`), whose launch files start viewers and probes.

## Prototype

Commit `feat(server): connection limits, handshake timeout and first-frame rule (prototype for
RFC 0014)` on `night2/rfc-drafts` implements §2–§6 without the command-line options, and the
tests of item 3. It changes nothing under `proto/`, so it does not include §1, which is the part
that closes the QA repro (with the prototype alone, 16 waiting connections can still each hold
64 MiB: 1 GiB, against an unbounded amount today). It was merged to `dev` with #331; §1 and the
rest followed after acceptance (Implementation, below).

## Implementation (2026-11-14)

What landed after acceptance, on `feat/rfc-0014-frame-limits` (the §2–§5 prototype was already
on `dev` with #331):

| Part | Where | Notes |
|---|---|---|
| §1 frame cap | `FrameReader::setMaxFrameSize`, `proto::kPreHandshakeMaxFrameSize` (`Framing.hpp`), `Connection::setMaxIncomingFrameSize` and `maxIncomingFrameSize` (`Connection.hpp`); `Server.cpp` `accept()` lowers the incoming limit to 64 KiB, `handshake()` raises it to 64 MiB after `greeted = true` | C++ additions only: no wire or `.proto` change. A frame over the cap before the handshake is refused ("connection refused: a frame larger than 64 KiB before the handshake") and closed. The constant lives in `lagrange_proto` so that the viewer side (§7) can use it later |
| §6 options | `lagrange_server --max-connections N --max-pending N --handshake-timeout S --peer-rate BURST,PER_SECOND` (`CommandLine.cpp`, `test_command_line`) | additive; the 64 KiB cap is a constant |
| Question 4 | `ServerStats.viewers` counts greeted, open sessions only | |
| Test plan 1 | `test_framing`: `FrameReader.OneOverALoweredLimitIsOversize`, `RaisingTheLimitAppliesToFramesAlreadyBuffered`, `AHeaderBufferedUnderTheLimitIsCheckedWhenItComesOut`, `RaisingTheLimitAfterOversizeDoesNotReviveTheStream`; `test_transport`: `TransportTest.TheIncomingLimitCanBeRaisedAfterTheHello` | |
| Test plan 2 | `FrameReaderHarness.hpp`: the top bit of the first byte picks the second mode (64 bytes, raised to 4096 after the first frame; aborts on a frame over the limit in force or a buffer over one chunk + limit + header). `test_framing` runs every pseudo-random stream in both modes | `fuzz_frame_reader` built with Clang 21 and `-DLAGRANGE_BUILD_FUZZERS=ON` (on Ubuntu 26.04 Clang needs `--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/15` in `CMAKE_CXX_FLAGS` when GCC 16's libstdc++ dev files are not installed) and run for 120 s, 74 million inputs, no crash; without ASan. `TODO(verify)`: the nightly hour under ASan once CI is back |
| Test plans 3 and 4 | `server/test/test_connection_limits.cpp`, binary `test_connection_limits` (60 s timeout; the prototype's `ConnectionLimitsTest.*` moved there from `test_server`) | QA #7 as tests: 8 clients announcing 60 MiB and offering 8 MiB each are refused at their header, and resident memory stays within 10 MiB (with the cap disabled the same test measured 76 MB → 189 MB and failed); 3000 connections in a loop on a Unix socket and on TCP with a token leave at most 16 waiting, all others are refused, memory stays flat and a viewer is greeted at the end. Also `OnlyGreetedSessionsCountAsViewers` and `TheFrameLimitIsRaisedByTheHandshake` (a 100 KiB frame pipelined behind the `ClientHello` is accepted; the same frame from a stranger is refused) |
| Not done | §7 (the viewer-side cap) and question 6 (the mock server), as proposed: later or not at all. Question 3: no `refused_connections` field | |

## Alternatives considered

| Alternative | Why not |
|---|---|
| A smaller frame limit for everything | A greeted viewer sends robot files inline (`ImportRobot.content`, a `DataRef`) and profile text; a smaller limit would break remote imports of large robots, a v1 behaviour |
| Check the token before reading frames (a token prefix outside the protobuf) | A wire change: every client would have to send it; needs `lagrange.viewer.v2` under RFC 0007 |
| Rely on TLS client certificates instead of the token | Changes deployment for every remote user; the token is documented (`security.md` §2). Could be added later as an option without replacing these limits |
| `setrlimit(RLIMIT_NOFILE)` or the backlog as the only limit | They limit the whole process (MuJoCo, plugins, recordings, ROS) and refuse nothing with a reason |
| Per-address limits on Unix sockets via `SO_PEERCRED` | Every peer is the same user (mode 0600) |
| Limits inside `proto::Listener` | The listener does not know which sessions are greeted; the server owns that state, and the mock server and tests use the listener with other needs |

## Open questions (with a proposed answer)

1. *Are the defaults right* (64 KiB, 16 pending, 32 open, 8 + 2/s, 10 s)? Proposed: yes, with
   the reasoning above; all but the frame cap can be changed per run.
2. *Should a loopback TCP server without a token also be limited per address?* Every local user
   shares 127.0.0.1, so the bucket throttles them together. Proposed: yes; the limits are loose
   enough for normal use, and a run that needs more raises them.
3. *A `refused_connections` counter in `ServerStats` (field 11)?* Proposed: not now; the log line
   carries the count. Add it when a viewer wants to show it.
4. *Should `ServerStats.viewers` count greeted sessions only?* It counts every session
   (`Server.cpp:6133`), including ones still waiting. Proposed: yes, greeted only, as a fix
   with the prototype's merge.
5. *The viewer-side cap (§7)*: now or later? Proposed: later, in its own change.
6. *The Python mock server* (`mock_server/`, a test tool, Q15) has the same 64 MiB limit
   (`framing.py:13`) and no limits. Proposed: leave it; it is not shipped as a server for
   others to reach.
