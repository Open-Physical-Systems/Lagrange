# Security review (v1.0)

The v1.0 security review (roadmap §9): what Lagrange trusts, how the viewer protocol is
protected, what is fuzzed, and the state of the dependencies. Written on 2026-10-29 for v1.0.0.
Reporting a vulnerability: `SECURITY.md`.

## 1. What is trusted

| Input | Trust | Protection |
|---|---|---|
| Viewer protocol from a client | **Untrusted** when reachable over TCP | Unix sockets are created mode 0600 before `bind()` (only the user). TCP: a shared token in `ClientHello.auth_token` (compared in constant time, `proto::sameSecret`); listening beyond loopback without one is refused; TLS 1.2+ on `tls://` endpoints (§2). Frames are length-limited (`FrameReader`, 64 MiB default), and every size, index and reference in a scene message is validated before use (`SceneBuilder`) |
| Viewer protocol from a server, in the viewer | **Untrusted** (a viewer may connect to any server) | The same frame limits and checks; TLS verifies the server's certificate and host name or address (`--tls-ca` or the system's trust store) |
| Robot and world files (URDF, SDFormat, MJCF, USD), profiles, recordings, OpenStreetMap, elevation rasters, 3D Tiles | Trusted as code-adjacent content: like the files a user opens in an editor | Parsed by OpenUSD, Assimp, tinyxml2, jsoncons, GDAL and the MCAP reader at load or build time, never during a step. Do not open files from someone you do not trust in a run that has access to secrets |
| Plugins (`core/plugin_abi`, `dlopen`) and FMUs | **Code execution** by design | Load only plugins and FMUs you would run as programs. The ABI checks the major version; it cannot sandbox |
| ROS 2 topics and services | As ROS 2 itself: the DDS domain is the boundary | Use SROS2 and a private `ROS_DOMAIN_ID` / `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on shared networks |

## 2. The viewer protocol over a network

```bash
# A certificate and key (PEM), e.g. from your CA or, for a test, self-signed:
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes -days 365 \
    -subj /CN=sim-host -addext subjectAltName=DNS:sim-host -keyout key.pem -out cert.pem
head -c 32 /dev/urandom | base64 > token.txt; chmod 600 token.txt
lagrange_server --listen tls://0.0.0.0:47610 --tls-cert cert.pem --tls-key key.pem \
    --auth-token-file token.txt
# On the other machine:
lagrange_viewer --connect tls://sim-host:47610 --tls-ca cert.pem --auth-token-file token.txt
```

- `tcp://` on loopback needs nothing; `tcp://` or `tls://` beyond loopback needs
  `--auth-token-file`. Send a token over `tls://` only: over `tcp://` it travels in clear text.
- The viewer also reads `LAGRANGE_TLS_CA` and `LAGRANGE_AUTH_TOKEN`.
- Shared memory (the `memfd` fast path) is offered on Unix sockets only.

Checked by `Tls.*` (`proto/transport/test/test_transport.cpp`: frames both ways over TLS, a
server with an untrusted certificate refused, `tls://` without a client context refused) and
`Security.TlsWithATokenGreetsOnlyClientsThatHaveIt` (`server/test/test_server.cpp`).

## 3. Fuzzing

| Decoder | Harness | Run in every build | libFuzzer (Clang, `-DLAGRANGE_BUILD_FUZZERS=ON`) |
|---|---|---|---|
| Frame reader (length prefix, limits) | `proto/framing/fuzz/FrameReaderHarness.hpp` | `test_framing`: pseudo-random streams | `fuzz_frame_reader` |
| Scene snapshots and deltas | `viewer/client/fuzz/SceneBuilderHarness.hpp` | `SceneFuzz.MutatedFramesNeverBreakTheSceneBuilder`: every golden frame and a valid snapshot, each mutated 2000 ways (54 000 inputs) | `fuzz_scene_builder` |

The libFuzzer targets have not been run for a long campaign on this machine (GitHub Actions,
where a nightly job would run them, are disabled); `TODO(verify)`: a nightly run of each for an
hour under ASan once CI is back. Not fuzzed: the USD, URDF, MJCF and OpenStreetMap readers
(trusted content, §1), and the server's command handling beyond the frame and the protobuf
parser.

## 4. Dependencies

Every dependency, its version, source and licence is in `dependencies.md`. For security:

- **From Ubuntu 26.04 and ROS Lyrical** (apt): Protobuf, OpenSSL 3.5, Embree, Assimp,
  tinyxml2, jsoncons, GDAL, PROJ, SDL3, Vulkan, NanoVDB/OpenVDB headers, MuJoCo (the ROS vendor
  package), SDFormat, rclcpp and the message packages. They get security fixes through the
  distribution's updates; keep the machine updated.
- **Vendored at a pinned version** (`vendor/`): OpenUSD v26.08, Jolt v5.6.0, FMI Library 3.0.4,
  Dear ImGui and ImPlot. They do not update with the system: a fix needs a new pin and a
  Lagrange release. `TODO(verify)`: an advisory scan of these four (OSV or the projects'
  security pages) before each release; none was run for v1.0.0 (no scanner on this machine).
- **Python** (pip, `packaging/pip-constraints.txt`): mcap and coacd, pinned by the
  constraints file.
- **Licences**: Apache-2.0 compatible throughout (`dependencies.md`); no GPL in linked code.

## 5. Open items

- An advisory scan of the vendored packages, and long fuzzing campaigns (§3, §4).
- Signed packages: Sigstore signatures for the release artefacts are the owner's to make
  (`cosign`, with the owner's identity); the `.deb`s, container image and wheel are built and
  checked locally (roadmap §9).
- SROS2 for the ROS 2 bridge is documented, not tested.
