# UI↔server protocol

Status: draft for review; decided in `rfcs/0003-ui-server-protocol.md`. This is the only channel
between `lagrange_viewer` and a simulation (invariant 14). The mock server and the real server
implement the same protocol; the viewer cannot tell them apart except through
`ServerHello.server_kind`.

## 1. Decisions proposed

| Topic | Proposal | Basis |
|---|---|---|
| Encoding | Protobuf, proto3 syntax | `libprotobuf-dev` 3.21.12 and a rosdep key exist on Resolute; `buf` gives lint and breaking-change checks. No editions, because 3.21 predates them |
| Bulk numeric data | Raw little-endian arrays inside `bytes` fields | Avoids per-sample varint cost for 10 kHz signals and large transform sets |
| Transport, same machine | Unix domain stream socket | Needed anyway to pass shared-memory file descriptors (`SCM_RIGHTS`) |
| Transport, other machine | TCP | Same framing |
| Shared memory | `memfd_create` regions passed over the Unix socket | Linux-only by design; hidden behind `viewer/client/` and `server/frontend/` |
| Zenoh | Not in v0.1. Reserved as a later transport behind the same `Envelope` | `libzenohc-dev` 1.8.0 is available, but a second transport adds test surface before there is a user need |
| Security | Unix socket: file mode 0600. TCP: bound to 127.0.0.1 unless `--listen` is given; no authentication in v0.1 (documented), TLS + token later | Stated in `SECURITY.md` |

## 2. Framing

Every message on the stream is:

```
+----------------+----------------------------+
| length: u32 LE | Envelope (protobuf), length bytes |
+----------------+----------------------------+
```

- Maximum frame size: 64 MiB. Larger payloads (meshes, textures, point clouds) are chunked with
  `Blob` messages or sent through shared memory.
- Until the server has accepted its `ClientHello`, a client may send frames of at most 64 KiB
  (`proto::kPreHandshakeMaxFrameSize`; RFC 0014 §1). A larger header closes the connection with
  a `ServerHello` whose `refusal` says why. Frames pipelined behind the `ClientHello` are read
  under the normal 64 MiB limit. What the server sends keeps the 64 MiB limit from the start.
- The first frame in each direction is a handshake (`ClientHello`, then `ServerHello`). Any
  other first frame, and one whose payload does not parse, closes the connection, with a
  `ServerHello` refusal ("the first message must be ClientHello").
- Before the handshake the server also limits connections (RFC 0014 §2 to §4): at most 16
  waiting for their `ClientHello` and 32 open in all, at most 8 new TCP connections in a burst
  and then 2 a second from one address (IPv6: per /64), and 10 s from accept to the
  `ClientHello` (TLS included). Over a limit a connection is sent a `ServerHello` refusal
  (best effort; not over TLS before its handshake) and closed. `lagrange_server`
  `--max-connections N`, `--max-pending N`, `--handshake-timeout S`,
  `--peer-rate BURST,PER_SECOND` change them; the 64 KiB cap is a constant.
  `ServerStats.viewers` counts greeted sessions only.
- Default endpoints: Unix socket `$XDG_RUNTIME_DIR/lagrange/<instance>.sock` (instance defaults to
  `default`); TCP port `47610` (a project default, configurable; not an IANA registration).

## 3. Message catalog

| Direction | Message | When | Size class |
|---|---|---|---|
| C→S | `ClientHello` | first frame | small |
| S→C | `ServerHello` | reply | small |
| S→C | `ServerStatus` | 10 Hz and on change | small |
| S→C | `SignalCatalog` | after hello and when wiring changes | medium |
| C→S | `Subscribe` / `Unsubscribe` | scopes opened/closed | small |
| S→C | `SignalBlock` | per subscription, batched (default every 20 ms) | medium |
| S→C | `SceneSnapshot` | on request and after a delta out of sequence | large (chunked/shm) |
| S→C | `SceneDelta` | at viewport rate (default 60 Hz) | medium |
| S→C | `Blob` | chunks of meshes, textures, images | large |
| C→S | `Command` | user actions | small |
| S→C | `CommandResult` | one per command | small |
| S→C | `ProfileState` | after hello and on change | small |
| S→C | `LogRecord` | console output | small |
| S→C | `RosGraph` | 1 Hz when the ROS panel is open | medium |
| S→C | `SensorFrame` | subscribed sensor views (image, point cloud) | large (shm when local) |
| S→C | `EventList` | after hello and when another run is loaded | medium |
| S→C | `Event` | when something happens in a live run | small |
| S→C | `ServerStats` | 1 Hz | small |
| S→C | `RecordingInfo` | after hello and when another recording is opened (replay) | small |
| both | `Ping` / `Pong` | every 2 s; timeout 10 s | small |

The viewer does not link `rclcpp`. The ROS 2 topic browser is filled from `RosGraph`, which the
server's ROS bridge produces. The mock server fills it from the MCAP channel list.

## 4. Draft `.proto` files

Layout: `proto/lagrange/viewer/v1/*.proto`, package `lagrange.viewer.v1`. `buf.yaml` is in
`docs/plan/repo-tooling.md`.

### 4.1 `common.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

// Simulation time, integer nanoseconds since run start.
message SimTime { int64 ns = 1; }

// SI dimension exponents + scale + kind. Mirrors lagrange::Unit.
message Unit {
  sint32 length = 1;
  sint32 mass = 2;
  sint32 time = 3;
  sint32 current = 4;
  sint32 temperature = 5;
  sint32 amount = 6;
  sint32 luminous = 7;
  sint32 angle = 8;
  double scale = 9;          // 1.0 for SI
  uint32 kind = 10;          // lagrange::QuantityKind
  string display = 11;       // "N·m", for labels only
}

enum SignalType {
  SIGNAL_TYPE_UNSPECIFIED = 0;
  SIGNAL_TYPE_SCALAR = 1;
  SIGNAL_TYPE_VEC3 = 2;
  SIGNAL_TYPE_QUAT = 3;
  SIGNAL_TYPE_POSE = 4;
  SIGNAL_TYPE_WRENCH = 5;
  SIGNAL_TYPE_SCALAR_ARRAY = 6;
  SIGNAL_TYPE_BOOL = 7;
  SIGNAL_TYPE_INT = 8;
  SIGNAL_TYPE_BYTES = 9;
  SIGNAL_TYPE_FIELD3D = 10;
  SIGNAL_TYPE_IMAGE = 11;
  SIGNAL_TYPE_POINT_CLOUD = 12;
}

// A payload that is either inline, chunked through Blob messages, or in shared memory.
message DataRef {
  oneof location {
    bytes inline = 1;
    uint64 blob_id = 2;        // assembled from Blob chunks
    ShmRef shm = 3;
  }
  uint64 size = 4;             // total bytes
  fixed64 xxh3 = 5;            // content hash; also the cache key for meshes and textures
}

message ShmRef {
  uint32 region = 1;           // index announced in ServerHello / ShmRegionAdded
  uint64 offset = 2;
  uint64 size = 3;
  uint64 generation = 4;       // must equal the slot header's generation when read
}

message Blob {
  uint64 blob_id = 1;
  uint64 offset = 2;
  bytes data = 3;              // <= 4 MiB per chunk
  bool last = 4;
}
```

### 4.2 `session.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

import "lagrange/viewer/v1/common.proto";

message ProtocolVersion { uint32 major = 1; uint32 minor = 2; }

message ClientHello {
  ProtocolVersion version = 1;
  string client_name = 2;            // "lagrange_viewer 0.1.0"
  bool wants_shm = 3;                // only honoured on Unix sockets
  repeated string compression = 4;   // "zstd"; empty = none
  string locale = 5;                 // "ja_JP", for server-side message text (later)
  string auth_token = 6;   // v1.0: the token a TCP server requires (security.md)
}

enum ServerKind {
  SERVER_KIND_UNSPECIFIED = 0;
  SERVER_KIND_SIM = 1;
  SERVER_KIND_MOCK = 2;
  SERVER_KIND_REPLAY = 3;
}

message ServerHello {
  ProtocolVersion version = 1;
  ServerKind server_kind = 2;
  string server_name = 3;
  uint64 session_id = 4;
  bool shm_enabled = 5;              // file descriptors follow as ancillary data
  repeated ShmRegion shm_regions = 6;
  string compression = 7;            // chosen, or empty
  Capabilities capabilities = 8;
  string refusal = 9;                // non-empty = connection refused, human-readable reason
}

message ShmRegion { uint32 region = 1; uint64 size = 2; string purpose = 3; }

message Capabilities {
  bool can_seek = 1;                 // replay/mock
  bool can_step = 2;
  bool can_edit_scene = 3;
  bool can_apply_profile = 4;
  bool can_import = 5;               // URDF/SDF/MJCF import through the server
  uint32 tier = 6;                   // 0, 1, 2
  bool has_ros_graph = 7;
}

enum RunState {
  RUN_STATE_UNSPECIFIED = 0;
  RUN_STATE_STOPPED = 1;
  RUN_STATE_PAUSED = 2;
  RUN_STATE_RUNNING = 3;
  RUN_STATE_ERROR = 4;
}

message ServerStatus {
  SimTime sim_time = 1;
  RunState state = 2;
  double real_time_factor = 3;       // measured
  double real_time_factor_target = 4;
  SimTime replay_begin = 5;          // replay only
  SimTime replay_end = 6;
  uint64 dropped_deltas = 7;
  uint64 dropped_signal_blocks = 8;
  string error = 9;
}

message Ping { uint64 nonce = 1; int64 client_monotonic_ns = 2; }
message Pong { uint64 nonce = 1; int64 client_monotonic_ns = 2; SimTime sim_time = 3; }

enum LogLevel { LOG_LEVEL_UNSPECIFIED = 0; LOG_LEVEL_DEBUG = 1; LOG_LEVEL_INFO = 2; LOG_LEVEL_WARN = 3; LOG_LEVEL_ERROR = 4; }

message LogRecord {
  SimTime sim_time = 1;
  int64 wall_unix_ns = 2;
  LogLevel level = 3;
  string source = 4;                 // "scheduler", "electrical/averaged_network"
  string text = 5;
}

message RosGraph {
  message Topic { string name = 1; string type = 2; uint32 publishers = 3; uint32 subscribers = 4; double rate_hz = 5; }
  repeated Topic topics = 1;
  repeated string nodes = 2;
  string rmw = 3;
}
```

### 4.3 `scene.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

import "lagrange/viewer/v1/common.proto";

// Positions are double. The snapshot states the origin of the local frame so that a viewer can
// show geodetic coordinates and survive a floating-origin rebase.
message LocalFrame {
  double ecef_x = 1; double ecef_y = 2; double ecef_z = 3;   // m; all zero if not georeferenced
  repeated double enu_from_ecef = 4;                         // 9 values, row-major
  uint64 rebase_count = 5;                                   // increments at each rebase
  repeated double region_enu = 6;  // 3 values: world ENU of the region-local origin, m (v0.7)
}

message Prim {
  uint32 id = 1;                 // dense, stable for the session
  uint32 parent = 2;             // 0xFFFFFFFF for roots
  string name = 3;
  string path = 4;               // full USD path
  string type_name = 5;          // "Xform", "Mesh", "PhysicsRevoluteJoint"
  repeated string api_schemas = 6;
  uint32 mesh = 7;               // index into SceneSnapshot.meshes, or 0xFFFFFFFF
  uint32 material = 8;
  bool visible = 9;
  uint32 child_count = 10;       // lets the scene tree virtualize without the whole list
}

message Attribute {
  string name = 1;               // "elec:windingResistance"
  oneof value {
    double real = 2;
    int64 integer = 3;
    bool boolean = 4;
    string token = 5;
    string asset = 6;
    RealArray reals = 7;
    Targets targets = 8;         // relationship
  }
  Unit unit = 9;
  bool editable = 10;
  bool authored = 11;            // false = schema fallback
  string doc = 12;
}
message RealArray { repeated double v = 1; }
message Targets { repeated uint32 prims = 1; }

message PrimAttributes { uint32 prim = 1; repeated Attribute attributes = 2; }

message Mesh {
  uint32 id = 1;
  uint32 vertex_count = 2;
  uint32 index_count = 3;
  DataRef positions = 4;         // float32 x3, mesh-local
  DataRef normals = 5;           // float32 x3
  DataRef uvs = 6;               // float32 x2, optional
  DataRef indices = 7;           // uint32
  repeated float aabb = 8;       // min xyz, max xyz
}

message Material {
  uint32 id = 1;
  string name = 2;
  repeated float base_color = 3; // rgba
  float metallic = 4;
  float roughness = 5;
  repeated float emissive = 6;
  DataRef base_color_texture = 7;      // PNG/JPEG/KTX2 bytes, optional
  string base_color_texture_mime = 8;
}

// Electrical topology for the schematic panel. Derived from PortAPI / ElectricalBusAPI prims.
message ElectricalNet {
  uint32 id = 1;
  string name = 2;                     // "dc_48v"
  uint32 bus_prim = 3;
  repeated NetTerminal terminals = 4;
  string voltage_signal = 5;           // signal name, empty if none
}
message NetTerminal { uint32 prim = 1; string port = 2; string current_signal = 3; }

message SceneSnapshot {
  uint64 sequence = 1;                 // deltas apply on top of this
  SimTime sim_time = 2;
  LocalFrame frame = 3;
  repeated Prim prims = 4;
  DataRef transforms = 5;              // prim_count x (3 f64 position + 4 f32 quat + 3 f32 scale), prim id order
  repeated Mesh meshes = 6;
  repeated Material materials = 7;
  repeated ElectricalNet nets = 8;
  uint64 topology_hash = 9;
  string stage_path = 10;
  // Attributes are not included. The inspector requests them per prim (GetAttributes command).
}

message SceneDelta {
  uint64 sequence = 1;                 // snapshot.sequence + n
  uint64 base_sequence = 2;            // must equal the client's current sequence, else resync
  SimTime sim_time = 3;
  // Changed transforms: parallel arrays.
  repeated uint32 transform_prims = 4 [packed = true];
  bytes transform_data = 5;            // per entry: 3 f64 + 4 f32 (scale never changes in a delta)
  repeated PrimAttributes attribute_changes = 6;
  repeated Prim added = 7;
  repeated uint32 removed = 8 [packed = true];
  LocalFrame rebased_frame = 9;        // set only when the floating origin moved
  repeated Overlay overlays = 10;      // contact points, force arrows
}

message Overlay {
  enum Kind { KIND_UNSPECIFIED = 0; KIND_CONTACT_POINT = 1; KIND_FORCE_ARROW = 2; KIND_TORQUE_ARROW = 3; KIND_PATH = 4; }
  Kind kind = 1;
  bytes data = 2;                      // kind-specific packed f64: point(3)+normal(3)+magnitude(1) ...
  uint32 count = 3;
}

message SensorFrame {
  string sensor = 1;                   // "sensor/front_lidar"
  SimTime sim_time = 2;
  oneof payload { ImageData image = 3; PointCloudData points = 4; }
}
message ImageData { uint32 width = 1; uint32 height = 2; string encoding = 3; DataRef data = 4; }   // "rgba8", "r32f"
message PointCloudData { uint32 count = 1; uint32 stride = 2; repeated string fields = 3; DataRef data = 4; }   // "x","y","z","intensity"
```

### 4.4 `signals.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

import "lagrange/viewer/v1/common.proto";

message SignalInfo {
  uint32 id = 1;                 // dense, stable until the catalog generation changes
  string name = 2;               // "joint/left_wheel/torque"
  SignalType type = 3;
  Unit unit = 4;
  uint32 components = 5;         // f64 components per sample
  int64 native_dt_ns = 6;        // writer's step; 0 for event-driven
  string writer = 7;             // "electrical/averaged_network"
  bool substituted = 8;          // written by a substitute
  uint32 prim = 9;               // related prim or 0xFFFFFFFF
  repeated string component_names = 10;   // "x","y","z"
}

message SignalCatalog {
  uint64 generation = 1;         // changes when wiring changes (profile applied, stage reloaded)
  repeated SignalInfo signals = 2;
}

message Subscribe {
  uint64 catalog_generation = 1;
  repeated SignalSubscription signals = 2;
}
message SignalSubscription {
  uint32 signal = 1;
  // 1 = every sample. n > 1 = server sends min/max pairs over n samples (decimation="minmax")
  // or every n-th sample (decimation="stride").
  uint32 decimation = 2;
  enum Mode { MODE_UNSPECIFIED = 0; MODE_FULL = 1; MODE_MINMAX = 2; MODE_STRIDE = 3; }
  Mode mode = 3;
  uint32 component_mask = 4;     // bit i = component i; 0 = all
}
message Unsubscribe { repeated uint32 signals = 1; }

enum SampleEncoding {
  SAMPLE_ENCODING_UNSPECIFIED = 0;
  SAMPLE_ENCODING_F64_LE = 1;
  SAMPLE_ENCODING_F32_LE = 2;    // allowed only for MINMAX display streams
}

// A run of consecutive samples of one signal.
message SignalRun {
  uint32 signal = 1;
  SimTime t0 = 2;                // time of the first sample
  int64 dt_ns = 3;               // uniform spacing; 0 = see timestamps
  uint32 sample_count = 4;
  uint32 components = 5;
  SampleEncoding encoding = 6;
  bytes samples = 7;             // sample_count x components values; MINMAX: x2 (min then max)
  bytes timestamps_ns = 8;       // int64 LE per sample, only when dt_ns == 0
  uint64 first_index = 9;        // writer step index of the first sample; gaps reveal drops
  SignalSubscription.Mode mode = 10;
}

// All runs produced since the previous block (default: 20 ms of simulation per block).
message SignalBlock {
  uint64 catalog_generation = 1;
  repeated SignalRun runs = 2;
}
```

Sizing check for the §13.4 budget: 16 scalar signals at 10 kHz in `MODE_FULL`, `F64_LE`,
blocks every 20 ms → 16 × 200 samples × 8 bytes = 25.6 kB per block, 1.28 MB/s. This is small for
a local socket and acceptable on a LAN.

### 4.5 `command.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

import "lagrange/viewer/v1/common.proto";
import "lagrange/viewer/v1/scene.proto";

message Command {
  uint64 request_id = 1;               // echoed in CommandResult
  oneof command {
    Play play = 10;
    Pause pause = 11;
    Step step = 12;
    Seek seek = 13;
    SetRealTimeFactor set_rtf = 14;
    Reset reset = 15;

    RequestSnapshot request_snapshot = 20;
    GetAttributes get_attributes = 21;
    SetAttribute set_attribute = 22;
    SetTransform set_transform = 23;
    SetVisibility set_visibility = 24;

    LoadStage load_stage = 30;
    ImportRobot import_robot = 31;
    OpenRecording open_recording = 32;
    RemoveRobot remove_robot = 33;     // added after 1.0 (RFC 0016 §7.2)

    ValidateProfile validate_profile = 40;
    ApplyProfile apply_profile = 41;
    GetProfile get_profile = 42;

    SubscribeSensor subscribe_sensor = 50;
    SetRosGraphEnabled set_ros_graph = 51;
    SetOverlays set_overlays = 52;
  }
}

message Play {}
message Pause {}
message Step { uint32 ticks = 1; int64 duration_ns = 2; }      // one of the two; 0 = unused
message Seek { SimTime to = 1; }                               // replay only
message SetRealTimeFactor { double target = 1; }               // 0 = unlimited
message Reset {}

message RequestSnapshot {}
message GetAttributes { repeated uint32 prims = 1; }
message SetAttribute { uint32 prim = 1; Attribute attribute = 2; bool save_to_session_layer = 3; }
message SetTransform { uint32 prim = 1; repeated double position = 2; repeated double quat_wxyz = 3; }
message SetVisibility { uint32 prim = 1; bool visible = 2; }

message LoadStage { string path = 1; }
message ImportRobot {
  enum Format { FORMAT_UNSPECIFIED = 0; FORMAT_URDF = 1; FORMAT_XACRO = 2; FORMAT_SDF = 3; FORMAT_MJCF = 4; }
  Format format = 1;
  string path = 2;                     // path on the server's filesystem
  DataRef content = 3;                 // or the file content, inline, when the viewer is remote
  string overlay_path = 4;             // on the server's filesystem
  repeated string xacro_args = 5;
  DataRef overlay_content = 6;         // or the overlay's text, inline; added after 1.0
}
message OpenRecording { string path = 1; }
message RemoveRobot { string name = 1; } // added after 1.0 (RFC 0016 §7.2)
// ImportRobot on lagrange_server (can_import): the robot is spawned into the running
// simulation, as simulation_interfaces' spawn_entity does (docs/guide/cli.md "Spawning
// robots"), at the origin of an empty run or 3 m beside the robots there are. By `path`, the
// result carries the path in imported_stage_path. By `content` (inline only; blob and shm are
// refused with "send the robot inline"), the text is written to a temporary file, the robot is
// named by its own <robot name> (SDFormat: the first <model name>; MJCF: <mujoco model>),
// renamed `name_2`, `name_3`… on a clash, and the result's imported_stage_path is its scene
// path, "/name": the name RemoveRobot takes. A `format` other than FORMAT_UNSPECIFIED must
// match the text. `overlay_path`, or `overlay_content` (inline YAML, or a SPICE netlist ending
// in `.end`), is applied to that robot as --overlay is (formats.md §9). `xacro_args` and
// FORMAT_XACRO are refused (run xacro first). The robot is passive (nothing drives it) when the
// run already has one; else it is the robot ROS drives.
// RemoveRobot: as /delete_entity (cli.md "Spawning robots"), by top prim name with or without
// "/"; refused for the robot ros2_control drives and for scenery. A server older than these
// additions answers an unknown command with ok = false and an error, so a viewer can try it.

message ValidateProfile { string yaml = 1; }
message ApplyProfile { string yaml = 1; bool restart = 2; }
message GetProfile {}

message SubscribeSensor { string sensor = 1; bool enabled = 2; double max_rate_hz = 3; }
message SetRosGraphEnabled { bool enabled = 1; }
message SetOverlays { bool contacts = 1; bool forces = 2; string field_signal = 3; }

message CommandResult {
  uint64 request_id = 1;
  bool ok = 2;
  string error = 3;                    // human-readable, one line
  oneof result {
    PrimAttributesList attributes = 10;
    ProfileValidation validation = 11;
    string imported_stage_path = 12;
  }
}
message PrimAttributesList { repeated PrimAttributes items = 1; }

message ProfileValidation {
  bool valid = 1;
  repeated ProfileIssue issues = 2;
}
message ProfileIssue {
  enum Severity { SEVERITY_UNSPECIFIED = 0; SEVERITY_ERROR = 1; SEVERITY_WARNING = 2; }
  Severity severity = 1;
  string json_pointer = 2;             // "/domains/electrical/dt"
  string message = 3;
}
```

### 4.6 `profile.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

// The profile itself travels as YAML text, the same text a user would save. The structured
// part below is what the domain panel needs in order to draw toggles without parsing YAML rules.
message ProfileState {
  string yaml = 1;                     // the active profile, canonical formatting
  uint32 schema_version = 2;
  string json_schema = 3;              // profiles/schema.json content, so the viewer validates with the server's schema
  repeated DomainState domains = 4;
  bool restart_required = 5;           // the last ApplyProfile needs a restart to take effect
}

message DomainState {
  string domain = 1;                   // "electrical"
  bool enabled = 2;
  bool available = 3;                  // false: cannot be enabled for this stage/tier
  string unavailable_reason = 4;       // "no ElectricalMotorAPI prims in stage"
  string active_module = 5;            // "averaged_network" or the substitute name
  bool substituted = 6;
  repeated SubstituteOption substitutes = 7;
  repeated string backends = 8;        // rigid_body: physx, mujoco, jolt (as installed)
  repeated string fidelities = 9;
  int64 dt_ns = 10;
  int64 min_dt_ns = 11;
  int64 max_dt_ns = 12;
  double cpu_ms_per_sim_s = 13;        // measured cost, for the panel's load bar
  repeated string depends_on = 14;     // domains whose signals this one reads
}

message SubstituteOption { string token = 1; string kind = 2; string description = 3; bool needs_source = 4; }
```

### 4.7 `envelope.proto`

```proto
syntax = "proto3";
package lagrange.viewer.v1;

import "lagrange/viewer/v1/common.proto";
import "lagrange/viewer/v1/session.proto";
import "lagrange/viewer/v1/scene.proto";
import "lagrange/viewer/v1/signals.proto";
import "lagrange/viewer/v1/command.proto";
import "lagrange/viewer/v1/profile.proto";

message Envelope {
  oneof body {
    ClientHello client_hello = 1;
    ServerHello server_hello = 2;
    ServerStatus status = 3;
    Ping ping = 4;
    Pong pong = 5;
    LogRecord log = 6;
    RosGraph ros_graph = 7;

    SignalCatalog catalog = 10;
    Subscribe subscribe = 11;
    Unsubscribe unsubscribe = 12;
    SignalBlock signal_block = 13;

    SceneSnapshot snapshot = 20;
    SceneDelta delta = 21;
    Blob blob = 22;
    SensorFrame sensor_frame = 23;

    Command command = 30;
    CommandResult command_result = 31;
    ProfileState profile_state = 32;
  }
}
```

### 4.1 Additions of 2026-10-14 (version 0.1, backwards compatible)

Added while building the viewer's screens, as new messages and new fields only; `buf breaking`
passes against the previous files. The `.proto` files in `proto/` are the reference; the
draft above predates them.

| Addition | Where | Used by |
|---|---|---|
| `Event`, `EventList` | `session.proto`, `Envelope` 40 and 41 | timeline event lanes (Scene, Replay), Events list |
| `ServerStats` with per-module timings | `session.proto`, `Envelope` 42 | Performance workspace |
| `RecordingInfo` with channels | `session.proto`, `Envelope` 43 | Replay workspace: Recording and Channels |
| `Wiring` (nodes, wires with source and sink units) in `ProfileValidation` | `command.proto` | Domains workspace: wiring graph, unit checks, schedule |
| `NetLimit` in `ElectricalNet` | `scene.proto` | Electrical workspace: limits |

Rules that go with them:

- An `EventList` replaces what the viewer holds; an `Event` appends. A recording sends its
  whole list with `complete = true`.
- `ServerStats.estimated` says whether module costs are measured or estimated. The mock server
  sets it: it replays, so its module costs are the profile's estimates.
- `Wiring` is sent whenever the profile text could be parsed, also when a unit check fails:
  a wire with `unit_ok = false` is what the user has to see (invariant 2).
- `QuantityKind` gained `LinearAcceleration`, appended to the list (architecture §2).

### 4.2 Additions of 2026-10-14, second set (version 0.1, backwards compatible)

For the Environment, ROS 2 and World workspaces. Again new messages and new fields only.

| Addition | Where | Used by |
|---|---|---|
| `EnvironmentState` with the fields that can be sliced | `session.proto`, `Envelope` 44 | Environment workspace |
| `FieldSlice`, `SubscribeField` | `session.proto`, `Envelope` 45; `command.proto` 53 | field overlays and the probe |
| `WorldState`, `SetWorldEnabled` | `session.proto`, `Envelope` 46; `command.proto` 54 | World workspace |
| `SetCameraInterest` | `command.proto` 55 (v0.7) | the viewer's camera as a hint for a streamed world's tile selection, at most twice a second; ignored by deterministic runs |
| `RosGraph`: nodes with their topics, `Control`, distro, domain, a `note`; `Topic`: bandwidth, origin, status | `session.proto` | ROS 2 workspace |

Rules that go with them:

- `FieldSlice`, `RosGraph` and `WorldState` are sent only to viewers that asked
  (`SubscribeField`, `SetRosGraphEnabled`, `SetWorldEnabled`); a viewer asks while a panel that
  shows them is open.
- World positions in `WorldState` are `double` in the world's ENU frame (invariant 6). Tiles
  are parallel arrays and are sent again only when `tiles_generation` changes.
- `EnvironmentState.synthetic`, `WorldState.synthetic` and `RosGraph.note` say when a server
  describes something it does not model or run. The mock server sets all three: it has no
  environment model, no tile streamer and no ROS 2 bridge.
- An EMI source's frequency is a parameter of the site or the device, never a default.

### 4.3 Addition of 2026-10-25 (version 0.1, backwards compatible)

Pre-approved by the owner on 2026-10-24 for autonomous work; listed for review in
`docs/plan/progress.md`.

| Addition | Where | Used by |
|---|---|---|
| `Collider` (prim, mesh, pose in the prim's frame), `SceneSnapshot.colliders` 11 | `scene.proto` | viewport Collision shading |

- Each collider's mesh is its shape at its size, appended to `SceneSnapshot.meshes` after the
  visual meshes, so `Prim.mesh` indices are unchanged. Planes are not sent.
- A collider moves with its prim, `0xFFFFFFFF` for fixed scenery. A viewer that does not know
  the field ignores it; the mock server sends none, and the menu entry is then disabled.

## 5. Session flow

```mermaid
sequenceDiagram
  participant V as viewer (client worker thread)
  participant S as server / mock server
  V->>S: ClientHello {v1.0, wants_shm}
  S->>V: ServerHello {v1.0, kind, capabilities, shm fds}
  S->>V: ProfileState
  S->>V: SignalCatalog {generation 1}
  V->>S: Command RequestSnapshot
  S->>V: SceneSnapshot {sequence 100} (+ Blob chunks or shm refs)
  loop running
    S->>V: SceneDelta {sequence 101.., base = previous}
    S->>V: SignalBlock (subscribed signals)
    S->>V: ServerStatus (10 Hz)
  end
  V->>S: Subscribe {signals for a new scope}
  V->>S: Command ApplyProfile {yaml}
  S->>V: CommandResult {ok}
  S->>V: ProfileState, SignalCatalog {generation 2}
  Note over V: subscriptions from generation 1 are void; the viewer re-subscribes by name
```

Clarifications fixed by the first implementation (the mock server):

- Transforms in `SceneSnapshot` and `SceneDelta` are absolute in the region-local frame, not
  relative to the parent prim.
- A `SceneSnapshot` also marks a discontinuity in time. After a seek, a reset, or a looping
  replay restarting, the server sends a snapshot to every client, and signal sample indices
  restart from the new time after it. Between two snapshots, runs of a signal are contiguous.
- A `MODE_MINMAX` run has `2 x components` values per sample: all minima, then all maxima. A
  bucket is sent once, in the block in which its last sample falls.

Resynchronisation rules:

- `SceneDelta.base_sequence` not equal to the client's sequence → the client drops deltas and
  sends `RequestSnapshot`.
- A congested server skips deltas without breaking the sequence (`dropped_deltas` counts them):
  its deltas carry the absolute pose of every moving prim and nothing else, so the next delta
  replaces a skipped one. A server whose deltas carry added or removed prims must instead
  break the sequence when it skips one.
- `SignalBlock.catalog_generation` older than the client's → the block is discarded.
- `SignalRun.first_index` not contiguous with the previous run → the scope draws a gap marker
  and the dropped-sample counter in the status bar increases. This is how the "no dropped
  samples" budget is measured.

## 6. Shared-memory fast path

Used only when both ends are on the same machine and connected by the Unix socket.

- The server creates regions with `memfd_create` and seals their size. File descriptors are sent
  with `ServerHello` as `SCM_RIGHTS` ancillary data, in the order of `shm_regions`.
- Regions and purposes in v0.1: `scene` (mesh and texture payloads, 256 MiB default) and
  `stream` (sensor frames, large signal blocks; 256 MiB default). Sizes are server options.
- A region is an arena of slots. Each slot starts with a 32-byte header
  `{u64 generation, u64 size, u64 xxh3, u64 reserved}`. The server writes the payload, then the
  header, then publishes a `ShmRef`. The client maps regions read-only, copies or uploads the
  payload, then re-reads `generation`; a changed generation means the slot was reused during the
  read and the payload is discarded. This is a seqlock; no cross-process mutex is used.
- Lifetime: `scene` slots live until the next `SceneSnapshot`. `stream` slots are a ring; the
  server never reuses a slot sooner than 500 ms after publishing it. (The first implementation,
  `lagrange::proto::ShmArena`, is a plain ring without the 500 ms rule; a reader that loses the
  race gets `Stale` and asks again. The rule is added with the server's stream region.)
- The descriptor sent to the viewer is opened read-only, so the viewer cannot map the region
  writable, and the reader refuses any descriptor whose size is not sealed.
- The fast path is an optimisation only. With `wants_shm = false` everything travels in-band and
  all tests must pass in both modes.
- Later, for a server-side Vulkan device shared with the viewer on one GPU, exporting images by
  `VK_KHR_external_memory_fd` is possible. It is out of scope until an RFC.

## 7. Versioning and compatibility

- Package `lagrange.viewer.v1`. A breaking change creates `v2` in a new directory; both may be
  served during a deprecation window.
- Handshake: the server refuses a different `major` with `ServerHello.refusal` set to a sentence
  that names both versions. A different `minor` is accepted; unknown fields are ignored (proto3
  behaviour), and new messages are added as new `oneof` cases that old peers skip.
- Until v1.0 of the product the protocol major is `0` and may break between minor releases. The
  viewer and server from the same release are always compatible. From v1.0 the `v1` package is
  frozen under `buf breaking`.

`buf breaking` policy:

| Phase | Rule set | Against | Effect |
|---|---|---|---|
| before v1.0 | `WIRE_JSON` | `main` | Breaking changes are allowed only in PRs labelled `proto-break`, which need an RFC or a PR note and two approvals (CLAUDE.md §9). The job fails without the label |
| from v1.0 | `FILE` | latest release tag | No breaking changes in `v1`; new major only through an RFC |

Lint: `buf lint` with the `STANDARD` rule set, except `RPC_*` rules (there are no services) and
with `ENUM_ZERO_VALUE_SUFFIX` set to `_UNSPECIFIED`. Nested enum values use a prefix of the enum
name, as in the drafts above; TODO(verify) the drafts against `buf lint` when the files are
created, since they were not linted in this planning task.

## 8. Conformance tests

- `tests/protocol/golden/`: binary frames recorded from the mock server for each message. The
  C++ decoder and the Python encoder must both round-trip them.
- A Python reference client (`python/lagrange/viewer_client.py`) connects to any server and checks
  the handshake, snapshot/delta sequencing, catalog generations and gap-free signal runs. The
  same script runs against the mock server in v0.1 and the real server in v0.2. This is what
  keeps the two servers interchangeable.
- Fuzzing: the C++ frame reader and `Envelope` parser get a libFuzzer target; malformed input
  must never crash the viewer (it receives data from the network).
