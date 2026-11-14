# RFC 0016: A robot builder in the viewer

| | |
|---|---|
| Status | Accepted (2026-11-14): the owner accepted every proposed answer below |
| Created | 2026-11-14 |
| Affects | `viewer/` (a new library `viewer/builder/`, panels, one workspace entry in `Chrome.cpp`, new flags), `docs/design/ui/12-builder/` (new), `modules/sensors/` (a catalog data file and its test, no code change), `docs/design/viewer.md`, `docs/design/dependencies.md`; later phases: `server/src/Server.cpp` (§7.2) and `proto/` (§11, only with the owner's approval) |
| Not affected (MVP) | `proto/`, `schemas/`, `core/plugin_abi/`, the server, every simulation module |
| Builds on | RFC 0003 (protocol), RFC 0006 (viewer stack), RFC 0007 (the 1.x freeze), RFC 0008 (custom sensors and kinds) |

## Summary

Build a robot inside `lagrange_viewer`, in a workspace of its own, **Builder**, with three modes:

- **Build**: add links (box, cylinder, sphere; meshes later), join them with fixed, revolute,
  continuous or prismatic joints, set axes with a gizmo and limits, damping and friction in a
  form, pose the robot with joint sliders (forward kinematics only), and add sensors from a
  catalog of the 3 built-in URDF sensor types and the 37 built-in kinds, each with a form made
  from its parameter list (units, defaults, ranges). Mass and inertia come from shape and
  density, with overrides. Nothing simulates: no stepping, no sensor rendering, no realistic
  pass; only the edited robot is drawn, Lit. It works without any server.
- **Test**: one click writes the robot as URDF and runs it in a simulator, in an empty world or
  a chosen one; the user drives it; **Back** stops the test and returns to Build with every edit
  kept.
- **Save**: URDF (the primary format) on the viewer's machine, plus an overlay YAML when the
  robot needs one (later phases), and USD through the existing `lagrange_import`.

The MVP needs **no protocol change, no server change and no schema change**: Test runs the
robot in a test server of the viewer's own (§7.1), as the Launcher already starts
`lagrange_server`. Testing inside an already connected run needs a small server change and one
additive protocol message (§7.2, §11): proposed as a later phase, for the owner to approve.

## Motivation

Today a robot is written by hand in URDF (or xacro, SDFormat, MJCF), started with `lagrange`,
and fixed from the loader's notes and `lagrange check`. The viewer shows a robot and edits a
few numbers (`SetAttribute`, E6) but cannot make one. For the robots Lagrange's users start
with (a base with wheels and a few sensors, a pan-tilt, a small arm) a form and a 3D view are
faster than XML, and the sensor catalog has grown to 37 built-in kinds whose parameters live in
`docs/design/domains/sensors.md` §9: a form that knows them with units and defaults saves a trip
through the docs for each one.

The owner asked for it to be a separate mode "so it doesn't weigh the simulation down".

## Non-goals

- Not a CAD tool: no mesh modelling, sketching or booleans. Meshes are imported, not made.
- Not a xacro editor: macros are not authored or kept (§10).
- Not an editor of a running simulation: changing a live robot stays `SetAttribute` (E6).
- USD is not the authoring format of the builder in the MVP (URDF is, by the owner's choice);
  USD is an export.
- No new physics, sensors or schemas.

## What exists today (checked on `origin/dev` 2bb5b20)

| Need | Today | Consequence |
|---|---|---|
| Spawn from text | `ImportRobot.content` (`DataRef`, field 3) exists in `proto/lagrange/viewer/v1/command.proto`, but `Server.cpp` refuses an `ImportRobot` without `path` ("name the robot file by its path on the server's machine") and refuses `overlay_path` and `xacro_args` ("not taken when importing yet") | `content` works only after a **server** change (no proto change): `writeResource`, which `/spawn_entity` already uses for `resource_string`, writes the text to a file the spawn reads |
| Name of an imported robot | `spawn()` names it from the file's stem when no name is given; for content that file is `lagrange-spawn-<pid>-<n>.urdf` | with the server change, name it from `<robot name>` (renamed `_2`, … if taken, as now) |
| Remove a robot from the viewer | **no Command removes a robot**. `Server::removeRobot` is reached only by ROS `/delete_entity` and `lagrange delete`; `Reset` keeps spawned robots (`restart()` recomposes with `spawned`) | Back in a shared run needs an additive `RemoveRobot` command (§11): needs the owner's approval and two approvals (CLAUDE.md §9) |
| Command a joint from the viewer | no Command; `Server::commandJoint` is reached by ROS `/joint_commands` | joint sliders cannot drive a tested robot without an additive `CommandJoint` (§7.3, §11) |
| A spawned robot in a run that has one | "the first robot in a run without one is the robot ROS drives, later ones passive" | testing inside a run with a robot gives a passive robot: another reason for a test server of its own |
| The viewer starting a server | `viewer/app/src/App.cpp` finds `lib/lagrange_server/lagrange_server` and `posix_spawn`s it (Launcher, Start local simulator) | the test server reuses that path |
| Test server start-up | `lagrange_server --robot assets/robots/amr_diffdrive/amr_diffdrive.urdf --world none --joint-control --listen unix:…`: 0.75 s from exec to the socket listening, with ROS, on the development machine (one measurement, 2026-11-14) | a fresh server per Test is cheap enough |
| What the viewer links | `lagrange_proto`, `lagrange_gpu`, `lagrange_gpu_raster`, `imgui_vendor`, SDL3. No `io/` library | the builder brings its own small URDF reader and writer (§6) |
| Where URDF is read | `server/src/Robot.cpp` (`loadUrdf`, `addUrdf`: urdfdom plus tinyxml2 for the `<gazebo>`, `<ros2_control>` and `<lagrange>` blocks), inside `lagrange_server`, which links every module. `io/importers` reads SDFormat and MJCF into `World` tables; `io/stage` writes USD (OpenUSD). Nothing writes URDF | reuse is not possible without linking the server (invariant 14) |
| Sensors in URDF | `<gazebo reference="L"><sensor name= type=>`. Built-in types: `imu`; `lidar`, `gpu_lidar`, `ray`, `gpu_ray`; `camera`, `depth`, `depth_camera`, `rgbd_camera`, `thermal`, `thermal_camera`. Any other type is a custom sensor; 37 kinds are built in (`kKinds` in `modules/sensors/src/kinds/Kinds.cpp`). GNSS, magnetometer, barometer and radar are not read from URDF: they come from an overlay (`create_child` with `GnssAPI_1` …, as in `quad_x500.overlay.yaml`), SDFormat or USD | the catalog has 3 types + 37 kinds in URDF, and 4 more through the overlay (phase 3) |
| Kinds' parameters | each kind's section of `sensors.md` §9 has a Parameter / Default / Meaning table; the code reads them with `kindSetting` / `kindText` (275 calls in `src/kinds/`), defaults as named constants per file | the source for the catalog (§5) |
| Joint dynamics in URDF | `<dynamics damping friction>` and `<mimic>` reach `World::Joints` | the joint form has them |
| A static load check | `lagrange check ROBOT` (`lagrange_check`, `server/src/RobotCheck.cpp`): joint loads against effort limits, exit 1 when the start pose cannot be held | offered from Save and Test when installed on the viewer's machine |

## Design

### 1. Three modes and what each costs

| | Build | Test | Save |
|---|---|---|---|
| Simulation | none | a test server of its own (§7.1) | none |
| Viewer draws | the builder view only: the document's primitives on the Lit pipeline | the normal Viewport of the test run, any shading the tier allows | — |
| Protocol traffic | none it asks for: entering Builder unsubscribes sensors, fields, world state and the ROS graph (restored on leaving); a connected run is not touched | the test server's normal stream | none |
| Needs a server | no | `lagrange_server` installed on the viewer's machine | no (USD export: `lagrange_import` on the machine) |
| Undoable | every document edit | nothing (the document is read-only while testing) | — |

Why Build is light: the builder view draws a few hundred triangles per primitive with the Lit
pipeline that exists for Tier 0; there is no scene from a server to draw, no camera or lidar
rendering, no realistic pass, no fields; hidden ImGui windows of other workspaces submit
nothing. A connected run keeps running unless the user asks for "Pause the run while building"
(View menu): the run may be someone else's (open question 7).

### 2. Where it lives (invariant 14)

```
viewer/builder/          library lagrange_viewer_builder: no ImGui, no Vulkan, gtest-tested
  Document.hpp/.cpp        the robot being built (§3), its validation (§8)
  MassProperties.cpp       mass, COM and inertia from shapes (§4)
  Urdf.cpp                 URDF writer and reader (§6)
  SensorCatalog.cpp        loads the catalog files (§5)
  History.cpp              undo and redo (§9)
  TestServer.cpp           starts and stops the test server (§7.1)
viewer/panels/src/       BuilderTree.cpp, BuilderProperties.cpp, BuilderJoints.cpp,
                         SensorCatalogPanel.cpp, BuilderAdd.cpp, BuilderView.cpp
viewer/render/           a builder draw list: generated box, cylinder, sphere meshes on the Lit pipeline
viewer/panels/src/Chrome.cpp   one workspace entry, the builder's mode controls and strip
```

The builder links no simulation module, not `lagrange_server`, not `lagrange_core`. New
dependencies of the viewer: **tinyxml2** (URDF; zlib licence, already used by
`lagrange_server` and `lagrange_importers`, in `dependencies.md`) and **yaml-cpp** (the catalog,
later the overlay; already in `dependencies.md`). Both are rosdep keys on Ubuntu 26.04.

### 3. The document

```cpp
namespace lagrange::viewer::builder {
struct Pose { std::array<double, 3> xyz{}; std::array<double, 3> rpy{}; };  // m, rad (URDF's)
struct Shape {
    enum class Kind { Box, Cylinder, Sphere, Mesh } kind = Kind::Box;
    std::array<double, 3> size{};   // box: x y z; cylinder: radius, length (along z); sphere: radius; m
    std::string mesh;               // Mesh: the file as written (package://, relative or absolute)
    std::array<double, 3> scale{1, 1, 1};
    Pose origin;                    // in the link frame
    bool visual = true, collision = true, solid = true;  // solid: counts for mass (§4)
    std::array<float, 4> rgba{};
};
struct Inertial {
    double density = 0;                         // kg/m³, from a material preset or typed
    std::optional<double> mass;                 // kg, override
    std::optional<std::array<double, 6>> inertia;  // ixx ixy ixz iyy iyz izz, kg·m², about the COM
    std::optional<std::array<double, 3>> com;      // m, override
};
struct Link { std::string name; std::vector<Shape> shapes; Inertial inertial; std::string keptXml; };
struct Joint {
    enum class Type { Fixed, Revolute, Continuous, Prismatic } type = Type::Fixed;
    std::string name, parent, child;
    Pose origin; std::array<double, 3> axis{1, 0, 0};
    std::optional<double> lower, upper, effort, velocity;  // rad or m, N·m or N, rad/s or m/s
    double damping = 0, friction = 0;
    std::optional<std::tuple<std::string, double, double>> mimic;  // joint, multiplier, offset
    std::string keptXml;
};
struct Sensor {
    std::string name, link, type;   // type: "imu", "lidar", "camera", ... or a kind
    Pose origin; double updateRate = 10;  // Hz (the URDF reader's default)
    std::optional<double> latency, jitter; // s
    std::map<std::string, std::string, std::less<>> params;  // by the catalog's path, text as typed
};
struct Document {
    std::string robot; std::vector<Link> links; std::vector<Joint> joints;
    std::vector<Sensor> sensors; std::string keptXml;  // robot-level elements kept as written
};
}  // namespace lagrange::viewer::builder
```

The tree is derived from the joints (one parent per link, one root, no loops). Joint preview
positions are view state, not document state. Names follow URDF; the server's own name check
(`validName`: letters, digits, `_`, not first a digit) is applied too, so a robot that saves also
spawns.

### 4. Mass and inertia without the server

Per solid shape of density ρ, in the shape's frame (closed forms, uniform solid):

| Shape | Mass | Inertia about its centre |
|---|---|---|
| box a × b × c | ρabc | Ixx = m(b² + c²)/12, Iyy = m(a² + c²)/12, Izz = m(a² + b²)/12 |
| cylinder r, h (axis z) | ρπr²h | Ixx = Iyy = m(3r² + h²)/12, Izz = mr²/2 |
| sphere r | ρ·4πr³/3 | I = 2mr²/5 |
| closed triangle mesh (phase 2) | volume integrals over the surface (Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties", J. Graphics Tools 1(2), 1996) | same method; a mesh that is not closed is a Problem, with the mass taken from its convex hull |

A link's shapes combine by rotating each tensor into the link frame (R I Rᵀ) and moving it to
the common COM (parallel axis theorem). The written `<inertial>` has its origin at the COM. A
mass override scales the computed tensor by m_override / m_computed (the shape's distribution
kept); an inertia override replaces it. The result is checked as the engines need it: positive
definite and the triangle inequality on the principal moments (Ixx + Iyy ≥ Izz and so on).

Density presets carry a source each (aluminium, steel, PLA, ABS, nylon, rubber, wood):
TODO(verify) the values and their sources when the presets are written; the reference picture
uses 2700, 1240 and 1100 kg/m³ for illustration only.

### 5. The sensor catalog: a small machine-readable file

Decision: one YAML file, **`modules/sensors/catalog/sensor_catalog.yaml`**, owned by
`lagrange_sensors` next to the kinds it describes, installed to
`share/lagrange_sensors/catalog/`. The viewer build-depends on `lagrange_sensors` for this
**data file only** (no library is linked, invariant 14) and installs a copy in its own share,
so a viewer-only install still has it. Third-party plugins may install their own catalog files
under the ament resource `lagrange_sensor_catalog` (the sample `tof`, `ultrasonic`,
`thermistor`, `ir_thermometer` plugins get one each); kinds without any entry get a free
key/value form.

```yaml
schema_version: 1
types:                               # built-in types the URDF reader takes (server/src/Robot.cpp)
  lidar:
    urdf_types: [lidar, gpu_lidar, ray, gpu_ray]
    summary: 2D or 3D scans, channel tables, patterns
    params:
      - {path: scan/horizontal/samples, type: int, default: 360}
      - {path: scan/horizontal/min_angle, type: number, unit: rad, default: -3.14159265358979}
      - {path: range/max, type: number, unit: m, default: 30}
      # ...
kinds:
  rangefinder:
    group: range_and_contact
    summary: one laser beam along +x · TFmini-S
    doc: docs/design/domains/sensors.md#rangefinder
    params:
      - {path: range/min, type: number, unit: m, default: 0.1, min: 0}
      - {path: range/max, type: number, unit: m, default: 12}
      - {path: noise/stddev, type: number, unit: m, default: 0.03, min: 0, meaning: "1σ, the least"}
      - {path: noise/fraction, type: number, unit: "1", default: 0.005, min: 0, meaning: "1σ of the range"}
      - {path: resolution, type: number, unit: m, default: 0.01}
    signals: [{name: range, unit: m}, {name: valid, type: bool}]
    ros: [{topic: range, type: sensor_msgs/msg/Range}]
```

A parameter's `path` is the kind's own (`<range><max>` is `range/max`), so the writer turns the
form back into the same XML the server reads. Values stay SI in the file; angles may be shown in
degrees.

Kept honest by two checks:
1. `test_sensor_catalog` (gtest, `lagrange_sensors`): the kinds in the file are exactly
   `builtinKinds()`; for every kind, a sensor with **every catalog default written explicitly**
   gives bit-identical signals over 1 s of CPU run to one with **no parameters**: a wrong default
   in the catalog fails here.
2. `tools/ci/check_sensor_catalog.py`: every parameter and default in the file matches the
   kind's table in `sensors.md` §9, both ways.

Alternatives: parsing `sensors.md` at run time (Markdown is not data and the viewer would ship
the docs); a table inside the viewer (drifts, and nothing checks it against the code); asking
the server (a protocol addition, and no catalog without a server); linking `lagrange_sensors`
(invariant 14). Rejected.

### 6. URDF writer and reader in the viewer

The builder writes its document with tinyxml2: `<robot>`, links (`<visual>`, `<collision>`,
`<inertial>`), joints (`<origin>`, `<axis>`, `<limit>`, `<dynamics>`, `<mimic>`), and each sensor
in a `<gazebo reference="LINK"><sensor name= type=>` block with `<pose>`, `<update_rate>`,
`<latency>`, `<jitter>` and its parameters as nested elements, which is the form the server reads
(`server/src/Robot.cpp`). Output is deterministic (document order, numbers in shortest
round-trip form, `std::to_chars`), so a saved file diffs cleanly. Mesh paths: relative to the
saved file on Save, absolute in the test copy.

The reader opens a URDF into the builder: what the document models is parsed into it; every
element it does not model is kept verbatim (`keptXml`, per link, joint and robot) and written
back in place, shown read-only under "Kept as written".

Why not reuse server code: URDF reading is in `lagrange_server`, which links every module;
`io/` has no URDF writer and its readers produce `World` tables through `lagrange_core`.
Two readers can disagree, so the server stays the judge: Test loads the file with the real
loader (§7), and the round-trip tests (Test plan 3) load every builder-written file with
`lagrange_server`.

### 7. Test

#### 7.1 A test server of its own (MVP, no protocol or server change)

1. Validate the document (§8); refuse Test on an error.
2. Write `$XDG_RUNTIME_DIR/lagrange/builder/<robot>/<robot>.urdf` (meshes by absolute path).
3. `posix_spawn` `lagrange_server --robot <file> --world none|<world> --joint-control --listen
   unix:$XDG_RUNTIME_DIR/lagrange/builder/<robot>.sock`, found and started as the Launcher's
   Start local simulator does (`App.cpp`); ROS on, in the user's `ROS_DOMAIN_ID`.
4. The viewer's client connects to it, remembering the endpoint it was on.
5. A refused robot: the server exits with its message on stderr, captured into Problems; the
   loader's notes arrive as log lines into Console and, when they name a link, joint or sensor,
   are linked to it in Problems.
6. **Back**: SIGTERM (then SIGKILL after 3 s, the project's ROS shared-memory hygiene), reconnect
   to the previous endpoint if there was one, return to Build. The document was never changed by
   Test.

Why a separate process: the user's run is untouched; the robot is the first and only robot, so
ROS drives it and joint control applies; files, overlays and meshes work as on the command line;
Back is "stop". World choices: empty ground, a sample world (the Launcher's list), or a world file
on this machine.

#### 7.2 Into the connected run (later, needs approval)

`ImportRobot { format: FORMAT_URDF, content: inline }` to the connected server. Needs: the
server change of the table above (accept `content` through `writeResource`, name from
`<robot name>`); for Back, an additive `RemoveRobot` command; for overlays,
`ImportRobot.overlay_content`. The robot is passive when the run already has one.

#### 7.3 Driving it

Through ROS in the MVP: `/cmd_vel` when the server drives the robot (diff drive, swerve, car,
multicopter, as it recognises them) and `/joint_commands` for every joint (joint control). The
Test layout's Drive panel lists the topics with ready-to-copy commands. Joint sliders driving the
tested robot need an additive `CommandJoint` command onto `Server::commandJoint` (§11).

### 8. Validation

1. **Builder, live** (Problems): names valid and unique; one root, no loops, every joint's
   parent and child exist; revolute and prismatic joints have `lower < upper`, `effort > 0`,
   `velocity > 0` (URDF requires `<limit>` for them; empty fields are an error, never a made-up
   default); a non-zero axis; positive mass and a valid inertia (§4); a mimic's leader exists;
   sensor names unique, parameters within the catalog's min and max and of the right type;
   meshes found. Hints for what is legal but probably unmeant (a sphere caster on a fixed joint
   slides).
2. **The server's loader** on Test: authoritative (§7.1 step 5).
3. **`lagrange check`** from Save and Test (Check loads…): `lagrange_check FILE` run on the
   viewer's machine when installed, its lines into Problems.

### 9. Undo and redo

Snapshots of the whole document per committed edit (a document is tens of kB; 256 steps is a few
MB at most), with a label per step for the History tab. A slider or field edit commits on release
or Enter; edits to the same field within one second merge. Joint previews, selection and camera
are not steps. Ctrl Z, Ctrl Shift Z, Ctrl Y. Snapshots were chosen over a command pattern:
less code to get wrong, and every edit, including re-parenting, is undoable the same way.

### 10. Files

**Save** writes the URDF atomically (temporary file, rename); never over a `.xacro`; asks before
overwriting a file the builder did not open. **Export USD…** runs `lagrange_import ROBOT.urdf
ROBOT.usda` on the machine. An autosave every 30 s goes to
`$XDG_STATE_HOME/lagrange/builder/<robot>.autosave.urdf`, offered back after a crash.

Opening an existing file:

| | |
|---|---|
| Edited | links, visuals, collisions, inertials, joints (with dynamics, limits, mimic), sensors of the built-in types and known kinds |
| Kept as written, read-only | other `<gazebo>` children (`mu1`, plugins, `sensor_material`, `visual_material`, `thermal_material`, `heat_source`), `<transmission>`, `<ros2_control>`, the `<lagrange>` block, `<material>` definitions, comments |
| Lost | xacro: a `.xacro` is expanded by `xacro` when installed and opened as a new, unsaved URDF (macros, properties and arguments are gone; the `.xacro` is never written); whitespace and attribute order |
| Not opened (MVP) | SDFormat, MJCF, USD: import them to USD with `lagrange_import`, or open their URDF |

An `<inertial>` read from a file is kept as an override until the user asks to compute it.

### 11. Protocol

None for the MVP. Possible additions, each needing the owner's approval and two approvals
(CLAUDE.md §9), all additive under RFC 0007:

| Addition | For | Phase |
|---|---|---|
| `Command.remove_robot = 33`, `message RemoveRobot { string name = 1; }` → `Server::removeRobot` | Back after testing inside the connected run | 1b |
| `ImportRobot.overlay_content = 6` (`DataRef`) | an overlay with a robot sent as content | 1b |
| `Command.command_joint = 34`, `message CommandJoint { string joint = 1; string mode = 2; double value = 3; }` → `Server::commandJoint` | joint sliders driving a tested robot | 2 |

The server change that makes `ImportRobot.content` work is not a protocol change (the field
exists since 1.0).

### 12. Tiers

Build is the same on Tier 0, 1 and 2: the Lit raster pipeline runs on the software Vulkan device
(llvmpipe) the viewer's headless tests use. Test runs the server on the machine's tier (CPU lidar
on Tier 0). No feature of the builder needs a GPU.

### 13. The UI

`docs/design/ui/12-builder/` (draft, made from the design system's stylesheet for the owner's
review): an 11th workspace **Builder** after Performance (Ctrl B; Ctrl 1…0 are taken), the mode
controls in the workspace bar, a Robot strip in place of the domain strip, and the panels Robot,
Add, Builder view, Joints, Sensor catalog, Problems, Properties and History. New widgets proposed
there (`JointSlider`, `ShapeTile`) and new icons go into `00-design-system` when accepted. A
fourth Launcher tile **Build a robot** would change the owner's `01-launcher` design: only when
accepted.

## Phases

| Phase | Contents | Changes outside `viewer/` |
|---|---|---|
| 1 MVP | primitives, joints, joint sliders, built-in types and kinds from the catalog, mass and inertia, validation, undo, open and save URDF, Test in a test server of its own, Check loads | the catalog file and its two checks in `modules/sensors`, `tools/ci` |
| 1b | Test into the connected run | `Server.cpp` (`ImportRobot.content`, name from `<robot name>`); proto `RemoveRobot`, `overlay_content` (approval) |
| 2 | meshes (draw, mass from the mesh, collision), `lagrange bake` on concave meshes from the builder, joint sliders driving a test | a mesh reader for the viewer (open question 8); proto `CommandJoint` (approval) |
| 3 | motors, battery, bus and the overlay-only sensors (GNSS, magnetometer, barometer, radar) written to the overlay YAML; forms from `formats.md` §9's overlay keys | none expected |
| 4 | USD round trip (open `.usda` made by `lagrange_import`), SDFormat | to be decided |

## Test plan

1. `viewer/builder` gtests: mass properties against the closed forms (each shape, a composite
   through the parallel axis theorem, a rotated shape, a mass override scaling the tensor, the
   positive-definite and triangle checks); validation (one case per rule of §8); undo and redo
   (merge, depth, labels); the catalog loader (a malformed file is an error with its line).
2. URDF golden files: the writer's output for sample documents equals checked-in URDF byte for
   byte.
3. Round trip and agreement (pytest in `viewer/test`, needs `lagrange_server`): read → write is
   semantically identical (elements, attributes, numbers) for every URDF under `assets/robots`,
   kept blocks included; every builder-written golden file loads in `lagrange_server` with no
   notes, and `lagrange_import FILE.urdf OUT.usda` gives each body the document's mass, COM and
   inertia and each joint its type, axis and limits (read from the written `.usda`; the Python
   API's `bodies()` and `joints()` give names only); one sensor of every catalog kind with its
   defaults on a test robot loads and runs 1 s.
4. Catalog: `test_sensor_catalog` and `tools/ci/check_sensor_catalog.py` (§5).
5. Headless UI (`test_builder_input.py`, as `test_viewer_input.py`, with new flags
   `--workspace builder`, `--builder-open FILE`, `--builder-save FILE`): add a box, make its joint
   revolute with limits, add a rangefinder from the catalog, set `range/max`, undo and redo,
   Test (the test server starts, the robot is in the Test view), Back (the document unchanged),
   Save (the file equals a golden URDF); and the screenshot against `reference.png`.
6. Cost: headless Build mode on llvmpipe within the viewer's frame budget; connected to a server,
   no sensor, field or world messages arrive while Builder is in front.

## Alternatives considered

| Alternative | Why not |
|---|---|
| A separate application (`lagrange_builder`) | duplicates the chrome, renderer and theme; the owner asked for it inside the viewer |
| Author USD first | heavier for the robots meant here; the owner chose URDF as primary; USD stays an export |
| Builder state on the server, edited through the protocol | protocol growth, and no building without a server |
| Testing only inside the connected run | needs `RemoveRobot` for Back, gives a passive robot when the run has one, and changes a run that may not be the user's |
| Reading URDF with urdfdom in the viewer | drops the elements it does not know (`<gazebo>`, `<ros2_control>`), so they could not be kept |
| Writing xacro | macros cannot be produced from a flat document in a way people would keep |

## Open questions (answered: the owner accepted each proposed answer, 2026-11-14)

1. *Test's default target*: the viewer's own test server (§7.1) in the MVP, and testing inside the
   connected run only in phase 1b? Proposed: yes.
2. *Approve the additive `RemoveRobot` and `ImportRobot.overlay_content`* for phase 1b, and
   `CommandJoint` for phase 2? Proposed: yes, each in its own PR with the note §9 asks for.
3. *The server change for `ImportRobot.content`* (accept the text through `writeResource`, name
   the robot from `<robot name>`): proposed for phase 1b, server only.
4. *Catalog location*: owned by `lagrange_sensors`, a data-only build dependency of the viewer,
   copied into the viewer's share (§5)? Proposed: yes.
5. *yaml-cpp in the viewer* for the catalog (and later the overlay)? Proposed: yes.
6. *Workspace placement*: an 11th tab "Builder" after Performance with Ctrl B, plus a Launcher
   tile? Proposed: yes; the Launcher tile changes your 01 design, so only if you want it.
7. *A connected run while building*: left running (default), with "Pause the run while building"
   in the View menu? Proposed: yes; the builder never pauses a run on its own.
8. *Meshes (phase 2)*: link `lagrange_meshes` (`io/meshes`: `lagrange_core` and assimp; a library,
   not a simulation module), so the builder reads meshes exactly as the server does, or a small
   OBJ/STL reader in the viewer? Proposed: `lagrange_meshes`, for one reading of a mesh.
9. *Density presets*: which materials, from which sources (§4)? Proposed: aluminium 6061, steel,
   PLA, ABS, nylon, natural rubber, pine, each from a cited data sheet, TODO(verify) until then.
10. *Start pose for Test*: the joint preview positions as the test's start pose (the server holds
    joints at `World::Joints::initialPosition`)? Proposed: phase 2, if the URDF can carry them
    without a schema change; otherwise start at zero.
11. *Templates* (diff-drive base, 3-joint arm, pan-tilt head) shipped as builder documents?
    Proposed: yes, as plain URDF files in `assets/robots/templates/`, each loaded by the round-trip
    test.
12. *Which shapes count for mass*: proposed every shape marked solid, by default the collision
    shapes; a visual-only shape is massless.
