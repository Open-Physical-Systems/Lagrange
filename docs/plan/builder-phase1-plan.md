# Robot builder: phase 1 and 1b work plan (RFC 0016)

Status: plan, 2026-11-14. RFC 0016 is accepted with every proposed answer to its 12 open
questions. This file splits phase 1 (the MVP) and phase 1b into work packages for 3 to 5 agents
working at the same time, names the seams between them, and lists what the RFC leaves open. It
was written from `origin/dev` d5b400a by reading the code. Nothing was built or run.

Read with: `rfcs/0016-robot-builder.md` (§ numbers below are the RFC's),
`docs/design/ui/12-builder/SPEC.md` (binding UI), `.claude/rules/viewer-ui.md`.

## 1. Decisions this plan makes where the RFC is silent

These keep the packages apart. Each one is small. Change one only by a note in this file.

1. **The Builder view reuses `SceneRenderer`.** The document plus the joint preview becomes a
   `SceneFrame` (`viewer/client/include/lagrange/viewer/SceneStore.hpp:92-118`). Primitives
   become `MeshData`, collision shapes become `ColliderData`, and link poses from forward
   kinematics become `transforms`. The existing Lit pipeline then draws it, and
   `pickPrim(const SceneFrame&, const Ray&)` (`render/include/lagrange/viewer/Picking.hpp:14`)
   picks from it. No new pipeline and no new shader. Joint axes, frames, the COM marker,
   sensor rays and the selection outline are `OverlayLines` (`SceneRenderer.hpp:27-31`).
2. **A joint preview changes only `SceneFrame::transforms`.** A document edit makes a new
   `SceneStatics` with a higher `generation`, so the meshes upload again
   (`SceneRenderer.hpp:90`). Dragging a slider must never upload meshes again.
3. **Panels never start processes or touch the socket** (`Panel.hpp:78`). Builder panels write
   requests into a `BuilderSession` (as `LauncherState` does, `Panel.hpp:71-75`). After
   `drawPanels`, a `BuilderHost` in `viewer/app/` carries them out.
4. **The test server's socket** is `proto::defaultUnixSocketPath("builder-<viewer pid>")`
   (`proto/transport/src/Endpoint.cpp:117`). It is not `<robot>.sock`, because a long robot
   name would pass the `sun_path` limit (`proto/transport/src/Connection.cpp:37`). The URDF and
   the server log go to `$XDG_RUNTIME_DIR/lagrange/builder/<pid>/`, a folder the viewer makes
   itself. The listener only makes one missing folder level (`Connection.cpp:619-623`).
5. **Undo merges by ImGui time** (`ImGui::GetTime()`), not wall time. Headless runs use a fixed
   `DeltaTime` of 1/60 s (`App.cpp:1439`), so the "same field within 1 s" rule gives the same
   result on every run.
6. **The catalog file names a wrapper element per entry** (`element:`). The URDF reader reads
   `<ray>` before `<lidar>` (`server/src/Robot.cpp:1043-1046`) and reads `<camera>`
   (`:1072`). A kind's first element named like its type is dropped from the path
   (`Robot.cpp:779-781`). The writer always writes the wrapper: `<rangefinder>…` as in
   sensors.md's examples, `<lidar>` for lidars, `<camera>` and `<imu>`.
7. **What URDF has no place for is written as attributes the loader ignores** (WP-B; urdfdom and
   the `<gazebo>` reader skip unknown attributes, checked by `test_builder_roundtrip.py`: the
   goldens load with no notes). `<link lagrange_density="ρ">` (kg/m³);
   `<inertial lagrange_overrides="mass com inertia">` on a link with a density lists the user's
   overrides, and the parts not listed are computed again when the file is opened (a file without
   it, any other URDF, has all three as overrides, RFC §10); `lagrange_solid="true|false"` on a
   `<visual>` or `<collision>` only when it differs from the default. Colours are robot-level
   `<material>` definitions (a visual's local one is moved up). Where kept elements stood among
   links, joints, sensors, visuals and collisions is held by comment markers
   `<!--lagrange-builder:link-->` in `keptXml` (`keptForDisplay()` hides them), so a file read
   and written again keeps its order and, after one save, its bytes. Sensor attributes are
   parameters `<path>/@<name>`; an IMU's equal x, y, z read as the catalog's `{axis}`.

## 2. Work packages, phase 1

Estimates: S is up to a day, M is 2 to 3 days, L is 4 or more days, for one agent. Paths are
real paths, or new paths in folders that exist. The Document types are RFC §3, copied as they
are.

### WP0: interfaces and skeleton (S). Lands first, alone.

Goal: every header the other packages code against, plus build plumbing, with stubs that
compile. After WP0 no package edits another package's header without a note in its PR.

Files:
- New `viewer/builder/include/lagrange/viewer/builder/`: `Document.hpp` (RFC §3 verbatim),
  `MassProperties.hpp`, `Kinematics.hpp`, `Validation.hpp`, `History.hpp`, `Urdf.hpp`,
  `SensorCatalog.hpp`, `Materials.hpp`, `Primitives.hpp`, `SceneAdapter.hpp`,
  `TestServer.hpp`. New `viewer/builder/src/*.cpp` stubs, one per header.
- `viewer/panels/include/lagrange/viewer/BuilderSession.hpp` (new): the UI-side state, in §2.1.
- `viewer/panels/include/lagrange/viewer/Panel.hpp`: add `Workspace::Builder` after
  `Performance` (`:22-33`; fix the "All ten" comment at `:21`), add
  `BuilderSession* builder{nullptr};` to `PanelContext`, and declare
  `createBuilderPanels()`.
- `viewer/panels/src/Chrome.cpp`: an 11th `kWorkspaces` entry (`:38-62`, `std::array<…, 10>`
  becomes 11): label "Builder", `ImGuiKey_B`, "Ctrl+B", design "12-builder", and dockspace
  `nullptr` for now, so the tab shows "Builder is not built yet" (`:736-760`).
  `widgets::Icon::Builder` (the wrench path from `12-builder/reference.html`) goes in
  `viewer/app/widgets/include/lagrange/viewer/Icons.hpp` and `src/Icons.cpp`.
- `viewer/app/src/BuilderHost.hpp/.cpp` (new, empty bodies): `void BuilderHost::afterUi()`
  and `const SceneFrame* BuilderHost::frameToDraw(Camera&)`. Their call sites go into
  `App.cpp` once, here: after `drawPanels` at `:529`, and in `renderScene` (`:571`).
- `viewer/CMakeLists.txt`: a library `lagrange_viewer_builder` (no ImGui, no Vulkan). It links
  `tinyxml2::tinyxml2`, `yaml-cpp::yaml-cpp` and `lagrange_viewer_client` (for the
  `SceneFrame` types only). Add a gtest `test_viewer_builder` (smoke test) and a pytest
  placeholder `test_builder_input`.
- `viewer/package.xml`: `<depend>tinyxml2</depend>`, `<depend>yaml-cpp</depend>`,
  `<build_depend>lagrange_sensors</build_depend>` (see risk R11), and
  `<test_depend>lagrange_assets</test_depend>` if it is missing.
- `docs/design/dependencies.md`: add `lagrange_viewer` to the users of tinyxml2 (`:56`) and
  yaml-cpp (`:48`).
- `viewer/test/builder/rover_mk1.urdf`: the sample document of `12-builder/README.md`, written
  by hand. It is the input of tests and of the screenshot.

#### 2.1 Interfaces fixed by WP0

```cpp
namespace lagrange::viewer::builder {
// MassProperties.hpp: RFC §4. Inertia about the COM in the link frame: ixx ixy ixz iyy iyz izz.
struct MassProperties { double mass{0}; std::array<double, 3> com{}; std::array<double, 6> inertia{}; };
[[nodiscard]] MassProperties shapeMass(const Shape& shape, double density);   // in the link frame
[[nodiscard]] std::optional<MassProperties> linkMass(const Link& link);       // overrides applied
enum class InertiaCheck { Ok, NotPositiveDefinite, TriangleInequality };
[[nodiscard]] InertiaCheck checkInertia(const std::array<double, 6>& inertia);
// Kinematics.hpp: forward kinematics only. Links in document order; root at the identity.
struct Frame { std::array<double, 3> position{}; std::array<double, 4> rotation{1, 0, 0, 0}; }; // w x y z
struct Tree { std::vector<int> parentJoint; int root{-1}; bool ok{false}; std::string error; };
[[nodiscard]] Tree buildTree(const Document& document);
[[nodiscard]] std::vector<Frame> linkFrames(const Document& document, const Tree& tree,
                                            const std::map<std::string, double, std::less<>>& preview);
[[nodiscard]] MassProperties robotMass(const Document& document);   // at zero preview, root frame
// Validation.hpp: RFC §8.1
enum class Level { Error, Warning, Hint };
struct Problem { Level level; std::string item; std::string message; };  // item: link/joint/sensor name
[[nodiscard]] std::vector<Problem> validate(const Document& document, const SensorCatalog* catalog,
                                            const std::filesystem::path& meshBase);
// History.hpp: RFC §9
class History {
public:
    explicit History(Document initial, std::size_t depth = 256);
    void commit(Document next, std::string label, std::string mergeKey, double nowS);
    bool undo(); bool redo(); bool goTo(std::size_t step);
    [[nodiscard]] const Document& current() const;
    [[nodiscard]] std::span<const std::string> labels() const;  // newest last
    void markSaved(); [[nodiscard]] int editsSinceSave() const;
};
// Urdf.hpp: RFC §6
struct WriteOptions { std::filesystem::path savedAt; bool absoluteMeshes{false}; };
[[nodiscard]] std::string writeUrdf(const Document& document, const WriteOptions& options = {});
struct ReadResult { std::optional<Document> document; std::string error; int errorLine{0};
                    std::vector<std::string> notes; };
[[nodiscard]] ReadResult readUrdf(std::string_view text);
[[nodiscard]] std::string saveAtomically(const std::filesystem::path& path, std::string_view text); // "" ok
// SensorCatalog.hpp: RFC §5
struct CatalogParam { std::string path; enum class Type { Number, Int, Bool, Text, Enum, List } type;
                      std::string unit; std::string defaultText; std::optional<double> min, max;
                      std::vector<std::string> options; std::string meaning; };
struct CatalogEntry { std::string name; bool builtinType{false}; std::vector<std::string> urdfTypes;
                      std::string element; std::string group, summary, doc; bool overlayOnly{false};
                      std::vector<CatalogParam> params; std::vector<std::string> signals, rosTopics; };
class SensorCatalog {
public:
    struct LoadError { std::string file; int line{0}; std::string message; };
    [[nodiscard]] static std::variant<SensorCatalog, LoadError> load(const std::vector<std::filesystem::path>& files);
    [[nodiscard]] const CatalogEntry* find(std::string_view typeOrKind) const;
    [[nodiscard]] std::span<const CatalogEntry> entries() const;
};
[[nodiscard]] std::vector<std::filesystem::path> catalogFiles();  // own share + ament resource
// Materials.hpp: open question 9
struct DensityPreset { std::string_view name; double density; std::string_view source; };  // kg/m³
[[nodiscard]] std::span<const DensityPreset> densityPresets();
// Primitives.hpp: the same tessellation as core/world/include/lagrange/world/Model.hpp:46-47
[[nodiscard]] MeshData boxMesh(const std::array<double, 3>& size);
[[nodiscard]] MeshData cylinderMesh(double radius, double length, int segments = 32);
[[nodiscard]] MeshData sphereMesh(double radius, int segments = 32, int rings = 16);
// SceneAdapter.hpp
struct BuiltScene { std::shared_ptr<const SceneStatics> statics; std::vector<std::string> primItem; };
[[nodiscard]] BuiltScene buildStatics(const Document& document, std::uint64_t generation);
[[nodiscard]] std::shared_ptr<const SceneFrame> poseScene(const BuiltScene& scene,
                                                          const std::vector<Frame>& links);
// TestServer.hpp: RFC §7.1, never blocks the UI thread
struct TestRequest { std::string program; std::filesystem::path urdf; std::string world;
                     std::string endpoint; std::filesystem::path log; };
class TestServer {
public:
    enum class State { Idle, Running, Stopping, Exited };
    [[nodiscard]] std::string start(const TestRequest& request);  // "" or why not
    State poll();               // waitpid(WNOHANG); SIGKILL 3 s after requestStop()
    void requestStop();         // SIGTERM
    [[nodiscard]] int exitStatus() const; [[nodiscard]] std::string logTail(std::size_t bytes) const;
};
}  // namespace lagrange::viewer::builder
```

`BuilderSession` (panels side): the `History`, the loaded `SensorCatalog`, the selection
(`kind` plus name), the preview map, `std::vector<Problem> problems` (builder plus loader plus
`lagrange check`, by origin), the mode (`Build`/`Starting`/`Testing`), the file path and the
autosave time. Out-requests, read by `BuilderHost::afterUi()`: `testRequested` (+ world),
`backRequested`, `saveRequested` (+ path), `openRequested` (+ path), `exportUsdRequested`,
`checkRequested`, `templateRequested`.

### WP-A: model (M). After WP0. Parallel with B, C, D, E.

Goal: mass, kinematics, validation, undo and the density presets. Pure C++, no UI.
Files: `viewer/builder/src/MassProperties.cpp`, `Kinematics.cpp`, `Validation.cpp`,
`History.cpp`, `Materials.cpp`; tests `viewer/builder/test/test_mass_properties.cpp`,
`test_kinematics.cpp`, `test_validation.cpp`, `test_history.cpp` (all in `test_viewer_builder`).
Tests (gtest names):
- `MassProperties.BoxCylinderSphereMatchClosedForms` and `.RotatedShapeRotatesTensor`.
- `MassProperties.CompositeUsesParallelAxis` and `.OverrideScalesTensor`.
- `MassProperties.ChecksPositiveDefiniteAndTriangle`.
- `MassProperties.RoverMk1TotalsTheReadme`: 10.99 kg, COM (0.007, 0.000, 0.003) m, from
  `12-builder/README.md`.
- `Kinematics.RevolutePrismaticContinuousMimic` and `.RefusesLoopsAndTwoRoots`.
- `Validation.<Rule>`: one test per rule of RFC §8.1. Include `EmptyLimitIsAnError`,
  `ServerNameRule` (the same rule as `validName`, `server/src/Server.cpp:4655-4661`),
  `MimicLeaderExists`, `ParamOutOfCatalogRange`, `MeshNotFound` and
  `LagrangeAssetMeshResolvesThroughAssetDir`.
- `History.MergeWithinOneSecond`, `.DepthDropsOldest`, `.RedoClearedByCommit`, `.GoTo`, and
  `.EditsSinceSave`.
Depends on: WP0. Validation reads `SensorCatalog` through its interface only. Its tests use a
small catalog in the test itself.

### WP-B: URDF reader and writer, goldens, templates (L). After WP0.

Goal: RFC §6 and §10, without the UI.
Files: `viewer/builder/src/Urdf.cpp`; `viewer/builder/test/test_urdf.cpp`; goldens
`viewer/test/builder/golden/*.urdf` (rover_mk1, one per joint type, sensors with kept blocks);
templates `assets/robots/templates/diff_drive_base.urdf`, `arm_3dof.urdf`, `pan_tilt.urdf`
(open question 11); `viewer/test/test_builder_roundtrip.py` (pytest, needs `lagrange_server`).
Rules: numbers through `std::to_chars` (shortest round trip). Output follows document order.
Unmodelled elements are kept in `keptXml` per robot, link and joint, in place. Sensors go in
`<gazebo reference=><sensor name type>` with `<pose>`, `<update_rate>`, `<latency>`,
`<jitter>` and the wrapper of decision 6. Mesh paths: relative to `savedAt`, or absolute for
the test copy. `lagrange-asset://` and `package://` stay as written. Never write over a
`.xacro`.
Tests:
- gtest `Urdf.WritesGoldenByteForByte`, `.ReadsWhatItWrites`, `.KeepsUnknownBlocksInPlace`
  (`<transmission>`, `<ros2_control>`, `<lagrange>`, `<gazebo>` `mu1` / `sensor_material`),
  `.InertialFromFileBecomesOverride`, `.MalformedXmlGivesLine`, `.KindWrapperAcceptedBothWays`
  (`<rangefinder><range>` and `<range>`), and `.SaveIsAtomic`.
- pytest: read then write is semantically the same for all 11 `assets/robots/**/*.urdf` and the
  3 templates. Every golden and template loads in `lagrange_server` with no notes. For every
  golden, `lagrange_import F.urdf OUT.usda` gives each body the document's mass, COM and
  inertia, and each joint its type, axis and limits (read from the `.usda` text).
Depends on: WP0 only. Sensor parameters are path/text maps, so no catalog is needed.

### WP-C: sensor catalog (L). After WP0. Touches no viewer UI.

Goal: RFC §5. The data file, its two checks, plugin catalogs, and the viewer's loader.
Files:
- `modules/sensors/catalog/sensor_catalog.yaml` (3 types plus the 37 kinds of
  `kKinds`, `modules/sensors/src/kinds/Kinds.cpp:69-107`; the 4 overlay-only sensors as
  `overlay_only: true` entries for the catalog's Overlay filter). Install rule in
  `modules/sensors/CMakeLists.txt` to `share/lagrange_sensors/catalog/`.
- `modules/sensors/test/test_sensor_catalog.cpp` (gtest `test_sensor_catalog`).
- `tools/ci/check_sensor_catalog.py`, plus a pytest wrapper that `test_sensor_catalog` or
  `tools/` already runs.
- Plugin catalogs: `core/plugin_abi/samples/catalog/{tof,ultrasonic,thermistor,ir_thermometer}.yaml`
  under the ament resource `lagrange_sensor_catalog` (check where the samples' `lagrange_plugins`
  index entry is installed and do the same).
- Viewer: `viewer/builder/src/SensorCatalog.cpp`; install a copy into
  `share/lagrange_viewer/catalog/` (R11); `viewer/builder/test/test_sensor_catalog_loader.cpp`.
Tests:
- `SensorCatalog.KindsAreExactlyBuiltinKinds` (against `builtinKinds()`).
- `SensorCatalog.ExplicitDefaultsChangeNothing/<kind>` (parameterised): bit-identical signals
  over 1 s, defaults written versus none. The test robot must give each kind something to read
  (R10).
- `check_sensor_catalog.py` both ways against sensors.md §9.
- Loader: `CatalogLoader.MalformedFileGivesFileAndLine`, `.PluginFileAddsKinds`, and
  `.DuplicateKindIsAnError`.
Depends on: WP0 for the loader's header. The YAML and checks can start at once.

### WP-D: Builder view and its rendering (M). After WP0.

Goal: SPEC "Builder view". The scene from the document (decisions 1 and 2), a builder camera,
tools Q/W/E/A/M, toggles, overlays, picking, frame-time overlay.
Files: `viewer/builder/src/Primitives.cpp`, `viewer/builder/src/SceneAdapter.cpp`,
`viewer/app/src/BuilderHost.cpp` (`frameToDraw` only, plus overlay lines),
`viewer/panels/src/BuilderView.cpp`; tests `viewer/builder/test/test_scene_adapter.cpp`.
Gizmos (Move, Rotate, Joint axis ring) are drawn with the ImGui draw list over the view image,
as the Viewport's overlays are. They write through `History::commit`.
Tests: `SceneAdapter.OneMeshPerShapeAndCollidersSeparate`, `.PreviewChangesTransformsOnly`
(statics pointer and generation unchanged), `.PickReturnsTheLink` (with `pickPrim`), and
`Primitives.SameVertexCountsAsCore` (32 segments, 16 rings).
Depends on: WP0. It uses `linkFrames` from WP-A, so stub it as the identity until A lands.

### WP-E: panels and chrome (L). After WP0. Split E1 and E2 if 5 agents are free.

E1 (M): `Chrome.cpp`. The Builder `buildLayout` branch (SPEC dock tree,
`lagrange-dock-builder-1`), the Robot strip in place of `drawDomainStrip` (`:875-876`), and the
mode controls in place of the run controls (`:445-470`). It sets the real dockspace name in
the entry. Panels `viewer/panels/src/BuilderTree.cpp` (Robot, Files) and `BuilderAdd.cpp` (Add,
Templates). New widgets in `viewer/app/widgets/src/Widgets.cpp`: `JointSlider`, `ShapeTile`,
`Vec3Edit` (the existing `Vec3Field` at `Widgets.hpp:128` is read-only). New icons in
`Icons.cpp` (SPEC "New in this screen").
E2 (M): `BuilderProperties.cpp` (Link/Joint/Sensor, History tab), `BuilderJoints.cpp` (Joints,
Poses), `SensorCatalogPanel.cpp`, and `BuilderProblems.cpp`.
Window IDs: `###BuilderProblems`, `###BuilderProperties`, `###BuilderHistory`, and so on. They
are never `###Problems` or `Console` (R5).
Tests: `viewer/test/test_builder_input.py` steps for its panels (§2.2). Panel logic that can be
tested without ImGui goes into `viewer/builder/` (for example `formatParam`, `parseParam` with
units and degrees) with gtests there.
Depends on: WP0. It uses A's `History` and `validate`, and C's `SensorCatalog`, through the
headers. Until they land it works on stubs.

### WP-F: Test, files, headless (M). After A and B; the end of the test needs E.

Goal: RFC §7.1, §10, SPEC states 4 to 6, and the cost rules.
Files: `viewer/builder/src/TestServer.cpp`; `viewer/app/src/BuilderHost.cpp` (`afterUi`);
`viewer/app/src/main.cpp` (`--workspace builder` already parses, through `parseWorkspace` at
`Chrome.cpp:776`; add `--builder-open FILE` and `--builder-save FILE`); `App.cpp` headless
changes (R6); `viewer/client/include/lagrange/viewer/ViewerClient.hpp` and
`client/src/ViewerClient.cpp` (`setQuiet(bool)`, R7); `viewer/client/test/test_viewer_client.cpp`
(quiet-mode test); `viewer/builder/test/test_test_server.cpp`.
Behaviour: Test validates, writes the URDF (absolute meshes), and starts
`lagrange_server --robot F --world none|W --joint-control --listen unix:<socket>` with stderr to
the log file through `posix_spawn_file_actions_addopen`. The client switches to the test server,
remembering the previous endpoint or "launcher". If the server exits before it is Ready, its log
tail goes into Problems and the mode returns to Build. Back sends SIGTERM, SIGKILL after 3 s,
and reconnects. Save, Export USD (`lagrange_import`), Check loads (`lagrange_check`) and xacro
expansion run as child processes polled each frame, never `system()` and never blocking. The
autosave goes to `$XDG_STATE_HOME/lagrange/builder/<robot>.autosave.urdf`.
Tests: `TestServer.StartsPollsAndStops` (with `/bin/sleep`, not the server),
`.KillsAfterGrace`, `.LogTailCaptured`; `ViewerClient.QuietSendsEmptySubscriptionsAndRestores`
(against the in-process test server of `test_viewer_client.cpp`).
Depends on: A (validate, History), B (writeUrdf/readUrdf), WP0. The headless acceptance needs E.

### WP-G: integration and acceptance (S). Last.

Run the SPEC acceptance: the headless script (§2.2), the frame time on llvmpipe at 1600×1000
(the median and p95 lines printed by headless runs, `App.cpp:55-85`), no sensor/field/world
messages while the Builder is in front (client counters), and the screenshot against
`12-builder/reference.png` with every remaining difference listed in the PR.
Docs: `docs/design/viewer.md` (Builder section), `CLAUDE.md` §7 (the new flags, Ctrl B), and a
user page `docs/guide/builder.md` with its link in `docs/index.md`. The two `00-design-system`
widgets wait for the owner's review (SPEC "New in this screen"): the PR says so.

### 2.2 The headless script (`viewer/test/test_builder_input.py`)

It runs `lagrange_viewer --headless --size 1600x1000 --workspace builder --builder-open
viewer/test/builder/empty.urdf --input "…" --builder-save OUT.urdf --screenshot OUT.png`, with
no `--connect`. Steps: click Box in Add; select its joint; Segmented → revolute; type lower,
upper, effort and velocity (`text:`, `key:tab`); catalog row `rangefinder` → Add; Properties
`range/max` → 20 `key:enter`; `key:ctrl+z`, then `key:ctrl+shift+z`; Test; wait for the
connection; check the Test layout; `key:escape` (Back); then compare `OUT.urdf` with
`golden/builder_input.urdf` byte for byte. Click coordinates are constants at the top, as in
`test_viewer_input.py:19-28`.

## 3. Merge order and seams

```
WP0 ──┬─ WP-A ─┬─ WP-F ─┐
      ├─ WP-B ─┘        ├─ WP-G
      ├─ WP-C ──────────┤
      ├─ WP-D ──────────┤
      └─ WP-E (E1, E2) ─┘
```

1. **WP0 first, alone.** It is the only PR that touches `Panel.hpp`, the `kWorkspaces` table,
   `viewer/CMakeLists.txt` (target list) and `viewer/package.xml`. Later packages add their
   `.cpp` files to the existing target lists only (one line each, easy to merge).
2. **A, B and C** in any order. They share no files. C's YAML can merge before its loader.
3. **D and E** after A if they can, so they replace their stubs in the same PR.
4. **F** after A and B. It is the only package that edits `App.cpp` beyond WP0's two call sites
   and `ViewerClient`.
5. **G** last. It also switches the Builder tab from "not built yet" to live, if E1 has not.

Agents: 5 at once after WP0: A then F (one agent), B, C, D, E. Use 6 if E is split.
Hot files and who owns them: `Chrome.cpp` (WP0 then E1), `App.cpp` (WP0 then F),
`Icons.cpp` and `Widgets.cpp` (WP0 then E1), `viewer/CMakeLists.txt` (WP0, then one-line
additions). Each package works in its own worktree from `dev`, and rebases on `dev` before its
PR.

## 4. Risks and gaps found in the code

R1. **No Builder without a server today.** `drawPanels` draws only the launcher while
`launcher->active` (`Chrome.cpp:832-836`). Entering the Builder with no connection must set
`launcher_.active = false` without connecting (Ctrl B on the launcher). The Launcher tile is a
change to the owner's `01-launcher` design: build it only if the owner asks (RFC open question 6).

R2. **Run shortcuts act in every workspace.** Space plays or pauses, Ctrl R records, and `.`
steps the connected run (`Chrome.cpp:851-866`). In the Builder that would change a run the
builder must leave alone (open question 7). Gate them off while the Builder is in Build mode.
Esc also clears `selectedPrim` (`:867`): Back (Esc) must not fight it. Ctrl Z, Ctrl S and
Ctrl Y must check `!WantTextInput`, or they fight ImGui's own text undo.

R3. **One client, one renderer.** `App` has one `ViewerClient` and one `SceneRenderer`. Test
switches the client's endpoint (stop, `setEndpoint`, start, as the launcher does,
`App.cpp:546-548`). While that happens, `frameScene` re-aims the camera on a new stage
(`:446-447`), and `recordRecent_` adds the test run to the Recent list (`:460-468`): suppress
both for test servers. The Builder view needs its own `Camera`, and the renderer's
`drawnKey_` cache (`:584-598`) must key on which scene is drawn.

R4. **`LocalServer` cannot be reused as it is.** It passes only `--listen` (`App.cpp:213`),
inherits stderr (no capture, `:219-220`), and `stop()` blocks on `waitpid` with no SIGKILL
(`:229-235`). Blocking breaks CLAUDE.md §8 (no blocking in the frame loop). Hence `TestServer`.
Memory note: after a SIGKILL, run Fast DDS `/dev/shm` hygiene (`fastdds shm clean`) in tests.

R5. **ImGui window names collide.** The Scene workspace's Problems window is `###Problems`
(`ConsolePanel.cpp:58`), and "Console" is docked by name (`Chrome.cpp:103`). A window lives in
one dock node, so the builder's Problems, Console and Properties need their own IDs. The SPEC's
Test layout puts "the normal server-fed Viewport" in the Builder. That needs a second
`ViewportPanel` instance under another ID, and a second dock space or a layout rebuild for
Test. The RFC does not say which. Proposal: dock space `lagrange-dock-builder-test-1`, chosen
in `drawDockSpaces` (`Chrome.cpp:707-735`) while testing.

R6. **Headless needs three changes.** (a) Headless connects on start-up unless `--launcher`
(`App.cpp:1363-1364`). (b) Headless exits with code 4 when no scene arrives (`:1424-1433`), and
the Builder has no server scene. (c) `--input` keys are only enter, tab and escape
(`:1338-1341`), so add `ctrl+z`, `ctrl+shift+z`, `ctrl+s` and `ctrl+b`. Frames are not timed,
so a server start (0.75 s, RFC table) is a matter of frames. Add an input kind
`FRAME:wait:connected` that holds the frame counter until the client is Ready, instead of
guessing frame gaps.

R7. **"Builder unsubscribes" has no mechanism.** Streams are owned by panels:
`SensorsPanels.cpp:166/169`, `EnvironmentPanels.cpp:96`, scopes through `ScopeSession`, which
updates every frame in every workspace (`Chrome.cpp:840`). `setWorldWanted(true)` and
`setRosGraphWanted(true)` are never set back to false (`WorldPanels.cpp:52`,
`Ros2Panels.cpp:45`). Hidden panels do not drop their streams. A reconnect re-sends every
sensor and field subscription (`ViewerClient.cpp:940-953`), so the test server would get them
too. Fix: `ViewerClient::setQuiet(bool)`. While quiet it sends `Unsubscribe` for every signal
and `SubscribeSensor` / `SubscribeField` / `SetWorld` / `SetRosGraph` in their "off" form. These
are existing messages (`ViewerClient.cpp:163-187`, `589-598`), so there is no protocol change.
It restores everything on leaving. Scene deltas keep coming (the SPEC does not ask to stop them).

R8. **Test server ROS collides with a local run.** The test server runs with ROS in the user's
`ROS_DOMAIN_ID` (RFC §7.1.3). If the run the viewer was connected to also has ROS on that
domain, both publish `/clock`, `/tf` and `/odom`, and both take `/cmd_vel`: a teleop drives
both robots. The RFC does not cover this. Options for the owner: warn and offer `--no-ros`; or
remap the test server's topics (`--ros-args` is passed through, `server/src/CommandLine.cpp:207`;
TODO(verify) that `/clock` and `/tf` can be remapped as well); or set another `ROS_DOMAIN_ID`
for the test server and print it in the Drive panel. Proposed: the last one.

R9. **Catalog paths for built-in types.** IMU noise is read from the first axis that has
`<noise>` per channel (`imuNoise`/`firstNoise`, `Robot.cpp:733-755`). The catalog's IMU
parameters must be per channel, and the writer writes the same noise on x, y and z. `latency`
and `jitter` are read for every sensor (`Robot.cpp:1026-1030`), but `kNotParameters`
(`:786-787`) does not hold them, so for kinds they also land as parameters. The catalog must
not list them as kind parameters, and the writer writes them once.

R10. **`test_sensor_catalog` can pass without testing anything.** `event_camera` and
`stereo_camera` need a Vulkan device (`kRenderingKinds`, `Kinds.cpp:110`): run them on llvmpipe
or skip them with a reason. Kinds that read context (battery_monitor, current_sensor,
voltage_sensor, esc_telemetry, tension_sensor, tpms, vehicle_can, wheel_odometry, joint_torque,
force_torque, gripper_sensor) give the same nothing either way on a bare body. Run each kind on a
robot that gives it what it reads, and assert that a signal changes over the second.
`check_sensor_catalog.py`: the §9 tables are not uniform. Some have 3 columns and some 4
(`rangefinder` at sensors.md:1281 against the 4-column tables at :811, :853, :896). Some rows group parameters
(`range/min`, `range/max`), some use ellipses (`fx_range ... tz_range`), and some defaults are
prose ("∞ (or the preset's)"). Either normalise the tables in the same PR (docs only) or give
the script explicit grouping rules. Decide before writing 37 entries.

R11. **The build dependency on `lagrange_sensors` is heavy.** That package depends on
`lagrange_core`, `lagrange_gpu`, `lagrange_thermal`, embree and tbb (its `package.xml`). With
`<depend>` and `find_package(lagrange_sensors)`, the viewer waits for the whole sensor stack in
colcon and imports its exported targets. Use `<build_depend>` only. Find the YAML at configure
time from `AMENT_PREFIX_PATH` (`share/lagrange_sensors/catalog/`) without `find_package`, and
install the copy. The deb build-depends on `ros-lyrical-lagrange-sensors` either way: check
with `bloom-generate` (packaging README) that the graph stays acyclic.

R12. **Mesh shapes in phase 1.** `Shape::Kind::Mesh` is read and written in phase 1 (11 sample
URDFs; `holybro_x500_v2.urdf` names 13 `lagrange-asset://` meshes), but it is drawn and given
mass only in phase 2. Phase 1 draws its collision box if the file has one, or nothing, with a
hint. "Meshes found" must resolve `lagrange-asset://` through `$LAGRANGE_ASSET_DIR` and
`package://` through the ament index, or every opened sample shows false errors.

R13. **`lagrange check` prints a table, not problems** (`server/src/check_main.cpp:46-59`;
exit 1 when the pose cannot be held). Problems shows its exit status as one row and the table
in mono under it. Rows map to links by name.

R14. **The primitive tessellation lives in `lagrange_core`** (`Model.hpp:46-47`), which the
viewer may not link. `Primitives.cpp` copies the same counts, so the builder and the test run
look the same. A test pins the counts (WP-D).

R15. **Frame budget.** The SPEC wants Build within 16.6 ms on llvmpipe. Statics are rebuilt
only on document edits (decision 2). `validate()` runs on commit, not every frame. The catalog
panel uses `ImGuiListClipper`. FK runs per frame only while a slider moves.

## 5. Phase 1b: protocol and server PRs

All are additive under RFC 0007. Each PR carries the CLAUDE.md §9 note, needs 2 approvals, and
passes `buf lint && buf format --diff --exit-code && buf breaking --against
'.git#ref=origin/dev'`. Each PR updates `docs/design/protocol.md` (the Command list at
`:464-466`) and `mock_server/lagrange_mock_server/server.py` (`:549`, `:611`). An old server
answers an unknown command with a generic error (`server/src/Server.cpp:3478-3482`), so a new
viewer degrades cleanly.

**1b-1 `fix(server)`: accept `ImportRobot.content`, name from `<robot name>` (S, no proto).**
`Server.cpp:3391-3412`. When `path` is empty and `content.inline` is set, run it through
`writeResource` (`:6405`) and `spawn(path, robotName, true, importPose(), /*temporary*/ true)`.
The name comes from `<robot name>`, and `spawn` renames on a clash (`:4697-4712`). `blob_id`
and `shm` content are refused by line ("send it inline"). `overlay_path` and `xacro_args` stay
refused until 1b-3. Tests: `server/test/test_spawn.cpp`
(`ImportContent.NamesFromRobotElement`, `.RenamesWhenTaken`, `.RefusesNonUrdfText`), plus a
socket-level case where the server test suite already drives commands (TODO(verify) which
file). PR note: "No change to `proto/`: `ImportRobot.content` (field 3) exists since 1.0."

**1b-2 `feat(proto)`: `RemoveRobot` (M).** `Command.remove_robot = 33`,
`message RemoveRobot { string name = 1; }` → `Server::removeRobot`. Gap: the viewer must know
the spawned name to remove it. `ImportRobot`'s result carries only `imported_stage_path`
(`command.proto:152-156`, set at `Server.cpp:3411`), and `spawn` may have renamed the robot.
Proposed, in the same PR and needing the owner's approval because RFC §11 does not list it:
`CommandResult.imported_robot = 4` (`string`, the spawned name) and
`Capabilities.can_remove_robot = 8`, so the viewer offers "Into the connected run" only when the
server takes it. Without them, the viewer must diff the top-level prims of the snapshots before
and after, which is fragile. PR note: "Changes `proto/` (CLAUDE.md §9): additive under RFC 0007,
RFC 0016 §11 and open question 2; field numbers 33 and (if approved) CommandResult 4 and
Capabilities 8 were unused. Needs 2 approvals." Tests: `buf breaking`; the
`test_viewer_client.cpp` round trip; server `RemoveRobot.RemovesASpawnedRobot`,
`.RefusesTheStartupRobotUnderRos2Control` (the existing rule, `Server.cpp:4755-4758`); the mock
server answers it.

**1b-3 `feat(proto)`: `ImportRobot.overlay_content = 6` (S).** `DataRef`, inline only. The
server writes it beside the robot's temporary file and applies it as `--overlay` does. Same §9
note, field 6 unused. Tests: a spawned robot with an overlay battery has `bus/…` signals. The
builder only uses this in phase 3 (overlay sensors and motors), so it can merge later than
1b-2.

**1b-4 `feat(viewer)`: Test into the connected run (M).** Enable the greyed menu item when
`can_import` and `can_remove_robot` are set. Send the content, keep the returned name, and Back
sends `RemoveRobot`. Show "passive: the run already has a robot" when it is passive
(`Server.cpp:4732-4736`). Headless test against a real server started by the test.

`CommandJoint` (field 34) stays in phase 2, as RFC §11 says.

## 6. Density presets (open question 9)

No primary data sheet was read for this plan; one site gave a network error and another a
not-found page. The repository already carries densities with named sources in the thermal
presets of RFC 0012 (`modules/thermal/conduction/src/Materials.cpp:16-55`). The builder should
use **the same names and numbers** as those presets, so a link's mass and its thermal material
agree. The viewer cannot link `lagrange_thermal` (invariant 14), so `viewer/builder/src/Materials.cpp`
holds a copy, and a check in `check_sensor_catalog.py`'s family (or a gtest in
`lagrange_thermal`) compares the two tables. Every value stays TODO(verify) until someone reads
its primary source.

| Preset | ρ kg/m³ | Source as cited in the repository | Status |
|---|---|---|---|
| `aluminium_6061` (6061-T6) | 2700 | "ASM/MatWeb data sheet" (`Materials.cpp:18-21`) | TODO(verify): read ASM Aerospace Specification Metals, 6061-T6 |
| `steel_304` (stainless AISI 304) | 7900 | Incropera et al., *Fundamentals of Heat and Mass Transfer*, 7th ed., Table A.1, 300 K (`:27-29`) | TODO(verify): read Table A.1 |
| steel, carbon (AISI 1018) | none | none in the repository | TODO(verify): candidate Incropera Table A.1 "plain carbon steel", or a mill data sheet; or drop it and keep 304 |
| `pla` | 1240 | `Materials.cpp:34-36` names makeitfrom (k) and the Prusament PLA TDS (Tg) but no source for ρ | TODO(verify): candidate NatureWorks Ingeo 4043D technical data sheet (specific gravity, ASTM D792) |
| `abs` | 1100 | Ultimaker ABS TDS v5 (`:37`) | TODO(verify): read the TDS |
| `nylon_pa12` (MJF/SLS) | 1010 | Weerg and Xometry data sheets (`:42-43`) | TODO(verify); note that PA6 or PA66 injection grades are denser (TODO(verify), candidate BASF Ultramid B3S / A3K sheets, ISO 1183) |
| rubber, natural | none | none | TODO(verify): candidate a natural rubber compound data sheet (ISO 2781); `Materials.cpp:52` has *silicone* rubber 1200, itself TODO(verify) |
| pine | none | none | TODO(verify): candidate USDA FPL *Wood Handbook* (FPL-GTR-190), Table 5-3a, specific gravity of a named pine species at 12 % moisture; give the species, as density varies by species and moisture |

The picture's 1100, 1240 and 2700 kg/m³ (`12-builder/README.md`) match the repository's ABS,
PLA and aluminium rows. Each preset in code carries its `source` string, shown as a tooltip in
the Add panel's material menu. "custom" asks for a number and stores it as `density`.

## 7. Out of scope here

Phase 2 (meshes through `lagrange_meshes`, `CommandJoint`, the start pose), phase 3 (the
overlay) and phase 4 (USD, SDFormat) get their own plan when phase 1 is merged.

## 8. Note: Addendum A changes to the §3 structs and interfaces (2026-11-15)

RFC 0016 Addendum A (meshes and the collision choice) changes the WP0 interfaces by addition:

- `Shape::Kind` gains `Capsule` (appended, so `kDragShape` values keep): `size` {radius,
  length of the straight part}, written as `<capsule>` with `<robot version="1.1">`, which
  urdfdom needs to read it (it ignores a capsule in a 1.0 file).
- `Link` gains `CollisionChoice collision` (Unset, Mesh, Split, Shapes, Drawn), written as
  `lagrange_collision` on `<link>` like `lagrange_density`.
- `lagrange_viewer_builder` links `lagrange_meshes` privately (open question 8); new headers
  `MeshShapes.hpp` (mesh files, Mirtich mass, the server's hull, Fit to mesh), `Collision.hpp`
  (the choice's edits, `addMeshLink`) and `CollisionSplit.hpp` (CoACD through `bake_urdf` as a
  polled child process). No builder header includes a core header.
- `buildStatics` takes `BuildOptions` (draw collision geometry translucent) and `BuiltScene`
  gains `primShape` (which shape a prim draws), for the gizmos of choice 4.
- `BuilderSession` gains the split request and status, and `recentMeshes`.

## 9. Note: phase 3 changes to the §3 structs and interfaces (2026-11-15)

RFC 0016 phase 3 (motors, battery, bus and the overlay-only sensors, written to the overlay
YAML) changes the WP0 interfaces by addition:

- `Document` gains `Overlay overlay` (appended): the overlay file as it says it, entries of
  schema instances by prim (`prim`, `create_child`, `create`), values as typed, so any overlay
  (thermal nodes, rotors, wheels) reads and writes back the same. An entry may instead be a
  placeholder (`sensor`) that keeps an overlay-only sensor's place in the file.
- GNSS, magnetometer, barometer and radar stay `Document::sensors` (the tree, the catalog's form
  and validation treat them as any sensor); `writeUrdf` leaves them out and `writeOverlay` writes
  them as `create_child` entries with `SensorAPI_1` and their schema. The overlay has no pose: they
  sit at their link's origin (a warning otherwise).
- New header `Overlay.hpp`: `readOverlay`, `writeOverlay`, `sameOverlay` (the round-trip tests),
  the forms' fields of `BatteryAPI_1`, `ElectricalBusAPI_1`, `ElectricalMotorAPI_1` and `PortAPI_1`
  (checked against `schemas/electrical/schema.usda` by `test_overlay`), presets from the sample
  overlays, and the edits (`addBus`, `addBattery`, `addMotor`, `removeEntry`; renames and removals
  of links, joints and sensors follow into the overlay). `Validation.hpp` gains `checkOverlay`,
  which `validate` includes.
- `TestRequest` gains `overlay` (`--overlay FILE`); Test into the connected run sends the text as
  `ImportRobot.overlay_content`; Save and Open read and write `<robot>.overlay.yaml` beside the URDF.
- UI: a **Power** section in the robot's Properties (batteries, buses, other entries kept as
  written) and a **Motor** section in a joint's (12-builder SPEC "Properties", added there).
