# Robot builder: phase 4 work plan, USD round trip and SDFormat (RFC 0016)

Status: plan, 2026-11-15. RFC 0016's Phases table leaves phase 4 "to be decided": "USD round trip
(open `.usda` made by `lagrange_import`), SDFormat". This file says what that would mean, lists
the options with their costs, recommends one, splits it into work packages for agents working at
the same time, and lists the risks found in the code and the questions for the owner. §12 is a
draft Addendum B for the RFC (Status: Draft) that the owner can accept or reject.

Written from `origin/dev` d535fef (builder phases 1 to 3 and Addendum A merged) by reading the
code. Nothing was built or run. Every claim about the code cites `file:line`; claims that need a
run to confirm say "to verify".

Read with: `rfcs/0016-robot-builder.md` (§ numbers below are the RFC's),
`docs/plan/builder-phase1-plan.md` (decisions 1 to 7, notes §8 and §9),
`io/stage/include/lagrange/stage/Stage.hpp`, `io/importers/include/lagrange/importers/Importers.hpp`.

## 0. The recommendation in one paragraph

Keep URDF plus the overlay YAML as the builder's document on disk; do not link OpenUSD or
libsdformat into the viewer. **Save USD through `lagrange_import`** (as Export USD already does),
made atomic and with the builder's own URDF and overlay **embedded in the layer's
`customLayerData`** together with a digest of what the layer loads to. **Open USD, SDFormat and
MJCF through a converter child process**, `lagrange_builder_convert`, polled like the xacro
expansion: it loads the file with the simulator's own readers (`stage::loadStage`,
`importers::importSdf`, `importers::importMjcf`) and either returns the embedded source when the
layer still loads to the digest (lossless), or rebuilds a builder `Document` from the world tables
and writes it with the builder's own URDF and overlay writers (one writer, not two), with a
Problems note for each thing that could not come back. SDFormat is **open-only** in phase 4;
saving SDFormat is a later, optional step (§5.4). Two gaps in what USD carries (joint velocity
limits, effort of passive joints) make every revolute and prismatic joint read from a USD file
fail the builder's validation today; the owner decides how to close them (§11 question 1).

## 1. What "open and save USD and SDFormat in the builder" means

User stories, in the order they matter:

1. **Builder-made USD**: I built a robot, exported USD, sent the `.usda` on. Later I open the
   `.usda` in the builder and get my robot back exactly: names, primitives, mesh paths, densities,
   collision choices, sensors with every parameter, the power system.
2. **USD made by `lagrange_import` from someone's URDF** (the samples: `amr_diffdrive.usda`,
   `quad_x500.usda`, `sedan_base.usda`): I open it and get an editable robot as close to the
   original as the file allows, with a list of what could not come back.
3. **USD written by hand or composed** (`amr_diffdrive_powered.usda`, `sedan.usda`,
   `quad_x500_emi.usda` sublayer an imported base and add schemas with `over`): I open it and see
   the robot and its power system; saving never destroys the composition.
4. **SDFormat** (`amr_diffdrive.sdf`, Gazebo models): I open a `<model>` and edit it.
5. **Save as USD** from a document opened from USD, so the file I opened stays the file I keep.
6. (Optional) **Save as SDFormat** for Gazebo.

Out of scope: editing a world (several models, scenery), round-tripping USD composition arcs,
editing USD prims the builder does not model (they are kept only through the embedded source).

## 2. What exists today (checked on d535fef)

| Need | Today | Consequence |
|---|---|---|
| Builder open | `.xacro` goes through a polled `xacro` child process into a URDF; anything else is read as URDF text (`viewer/app/src/BuilderHost.cpp:757-793`) | a `.usda` or `.sdf` fails as malformed URDF. The xacro job (`BuilderHost.hpp:84-85`, `Kind::Xacro`) is the pattern to copy |
| Builder Export USD | writes the document to a runtime copy, then `lagrange_import COPY OUT.usda [--overlay COPY.yaml]` next to the saved file (`BuilderHost.cpp:860-905`) | Save USD already exists as an export. `lagrange_import` is found under `lib/lagrange_server` (`BuilderHost.cpp:597-601`) |
| `lagrange_import` | URDF, SDFormat or MJCF to USD through `server::loadModel`, refuses USD input (`server/src/import_main.cpp:72-78`), writes with `stage::writeStage` | the only USD writer; the loader stays the judge |
| USD writer | Model to USD (`io/stage/src/StageWriter.cpp`): visuals as triangulated `Mesh` prims (`:178-207`), colliders as `Cube`/`Sphere`/`Cylinder`/`Capsule` or convex-hull `Mesh` with `purpose = guide` (`:511-615`), mass as `float` and inertia as float principal moments plus a float quaternion (`:457-461`), joints as `UsdPhysics` joints with float limits in degrees (`:700-722`), no velocity limit anywhere in `writeJoints` (`:666-780`), effort only on actuated joints as `PhysicsDriveAPI` `maxForce` (`:766-775`), fixed joints named `<child>_weld` (`:683`), sensors as `SensorAPI_1` plus a type schema from the world's settings (`:276-443`), schema instances on their prims with fallback values left out (`:825-905`), written with `SdfLayer::Export` (`:934`) | what a USD file can give back is what the world tables hold, in float |
| USD reader | `stage::loadStage` (`io/stage/src/StageReader.cpp`): bodies, joints at body1's origin, colliders, scene, sensors, schema instances; reads `customLayerData` (`:1599-1603`, `lagrangeGroundPlane`) | layer metadata is already a channel the loader reads; a precedent for §5.3 |
| SDFormat reader | `importers::importSdf` (libsdformat 16, `io/importers/src/SdfImporter.cpp`) into a `Description`, then `build()` into world tables (`io/importers/src/Description.hpp:105-115`) | the same Model as USD; SDF frames are resolved by libsdformat |
| MJCF reader | `importers::importMjcf` | comes for free with the SDFormat path |
| Overlay | `stage::applyOverlay` (formats.md §9); the builder reads and writes it as typed entries (`viewer/builder/include/lagrange/viewer/builder/Overlay.hpp`) | the overlay is the builder's form of schema instances; USD's schema instances map onto it (§3.6) |
| Builder library | `lagrange_viewer_builder`, static, not exported (`viewer/CMakeLists.txt:44-75`, `:235-244`) | another package cannot link the builder's writers until it is exported (WP4-0) |

## 3. Field by field: the Document in USD and in SDFormat

"USD" below is what `lagrange_import` writes today and `loadStage` reads back. "Back" is what a
rebuilt Document gets from the world tables (the reconstruction of §5.2), without the embedded
source of §5.3. With the embedded source and an unchanged layer, every row comes back exactly.

### 3.1 Robot and links

| Document | USD | Back | SDFormat |
|---|---|---|---|
| `robot` | the top `Xform`, also `defaultPrim`; name through `primName` (non-alphanumerics to `_`, `StageWriter.cpp:93-102`) | the top prim's name | `<model name>` |
| `Link::name` | an `Xform` with `PhysicsRigidBodyAPI` + `PhysicsMassAPI`, nested along the joint tree | the prim's name | `<link name>` |
| link frame | the child prim's `xformOp:translate` (double) and `orient` (quatd), at rest | exact (double) | `<link><pose relative_to=…>`; libsdformat resolves it |
| root placement | the URDF loader lifts the robot onto the ground; the top prim carries it (`amr_diffdrive.usda`: `translate = (0, 0, 0.10100000066987239)`) | dropped: the root link is the document's origin | `<model><pose>`: dropped |
| fixed base | URDF root link `world`; in USD a `PhysicsFixedJoint` with no `body0` | a root link `world` and a fixed joint to it | `<joint>` to `world` |
| floating base | none written (`StageWriter.cpp:666-668`); the reader adds `<body>_floating` (`StageReader.cpp:445-448`) | no joint (URDF's floating root) | none |
| `Link::keptXml` (`<gazebo reference>` children) | some become schemas or materials (§3.7); the rest is lost | see §3.7 | some have SDF elements (§3.7) |

### 3.2 Shapes

| Document `Shape` | USD | Back | SDFormat |
|---|---|---|---|
| Box, Cylinder, Sphere **visual** | triangulated `Mesh` prims `visual`, `visual_N` with `primvars:displayColor` (`Robot.cpp:502-546`; tessellation `core/world/include/lagrange/world/Model.hpp:24-50`) | **kind lost**; recovered when the triangles equal the core tessellation of a fitted size (decision D4) | `<visual><geometry><box><size>` (full edges), `<cylinder><radius><length>`, `<sphere><radius>` |
| Capsule **visual** | drawn as a cylinder (`Robot.cpp:542-546`, "capsule visuals are drawn as cylinders") | comes back as a cylinder (lost) | `<capsule>` |
| Mesh **visual** (`mesh`, `scale`) | the file's triangles inline, one `Mesh` prim per material; the file path is not written | **path lost**; triangles written as OBJ files beside the document on Save (decision D5) | `<mesh><uri><scale>` |
| Box collision | `Cube` `size = 1`, `xformOp:scale` = full edges (`StageWriter.cpp:511-519`) | exact kind, size in float | `<collision><geometry><box>` |
| Cylinder, Capsule collision | `Cylinder`/`Capsule` `axis = Z`, `height` = full straight length (`:527-544`) | exact | `<cylinder>`, `<capsule>` |
| Sphere collision | `Sphere` `radius` | exact | `<sphere>` |
| Mesh collision (choice 1) | the convex hull's points as a `Mesh` with `PhysicsMeshCollisionAPI` `convexHull` (`:545-569`), scale baked in | a hull, path lost; choice `Mesh` when the hull equals the visual mesh's hull, else a collision-only mesh shape written as OBJ | `<mesh>` (the importer skips it today, risk R9) |
| Split parts (choice 2) | one hull `Mesh` per part | `Split`, the parts written as OBJ into `<name>_collision/` | one `<collision>` per part |
| `Shape::visual`/`collision` on one shape | separate prims | one Shape with both flags when a visual primitive and a collider coincide in kind, size and pose (float tolerance) | separate `<visual>` and `<collision>` |
| `Shape::solid` | nothing (mass sits on the body) | default rule (collision shapes solid) | nothing |
| `Shape::rgba` | `displayColor`, `displayOpacity` (`:199-205`) | float rgba | `<material><diffuse>` |
| `Link::collision` (`CollisionChoice`) | nothing | inferred as above, `Unset` when unclear | nothing |

### 3.3 Inertial

| Document `Inertial` | USD | Back | SDFormat |
|---|---|---|---|
| `mass` | `physics:mass` **float** (`StageWriter.cpp:457`) | override, printed as the shortest decimal that gives the same float (decision D3) | `<inertial><mass>` |
| `com` | `physics:centerOfMass` float3 | override, same printing | `<inertial><pose>` |
| `inertia` (ixx ixy ixz iyy iyz izz) | `diagonalInertia` float3 + `principalAxes` quatf (`:459-461`) | full tensor rebuilt as `inertiaTensor` does (`StageReader.cpp:416-424`); off-diagonals of 1e-9 noise snapped to 0 at float precision; **reordered** from the world's xx, yy, zz, xy, xz, yz (`World.hpp:157`) to the document's order | `<inertia><ixx>…` |
| `density` | nothing (`UsdPhysicsMassAPI` has `physics:density`, but the reader uses mass only, `StageReader.cpp:426-431`) | 0, so the inertial is shown as the user's overrides (RFC §10 rule for files) | nothing |
| a massless link | written with `mass = 0` (`:119-120`, `:457`), which the reader turns into 1 g with a note (`StageReader.cpp:426-430`) | 1 g, risk R5 | none |

### 3.4 Joints

| Document `Joint` | USD | Back | SDFormat |
|---|---|---|---|
| `name` | the joint prim under `<top>/joints`; a **fixed** joint is named `<child>_weld` (`StageWriter.cpp:683`), because the URDF loader keeps no row for fixed joints (`Robot.cpp:352-353`) | moving joints exact; fixed joints `<child>_weld` | `<joint name>` |
| `type` | `PhysicsRevoluteJoint` (limits) or continuous (no limits), `PhysicsPrismaticJoint`, `PhysicsFixedJoint`; spherical exists in the world tables | exact for the four builder types; spherical and free have no builder type (note, made fixed) | `revolute`, `continuous`, `prismatic`, `fixed`, `ball` |
| `parent`, `child` | `physics:body0`, `body1` | exact | `<parent>`, `<child>` |
| `origin` | the child prim's rest pose (double); `localPos0`/`localRot0` (float) are written but the reader takes the prim pose (`StageReader.cpp:396-408`) | exact | the child link's `<pose relative_to=parent>`; libsdformat moves a joint frame that is not the child frame, and `build()` re-anchors the body at the joint (`Description.hpp:105-108`), so visual and collision origins move |
| `axis` | `physics:axis = X` turned by `localRot1` (quatf) | float precision; components within 1e-6 of 0 or ±1 snapped (decision D3) | `<axis><xyz>` |
| `lower`, `upper` | revolute in **degrees, float** (`:705-709`); prismatic in m, float | radians from float degrees: `0.7853981` comes back as `0.785398185…` before printing; shortest-float printing of the degree value then conversion (decision D3) | `<limit><lower><upper>` |
| `effort` | `maxForce` of `PhysicsDriveAPI` on **actuated joints only** (`:766-775`) | empty on passive joints | `<limit><effort>` |
| `velocity` | **not written** (`writeJoints`, `:670-777`; the world has `velocityLimit`, `World.hpp:175`) | **empty**: every revolute and prismatic joint is then an Error in `validate` (`viewer/builder/src/Validation.cpp:98-101`), so Test is refused (risk R1) | `<limit><velocity>` |
| `damping`, `friction`, `mimic` | `JointDynamicsAPI_1` (doubles, `:746-764`) | exact | `<dynamics>`, `<axis><mimic>` |
| actuation | `PhysicsDriveAPI` | a `<ros2_control>` block listing the actuated joints, kept XML at robot level (decision D6), so the server drives the same joints | joints named by a model `<plugin>` |
| `keptXml` | lost | lost | lost |

### 3.5 Sensors, the catalog kinds as `CustomSensorAPI_1`

| Document `Sensor` | USD | Back | SDFormat |
|---|---|---|---|
| `name`, `link`, `origin` | a prim named as the sensor under its link's prim; a camera is a `UsdGeomCamera` turned to look along −z (`StageWriter.cpp:209-218`) | exact (the reader turns it back) | `<sensor name>` in `<link>`, `<pose>` |
| `updateRate`, `latency`, `jitter` | `sensor:rate`, `sensor:latency`, `sensor:jitter` (`:290-297`) | exact | `<update_rate>`; latency and jitter are not read by `importSdf` (lost) |
| `imu` | `ImuAPI_1`: `accelNoiseStd`, `gyroNoiseStd` only (`:437-441`) | two numbers; the catalog's per-axis bias, dynamic bias and precision (`modules/sensors/catalog/sensor_catalog.yaml:75-139`) are lost | `<imu>` noise per axis (the importer keeps the largest stddev, `SdfImporter.cpp:532-540`) |
| `lidar` | `LidarAPI_1`: channels, samples, fields of view, range, range noise, pattern (`:298-316`, `:229-274`) | those parameters; the catalog's others (noise type, resolution, …) lost | `<lidar>`/`<ray>` |
| `camera`, thermal | `UsdGeomCamera` lens + `CameraSensorAPI_1` + `ThermalCameraAPI_1` (`:317-392`) | the lens and sensor model; image format and other URDF-only elements lost | `<camera>` |
| GNSS, magnetometer, barometer, radar (overlay-only in the builder) | `GnssAPI_1`, `MagnetometerAPI_1`, `BarometerAPI_1`, `RadarAPI_1` (`:393-422`) | `Document::sensors` with their overlay placeholder entries, as `readOverlay` makes them | `navsat`, `magnetometer`, `air_pressure` are read as **custom kinds** today (`SdfImporter.cpp:573-577`; risk R10) |
| a **catalog kind** (`rangefinder`, …) | `CustomSensorAPI_1`: `sensor:custom:kind`, and each parameter as `sensor:custom:param:<path with ':' for '/'>`, double or string (`StageWriter.cpp:423-436`, schema `schemas/sensor/schema.usda:393-410`) | kind exact; numbers as doubles printed shortest (`"10.0"` becomes `"10"`), text exact; the parameters equal the catalog paths (`Robot.cpp:820-848`), so no table is needed | `<sensor type="rangefinder">` with the same nested elements (formats.md §9.3: one block pastes between URDF and SDFormat) |
| kind parameters that are XML attributes (`<path>/@<name>`, `Urdf.hpp` header) | not written: the server never reads attributes of a custom sensor (`Robot.cpp:820-848`), and `@` is not legal in a USD property name | lost (a note per sensor) | attributes kept by the XML |
| `latency`/`jitter` of a kind | also land in the custom parameters (plan R9) and so as `sensor:custom:param:latency` | taken out of the parameters into `Sensor::latency`/`jitter` (decision D7) | — |

Built-in types need a table from the world's setting keys back to catalog paths (`samples` to
`lidar/scan/horizontal/samples`, …). Decision D2 puts that table into the catalog itself.

### 3.6 The overlay and USD in one file

A URDF needs the overlay for the power system and the overlay-only sensors; a USD layer holds both
the robot and its schemas. The mapping:

| Overlay entry | USD (as `writeStage` writes `World::schemaInstances`, `StageWriter.cpp:793-905`) | Back to an overlay entry |
|---|---|---|
| `prim: <link>` + schemas | applied API on the link's body prim | `prim: <link>` |
| `prim: <joint>` + schemas | on the `UsdPhysics` joint prim `<top>/joints/<joint>` (`pathOf`, `:793-823`) | `prim: <joint>` (a fixed joint's is `<child>_weld`) |
| `prim: <sensor>` + schemas (an EMI victim) | on the sensor's prim | `prim: <sensor>` |
| `prim: X, create_child: C` | a new `Xform` `<X's prim>/C`; for a joint, under its parent link (formats.md §9) | `prim: <link the prim is under>, create_child: C` (the same path) |
| `create: power/dc_48v` | an `Xform` at `<top>/power/dc_48v` | `create: power/dc_48v` (path under the top prim) |
| relationship values (names or paths) | `rel` targets as prim paths | names when the target is a link, joint or sensor, else the path under the top prim |
| asset values | relative to the layer when in the same tree (`:871-884`) | absolute against the layer's folder, then `readOverlay`'s base rule |
| a value equal to the schema's fallback | not authored | not written (needs the fallback: decision D8) |
| overlay-only sensor entries | `SensorAPI_1` + type schema on the created prim | `Document::sensors` + placeholder |
| header comment, unit comments, entry order | lost | entries in stage order |
| a netlist (`.cir`) or a URDF `<lagrange>` block | plain schema instances | overlay YAML entries (the netlist form is gone) |

A document opened from USD keeps its overlay as `Document::overlay`; Save as URDF writes the URDF
and `<robot>.overlay.yaml` again, Save as USD folds them into one layer through
`lagrange_import --overlay` (as Export does now).

### 3.7 Kept XML that has a USD or SDFormat home

| URDF (kept by the builder) | USD | Back | SDFormat |
|---|---|---|---|
| `<gazebo reference=L><mu1>` | a physics material on the colliders (`bindFriction`, `:646-664`) | `<gazebo reference=L><mu1>` (kept XML) | `<surface><friction><ode><mu>` |
| `<visual_material preset metallic roughness emissive>` | a `UsdPreviewSurface` look (`bindLook`, `:617-642`) | `<visual_material>` with the numbers (kept XML) | `<material><pbr><metal>` |
| `<sensor_material>`, `<thermal_material>`, `<heat_source>` | `SensorMaterialAPI_1`, `ThermalMaterialAPI_1`, `HeatSourceAPI_1` | overlay entries (equivalent at load) | `lagrange:` elements (formats.md §9.3), not read for these today |
| `<ros2_control>` | `PhysicsDriveAPI` on the joints | a minimal block (decision D6); controller parameters lost | model `<plugin>` |
| `<transmission>`, Gazebo `<plugin>`s, `<material>` names, comments | lost | lost | plugins kept by Gazebo, not read by Lagrange |

## 4. What is lost, each way (summary)

| Way | Lost |
|---|---|
| Builder → USD → builder, **with** the embedded source and an unchanged layer | nothing |
| Builder → USD → builder, layer edited elsewhere (reconstruction) | visual primitive kinds that do not match the core tessellation, capsule visuals (as cylinders), mesh file paths (OBJ written instead), fixed joint names, velocity limits, passive joints' effort, density and the overrides list, collision choice when unclear, `solid`, the catalog parameters USD has no attribute for (IMU bias, lidar noise type, …), custom-sensor attribute parameters, kept XML without a USD home, overlay comments and order; numbers to float precision |
| `lagrange_import` USD of any URDF → builder | the same list as the row above |
| Hand-written or composed USD → builder | as above, plus the composition (sublayers, `over`s): opened flattened, saved only as a new file |
| SDFormat → builder | link frames that are not at the joint (re-anchored), nested model names (flattened with `_`), `<frame>`s, `<include>`s (resolved, not kept), plugins (except which joints are actuated), sensor latency and jitter, mesh visuals and collisions (not loaded by the importer today, risk R9), navsat/magnetometer/air_pressure as custom kinds (risk R10) |
| Builder → SDFormat (optional, §5.4) | `<ros2_control>`, `<transmission>`, Lagrange's URDF-only `<gazebo>` children without an SDF home; the overlay stays a separate file |

## 5. Options

### 5.1 Who writes USD

| | How | For | Against |
|---|---|---|---|
| A1 | the builder writes `.usda` text itself | no child process | a second USD writer that drifts from `writeStage`; the builder would also need the URDF loader's rules (placement, tessellation, sensors' settings) to write what the server would |
| A2 | the builder links `lagrange_stage` (OpenUSD) and the URDF loader | in-process | OpenUSD in the viewer (the largest dependency of the project, and every viewer build waits for it); the URDF loader lives in `lagrange_server`, which the viewer may not link (invariant 14) |
| **A3** | **`lagrange_import` as a child process** (today's Export), atomic, with `--embed-source` | one writer, the server's loader is the judge, nothing new in the viewer | a process per save (Export already pays it) |

### 5.2 Who reads USD and SDFormat, and who writes the URDF that comes back

| | How | For | Against |
|---|---|---|---|
| B1 | in the viewer: link `lagrange_stage` and `lagrange_importers` | in-process | OpenUSD, libsdformat and MuJoCo in the viewer |
| B2 | a converter in `lagrange_server`'s package with its own Model → URDF writer (`lagrange export urdf`) | next to `lagrange_import` and `loadModel` | a **second URDF writer** beside `viewer/builder/src/Urdf.cpp`, with its own sensor wrappers, kept-XML rules and number printing; the two drift |
| **B3** | **a new package `lagrange_builder_convert`** linking `lagrange_stage`, `lagrange_importers` and the exported `lagrange_viewer_builder`: Model → `builder::Document` → `builder::writeUrdf`/`writeOverlay` | one URDF writer; the mapping Model → Document is a pure function with gtests; no server, ROS or physics linked | the builder library must be exported (with `lagrange_meshes` and `lagrange_viewer_client` as its dependencies); one more package and deb; SDFormat's `<lagrange>` power block is read only by `lagrange_server` (`Robot.cpp:1286`), so the converter misses it (note) unless that function moves to `lagrange_importers` |

### 5.3 How builder-made USD comes back exactly

| | How | For | Against |
|---|---|---|---|
| C1 | reconstruction only | simple | every loss of §4 on every round trip, including the user's own files |
| **C2** | **embedded source**: `lagrange_import --embed-source URDF [--embed-overlay YAML]` stores `customLayerData = { dictionary lagrangeBuilder = { string urdf, string overlay, string digest, int version } }`; `digest` is a hash of a canonical dump of what `loadStage` makes of the written layer (bodies, joints, colliders, sensors, schema instances, numbers at float precision). The converter recomputes it on open: equal, the embedded text is returned; different, the layer was edited elsewhere and it falls back to C1 with a note saying so | lossless for the builder's own files; a semantic check, not a byte check, so a re-save by `usdcat` does not count as an edit; no schema change (layer metadata, as `lagrangeGroundPlane` already is, `StageReader.cpp:1599-1603`) | the layer carries a copy of the URDF (tens of kB); a digest function to maintain (it must change when the reader changes, which only makes old files fall back to C1, never wrong) |
| C3 | provenance per prim: `customData` on prims (a visual's source kind and size, a mesh's URI, a fixed joint's name, velocity limits) | helps `lagrange_import` files of any URDF too | many small rules in the writer and the reader; partly duplicates C2; velocity limits in prim metadata would be data the simulator ignores, which a schema addition should hold instead (question 1) |

### 5.4 SDFormat as a Save target

| | How | For | Against |
|---|---|---|---|
| D1 | not in phase 4 (open only) | small | Gazebo users export URDF and convert with `gz sdf -p` (TODO(verify) that `gz sdf` is in Jetty and keeps `<gazebo>` sensors) |
| D2 | a builder-native SDFormat writer from the Document (tinyxml2, `<pose relative_to>` of SDFormat 1.7+, sensors as the same nested XML) | Gazebo gets a file directly; sensors paste as they are (formats.md §9.3) | another writer and its goldens; `<ros2_control>` has no SDFormat home; libsdformat may refuse unknown sensor types in strict mode (TODO(verify)) |

## 6. Recommendation

**A3 + B3 + C2, D1 now and D2 later only if asked.** Concretely:

1. **Open** `.usd`, `.usda`, `.usdc`, `.sdf` and MJCF `.xml` in the builder through
   `lagrange_builder_convert INPUT --out DIR [--json REPORT]`, polled like the xacro job. It writes
   `DIR/<robot>.urdf`, `DIR/<robot>.overlay.yaml` when needed, `DIR/meshes/*.obj` for meshes that
   came inline, and a report (notes, losses, `composed`, `embedded: used|stale|none`). The builder
   opens the result as a new, unsaved document, exactly as an expanded xacro (RFC §10), with the
   report's lines in Problems (Hints, and Warnings for losses that change behaviour).
2. **Save USD** stays Export USD, plus Save as… `.usda`: the copy goes through
   `lagrange_import --embed-source`, written to a temporary file and renamed (risk R12). A
   document opened from a `.usda` that the builder or `lagrange_import` wrote (its `doc` or its
   `lagrangeBuilder` dictionary says so) and that has no composition arcs may be saved over it;
   any other USD is Save as… only.
3. **SDFormat**: open only. Two importer fixes belong to the phase because opening SDFormat is not
   useful without them (meshes, risk R9) or gives wrong sensor types (risk R10); both change what
   `lagrange_server` loads, so each is its own PR with the owner's yes (questions 6 and 7).
4. Joint velocity and passive effort: the owner picks (question 1). Until then, a document from USD
   marks those joints with a Warning-level "the file has no velocity limit" instead of the Error,
   and Test writes nothing for it (the builder never invents a limit). TODO(verify) that urdfdom
   accepts `<limit lower upper effort>` without `velocity`; if it does not, Test of such a document
   must stay refused until the user types one, and the Error stays.

Why: it adds no dependency to the viewer process (invariant 14 and the viewer's build time), keeps
the simulator's readers and writer as the only judges of USD and SDFormat, keeps one URDF writer,
makes the builder's own files lossless, and reuses a pattern (polled child process, new unsaved
document) the builder already has and tests.

## 7. Decisions this plan makes where the RFC is silent

D1. **The document stays URDF plus overlay.** USD and SDFormat are opened by conversion and saved
    by export; `BuilderSession::path` of a converted document is empty until Save as…, and a new
    `BuilderSession::origin` remembers the file it came from, its kind (`usd`, `sdf`, `mjcf`) and
    whether it may be saved over (recommendation 2).
D2. **The catalog names each built-in parameter's world setting.** A param of a built-in type gets
    `setting: samples` (the key `server/src/Robot.cpp` stores, the one `StageWriter` reads), and
    optionally `scale: 0.5` for keys stored halved. `test_sensor_catalog` checks every `setting`
    by writing a URDF with a non-default value and reading `World::Sensors::settings`. The
    converter inverts it; parameters with no `setting` cannot come back from USD (listed). Catalog
    `schema_version` stays 1 (an added key; the loader reads keys by name, `SensorCatalog.cpp:157-175`).
D3. **Numbers from float attributes are printed from the float.** The converter prints
    `std::to_chars(float)` shortest form for values USD stores as float (mass, COM, inertia moments,
    limits, sizes in `xformOp:scale`), converting degrees after rounding the degree value, so
    `physics:mass = 20` reads `20`, not `20.000000298`. Axis components within 1e-6 of 0 or ±1 snap.
    Doubles print as the builder already prints them.
D4. **Primitive recognition.** A visual `Mesh` is a box, cylinder or sphere when its triangle count
    is the core tessellation's (`Model.hpp:24-50`: 12 triangles; 32 segments; 32 × 16) and its
    vertices equal the core mesh of the fitted size within 1e-6 relative. Anything else is a mesh.
D5. **Inline meshes become OBJ files** in `<out>/meshes/<link>_<prim>.obj` (positions only, one
    file per prim, written by the converter); the builder treats them as ordinary mesh shapes and
    Save as… copies them beside the saved URDF (`WriteOptions::meshFolder` rule). The report lists
    each one so the user can point the shape at the original file.
D6. **Actuation survives as a `<ros2_control>` block** at robot level (kept XML) naming each joint
    with `PhysicsDriveAPI` with an `effort` command interface and `position`, `velocity` state
    interfaces, as the server reads it (`Robot.cpp`, `amr_diffdrive.urdf:318-333`).
D7. **A custom sensor's `latency` and `jitter` parameters** become `Sensor::latency`/`jitter`, never
    parameters (plan R9).
D8. **Fallbacks come from the schema registry.** `stage::SchemaAttribute` gains a `fallback` (text)
    so the converter writes only values that differ from it, as `writeStage` does. An addition to
    `io/stage`'s C++ API, not to `schemas/`.
D9. **One robot per file.** The converter takes the first robot (`World::robots`) and notes the
    others; a file with no body is refused ("a world, not a robot"). Lagrange's ground is never
    taken (`ReadOptions::addGround = false`, `ImportOptions::addGround = false`); `armFixed` and
    `placed` (`Robot.cpp:1437-1497`) are not applied, since the converter calls the readers
    directly.

## 8. Work packages

S is up to a day, M 2 to 3 days, L 4 or more, for one agent. Each package works in its own
worktree from `dev`.

### WP4-0: skeleton and interfaces (S). Lands first, alone.

- `viewer/CMakeLists.txt`: install and export `lagrange_viewer_builder` (static, PIC) with its
  headers as `lagrange_viewer::lagrange_viewer_builder`; `ament_export_dependencies` for
  `tinyxml2`, `yaml-cpp`, `lagrange_meshes`, `lagrange_proto` (it links `lagrange_viewer_client`
  publicly, `:68-75`: export that too, or make the client dependency private if only `SceneAdapter`
  needs it; WP4-0 decides and notes it).
- New package `tools/builder_convert/` (`lagrange_builder_convert`, Apache-2.0): `package.xml`
  (`lagrange_viewer`, `lagrange_stage`, `lagrange_importers`, `lagrange_meshes`), `CMakeLists.txt`,
  headers `include/lagrange/builder_convert/FromModel.hpp`:
  `struct ConvertReport { std::vector<std::string> notes, losses; bool composed; enum class Embedded { None, Used, Stale } embedded; };`
  `Expected<builder::Document, std::string> documentFromModel(const Model&, const FromModelOptions&, ConvertReport&);`
  `Expected<std::string, std::string> modelDigest(const Model&);` (moves to `io/stage` in WP4-D),
  and `src/main.cpp` with the CLI of recommendation 1, stubs returning "not yet".
- `viewer/panels/include/lagrange/viewer/BuilderSession.hpp`: `origin` (D1), `Job::Kind::Convert`
  in `viewer/app/src/BuilderHost.hpp:85`.
- `modules/sensors/catalog/sensor_catalog.yaml`: document the `setting`/`scale` keys in its header
  comment (D2); no entries yet.

### WP4-A: links, shapes, inertial, joints from a Model (L). After WP4-0.

Files: `tools/builder_convert/src/FromModel.cpp`, `Primitives.cpp` (D4), `MeshFiles.cpp` (D5);
tests `tools/builder_convert/test/test_from_model.cpp`.
Rules: §3.1 to §3.4 and D3 to D6, D9. Visual/collider pairing into one Shape; the collision choice
inferred; the inertia reordered; `world` root for a fixed base; ball and free joints made fixed
with a note.
Tests: `FromModel.BoxCylinderSphereVisualsAreRecognised` (core meshes in, kinds out),
`.ScaledOrIrregularMeshStaysAMesh`, `.FloatValuesPrintShort` (20, 0.03, 45° limits),
`.InertiaOrderIsTheDocuments`, `.FixedBaseGetsWorldRoot`, `.ActuatedJointsGetRos2Control`,
`.BallJointIsFixedWithANote`, and `.AmrDiffdriveUsdComesBack` (loads
`assets/robots/amr_diffdrive/amr_diffdrive.usda`; link and joint names, masses, limits against
`amr_diffdrive.urdf`, the listed losses only).

### WP4-B: sensors (M). After WP4-0; parallel with A.

Files: `modules/sensors/catalog/sensor_catalog.yaml` (`setting`/`scale` on every built-in param
the URDF reader stores), `modules/sensors/test/test_sensor_catalog.cpp` (the D2 check),
`tools/builder_convert/src/Sensors.cpp` and its test; `viewer/builder/src/SensorCatalog.cpp`
(read the two keys into `CatalogParam`, additive).
Tests: `SensorCatalog.EverySettingIsWhatTheReaderStores` (`lagrange_sensors`, needs
`lagrange_server`'s URDF loader: put it where `test_sensor_catalog` already loads robots);
`Sensors.KindParametersComeBackByPath` (every catalog kind, non-default numbers and texts),
`.LatencyAndJitterAreNotParameters`, `.ImuKeepsTwoNoisesAndListsTheRest`,
`.OverlayOnlySensorsBecomePlaceholders`, `.CameraLensComesBack` (pinhole, fisheye).

### WP4-C: the overlay from schema instances (M). After WP4-0; parallel with A and B.

Files: `io/stage/include/lagrange/stage/Stage.hpp` and `src/Schemas.cpp` (`SchemaAttribute::fallback`,
D8), `tools/builder_convert/src/Overlay.cpp` and its test.
Rules: §3.6. Entries in stage order; prim names over paths; relationships to names; assets
absolute; non-fallback values only; list values as YAML lists.
Tests: for every sample overlay (`amr_diffdrive`, `arm6`, `quad_x500`, `quad_x500_folding`,
`quad_x500_inverted`, `holybro_x500_v2`, `lexus_rx450h`): `lagrange_import URDF OUT.usda --overlay
O.yaml`, convert, `builder::sameOverlay(O.yaml, converted)` after entry order is normalised
(pytest, `tools/builder_convert/test/test_overlay_roundtrip.py`); and the hand-written
`amr_diffdrive_powered.usda` against `amr_diffdrive.overlay.yaml` (its `doc` says they hold the
same data; differences in prim placement are expected and listed, TODO(verify) by running).

### WP4-D: embedded source and atomic USD writes (M). After WP4-0; parallel.

Files: `io/stage/include/lagrange/stage/Stage.hpp` (`WriteOptions::customLayerData`;
`modelDigest`, moved here from WP4-0), `io/stage/src/StageWriter.cpp` (write the dictionary; write
to `<path>.tmp-<pid>` and rename, R12), `server/src/import_main.cpp` (`--embed-source`: embeds
INPUT's text and the overlay's, then loads the written layer, digests it and writes the digest),
`tools/builder_convert/src/Embedded.cpp`.
Tests: `io/stage/test`: `Digest.SameModelSameDigest`, `.FloatNoiseDoesNotChangeIt`,
`.ChangedMassChangesIt`; `StageWriter.WritesAtomically` (a failing export leaves the old file);
pytest `test_embedded_source.py`: export with embed, convert: `embedded: used` and the URDF is
byte-equal to the builder's; change one `physics:mass` in the text: `embedded: stale`, a
reconstruction and a note; `usdcat` re-save (when installed): still `used`.

### WP4-E: SDFormat (M; E2 is S). After WP4-A.

E1: `tools/builder_convert` SDFormat and MJCF inputs (the same `documentFromModel`), notes of
§4's SDFormat row; MJCF's helper bodies (1 g frames for multi-joint bodies) noted as such.
E2 (owner's yes, questions 6 and 7; a server-visible fix, own PR each):
`io/importers/src/SdfImporter.cpp` loads `<mesh>` through `lagrange_meshes` (`resolveMeshUri`
with `model://` paths, `:305-309`; `Description.cpp:531-536` then draws them;
`io/importers/package.xml` gains `lagrange_meshes`), and reads `navsat`, `magnetometer`,
`air_pressure` as GNSS, magnetometer and barometer (`:573-577`), as formats.md §9 already says.
Tests: `test_importers.cpp` cases for both; pytest: `amr_diffdrive.sdf` converts to a document
that `lagrange_server` loads to the same digest as `loadModel(amr_diffdrive.sdf)` (modulo root
placement).

### WP4-F: the builder's Open and Save (M). After WP4-0; finishes after A to D.

Files: `viewer/app/src/BuilderHost.cpp` (`open`: by extension and, for `.xml`, root element, a
`Convert` job; on success `openText` of the converted URDF with the report into Problems; Save
as… `.usda` through `lagrange_import --embed-source` with the overlay; the save-over rule of
recommendation 2), `viewer/app/src/main.cpp` (`--builder-open` takes the new kinds),
the SDL file dialog filters (`BuilderSession.hpp:149`), `viewer/panels/src/BuilderTree.cpp` (the
Files tab shows "from USD: …, Save as… to keep it"), docs.
Tests: `viewer/test/test_builder_files.py` cases: open `amr_diffdrive.usda` headless, screenshot;
open `amr_diffdrive.sdf`; Save as `.usda` and open again (`embedded: used`, the document equal);
open `sedan.usda` (composed): Save refuses to write over it, Save as… works. Screenshots in
`scratch/shots/` as AGENT_BRIEF4 asks.

### WP4-G: acceptance and docs (S). Last.

`docs/guide/builder.md` (Open USD and SDFormat, what is lost), `docs/design/formats.md` §9 (the
viewer's "Open URDF…" sentence that says it runs the importer through the protocol is out of date;
the SDFormat row's "gps/navsat, magnetometer" and mesh claims until E2 lands), RFC 0016's Phases
table row 4 and §10 "Not opened (MVP)" row (`rfcs/0016-robot-builder.md:325`), `CLAUDE.md` §7
(owner's file: proposed lines only, in the PR text).

### WP4-H (optional, D2 of §5.4): Save as SDFormat (M). Only if the owner asks.

`viewer/builder/src/Sdf.cpp` (`writeSdf(const Document&)`), goldens, and a pytest that every
golden loads in `lagrange_server` with the same digest as its URDF.

### Merge order

```
WP4-0 ──┬─ WP4-A ──┬─ WP4-E1 ─┐
        ├─ WP4-B ──┤           ├─ WP4-F ─ WP4-G
        ├─ WP4-C ──┤           │
        └─ WP4-D ──┘ WP4-E2 ───┘  (each needs the owner's yes)
```

Agents: 4 at once after WP4-0 (A, B, C, D), then E1 and F. Hot files: `viewer/CMakeLists.txt`
(WP4-0 only), `Stage.hpp` (C adds `fallback`, D adds `customLayerData` and `modelDigest`: D
rebases on C, or they agree the order), `sensor_catalog.yaml` (B only), `BuilderHost.cpp` (F only).

## 9. Round-trip tests (the matrix)

| # | Input | Path | Oracle |
|---|---|---|---|
| T1 | every `assets/robots/**/*.urdf`, the templates, the builder goldens | import → USD₁ → convert → URDF′ → import → USD₂ | `modelDigest(USD₁) == modelDigest(USD₂)`: conversion is a fixed point after one step. Exceptions only where §4 says (fixed joint names change once, then stay) |
| T2 | the builder goldens | builder export with embed → convert | `embedded: used`, URDF and overlay byte-equal to the builder's own |
| T3 | T2's layer with one value changed | convert | `embedded: stale`, a reconstruction, the note |
| T4 | the sample overlays | URDF + overlay → USD → convert | `sameOverlay` (WP4-C) |
| T5 | a test robot with one sensor of every catalog kind and every built-in type, non-default values | URDF → USD → convert | kind parameters equal as numbers or text; built-in parameters equal where the catalog has a `setting`; the others in the report |
| T6 | the sample USD files (`amr_diffdrive.usda`, `quad_x500.usda`, `sedan_base.usda`, `sedan.usda`, `amr_diffdrive_powered.usda`, `quad_x500_emi.usda`) | convert → builder → Test | loads in `lagrange_server` with no notes beyond the report's; Problems has no Error except the velocity limits until question 1 is settled |
| T7 | `amr_diffdrive.sdf` and an MJCF sample | convert | as T1, against `loadModel` of the source |
| T8 | negative: a world (`assets/worlds/...usda`), a stage with no body, a `.sdf` `<world>` with several models | convert | refused or first robot with notes (D9), never a crash |
| T9 | headless UI | `--builder-open X.usda --screenshot` | the screenshot; the Problems rows |

Tolerances: numbers that were float in USD compare at float precision (as
`viewer/test/test_builder_roundtrip.py` already does at 1e-5 relative for inertia).

## 10. Risks found in the code

R1. **Every revolute and prismatic joint from USD fails validation.** USD carries no velocity limit
(`StageWriter.cpp:670-777`, reader `StageReader.cpp:456-537`) and no effort for passive joints
(`StageWriter.cpp:766-775`); `validate` requires both (`viewer/builder/src/Validation.cpp:98-101`), and Test is
refused on an Error. Question 1.

R2. **Fixed joint names are lost.** The URDF loader keeps no world row for a fixed joint
(`server/src/Robot.cpp:352-353`), so the writer names it `<child>_weld` (`StageWriter.cpp:683`).
An overlay entry or a ROS consumer naming the original fixed joint breaks. Covered by the embedded
source for builder files only.

R3. **Visual primitives and mesh paths do not survive.** Visuals are tessellated by the URDF loader
(`Robot.cpp:502-546`) and written as `Mesh` prims; capsule visuals become cylinders
(`Robot.cpp:544`). D4 and D5 recover what they can.

R4. **Float precision.** Mass, COM, inertia, limits (in degrees), joint frames' `localRot` are
`float` (`StageWriter.cpp:457-461`, `:705-709`); `amr_diffdrive.usda` shows
`translate = (0, 0, 0.10100000066987239)`. Without D3 the builder shows noise digits and every
saved file differs from its source.

R5. **A massless link gains 1 g through USD (to verify).** `rigid_[b]` is true for a welded
child with zero mass (`StageWriter.cpp:119-120`), its mass is written as 0 (`:457`), and the
reader replaces mass ≤ 0 by 1 g with a note (`StageReader.cpp:426-430`). No sample has such a link
(`grep "physics:mass = 0"` over the sample `.usda` finds none); a URDF sensor frame without
`<inertial>` would. Fix: do not author `physics:mass` for a massless welded body, or let the
reader accept 0 on a welded body.

R6. **Inertia order differs.** The world stores xx, yy, zz, xy, xz, yz (`World.hpp:157`); the
document ixx, ixy, ixz, iyy, iyz, izz (`Document.hpp`, `Inertial`). A test pins the reorder
(WP4-A).

R7. **Names are sanitised on the way to USD.** `primName` turns every non-alphanumeric into `_`
(`StageWriter.cpp:93-102`): a URDF link `arm-1` comes back as `arm_1`, and `arm-1` beside `arm_1`
collide. The builder's own names follow `validName` and are safe; opened third-party URDFs are not.

R8. **Composed stages.** `sedan.usda`, `amr_diffdrive_powered.usda` and `quad_x500_emi.usda`
sublayer an imported base and add `over`s. A rebuilt document is flat; writing it over the top
layer would drop the composition, writing it over the base would leave `over`s pointing at prims
that may have been renamed. Recommendation 2 forbids both.

R9. **SDFormat meshes are not loaded.** `readGeometry` sets a mesh's scale but no mesh
(`io/importers/src/SdfImporter.cpp:305-309`); `build()` draws a magenta box for a mesh visual
(`Description.cpp:531-536`) and **skips a mesh collision without a note** (`:432-434`). CLAUDE.md
§7 ("a URDF, SDFormat or MJCF collision mesh collides as the convex hull") and formats.md §9 read
as if SDFormat meshes collided. Opening a Gazebo model with meshes is of little use until WP4-E2.

R10. **SDFormat navsat, magnetometer and air_pressure become custom kinds.** Any type other than
imu, lidar and camera is a custom sensor (`SdfImporter.cpp:573-577`), so these run only if a plugin
implements a kind of that name; formats.md §9's table lists "gps/navsat, magnetometer" as covered,
and RFC 0016's table (`rfcs/0016-robot-builder.md:67`) says GNSS, magnetometer and barometer come
"from an overlay, SDFormat or USD". In the builder they would also collide with the catalog's
overlay-only types of the same name (`magnetometer`).

R11. **A radar written to the overlay may be refused (phase 3, to verify).** The builder writes a
radar as an overlay entry (`viewer/builder/src/Overlay.cpp:30`; catalog `radar: overlay_only`,
`sensor_catalog.yaml:429-433`), but `applyOverlay` makes only IMU, GNSS, magnetometer and
barometer sensors and refuses the rest with "import the robot to USD" (`io/stage/src/Overlay.cpp:345-394`).
If confirmed by a run, a builder robot with a radar fails Test and Export today. Phase 4's USD path
is one fix (`writeStage` writes `RadarAPI_1`, `StageWriter.cpp:411-421`); letting `applyOverlay`
make radars is another (question 10).

R12. **Export is not atomic.** `lagrange_import` writes the output path directly
(`SdfLayer::Export`, `StageWriter.cpp:934`), and Export USD aims it at the file beside the
document (`BuilderHost.cpp:889`). TODO(verify) whether `SdfLayer::Export` writes through a
temporary file; if not, a failing export leaves a truncated `.usda`. Saving over a `.usda` (the
recommendation) makes this matter; WP4-D writes to a temporary file and renames.

R13. **`customParameters` reads no attributes** (`server/src/Robot.cpp:820-848`), so a kind's
`<path>/@<name>` parameters (the builder keeps them, `Urdf.hpp` header) do nothing in a run and
cannot be a USD property name (`@`). The converter cannot bring them back; a note says so.

R14. **The SDFormat `<lagrange>` power block is read in the server only** (`applyLagrangeBlock`,
`server/src/Robot.cpp:1280`, called from `loadModel` for SDFormat, `:1508-1515`). The
converter (B3) does not link the server: either move the function into `lagrange_importers` or
note "the model's `<lagrange>` block was not read".

R15. **Doc drift found while reading.** `Stage.hpp`'s comment says sensors are read "with
SensorAPI_1 and LidarAPI_1 or ImuAPI_1" (all eight types are read, `StageReader.cpp:908-1155`);
`import_main.cpp:10-11` says the same. formats.md §9 says the viewer's "Open URDF…" runs the
importer through the protocol (RFC 0016 opens files locally). WP4-G fixes the docs.

R16. **`loadModel` changes a model on the way in.** `armFixed` fixes the base of a 3-joint chain
read from USD or SDFormat and `placed` moves it by `--spawn` (`Robot.cpp:1437-1499`). The converter
must call the readers directly (D9), or the builder would show a fixed base the file does not have.

## 11. Open questions for the owner

1. **Joint velocity limits and passive effort in USD.** Options: (a) add `joint:velocityLimit` and
   `joint:effortLimit` (fallback 0 = not given) to `JointDynamicsAPI_1`, written by `writeStage`
   for every moving joint and read back into `World::Joints` (an addition under RFC 0007: needs the
   §9 note and two approvals, migration not needed); (b) leave USD as it is and let documents from
   USD carry no limit with a Warning (only if urdfdom accepts a `<limit>` without `velocity`,
   TODO(verify)); (c) rely on the embedded source for builder files and make the user type limits
   for others. Proposed: (a).
2. **Embed the builder's URDF and overlay in exported USD** (`customLayerData`, §5.3 C2)? Proposed:
   yes, for builder exports; `lagrange_import --embed-source` available to anyone.
3. **Save over a `.usda`**: only over layers the builder or `lagrange_import` wrote and that have
   no composition arcs, everything else Save as…? Proposed: yes.
4. **Composed stages**: open flattened as a new document (proposed), or edit the base layer and keep
   the `over`s (needs a check that every `over` still finds its prim)?
5. **The converter's home**: a new package `lagrange_builder_convert` that links the exported
   builder library (proposed, one URDF writer), or `lagrange export urdf` in `lagrange_server` with
   its own writer?
6. **Load SDFormat meshes** in `importSdf` through `lagrange_meshes` (R9)? It changes what
   `lagrange_server` loads from SDFormat (meshes collide and draw instead of being skipped or
   magenta). Proposed: yes, its own PR.
7. **Read SDFormat navsat, magnetometer, air_pressure as GNSS, magnetometer, barometer** (R10)?
   Changes runs of SDFormat robots that have them (today custom kinds that do nothing without a
   plugin). Proposed: yes, its own PR.
8. **Save as SDFormat** (§5.4 D2): wanted now, later, or never? Proposed: later, on request.
9. **Inline meshes as OBJ files** (D5) beside the opened document, or keep inline meshes out of the
   builder (draw them, but refuse to save until the user points each at a file)? Proposed: OBJ.
10. **Radar in the overlay** (R11, once confirmed): let `applyOverlay` make radars too (its own PR),
    or keep radar out of URDF robots and point users at USD? Proposed: let the overlay make radars.
11. **Worlds**: refuse files with several robots, or open the first and note the rest (D9,
    proposed)?

## 12. Draft: RFC 0016 Addendum B (Status: Draft)

To paste after Addendum A when the owner accepts it; edit the answers to the owner's.

> ### Addendum B (2026-11-15, Status: Draft): phase 4, opening USD and SDFormat
>
> Phase 4 opens USD (`.usd`, `.usda`, `.usdc`), SDFormat and MJCF files in the builder and saves
> USD, without linking OpenUSD, libsdformat or MuJoCo into the viewer:
>
> - **The document stays URDF plus the overlay.** Opening another format converts it, as a xacro
>   is expanded (§10): a polled child process, `lagrange_builder_convert`, loads the file with the
>   simulator's readers (`stage::loadStage`, `importers::importSdf`, `importers::importMjcf`),
>   rebuilds the builder's document from the world tables and writes it with the builder's own URDF
>   and overlay writers. The result opens as a new, unsaved document; what could not come back is
>   listed in Problems. The package links the builder library (exported by `lagrange_viewer`),
>   `lagrange_stage` and `lagrange_importers`, not the server.
> - **Saving USD** goes through `lagrange_import` (§10's Export USD), written atomically. The
>   builder's export embeds its URDF and overlay in the layer's `customLayerData`
>   (`lagrangeBuilder`) with a digest of what the layer loads to; opening a layer whose digest
>   still matches returns the embedded document exactly, and a layer edited elsewhere is rebuilt
>   from the world tables with a note. No schema change: layer metadata, as `lagrangeGroundPlane`.
> - A `.usda` the builder or `lagrange_import` wrote, without composition arcs, may be saved over;
>   any other USD (hand-written, sublayered) is opened flattened and saved only as a new file.
> - **SDFormat is opened, not saved**, in phase 4. `importSdf` loads mesh geometry through
>   `lagrange_meshes` and reads `navsat`, `magnetometer` and `air_pressure` as Lagrange's GNSS,
>   magnetometer and barometer (each a separate PR, as they change what the server loads).
> - The sensor catalog names, for each built-in parameter, the world setting the URDF reader
>   stores (`setting:`), checked by `test_sensor_catalog`, so built-in sensors come back from USD
>   by the catalog; a kind's parameters come back from `CustomSensorAPI_1` by their paths.
> - Joint velocity and passive effort limits, which USD does not carry today, are added to
>   `JointDynamicsAPI_1` as `joint:velocityLimit` and `joint:effortLimit` (additive under
>   RFC 0007, two approvals) **or** documents from USD carry no limit with a Warning: the owner's
>   choice (plan question 1).
> - Tests: conversion is a fixed point after one step (import → convert → import gives the same
>   digest) for every sample robot; the builder's own exports come back byte for byte; every
>   sample overlay comes back as the same overlay; one sensor of every catalog kind comes back
>   with its parameters.
>
> Phases table, row 4 becomes: "USD open (by conversion) and save (through `lagrange_import`, with
> the source embedded), SDFormat and MJCF open | `tools/builder_convert` (new package),
> `io/stage` (atomic write, `customLayerData`, schema fallbacks, digest), `server/src/import_main.cpp`
> (`--embed-source`), `io/importers` (SDFormat meshes and sensors, separate PRs),
> `modules/sensors/catalog` (`setting:`); `schemas/joint` only if the owner picks the schema
> addition".
