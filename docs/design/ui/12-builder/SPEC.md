# 12 · Builder workspace: SPEC (draft, RFC 0016)

Status: **draft for the owner**, with `rfcs/0016-robot-builder.md` (Draft). Nothing here is
built yet. Where this screen needs something the design system does not have, it says so under
"New in this screen" instead of improvising silently.

## Purpose
Build a robot inside the viewer: links with simple shapes, joints between them, sensors from the
catalog, then test it in a simulator with one click and save it as URDF. The workspace is a
separate **mode** so that building costs almost nothing: no simulation steps, no sensor
rendering, no realistic pass, only the robot being edited drawn Lit. It works with no server.

## Modes (the workspace bar's right side)
The sim controls of the other workspaces are replaced by the builder's own, as Replay replaces
them with its own (`ui-conventions.md` §1):

| Control | Widget | Behaviour |
|---|---|---|
| Undo, Redo | `.btn.ic` (Ctrl Z, Ctrl Shift Z / Ctrl Y) | redo is `.dis` when the stack top is reached |
| Document readout | `.ro`: `doc`, mono file name, warn `● unsaved · N` (N edits since the last save) | the name is the robot name until first saved |
| **Build** | `.btn.on` | the current mode |
| **Test · empty world ▾** | `.btn.pri` with a menu | the primary action. Menu: Empty world · A sample world… · A world file… · *Into the connected run* (RFC 0016 §7.2: enabled when connected to a server that imports robots; the robot goes into that run as URDF text, passive when the run already has a robot, and Back removes it; greyed with "Connect to a server first" otherwise) |
| **Save** | `.btn` (Ctrl S) | Save, Save as…, Export USD… (runs `lagrange_import`), Check loads… |

While testing, these become: **Back to Build** (`.btn`, Esc), the Scene workspace's Play/Pause,
Step and readouts (`sim t`, `RTF`), and an info pill **TEST · rover_mk1** in place of the
`REPLAY` pill. Back stops the test and returns here with the document exactly as it was.

## Strip under the workspace bar
The domain strip becomes the **Robot** strip in this workspace (label "Robot"):
- `DomainChip`-shaped chips, neutral: `N links` (rigid-body dot, meta `1 root`), `N joints`
  (meta by type), `N sensors` (sensors dot, meta `types · kinds`), `mass` (mono total, kg), `COM`
  (mono x · y · z, m, in the root link's frame). The dots are the rigid-body and sensors domain
  colours: they say which domain the parts belong to, nothing else.
- Right: an info pill `Build · no simulation · sensors not rendered · Lit` (always, so the user
  knows why nothing moves), the validity pill (ok `URDF valid · N hints`, or err
  `N problems` which opens Problems), and a ghost button **Check loads…** (runs `lagrange check`
  on the machine when installed; hidden otherwise).

## Dock layout (DockBuilder)
```
root
├─ left   270 px : top "Robot" | "Files"            (flex)
│                  bottom "Add" | "Templates"        212 px
├─ right  350 px : "Properties" | "History · N"
└─ center
   ├─ top (flex)  : "Builder view"
   └─ bottom 220 px: split 55 % "Joints · N moving" | "Poses"
                    /  45 % "Sensor catalog · N" | "Problems · N"
```
The dock space name carries a layout version like the others (`lagrange-dock-builder-1`).

## Panels
### Robot (tree)
- Search "Filter links, joints, sensors" and an **Add** button (menu: Link → Box / Cylinder /
  Sphere / Mesh…, Sensor → opens the catalog with the selected link as target, Joint between… for
  re-parenting).
- Rows (`TreeRow`): the robot at the top, then links nested by their joints. A child link's
  row shows its joint type and shape in t3 (`continuous · cyl`); the joint itself is selected by
  clicking that type text, or from the Joints panel. Sensor rows have the sensor icon and their
  type or kind in t3. Badges: rigid-body dot on links, sensors dot on sensors.
- Drag a row onto another link to re-parent it (its joint keeps its type; the pose is kept in
  world space). Delete removes the row and everything under it (one undo step). F2 renames.
- Footer: `N links · N joints · N sensors · N selected`.
- Files tab: the document's files: the URDF, the overlay YAML when one is needed, meshes, and
  the autosave, each with its path in mono.

### Add
- "a link on `<selected link>`, by a joint", four tiles 56×54: **Box**, **Cylinder**,
  **Sphere**, **Mesh…** (file dialog: STL, OBJ, DAE, glTF; RFC 0016 phase 2, disabled with a
  tooltip until then). Clicking adds a link with that shape at the selected link's origin,
  offset so it does not overlap, and selects it.
- `joint` (fixed, revolute, continuous, prismatic) and `material` (density presets, mono
  density with unit; "custom" opens a number field): the defaults for the next tile.
- Templates tab: a few starting robots (differential drive base, 3-joint arm, pan-tilt head),
  each a builder document; RFC 0016 open question 11.

### Builder view
Drawn by the viewer's own Vulkan renderer on the Lit pipeline only (never Realistic), from the
builder's document, not from the protocol. Shading menu shows Lit only; the button is a label.
- Overlay, top left: `Build · Perspective · Lit`, frame time, draw count, `no simulation`.
- Tool column, left: Select (Q), Move (W), Rotate (E), **Joint axis (A)**, Measure (M). Move
  and Rotate act on the selected link's joint origin, or the selected sensor's mount. Joint
  axis shows a ring gizmo on the selected joint: dragging snaps the axis to x, y, z of the
  parent or to any direction with Shift.
- Toggles, top right: grid, frames and joint axes, collision shapes, centre of mass and inertia
  (the COM marker and, per link, its equivalent inertia box), sensor frustums and rays, names.
- Bottom left: triad and frame (`base_link · Z-up · m`, grid spacing). Bottom right: snap
  (5 mm, 5° by default; Ctrl toggles) and Focus F.
- The selection draws in accent (one thing per region): a sensor's ray or frustum to its range,
  a link's outline, a joint's axis. Other sensors draw thin in the sensors colour at 0.45 alpha.
- Labels: the selected item, and joints being previewed (name and value).

### Joints
- A grid: joint (mono), type (`Pill`), **preview position** slider, value field (mono, unit in
  t3), limits (mono t2, or "none").
- Revolute and prismatic sliders run from lower to upper limit, with a tick at zero; the fill
  goes from zero to the knob. Continuous joints have a dashed track that wraps at ±180°.
  Dragging poses the robot by forward kinematics only; the document's joint origins do not
  change and the preview is not an undo step. Mimic joints follow their leader and show no knob.
- Tools: `Segmented` deg | rad (display only; files hold rad and m), **Zero all**.
- Footer: fixed joints not shown, and "Preview only: no physics".
- Poses tab: named sets of preview positions (saved in the document, useful for Test's start
  pose later; RFC 0016 open question 10).

### Sensor catalog
- Search, and `Segmented` All | URDF | Overlay · N (sensors that need the overlay file:
  GNSS, magnetometer, barometer, radar; RFC 0016 phase 3).
- Groups (`SectionHeader` 24 px, count as meta): Built-in types (camera, lidar, imu), Range and
  contact, Vision, Air and weather, Power, Force and motion, Radio and vehicle; then any kinds
  from installed plugins' catalog files, then "Other kind…" (a free key/value form).
- Rows: kind name (mono, 118 px), one-line summary (t3, ellipsis), and on the right `in use N`
  or, on hover, **Add** (adds it to the selected link at the link's origin and selects it).
  Dragging a row onto a link in the tree or the view adds it there.
- Data: the catalog file (RFC 0016 §5), read at start. Nothing comes from the server.

### Problems
- One row per finding of the builder's own validation (RFC 0016 §8): level (error / warn /
  hint), the item (mono, a link that selects it), the message. Test is refused while there is an
  error. After a Test, the server's loader messages and `lagrange check` output are listed here
  too, under their own heading.

### Properties (link, joint or sensor)
Header: icon (accent), name (editable, 14 px semibold), what it is, Focus; the mono path through
the tree; pills for type or kind and its documentation section.

Edits apply to the document at once (there is no Apply here: nothing runs). A field whose value
differs from its default has the `accentBorder` and its default in the unit slot
(`m · default 12`); the footer says how many and offers **Reset to defaults**. Every change is
one undo step; typing in a field is one step when it is committed (Enter, Tab or leaving it).

- **Link**: Shapes (one `SectionHeader` per shape: kind, size fields with units, origin
  `Vec3Field`s, visual and collision switches, colour), Mass (material or density, the computed
  mass, a mass override, the computed inertia tensor read-only with an override switch that makes
  it editable; COM in the link frame), Surface (friction μ, later the material presets of
  `sensor_material`, `visual_material`, `thermal_material`), Kept as written (the link's
  `<gazebo>` children this builder does not edit, read-only XML, mono).
- **Joint**: type (`Segmented`), parent and child (dropdowns of links; a change re-parents),
  origin (`Vec3Field` translate and rotate), axis (`Vec3Field`, normalised on write; the Joint
  axis tool sets it), limits (lower, upper, effort, velocity: required for revolute and
  prismatic, no invented defaults: empty fields are a Problem), dynamics (damping, friction),
  mimic (joint, multiplier, offset).
- **Sensor**: Mount (link, translate, rotate), Timing (`update_rate`, `latency`, `jitter`),
  one section named by the type or kind with the catalog's parameters: label = the parameter's
  path in mono, field with the catalog's unit and default, min and max checked as typed; enum
  parameters are dropdowns, text parameters plain fields, lists a mono text field. Then the
  catalog's description (t3, 11 px), Output (the signals and ROS topic the sensor will have,
  read-only, from the catalog), and Faults (collapsed; the sensor-fault parameters).
- History tab: the undo stack, newest at the top, one row per step ("Set range/max 12 → 20 m",
  "Add link caster"); clicking a row goes back to it.

## States
1. **Empty** (new document): the tree shows only the robot row and one link `base_link`
   (Box 0.3 × 0.2 × 0.1 m, so there is something to click); the view centres on it; Properties
   shows `base_link`; the Joints panel says "No moving joints yet: add a link with a revolute,
   continuous or prismatic joint"; the catalog is usable.
2. **Editing** (the reference image).
3. **Invalid**: the strip's pill is err `N problems`; Problems comes to the front once (not on
   every edit); the items with an error get a red dot badge with text in their tree row tooltip;
   **Test** is disabled with the tooltip "Fix N problems first". Save still works (a file can
   be saved while unfinished), with a warn line in the save dialog.
4. **Testing**: the workspace switches to a Test layout: the center top is the normal
   server-fed Viewport (any shading the tier allows), Joints shows the joints' live positions
   read-only (sliders become meters, until a joint-command message exists: RFC 0016 §7.3), the
   bottom right shows **Drive** (the ROS 2 topics this robot takes, `/cmd_vel` when the server
   drives it, `/joint_commands` for joint control, each with a copyable command line) and Console
   with the test server's log; Properties is read-only. The status bar shows the test server.
   If the server refuses the robot, the view does not change: Problems comes forward with the
   loader's message and Build stays active.
5. **Disconnected / no server**: Build and Save work fully. Test needs `lagrange_server` on
   this machine; if it is not installed, Test is disabled with "lagrange_server is not installed"
   (the Launcher's wording), and *Into the connected run* is the only target when a server is
   connected and imports robots.
6. **Error** (cannot read a file, cannot write): an err pill in the panel concerned, full
   message in Problems.

The panel states of `ui-conventions.md` §4 apply: "domain disabled" does not occur (no
simulation runs in Build).

## Cost in Build mode (why it is light)
- No `SubscribeSensor`, `SubscribeField`, `SetWorldEnabled` or ROS graph subscriptions while
  this workspace is in front (the viewer unsubscribes when entering and restores on leaving).
- The Scene viewport and sensor views are not drawn (hidden ImGui windows submit nothing); the
  Realistic pass is never used here; only the builder view is drawn, Lit, a few hundred
  triangles per primitive.
- A connected run is left alone (not paused) unless the user ticks **Pause the run while
  building** in the View menu.

## New in this screen (proposed, not yet in the design system)
| Element | Proposal |
|---|---|
| `Builder` workspace tab | an 11th tab after Performance, separated by a 1 px `ln` divider; shortcut **Ctrl B** (Ctrl 1…0 are taken); icon: a wrench, 16 px stroke (path in `reference.html`) |
| `JointSlider` widget | track 4 px `bg4`, fill `#3A4F6E` from zero, zero tick 1 × 12 px `ln2`, knob 14 px `t1` with a 2 px `bg1` ring and a 1 px `ln2` outline; dashed track for continuous joints. To be added to `00-design-system` |
| `ShapeTile` widget | 56 × 54, `bg2`, 1 px `ln2`, 3 px radius, a 20 px stroke icon over an 11 px label |
| Icons | Builder (wrench), Undo, Redo, Save, Joint axis tool, Collision shapes, COM, Names, Box, Cylinder, Sphere, Mesh: SVG paths in `reference.html`, to copy into `Icons.cpp` |
| Robot strip | the domain strip's band with neutral chips (no domain colour on the chip itself) |
| Launcher tile | a fourth action tile **Build a robot** (Ctrl B) on `01-launcher`, opening this workspace without a server: a change to the owner's 01 design, to be made only when accepted |

## Data
None from the protocol in Build. The document is the builder's (RFC 0016 §3); the catalog is a
file (§5). In Test the normal protocol messages feed the Test layout. The numbers in the
reference image are an illustrative sample document, `rover_mk1` (README.md lists its parts;
mass and COM are computed from them); the rangefinder's defaults are those of
`docs/design/domains/sensors.md` §9 (`rangefinder`), and the default update rate of 10 Hz is the
URDF reader's (`server/src/Robot.cpp`). In the implementation these numbers come from a sample
document file, never from panel code.

## Acceptance
- Build mode, the sample document, 1600×1000 on the software Vulkan device (llvmpipe, Tier 0):
  frame time within the viewer's 16.6 ms budget, measured headless; with a server connected,
  no sensor or field traffic arrives while this workspace is in front (client counters).
- Headless scripted test (`--workspace builder --input …`, as `test_viewer_input.py`): add a box,
  make its joint revolute, give limits, add a rangefinder from the catalog, change `range/max`,
  undo, redo, Test (own server), see the robot in the Test view, Back, the document unchanged,
  Save, the file equal to a golden URDF.
- A headless screenshot compared with `reference.png`, every remaining difference listed.
