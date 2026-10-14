# 02 · Scene workspace: SPEC (default workspace)

## Purpose
This is the everyday view: the 3D world, the USD hierarchy, one prim's properties with live
values, the run timeline and the console.

## Dock layout (DockBuilder)
```
root
├─ left   280 px  : "Scene" | "Layers"
├─ right  340 px  : "Inspector" | "History"
└─ center
   ├─ top (flex)  : "Viewport" | "camera_front" | "lidar_front · BEV"   (sensor views as extra tabs)
   └─ bottom 196px: split 62 % "Timeline" | "Bookmarks"  /  38 % "Console" | "Problems"
```

## Panels
### Scene (tree)
- A search field ("Filter prims"; fuzzy over name and path) and a "Schemas" filter menu (filter
  by applied API: Electrical, Thermal, EMI, Sensor…).
- Tree rows: caret, optional type icon, name, type in t3, and on the right **domain badges**
  (one dot per applied physics schema, in domain colors).
- Layers tab: USD layer stack (stage, robot, environment, overrides) with mute and edit-target.
- Footer: "N prims · N selected · N with physics schemas".
- Data: `SceneSnapshot` and `SceneDelta`. Clicking a prim selects it globally.

### Viewport
Rendered by the Vulkan renderer into an offscreen image and shown with `ImGui::Image`.
- Overlay, top left: camera mode, shading, frame time, draw count, tier.
- Gizmo tool column, left: Select (Q), Move (W), Rotate (E), Scale (R), Measure (M).
- Overlay toggles, top right: grid, frames/axes, contacts and forces, sensor frustums, Nav2
  path, field overlays, electrical overlay (current as color on wires and motors), weather effects.
- Bottom left: axis triad and frame info (ENU · Z-up · m, camera distance).
- Bottom right: cursor ground position and a Focus (F) hint.
- In-scene labels: name tags for robots (name and speed); the selected prim's label uses the
  accent color.
- Header menus: shading (Lit / Unlit / Wireframe / Collision / Thermal false-color), camera
  (Perspective / Ortho / follow robot), overlays.
- Floating origin: positions arrive as `double` and are rendered relative to the camera.

### Inspector
- Header: icon, name, type, mono path, the **applied schema chips** (domain-colored), "+ schema"
  and a lock button.
- Sections, collapsible:
  1. **Live**: rows of label, mono value with unit, and a 120×22 sparkline (last 10 s). Signals
     come from `SignalSubscribe` for this prim. "Plot in Signals" sends them to the Signals
     workspace.
  2. **Transform**: Vec3 fields for translate and rotate (local, m, deg).
  3. One section per applied schema (for example `ElectricalMotorAPI_v1`): `PropertyRow`s with
     units, asset links (FMU path with ✓ FMI 3.0 status), and relationship links (`→ /amr/power/dc_48v`).
- Edits are local until **Apply**. Edited fields get an accent border, and the footer shows
  "N edits · applies on next run", Revert and Apply. Apply sends a `Command{set_attribute}`.

### Timeline
- Transport: jump to start, play/pause, step, the time readout and a LIVE pill; step size,
  speed and window segmented control (5 s / 10 s / 60 s / all).
- Track (custom ImDrawList): time ruler; a "run" row with the buffered region (bg3) and a
  dashed "not simulated yet" region; event lanes per domain (contacts, electrical, comms…) built
  from `Event`; an accent playhead with a triangle head. Dragging the playhead seeks inside the
  buffer (pauses live).

### Console
Monospace log: time, level (INFO info-blue / WARN warn / ERROR error), source, message. Level
filter segmented control; the Problems tab collects warnings and errors with links to their source.

## States
- No stage: the viewport shows "Open a stage or import a robot" with buttons.
- A selected prim with no physics schemas: the Inspector shows only Transform and the generic USD attributes.

## Mock server must provide
A snapshot of the sample AMR (the 33 prims shown), deltas at 60 Hz for the moving robot, the 4
live signals per motor at 10 kHz, events for slip, regen and Wi-Fi loss, and log lines.

## Acceptance
- 60 fps with this scene and 4 sparklines on an RTX-class GPU.
- Selecting `left_drive_motor` in the tree highlights the wheel in the viewport and fills the
  Inspector within one frame.
