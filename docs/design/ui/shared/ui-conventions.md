# Lagrange viewer: shared UI conventions

This applies to every screen. Each screen's own `SPEC.md` covers only what is specific to it.

The viewer is a native C++20 app built with **Vulkan + Dear ImGui (docking branch) + ImPlot**,
using **SDL3 or GLFW** for the window. Nothing in it uses web technology. The `reference.html`
in each zip is only a picture of the layout. Do not port the HTML or CSS; rebuild every screen
with ImGui widgets that use `lagrange_theme.hpp`.

## 1. App chrome (identical on every connected screen)

From top to bottom, at a 1600×1000 window:

| Band | Height | ImGui construct | Contents |
|---|---|---|---|
| Menu bar | 28 | `BeginMainMenuBar` | L mark + "Lagrange", File, Edit, View, Simulation, Tools, Window, Help; on the right: the stage, robot and profile in use, plus a `Ctrl K` hint |
| Workspace bar | 40 | full-width window, no title, `NoDocking` | 10 workspace tabs (icon + label, accent underline on the active one); sim controls on the right |
| Domain strip | 32 | full-width window, no title | one chip per physics domain, plus a context pill and an action on the right |
| Dock space | rest | `DockSpaceOverViewport` with 4 px gaps | the workspace's panel layout |
| Status bar | 24 | full-width window at the bottom | connection, server, stream health, GPU tier, viewer frame time, ROS 2 domain |

### Workspace bar
- Tabs in this order: Scene · Domains · Electrical · Signals · Sensors · Environment · Replay ·
  ROS 2 · World · Performance. Each workspace is a **saved dock layout** (an ImGui `.ini`
  section per workspace). Switching workspace loads its layout and never destroys panel state.
- Sim controls: Reset, Play/Pause (primary button, accent fill), Step, and readouts for
  `sim t`, `RTF x / target` and `lockstep`, then a Record button whose red dot means recording.
- In Replay the controls become Previous event / Play / Next event, and the readouts become
  `t / duration`, speed and loop range, with a blue `REPLAY` pill.

### Domain strip (core feature)
- One chip per domain, in this fixed order: Rigid body, Electrical, Thermal, Aero, Weather, EMI,
  Radiation, Sensors. Each shows the domain color dot, the name and a mono status
  (`backend · dt`, or `→ substitute`).
- Chip states:
  - **on**: filled dot, solid border.
  - **substitute**: dashed border, hollow dot, `→ ideal|constant|replay|zero|clear`.
  - **off**: dashed border, dimmed.
  - **pending**: accent border with `· pending`.
  - **error**: red border, for example a unit conflict.
- Clicking a chip opens the Domains workspace with that domain selected. Right-clicking gives a
  quick toggle menu, which still goes through the "apply and restart" confirmation.

### Status bar
Left to right:
- connection dot and state (Connected / Connecting / Disconnected / Replay)
- server name, version, host and transport (`shm` when local)
- signals / deltas per second / dropped
- spacer
- GPU name and tier
- viewer frame time and fps
- ROS 2 distro, domain and RMW

## 2. Panels
- A tab bar 28 px tall; the selected tab gets a 2 px accent overline (`TabBarOverlineSize`).
- Tools sit on the right of the tab bar: `ImGui::SameLine` aligned right, small 22 px buttons.
- Section headers are 26 px tall on `bg2`, with a caret, an optional domain dot, a title and a
  mono right-aligned meta string.
- Key/value grid: a 112 px label column, with value fields 22 px tall on `bg0` with a 1 px border.
  The unit sits right-aligned inside the field in `t3`. An edited but unapplied field gets the
  `accentBorder` border.
- Tables use `ImGuiTableFlags_RowBg | BordersInnerH | ScrollY`. Numeric columns are right-aligned
  in mono. The selected row gets an `accentWeak` fill and a 2 px accent bar on the left.
- Tree rows are 22 px tall with 14 px indent. On the right of each row, small domain dots show
  which physics schemas that prim has applied.

## 3. Interaction and shortcuts
| Key | Action |
|---|---|
| Space | Play / pause |
| . | Step one scheduler tick |
| Ctrl R | Start / stop recording |
| Ctrl K | Command palette (fuzzy search over prims, signals, topics, commands) |
| Ctrl 1…0 | Switch workspace 1–10 |
| F | Focus the viewport on the selection |
| Q W E R | Select / move / rotate / scale gizmo |
| Ctrl S | Save profile |
| Ctrl O / Ctrl I | Open stage / import robot |
| Esc | Clear selection, close popups |

Selection is global. Choosing a prim in any panel (tree, schematic, map, table) selects it
everywhere. The selection is stored in the viewer, not on the server.

## 4. States every panel must handle
1. **Disconnected**: keep the last data greyed out, show a "Disconnected — reconnecting (n)" pill
   in the status bar, and disable sim controls. Never clear panels.
2. **Loading**: a skeleton row or a spinner in the panel body. Panels show no fake data.
3. **Empty**: one line of guidance, for example "No signals plotted — drag one from the
   Signals list".
4. **Domain disabled**: panels that depend on a disabled domain show its substitute clearly,
   for example "Electrical is off → ideal torque source. Values shown are commanded torque."
5. **Error**: an inline red pill with text, and the full message in Console → Problems.

## 5. Protocol bindings (names follow `proto/`; final field names come from RFC 0003)
| Message | Direction | Used for |
|---|---|---|
| `Hello` / `ServerInfo` | ↔ | protocol version check, server tier, transport negotiation (shm) |
| `SceneSnapshot` | server → viewer | full prim tree, schemas, transforms (once, and on resync) |
| `SceneDelta` | server → viewer | transform and attribute changes; ~60 Hz cap for the viewer |
| `SignalCatalog` | server → viewer | signal names, units, domains, rates |
| `SignalSubscribe` | viewer → server | which signals to stream, at what decimation |
| `SignalChunk` | server → viewer | batched samples (shm ring for local, compressed for remote) |
| `Event` | server → viewer | brownout, slip, packet loss, trigger fired, etc. |
| `Profile` / `ProfileValidate` / `ProfileApply` | ↔ | domain toggles, YAML text, validation result |
| `Command` | viewer → server | play, pause, step, reset, seek (replay), record, set attribute |
| `FieldSlice` | server → viewer | 2D slices or downsampled VDB bricks for overlays |
| `SensorFrame` | server → viewer | images and point clouds (shm fast path) |
| `Stats` | server → viewer | per-module timings, RTF, memory, stream health |

The viewer never links simulation modules (CLAUDE.md invariant 14). Everything shown arrives
through these messages.

## 6. v0.1 scope rule
In v0.1 everything runs against `lagrange_mock_server`, which must emit realistic data for
every screen; the canvas shows the target data. Screens that depend on later releases (for
example validation or huge worlds) are still built in v0.1 against mock data, so the protocol
and panels are proven early.
