# 03 · Domains & profile: SPEC

## Purpose
This is the core differentiator. Users turn each physics domain on or off, choose its fidelity
and step size, or choose a substitute. They see how modules are wired (with unit checks) and how
they are scheduled, and they edit the YAML profile, which is validated against
`profiles/schema.json`.

## Dock layout
```
left 520 px  : "Domains" | "Modules · N"
center flex  : top "Wiring" | "Unit checks" ; bottom 268 px "Schedule" | "Coupling"
right 420 px : "Profile" | "Diff · N" | "Schema"
```

## Panels
### Domains (table built with `ImGui::BeginTable`, 6 columns)
| col | width | content |
|---|---|---|
| color bar | 4 | domain color, or accent when the row has pending changes |
| domain | 150 | name (SemiBold) and a one-line description in t3 |
| on | 30 | `Switch` |
| mode | flex | when on: a fidelity or backend combo (e.g. `PhysX 5 · CPU`, `averaged · FMU`). When off: a substitute combo shown as `→ zero force`, `→ zero dose`… |
| step | 70 | a dt field with unit (µs/ms) or a read-only `frame` / `varies` |
| cost | 62 | mono estimated cost per 1 ms of sim (`gpu` suffix for GPU work) |

- The selected row expands a **substitute picker** below the table with 3 cards (ideal/zero,
  constant, replay from MCAP with a source picker) and the affected consumer signals, for
  example "`rigid_body.link/*/ext_wrench` [N, N·m] will read from the substitute".
- "Additional modules" section: comms, traffic, plus robot-class modules (vehicle_dynamics,
  rotor_aero, hydrodynamics) greyed out with "not used by <robot>".
- Footer: the estimated total CPU per 1 ms of sim, the GPU per sensor frame, and whether it fits real time.

### Wiring (node graph; imgui-node-editor or equivalent, TODO(verify) license)
- One node per enabled module or substitute. Substitutes have a dashed border and an accent title.
- Node body: inputs and outputs listed as `in …` / `out …` in mono, plus dt.
- Edges are labeled `symbol · unit` (for example `τ · N·m`). Stiff coupled pairs get a warn pill
  "stiff · implicit".
- The "preview — not applied" pill shows while the profile has pending edits. The graph comes
  from `ProfileValidate` and shows the wiring the server *would* build.
- The Unit checks tab lists every wire as source signal [unit] → sink [unit] ✓/✗. A mismatch is
  an error and blocks Apply.

### Schedule
A custom draw of a 10 ms window. Rows: electrical (every 100 µs, drawn as ticks), rigid_body
(blocks every 1 ms), imu, emi·comms ("next at 10 ms"), camera ("GPU · next frame at 33.3 ms"),
thermal ("next at 100 ms"). Sync barriers every 1 ms.

### Profile (YAML editor)
- A text editor with line numbers and syntax colors (keys blue-ish, strings green, numbers
  yellow, booleans accent, comments t3). Use ImGuiColorTextEdit or equivalent
  (TODO(verify) license and maintenance).
- Diff lines: removed lines on a red tint with `−`, added lines on a green tint.
- The header shows the file path and a validation pill (`✓ valid · schema v1` / `✗ 2 errors`).
- Footer: a warning "Applying rebuilds the wiring and restarts the run at t = 0", then Revert,
  Save as…, Save (Ctrl S) and **Apply and restart** (primary).
- Two-way binding: editing the table updates the YAML, and editing the YAML updates the table
  after validation.

## States
- Invalid YAML: show the error inline at the line and disable Apply.
- A unit conflict: the domain chip turns red, the Unit checks tab shows the conflict, and Apply
  is disabled.

## Mock server must provide
`Profile`, `ProfileValidate` (returning wiring and unit checks), and cost estimates. Apply can be
simulated as a restart.

## Acceptance
- Toggling Aerodynamics off generates YAML that validates against `profiles/schema.json`.
- The graph shows the substitute node within 200 ms.
