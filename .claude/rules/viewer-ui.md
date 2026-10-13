---
paths:
  - "viewer/**"
  - "mock_server/**"
  - "docs/design/ui/**"
  - "docs/design/viewer.md"
---

# Viewer UI: follow the design in `docs/design/ui/`

The project owner designed every viewer screen. That design is binding for all viewer UI work.
Do not invent layouts, colours, fonts or widgets when the design covers them.

## Before writing or changing any viewer UI

1. Read `docs/design/ui/shared/ui-conventions.md` (app chrome, workspace bar, domain strip,
   status bar, panel rules, shortcuts, panel states).
2. Read `docs/design/ui/shared/design-tokens.json` and `shared/lagrange_theme.hpp`. The JSON is
   the source of truth; the header maps it onto Dear ImGui and ImPlot.
3. Read the `SPEC.md` of the screen being worked on and look at its `reference.png`
   (rendered at 2x for a 1600x1000 logical window). `reference.html` gives exact text, numbers
   and colours when the image is not enough.

| Folder | Screen |
|---|---|
| `00-design-system` | theme, fonts, shared widgets (build this first) |
| `01-launcher` | first window when not connected |
| `02-scene` | default workspace: viewport, scene tree, inspector, timeline, console |
| `03-domains` | domain toggles, wiring, schedule, profile YAML |
| `04-electrical` | schematic and electrical views |
| `05-signals` | waveform scopes |
| `06-sensors` | camera, lidar and other sensor views |
| `07-environment` | weather, EMI and field overlays |
| `08-replay` | recordings and replay |
| `09-ros2` | ROS 2 topic browser |
| `10-world` | huge worlds, tiles |
| `11-performance` | timings, real-time factor, stream health |

Suggested build order from the design: 00, 02, 03, 05, 04, 01, then 06 to 11.

## Rules

- The HTML is a picture of the target. Never port HTML or CSS, and never add web technology
  (CLAUDE.md §3 invariant 13). Rebuild each screen with Dear ImGui widgets.
- Colours, sizes and fonts come from `lagrange::theme` and the shared widgets
  (`DomainChip`, `PanelTabBar`, `SectionHeader`, `PropertyRow`, `Pill`, `Switch`, ...). No
  inline `PushStyleColor` in panels except through those widgets.
- Fonts are IBM Plex Sans and IBM Plex Mono (SIL OFL 1.1, TTFs in `viewer/assets/fonts/`;
  `docs/design/ui/00-design-system/fonts-ttf` links there). Every number, unit, path and topic name is in
  Mono.
- Domain colours identify a domain only. Never use them for plotted series.
- Panels show only what arrives through the protocol (invariant 14). Numbers visible in a
  reference image belong in the mock server's sample data, never hard-coded in a panel.
- Every panel handles the five states in `ui-conventions.md` §4: disconnected, loading, empty,
  domain disabled, error.
- Anything the design marks `TODO(verify)` stays `TODO(verify)` until checked against the
  source (CLAUDE.md §10).
- Finish a screen by running its `SPEC.md` acceptance checks and comparing a headless viewer
  screenshot with `reference.png`.

## Where the design lives in the code

- Theme: `viewer/app/theme/` (`lagrange_theme.hpp` is the viewer's checked copy of the
  design's header; `Fonts.hpp` loads IBM Plex).
- Widgets: `viewer/app/widgets/` (`Widgets.hpp`). Add a widget there when the design system
  lists one that does not exist yet; do not draw it inline in a panel.
- Chrome (menu bar, workspace bar, domain strip, status bar, per-workspace dock spaces):
  `viewer/panels/src/Chrome.cpp`. A new workspace is one entry in its table plus a layout.
- Headless screenshots for comparing with `reference.png`:
  `lagrange_viewer --headless --workspace <name> --size 1600x1000 --screenshot out.png`.

## When the design and the code disagree

- The design wins over the earlier sketches in `docs/design/viewer.md`.
- CLAUDE.md invariants and accepted RFCs win over the design. Protocol message names in
  `ui-conventions.md` §5 are indicative; the real names are the ones in `proto/`.
- If a screen needs something the design does not show, say so and propose it; do not
  silently improvise a new look.
- Change files in `docs/design/ui/` only when the owner asks. They are the owner's design.
