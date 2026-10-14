# 00 · Design system: SPEC

## Purpose
This is the visual foundation for every screen. It is not a screen users open. Implement it once
as `viewer/app/theme/` plus a small widgets library (`viewer/app/widgets/`), and have every panel
use those widgets instead of styling inline.

## Deliverables in this zip
- `shared/design-tokens.json`: the source of truth for colors, type and metrics.
- `shared/lagrange_theme.hpp`: the ImGui and ImPlot style mapping (check the TODO(verify) enum names).
- `fonts/`: IBM Plex Sans (400/500/600) and IBM Plex Mono (400/500) as TTF, plus `OFL.txt`.
  These are converted from IBM's npm packages. Japanese glyphs need IBM Plex Sans JP as well.
  TODO(verify): package source.

## Fonts (ImGui)
- Atlas 1, the default: Plex Sans Regular 13 px, with Plex Sans JP merged
  (`GetGlyphRangesJapanese()`) and an icon font merged.
- Atlas entries: Sans Medium 12/13, Sans SemiBold 12/14, Mono Regular 11/12, Mono Medium 12, and Mono 20 for KPIs.
- Build the atlas with the per-monitor scale (HiDPI). Rebuild it when the window moves to a
  monitor with a different scale.
- Icons: the 24 icons on the canvas are drawn as 16 px stroke icons. Map them to an icon font
  such as Lucide or Tabler. TODO(verify): license and that a TTF build exists. Keep the names
  used on the canvas (scene, domains, electrical, signals, …).

## Widgets library (create these; every screen uses them)
| Widget | Notes |
|---|---|
| `DomainChip(domain, state, detail)` | states: on / substitute / off / pending / error. Clickable; right-click menu |
| `PanelTabBar(tabs, tools)` | 28 px; accent overline on the selected tab; right-aligned tool slot |
| `SectionHeader(title, meta, domain?)` | 26 px collapsible header on bg2 |
| `PropertyRow(label, value, unit, edited)` | 112 px label column; value field with unit in t3; accent border when edited |
| `Vec3Field(label, v, unit)` | three fields with colored X/Y/Z labels |
| `Pill(text, kind)` | ok / warn / err / info / acc / neutral |
| `Switch(bool&)` | 28×16 track |
| `Segmented(options, idx)` | for time windows, modes |
| `Meter(frac, color)` | 6 px bar |
| `TreeRow(...)` | 22 px; selected = accentWeak + 2 px accent bar; domain badges on the right |
| `KpiCard(label, value, sub)` | Mono 20 value |
| `Scope(...)` | ImPlot wrapper with min/max decimation, cursors A/B, threshold lines, title at right |

## Rules
- Use accent for one thing per region: the current selection or the primary action.
- Status colors always come with text or an icon.
- Domain colors are only for domain identity, never for plotted series.
- Every number is in Mono with its unit in t3.
- Text contrast: t1 15.4:1, t2 9.0:1, t3 5.7:1 on bg1. Error red on bg3 is 4.3:1, so use it
  for pills and icons, not for long text.

## Acceptance
- One call to `lagrange::theme::apply(scale)` styles the whole app. There are no inline
  `PushStyleColor` calls in panels except through the widgets.
- A demo window (`--theme-demo`) renders every widget so you can compare it against `reference.png`.
