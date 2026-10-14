# Lagrange viewer UI design

This folder is the owner's design for every screen of `lagrange_viewer`. It is the visual and
behavioural target for all viewer UI work. Claude Code loads the rules for it from
`.claude/rules/viewer-ui.md`.

| Folder | Contents |
|---|---|
| `shared/` | `ui-conventions.md`, `design-tokens.json` (source of truth), `lagrange_theme.hpp` |
| `00-design-system` … `11-performance` | one screen each: `SPEC.md`, `README.md`, `reference.png` (2x), `reference.html` |
| `fonts/` | IBM Plex web fonts used by the `reference.html` pages (each screen's `fonts` is a link to it) |
| `00-design-system/fonts-ttf/` | link to `viewer/assets/fonts/`: IBM Plex Sans and Mono TTFs, installed with the viewer |

The fonts are IBM Plex, under the SIL Open Font License 1.1 (`fonts/OFL.txt`,
`LICENSES/OFL-1.1.txt`).

`reference.html` is a picture of the layout to open in a browser while developing. It is not
shipped and is never ported: the viewer is native Dear ImGui with no web technology.

Suggested order: 00 (theme and widgets) → 02 Scene → 03 Domains → 05 Signals → 04 Electrical
→ 01 Launcher → 06–11.
