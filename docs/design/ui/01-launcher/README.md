# Lagrange UI · 01 · Launcher

Handoff package for **Claude Code**: one screen of the native Lagrange viewer
(C++20 · Vulkan · Dear ImGui docking · ImPlot · SDL3/GLFW). No web technology.
The HTML reference is a *picture of the target*, not code to ship.

## Contents
| File | What it is |
|---|---|
| `SPEC.md` | Purpose, dock layout, panels → ImGui widgets, protocol bindings, states, mock data, acceptance checks |
| `reference.png` | Target rendering at 2× (3200×2000 for a 1600×1000 window) |
| `reference.html` + `fonts/` | Same screen as an offline page. Open it in any browser to zoom in or inspect exact text, numbers and colors |
| `shared/design-tokens.json` | Colors, type scale, ImGui metrics (source of truth) |
| `shared/lagrange_theme.hpp` | Tokens mapped onto ImGui/ImPlot styles. `TODO(verify)` marks enum names that depend on the version |
| `shared/ui-conventions.md` | App chrome, workspace bar, domain strip states, shortcuts, panel states, protocol message table |

## Prompt for Claude Code
Unzip into `docs/design/ui/01-launcher/` in the repo, then paste:

```
Implement the "Launcher" screen of lagrange_viewer from docs/design/ui/01-launcher/.
Read CLAUDE.md first (especially §3 invariants 13–14 and §13), then SPEC.md,
shared/ui-conventions.md and shared/lagrange_theme.hpp. Use reference.png as the visual
target (it is 2x; the layout is for a 1600x1000 logical window).

Rules:
- Native C++20 + Dear ImGui docking + ImPlot only. No browser, webview or JS.
- Panels read data only from the protocol client (viewer/client/), fed by the mock
  server. Never hard-code the numbers from the reference image into panels; add
  them to the mock server's sample data instead.
- Use lagrange::theme and the shared widgets; no ad-hoc colors.
- Code goes under viewer/panels/launcher/.
- Anything marked TODO(verify) stays TODO(verify) until checked against the source.
- Plan first (this touches viewer/ and mock_server/), then implement in small commits
  with scope `viewer`.
- Finish with the SPEC.md acceptance checks, and take a screenshot from the headless viewer
  test to compare with reference.png.
```
