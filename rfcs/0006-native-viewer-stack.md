# RFC 0006: Native viewer stack

| | |
|---|---|
| Status | Draft |
| Created | 2026-10-13 |
| Affects | `viewer/`, `gpu/hal/`, `gpu/scene/`, `gpu/kernels/`, `vendor/imgui_vendor` |
| Decides | Window library (SDL3 or GLFW), UI libraries, how the viewport shares the renderer with sensors |
| Detail | `docs/design/viewer.md` |

## Summary

`lagrange_viewer` uses SDL3 for windows and input, Dear ImGui (docking branch) with ImPlot,
imgui-node-editor and ImGuizmo for the interface, and its own Vulkan renderer built on `gpu/hal/`
and Slang shaders that are shared with camera-sensor rendering. All ImGui-family libraries are
built from pinned commits in one vendor package. The default is a single docked window that
behaves the same on Wayland and X11.

## Motivation

Invariant 13 requires a native process without web technology that renders through the same HAL
and shaders as the sensor simulation. CLAUDE.md §13 fixes most of the stack and leaves open:
the window library, the node-graph and gizmo libraries, and the license of the UI test engine.
The viewer is the whole of v0.1, so these choices are needed first.

Requirements:

| # | Requirement |
|---|---|
| 1 | Works on Wayland (Ubuntu's default session) and X11 |
| 2 | Vulkan surface creation without the HAL depending on the window library |
| 3 | Per-monitor HiDPI including fractional scaling |
| 4 | Text input through an IME for Japanese |
| 5 | Dockable panels, saved layouts; optional torn-off windows |
| 6 | Available on Ubuntu 26.04 through apt or a small vendor package; Apache-2.0-compatible licenses |
| 7 | 60 fps with 8 scopes; zero steady-state allocations on the UI thread |
| 8 | Testable headless on software Vulkan |

## Design

### Window and input: SDL3

Findings (verification §9):

| Aspect | SDL3 | GLFW |
|---|---|---|
| Version in Ubuntu 26.04 | 3.4.2 (`libsdl3-dev`) | 3.4 (`libglfw3-dev`) |
| Upstream latest | 3.4.18 | 3.5.1 |
| License | Zlib | Zlib |
| rosdep key | none yet | `libglfw3-dev` |
| Wayland | Preferred over X11 by default in SDL3 (upstream README) | Supported; runtime selection since 3.4 [Medium, from GLFW 3.4 release notes knowledge] |
| ImGui docking-branch backend | Multi-viewport supported; IME supported ("position somehow broken" noted in the backend); uses SDL 3.4 display events | Multi-viewport supported; backend lists parent-viewport support as missing |
| HiDPI | `SDL_GetWindowDisplayScale`, pixel-density window flag, scale-change events | Content scale callbacks |
| IME | Text-input API with composition events | Character callback only; no composition API [Medium] |
| Extras the viewer needs | File-drop events, clipboard, gamepads (teleoperation), offscreen/dummy video drivers for tests | File drop, clipboard, joysticks |

**Decision proposed: SDL3.** The deciding points are IME composition for Japanese text entry,
the more complete ImGui backend for the docking branch, and gamepad support for teleoperating
robots from the viewer. The cost is one rosdep key to contribute.

Isolation: SDL is used only in `viewer/app/`. `gpu/hal/` receives a `VkInstance` extension list
and a `VkSurfaceKHR`. Replacing the window library touches one directory.

### UI libraries

| Library | Version to pin | License | Source | Notes |
|---|---|---|---|---|
| Dear ImGui, docking branch | a commit on the docking branch ("1.93.0 WIP" on 2026-10-13); prefer the docking commit matching the latest tagged release (v1.92.9b) | MIT | `imgui_vendor` | Ubuntu's `libimgui-dev` 1.92.2b is the master branch and has no docking API, so it cannot be used |
| ImPlot | v1.0 | MIT | `imgui_vendor` | Scopes |
| imgui-node-editor | v0.9.3 or a later commit that builds against the pinned ImGui | MIT | `imgui_vendor` | Schematic; behind `viewer/panels/schematic/Canvas.hpp` |
| ImGuizmo | 1.10 | MIT | `imgui_vendor` | Transform gizmo |
| imgui_freetype | from the ImGui tree | MIT | `imgui_vendor` | Uses apt FreeType |
| Dear ImGui Test Engine | pinned commit | Dear ImGui Test Engine License v1.04 | fetched in CI only | Not vendored, not shipped |

One vendor package builds all of them into a single static library with one `imconfig.h`
(`ImDrawIdx` 32-bit, `IMGUI_ENABLE_FREETYPE`, `ImTextureID` as a 64-bit handle for Vulkan
descriptor sets, assertions routed to the project's logger).

### Renderer and sharing with sensors

- The viewport is rendered offscreen and shown as an ImGui image. ImGui itself is drawn by the
  ImGui Vulkan backend into the swapchain image, using the HAL's device and queue.
- `gpu/scene/GpuScene` holds meshes, materials and instance transforms and is fed through a
  `GpuSceneSource` interface by either the viewer's `SceneStore` (protocol data) or the server's
  world tables. The opaque raster pipeline and its Slang modules live in `gpu/kernels/` and are
  the same for the viewport and for camera sensors.
- Viewer-only shaders (grid, gizmo, outline, field slices, scope decimation is CPU-side) live in
  `viewer/render/shaders/`.
- A frame graph describes passes so that Tier 2 additions do not restructure the renderer.

### Threading

UI thread: events, ImGui, Vulkan submission. Client worker: socket and Protobuf decoding.
Hand-off by a triple buffer (scene state), SPSC queues (structural edits, command results, logs)
and per-signal SPSC ring buffers with a min/max pyramid (scopes). The UI thread takes no locks.
Header sketches for `FrameLoop`, `Panel`, `PanelRegistry`, `ScopeBuffer` and `ViewerClient` are
in `viewer.md` §5–§8 and are part of this RFC.

### Wayland and X11 policy

1. Default: one window, everything docked. Multi-viewport is behind `--multi-viewport` and a
   setting, off by default on both session types, because Wayland clients cannot position
   toplevel windows (confirmed by SDL's Wayland notes and Dear ImGui issue #9370, open).
2. Window position is not persisted on Wayland; size and maximised state are.
3. Client-side decorations through libdecor on GNOME; package dependency declared.
4. Scale changes rebuild the font atlas and rescale the style from an unscaled copy.
5. The release checklist tests GNOME Wayland, GNOME X11 and KDE Wayland at 1× and a fractional
   scale, with NVIDIA and Mesa drivers.

### Fonts and text

Bundled SIL-OFL fonts (Latin UI, Japanese, monospace, icons) with license files, rasterised by
FreeType. Japanese glyphs are loaded on demand if the pinned ImGui has the dynamic font atlas
(to verify on the pinned commit), otherwise baked from the Japanese glyph range at startup with
the start-up budget measured. UTF-8 throughout; UI strings pass through a translation lookup
from the first version.

### Testing

Headless runs on lavapipe with golden images; unit tests for client and buffers without a
display; UI automation through the test engine in CI; sanitizer runs; a manual matrix per release
(`viewer.md` §12).

## Alternatives considered

| Alternative | Why not |
|---|---|
| GLFW | Smaller, with an existing rosdep key, but weaker IME handling and a less complete docking-branch backend (table above). Remains the fallback; the window layer is isolated to make a switch cheap |
| Qt 6 (Widgets or Quick) | Large dependency with LGPL/commercial licensing to manage; its own scene graph and render loop compete with a custom Vulkan renderer; RViz and Gazebo already occupy that design space. An immediate-mode UI suits live engineering data |
| Using Ubuntu's `libimgui-dev` | Master branch, no docking (verified) |
| RmlUi, Nuklear, egui (Rust), other UI kits | Fewer ready-made pieces for plots, node graphs and gizmos; egui would add a Rust toolchain |
| Rendering the viewport directly to the swapchain with ImGui on top | Simpler, but the viewport could not be a dockable panel, and offscreen rendering is needed for headless tests anyway |
| A separate renderer for the viewer (not shared with sensors) | Violates invariant 13 and doubles shader work; camera views and the viewport would drift apart |
| OGRE or gz-rendering | Different renderers from the sensor path; OGRE 1.12/OpenGL is what RViz2 uses on Lyrical, so no modern Vulkan path to share |
| imnodes instead of imgui-node-editor | Also MIT; last release 2022. Simpler API but fewer features (no built-in zoom at the time of its last release [Low]). Kept as second choice |
| In-house schematic canvas from the start | More control and no stale dependency, but a week or two of work that a library may save; the `Canvas` interface defers the decision at low cost |
| Embedding the test engine in the repository | Its license is not OSI-approved; keep it out of the source tree and packages |
| SDL_GPU or a wrapper library instead of own Vulkan HAL | The HAL must also serve compute kernels, ray queries and CUDA interop for sensors; a general wrapper would constrain that |

## Unresolved questions

1. **Which docking commit to pin**, and whether imgui-node-editor builds against it. Go/no-go
   when `imgui_vendor` is created; fallback is imnodes or the in-house canvas.
2. **Dynamic font atlas** availability on the pinned commit, and cold-start time with Japanese
   fonts against the 5 s budget.
3. **SDL3 headless mode**: which SDL video driver allows a run without a compositor in CI while
   still exercising the event path; whether the headless test needs a `VkSurfaceKHR` at all
   (proposed: render to an offscreen target and skip the swapchain in headless mode).
4. **IME candidate window position** with the ImGui SDL3 backend ("position somehow broken" in
   the backend notes): test with Fcitx5 and IBus on Wayland and X11; upstream a fix if needed.
5. **Multi-viewport on X11**: ship as an option in v0.1 or hide it until Wayland behaves
   comparably? Proposed: option, documented as experimental.
6. **Test engine usage** (open question Q12): confirm CI-only use is acceptable.
7. **Plugin panels**: third-party panels would need a UI plugin ABI. Out of scope before 1.0?
8. **Gizmo library**: ImGuizmo is proposed. Its interaction quality for precise editing should be
   judged in v0.1; an own implementation is the fallback.
9. **Colour management**: sRGB swapchain and linear rendering are assumed; HDR output is out of
   scope.
10. **Accessibility**: an immediate-mode UI has no screen-reader support. Accepted limitation
    for an engineering tool, to be stated in the docs.
