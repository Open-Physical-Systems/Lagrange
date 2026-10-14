#pragma once

#include <imgui.h>

/// The icons and the logo of the UI design, drawn as vector shapes. Their outlines are the
/// SVG paths of docs/design/ui/*/reference.html (16-unit grid, 1.5 unit strokes), so they stay
/// sharp at any monitor scale and need no icon font.
namespace lagrange::viewer::widgets {

enum class Icon : int {
    // Workspaces, in the order of the workspace bar.
    Scene,
    Domains,
    Electrical,
    Signals,
    Sensors,
    Environment,
    Replay,
    Ros2,
    World,
    Performance,
    // Run controls.
    Reset,
    Pause,
    Play,
    Step,
    JumpStart,
    // Panels.
    Search,
    Filter,
    Plus,
    Check,
    ChevronDown,
    Lock,
    Stage,
    Component,
    Warning,
    // Viewport tools and overlays.
    Select,
    Move,
    Rotate,
    Scale,
    Measure,
    Grid,
    Frames,
    Forces,
    Frustum,
    Path,
    Fields,
    // Launcher.
    Folder,
    Download,
    Count,
};

/// Draws `icon` in a square of `size` pixels at `topLeft`. `knockout` is the colour behind the
/// icon: a few icons cut small shapes out of their own strokes with it.
void DrawIcon(ImDrawList* draw, Icon icon, const ImVec2& topLeft, float size, ImU32 color,
              ImU32 knockout);

/// An icon as an item on the current line, centred on the text. `logicalSize` is in design
/// pixels (14 in toolbars, 12 in panels).
void IconItem(Icon icon, float logicalSize, const ImVec4& color);

/// Strokes SVG path data (the commands the design uses: M L H V C A Z, absolute or relative).
/// A point (x, y) of the path lands at origin + (x, y) * scale. For line art that is not on
/// the 16-unit icon grid.
void StrokeSvgPath(ImDrawList* draw, const char* data, const ImVec2& origin, float scale,
                   ImU32 color, float thickness);

/// The Lagrange mark: an L with the accent dot.
void DrawLogo(ImDrawList* draw, const ImVec2& topLeft, float size);

}  // namespace lagrange::viewer::widgets
