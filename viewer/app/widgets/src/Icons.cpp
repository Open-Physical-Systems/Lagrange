#include "lagrange/viewer/Icons.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <imgui_internal.h>
#include <numbers>
#include <vector>

namespace lagrange::viewer::widgets {

namespace {

constexpr float kGrid = 16.0F;  // icons are drawn on a 16 x 16 grid

enum class Paint : std::uint8_t { Stroke, Fill, Knockout };

/// One element of an icon, as in the design's SVG.
struct Shape {
    enum class Kind : std::uint8_t { Path, Circle, Rect } kind;
    Paint paint;
    const char* path;  // Kind::Path: SVG path data
    float a, b, c, d;  // Circle: cx, cy, r.  Rect: x, y, width, height.
    float stroke;      // stroke width in grid units; also outlines a Knockout circle
};

constexpr Shape path(const char* data, float stroke = 1.5F, Paint paint = Paint::Stroke) {
    return {Shape::Kind::Path, paint, data, 0, 0, 0, 0, stroke};
}
constexpr Shape circle(float cx, float cy, float r, float stroke = 1.5F,
                       Paint paint = Paint::Stroke) {
    return {Shape::Kind::Circle, paint, nullptr, cx, cy, r, 0, stroke};
}
constexpr Shape rect(float x, float y, float w, float h, float stroke = 1.5F,
                     Paint paint = Paint::Stroke) {
    return {Shape::Kind::Rect, paint, nullptr, x, y, w, h, stroke};
}

struct Definition {
    std::array<Shape, 4> shapes;
    int count;
};

// Path data copied from docs/design/ui/*/reference.html.
constexpr std::array<Definition, static_cast<std::size_t>(Icon::Count)> kIcons{{
    // Scene
    {{{path("M8 1.8l5.6 3.1v6.2L8 14.2l-5.6-3.1V4.9z M2.4 4.9L8 8l5.6-3.1 M8 8v6.2")}}, 1},
    // Domains
    {{{path("M2.5 4h11 M2.5 8h11 M2.5 12h11"), circle(6, 4, 1.6F, 1.5F, Paint::Knockout),
       circle(10.5F, 8, 1.6F, 1.5F, Paint::Knockout), circle(5, 12, 1.6F, 1.5F, Paint::Knockout)}},
     4},
    // Electrical
    {{{path("M9 1.5L3.5 9H8l-1 5.5L12.5 7H8z")}}, 1},
    // Signals
    {{{path("M1.5 8h2.2l1.6-4.5 3 9 2-6.5 1.5 2h2.7")}}, 1},
    // Sensors
    {{{path("M8 8l4.6-4.6 M14 8A6 6 0 1 1 8 2 M11 8a3 3 0 1 1-3-3")}}, 1},
    // Environment
    {{{path("M4.6 9.5a3 3 0 0 1 .3-6 3.6 3.6 0 0 1 6.8 1.1 2.5 2.5 0 0 1-.2 4.9z "
            "M5.5 11.5l-.8 2 M8.5 11.5l-.8 2 M11.5 11.5l-.8 2")}},
     1},
    // Replay
    {{{path("M2.5 8a5.5 5.5 0 1 0 1.6-3.9 M2.5 2.5v3h3 M7 5.8v4.4l3.2-2.2z")}}, 1},
    // Ros2
    {{{circle(4, 4, 1.7F), circle(12, 4, 1.7F), circle(8, 12, 1.7F),
       path("M5.7 4h4.6 M4.9 5.5L7.1 10.5 M11.1 5.5L8.9 10.5")}},
     4},
    // World
    {{{circle(8, 8, 6), path("M2 8h12 M8 2c2.2 2.2 2.2 9.8 0 12 M8 2c-2.2 2.2-2.2 9.8 0 12")}}, 2},
    // Performance
    {{{path("M2.5 11.5a5.5 5.5 0 1 1 11 0 M8 11.5l3-4")}}, 1},
    // Builder (12-builder)
    {{{path("M10.6 2.2a3.2 3.2 0 0 0-3.5 4.3L2.4 11.2a1.3 1.3 0 0 0 0 1.9l.5.5a1.3 1.3 0 0 0 1.9 "
            "0l4.7-4.7a3.2 3.2 0 0 0 4.3-3.5l-1.9 1.9-1.9-.5-.5-1.9z")}},
     1},
    // Reset
    {{{path("M3.5 8a4.5 4.5 0 1 0 1.3-3.2 M3.5 2.8v2.6h2.6")}}, 1},
    // Pause
    {{{path("M5.5 3.5v9 M10.5 3.5v9", 2.0F)}}, 1},
    // Play
    {{{path("M4.5 3v10l8-5z", 0.0F, Paint::Fill)}}, 1},
    // Step
    {{{path("M3.5 3.5v9l6.5-4.5z", 0.0F, Paint::Fill), rect(11, 3.5F, 1.8F, 9, 0.0F, Paint::Fill)}},
     2},
    // JumpStart
    {{{rect(3, 3.5F, 1.8F, 9, 0.0F, Paint::Fill), path("M13 3.5v9L6 8z", 0.0F, Paint::Fill)}}, 2},
    // Search
    {{{circle(7, 7, 4.5F, 1.6F), path("M10.5 10.5l3.5 3.5", 1.6F)}}, 2},
    // Filter
    {{{path("M2.5 3.5h11L9.5 8.5v4l-3 1.5v-5.5z")}}, 1},
    // Plus
    {{{path("M8 3v10M3 8h10", 1.6F)}}, 1},
    // Check
    {{{path("M3 8.5l3 3 7-7", 2.0F)}}, 1},
    // ChevronDown
    {{{path("M4 6l4 4 4-4", 1.8F)}}, 1},
    // Lock
    {{{rect(3.5F, 7, 9, 6.5F), path("M5.5 7V5a2.5 2.5 0 0 1 5 0v2")}}, 2},
    // Stage
    {{{circle(8, 8, 6), path("M2 8h12")}}, 2},
    // Component
    {{{circle(8, 8, 5.5F), path("M5.6 10.5v-5l2.4 3 2.4-3v5")}}, 2},
    // Warning
    {{{path("M8 2.2l6.3 11.3H1.7z M8 6.5v3.4 M8 11.6v.4")}}, 1},
    // Select
    {{{path("M3.5 2.5l9 5-4 1-2 4.5z")}}, 1},
    // Move
    {{{path("M8 1.5v13 M1.5 8h13 M6 3.5l2-2 2 2 M6 12.5l2 2 2-2 M3.5 6l-2 2 2 2 "
            "M12.5 6l2 2-2 2")}},
     1},
    // Rotate
    {{{path("M13 8a5 5 0 1 1-1.5-3.6 M13 2.5v2.5h-2.5")}}, 1},
    // Scale
    {{{rect(2.5F, 7.5F, 6, 6), path("M8.5 7.5l5-5 M10 2.5h3.5V6")}}, 2},
    // Measure
    {{{path("M2 11l9-9 3 3-9 9z M5 8l1.5 1.5 M7.5 5.5L9 7")}}, 1},
    // Grid
    {{{path("M2.5 2.5h11v11h-11z M2.5 6.2h11 M2.5 9.8h11 M6.2 2.5v11 M9.8 2.5v11", 1.4F)}}, 1},
    // Frames
    {{{path("M4 12V3 M4 12h9 M4 12l-2.5 2.5")}}, 1},
    // Forces
    {{{path("M8 13.5V3 M5 6l3-3 3 3"), circle(8, 13.5F, 1.2F, 0.0F, Paint::Fill)}}, 2},
    // Frustum
    {{{path("M2 8l11-5v10z")}}, 1},
    // Path
    {{{path("M2 13c4 0 3-10 12-10")}}, 1},
    // Fields
    {{{path("M8 2l6 3-6 3-6-3z M2 8l6 3 6-3 M2 11l6 3 6-3")}}, 1},
    // Folder
    {{{path("M2 4.5h4l1.5 1.5H14v6.5H2z", 1.4F)}}, 1},
    // Download
    {{{path("M8 2.5v8 M4.5 7L8 10.5 11.5 7 M3 13.5h10", 1.4F)}}, 1},
}};

// --- SVG path data to polylines --------------------------------------------------------

struct Outline {
    std::vector<ImVec2> points;
    bool closed{false};
};

class PathReader {
public:
    explicit PathReader(const char* data) : at_(data) {}

    /// The next command letter, or 0 at the end. Numbers without a letter repeat the last.
    char command() {
        skip();
        if (*at_ == '\0') {
            return '\0';
        }
        if ((*at_ >= 'a' && *at_ <= 'z') || (*at_ >= 'A' && *at_ <= 'Z')) {
            last_ = *at_++;
        } else if (last_ == 'M') {
            last_ = 'L';  // further coordinate pairs after a move are lines
        } else if (last_ == 'm') {
            last_ = 'l';
        }
        return last_;
    }

    float number() {
        skip();
        char* end = nullptr;
        const float value = std::strtof(at_, &end);
        at_ = end == at_ ? at_ + 1 : end;  // never stall on a malformed path
        return value;
    }

    /// Arc flags are single digits and may be written without separators.
    bool flag() {
        skip();
        const bool value = *at_ == '1';
        if (*at_ == '0' || *at_ == '1') {
            ++at_;
        }
        return value;
    }

private:
    void skip() {
        while (*at_ == ' ' || *at_ == ',' || *at_ == '\n') {
            ++at_;
        }
    }
    const char* at_;
    char last_{'\0'};
};

void cubic(Outline& out, ImVec2 p0, ImVec2 p1, ImVec2 p2, ImVec2 p3) {
    constexpr int kSegments = 10;
    for (int i = 1; i <= kSegments; ++i) {
        const float t = static_cast<float>(i) / kSegments;
        const float u = 1.0F - t;
        out.points.emplace_back(
            u * u * u * p0.x + 3 * u * u * t * p1.x + 3 * u * t * t * p2.x + t * t * t * p3.x,
            u * u * u * p0.y + 3 * u * u * t * p1.y + 3 * u * t * t * p2.y + t * t * t * p3.y);
    }
}

/// An SVG elliptical arc from `from` to `to` (endpoint form, no axis rotation: the design's
/// icons use none), flattened.
void arc(Outline& out, ImVec2 from, ImVec2 to, float rx, float ry, bool large, bool sweep) {
    if (rx <= 0.0F || ry <= 0.0F || (from.x == to.x && from.y == to.y)) {
        out.points.push_back(to);
        return;
    }
    // W3C SVG implementation notes, F.6.5, with phi = 0.
    const float x1 = (from.x - to.x) * 0.5F;
    const float y1 = (from.y - to.y) * 0.5F;
    const float lambda = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry);
    if (lambda > 1.0F) {  // radii too small to reach: scale them up
        rx *= std::sqrt(lambda);
        ry *= std::sqrt(lambda);
    }
    const float numerator = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
    const float denominator = rx * rx * y1 * y1 + ry * ry * x1 * x1;
    float factor = std::sqrt(std::max(numerator / denominator, 0.0F));
    if (large == sweep) {
        factor = -factor;
    }
    const float cxp = factor * rx * y1 / ry;
    const float cyp = -factor * ry * x1 / rx;
    const float cx = cxp + (from.x + to.x) * 0.5F;
    const float cy = cyp + (from.y + to.y) * 0.5F;
    const float start = std::atan2((y1 - cyp) / ry, (x1 - cxp) / rx);
    float delta = std::atan2((-y1 - cyp) / ry, (-x1 - cxp) / rx) - start;
    const float tau = 2.0F * std::numbers::pi_v<float>;
    if (sweep && delta < 0.0F) {
        delta += tau;
    } else if (!sweep && delta > 0.0F) {
        delta -= tau;
    }
    const int segments = std::max(static_cast<int>(std::ceil(std::abs(delta) / 0.2F)), 2);
    for (int i = 1; i <= segments; ++i) {
        const float angle = start + delta * static_cast<float>(i) / static_cast<float>(segments);
        out.points.emplace_back(cx + rx * std::cos(angle), cy + ry * std::sin(angle));
    }
}

std::vector<Outline> flatten(const char* data) {
    std::vector<Outline> outlines;
    PathReader reader(data);
    ImVec2 pen(0, 0);
    ImVec2 start(0, 0);
    const auto current = [&]() -> Outline& {
        if (outlines.empty()) {
            outlines.emplace_back().points.push_back(pen);
        }
        return outlines.back();
    };
    for (char command = reader.command(); command != '\0'; command = reader.command()) {
        const bool relative = command >= 'a';
        const auto point = [&] {
            const float x = reader.number();
            const float y = reader.number();
            return relative ? ImVec2(pen.x + x, pen.y + y) : ImVec2(x, y);
        };
        switch (command) {
            case 'M':
            case 'm':
                pen = point();
                start = pen;
                outlines.emplace_back().points.push_back(pen);
                break;
            case 'L':
            case 'l':
                pen = point();
                current().points.push_back(pen);
                break;
            case 'H':
            case 'h': {
                const float x = reader.number();
                pen.x = relative ? pen.x + x : x;
                current().points.push_back(pen);
                break;
            }
            case 'V':
            case 'v': {
                const float y = reader.number();
                pen.y = relative ? pen.y + y : y;
                current().points.push_back(pen);
                break;
            }
            case 'C':
            case 'c': {
                const ImVec2 p1 = point();
                const ImVec2 p2 = point();
                const ImVec2 p3 = point();
                cubic(current(), pen, p1, p2, p3);
                pen = p3;
                break;
            }
            case 'A':
            case 'a': {
                const float rx = reader.number();
                const float ry = reader.number();
                (void)reader.number();  // axis rotation: always 0 in the design
                const bool large = reader.flag();
                const bool sweep = reader.flag();
                const ImVec2 to = point();
                arc(current(), pen, to, rx, ry, large, sweep);
                pen = to;
                break;
            }
            case 'Z':
            case 'z':
                current().closed = true;
                pen = start;
                break;
            default:
                return outlines;  // a command the design does not use
        }
    }
    return outlines;
}

/// Flattened outlines of every path shape, built on first use. UI thread only.
const std::vector<Outline>& outlinesOf(const Shape& shape) {
    static std::vector<std::pair<const char*, std::vector<Outline>>> cache;
    for (const auto& [data, outlines] : cache) {
        if (data == shape.path) {
            return outlines;
        }
    }
    cache.emplace_back(shape.path, flatten(shape.path));
    return cache.back().second;
}

}  // namespace

void DrawIcon(ImDrawList* draw, Icon icon, const ImVec2& topLeft, float size, ImU32 color,
              ImU32 knockout) {
    const Definition& definition = kIcons.at(static_cast<std::size_t>(icon));
    const float scale = size / kGrid;
    const auto place = [&](float x, float y) {
        return ImVec2(topLeft.x + x * scale, topLeft.y + y * scale);
    };
    static std::vector<ImVec2> scratch;  // UI thread only; grows once
    for (int i = 0; i < definition.count; ++i) {
        const Shape& shape = definition.shapes.at(static_cast<std::size_t>(i));
        const float thickness = std::max(shape.stroke * scale, 1.0F);
        switch (shape.kind) {
            case Shape::Kind::Circle:
                if (shape.paint == Paint::Fill) {
                    draw->AddCircleFilled(place(shape.a, shape.b), shape.c * scale, color);
                } else {
                    if (shape.paint == Paint::Knockout) {
                        draw->AddCircleFilled(place(shape.a, shape.b), shape.c * scale, knockout);
                    }
                    draw->AddCircle(place(shape.a, shape.b), shape.c * scale, color, 0, thickness);
                }
                break;
            case Shape::Kind::Rect:
                if (shape.paint == Paint::Fill) {
                    draw->AddRectFilled(place(shape.a, shape.b),
                                        place(shape.a + shape.c, shape.b + shape.d), color,
                                        0.5F * scale);
                } else {
                    draw->AddRect(place(shape.a, shape.b),
                                  place(shape.a + shape.c, shape.b + shape.d), color, 1.0F * scale,
                                  thickness);
                }
                break;
            case Shape::Kind::Path:
                for (const Outline& outline : outlinesOf(shape)) {
                    scratch.clear();
                    for (const ImVec2& point : outline.points) {
                        scratch.push_back(place(point.x, point.y));
                    }
                    if (shape.paint == Paint::Fill) {
                        draw->AddConcavePolyFilled(scratch.data(), static_cast<int>(scratch.size()),
                                                   color);
                    } else {
                        draw->AddPolyline(scratch.data(), static_cast<int>(scratch.size()), color,
                                          thickness,
                                          outline.closed ? ImDrawFlags_Closed : ImDrawFlags_None);
                    }
                }
                break;
        }
    }
}

void StrokeSvgPath(ImDrawList* draw, const char* data, const ImVec2& origin, float scale,
                   ImU32 color, float thickness) {
    static std::vector<ImVec2> scratch;  // UI thread only
    // Cached by the address of the path data, like the icons: callers pass string literals.
    const Shape shape = path(data);
    for (const Outline& outline : outlinesOf(shape)) {
        scratch.clear();
        for (const ImVec2& point : outline.points) {
            scratch.emplace_back(origin.x + point.x * scale, origin.y + point.y * scale);
        }
        draw->AddPolyline(scratch.data(), static_cast<int>(scratch.size()), color, thickness,
                          outline.closed ? ImDrawFlags_Closed : ImDrawFlags_None);
    }
}

void IconItem(Icon icon, float logicalSize, const ImVec4& color) {
    const float size = logicalSize * ImGui::GetStyle().FontScaleDpi;
    const float line = ImGui::GetTextLineHeight();
    ImVec2 at = ImGui::GetCursorScreenPos();
    at.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + (line - size) * 0.5F;
    DrawIcon(ImGui::GetWindowDrawList(), icon, at, size, ImGui::ColorConvertFloat4ToU32(color),
             ImGui::GetColorU32(ImGuiCol_WindowBg));
    ImGui::Dummy(ImVec2(size, line));
}

void DrawLogo(ImDrawList* draw, const ImVec2& topLeft, float size) {
    // Design: viewBox 120; path "M34 18 V86 H102", stroke 16, round caps and joins; accent
    // dot at (68, 27.1), radius 11.
    const float scale = size / 120.0F;
    const auto place = [&](float x, float y) {
        return ImVec2(topLeft.x + x * scale, topLeft.y + y * scale);
    };
    const ImU32 ink = ImGui::ColorConvertFloat4ToU32(theme::c::t1);
    const float stroke = 16.0F * scale;
    const std::array<ImVec2, 3> points{place(34, 18), place(34, 86), place(102, 86)};
    draw->AddPolyline(points.data(), 3, ink, stroke, ImDrawFlags_None);
    for (const ImVec2& point : points) {
        draw->AddCircleFilled(point, stroke * 0.5F, ink);  // round caps and the round corner
    }
    draw->AddCircleFilled(place(68, 27.1F), 11.0F * scale,
                          ImGui::ColorConvertFloat4ToU32(theme::c::accent));
}

}  // namespace lagrange::viewer::widgets
