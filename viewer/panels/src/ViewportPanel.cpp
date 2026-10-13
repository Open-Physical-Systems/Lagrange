// Scene workspace, centre: the 3D viewport with its overlays (docs/design/ui/02-scene/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Picking.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr double kOrbitRadiansPerPixel = 0.006;
constexpr double kZoomPerWheelStep = 0.88;
constexpr float kClickSlopPixels = 4.0F;  // a press that moves less than this is a click

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// A text box drawn over the image: raised surface, thin border, mono text.
void overlayBox(ImDrawList* draw, const ImVec2& min, const char* first, const char* second,
                const ImVec4& border = c::lineStrong, const ImVec4& text = c::t1) {
    ImFont* font = theme::fonts().mono;
    const float size = px(theme::kMonoSize);
    const ImVec2 a = font->CalcTextSizeA(size, FLT_MAX, 0.0F, first);
    const ImVec2 b =
        second != nullptr ? font->CalcTextSizeA(size, FLT_MAX, 0.0F, second) : ImVec2(0.0F, 0.0F);
    const float pad = px(8.0F);
    const float lines = second != nullptr ? 2.0F : 1.0F;
    const ImVec2 max(min.x + std::max(a.x, b.x) + pad * 2.0F, min.y + px(18.0F) * lines + pad);
    // Design (.ov): rgba(9, 17, 32, .88) with a lineStrong border, 4 px radius.
    draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
    draw->AddRect(min, max, u32(border), px(4.0F));
    draw->AddText(font, size, ImVec2(min.x + pad, min.y + pad * 0.5F + px(2.0F)), u32(text), first);
    if (second != nullptr) {
        draw->AddText(font, size, ImVec2(min.x + pad, min.y + pad * 0.5F + px(20.0F)), u32(c::t2),
                      second);
    }
}

/// Where a point of the region frame lands in the viewport, if it is in front of the camera.
bool project(const Camera& camera, const std::array<double, 3>& point, const ImVec2& origin,
             const ImVec2& size, ImVec2& out) {
    const std::array<double, 3> eye = camera.eye();
    const Mat4 m = camera.viewProjection(static_cast<double>(size.x) / std::max(size.y, 1.0F));
    const std::array<float, 3> v{static_cast<float>(point[0] - eye[0]),
                                 static_cast<float>(point[1] - eye[1]),
                                 static_cast<float>(point[2] - eye[2])};
    const float x = m[0] * v[0] + m[1] * v[1] + m[2] * v[2] + m[3];
    const float y = m[4] * v[0] + m[5] * v[1] + m[6] * v[2] + m[7];
    const float w = m[12] * v[0] + m[13] * v[1] + m[14] * v[2] + m[15];
    if (w <= 1e-4F) {
        return false;
    }
    out = ImVec2(origin.x + (x / w * 0.5F + 0.5F) * size.x,
                 origin.y + (y / w * 0.5F + 0.5F) * size.y);  // Vulkan clip space: y down
    return out.x >= origin.x && out.x <= origin.x + size.x && out.y >= origin.y &&
           out.y <= origin.y + size.y;
}

class ViewportPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Viewport"; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(
            title(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        context.requestedViewportWidth =
            static_cast<std::uint32_t>(std::max(available.x * scale.x, 1.0F));
        context.requestedViewportHeight =
            static_cast<std::uint32_t>(std::max(available.y * scale.y, 1.0F));

        const ImVec2 origin = ImGui::GetCursorScreenPos();
        if (context.viewportTexture != 0 && context.scene) {
            ImGui::Image(static_cast<ImTextureID>(context.viewportTexture), available);
        } else {
            ImGui::Dummy(available);
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        if (!context.scene) {
            emptyState(context, origin, available);
            ImGui::End();
            return;
        }

        // One item over the whole image takes the mouse. The tool bars drawn after it may
        // overlap it and then take the mouse themselves.
        ImGui::SetCursorScreenPos(origin);
        ImGui::SetNextItemAllowOverlap();
        ImGui::InvisibleButton("##view", available,
                               ImGuiButtonFlags_MouseButtonLeft |
                                   ImGuiButtonFlags_MouseButtonRight |
                                   ImGuiButtonFlags_MouseButtonMiddle);
        const ImGuiIO& io = ImGui::GetIO();
        const bool hovered = ImGui::IsItemHovered();
        const double aspect = static_cast<double>(available.x) / std::max(available.y, 1.0F);
        const auto rayAt = [&](const ImVec2& mouse) {
            return context.camera.ray((mouse.x - origin.x) / available.x * 2.0 - 1.0,
                                      (mouse.y - origin.y) / available.y * 2.0 - 1.0, aspect);
        };
        if (ImGui::IsItemActive()) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                context.camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                                     io.MouseDelta.y * kOrbitRadiansPerPixel);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                context.camera.pan(-io.MouseDelta.x / std::max(available.y, 1.0F),
                                   io.MouseDelta.y / std::max(available.y, 1.0F));
            }
        }
        if (hovered && io.MouseWheel != 0.0F) {
            context.camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
        }
        // Select tool: a click that was not a drag selects the prim under it, or nothing.
        if (hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, kClickSlopPixels).x == 0.0F &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, kClickSlopPixels).y == 0.0F) {
            context.selectedPrim = pickPrim(*context.scene, rayAt(io.MousePos));
        }
        const bool hasSelection = context.selectedPrim < context.scene->statics->prims.size();
        // F: look at the selection (design shortcut), unless a text field has the keyboard.
        if (hasSelection && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            context.camera.target = context.scene->transforms[context.selectedPrim].position;
        }

        // Top left: camera and shading, frame time, draw count, tier.
        char tier[32];
        std::snprintf(tier, sizeof(tier), "%s", context.tierName.c_str());
        if (char* detail = std::strstr(tier, " (")) {
            *detail = '\0';  // "Tier 2 (full, hardware ray tracing)" -> "Tier 2"
        }
        char text[160];
        std::snprintf(text, sizeof(text), "Perspective · Lit   %.1f ms   %u draws   %s",
                      static_cast<double>(context.frameMs), context.drawCalls, tier);
        overlayBox(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), text, nullptr);

        // Bottom left: the frame's axes as the camera sees them, its convention, and how far
        // the camera is from what it looks at.
        triad(draw, context.camera,
              ImVec2(origin.x + px(10.0F), origin.y + available.y - px(58.0F)));

        // Bottom right: where the cursor is on the ground plane, and the Focus hint.
        if (hovered) {
            if (const auto ground = intersectGround(rayAt(io.MousePos))) {
                char cursor[96];
                std::snprintf(cursor, sizeof(cursor), "cursor  x %.2f  y %.2f  z %.2f   Focus [F]",
                              (*ground)[0], (*ground)[1], (*ground)[2]);
                const float width =
                    theme::fonts()
                        .mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, cursor)
                        .x +
                    px(16.0F);
                overlayBox(draw,
                           ImVec2(origin.x + available.x - width - px(10.0F),
                                  origin.y + available.y - px(36.0F)),
                           cursor, nullptr);
            }
        }

        // The selected prim's name, where it is.
        if (hasSelection) {
            ImVec2 at;
            if (project(context.camera, context.scene->transforms[context.selectedPrim].position,
                        origin, available, at)) {
                draw->AddCircle(at, px(5.0F), u32(c::accent), 0, px(1.5F));
                overlayBox(draw, ImVec2(at.x + px(10.0F), at.y + px(8.0F)),
                           context.scene->statics->prims[context.selectedPrim].name.c_str(),
                           nullptr, c::accent, c::accent);
            }
        }

        toolColumn(ImVec2(origin.x + px(10.0F), origin.y + px(46.0F)));
        overlayBar(context, ImVec2(origin.x + available.x - px(10.0F), origin.y + px(10.0F)));
        ImGui::End();
    }

private:
    /// The gizmo tools, as a column on the left. Selecting works; moving, turning, scaling
    /// and measuring change the scene, which a replay cannot do.
    static void toolColumn(const ImVec2& at) {
        struct Tool {
            const char* id;
            widgets::Icon icon;
            const char* tooltip;
        };
        static constexpr std::array<Tool, 5> kTools{{
            {"##select", widgets::Icon::Select, "Select (Q): click a prim"},
            {"##move", widgets::Icon::Move, "Move (W): needs a simulation server"},
            {"##rotate", widgets::Icon::Rotate, "Rotate (E): needs a simulation server"},
            {"##scale", widgets::Icon::Scale, "Scale (R): needs a simulation server"},
            {"##measure", widgets::Icon::Measure, "Measure (M): not built yet"},
        }};
        ImGui::SetCursorScreenPos(at);
        widgets::ToolBarFrame(static_cast<int>(kTools.size()), true);
        const float x = ImGui::GetCursorScreenPos().x;
        float y = ImGui::GetCursorScreenPos().y;
        for (std::size_t i = 0; i < kTools.size(); ++i) {
            ImGui::SetCursorScreenPos(ImVec2(x, y));
            ImGui::BeginDisabled(i != 0);
            widgets::ToolButton(kTools[i].id, kTools[i].icon, i == 0, kTools[i].tooltip);
            ImGui::EndDisabled();
            y += px(26.0F);
        }
    }

    /// The overlay toggles, as a row ending at `topRight`. Grid and frame axes are drawn by
    /// the viewer; the others show data that this server does not send yet.
    static void overlayBar(PanelContext& context, const ImVec2& topRight) {
        struct Overlay {
            const char* id;
            widgets::Icon icon;
            const char* tooltip;
            bool RenderOptions::* option;  // nullptr: no data to draw yet
        };
        static constexpr std::array<Overlay, 8> kOverlays{{
            {"##grid", widgets::Icon::Grid, "Grid", &RenderOptions::grid},
            {"##frames", widgets::Icon::Frames, "Frame axes", &RenderOptions::axes},
            {"##forces", widgets::Icon::Forces, "Contacts and forces: no data from this server",
             nullptr},
            {"##frustums", widgets::Icon::Frustum, "Sensor frustums: no data from this server",
             nullptr},
            {"##path", widgets::Icon::Path, "Planned path: no data from this server", nullptr},
            {"##fields", widgets::Icon::Fields, "Field overlays: no data from this server",
             nullptr},
            {"##electrical", widgets::Icon::Electrical, "Electrical overlay: not built yet",
             nullptr},
            {"##weather", widgets::Icon::Environment, "Weather effects: no data from this server",
             nullptr},
        }};
        const float width = px(24.0F) * 8.0F + px(2.0F) * 9.0F;
        ImGui::SetCursorScreenPos(ImVec2(topRight.x - width, topRight.y));
        widgets::ToolBarFrame(static_cast<int>(kOverlays.size()), false);
        float x = ImGui::GetCursorScreenPos().x;
        const float y = ImGui::GetCursorScreenPos().y;
        for (const Overlay& overlay : kOverlays) {
            ImGui::SetCursorScreenPos(ImVec2(x, y));
            if (overlay.option != nullptr) {
                bool& on = context.renderOptions->*overlay.option;
                if (widgets::ToolButton(overlay.id, overlay.icon, on, overlay.tooltip)) {
                    on = !on;
                }
            } else {
                ImGui::BeginDisabled(true);
                widgets::ToolButton(overlay.id, overlay.icon, false, overlay.tooltip);
                ImGui::EndDisabled();
            }
            x += px(26.0F);
        }
    }

    /// Axis triad and frame information (design: bottom left of the viewport).
    static void triad(ImDrawList* draw, const Camera& camera, const ImVec2& min) {
        char distance[48];
        std::snprintf(distance, sizeof(distance), "cam %.2f m", camera.distance);
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kMonoSize);
        const char* frame = "ENU · Z-up · m";
        const float textWidth = std::max(font->CalcTextSizeA(size, FLT_MAX, 0.0F, frame).x,
                                         font->CalcTextSizeA(size, FLT_MAX, 0.0F, distance).x);
        const float box = px(34.0F);
        const ImVec2 max(min.x + px(8.0F) + box + px(10.0F) + textWidth + px(10.0F),
                         min.y + px(48.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));

        // Each axis of the frame, as the camera sees it: its components along the view's
        // right and up directions.
        const auto axes = camera.axes();
        const ImVec2 centre(min.x + px(8.0F) + box * 0.5F, min.y + px(24.0F));
        const std::array<ImVec4, 3> colors{theme::hex(0xF0605D), theme::hex(0x4FC97A),
                                           theme::hex(0x5AA2FF)};
        const float length = px(14.0F);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto along = static_cast<float>(axes[0].at(axis));
            const auto upward = static_cast<float>(axes[1].at(axis));
            draw->AddLine(centre, ImVec2(centre.x + along * length, centre.y - upward * length),
                          u32(colors.at(axis)), px(2.0F));
        }
        const float x = min.x + px(8.0F) + box + px(10.0F);
        draw->AddText(font, size, ImVec2(x, min.y + px(7.0F)), u32(c::t2), frame);
        draw->AddText(font, size, ImVec2(x, min.y + px(25.0F)), u32(c::t1), distance);
    }

    static void emptyState(PanelContext& context, const ImVec2& origin, const ImVec2& size) {
        const bool connected = context.connection.state == ConnectionState::Ready;
        const char* message = connected ? "Open a robot or a recording to see it here."
                                        : "Not connected to a simulation server.";
        const ImVec2 text = ImGui::CalcTextSize(message);
        ImGui::SetCursorScreenPos(
            ImVec2(origin.x + (size.x - text.x) * 0.5F, origin.y + size.y * 0.5F - text.y));
        ImGui::TextDisabled("%s", message);
        if (connected && context.canOpenFiles) {
            const float width = widgets::ButtonWidth("Open ...");
            ImGui::SetCursorScreenPos(
                ImVec2(origin.x + (size.x - width) * 0.5F, origin.y + size.y * 0.5F + px(8.0F)));
            if (widgets::Button("Open ...")) {
                context.openFileRequested = true;
            }
        }
    }
};

}  // namespace

std::unique_ptr<Panel> createViewportPanel() {
    return std::make_unique<ViewportPanel>();
}

}  // namespace lagrange::viewer
