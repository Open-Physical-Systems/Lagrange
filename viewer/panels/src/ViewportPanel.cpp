// Scene workspace, centre: the 3D viewport with its overlays (docs/design/ui/02-scene/SPEC.md).
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
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
    draw->AddRectFilled(min, max, u32(theme::hex(0x0B1422, 0.88F)), px(3.0F));
    draw->AddRect(min, max, u32(border), px(3.0F));
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
        if (ImGui::IsItemHovered()) {
            const ImGuiIO& io = ImGui::GetIO();
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                context.camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                                     io.MouseDelta.y * kOrbitRadiansPerPixel);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                context.camera.pan(-io.MouseDelta.x / std::max(available.y, 1.0F),
                                   io.MouseDelta.y / std::max(available.y, 1.0F));
            }
            if (io.MouseWheel != 0.0F) {
                context.camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
            }
        }

        ImDrawList* draw = ImGui::GetWindowDrawList();
        if (!context.scene) {
            emptyState(context, origin, available);
            ImGui::End();
            return;
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

        // Bottom left: the frame the scene is drawn in and how far the camera is.
        char distance[48];
        std::snprintf(distance, sizeof(distance), "cam %.2f m", context.camera.distance);
        overlayBox(draw, ImVec2(origin.x + px(10.0F), origin.y + available.y - px(54.0F)),
                   "region frame · Z-up · m", distance);

        // The selected prim's name, where it is.
        if (context.selectedPrim < context.scene->statics->prims.size()) {
            ImVec2 at;
            if (project(context.camera, context.scene->transforms[context.selectedPrim].position,
                        origin, available, at)) {
                draw->AddCircle(at, px(5.0F), u32(c::accent), 0, px(1.5F));
                overlayBox(draw, ImVec2(at.x + px(10.0F), at.y + px(8.0F)),
                           context.scene->statics->prims[context.selectedPrim].name.c_str(),
                           nullptr, c::accent, c::accent);
            }
        }
        ImGui::End();
    }

private:
    static void emptyState(PanelContext& context, const ImVec2& origin, const ImVec2& size) {
        const bool connected = context.connection.state == ConnectionState::Ready;
        const char* message = connected ? "Open a robot or a recording to see it here."
                                        : "Not connected to a simulation server.";
        const ImVec2 text = ImGui::CalcTextSize(message);
        ImGui::SetCursorScreenPos(
            ImVec2(origin.x + (size.x - text.x) * 0.5F, origin.y + size.y * 0.5F - text.y));
        ImGui::TextDisabled("%s", message);
        if (connected && context.canOpenFiles) {
            const float width =
                ImGui::CalcTextSize("Open ...").x + ImGui::GetStyle().FramePadding.x * 2.0F;
            ImGui::SetCursorScreenPos(
                ImVec2(origin.x + (size.x - width) * 0.5F, origin.y + size.y * 0.5F + px(8.0F)));
            if (ImGui::Button("Open ...")) {
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
