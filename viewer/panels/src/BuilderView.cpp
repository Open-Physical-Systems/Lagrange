// Builder workspace, centre: the Builder view (docs/design/ui/12-builder/SPEC.md "Builder
// view"; RFC 0016 WP-D).
//
// The image is the viewer's own SceneRenderer drawing the builder's document on the Lit
// pipeline (BuilderHost::frameToDraw, plan decision 1), with the builder's own camera (R3).
// This panel moves that camera, picks with pickPrim, and draws the overlay text: the frame
// readout, the tool column, the toggles, the triad, snap and the labels. The gizmos of the
// Move, Rotate, Joint axis and Measure tools are BuilderGizmo's.
#include "BuilderGizmo.hpp"
#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Picking.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;
using Kind = BuilderSession::Selection::Kind;

constexpr double kOrbitRadiansPerPixel = 0.006;
constexpr double kZoomPerWheelStep = 0.88;
constexpr float kClickSlopPixels = 4.0F;
/// Fitting the robot in view: the camera stands this many bounding radii away, and never
/// closer than kMinFitDistance (m) (a view choice, not a physical quantity).
constexpr double kFitRadii = 4.0;
constexpr double kMinFitDistance = 1.0;
/// The first view of a document: from the front right, above (rad; the reference image's).
constexpr double kStartYaw = -0.7;
constexpr double kStartPitch = 0.5;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// A text box over the image (design .ov): raised surface, thin border, mono text; `second`
/// on the same line in t2.
ImVec2 overlayBox(ImDrawList* draw, const ImVec2& min, const char* first,
                  const char* second = nullptr, const ImVec4& border = c::lineStrong,
                  const ImVec4& text = c::t1) {
    return builder_ui::viewOverlayBox(draw, min, first, second, &border, &text);
}

/// Where a point lands in the view, if it is in front of the camera and inside the view.
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

bool isCamera(std::string_view type) {
    return type == "camera" || type == "depth" || type == "depth_camera" || type == "rgbd_camera" ||
           type == "thermal" || type == "thermal_camera";
}

std::array<double, 3> rotate(const std::array<double, 4>& q, const std::array<double, 3>& v) {
    const builder::Frame f = builder::compose(builder::Frame{{0, 0, 0}, q}, builder::Frame{v});
    return f.position;
}

class BuilderViewPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kView; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(
            title(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();
        if (!open || context.builder == nullptr) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        refresh(session);

        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        context.requestedViewportWidth =
            static_cast<std::uint32_t>(std::max(available.x * scale.x, 1.0F));
        context.requestedViewportHeight =
            static_cast<std::uint32_t>(std::max(available.y * scale.y, 1.0F));
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        if (!framed_ && available.x > 1.0F && available.y > 1.0F) {
            framed_ = true;
            session.camera.yaw = kStartYaw;
            session.camera.pitch = kStartPitch;
            fit(session.camera, kNone);
        }
        const bool building = session.mode == BuilderSession::Mode::Build;
        // Testing (SPEC state 4): the same view shows the test server's scene, drawn by the
        // renderer with this view's camera and the viewport's shading.
        if (context.viewportTexture != 0) {
            ImGui::Image(static_cast<ImTextureID>(context.viewportTexture), available);
        } else {
            ImGui::Dummy(available);
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        Camera& camera = session.camera;
        const builder_ui::BuilderGizmo::View view{&camera, origin, available};
        const ImGuiIO& io = ImGui::GetIO();
        const bool mouseInView = ImGui::IsWindowHovered() &&
                                 ImGui::IsMouseHoveringRect(origin, ImVec2(origin.x + available.x,
                                                                           origin.y + available.y));
        if (building) {
            gizmo_.prepare(session, links_, view, mouseInView);
        }

        ImGui::SetCursorScreenPos(origin);
        ImGui::SetNextItemAllowOverlap();
        ImGui::InvisibleButton("##builderview", available,
                               ImGuiButtonFlags_MouseButtonLeft |
                                   ImGuiButtonFlags_MouseButtonRight |
                                   ImGuiButtonFlags_MouseButtonMiddle);
        const bool hovered = ImGui::IsItemHovered();
        const double aspect = static_cast<double>(available.x) / std::max(available.y, 1.0F);
        // The gizmo takes the left button when it is pressed on a handle, until let go.
        const bool owned =
            building &&
            gizmo_.handle(session, view, hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left));
        Camera& camera = session.camera;  // also while testing: the server's scene, this camera
        if (ImGui::IsItemActive() && !owned) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                             io.MouseDelta.y * kOrbitRadiansPerPixel);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                camera.pan(-io.MouseDelta.x / std::max(available.y, 1.0F),
                           io.MouseDelta.y / std::max(available.y, 1.0F));
            }
        }
        if (hovered && io.MouseWheel != 0.0F) {
            camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
        }
        // A click that was not a drag selects the link or sensor under it (a second click on the
        // selected link, the shape under it), or nothing; with Measure it places a point.
        if (building && !owned && hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, kClickSlopPixels).x == 0.0F &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, kClickSlopPixels).y == 0.0F) {
            const Ray ray =
                camera.ray((io.MousePos.x - origin.x) / available.x * 2.0 - 1.0,
                           (io.MousePos.y - origin.y) / available.y * 2.0 - 1.0, aspect);
            const std::uint32_t prim = pickPrim(*frame_, ray);
            const bool any = prim != kNone && prim < built_.primItem.size();
            const bool sensor = any && built_.primSensor[prim] >= 0;
            if (building) {
                gizmo_.click(session, links_, builder::gizmo::Line{ray.origin, ray.direction},
                             sensor ? built_.primItem[prim] : std::string{},
                             any && !sensor ? built_.primItem[prim] : std::string{});
            } else {
                select(session, prim);
            }
        }
        const bool active = hovered || ImGui::IsWindowFocused();
        if (building && active && !io.WantTextInput &&
            ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            (void)gizmo_.escape(session);
        }
        shortcuts(session, active);

        // Top left: mode, camera, shading, frame time, draws, "no simulation".
        char text[160];
        std::snprintf(text, sizeof(text), "%s · Perspective · %s   %.1f ms   %u draws",
                      building ? "Build" : "Test", building ? "Lit" : "live",
                      static_cast<double>(context.frameMs), context.drawCalls);
        overlayBox(draw, ImVec2(origin.x + px(10.0F), origin.y + px(10.0F)), text,
                   building ? "no simulation" : "lagrange_server");
        if (!building) {
            // The document's marks do not belong over the simulated robot.
            ImGui::End();
            return;
        }
        if (session.view.names) {
            labels(session, draw, origin, available);
        }
        if (building) {
            gizmo_.draw(draw, session, view);
        }
        triad(draw, camera, session.document(),  // not held across the gizmo's commit
              ImVec2(origin.x + px(10.0F), origin.y + available.y - px(58.0F)));
        snapBox(draw, origin, available, io.KeyCtrl);
        toolColumn(session, ImVec2(origin.x + px(10.0F), origin.y + px(46.0F)));
        toggles(session, ImVec2(origin.x + available.x - px(10.0F), origin.y + px(10.0F)));
        ImGui::End();
    }

private:
    /// The scene as the renderer draws it (the same statics and FK as BuilderHost), for
    /// picking and labels: rebuilt on a document edit, re-posed on a preview change.
    void refresh(const BuilderSession& session) {
        const builder::Document& document = session.document();
        const bool edited =
            session.history.revision() != revision_ || session.opened != opened_ || !built_.statics;
        if (session.opened != opened_) {
            framed_ = false;  // a new document: centre the view on it (SPEC state 1)
        }
        if (edited) {
            revision_ = session.history.revision();
            opened_ = session.opened;
            built_ = builder::buildStatics(document, ++generation_);
        }
        if (edited || !frame_ || session.preview != preview_) {
            preview_ = session.preview;
            links_ = builder::previewFrames(document, preview_);
            frame_ = builder::poseScene(built_, links_);
        }
    }

    void select(BuilderSession& session, std::uint32_t prim) const {
        if (prim == kNone || prim >= built_.primItem.size()) {
            session.selection = {};
            return;
        }
        session.selection.kind = built_.primSensor[prim] >= 0 ? Kind::Sensor : Kind::Link;
        session.selection.name = built_.primItem[prim];
    }

    /// Q W E A M pick the tool; F looks at the selection (or the whole robot); never while a
    /// text field has the keyboard.
    void shortcuts(BuilderSession& session, bool active) {
        const ImGuiIO& io = ImGui::GetIO();
        if (!active || io.WantTextInput || io.KeyCtrl) {
            return;
        }
        using Tool = BuilderSession::Tool;
        const std::array<std::pair<ImGuiKey, Tool>, 5> keys{{{ImGuiKey_Q, Tool::Select},
                                                             {ImGuiKey_W, Tool::Move},
                                                             {ImGuiKey_E, Tool::Rotate},
                                                             {ImGuiKey_A, Tool::JointAxis},
                                                             {ImGuiKey_M, Tool::Measure}}};
        for (const auto& [key, tool] : keys) {
            if (ImGui::IsKeyPressed(key, false) && !gizmo_.dragging()) {
                session.tool = tool;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            fit(session.camera, selectedPrim(session));
        }
    }

    [[nodiscard]] std::uint32_t selectedPrim(const BuilderSession& session) const {
        if (session.selection.kind != Kind::Link && session.selection.kind != Kind::Sensor) {
            return kNone;
        }
        return builder::primOf(built_, session.selection.name);
    }

    /// Looks at `prim` and its children (kNone: the whole robot) from far enough to see it.
    void fit(Camera& camera, std::uint32_t prim) const {
        if (!frame_ || !frame_->statics) {
            return;
        }
        const SceneStatics& statics = *frame_->statics;
        std::array<double, 3> low{};
        std::array<double, 3> high{};
        bool any = false;
        for (std::size_t i = 0; i < statics.prims.size(); ++i) {
            const PrimData& p = statics.prims[i];
            const bool inside = prim == kNone || i == prim || p.parent == prim;
            if (!inside || p.mesh >= statics.meshes.size() ||
                (prim == kNone && p.typeName == "Sensor")) {
                continue;
            }
            const std::array<float, 6>& box = statics.meshes[p.mesh].aabb;
            const Transform& t = frame_->transforms[i];
            for (int k = 0; k < 8; ++k) {
                const std::array<double, 3> corner{(k & 1) != 0 ? box[3] : box[0],
                                                   (k & 2) != 0 ? box[4] : box[1],
                                                   (k & 4) != 0 ? box[5] : box[2]};
                const auto r =
                    rotate({t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]}, corner);
                for (std::size_t a = 0; a < 3; ++a) {
                    const double v = t.position.at(a) + r.at(a);
                    low.at(a) = any ? std::min(low.at(a), v) : v;
                    high.at(a) = any ? std::max(high.at(a), v) : v;
                }
                any = true;
            }
        }
        if (!any) {
            return;
        }
        double radius = 0.0;
        for (std::size_t a = 0; a < 3; ++a) {
            camera.target.at(a) = (low.at(a) + high.at(a)) * 0.5;
            radius = std::max(radius, (high.at(a) - low.at(a)) * 0.5);
        }
        camera.distance = std::max(kMinFitDistance, radius * kFitRadii);
    }

    /// The selected item's name where it is, and every joint previewed away from zero with
    /// its value (SPEC "Labels").
    void labels(const BuilderSession& session, ImDrawList* draw, const ImVec2& origin,
                const ImVec2& size) const {
        const builder::Document& document = session.document();
        const auto linkAt = [&](std::string_view name) -> const builder::Frame* {
            for (std::size_t l = 0; l < document.links.size() && l < links_.size(); ++l) {
                if (document.links[l].name == name) {
                    return &links_[l];
                }
            }
            return nullptr;
        };
        char value[64];
        for (const builder::Joint& joint : document.joints) {
            const bool chosen =
                session.selection.kind == Kind::Joint && session.selection.name == joint.name;
            const auto found = session.preview.find(joint.name);
            const bool previewed = found != session.preview.end() && found->second != 0.0;
            const builder::Frame* frame = linkAt(joint.child);
            ImVec2 at;
            if ((!chosen && !previewed) || frame == nullptr ||
                !project(session.camera, frame->position, origin, size, at)) {
                continue;
            }
            const double q = previewed ? found->second : 0.0;
            if (joint.type == builder::Joint::Type::Prismatic) {
                std::snprintf(value, sizeof(value), "%.3f m", q);
            } else if (session.showDegrees) {
                std::snprintf(value, sizeof(value), "%.1f°", q * kRadToDeg);
            } else {
                std::snprintf(value, sizeof(value), "%.3f rad", q);
            }
            overlayBox(draw, ImVec2(at.x + px(14.0F), at.y - px(28.0F)), joint.name.c_str(), value,
                       chosen ? c::accent : c::lineStrong, chosen ? c::accent : c::t1);
        }
        const std::uint32_t prim = selectedPrim(session);
        if (prim == kNone || !frame_ || prim >= frame_->transforms.size()) {
            return;
        }
        ImVec2 at;
        if (!project(session.camera, frame_->transforms[prim].position, origin, size, at)) {
            return;
        }
        draw->AddCircle(at, px(5.0F), u32(c::accent), 0, px(1.5F));
        std::string detail = "link";
        if (session.selection.kind == Kind::Sensor) {
            const builder::Sensor* sensor = builder::findSensor(document, session.selection.name);
            detail = sensor != nullptr ? sensor->type : "sensor";
            // A range sensor's label sits at the end of its beam, with its range (SPEC). The
            // range is the document's; the catalog's default needs WP-C's catalog.
            const std::string* range = nullptr;
            if (sensor != nullptr && sensor->type != "lidar" && !isCamera(sensor->type)) {
                if (const auto found = sensor->params.find("range/max");
                    found != sensor->params.end()) {
                    range = &found->second;
                }
            }
            if (range != nullptr) {
                const Transform& t = frame_->transforms[prim];
                const double r = std::strtod(range->c_str(), nullptr);
                const auto beam =
                    rotate({t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]}, {r, 0, 0});
                ImVec2 end;
                if (r > 0.0 && project(session.camera,
                                       {t.position[0] + beam[0], t.position[1] + beam[1],
                                        t.position[2] + beam[2]},
                                       origin, size, end)) {
                    at = end;
                }
                detail = *range + " m";
            }
        }
        overlayBox(draw, ImVec2(at.x + px(10.0F), at.y + px(8.0F)), session.selection.name.c_str(),
                   detail.c_str(), c::accent, c::accent);
    }

    /// Bottom left: the axes as the camera sees them, the frame and the grid spacing.
    static void triad(ImDrawList* draw, const Camera& camera, const builder::Document& document,
                      const ImVec2& min) {
        ImFont* font = theme::fonts().mono;
        const float size = px(theme::kMonoSize);
        char frame[96];
        std::snprintf(frame, sizeof(frame), "%s · Z-up · m",
                      document.links.empty() ? "robot" : document.links.front().name.c_str());
        const char* grid = "grid 0.1 m";  // BuilderHost's kGridStep
        const float textWidth = std::max(font->CalcTextSizeA(size, FLT_MAX, 0.0F, frame).x,
                                         font->CalcTextSizeA(size, FLT_MAX, 0.0F, grid).x);
        const float box = px(34.0F);
        const ImVec2 max(min.x + px(8.0F) + box + px(10.0F) + textWidth + px(10.0F),
                         min.y + px(48.0F));
        draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
        draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
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
        draw->AddText(font, size, ImVec2(x, min.y + px(25.0F)), u32(c::t2), grid);
    }

    /// Bottom right: snap steps (builder::gizmo's, the SPEC's defaults) and the Focus key.
    /// Holding Ctrl turns snapping off while it is held (SPEC: "Ctrl toggles").
    static void snapBox(ImDrawList* draw, const ImVec2& origin, const ImVec2& size, bool off) {
        const char* text = off ? "snap  off (Ctrl)   Focus [F]" : "snap  5 mm · 5°   Focus [F]";
        const float width =
            theme::fonts().mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, text).x +
            px(16.0F);
        overlayBox(draw,
                   ImVec2(origin.x + size.x - width - px(10.0F), origin.y + size.y - px(36.0F)),
                   text);
    }

    /// Left: Select (Q), Move (W), Rotate (E), Joint axis (A), Measure (M).
    static void toolColumn(BuilderSession& session, const ImVec2& at) {
        using Tool = BuilderSession::Tool;
        struct Entry {
            const char* id;
            widgets::Icon icon;
            Tool tool;
            const char* tooltip;
        };
        static constexpr std::array<Entry, 5> kTools{{
            {"##bselect", widgets::Icon::Select, Tool::Select,
             "Select (Q): click a link or sensor; click it again for the shape under the mouse"},
            {"##bmove", widgets::Icon::Move, Tool::Move,
             "Move (W): the joint origin, shape or sensor mount; face handles size a shape"},
            {"##brotate", widgets::Icon::Rotate, Tool::Rotate,
             "Rotate (E): the joint origin, shape or sensor mount"},
            {"##baxis", widgets::Icon::JointAxis, Tool::JointAxis,
             "Joint axis (A): drag to set the axis; Shift: any direction"},
            {"##bmeasure", widgets::Icon::Measure, Tool::Measure,
             "Measure (M): the distance between two points"},
        }};
        ImGui::SetCursorScreenPos(at);
        widgets::ToolBarFrame(static_cast<int>(kTools.size()), true);
        const float x = ImGui::GetCursorScreenPos().x;
        float y = ImGui::GetCursorScreenPos().y;
        for (const Entry& entry : kTools) {
            ImGui::SetCursorScreenPos(ImVec2(x, y));
            if (widgets::ToolButton(entry.id, entry.icon, session.tool == entry.tool,
                                    entry.tooltip)) {
                session.tool = entry.tool;
            }
            y += px(26.0F);
        }
    }

    /// Top right: grid, frames and joint axes, collision shapes, COM, sensors, names.
    static void toggles(BuilderSession& session, const ImVec2& topRight) {
        using Toggles = BuilderSession::ViewToggles;
        struct Entry {
            const char* id;
            widgets::Icon icon;
            bool Toggles::* option;
            const char* tooltip;
        };
        // TODO(RFC 0016 WP-E1): the Collision, COM and Names icons of 12-builder/reference.html;
        // the nearest existing icons until then.
        static constexpr std::array<Entry, 6> kToggles{{
            {"##bgrid", widgets::Icon::Grid, &Toggles::grid, "Grid"},
            {"##bframes", widgets::Icon::Frames, &Toggles::frames, "Frames and joint axes"},
            {"##bcollision", widgets::Icon::Stage, &Toggles::collision, "Collision shapes"},
            {"##bcom", widgets::Icon::Forces, &Toggles::com, "Centre of mass"},
            {"##bsensors", widgets::Icon::Frustum, &Toggles::sensors, "Sensor rays and frustums"},
            {"##bnames", widgets::Icon::Component, &Toggles::names, "Names"},
        }};
        const auto count = static_cast<float>(kToggles.size());
        const float width = px(24.0F) * count + px(2.0F) * (count + 1.0F);
        ImGui::SetCursorScreenPos(ImVec2(topRight.x - width, topRight.y));
        widgets::ToolBarFrame(static_cast<int>(kToggles.size()), false);
        float x = ImGui::GetCursorScreenPos().x;
        const float y = ImGui::GetCursorScreenPos().y;
        for (const Entry& entry : kToggles) {
            ImGui::SetCursorScreenPos(ImVec2(x, y));
            bool& on = session.view.*entry.option;
            if (widgets::ToolButton(entry.id, entry.icon, on, entry.tooltip)) {
                on = !on;
            }
            x += px(26.0F);
        }
    }

    bool framed_{false};
    std::uint64_t revision_{~std::uint64_t{0}};
    std::uint64_t opened_{0};
    std::uint64_t generation_{0};
    builder::BuiltScene built_;
    builder::Preview preview_;
    std::vector<builder::Frame> links_;
    std::shared_ptr<const SceneFrame> frame_;
    builder_ui::BuilderGizmo gizmo_;
};

}  // namespace

std::unique_ptr<Panel> createBuilderViewPanel() {
    return std::make_unique<BuilderViewPanel>();
}

}  // namespace lagrange::viewer
