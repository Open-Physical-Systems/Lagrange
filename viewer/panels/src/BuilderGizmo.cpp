// The Builder view's gizmos (docs/design/ui/12-builder/SPEC.md "Builder view"). See
// BuilderGizmo.hpp.
#include "BuilderGizmo.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <utility>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
namespace g = builder::gizmo;
using g::Vec3;
using widgets::px;
using Kind = BuilderSession::Selection::Kind;
using Tool = BuilderSession::Tool;

/// The gizmo's arm on screen (px): the reference image's arrows are about 44 px long.
constexpr double kArmPixels = 90.0;
/// How close (px) the mouse must be to a handle to take it.
constexpr float kPickPixels = 7.0F;
/// Half the edge of a size handle's square and of the centre square (px; the reference's
/// centre square is 9 px).
constexpr float kSizeHandleHalf = 4.5F;
constexpr int kRingSegments = 64;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;
/// Plane handles sit between two arrows, from this fraction of the arm to the next.
constexpr double kPlaneFrom = 0.22;
constexpr double kPlaneTo = 0.42;
/// A shape's Move arrows reach this many half sizes out of its centre, at least.
constexpr double kArmBeyondShape = 1.45;

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

/// x, y, z in the design's axis colours (the reference image's gizmo and triad).
ImVec4 axisColor(int axis, float alpha = 1.0F) {
    static constexpr std::array<std::uint32_t, 3> kAxes{0xF0605D, 0x4FC97A, 0x5AA2FF};
    return theme::hex(kAxes.at(static_cast<std::size_t>(std::clamp(axis, 0, 2))), alpha);
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 mul(const Vec3& a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 unit(int axis) {
    Vec3 v{};
    v.at(static_cast<std::size_t>(axis)) = 1.0;
    return v;
}
const char* axisName(int axis) {
    static constexpr std::array<const char*, 3> kNames{"x", "y", "z"};
    return kNames.at(static_cast<std::size_t>(std::clamp(axis, 0, 2)));
}

/// The joint whose child `link` is, or nullptr (a root).
const builder::Joint* jointOf(const builder::Document& document, std::string_view link) {
    for (const builder::Joint& joint : document.joints) {
        if (joint.child == link) {
            return &joint;
        }
    }
    return nullptr;
}

int linkIndex(const builder::Document& document, std::string_view link) {
    for (std::size_t l = 0; l < document.links.size(); ++l) {
        if (document.links[l].name == link) {
            return static_cast<int>(l);
        }
    }
    return -1;
}

/// The words for what a target is: "box_link_joint origin", "base_link · box 1", "imu mount".
std::string describe(const builder::Document& document, const g::Target& target) {
    switch (target.kind) {
        case g::Target::Kind::JointOrigin:
            return target.name + " origin";
        case g::Target::Kind::Shape: {
            const builder::Link* link = builder::findLink(document, target.name);
            std::string kind = "shape";
            if (link != nullptr && target.shape >= 0 &&
                static_cast<std::size_t>(target.shape) < link->shapes.size()) {
                kind = std::string(
                    builder::toString(link->shapes[static_cast<std::size_t>(target.shape)].kind));
            }
            return target.name + " · " + kind + " " + std::to_string(target.shape + 1);
        }
        case g::Target::Kind::Sensor:
            return target.name + " mount";
        case g::Target::Kind::None:
            break;
    }
    return {};
}

std::string metres(double value, bool sign = false) {
    char text[32];
    std::snprintf(text, sizeof(text), sign ? "%+.3f m" : "%.3f m", value == 0.0 ? 0.0 : value);
    return text;
}

bool insidePolygon(const ImVec2& p, const std::vector<ImVec2>& polygon) {
    // A convex polygon in either winding: the point is on the same side of every edge.
    int positive = 0;
    int negative = 0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const ImVec2& a = polygon[i];
        const ImVec2& b = polygon[(i + 1) % polygon.size()];
        const float side = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        positive += side > 0.0F ? 1 : 0;
        negative += side < 0.0F ? 1 : 0;
    }
    return positive == 0 || negative == 0;
}

float distanceTo(const ImVec2& p, const std::vector<ImVec2>& outline, bool closed) {
    float best = FLT_MAX;
    const std::size_t n = outline.size();
    for (std::size_t i = 0; i + 1 < n || (closed && i < n); ++i) {
        const ImVec2& a = outline[i];
        const ImVec2& b = outline[(i + 1) % n];
        best = std::min(best, g::distanceToSegment({p.x, p.y}, {a.x, a.y}, {b.x, b.y}));
    }
    return best;
}

void dashedLine(ImDrawList* draw, const ImVec2& a, const ImVec2& b, ImU32 color, float thickness) {
    // The reference's joint axis: dashes 7 px, gaps 5 px.
    const float dash = px(7.0F);
    const float gap = px(5.0F);
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length < 1.0F) {
        return;
    }
    for (float s = 0.0F; s < length; s += dash + gap) {
        const float e = std::min(s + dash, length);
        draw->AddLine(ImVec2(a.x + dx * s / length, a.y + dy * s / length),
                      ImVec2(a.x + dx * e / length, a.y + dy * e / length), color, thickness);
    }
}

}  // namespace

ImVec2 viewOverlayBox(ImDrawList* draw, const ImVec2& min, const char* first, const char* second,
                      const ImVec4* border, const ImVec4* text) {
    ImFont* font = theme::fonts().mono;
    const float size = px(theme::kMonoSize);
    const float pad = px(8.0F);
    const float a = font->CalcTextSizeA(size, FLT_MAX, 0.0F, first).x;
    const float gap = second != nullptr ? px(12.0F) : 0.0F;
    const float b = second != nullptr ? font->CalcTextSizeA(size, FLT_MAX, 0.0F, second).x : 0.0F;
    const ImVec2 max(min.x + a + gap + b + pad * 2.0F, min.y + px(18.0F) + pad);
    draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
    draw->AddRect(min, max, u32(border != nullptr ? *border : c::lineStrong), px(4.0F));
    const float y = min.y + pad * 0.5F + px(2.0F);
    draw->AddText(font, size, ImVec2(min.x + pad, y), u32(text != nullptr ? *text : c::t1), first);
    if (second != nullptr) {
        draw->AddText(font, size, ImVec2(min.x + pad + a + gap, y), u32(c::t2), second);
    }
    return max;
}

// --- view -------------------------------------------------------------------------------------

g::Line BuilderGizmo::View::ray(const ImVec2& mouse) const {
    const double aspect = static_cast<double>(size.x) / std::max(size.y, 1.0F);
    const Ray r = camera->ray((mouse.x - origin.x) / size.x * 2.0 - 1.0,
                              (mouse.y - origin.y) / size.y * 2.0 - 1.0, aspect);
    return g::Line{r.origin, r.direction};
}

bool BuilderGizmo::View::toScreen(const Vec3& point, ImVec2& out) const {
    const Vec3 eye = camera->eye();
    const Mat4 m = camera->viewProjection(static_cast<double>(size.x) / std::max(size.y, 1.0F));
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
    return true;
}

double BuilderGizmo::View::metresPerPixel(const Vec3& point) const {
    const auto axes = camera->axes();
    const double depth = std::max(dot(sub(point, camera->eye()), axes[2]), camera->nearPlane);
    return 2.0 * depth * std::tan(camera->verticalFov * 0.5) / std::max(size.y, 1.0F);
}

// --- handles ----------------------------------------------------------------------------------

void BuilderGizmo::prepare(const BuilderSession& session, const std::vector<builder::Frame>& links,
                           const View& view, bool mouseInView) {
    handles_.clear();
    hovered_ = -1;
    target_ = {};
    jointForAxis_.clear();
    const builder::Document& document = session.document();
    const BuilderSession::Selection& selection = session.selection;
    if (session.mode != BuilderSession::Mode::Build) {
        drag_ = {};
        return;
    }

    // A drilled shape lasts while its link stays selected and the shape exists.
    if (shape_.kind == g::Target::Kind::Shape &&
        (selection.kind != Kind::Link || selection.name != shape_.name ||
         g::poseOf(document, shape_) == nullptr)) {
        shape_ = {};
    }
    const builder::Joint* joint = nullptr;
    if (selection.kind == Kind::Link) {
        joint = jointOf(document, selection.name);
        if (shape_.kind == g::Target::Kind::Shape) {
            target_ = shape_;
        } else if (joint != nullptr) {
            target_ = {g::Target::Kind::JointOrigin, joint->name, -1};
        } else if (const builder::Link* link = builder::findLink(document, selection.name);
                   link != nullptr && link->shapes.size() == 1) {
            target_ = {g::Target::Kind::Shape, link->name, 0};  // a root: its one shape
        }
    } else if (selection.kind == Kind::Joint) {
        joint = builder::findJoint(document, selection.name);
        if (joint != nullptr) {
            target_ = {g::Target::Kind::JointOrigin, joint->name, -1};
        }
    } else if (selection.kind == Kind::Sensor &&
               builder::findSensor(document, selection.name) != nullptr) {
        target_ = {g::Target::Kind::Sensor, selection.name, -1};
    }
    if (joint != nullptr) {
        jointForAxis_ = joint->name;
    }

    const auto screen = [&](const Vec3& p) {
        ImVec2 out(-FLT_MAX, -FLT_MAX);
        (void)view.toScreen(p, out);
        return out;
    };
    const auto square = [&](const Vec3& p, float half) {
        const ImVec2 s = screen(p);
        const float h = px(half);
        return std::vector<ImVec2>{ImVec2(s.x - h, s.y - h), ImVec2(s.x + h, s.y - h),
                                   ImVec2(s.x + h, s.y + h), ImVec2(s.x - h, s.y + h)};
    };

    // Size handles: on the faces of the selected link's shapes (or of the drilled one), with the
    // Select and Move tools.
    if ((session.tool == Tool::Select || session.tool == Tool::Move) &&
        selection.kind == Kind::Link) {
        const int l = linkIndex(document, selection.name);
        if (l >= 0 && static_cast<std::size_t>(l) < links.size()) {
            const builder::Link& link = document.links[static_cast<std::size_t>(l)];
            for (std::size_t s = 0; s < link.shapes.size(); ++s) {
                if (target_.kind == g::Target::Kind::Shape &&
                    target_.shape != static_cast<int>(s)) {
                    continue;
                }
                const builder::Shape& shape = link.shapes[s];
                const builder::Frame frame = builder::compose(links[static_cast<std::size_t>(l)],
                                                              builder::toFrame(shape.origin));
                for (int axis = 0; axis < 3; ++axis) {
                    const auto extent = g::extentOf(shape, axis);
                    if (!extent) {
                        continue;
                    }
                    for (const int sign : {1, -1}) {
                        Handle handle;
                        handle.kind = HandleKind::Size;
                        handle.axis = axis;
                        handle.sign = sign;
                        handle.shape = static_cast<int>(s);
                        handle.dir = g::rotate(frame.rotation, mul(unit(axis), sign));
                        handle.at = add(frame.position, mul(handle.dir, *extent * 0.5));
                        handle.outline = square(handle.at, kSizeHandleHalf);
                        handle.filled = true;
                        handles_.push_back(std::move(handle));
                    }
                }
            }
        }
    }

    if ((session.tool == Tool::Move || session.tool == Tool::Rotate) &&
        target_.kind != g::Target::Kind::None) {
        const auto parent = g::parentFrame(document, links, target_);
        const builder::Pose* pose = g::poseOf(document, target_);
        if (parent && pose != nullptr) {
            frame_ = *parent;
            centre_ = builder::compose(frame_, builder::toFrame(*pose)).position;
            scale_ = view.metresPerPixel(centre_) * px(static_cast<float>(kArmPixels));
            if (target_.kind == g::Target::Kind::Shape) {
                // The arrows reach out of the shape, so their tips are not its face handles.
                const builder::Link* link = builder::findLink(document, target_.name);
                const builder::Shape& shape = link->shapes[static_cast<std::size_t>(target_.shape)];
                for (int k = 0; k < 3; ++k) {
                    scale_ = std::max(scale_,
                                      kArmBeyondShape * 0.5 * g::extentOf(shape, k).value_or(0.0));
                }
            }
            std::array<Vec3, 3> axes{};
            for (int k = 0; k < 3; ++k) {
                axes.at(static_cast<std::size_t>(k)) = g::rotate(frame_.rotation, unit(k));
            }
            if (session.tool == Tool::Move) {
                for (int k = 0; k < 3; ++k) {
                    const Vec3& a = axes.at(static_cast<std::size_t>(k));
                    // Plane handles, normal k, between the other two arrows.
                    const Vec3& u = axes.at(static_cast<std::size_t>((k + 1) % 3));
                    const Vec3& v = axes.at(static_cast<std::size_t>((k + 2) % 3));
                    Handle plane;
                    plane.kind = HandleKind::Plane;
                    plane.axis = k;
                    plane.at = centre_;
                    plane.dir = a;
                    for (const auto& [i, j] :
                         {std::pair{kPlaneFrom, kPlaneFrom}, std::pair{kPlaneTo, kPlaneFrom},
                          std::pair{kPlaneTo, kPlaneTo}, std::pair{kPlaneFrom, kPlaneTo}}) {
                        plane.outline.push_back(
                            screen(add(centre_, add(mul(u, i * scale_), mul(v, j * scale_)))));
                    }
                    plane.filled = true;
                    handles_.push_back(std::move(plane));
                    Handle arrow;
                    arrow.kind = HandleKind::Axis;
                    arrow.axis = k;
                    arrow.at = centre_;
                    arrow.dir = a;
                    arrow.outline = {screen(centre_), screen(add(centre_, mul(a, scale_)))};
                    handles_.push_back(std::move(arrow));
                }
                Handle free;
                free.kind = HandleKind::Free;
                free.at = centre_;
                free.dir = view.camera->axes()[2];
                free.outline = square(centre_, kSizeHandleHalf);
                free.filled = true;
                handles_.push_back(std::move(free));
            } else {
                for (int k = 0; k < 3; ++k) {
                    const Vec3& u = axes.at(static_cast<std::size_t>((k + 1) % 3));
                    const Vec3& v = axes.at(static_cast<std::size_t>((k + 2) % 3));
                    Handle ring;
                    ring.kind = HandleKind::Ring;
                    ring.axis = k;
                    ring.at = centre_;
                    ring.dir = axes.at(static_cast<std::size_t>(k));
                    for (int i = 0; i < kRingSegments; ++i) {
                        const double angle = 2.0 * std::numbers::pi * i / kRingSegments;
                        ring.outline.push_back(
                            screen(add(centre_, add(mul(u, std::cos(angle) * scale_),
                                                    mul(v, std::sin(angle) * scale_)))));
                    }
                    handles_.push_back(std::move(ring));
                }
            }
        }
    }

    if (session.tool == Tool::JointAxis && !jointForAxis_.empty()) {
        const builder::Joint* axisJoint = builder::findJoint(document, jointForAxis_);
        const g::Target target{g::Target::Kind::JointOrigin, jointForAxis_, -1};
        const auto parent = g::parentFrame(document, links, target);
        if (axisJoint != nullptr && parent) {
            frame_ = builder::compose(*parent, builder::toFrame(axisJoint->origin));
            centre_ = frame_.position;
            scale_ = view.metresPerPixel(centre_) * px(static_cast<float>(kArmPixels));
            Vec3 axis = axisJoint->axis;
            const double norm = std::sqrt(dot(axis, axis));
            axis = norm > 0.0 ? mul(axis, 1.0 / norm) : Vec3{1, 0, 0};
            Handle arrow;
            arrow.kind = HandleKind::JointAxis;
            arrow.axis = -1;
            arrow.at = centre_;
            arrow.dir = g::rotate(frame_.rotation, axis);
            arrow.outline = {screen(centre_), screen(add(centre_, mul(arrow.dir, scale_)))};
            handles_.push_back(std::move(arrow));
            // The sphere the axis turns in, as three great circles in the joint frame: dragging
            // anywhere on them sets the axis too.
            for (int k = 0; k < 3; ++k) {
                const Vec3 u = g::rotate(frame_.rotation, unit((k + 1) % 3));
                const Vec3 v = g::rotate(frame_.rotation, unit((k + 2) % 3));
                Handle ring;
                ring.kind = HandleKind::JointAxis;
                ring.axis = k;
                ring.at = centre_;
                for (int i = 0; i < kRingSegments; ++i) {
                    const double angle = 2.0 * std::numbers::pi * i / kRingSegments;
                    ring.outline.push_back(
                        screen(add(centre_, add(mul(u, std::cos(angle) * scale_),
                                                mul(v, std::sin(angle) * scale_)))));
                }
                handles_.push_back(std::move(ring));
            }
        }
    }

    if (!mouseInView || dragging()) {
        return;
    }
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    float best = px(kPickPixels);
    for (std::size_t i = 0; i < handles_.size(); ++i) {
        const Handle& handle = handles_[i];
        const bool closed = handle.filled || handle.kind == HandleKind::Ring ||
                            (handle.kind == HandleKind::JointAxis && handle.axis >= 0);
        float distance = distanceTo(mouse, handle.outline, closed);
        if (handle.filled && insidePolygon(mouse, handle.outline)) {
            distance = 0.0F;
        }
        // Small square handles win over lines through them at the same distance.
        if (distance < best || (distance <= best && handle.filled)) {
            best = distance;
            hovered_ = static_cast<int>(i);
        }
    }
}

// --- the mouse --------------------------------------------------------------------------------

bool BuilderGizmo::handle(BuilderSession& session, const View& view, bool pressed) {
    if (session.mode != BuilderSession::Mode::Build) {
        return false;
    }
    if (dragging()) {
        if (session.history.revision() != drag_.revision) {
            drag_ = {};  // the document changed under the drag (Ctrl Z): the drag is over
            pressOwned_ = ImGui::IsMouseDown(ImGuiMouseButton_Left);
            return true;
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            drag_ = {};  // let go: the step stays as committed
            pressOwned_ = false;
            return true;
        }
        updateDrag(session, view);
        return true;
    }
    if (pressed && hovered_ >= 0) {
        startDrag(session, handles_[static_cast<std::size_t>(hovered_)], view);
        pressOwned_ = true;
        return true;
    }
    if (pressOwned_) {
        pressOwned_ = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        return true;
    }
    return false;
}

void BuilderGizmo::startDrag(BuilderSession& session, const Handle& handle, const View& view) {
    const builder::Document& document = session.document();
    drag_ = {};
    drag_.kind = handle.kind;
    drag_.handle = handle;
    drag_.start = document;
    drag_.ray0 = view.ray(ImGui::GetIO().MousePos);
    drag_.revision = session.history.revision();
    drag_.frame = frame_;
    switch (handle.kind) {
        case HandleKind::Size: {
            drag_.target = {g::Target::Kind::Shape, session.selection.name, handle.shape};
            shape_ = drag_.target;  // the gizmo follows the shape being sized
            drag_.t0 = g::alongAxis(drag_.ray0, handle.at, handle.dir).value_or(0.0);
            break;
        }
        case HandleKind::JointAxis:
            drag_.target = {g::Target::Kind::JointOrigin, jointForAxis_, -1};
            break;
        case HandleKind::Axis:
            drag_.target = target_;
            drag_.t0 = g::alongAxis(drag_.ray0, handle.at, handle.dir).value_or(0.0);
            break;
        case HandleKind::Ring: {
            drag_.target = target_;
            drag_.reference = g::rotate(frame_.rotation, unit((handle.axis + 1) % 3));
            drag_.lastAngle =
                g::angleOnRing(drag_.ray0, handle.at, handle.dir, drag_.reference).value_or(0.0);
            break;
        }
        default:
            drag_.target = target_;
            break;
    }
}

void BuilderGizmo::updateDrag(BuilderSession& session, const View& view) {
    const ImGuiIO& io = ImGui::GetIO();
    const bool snapping = !io.KeyCtrl;  // SPEC: Ctrl toggles snapping
    const double step = snapping ? g::kMoveStep : 0.0;
    const double angleStep = snapping ? g::kAngleStep : 0.0;
    const g::Line ray = view.ray(io.MousePos);
    const Handle& handle = drag_.handle;
    builder::Document next = drag_.start;
    const std::string item = describe(drag_.start, drag_.target);
    std::string label;
    char value[96];

    if (handle.kind == HandleKind::Size) {
        builder::Link* link = builder::findLink(next, drag_.target.name);
        const auto t = g::alongAxis(ray, handle.at, handle.dir);
        if (link == nullptr || !t || drag_.target.shape < 0 ||
            static_cast<std::size_t>(drag_.target.shape) >= link->shapes.size()) {
            return;
        }
        builder::Shape& shape = link->shapes[static_cast<std::size_t>(drag_.target.shape)];
        const builder::Shape before = shape;
        (void)g::resize(shape, handle.axis, handle.sign, *t - drag_.t0, step);
        const bool radius = shape.kind == builder::Shape::Kind::Sphere ||
                            (shape.kind == builder::Shape::Kind::Cylinder && handle.axis != 2);
        const std::size_t slot = shape.kind == builder::Shape::Kind::Box
                                     ? static_cast<std::size_t>(handle.axis)
                                 : radius ? 0U
                                          : 1U;
        // What the file holds and Properties calls it: a box's edge, a cylinder's length
        // (along its z) or radius, a sphere's radius.
        drag_.readout = radius ? std::string("radius")
                        : shape.kind == builder::Shape::Kind::Cylinder
                            ? std::string("length")
                            : std::string("size ") + axisName(handle.axis);
        std::snprintf(value, sizeof(value), "%.3f → %.3f m", before.size.at(slot),
                      shape.size.at(slot));
        drag_.readoutValue = value;
        label = "Resize " + item + " " + drag_.readout + " " + metres(shape.size.at(slot));
    } else if (handle.kind == HandleKind::JointAxis) {
        builder::Joint* joint = builder::findJoint(next, drag_.target.name);
        if (joint == nullptr) {
            return;
        }
        // Where the mouse ray meets the sphere of the gizmo's radius (or passes nearest to it).
        const Vec3 to = sub(handle.at, ray.origin);
        const double along = dot(to, ray.direction);
        Vec3 point = add(ray.origin, mul(ray.direction, along));
        const Vec3 off = sub(point, handle.at);
        const double miss = dot(off, off);
        if (miss < scale_ * scale_) {
            point = add(ray.origin, mul(ray.direction, along - std::sqrt(scale_ * scale_ - miss)));
        }
        const Vec3 world = sub(point, handle.at);
        const builder::Frame origin = builder::toFrame(joint->origin);
        const g::Quat back{drag_.frame.rotation[0], -drag_.frame.rotation[1],
                           -drag_.frame.rotation[2], -drag_.frame.rotation[3]};
        joint->axis = g::snapAxis(g::rotate(back, world), origin.rotation, io.KeyShift);
        drag_.readout = "axis";
        std::snprintf(value, sizeof(value), "%.3g %.3g %.3g", joint->axis[0] + 0.0,
                      joint->axis[1] + 0.0, joint->axis[2] + 0.0);
        drag_.readoutValue = value;
        label = "Set " + joint->name + " axis " + value;
    } else {
        builder::Pose* pose = g::poseOf(next, drag_.target);
        const builder::Pose* start = g::poseOf(drag_.start, drag_.target);
        if (pose == nullptr || start == nullptr) {
            return;
        }
        if (handle.kind == HandleKind::Axis) {
            const auto t = g::alongAxis(ray, handle.at, handle.dir);
            if (!t) {
                return;
            }
            *pose = g::moved(*start, handle.axis, *t - drag_.t0, step);
            const auto k = static_cast<std::size_t>(handle.axis);
            drag_.readout = std::string(axisName(handle.axis)) + " " +
                            metres(pose->xyz.at(k) - start->xyz.at(k), true);
            drag_.readoutValue = "→ " + metres(pose->xyz.at(k));
            label = "Move " + item + " " + drag_.readout;
        } else if (handle.kind == HandleKind::Plane || handle.kind == HandleKind::Free) {
            const auto from = g::onPlane(drag_.ray0, handle.at, handle.dir);
            const auto to = g::onPlane(ray, handle.at, handle.dir);
            if (!from || !to) {
                return;
            }
            const Vec3 delta = sub(*to, *from);
            for (int k = 0; k < 3; ++k) {
                if (handle.kind == HandleKind::Plane && k == handle.axis) {
                    continue;
                }
                const Vec3 a = g::rotate(drag_.frame.rotation, unit(k));
                *pose = g::moved(*pose, k, dot(delta, a), step);
            }
            drag_.readout = "Δ";
            std::snprintf(value, sizeof(value), "%+.3f %+.3f %+.3f m",
                          pose->xyz[0] - start->xyz[0] + 0.0, pose->xyz[1] - start->xyz[1] + 0.0,
                          pose->xyz[2] - start->xyz[2] + 0.0);
            drag_.readoutValue = value;
            label = "Move " + item;
        } else if (handle.kind == HandleKind::Ring) {
            const auto angle = g::angleOnRing(ray, handle.at, handle.dir, drag_.reference);
            if (angle) {
                double turn = *angle - drag_.lastAngle;
                turn -= 2.0 * std::numbers::pi * std::round(turn / (2.0 * std::numbers::pi));
                drag_.angle += turn;
                drag_.lastAngle = *angle;
            }
            *pose = g::rotated(*start, handle.axis, drag_.angle, angleStep);
            const double shown = g::snap(drag_.angle, angleStep) * kRadToDeg;
            drag_.readout = axisName(handle.axis);
            std::snprintf(value, sizeof(value), "%+.1f°", shown == 0.0 ? 0.0 : shown);
            drag_.readoutValue = value;
            label = "Rotate " + item + " " + drag_.readout + " " + value;
        }
    }
    apply(session, std::move(next), label);
}

void BuilderGizmo::apply(BuilderSession& session, builder::Document next,
                         const std::string& label) {
    const std::string wanted = g::geometryJson(next);
    if (wanted == g::geometryJson(session.document())) {
        return;  // the snapped values did not change
    }
    if (drag_.committed) {
        (void)session.history.undo();  // this drag's step is replaced, never added to
        drag_.committed = false;
    }
    if (wanted != g::geometryJson(drag_.start)) {
        if (drag_.before.empty()) {
            drag_.before = session.history.checkpoint();  // before its first step
        }
        session.history.commit(std::move(next), label, {}, ImGui::GetTime());
        drag_.committed = true;
    } else {
        session.history.restore(drag_.before);  // back where it began: redo as it was
    }
    drag_.revision = session.history.revision();
}

bool BuilderGizmo::escape(BuilderSession& session) {
    if (dragging()) {
        if (session.history.revision() == drag_.revision) {
            // Cancelled: undo and redo exactly as before the drag (its step never redoable).
            session.history.restore(drag_.before);
        }
        drag_ = {};
        pressOwned_ = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        return true;
    }
    if (!measure_.empty()) {
        measure_.clear();
        return true;
    }
    if (shape_.kind != g::Target::Kind::None) {
        shape_ = {};
        return true;
    }
    return false;
}

void BuilderGizmo::click(BuilderSession& session, const std::vector<builder::Frame>& links,
                         const g::Line& ray, const std::string& pickedSensor,
                         const std::string& pickedLink) {
    const builder::Document& document = session.document();
    const auto hit = g::pickShape(document, links, ray, session.view.collision);
    if (session.tool == Tool::Measure) {
        std::optional<Vec3> point;
        if (hit) {
            point = hit->point;
        } else {
            point = g::onPlane(ray, {0, 0, 0}, {0, 0, 1});  // the root link's ground plane
        }
        if (!point) {
            return;
        }
        if (measure_.size() >= 2) {
            measure_.clear();
        }
        measure_.push_back(*point);
        return;
    }
    if (!pickedSensor.empty()) {
        session.selection = {Kind::Sensor, pickedSensor};
        shape_ = {};
        return;
    }
    if (hit) {
        const std::string& name = document.links[static_cast<std::size_t>(hit->link)].name;
        if (session.selection.kind == Kind::Link && session.selection.name == name) {
            shape_ = {g::Target::Kind::Shape, name, hit->shape};  // a second click: the shape
        } else {
            session.selection = {Kind::Link, name};
            shape_ = {};
        }
        return;
    }
    shape_ = {};
    if (!pickedLink.empty()) {
        session.selection = {Kind::Link, pickedLink};
    } else {
        session.selection = {};
    }
}

// --- drawing ----------------------------------------------------------------------------------

void BuilderGizmo::draw(ImDrawList* draw, const BuilderSession& session, const View& view) const {
    if (session.mode != BuilderSession::Mode::Build) {
        return;
    }
    const ImU32 dark = u32(theme::hex(0x0A1322));
    const auto active = [&](std::size_t i) {
        const Handle& h = handles_[i];
        if (dragging()) {
            const Handle& d = drag_.handle;
            return d.kind == h.kind && d.axis == h.axis && d.sign == h.sign && d.shape == h.shape;
        }
        return static_cast<int>(i) == hovered_;
    };
    for (std::size_t i = 0; i < handles_.size(); ++i) {
        const Handle& h = handles_[i];
        const bool hot = active(i);
        const ImU32 color = u32(hot ? c::t1 : axisColor(h.axis));
        switch (h.kind) {
            case HandleKind::Axis:
                draw->AddLine(h.outline[0], h.outline[1], color, px(hot ? 3.5F : 2.5F));
                draw->AddCircleFilled(h.outline[1], px(hot ? 4.5F : 3.5F), color);
                break;
            case HandleKind::Plane:
                draw->AddConvexPolyFilled(
                    h.outline.data(), static_cast<int>(h.outline.size()),
                    u32(hot ? theme::hex(0xE6EBF2, 0.55F) : axisColor(h.axis, 0.35F)));
                draw->AddPolyline(h.outline.data(), static_cast<int>(h.outline.size()), color,
                                  ImDrawFlags_Closed, px(1.0F));
                break;
            case HandleKind::Free:
                draw->AddConvexPolyFilled(h.outline.data(), static_cast<int>(h.outline.size()),
                                          u32(hot ? c::t1 : c::accent));
                draw->AddPolyline(h.outline.data(), static_cast<int>(h.outline.size()), dark,
                                  ImDrawFlags_Closed, px(1.0F));
                break;
            case HandleKind::Ring:
                draw->AddPolyline(h.outline.data(), static_cast<int>(h.outline.size()), color,
                                  ImDrawFlags_Closed, px(hot ? 3.5F : 2.0F));
                break;
            case HandleKind::Size:
                draw->AddConvexPolyFilled(h.outline.data(), static_cast<int>(h.outline.size()),
                                          color);
                draw->AddPolyline(h.outline.data(), static_cast<int>(h.outline.size()), dark,
                                  ImDrawFlags_Closed, px(1.0F));
                break;
            case HandleKind::JointAxis:
                if (h.axis < 0) {
                    const ImU32 axisColour = u32(hot ? c::t1 : c::accent);
                    dashedLine(draw, h.outline[0], h.outline[1], axisColour, px(2.0F));
                    // The arrow head: a triangle at the tip, along the line on screen.
                    const float dx = h.outline[1].x - h.outline[0].x;
                    const float dy = h.outline[1].y - h.outline[0].y;
                    const float n = std::max(std::sqrt(dx * dx + dy * dy), 1.0F);
                    const ImVec2 f(dx / n, dy / n);
                    const ImVec2 s(-f.y, f.x);
                    const ImVec2 tip = h.outline[1];
                    const float l = px(11.0F);
                    const float w = px(6.0F);
                    draw->AddTriangleFilled(
                        ImVec2(tip.x + f.x * l * 0.4F, tip.y + f.y * l * 0.4F),
                        ImVec2(tip.x - f.x * l * 0.6F + s.x * w, tip.y - f.y * l * 0.6F + s.y * w),
                        ImVec2(tip.x - f.x * l * 0.6F - s.x * w, tip.y - f.y * l * 0.6F - s.y * w),
                        axisColour);
                } else {
                    draw->AddPolyline(h.outline.data(), static_cast<int>(h.outline.size()),
                                      u32(hot ? c::t1 : axisColor(h.axis, 0.55F)),
                                      ImDrawFlags_Closed, px(hot ? 2.0F : 1.0F));
                }
                break;
            case HandleKind::None:
                break;
        }
    }

    // Measure: the points, the line between them and the distance.
    if (session.tool == Tool::Measure) {
        std::array<ImVec2, 2> at{};
        for (std::size_t i = 0; i < measure_.size(); ++i) {
            (void)view.toScreen(measure_[i], at.at(i));
            draw->AddCircleFilled(at.at(i), px(4.0F), u32(c::accent));
            draw->AddCircle(at.at(i), px(4.0F), dark, 0, px(1.0F));
        }
        if (measure_.size() == 2) {
            draw->AddLine(at[0], at[1], u32(c::accent), px(1.5F));
            const Vec3 d = sub(measure_[1], measure_[0]);
            char first[48];
            char second[96];
            std::snprintf(first, sizeof(first), "%.4f m", std::sqrt(dot(d, d)));
            std::snprintf(second, sizeof(second), "Δ %.3f · %.3f · %.3f", d[0] + 0.0, d[1] + 0.0,
                          d[2] + 0.0);
            viewOverlayBox(draw,
                           // above the line's middle, clear of the points
                           ImVec2((at[0].x + at[1].x) * 0.5F + px(8.0F),
                                  std::min(at[0].y, at[1].y) - px(36.0F)),
                           first, second, &c::accent, &c::accent);
        }
    }

    // What the tool acts on; the drag's value by the mouse.
    std::string hint;
    const builder::Document& document = session.document();
    switch (session.tool) {
        case Tool::Select:
            break;
        case Tool::Move:
        case Tool::Rotate:
            hint = std::string(session.tool == Tool::Move ? "Move · " : "Rotate · ") +
                   (target_.kind != g::Target::Kind::None
                        ? describe(document, target_)
                        : std::string("select a link, joint or sensor"));
            break;
        case Tool::JointAxis:
            hint = jointForAxis_.empty() ? std::string("Joint axis · select a joint")
                                         : "Joint axis · " + jointForAxis_ +
                                               " · snaps to x y z · Shift: any direction";
            break;
        case Tool::Measure:
            hint = measure_.size() == 1 ? "Measure · click the second point"
                                        : "Measure · click two points · Esc clears";
            break;
    }
    if (!hint.empty()) {
        const float width =
            theme::fonts()
                .mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, hint.c_str())
                .x +
            px(16.0F);
        // Bottom centre, between the triad and the snap box (the top is the frame readout's).
        viewOverlayBox(draw,
                       ImVec2(view.origin.x + (view.size.x - width) * 0.5F,
                              view.origin.y + view.size.y - px(36.0F)),
                       hint.c_str());
    }
    if (dragging() && !drag_.readout.empty()) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        viewOverlayBox(draw, ImVec2(mouse.x + px(16.0F), mouse.y + px(14.0F)),
                       drag_.readout.c_str(), drag_.readoutValue.c_str(), &c::accent, &c::accent);
    }
}

}  // namespace lagrange::viewer::builder_ui
