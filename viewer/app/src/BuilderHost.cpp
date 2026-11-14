#include "BuilderHost.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <numbers>
#include <string_view>
#include <variant>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using V3 = std::array<double, 3>;

// Sizes of the overlay marks (m): what reads well around a robot of half a metre. View marks
// only, not physical quantities.
constexpr double kFrameAxisLength = 0.06;     // each link's x y z
constexpr double kSelectedAxisMargin = 0.05;  // the selection's frame reaches this past its shapes
constexpr double kJointAxisLength = 0.15;     // a moving joint's axis, both ways from its origin
constexpr double kDash = 0.04;                // dashed rays: dash and gap
constexpr int kCircleSegments = 48;
// The Builder view's own grid (the renderer's is 1 m, too coarse for a robot this size): lines
// every kGridStep, a major line every kGridMajorEvery, kGridHalfCells to each side, on the
// plane under the robot's lowest point.
constexpr double kGridStep = 0.1;
constexpr int kGridMajorEvery = 5;
constexpr int kGridHalfCells = 20;
/// Shown when a sensor's range is not in the document or the catalog (the catalog is WP-C's).
constexpr double kUnknownRange = 1.0;
/// Sensors that are not selected draw their reach only this far (m), so a 30 m lidar ring or a
/// 20 m frustum does not cover the robot; the selected one draws to its full range (SPEC).
constexpr double kUnselectedReach = 1.0;

std::array<float, 4> rgba(const ImVec4& color, float alpha = 1.0F) {
    return {color.x, color.y, color.z, color.w * alpha};
}

V3 rotate(const std::array<double, 4>& q, const V3& v) {
    // v + 2 w (u x v) + 2 u x (u x v), u = q's vector part.
    const V3 u{q[1], q[2], q[3]};
    const auto cross = [](const V3& a, const V3& b) {
        return V3{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    const V3 t = cross(u, v);
    const V3 t2{2.0 * t[0], 2.0 * t[1], 2.0 * t[2]};
    const V3 r = cross(u, t2);
    return {v[0] + q[0] * t2[0] + r[0], v[1] + q[0] * t2[1] + r[1], v[2] + q[0] * t2[2] + r[2]};
}

V3 apply(const builder::Frame& f, const V3& p) {
    const V3 r = rotate(f.rotation, p);
    return {f.position[0] + r[0], f.position[1] + r[1], f.position[2] + r[2]};
}

/// Lines in one colour, appended segment by segment in a frame.
struct Pen {
    OverlayLines& lines;
    builder::Frame frame;
    void segment(const V3& a, const V3& b) const {
        for (const V3& p : {apply(frame, a), apply(frame, b)}) {
            for (std::size_t k = 0; k < 3; ++k) {
                lines.vertices.push_back(static_cast<float>(p.at(k) - lines.origin.at(k)));
            }
        }
    }
    void dashed(const V3& a, const V3& b) const {
        const V3 d{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double length = std::hypot(d[0], d[1], d[2]);
        const int dashes = std::max(1, static_cast<int>(length / (2.0 * kDash)));
        for (int i = 0; i < dashes; ++i) {
            const double t0 = static_cast<double>(2 * i) * kDash / std::max(length, 1e-9);
            const double t1 = std::min(1.0, t0 + kDash / std::max(length, 1e-9));
            segment({a[0] + d[0] * t0, a[1] + d[1] * t0, a[2] + d[2] * t0},
                    {a[0] + d[0] * t1, a[1] + d[1] * t1, a[2] + d[2] * t1});
        }
    }
    /// A circle of `radius` about the origin in the plane spanned by `u` and `v`, between two
    /// angles (rad).
    void arc(double radius, const V3& u, const V3& v, double from = 0.0,
             double to = 2.0 * std::numbers::pi) const {
        const auto at = [&](double angle) {
            const double cu = radius * std::cos(angle);
            const double sv = radius * std::sin(angle);
            return V3{u[0] * cu + v[0] * sv, u[1] * cu + v[1] * sv, u[2] * cu + v[2] * sv};
        };
        const int steps =
            std::max(4, static_cast<int>(kCircleSegments * (to - from) / (2.0 * std::numbers::pi)));
        for (int i = 0; i < steps; ++i) {
            segment(at(from + (to - from) * i / steps), at(from + (to - from) * (i + 1) / steps));
        }
    }
};

/// The edges of a shape in its link's frame: a box's twelve, a cylinder's two rims and four
/// sides, a sphere's three great circles.
void shapeWire(const Pen& link, const builder::Shape& shape) {
    const Pen pen{link.lines, builder::compose(link.frame, builder::toFrame(shape.origin))};
    switch (shape.kind) {
        case builder::Shape::Kind::Box: {
            const V3 h{shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2};
            for (int axis = 0; axis < 3; ++axis) {
                const int u = (axis + 1) % 3;
                const int v = (axis + 2) % 3;
                for (const double su : {-1.0, 1.0}) {
                    for (const double sv : {-1.0, 1.0}) {
                        V3 a{};
                        a.at(u) = su * h.at(u);
                        a.at(v) = sv * h.at(v);
                        V3 b = a;
                        a.at(axis) = -h.at(axis);
                        b.at(axis) = h.at(axis);
                        pen.segment(a, b);
                    }
                }
            }
            break;
        }
        case builder::Shape::Kind::Cylinder: {
            const double r = shape.size[0];
            const double z = shape.size[1] / 2;
            for (const double side : {-z, z}) {
                const Pen rim{pen.lines, builder::compose(pen.frame, builder::Frame{{0, 0, side}})};
                rim.arc(r, {1, 0, 0}, {0, 1, 0});
            }
            for (int i = 0; i < 4; ++i) {
                const double a = std::numbers::pi / 2 * i;
                pen.segment({r * std::cos(a), r * std::sin(a), -z},
                            {r * std::cos(a), r * std::sin(a), z});
            }
            break;
        }
        case builder::Shape::Kind::Sphere: {
            const double r = shape.size[0];
            pen.arc(r, {1, 0, 0}, {0, 1, 0});
            pen.arc(r, {1, 0, 0}, {0, 0, 1});
            pen.arc(r, {0, 1, 0}, {0, 0, 1});
            break;
        }
        case builder::Shape::Kind::Mesh:
            break;  // mesh files are not drawn yet (SceneAdapter.cpp)
    }
}

/// A sensor parameter as a number: the document's text, else the catalog's default.
std::optional<double> parameter(const builder::Sensor& sensor,
                                const builder::SensorCatalog& catalog, std::string_view path) {
    std::string_view text;
    if (const auto found = sensor.params.find(path); found != sensor.params.end()) {
        text = found->second;
    } else if (const builder::CatalogEntry* entry = catalog.find(sensor.type)) {
        for (const builder::CatalogParam& param : entry->params) {
            if (param.path == path) {
                text = param.defaultText;
            }
        }
    }
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || error != std::errc{} || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

bool isLidar(std::string_view type) {
    return type == "lidar" || type == "gpu_lidar" || type == "ray" || type == "gpu_ray";
}
bool isCamera(std::string_view type) {
    return type == "camera" || type == "depth" || type == "depth_camera" || type == "rgbd_camera" ||
           type == "thermal" || type == "thermal_camera";
}

/// What a sensor sees, in its mount frame (x forward): a lidar's scan arc at its range, a
/// camera's frustum to its far clip, any other a ray along +x to its range (dashed). An IMU
/// draws nothing more than its marker.
void sensorReach(const Pen& pen, const builder::Sensor& sensor,
                 const builder::SensorCatalog& catalog, double limit) {
    if (sensor.type == "imu" || sensor.type == "gnss" || sensor.type == "navsat" ||
        sensor.type == "magnetometer" || sensor.type == "altimeter" ||
        sensor.type == "air_pressure") {
        return;
    }
    if (isLidar(sensor.type)) {
        const double range =
            std::min(limit, parameter(sensor, catalog, "range/max").value_or(kUnknownRange));
        const double from =
            parameter(sensor, catalog, "scan/horizontal/min_angle").value_or(-std::numbers::pi);
        const double to =
            parameter(sensor, catalog, "scan/horizontal/max_angle").value_or(std::numbers::pi);
        pen.arc(range, {1, 0, 0}, {0, 1, 0}, from, to);
        for (const double a : {from, to, 0.0}) {
            pen.dashed({0, 0, 0}, {range * std::cos(a), range * std::sin(a), 0});
        }
        return;
    }
    if (isCamera(sensor.type)) {
        const double far =
            std::min(limit, parameter(sensor, catalog, "clip/far").value_or(kUnknownRange));
        const double fov = parameter(sensor, catalog, "horizontal_fov").value_or(1.047);
        const double width = parameter(sensor, catalog, "image/width").value_or(4.0);
        const double height = parameter(sensor, catalog, "image/height").value_or(3.0);
        const double y = far * std::tan(fov / 2.0);
        const double z = y * height / std::max(width, 1.0);
        const std::array<V3, 4> corners{V3{far, y, z}, V3{far, -y, z}, V3{far, -y, -z},
                                        V3{far, y, -z}};
        for (std::size_t i = 0; i < 4; ++i) {
            pen.segment({0, 0, 0}, corners.at(i));
            pen.segment(corners.at(i), corners.at((i + 1) % 4));
        }
        return;
    }
    const double range =
        std::min(limit, parameter(sensor, catalog, "range/max").value_or(kUnknownRange));
    pen.dashed({0, 0, 0}, {range, 0, 0});
}

bool sameSelection(const BuilderSession::Selection& a, const BuilderSession::Selection& b) {
    return a.kind == b.kind && a.name == b.name;
}

bool sameToggles(const BuilderSession::ViewToggles& a, const BuilderSession::ViewToggles& b) {
    return a.grid == b.grid && a.frames == b.frames && a.collision == b.collision &&
           a.com == b.com && a.sensors == b.sensors && a.names == b.names;
}

}  // namespace

BuilderHost::BuilderHost(BuilderSession& session, ViewerClient& client)
    : session_(session), client_(client) {
    auto loaded = builder::SensorCatalog::load(builder::catalogFiles());
    if (auto* catalog = std::get_if<builder::SensorCatalog>(&loaded)) {
        session_.catalog = std::move(*catalog);
    } else {
        const auto& error = std::get<builder::SensorCatalog::LoadError>(loaded);
        session_.catalogError =
            error.file + ":" + std::to_string(error.line) + ": " + error.message;
    }
}

void BuilderHost::afterUi() {
    // Validation runs on a document change, not every frame (R15).
    if (session_.history.revision() != validatedRevision_) {
        validatedRevision_ = session_.history.revision();
        session_.problems.builder = builder::validate(
            session_.document(), &session_.catalog,
            session_.path.empty() ? std::filesystem::path{} : session_.path.parent_path());
    }
    // WP-F carries these out. Until then they are dropped, so none stays set across frames.
    session_.testRequested = false;
    session_.backRequested = false;
    session_.saveRequested = false;
    session_.openRequested = false;
    session_.exportUsdRequested = false;
    session_.checkRequested = false;
    session_.templateRequested = false;
    (void)client_;
}

std::shared_ptr<const SceneFrame> BuilderHost::frameToDraw(Camera& camera) {
    camera = session_.camera;
    const builder::Document& document = session_.document();
    const bool rebuilt = session_.history.revision() != builtRevision_;
    if (rebuilt) {
        builtRevision_ = session_.history.revision();
        built_ = builder::buildStatics(document, ++generation_);
    }
    if (rebuilt || !frame_ || session_.preview != posedPreview_) {
        posedPreview_ = session_.preview;
        links_ = builder::previewFrames(document, posedPreview_);
        frame_ = builder::poseScene(built_, links_);
    }
    if (frame_.get() != overlaysFrame_ || !sameSelection(session_.selection, overlaysSelection_) ||
        !sameToggles(session_.view, overlaysToggles_)) {
        overlaysFrame_ = frame_.get();
        overlaysSelection_ = session_.selection;
        overlaysToggles_ = session_.view;
        buildOverlays(links_);
        ++overlayVersion_;
    }
    return frame_;
}

}  // namespace lagrange::viewer

namespace lagrange::viewer {

double BuilderHost::groundHeight() const {
    // The lowest corner of every drawn mesh's box, turned and placed like its prim.
    double lowest = 0.0;
    bool any = false;
    if (frame_ && frame_->statics) {
        const SceneStatics& statics = *frame_->statics;
        for (std::size_t i = 0; i < statics.prims.size() && i < frame_->transforms.size(); ++i) {
            const PrimData& prim = statics.prims[i];
            if (prim.mesh >= statics.meshes.size() || prim.typeName == "Sensor") {
                continue;
            }
            const std::array<float, 6>& box = statics.meshes[prim.mesh].aabb;
            const Transform& t = frame_->transforms[i];
            const builder::Frame f{t.position,
                                   {t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]}};
            for (int k = 0; k < 8; ++k) {
                const V3 corner{(k & 1) != 0 ? box[3] : box[0], (k & 2) != 0 ? box[4] : box[1],
                                (k & 4) != 0 ? box[5] : box[2]};
                const double z = apply(f, corner)[2];
                lowest = any ? std::min(lowest, z) : z;
                any = true;
            }
        }
    }
    return lowest;
}

void BuilderHost::buildOverlays(const std::vector<builder::Frame>& links) {
    using Kind = BuilderSession::Selection::Kind;
    const builder::Document& document = session_.document();
    const BuilderSession::Selection& selected = session_.selection;
    const BuilderSession::ViewToggles& view = session_.view;
    // One list per colour, kept with their capacity from frame to frame.
    enum List : std::uint8_t {
        GridMinor,
        GridMajor,
        X,
        Y,
        Z,
        Axis,
        Collision,
        Sensors,
        Accent,
        Count
    };
    overlays_.resize(Count);
    const std::array<std::array<float, 4>, Count> colours{
        rgba(c::line),
        rgba(c::lineStrong),
        rgba(theme::hex(0xF0605D)),
        rgba(theme::hex(0x4FC97A)),
        rgba(theme::hex(0x5AA2FF)),
        rgba(widgets::domainColor("rigid_body")),
        rgba(c::t2, 0.6F),
        rgba(widgets::domainColor("sensors"), 0.45F),
        rgba(c::accent)};
    for (std::size_t i = 0; i < overlays_.size(); ++i) {
        overlays_[i].color = colours.at(i);
        overlays_[i].origin = {};
        overlays_[i].vertices.clear();
    }
    const auto linkFrame = [&](std::string_view name) -> std::optional<builder::Frame> {
        for (std::size_t l = 0; l < document.links.size() && l < links.size(); ++l) {
            if (document.links[l].name == name) {
                return links[l];
            }
        }
        return std::nullopt;
    };

    if (view.grid) {
        const double z = groundHeight();
        const double extent = kGridStep * kGridHalfCells;
        for (int i = -kGridHalfCells; i <= kGridHalfCells; ++i) {
            const double at = kGridStep * i;
            const Pen pen{overlays_[i % kGridMajorEvery == 0 ? GridMajor : GridMinor], {}};
            pen.segment({at, -extent, z}, {at, extent, z});
            pen.segment({-extent, at, z}, {extent, at, z});
        }
    }
    for (std::size_t l = 0; l < document.links.size() && l < links.size(); ++l) {
        const builder::Link& link = document.links[l];
        const bool linkSelected = selected.kind == Kind::Link && selected.name == link.name;
        if (view.frames || linkSelected) {
            // The selected link's frame reaches out of its shapes, so it shows.
            double length = kFrameAxisLength;
            if (linkSelected) {
                for (const builder::Shape& shape : link.shapes) {
                    const double reach =
                        std::hypot(shape.origin.xyz[0], shape.origin.xyz[1], shape.origin.xyz[2]) +
                        (shape.kind == builder::Shape::Kind::Box
                             ? std::max({shape.size[0], shape.size[1], shape.size[2]}) / 2
                             : std::max(shape.size[0], shape.size[1] / 2));
                    length = std::max(length, reach + kSelectedAxisMargin);
                }
            }
            Pen{overlays_[X], links[l]}.segment({0, 0, 0}, {length, 0, 0});
            Pen{overlays_[Y], links[l]}.segment({0, 0, 0}, {0, length, 0});
            Pen{overlays_[Z], links[l]}.segment({0, 0, 0}, {0, 0, length});
        }
        for (const builder::Shape& shape : link.shapes) {
            if (view.collision && shape.collision) {
                shapeWire(Pen{overlays_[Collision], links[l]}, shape);
            }
            if (linkSelected && shape.visual) {
                shapeWire(Pen{overlays_[Accent], links[l]}, shape);  // the selection's outline
            }
        }
    }
    // Joint axes, at the child link's frame (the joint frame), both ways along the axis.
    for (const builder::Joint& joint : document.joints) {
        const bool jointSelected = selected.kind == Kind::Joint && selected.name == joint.name;
        if (!(view.frames && builder::isMoving(joint.type)) && !jointSelected) {
            continue;
        }
        const auto frame = linkFrame(joint.child);
        if (!frame) {
            continue;
        }
        V3 axis = joint.axis;
        const double length = std::hypot(axis[0], axis[1], axis[2]);
        if (length <= 0.0) {
            continue;
        }
        for (double& v : axis) {
            v *= kJointAxisLength / length;
        }
        const Pen pen{overlays_[jointSelected ? Accent : Axis], *frame};
        pen.dashed({-axis[0], -axis[1], -axis[2]}, axis);
        // A ring about a revolute or continuous axis says it turns (the SPEC's ring gizmo).
        if (joint.type != builder::Joint::Type::Prismatic) {
            const V3 n{axis[0] / kJointAxisLength, axis[1] / kJointAxisLength,
                       axis[2] / kJointAxisLength};
            const V3 helper = std::abs(n[2]) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0};
            V3 u{n[1] * helper[2] - n[2] * helper[1], n[2] * helper[0] - n[0] * helper[2],
                 n[0] * helper[1] - n[1] * helper[0]};
            const double ul = std::hypot(u[0], u[1], u[2]);
            for (double& v : u) {
                v /= ul;
            }
            const V3 w{n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2],
                       n[0] * u[1] - n[1] * u[0]};
            pen.arc(kJointAxisLength * 0.4, u, w);
        }
    }
    // Sensors: thin in the sensors colour; the selected one in accent, drawn even with the
    // toggle off.
    for (const builder::Sensor& sensor : document.sensors) {
        const bool sensorSelected = selected.kind == Kind::Sensor && selected.name == sensor.name;
        if (!view.sensors && !sensorSelected) {
            continue;
        }
        const builder::Frame mount = builder::compose(
            linkFrame(sensor.link).value_or(builder::Frame{}), builder::toFrame(sensor.origin));
        const Pen pen{overlays_[sensorSelected ? Accent : Sensors], mount};
        sensorReach(pen, sensor, session_.catalog,
                    sensorSelected ? std::numeric_limits<double>::infinity() : kUnselectedReach);
        if (sensorSelected) {
            Pen{overlays_[X], mount}.segment({0, 0, 0}, {kFrameAxisLength * 2, 0, 0});
            Pen{overlays_[Y], mount}.segment({0, 0, 0}, {0, kFrameAxisLength * 2, 0});
            Pen{overlays_[Z], mount}.segment({0, 0, 0}, {0, 0, kFrameAxisLength * 2});
            shapeWire(pen, builder::Shape{.kind = builder::Shape::Kind::Box,
                                          .size = {builder::kSensorMarkerSize * 1.2,
                                                   builder::kSensorMarkerSize * 1.2,
                                                   builder::kSensorMarkerSize * 1.2}});
        }
    }
    // The COM marker needs WP-A's robotMass (a stub until it lands: mass 0, nothing drawn).
    if (view.com) {
        const builder::MassProperties mass = builder::robotMass(document);
        if (mass.mass > 0.0) {
            const Pen pen{overlays_[Accent], builder::Frame{mass.com}};
            constexpr double kComMark = 0.03;
            pen.segment({-kComMark, 0, 0}, {kComMark, 0, 0});
            pen.segment({0, -kComMark, 0}, {0, kComMark, 0});
            pen.segment({0, 0, -kComMark}, {0, 0, kComMark});
        }
    }
}

}  // namespace lagrange::viewer
