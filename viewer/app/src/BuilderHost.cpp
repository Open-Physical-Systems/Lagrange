#include "BuilderHost.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Collision.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/MeshShapes.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Urdf.hpp"
#include "lagrange/viewer/builder/Validation.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
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

void shapeWire(const Pen& link, const builder::Shape& shape, bool outline = false);

/// A mesh's box (min x y z, max x y z) at its scale, as a box shape to outline.
std::optional<builder::Shape> meshBox(const builder::Shape& shape) {
    const auto file = builder::loadMesh(shape.mesh);
    if (!file->error.empty() || file->welded.points.empty()) {
        return std::nullopt;
    }
    V3 low{1e300, 1e300, 1e300};
    V3 high{-1e300, -1e300, -1e300};
    for (const V3& p : file->welded.points) {
        for (std::size_t k = 0; k < 3; ++k) {
            low.at(k) = std::min(low.at(k), p.at(k) * shape.scale.at(k));
            high.at(k) = std::max(high.at(k), p.at(k) * shape.scale.at(k));
        }
    }
    builder::Shape box;
    box.kind = builder::Shape::Kind::Box;
    box.size = {high[0] - low[0], high[1] - low[1], high[2] - low[2]};
    const builder::Frame centre = builder::compose(
        builder::toFrame(shape.origin),
        builder::Frame{{(high[0] + low[0]) / 2, (high[1] + low[1]) / 2, (high[2] + low[2]) / 2}});
    box.origin.xyz = centre.position;
    box.origin.rpy = shape.origin.rpy;
    return box;
}

/// The edges of a shape in its link's frame: a box's twelve, a cylinder's two rims and four
/// sides, a sphere's three great circles; a mesh its server hull's, or with `outline` its box.
void shapeWire(const Pen& link, const builder::Shape& shape, bool outline) {
    if (outline && shape.kind == builder::Shape::Kind::Mesh) {
        if (const auto box = meshBox(shape)) {
            shapeWire(link, *box);
        }
        return;
    }
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
        case builder::Shape::Kind::Capsule: {  // Addendum A: rims, sides and the two caps
            const double r = shape.size[0];
            const double z = shape.size[1] / 2;
            for (const double side : {-z, z}) {
                const Pen rim{pen.lines, builder::compose(pen.frame, builder::Frame{{0, 0, side}})};
                rim.arc(r, {1, 0, 0}, {0, 1, 0});
                const double from = side > 0 ? 0.0 : std::numbers::pi;
                rim.arc(r, {1, 0, 0}, {0, 0, 1}, from, from + std::numbers::pi);
                rim.arc(r, {0, 1, 0}, {0, 0, 1}, from, from + std::numbers::pi);
            }
            for (int i = 0; i < 4; ++i) {
                const double a = std::numbers::pi / 2 * i;
                pen.segment({r * std::cos(a), r * std::sin(a), -z},
                            {r * std::cos(a), r * std::sin(a), z});
            }
            break;
        }
        case builder::Shape::Kind::Mesh: {  // Addendum A: the edges of its server hull
            const auto file = builder::loadMesh(shape.mesh);
            if (!file->error.empty()) {
                break;
            }
            const builder::MeshSolid& solid = file->solid(shape.scale);
            if (!solid.hull) {
                break;
            }
            std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
            for (const auto& t : solid.hull->triangles) {
                for (std::size_t e = 0; e < 3; ++e) {
                    const std::uint32_t a = t.at(e);
                    const std::uint32_t b = t.at((e + 1) % 3);
                    if (edges.emplace(std::min(a, b), std::max(a, b)).second) {
                        pen.segment(solid.hull->points[a], solid.hull->points[b]);
                    }
                }
            }
            break;
        }
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

BuilderHost::~BuilderHost() {
    jobs_.clear();  // each stops its child
    if (test_.state() == builder::TestServer::State::Running) {
        test_.requestStop();
    }
    // The viewer is closing: wait out the stop (at most the grace and a SIGKILL), then remove
    // the test copies, logs and the folder (plan decision 4).
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(builder::TestServer::kStopGraceS + 1.0);
    while (test_.poll() == builder::TestServer::State::Stopping &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!runtime_.empty()) {
        std::error_code ec;
        std::filesystem::remove_all(runtime_, ec);
    }
}

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
    // Relative mesh paths are read against the document's folder (Addendum A): set before
    // validation and drawing, which read the meshes.
    builder::setMeshBase(session_.path.empty() ? std::filesystem::path{}
                                               : session_.path.parent_path());
    // Validation runs on a document change, not every frame (R15).
    if (session_.history.revision() != validatedRevision_ || session_.opened != validatedOpened_) {
        validatedRevision_ = session_.history.revision();
        validatedOpened_ = session_.opened;
        session_.problems.builder = builder::validate(
            session_.document(), &session_.catalog,
            session_.path.empty() ? std::filesystem::path{} : session_.path.parent_path());
    }
    const double now = ImGui::GetTime();
    pollTest(now);
    pollRemovals();
    pollJobs();
    runSplit();  // Addendum A: CoACD, polled in every mode
    if (std::exchange(session_.backRequested, false) &&
        session_.mode != BuilderSession::Mode::Build) {
        endTest({});
    }
    if (std::exchange(session_.testRequested, false) &&
        session_.mode == BuilderSession::Mode::Build) {
        startTest(now);
    }
    if (session_.mode != BuilderSession::Mode::Build) {
        // The document is read-only while testing: file requests wait for Build.
        session_.saveRequested = session_.openRequested = session_.templateRequested = false;
        session_.exportUsdRequested = session_.checkRequested = false;
        return;
    }
    if (std::exchange(session_.saveRequested, false)) {
        const std::filesystem::path target =
            session_.savePath.empty() ? session_.path : session_.savePath;
        session_.savePath.clear();
        (void)save(target);
    }
    if (std::exchange(session_.openRequested, false)) {
        open(std::exchange(session_.openPath, {}));
    }
    if (std::exchange(session_.templateRequested, false)) {
        openTemplate(std::exchange(session_.templateName, {}));
    }
    if (std::exchange(session_.exportUsdRequested, false)) {
        exportUsd();
    }
    if (std::exchange(session_.checkRequested, false)) {
        checkLoads();
    }
    autosave(now);
}

// --- programs and folders -------------------------------------------------------------------

namespace {

/// `relative` under the first AMENT_PREFIX_PATH entry that has it, or empty.
std::filesystem::path underPrefix(const std::filesystem::path& relative) {
    const char* prefixes = std::getenv("AMENT_PREFIX_PATH");
    std::string_view list = prefixes == nullptr ? std::string_view{} : prefixes;
    while (!list.empty()) {
        const std::size_t end = std::min(list.find(':'), list.size());
        const std::filesystem::path candidate =
            std::filesystem::path(list.substr(0, end)) / relative;
        std::error_code ec;
        if (end > 0 && std::filesystem::exists(candidate, ec)) {
            return candidate;
        }
        list.remove_prefix(std::min(end + 1, list.size()));
    }
    return {};
}

/// A program on PATH, or empty.
std::filesystem::path onPath(std::string_view name) {
    const char* path = std::getenv("PATH");
    std::string_view list = path == nullptr ? std::string_view{} : path;
    while (!list.empty()) {
        const std::size_t end = std::min(list.find(':'), list.size());
        const std::filesystem::path candidate = std::filesystem::path(list.substr(0, end)) / name;
        if (end > 0 && ::access(candidate.c_str(), X_OK) == 0) {
            return candidate;
        }
        list.remove_prefix(std::min(end + 1, list.size()));
    }
    return {};
}

std::string environment(const char* name) {
    const char* value = std::getenv(name);
    return value == nullptr ? std::string{} : std::string(value);
}

/// The last non-empty lines of `text`, at most `count`.
std::vector<std::string> lastLines(const std::string& text, std::size_t count) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    for (std::string line; std::getline(in, line);) {
        if (line.find_first_not_of(" \t\r") != std::string::npos) {
            lines.push_back(line);
        }
    }
    if (lines.size() > count) {
        lines.erase(lines.begin(), lines.end() - static_cast<std::ptrdiff_t>(count));
    }
    return lines;
}

std::string readFile(const std::filesystem::path& path, std::string& error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "cannot read " + path.string() + ": " + std::strerror(errno);
        return {};
    }
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

/// The test server's ROS_DOMAIN_ID (plan R8): $LAGRANGE_BUILDER_ROS_DOMAIN_ID ("off": no ROS),
/// else the viewer's domain + 1, kept in 0..101 (the range ROS 2 calls safe on Linux), so a
/// teleop on the user's domain drives only the run it was meant for.
std::string testRosDomain() {
    constexpr int kSafeDomains = 102;  // 0..101: docs.ros.org "The ROS_DOMAIN_ID", Linux
    const std::string chosen = environment("LAGRANGE_BUILDER_ROS_DOMAIN_ID");
    if (chosen == "off") {
        return {};
    }
    if (!chosen.empty()) {
        return chosen;
    }
    const int user = std::atoi(environment("ROS_DOMAIN_ID").c_str());
    return std::to_string((user + 1) % kSafeDomains);
}

}  // namespace

std::filesystem::path BuilderHost::autosavePath(const std::string& robot) {
    const std::string state = environment("XDG_STATE_HOME");
    const std::filesystem::path base =
        !state.empty() ? std::filesystem::path(state)
                       : std::filesystem::path(environment("HOME")) / ".local/state";
    return base / "lagrange" / "builder" / (robot + ".autosave.urdf");
}

std::filesystem::path BuilderHost::runtimeDirectory() {
    if (runtime_.empty()) {
        const std::string xdg = environment("XDG_RUNTIME_DIR");
        const std::filesystem::path base = !xdg.empty()
                                               ? std::filesystem::path(xdg) / "lagrange"
                                               : std::filesystem::temp_directory_path() /
                                                     ("lagrange-" + std::to_string(::getuid()));
        runtime_ = base / "builder" / std::to_string(::getpid());
        std::error_code ec;
        std::filesystem::create_directories(runtime_, ec);
        std::filesystem::permissions(runtime_, std::filesystem::perms::owner_all,
                                     std::filesystem::perm_options::replace, ec);
    }
    return runtime_;
}

std::filesystem::path BuilderHost::program(std::string_view name) const {
    if (name == "lagrange_server" && !serverProgram_.empty()) {
        return serverProgram_;
    }
    return underPrefix(std::filesystem::path("lib/lagrange_server") / name);
}

void BuilderHost::fileProblem(builder::Level level, std::string item, std::string message) {
    if (level == builder::Level::Error) {
        session_.fileError = message;
    }
    session_.problems.files.push_back({level, std::move(item), std::move(message)});
    constexpr std::size_t kKept = 20;  // the newest outcomes; older ones say nothing new
    if (session_.problems.files.size() > kKept) {
        session_.problems.files.erase(session_.problems.files.begin());
    }
    if (level == builder::Level::Error) {
        ImGui::SetWindowFocus(builder_windows::kProblems);  // SPEC state 6
    }
}

std::filesystem::path BuilderHost::writeCopy(const std::string& name, std::string& error) {
    builder::WriteOptions options;
    options.savedAt = session_.path.empty()
                          ? std::filesystem::current_path() / (session_.document().robot + ".urdf")
                          : session_.path;
    options.absoluteMeshes = true;
    const std::filesystem::path copy = runtimeDirectory() / name;
    error = builder::saveAtomically(copy, builder::writeUrdf(session_.document(), options));
    return error.empty() ? copy : std::filesystem::path{};
}

// --- files ----------------------------------------------------------------------------------

bool BuilderHost::save(const std::filesystem::path& target) {
    if (target.empty()) {
        fileProblem(builder::Level::Error, "", "Save: the robot has no file yet; use Save as…");
        return false;
    }
    std::filesystem::path path = target;
    std::error_code ec;
    if (path.is_relative()) {
        path = std::filesystem::absolute(path, ec);
    }
    builder::WriteOptions options;
    options.savedAt = path;
    const std::string error =
        builder::saveAtomically(path, builder::writeUrdf(session_.document(), options));
    if (!error.empty()) {
        fileProblem(builder::Level::Error, "", "Save: " + error);
        return false;
    }
    session_.path = path.lexically_normal();
    session_.history.markSaved();
    session_.fileError.clear();
    // The autosave is behind the file now.
    std::filesystem::remove(autosavePath(session_.document().robot), ec);
    autosavedRevision_ = session_.history.revision();
    return true;
}

void BuilderHost::replaceDocument(builder::Document document, const std::filesystem::path& path,
                                  const std::string& label) {
    session_.history = builder::History(std::move(document), label);
    session_.path = path;
    session_.selection = {};
    session_.preview.clear();
    session_.problems.loader.clear();
    session_.problems.check.clear();
    session_.problems.checkTable.clear();
    session_.problems.checkExit = -1;
    session_.fileError.clear();
    ++session_.opened;
    autosavedRevision_ = session_.history.revision();
}

bool BuilderHost::openText(const std::string& text, const std::filesystem::path& from,
                           const std::filesystem::path& path, const std::string& label) {
    builder::ReadResult read = builder::readUrdf(text);
    if (!read.document) {
        fileProblem(builder::Level::Error, "",
                    "Open " + from.filename().string() + ": " +
                        (read.errorLine > 0 ? "line " + std::to_string(read.errorLine) + ": "
                                            : std::string{}) +
                        read.error);
        return false;
    }
    replaceDocument(std::move(*read.document), path, label);
    for (const std::string& note : read.notes) {
        fileProblem(builder::Level::Hint, "", from.filename().string() + ": " + note);
    }
    return true;
}

void BuilderHost::open(const std::filesystem::path& requested) {
    if (requested.empty()) {
        return;
    }
    std::error_code ec;
    const std::filesystem::path path = std::filesystem::absolute(requested, ec).lexically_normal();
    if (path.extension() == ".xacro") {
        // RFC §10: a xacro file opens by expansion (a child process, polled); it is saved as
        // .urdf, never written back over the .xacro.
        const std::filesystem::path xacro = onPath("xacro");
        if (xacro.empty()) {
            fileProblem(builder::Level::Error, "",
                        "Open " + path.filename().string() +
                            ": xacro is not installed (ros-lyrical-xacro)");
            return;
        }
        auto job = std::make_unique<Job>();
        job->kind = Job::Kind::Xacro;
        job->source = path;
        job->output = runtimeDirectory() / (path.stem().string() + ".expanded.urdf");
        const std::string error =
            job->process.start({xacro.string(), path.string(), "-o", job->output.string()},
                               runtimeDirectory() / "xacro.log");
        if (!error.empty()) {
            fileProblem(builder::Level::Error, "",
                        "Open " + path.filename().string() + ": " + error);
            return;
        }
        jobs_.push_back(std::move(job));
        return;
    }
    std::string error;
    const std::string text = readFile(path, error);
    if (!error.empty()) {
        fileProblem(builder::Level::Error, "", "Open: " + error);
        return;
    }
    (void)openText(text, path, path, "Open " + path.filename().string());
}

void BuilderHost::openTemplate(const std::string& name) {
    if (name.empty()) {
        return;
    }
    // assets/robots/templates (WP-B): $LAGRANGE_BUILDER_TEMPLATES, else lagrange_assets'.
    std::filesystem::path directory = environment("LAGRANGE_BUILDER_TEMPLATES");
    if (directory.empty()) {
        directory = underPrefix("share/lagrange_assets/robots/templates");
    }
    const std::filesystem::path file = directory / (name + ".urdf");
    std::string error;
    const std::string text = directory.empty() ? std::string{} : readFile(file, error);
    if (directory.empty() || !error.empty()) {
        fileProblem(
            builder::Level::Error, "",
            "Template " + name + ": " +
                (directory.empty() ? "lagrange_assets' templates are not installed" : error));
        return;
    }
    // A template opens as a new document: Save asks for a file, the template stays as it is.
    (void)openText(text, file, {}, "Template " + name);
}

void BuilderHost::autosave(double now) {
    constexpr double kAutosaveEveryS = 30.0;  // RFC §10: a crash loses at most this much
    if (session_.history.editsSinceSave() == 0 ||
        session_.history.revision() == autosavedRevision_ ||
        now - session_.autosaveTime < kAutosaveEveryS) {
        return;
    }
    session_.autosaveTime = now;
    autosavedRevision_ = session_.history.revision();
    const std::filesystem::path path = autosavePath(session_.document().robot);
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    builder::WriteOptions options;
    options.savedAt = session_.path.empty() ? path : session_.path;
    options.absoluteMeshes = true;  // it lives elsewhere than the file
    if (const std::string error =
            builder::saveAtomically(path, builder::writeUrdf(session_.document(), options));
        !error.empty()) {
        fileProblem(builder::Level::Warning, "", "Autosave: " + error);
    }
}

// --- Export USD, Check loads ------------------------------------------------------------------

bool BuilderHost::jobRunning(Job::Kind kind) const {
    return std::ranges::any_of(
        jobs_, [kind](const std::unique_ptr<Job>& job) { return job->kind == kind; });
}

void BuilderHost::exportUsd() {
    if (jobRunning(Job::Kind::Export)) {
        return;
    }
    if (session_.path.empty()) {
        fileProblem(builder::Level::Error, "",
                    "Export USD: save the robot first; the .usda is written next to its file");
        return;
    }
    const std::filesystem::path importer = program("lagrange_import");
    if (importer.empty()) {
        fileProblem(builder::Level::Error, "", "Export USD: lagrange_import is not installed");
        return;
    }
    std::string error;
    const std::filesystem::path copy = writeCopy("export.urdf", error);
    if (copy.empty()) {
        fileProblem(builder::Level::Error, "", "Export USD: " + error);
        return;
    }
    auto job = std::make_unique<Job>();
    job->kind = Job::Kind::Export;
    job->output = std::filesystem::path(session_.path).replace_extension(".usda");
    error = job->process.start({importer.string(), copy.string(), job->output.string()},
                               runtimeDirectory() / "export.log");
    if (!error.empty()) {
        fileProblem(builder::Level::Error, "", "Export USD: " + error);
        return;
    }
    jobs_.push_back(std::move(job));
}

void BuilderHost::checkLoads() {
    if (jobRunning(Job::Kind::Check)) {
        return;
    }
    const std::filesystem::path checker = program("lagrange_check");
    if (checker.empty()) {
        fileProblem(builder::Level::Error, "", "Check loads: lagrange_check is not installed");
        return;
    }
    std::string error;
    const std::filesystem::path copy = writeCopy("check.urdf", error);
    if (copy.empty()) {
        fileProblem(builder::Level::Error, "", "Check loads: " + error);
        return;
    }
    auto job = std::make_unique<Job>();
    job->kind = Job::Kind::Check;
    error = job->process.start({checker.string(), copy.string()}, runtimeDirectory() / "check.log");
    if (!error.empty()) {
        fileProblem(builder::Level::Error, "", "Check loads: " + error);
        return;
    }
    session_.problems.check.clear();
    session_.problems.checkTable.clear();
    session_.problems.checkExit = -1;
    jobs_.push_back(std::move(job));
}

void BuilderHost::readCheck(const builder::ChildProcess& process) {
    BuilderSession::Problems& problems = session_.problems;
    std::string table = process.logTail(64 * 1024);
    // The copy's temporary path means nothing to the user: name the robot instead.
    const std::string copy = (runtimeDirectory() / "check.urdf").string();
    for (std::size_t at = table.find(copy); at != std::string::npos; at = table.find(copy)) {
        table.replace(at, copy.size(), session_.document().robot);
    }
    problems.checkTable = table;
    problems.checkExit = WIFEXITED(process.exitStatus()) ? WEXITSTATUS(process.exitStatus()) : -1;
    // R13: one row per table line that names a link or joint of the document and ends in a
    // verdict ("CANNOT HOLD where it starts", "cannot hold stretched out", "denser than ...").
    const builder::Document& document = session_.document();
    const auto known = [&](const std::string& name) {
        return std::ranges::any_of(document.links, [&](const auto& l) { return l.name == name; }) ||
               std::ranges::any_of(document.joints, [&](const auto& j) { return j.name == name; });
    };
    std::istringstream in(table);
    for (std::string line; std::getline(in, line);) {
        const std::size_t end = line.find(' ');
        const std::string name = line.substr(0, end);
        const std::size_t verdict =
            line.find_first_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz",
                               line.find_first_not_of(' ', end));
        if (end == std::string::npos || !known(name) || verdict == std::string::npos ||
            line.compare(verdict, 1, "-") == 0) {
            continue;
        }
        std::string text = line.substr(verdict);
        problems.check.push_back({line.find("CANNOT") != std::string::npos ? builder::Level::Warning
                                                                           : builder::Level::Hint,
                                  name, text});
    }
    if (problems.checkExit == 2) {
        // The loader refused the robot: its message is the table's last line.
        const auto lines = lastLines(table, 1);
        problems.check.push_back(
            {builder::Level::Warning, "", lines.empty() ? process.exitText() : lines.front()});
    }
}

void BuilderHost::pollJobs() {
    std::string busy;
    for (auto it = jobs_.begin(); it != jobs_.end();) {
        Job& job = **it;
        if (job.process.poll() != builder::ChildProcess::State::Exited) {
            busy = job.kind == Job::Kind::Export  ? "Exporting USD…"
                   : job.kind == Job::Kind::Check ? "Checking loads…"
                                                  : "Expanding xacro…";
            ++it;
            continue;
        }
        const std::vector<std::string> tail = lastLines(job.process.logTail(4096), 3);
        std::string why = job.process.exitText();
        for (const std::string& line : tail) {
            why += " · " + line;
        }
        switch (job.kind) {
            case Job::Kind::Export:
                if (job.process.succeeded()) {
                    fileProblem(builder::Level::Hint, "", "Exported USD to " + job.output.string());
                } else {
                    fileProblem(builder::Level::Error, "", "Export USD failed: " + why);
                }
                break;
            case Job::Kind::Check:
                readCheck(job.process);
                ImGui::SetWindowFocus(builder_windows::kProblems);
                break;
            case Job::Kind::Xacro:
                if (job.process.succeeded()) {
                    std::string error;
                    const std::string text = readFile(job.output, error);
                    if (!error.empty()) {
                        fileProblem(builder::Level::Error, "", "Open: " + error);
                    } else if (openText(text, job.source, {},
                                        "Open " + job.source.filename().string())) {
                        fileProblem(builder::Level::Hint, "",
                                    "Expanded " + job.source.filename().string() +
                                        "; Save writes a .urdf (the .xacro stays as it is)");
                    }
                } else {
                    fileProblem(builder::Level::Error, "",
                                "Open " + job.source.filename().string() + ": xacro " + why);
                }
                break;
        }
        it = jobs_.erase(it);
    }
    if (busy.empty() && session_.mode == BuilderSession::Mode::Starting) {
        busy = "Starting the test server…";
    }
    session_.busy = busy;
}

// --- Test -----------------------------------------------------------------------------------

void BuilderHost::startTest(double now) {
    session_.problems.loader.clear();
    if (session_.problems.errors() > 0) {
        return;  // the button is disabled then; a key or a script may still ask
    }
    if (session_.testIntoRun) {
        startTestInRun(now);
        return;
    }
    const std::filesystem::path server = program("lagrange_server");
    if (server.empty()) {
        session_.problems.loader.push_back(
            {builder::Level::Warning, "", "Test: lagrange_server is not installed"});
        return;
    }
    std::string error;
    const std::filesystem::path urdf = writeCopy(
        session_.document().robot.empty() ? "robot.urdf" : session_.document().robot + ".urdf",
        error);
    if (urdf.empty()) {
        session_.problems.loader.push_back({builder::Level::Warning, "", "Test: " + error});
        return;
    }
    builder::TestRequest request;
    request.program = server.string();
    request.urdf = urdf;
    request.world = session_.testWorld.empty() ? "none" : session_.testWorld;
    // Plan decision 4: a socket named by the viewer's pid, never by the robot (sun_path).
    request.endpoint =
        "unix:" + proto::defaultUnixSocketPath("builder-" + std::to_string(::getpid()));
    request.log = runtimeDirectory() / "server.log";
    request.rosDomain = testRosDomain();
    const auto endpoint = proto::parseEndpoint(request.endpoint);
    if (!endpoint) {
        session_.problems.loader.push_back(
            {builder::Level::Warning, "", "Test: no socket path for the test server"});
        return;
    }
    error = test_.start(request);
    if (!error.empty()) {
        session_.problems.loader.push_back({builder::Level::Warning, "", "Test: " + error});
        ImGui::SetWindowFocus(builder_windows::kProblems);
        return;
    }
    // The client follows the test server; Back puts it where it was (R3).
    previousEndpoint_ = client_.endpoint();
    previousRunning_ = client_.running();
    client_.stop();
    client_.setEndpoint(*endpoint);
    client_.start();
    snapshotsAtStart_ = client_.stats().snapshots;
    startedAt_ = now;
    logReadAt_ = 0.0;
    session_.mode = BuilderSession::Mode::Starting;
    session_.testEndpoint = request.endpoint;
    session_.testRosDomain = request.rosDomain;
    session_.testLog.clear();
    ++session_.testSerial;
}

void BuilderHost::startTestInRun(double now) {
    const ConnectionInfo connection = client_.connection();
    if (connection.state != ConnectionState::Ready || !connection.canImport) {
        session_.problems.loader.push_back(
            {builder::Level::Warning, "", "Test: not connected to a server that imports robots"});
        ImGui::SetWindowFocus(builder_windows::kProblems);
        return;
    }
    // The text the server reads, with meshes by absolute path (the server may run elsewhere:
    // then they must exist there too; phase 2 sends meshes).
    std::string error;
    const std::filesystem::path urdf = writeCopy(
        session_.document().robot.empty() ? "robot.urdf" : session_.document().robot + ".urdf",
        error);
    std::string text;
    if (!urdf.empty()) {
        std::ifstream in(urdf, std::ios::binary);
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    if (text.empty()) {
        session_.problems.loader.push_back(
            {builder::Level::Warning, "", "Test: " + (error.empty() ? "no URDF written" : error)});
        return;
    }
    importRequest_ = client_.importRobotText(text);
    inRun_ = true;
    startedAt_ = now;
    logReadAt_ = 0.0;
    session_.mode = BuilderSession::Mode::Starting;
    session_.testEndpoint = proto::toString(client_.endpoint());
    session_.testRosDomain.clear();
    session_.testRobotName.clear();
    session_.testPassive = false;
    session_.testLog.clear();
    ++session_.testSerial;
}

namespace {

/// The connected run's log as the Console shows it: "[source] text" per line, the last ones.
std::string logText(const LogBook& book) {
    constexpr std::size_t kLines = 200;
    std::string text;
    const std::size_t first = book.entries.size() > kLines ? book.entries.size() - kLines : 0;
    for (std::size_t i = first; i < book.entries.size(); ++i) {
        text += "[" + book.entries[i].source + "] " + book.entries[i].text + "\n";
    }
    return text;
}

}  // namespace

void BuilderHost::pollTestInRun(double now) {
    if (client_.connection().state != ConnectionState::Ready) {
        endTest("The connection to the run was lost");
        return;
    }
    constexpr double kLogEveryS = 0.25;
    if (now - logReadAt_ >= kLogEveryS) {
        logReadAt_ = now;
        const std::shared_ptr<const LogBook> book = client_.logs();
        session_.testLog = logText(*book);
        // The server says whether the spawned robot is the one ROS drives (Server.cpp spawn).
        if (!session_.testRobotName.empty() && !session_.testPassive) {
            const std::string spawned = "spawned " + session_.testRobotName + " from ";
            for (const LogEntry& entry : book->entries) {
                if (entry.text.starts_with(spawned) &&
                    entry.text.find("(passive") != std::string::npos) {
                    session_.testPassive = true;
                }
            }
        }
    }
    if (session_.mode != BuilderSession::Mode::Starting) {
        return;
    }
    if (const std::shared_ptr<const CommandOutcome> answer = client_.outcome(importRequest_)) {
        importRequest_ = 0;
        if (!answer->ok) {
            // An older server takes robot files by path only (protocol.md "Commands").
            const bool older = answer->error.find("by its path") != std::string::npos;
            endTest("The run refused the robot: " + answer->error +
                    (older ? " (this server is older than RFC 0016 §7.2: test in a server of "
                             "the robot's own instead)"
                           : ""));
            return;
        }
        std::string name = answer->importedStagePath;
        if (name.starts_with('/')) {
            name.erase(0, 1);
        }
        session_.testRobotName = name;
        session_.mode = BuilderSession::Mode::Testing;
        logReadAt_ = 0.0;  // look for the passive line at once
    } else if (now - startedAt_ > kStartTimeoutS) {
        endTest("The run did not answer within " +
                std::to_string(static_cast<int>(kStartTimeoutS)) + " s");
    }
}

void BuilderHost::pollRemovals() {
    if (removeAfterImport_ != 0) {
        if (const std::shared_ptr<const CommandOutcome> answer =
                client_.outcome(removeAfterImport_)) {
            removeAfterImport_ = 0;
            if (answer->ok && !answer->importedStagePath.empty()) {
                removeRequest_ = client_.removeRobot(answer->importedStagePath);
            }
        }
    }
    if (removeRequest_ != 0) {
        if (const std::shared_ptr<const CommandOutcome> answer = client_.outcome(removeRequest_)) {
            removeRequest_ = 0;
            if (!answer->ok) {
                session_.problems.loader.push_back(
                    {builder::Level::Warning, "",
                     "Back: the robot stays in the run (" + answer->error + ")"});
            }
        }
    }
}

void BuilderHost::restoreClient() {
    client_.stop();
    client_.setEndpoint(previousEndpoint_);
    if (previousRunning_) {
        client_.start();  // the run the viewer showed before: as it was (it was left alone)
    }
}

void BuilderHost::endTest(const std::string& failure) {
    if (inRun_) {
        // Back takes the robot out of the run again; the run itself goes on.
        if (!session_.testRobotName.empty() &&
            client_.connection().state == ConnectionState::Ready) {
            removeRequest_ = client_.removeRobot(session_.testRobotName);
        } else if (importRequest_ != 0) {
            removeAfterImport_ = importRequest_;  // not answered yet
        }
        inRun_ = false;
        importRequest_ = 0;
        session_.testRobotName.clear();
        session_.testPassive = false;
    } else {
        if (test_.state() == builder::TestServer::State::Running) {
            test_.requestStop();  // poll() sends SIGKILL after the grace
        }
        restoreClient();
    }
    session_.mode = BuilderSession::Mode::Build;
    if (!failure.empty()) {
        // SPEC state 4: the view does not change; Problems comes forward with the loader's
        // message and Build stays active.
        session_.problems.loader.push_back({builder::Level::Warning, "", failure});
        constexpr std::size_t kTailBytes = 4096;
        constexpr std::size_t kTailLines = 8;
        for (const std::string& line : lastLines(test_.logTail(kTailBytes), kTailLines)) {
            session_.problems.loader.push_back({builder::Level::Warning, "", line});
        }
        ImGui::SetWindowFocus(builder_windows::kProblems);
    }
}

void BuilderHost::pollTest(double now) {
    const builder::TestServer::State state = test_.poll();
    if (session_.mode == BuilderSession::Mode::Build) {
        return;
    }
    if (inRun_) {
        pollTestInRun(now);
        return;
    }
    constexpr double kLogEveryS = 0.25;
    if (now - logReadAt_ >= kLogEveryS) {
        logReadAt_ = now;
        constexpr std::size_t kConsoleBytes = 16 * 1024;
        session_.testLog = test_.logTail(kConsoleBytes);
    }
    if (state == builder::TestServer::State::Exited) {
        endTest(session_.mode == BuilderSession::Mode::Starting
                    ? "The test server stopped before it was ready (" + test_.exitText() + ")"
                    : "The test server stopped (" + test_.exitText() + ")");
        return;
    }
    if (session_.mode == BuilderSession::Mode::Starting) {
        if (client_.connection().state == ConnectionState::Ready &&
            client_.stats().snapshots > snapshotsAtStart_ && client_.scene() != nullptr) {
            session_.mode = BuilderSession::Mode::Testing;
            // The loader's notes and warnings (the run started): listed under Loader.
            for (const std::string& line : lastLines(test_.logTail(16 * 1024), 200)) {
                if (line.find("warning") != std::string::npos ||
                    line.find("note:") != std::string::npos) {
                    session_.problems.loader.push_back({builder::Level::Hint, "", line});
                }
            }
        } else if (now - startedAt_ > kStartTimeoutS) {
            endTest("The test server did not answer within " +
                    std::to_string(static_cast<int>(kStartTimeoutS)) + " s");
        }
    }
}

void BuilderHost::runSplit() {
    // Addendum A, choice 2: CoACD as a polled child process (never waited for here).
    if (session_.splitCancelRequested) {
        session_.splitCancelRequested = false;
        split_.cancel();
    }
    if (session_.splitRequested) {
        session_.splitRequested = false;
        const builder::Link* link = builder::findLink(session_.document(), session_.splitLink);
        const auto mesh = link != nullptr ? builder::collisionMeshOf(*link) : std::nullopt;
        session_.split = {};
        session_.split.link = session_.splitLink;
        if (!mesh) {
            session_.split.failed = true;
            session_.split.message = "the link has no mesh shape to split";
        } else {
            const builder::Shape& shape = link->shapes[*mesh];
            builder::SplitRequest request;
            request.robotFile = session_.path;
            request.robot = session_.document().robot;
            request.link = link->name;
            request.mesh = shape.mesh;
            request.meshBase = builder::meshBase();
            request.scale = shape.scale;
            request.threshold = session_.splitThreshold;
            request.seed = session_.splitSeed;
            split_.reset();
            const std::string error = split_.start(request);
            session_.split.running = error.empty();
            session_.split.failed = !error.empty();
            session_.split.message = error;
        }
    }
    const builder::CollisionSplit::State state = split_.poll();
    if (state == builder::CollisionSplit::State::Running) {
        session_.split.running = true;
        session_.split.seconds = split_.seconds();
    } else if (state == builder::CollisionSplit::State::Done) {
        const std::string name = split_.link();
        const std::vector<std::string> parts = split_.parts();
        builder::Document next = session_.document();
        if (builder::Link* link = builder::findLink(next, name)) {
            builder::setSplitParts(*link, parts);
            session_.history.commit(
                std::move(next),
                "Split " + name + " into " + std::to_string(parts.size()) + " convex parts", {},
                ImGui::GetTime());
        }
        session_.split.running = false;
        session_.split.failed = false;
        session_.split.message = std::to_string(parts.size()) + " convex parts in " +
                                 session_.document().robot + "_collision/";
        session_.view.collision = true;
        split_.reset();
    } else if (state == builder::CollisionSplit::State::Failed) {
        session_.split.running = false;
        session_.split.failed = true;
        session_.split.message = split_.error();
        split_.reset();
    }
}

std::shared_ptr<const SceneFrame> BuilderHost::frameToDraw(Camera& camera) {
    camera = session_.camera;
    const builder::Document& document = session_.document();
    const bool rebuilt = session_.history.revision() != builtRevision_ ||
                         session_.opened != builtOpened_ ||
                         session_.view.collision != builtCollision_;
    if (rebuilt) {
        builtRevision_ = session_.history.revision();
        builtOpened_ = session_.opened;
        // The collision toggle draws collision geometry translucent (Addendum A).
        builtCollision_ = session_.view.collision;
        built_ = builder::buildStatics(document, ++generation_,
                                       builder::BuildOptions{.collision = builtCollision_});
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
                shapeWire(Pen{overlays_[Accent], links[l]}, shape,
                          true);  // the selection's outline
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
