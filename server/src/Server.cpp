#include "Recorder.hpp"
#include "lagrange/aerodynamics/Aerodynamics.hpp"
#include "lagrange/comms/Comms.hpp"
#include "lagrange/electrical/IdealTorqueSource.hpp"
#include "lagrange/electrical/Network.hpp"
#include "lagrange/emi/Emi.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/humans/Body.hpp"
#include "lagrange/humans/Humans.hpp"
#include "lagrange/importers/Importers.hpp"
#include "lagrange/mavlink/ArduPilotJson.hpp"
#include "lagrange/mavlink/Px4Lockstep.hpp"
#include "lagrange/meshes/MeshImport.hpp"
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/openscenario/Scenario.hpp"
#include "lagrange/proto/Connection.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/recording/Recording.hpp"
#include "lagrange/rigid_body/FixedRobot.hpp"
#include "lagrange/rigid_body/JoltBackend.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/ros2_control/ControllerHost.hpp"
#include "lagrange/rotor_aero/RotorAero.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/BodyScene.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/server/CarDrive.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Multicopter.hpp"
#include "lagrange/server/Plugins.hpp"
#include "lagrange/server/Profile.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/server/SwerveDrive.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/services/WrenchSum.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/thermal/ThermalNetwork.hpp"
#include "lagrange/traffic/Traffic.hpp"
#include "lagrange/vehicle_dynamics/VehicleDynamics.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"
#include "lagrange/weather/Weather.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/RoadLanes.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <poll.h>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lagrange::server {

namespace v1 = lagrange::viewer::v1;

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint32_t kProtocolMajor = 0;  // the mock server's version (server.py)
constexpr std::uint32_t kProtocolMinor = 1;
constexpr char kServerName[] = "lagrange_server " LAGRANGE_VERSION;
constexpr std::int64_t kStatusPeriodNs = 100'000'000;  // 10 Hz, as the mock server
constexpr std::int64_t kStatsPeriodNs = 1'000'000'000;
constexpr std::int64_t kFieldPeriodNs = 500'000'000;  // field slices: twice a second
// The environment's fields (EnvironmentState.Field.name), as the mock server names them.
constexpr const char* kFogField = "weather/fog_density";
constexpr const char* kWindField = "weather/wind";
constexpr const char* kFluxField = "emi/flux_density";
constexpr std::uint32_t kSliceWidth = 64;   // cells along +X
constexpr std::uint32_t kSliceHeight = 48;  // cells along +Y
constexpr double kSliceCell = 0.5;          // m
// Most simulation time one iteration may compute; beyond it the run falls behind real time
// instead of freezing the viewer connection.
constexpr std::int64_t kMaxStepWallNs = 40'000'000;
constexpr std::size_t kMaxQueuedBytes = 32U * 1024U * 1024U;  // a viewer that does not read
constexpr std::size_t kMaxLogRecords = 500;
constexpr std::int64_t kStepCommandDefaultNs = 1'000'000;  // Step{ticks = n}: n ms, as the mock

std::int64_t nanosSince(Clock::time_point since, Clock::time_point now) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now - since).count();
}

void setUnit(v1::Unit& out, const Unit& unit) {
    out.set_length(unit.dim.length);
    out.set_mass(unit.dim.mass);
    out.set_time(unit.dim.time);
    out.set_current(unit.dim.current);
    out.set_temperature(unit.dim.temperature);
    out.set_amount(unit.dim.amount);
    out.set_luminous(unit.dim.luminous);
    out.set_angle(unit.dim.angle);
    out.set_scale(unit.scale);
    out.set_kind(static_cast<std::uint32_t>(unit.kind));
    out.set_display(label(unit));
}

std::vector<std::string> componentNames(const SignalSpec& spec) {
    switch (spec.type) {
        case SignalType::Vec3:
            return {"x", "y", "z"};
        case SignalType::Quat:
            return {"qw", "qx", "qy", "qz"};
        case SignalType::Pose:
            return {"x", "y", "z", "qw", "qx", "qy", "qz"};
        case SignalType::Wrench:
            if (spec.unit.kind == QuantityKind::Force) {
                return {"fx", "fy", "fz", "tx", "ty", "tz"};
            }
            if (spec.unit.kind == QuantityKind::LinearVelocity) {
                return {"vx", "vy", "vz", "wx", "wy", "wz"};
            }
            return {"ax", "ay", "az", "alphax", "alphay", "alphaz"};
        case SignalType::ScalarArray: {
            std::vector<std::string> names;
            for (std::uint32_t i = 0; i < spec.arrayLength; ++i) {
                names.push_back(std::to_string(i));
            }
            return names;
        }
        default:
            return {};
    }
}

void appendPose(std::string& out, const Pose& pose, bool withScale,
                const std::array<float, 3>& scale) {
    char bytes[52];
    std::memcpy(bytes, pose.position.data(), 24);
    const std::array<float, 4> q{
        static_cast<float>(pose.orientation[0]), static_cast<float>(pose.orientation[1]),
        static_cast<float>(pose.orientation[2]), static_cast<float>(pose.orientation[3])};
    std::memcpy(bytes + 24, q.data(), 16);
    if (withScale) {
        std::memcpy(bytes + 40, scale.data(), 12);
    }
    out.append(bytes, withScale ? 52 : 40);
}

template <class T>
void setInline(v1::DataRef& ref, const std::vector<T>& values) {
    ref.set_inline_(values.data(), values.size() * sizeof(T));
    ref.set_size(values.size() * sizeof(T));
}

/// A collider's shape as a mesh at its size (World::Colliders conventions: half extents, radius
/// and half length), for drawing; empty for a plane.
TriMesh colliderMesh(ShapeType shape, const std::array<double, 3>& size, const ConvexHull* hull,
                     const HeightField* field = nullptr) {
    constexpr int kSegments = 24;  // round shapes: enough to read the shape, cheap to draw
    constexpr int kRings = 12;
    const auto append = [](TriMesh& to, const TriMesh& from, double dz) {
        const auto base = static_cast<std::uint32_t>(to.positions.size() / 3);
        for (std::size_t v = 0; v < from.positions.size(); v += 3) {
            to.positions.insert(to.positions.end(),
                                {from.positions[v], from.positions[v + 1],
                                 from.positions[v + 2] + static_cast<float>(dz)});
        }
        to.normals.insert(to.normals.end(), from.normals.begin(), from.normals.end());
        for (const std::uint32_t i : from.indices) {
            to.indices.push_back(base + i);
        }
    };
    switch (shape) {
        case ShapeType::Plane:
            return {};
        case ShapeType::HeightField: {
            TriMesh mesh;
            if (field == nullptr) {
                return mesh;
            }
            const ConvexHull surface = field->triangles();
            std::vector<std::array<double, 3>> normals(surface.points.size());
            for (const auto& t : surface.triangles) {
                const auto& a = surface.points[t[0]];
                const auto& b = surface.points[t[1]];
                const auto& c = surface.points[t[2]];
                const std::array<double, 3> n{
                    (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
                    (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
                    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])};
                for (const std::uint32_t k : t) {
                    for (std::size_t i = 0; i < 3; ++i) {
                        normals[k].at(i) += n.at(i);
                    }
                }
                mesh.indices.insert(mesh.indices.end(), {t[0], t[1], t[2]});
            }
            for (std::size_t v = 0; v < surface.points.size(); ++v) {
                const double len = std::hypot(normals[v][0], normals[v][1], normals[v][2]);
                for (std::size_t i = 0; i < 3; ++i) {
                    mesh.positions.push_back(static_cast<float>(surface.points[v].at(i)));
                    mesh.normals.push_back(len > 0.0 ? static_cast<float>(normals[v].at(i) / len)
                                                     : (i == 2 ? 1.0F : 0.0F));
                }
            }
            return mesh;
        }
        case ShapeType::Sphere:
            return sphereMesh(size[0], kSegments, kRings);
        case ShapeType::Box:
            return boxMesh(2.0 * size[0], 2.0 * size[1], 2.0 * size[2]);
        case ShapeType::Cylinder:
            return cylinderMesh(size[0], 2.0 * size[1], kSegments);
        case ShapeType::Capsule: {
            TriMesh mesh = cylinderMesh(size[0], 2.0 * size[1], kSegments);
            const TriMesh cap = sphereMesh(size[0], kSegments, kRings);
            append(mesh, cap, size[1]);
            append(mesh, cap, -size[1]);
            return mesh;
        }
        case ShapeType::ConvexHull: {
            TriMesh mesh;
            if (hull == nullptr || hull->points.empty()) {
                return mesh;
            }
            std::array<double, 3> centre{};
            for (const auto& p : hull->points) {
                for (std::size_t a = 0; a < 3; ++a) {
                    centre.at(a) += p.at(a) / static_cast<double>(hull->points.size());
                }
            }
            // Flat faces, each turned away from the centre whatever the triangle's winding.
            for (const auto& t : hull->triangles) {
                std::array<std::array<double, 3>, 3> corner{
                    hull->points.at(t[0]), hull->points.at(t[1]), hull->points.at(t[2])};
                const std::array<double, 3> u{corner[1][0] - corner[0][0],
                                              corner[1][1] - corner[0][1],
                                              corner[1][2] - corner[0][2]};
                const std::array<double, 3> v{corner[2][0] - corner[0][0],
                                              corner[2][1] - corner[0][1],
                                              corner[2][2] - corner[0][2]};
                std::array<double, 3> n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                                        u[0] * v[1] - u[1] * v[0]};
                const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                if (length <= 0.0) {
                    continue;
                }
                double outward = 0.0;
                for (std::size_t a = 0; a < 3; ++a) {
                    outward += n.at(a) * (corner[0].at(a) - centre.at(a));
                }
                if (outward < 0.0) {
                    std::swap(corner[1], corner[2]);
                    n = {-n[0], -n[1], -n[2]};
                }
                for (const auto& p : corner) {
                    mesh.indices.push_back(static_cast<std::uint32_t>(mesh.positions.size() / 3));
                    for (std::size_t a = 0; a < 3; ++a) {
                        mesh.positions.push_back(static_cast<float>(p.at(a)));
                        mesh.normals.push_back(static_cast<float>(n.at(a) / length));
                    }
                }
            }
            return mesh;
        }
    }
    return {};
}

/// A schema's unit text ("N.m/A", "W/(m2.K)", "ohm") as a label: "N·m/A", "W/(m²·K)", "Ω".
std::string displayUnit(const std::string& text) {
    static constexpr std::array<const char*, 10> kSuperscript{"⁰", "¹", "²", "³", "⁴",
                                                              "⁵", "⁶", "⁷", "⁸", "⁹"};
    std::string out;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char ch = text[i];
        const bool afterSymbol =
            i > 0 &&
            (std::isalpha(static_cast<unsigned char>(text[i - 1])) != 0 || text[i - 1] == ')');
        if (ch == '.') {
            out += "·";
        } else if (afterSymbol && ch == '-' && i + 1 < text.size() &&
                   std::isdigit(static_cast<unsigned char>(text[i + 1])) != 0) {
            out += "⁻";
        } else if ((afterSymbol || (!out.empty() && out.ends_with("⁻"))) &&
                   std::isdigit(static_cast<unsigned char>(ch)) != 0) {
            out += kSuperscript.at(static_cast<std::size_t>(ch - '0'));
        } else if (text.compare(i, 3, "ohm") == 0) {
            out += "Ω";
            i += 2;
        } else {
            out += ch;
        }
    }
    return out;
}

/// The scene a recording starts with (its SceneSnapshot) as a model to replay: prims at their
/// recorded poses, meshes and materials. No bodies: a replay moves prims by their recorded
/// "scene<path>/pose" signals.
Expected<Model, std::string> modelOfSnapshot(const std::string& bytes) {
    v1::SceneSnapshot snap;
    if (bytes.empty() || !snap.ParseFromString(bytes)) {
        return Unexpected{std::string("the recording has no scene snapshot to show")};
    }
    Model model;
    model.name = "replay";
    model.sourcePath = snap.stage_path();
    const std::string& poses = snap.transforms().inline_();
    constexpr std::size_t kPoseBytes = 52;  // appendPose with scale
    for (int i = 0; i < snap.prims_size(); ++i) {
        const v1::Prim& in = snap.prims(i);
        ScenePrim prim;
        prim.name = in.name();
        prim.path = in.path();
        prim.parent = in.parent();
        prim.typeName = in.type_name();
        prim.apiSchemas.assign(in.api_schemas().begin(), in.api_schemas().end());
        prim.mesh = in.mesh();
        prim.material = in.material();
        const std::size_t at = static_cast<std::size_t>(i) * kPoseBytes;
        if (at + kPoseBytes <= poses.size()) {
            std::array<float, 4> q{};
            std::memcpy(prim.offset.position.data(), poses.data() + at, 24);
            std::memcpy(q.data(), poses.data() + at + 24, 16);
            std::memcpy(prim.scale.data(), poses.data() + at + 40, 12);
            prim.offset.orientation = {q[0], q[1], q[2], q[3]};
        }
        model.scene.prims.push_back(std::move(prim));
    }
    const auto floats = [](const v1::DataRef& ref) {
        std::vector<float> out(ref.inline_().size() / sizeof(float));
        std::memcpy(out.data(), ref.inline_().data(), out.size() * sizeof(float));
        return out;
    };
    for (const v1::Mesh& in : snap.meshes()) {
        TriMesh mesh;
        mesh.positions = floats(in.positions());
        mesh.normals = floats(in.normals());
        mesh.indices.resize(in.indices().inline_().size() / sizeof(std::uint32_t));
        std::memcpy(mesh.indices.data(), in.indices().inline_().data(),
                    mesh.indices.size() * sizeof(std::uint32_t));
        model.scene.meshes.push_back(std::move(mesh));
    }
    for (const v1::Material& in : snap.materials()) {
        SceneMaterial material;
        material.name = in.name();
        for (int c = 0; c < std::min(in.base_color_size(), 4); ++c) {
            material.rgba.at(static_cast<std::size_t>(c)) = in.base_color(c);
        }
        model.scene.materials.push_back(std::move(material));
    }
    return model;
}

std::string readFile(const std::string& path) {
    std::ifstream file(path);
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

/// Resident memory of this process, from /proc/self/statm.
std::uint64_t residentBytes() {
    std::ifstream statm("/proc/self/statm");
    std::uint64_t pages = 0;
    std::uint64_t resident = 0;
    statm >> pages >> resident;
    return resident * static_cast<std::uint64_t>(::sysconf(_SC_PAGESIZE));
}

/// Samples of one signal committed since the last emit, for the viewers that subscribe.
struct Captured {
    SignalId id;
    std::size_t components{0};
    std::int64_t dtNs{0};
    SimTime lastStamp{-1};
    std::uint64_t firstIndex{0};
    std::size_t count{0};
    std::vector<double> values;
};

struct Subscription {
    std::uint32_t signal{0};
    v1::SignalSubscription::Mode mode{v1::SignalSubscription::MODE_FULL};
    std::uint32_t every{1};
    // MINMAX: the bucket being filled, carried over from one emit to the next.
    std::int64_t bucket{-1};
    std::uint32_t inBucket{0};
    std::vector<double> low, high;
};

struct Session {
    proto::Connection connection;
    bool greeted{false};
    std::uint64_t sequence{0};
    bool hasSnapshot{false};
    std::map<std::uint32_t, Subscription> subscriptions;
    std::uint64_t droppedDeltas{0};
    std::uint64_t droppedBlocks{0};
    std::uint64_t bytesOut{0};
    bool wantsRosGraph{false};
    bool wantsWorld{false};
    std::set<std::string> sensors;  // prim paths of the sensors whose frames this viewer wants
    std::map<std::string, double> fields;  // fields whose slices this viewer wants: their height

    bool congested() const { return connection.pendingOutputBytes() > kMaxQueuedBytes; }
    void send(const v1::Envelope& envelope) {
        bytesOut += envelope.ByteSizeLong() + proto::kFrameHeaderSize;
        (void)connection.queue(envelope);
    }
};

/// The domains a profile names, in the order of profiles/schema.json and the mock server.
struct DomainRow {
    const char* domain;
    const char* substitute;
    const char* substituteKind;
};
constexpr std::array<DomainRow, 14> kDomains{{
    {"rigid_body", "fixed", "constant"},
    {"electrical", "ideal", "ideal"},
    {"thermal", "ambient", "constant"},
    {"aerodynamics", "zero", "constant"},
    {"weather", "clear", "constant"},
    {"emi", "zero", "constant"},
    {"radiation", "zero", "constant"},
    {"sensors", "ground_truth", "ideal"},
    {"comms", "ideal", "ideal"},
    {"traffic", "zero", "constant"},
    {"humans", "zero", "constant"},
    {"vehicle_dynamics", "ideal", "ideal"},
    {"rotor_aero", "ideal", "ideal"},  // a drone cannot fly on zero (aerodynamics.md §5)
    {"hydrodynamics", "zero", "constant"},
}};

}  // namespace

struct Server::Impl {
    ServerOptions options;
    Robot robot;
    std::unique_ptr<Scheduler> scheduler;
    proto::Listener listener;
    std::vector<std::unique_ptr<Session>> sessions;
    std::uint64_t nextSessionId{1};
    std::uint64_t catalogGeneration{1};

    v1::RunState state{v1::RUN_STATE_RUNNING};
    bool stopped{false};        // paused at t = 0 by simulation_interfaces' STATE_STOPPED
    bool quitRequested{false};  // by simulation_interfaces' STATE_QUITTING
    double rtfTarget{1.0};
    std::string error;

    std::vector<SignalId> bodyPose;   // pose signal per body, invalid for static bodies
    std::vector<SignalId> primHinge;  // angle signal per prim with a hinge, else invalid
    std::vector<Captured> captured;   // by signal index; empty values when nobody subscribes
    std::deque<v1::LogRecord> logs;
    std::string schemaText;
    std::array<double, 3> shift{};  // world ENU of the region-local origin (moves at a rebase)
    std::shared_ptr<proto::TlsContext> tls;  // tls:// endpoints: the server's certificate

    // Streamed tiles (huge-worlds.md §2): the selection, what was there before any tile (the
    // robot, the world's other scenery), the loaded tiles' content in world ENU, and the loads
    // under way on worker threads (at most kTileWorkers at once, the rest queued).
    std::unique_ptr<streaming::TileSelector> tileSelector;
    ModelExtent baseExtent;
    std::map<std::uint32_t, std::shared_ptr<const Model>> tileContent;
    std::map<std::uint32_t, std::future<Expected<Model, std::string>>> tileLoads;
    std::deque<std::uint32_t> tileQueue;
    bool waitingForTiles{false};  // a served run holds until the robot's ground is loaded
    // A viewer's camera (world ENU), a streaming hint for runs that are not deterministic.
    std::optional<std::array<double, 3>> cameraInterest;
    std::uint64_t tilesGeneration{0};  // bumped whenever the tiles in the world change
    SimTime lastRebase{};              // simulated time of the last rebase during the run

    Clock::time_point lastWall{Clock::now()};
    Clock::time_point lastStatus{Clock::now()};
    Clock::time_point lastStats{Clock::now()};
    Clock::time_point lastFields{Clock::now()};
    mutable fields::FieldSignal fogFieldSignal, windFieldSignal;
    mutable std::vector<emi::Source> cachedEmiSources;  // emiSources()
    mutable bool emiSourcesRead{false};
    SimTime statsSimTime{};
    std::vector<std::int64_t> tickNs;

    // --- simulation ----------------------------------------------------------------------

    /// The modules for `settings`, wired: the run itself (build) or a profile's preview.
    /// The plugin module `name` of `domain` (a backend, or with `substitute` a substitute).
    [[nodiscard]] const PluginModule* pluginModule(std::string_view domain, std::string_view name,
                                                   bool substitute) const {
        for (const PluginModule& module : plugins) {
            if (module.domain == domain && module.name == name && module.substitute == substitute) {
                return &module;
            }
        }
        return nullptr;
    }

    /// `data` with the profile's config for the module's library (plugins[].config).
    static ModuleConfig pluginConfig(const PluginModule& module, ModuleConfigData data,
                                     const ProfileSettings& settings) {
        for (const ProfileSettings::Plugin& plugin : settings.plugins) {
            if (plugin.library == module.library) {
                for (const auto& [key, value] : plugin.config) {
                    data.values.insert_or_assign(key, value);
                }
            }
        }
        return ModuleConfig(std::move(data));
    }

    Expected<std::unique_ptr<Scheduler>, std::string> makeScheduler(const ProfileSettings& settings,
                                                                    DriveCommand** driveOut) {
        const auto off = [&settings](const char* domain) {
            return settings.substituted.contains(domain);
        };
        if (replay) {
            // Every domain the recording has, replayed: nothing is simulated.
            auto next = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
            std::set<std::string> domains;
            for (const recording::Recording::CatalogEntry& entry : replay->catalog) {
                domains.insert(entry.writer.substr(0, entry.writer.find('/')));
            }
            for (const std::string& domain : domains) {
                next->add(
                    std::make_unique<recording::ReplaySubstitute>(domain, options.replayPath));
            }
            auto wired = next->wire(WorldView(robot.world));
            if (!wired) {
                return Unexpected{"the recording cannot be replayed: " +
                                  to_string(wired.error().front())};
            }
            return next;
        }
        // The site's place on the Earth, for the sensors that need it.
        robot.world.geoOrigin = settings.geoOrigin;
        // A run that takes commands from ROS is not deterministic: its input comes from outside.
        // So is one with the lidar on the GPU or rendering cameras (invariant 7: the CPU is the
        // reference).
        const bool rendering = cameraScene != nullptr && !off("sensors");
        auto next = std::make_unique<Scheduler>(
            SchedulerOptions{.deterministic = !bridge && !rayContext && !rendering &&
                                              !options.px4Port && !options.ardupilotPort});
        // What plugin modules get from the host (ABI 1.1): the run's colliders for their ray and
        // overlap queries, placed at the bus's body poses when they ask.
        PluginServices pluginServices;
        if (!plugins.empty()) {
            auto scene = std::make_shared<sensors::BodyScene>(robot.world);
            pluginServices.scene = [scene](const SignalBus& bus) { return &scene->at(bus); };
        }
        // A plugin substitute the profile names for a domain (domains.<d>.substitute: <name>).
        const auto pluginSubstitute = [&](const char* domain) -> const PluginModule* {
            const auto token = settings.substituteTokens.find(domain);
            return token == settings.substituteTokens.end()
                       ? nullptr
                       : pluginModule(domain, token->second, true);
        };
        // Adds the plugin substitute chosen for `domain`; false when there is none.
        const auto addPluginSubstitute = [&](const char* domain, ModuleConfigData data = {}) {
            const PluginModule* module = pluginSubstitute(domain);
            if (module == nullptr) {
                return false;
            }
            next->add(module->create(pluginServices),
                      pluginConfig(*module, std::move(data), settings));
            return true;
        };
        if (off("rigid_body") && addPluginSubstitute("rigid_body")) {
            // the plugin stands in for physics
        } else if (off("rigid_body") && settings.rigidBodyReplay) {
            // Bench mode: physics replayed from a recording, open loop.
            next->add(std::make_unique<recording::ReplaySubstitute>("rigid_body",
                                                                    *settings.rigidBodyReplay));
        } else if (off("rigid_body")) {
            next->add(std::make_unique<rigid_body::FixedRobot>());
        } else {
            ModuleConfigData physics;
            physics.backend = settings.rigidBodyBackend.value_or(options.physics);
            if (settings.rigidBodyDtNs) {
                physics.dt = Duration{*settings.rigidBodyDtNs};
            }
            if (physics.backend == "jolt") {
                next->add(std::make_unique<rigid_body::JoltBackend>(), ModuleConfig(physics));
            } else if (physics.backend == "mujoco") {
                next->add(std::make_unique<rigid_body::MujocoBackend>(), ModuleConfig(physics));
            } else if (const PluginModule* engine =
                           pluginModule("rigid_body", physics.backend, false)) {
                next->add(engine->create(pluginServices), pluginConfig(*engine, physics, settings));
            } else {
                return Unexpected{"no rigid body backend " + physics.backend +
                                  ": mujoco, jolt, or a plugin's (--plugin)"};
            }
        }
        if (controllers) {
            // ros2_control drives the joints; the controller manager outlives this scheduler.
            next->add(ros2_control::createControlModule(*controllers, controlJoints));
        } else if (robot.actuatedJoints.size() == 2) {
            std::function<std::optional<VelocityCommand>()> external;
            if (bridge) {
                external = [this]() -> std::optional<VelocityCommand> {
                    const auto command = bridge->latestCommand(std::chrono::milliseconds(500));
                    if (!command) {
                        return std::nullopt;
                    }
                    return VelocityCommand{command->linearMps, command->angularRadps};
                };
            }
            DriveCommand::Settings demo;
            demo.demo = options.demoDrive;
            auto drive = std::make_unique<DriveCommand>(demo, external);
            if (driveOut != nullptr) {
                *driveOut = drive.get();
            }
            next->add(std::move(drive));
            next->add(std::make_unique<DiffDriveController>(driveSettings()));
        } else if (const auto swerve = swerveSettings()) {
            // A swerve base: /cmd_vel's forward, sideways and turn rates to every module.
            std::function<std::optional<VelocityCommand>()> external;
            if (bridge) {
                external = [this]() -> std::optional<VelocityCommand> {
                    const auto command = bridge->latestCommand(std::chrono::milliseconds(500));
                    if (!command) {
                        return std::nullopt;
                    }
                    return VelocityCommand{command->linearMps, command->angularRadps,
                                           command->lateralMps};
                };
            }
            DriveCommand::Settings demo;
            demo.demo = options.demoDrive;
            demo.lateral = true;
            demo.speedMps = 0.4;
            demo.turnRadiusM = 0.5;
            auto drive = std::make_unique<DriveCommand>(demo, external);
            if (driveOut != nullptr) {
                *driveOut = drive.get();
            }
            next->add(std::move(drive));
            next->add(std::make_unique<SwerveController>(*swerve));
        } else if (hasVehicles()) {
            // A road vehicle: /cmd_vel through the car's driver to throttle, brake and
            // steering; until the first command, its demo circle.
            std::function<std::optional<CarCommand>()> external;
            if (bridge) {
                external = [this]() -> std::optional<CarCommand> {
                    const auto command = bridge->latestCommand(std::chrono::milliseconds(500));
                    if (!command) {
                        return std::nullopt;
                    }
                    return CarCommand{command->linearMps, command->angularRadps,
                                      command->steeringAngle};
                };
            }
            next->add(std::make_unique<CarController>(external, options.demoDrive));
        }
        // The scenario's runner: its entities, its storyboard.
        if (scenario) {
            ModuleConfigData runner;
            const World& world = robot.world;
            for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
                if (world.bodies.parent[b] == kWorldBody && !world.bodies.kinematic[b] &&
                    movingBodies(world)[b]) {
                    runner.values["ego_body"] = world.bodies.names[b];
                    break;
                }
            }
            next->add(
                std::make_unique<openscenario::ScenarioRunner>(*scenario, options.scenarioEgo),
                ModuleConfig(runner));
        }
        // Traffic (domains/traffic.md): moves the agents addAgents made, around the robots.
        if (hasTraffic()) {
            ModuleConfigData traffic;
            traffic.values["seed"] = static_cast<std::int64_t>(options.trafficSeed);
            std::vector<std::string> egos;
            const World& world = robot.world;
            const std::vector<bool> moving = movingBodies(world);
            for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
                if (world.bodies.parent[b] == kWorldBody && moving[b] &&
                    !world.bodies.kinematic[b]) {
                    egos.push_back(world.bodies.names[b]);
                }
            }
            traffic.values["ego_bodies"] = egos;
            if (!(off("traffic") && addPluginSubstitute("traffic", traffic))) {
                next->add(std::make_unique<traffic::Traffic>(off("traffic")),
                          ModuleConfig(traffic));
            }
        }
        // People (RFC 0009): moved like traffic agents, around the robots.
        if (hasHumans()) {
            ModuleConfigData people;
            std::vector<std::string> robots;
            const World& world = robot.world;
            const std::vector<bool> moving = movingBodies(world);
            for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
                if (world.bodies.parent[b] == kWorldBody && moving[b] &&
                    !world.bodies.kinematic[b]) {
                    robots.push_back(world.bodies.names[b]);
                }
            }
            people.values["robot_bodies"] = robots;
            auto [ground, friction] = groundQuery();
            (void)friction;
            if (!(off("humans") && addPluginSubstitute("humans", people))) {
                next->add(std::make_unique<humans::Humans>(off("humans"), fixedScenery(),
                                                           std::move(ground)),
                          ModuleConfig(people));
            }
        }
        // Road vehicles (domains/vehicle_dynamics.md): before the electrical domain, which turns
        // their drive and steering commands into joint torques.
        if (hasVehicles()) {
            auto [ground, friction] = groundQuery();
            ModuleConfigData vehicle;
            vehicle.values["tire_model"] = appliedTireModel.value_or("pacejka");
            if (!(off("vehicle_dynamics") && addPluginSubstitute("vehicle_dynamics", vehicle))) {
                next->add(std::make_unique<vehicle_dynamics::VehicleDynamics>(
                              off("vehicle_dynamics"), std::move(ground), std::move(friction)),
                          ModuleConfig(vehicle));
            }
        }
        // The electrical domain turns the controllers' torque commands into joint torques: the
        // network when the robot has motors (from USD or an overlay), else its ideal substitute
        // (domains/electrical.md §5).
        if (hasMotors() && !off("electrical")) {
            ModuleConfigData power;
            power.fidelity = settings.electricalFidelity.value_or("averaged");
            // The physics step the joint speeds arrive at: speed prediction and the stability
            // check (architecture.md §8). A fixed robot has no speeds to predict.
            if (!off("rigid_body")) {
                power.values.emplace(
                    "exchange_step",
                    static_cast<double>(settings.rigidBodyDtNs.value_or(1'000'000)) * 1e-9);
            }
            power.values.emplace("speed_prediction", settings.speedPrediction.value_or(true));
            if (settings.electricalDtNs) {
                power.dt = Duration{*settings.electricalDtNs};
            }
            const bool fmu = power.fidelity == "fmu";
            next->add(std::make_unique<electrical::ElectricalNetwork>(), ModuleConfig(power));
            // At fidelity fmu, motors with a behaviour model are their FMUs (formats.md §6).
            if (fmu) {
                for (electrical::FmuBinding& binding :
                     electrical::motorFmuBindings(WorldView(robot.world))) {
                    next->add(std::make_unique<electrical::FmuComponent>(std::move(binding)));
                }
            }
        } else if (!addPluginSubstitute("electrical")) {
            next->add(std::make_unique<electrical::IdealTorqueSource>());
        }
        // Thermal, after electrical: it reads the losses. It needs the site's ambient
        // temperature; without one nothing stands in, and the electrical network keeps its
        // reference temperatures.
        if (hasThermalNodes() && settings.ambientTemperatureK) {
            ModuleConfigData heat;
            heat.fidelity = "lumped";
            heat.values.emplace(thermal::kAmbientKey, *settings.ambientTemperatureK);
            if (settings.thermalDtNs) {
                heat.dt = Duration{*settings.thermalDtNs};
            }
            if (off("thermal") && addPluginSubstitute("thermal", heat)) {
                // the plugin stands in for the thermal network
            } else if (off("thermal")) {
                next->add(std::make_unique<thermal::AmbientTemperature>(), ModuleConfig(heat));
            } else {
                next->add(std::make_unique<thermal::ThermalNetwork>(), ModuleConfig(heat));
            }
        }
        bool weatherInRun = false;  // the sensors see the weather when it runs
        // The weather (domains/weather.md): the site's atmosphere from --weather, a WeatherAPI_1
        // prim and the profile (the profile wins), the temperature falling back to the ambient
        // one. Without the site it does not run (nothing needs it yet); off, its clear substitute.
        {
            ModuleConfigData weather;
            for (const auto& [key, value] : options.weather) {
                weather.values[key] = value;
            }
            for (const auto& [key, value] : options.weatherFiles) {
                weather.values[key] = value;
            }
            for (const auto& [key, value] : settings.weather) {
                weather.values[key] = value;
            }
            if (!weather.values.contains("temperature") && settings.ambientTemperatureK) {
                weather.values["temperature"] = *settings.ambientTemperatureK;
            }
            const ModuleConfig config(weather);
            if (weather::Weather::hasSite(WorldView(robot.world), config)) {
                if (!(off("weather") && addPluginSubstitute("weather", weather))) {
                    next->add(std::make_unique<weather::Weather>(off("weather")), config);
                }
                weatherInRun = true;
            }
        }
        bool windInRun = false;
        // Link drag against the wind (an AeroAPI_1 robot, with the site's air density): the
        // aerodynamics config's own density, else rotor_aero's.
        if (!WorldView(robot.world).instancesOf("AeroAPI_1").empty() && !off("aerodynamics")) {
            ModuleConfigData aero;
            aero.values.insert(settings.aerodynamics.begin(), settings.aerodynamics.end());
            aero.values["weather"] = weatherInRun;  // its wind grid, if any
            if (!aero.values.contains("air_density") && settings.airDensityKgM3) {
                aero.values.emplace("air_density", *settings.airDensityKgM3);
            }
            if (aero.values.contains("air_density")) {
                next->add(std::make_unique<aerodynamics::AeroDrag>(), ModuleConfig(aero));
                windInRun = true;
            }
        }
        // Rotors after electrical: the rotor joints' load torque and the thrust on their links.
        // They need the site's air density; without one they do not run (as thermal without
        // the ambient temperature) and the server says so at load.
        if (hasRotors() && !settings.airDensityKgM3) {
            services::log(services::LogLevel::Warning, "rotor_aero",
                          "the rotors do not run without the site's air density: "
                          "--air-density or the profile's rotor_aero.air_density (kg/m³)");
        }
        if (hasRotors() && settings.airDensityKgM3) {
            ModuleConfigData air;
            air.values.insert(settings.rotorAero.begin(), settings.rotorAero.end());
            // The rotors fly in the aerodynamics domain's wind when it is in the run.
            air.values.emplace("wind", windInRun);
            air.values.emplace(rotor_aero::kAirDensityKey, *settings.airDensityKgM3);
            next->add(std::make_unique<rotor_aero::RotorAero>(off("rotor_aero")),
                      ModuleConfig(air));
            if (options.ardupilotPort) {
                if (!ardupilotLink) {
                    std::string why;
                    ardupilotLink = mavlink::openArduPilotLink(*options.ardupilotPort, why);
                    if (!ardupilotLink) {
                        return Unexpected{why};
                    }
                }
                mavlink::ArduPilotSettings ardupilot;
                ardupilot.port = *options.ardupilotPort;
                ardupilot.airDensity = *settings.airDensityKgM3;
                next->add(mavlink::createArduPilotJson(ardupilot, ardupilotLink));
            } else if (options.px4Port) {
                if (!px4Link) {
                    std::string why;
                    px4Link = mavlink::openPx4Link(*options.px4Port, why);
                    if (!px4Link) {
                        return Unexpected{why};
                    }
                }
                mavlink::Px4Settings px4;
                px4.port = *options.px4Port;
                px4.airDensity = *settings.airDensityKgM3;
                next->add(mavlink::createPx4Lockstep(px4, px4Link));
            } else if (!controllers && MulticopterController::canFly(WorldView(robot.world))) {
                // The built-in controller hovers it, and with ROS flies it by /cmd_vel: forward,
                // left and up in m/s, yaw rate in rad/s; without a command for 0.5 s it holds.
                MulticopterController::CommandSource commands;
                if (bridge) {
                    commands = [this]() -> std::optional<MulticopterController::FlightCommand> {
                        const auto c = bridge->latestCommand(std::chrono::milliseconds(500));
                        if (!c) {
                            return std::nullopt;
                        }
                        return MulticopterController::FlightCommand{
                            c->linearMps, c->lateralMps, c->verticalMps, c->angularRadps};
                    };
                }
                next->add(std::make_unique<MulticopterController>(MulticopterController::Settings{},
                                                                  std::move(commands)),
                          ModuleConfig(air));
            }
        }
        // EMI (domains/emi.md) when the stage has sources or victims: before the sensors, which
        // read its disturbances; off, its zero substitute.
        if (hasEmi()) {
            ModuleConfigData emi;
            if (settings.emiCoupling) {
                emi.values["coupling"] = *settings.emiCoupling;
            }
            if (!(off("emi") && addPluginSubstitute("emi", emi))) {
                next->add(std::make_unique<emi::Emi>(off("emi")), ModuleConfig(emi));
            }
        }
        // Comms (domains/comms.md) when the stage has radio links or CAN buses: after EMI, whose
        // interference and bit errors it reads; walls are the fixed scenery's shapes.
        if (hasComms()) {
            if (!(off("comms") && addPluginSubstitute("comms"))) {
                next->add(std::make_unique<comms::Comms>(off("comms"), fixedScenery()));
            }
        }
        ModuleConfigData sensing;
        sensing.values["weather"] = weatherInRun;
        sensing.tier =
            rayContext ? static_cast<CapabilityTier>(rayContext->tier()) : CapabilityTier::Tier0;
        for (auto& sensor : off("sensors")
                                ? sensors::createSensorSubstitutes(robot.world)
                                : sensors::createSensorModules(robot.world, rayContext, cameraScene,
                                                               !options.cameras)) {
            next->add(std::move(sensor), ModuleConfig(sensing));
        }
        // Topics written into signals (--ros-topics subscribe), for plugins to read.
        if (bridge) {
            if (auto inputs = bridge->topicInputs()) {
                next->add(std::move(inputs));
            }
        }
        // Plugins' own domains (RFC 0008): each runs unless --domain <d>=off.
        for (const PluginModule& module : plugins) {
            if (!isBuiltInDomain(module.domain) && !module.substitute &&
                !settings.substituted.contains(module.domain)) {
                next->add(module.create(pluginServices), pluginConfig(module, {}, settings));
            }
        }
        // Always last: it sums the body/<b>/wrench/<source> written by the modules above.
        next->add(std::make_unique<services::WrenchSum>());
        auto wired = next->wire(WorldView(robot.world));
        if (!wired) {
            std::string message = "the simulation could not start:";
            for (const WiringError& e : wired.error()) {
                message += "\n  " + to_string(e);
            }
            return Unexpected{message};
        }
        return next;
    }

    /// The settings of the profile the run uses (the command line's, or the last applied).
    ProfileSettings active() const {
        ProfileSettings settings;
        settings.substituted = options.substituted;
        settings.rigidBodyDtNs = rigidBodyDtNs;
        settings.rigidBodyBackend = rigidBodyBackend;
        settings.electricalDtNs = electricalDtNs;
        settings.thermalDtNs = thermalDtNs;
        settings.speedPrediction = speedPrediction;
        settings.electricalFidelity = electricalFidelity;
        settings.rigidBodyReplay = rigidBodyReplay;
        if (!settings.rigidBodyReplay && !options.benchReplayPath.empty()) {
            settings.rigidBodyReplay = options.benchReplayPath;  // --bench
        }
        settings.ambientTemperatureK = ambient();
        settings.airDensityKgM3 = appliedAirDensity ? appliedAirDensity : options.airDensityKgM3;
        settings.geoOrigin = appliedGeoOrigin ? appliedGeoOrigin : options.geoOrigin;
        settings.streamingLoadRadius = appliedLoadRadius;
        settings.streamingUnloadRadius = appliedUnloadRadius;
        settings.streamingCollisionRadius = appliedCollisionRadius;
        settings.aerodynamics = appliedAerodynamics;
        settings.rotorAero = appliedRotorAero;
        settings.weather = appliedWeather;
        settings.emiCoupling = appliedEmiCoupling;
        settings.substituteTokens = appliedSubstituteTokens;
        settings.plugins = appliedPlugins;
        return settings;
    }
    /// Whether the world has traffic agents.
    bool hasTraffic() const {
        return std::ranges::any_of(robot.world.bodies.names, [](const std::string& n) {
            return n.starts_with("traffic_vehicle_") || n.starts_with("traffic_pedestrian_");
        });
    }
    /// Whether the run has people.
    bool hasHumans() const { return !WorldView(robot.world).instancesOf("HumanAPI_1").empty(); }
    /// Whether the stage has road vehicles.
    bool hasVehicles() const { return !WorldView(robot.world).instancesOf("VehicleAPI_1").empty(); }
    /// A scene query over what wheels roll on: the fixed scenery's shapes and the ground
    /// plane, with each shape's friction coefficient.
    std::pair<std::shared_ptr<const services::SceneQuery>, std::vector<double>> groundQuery()
        const {
        const World& world = robot.world;
        const std::vector<Pose> rest = restPoses(world);
        const std::vector<bool> moving = movingBodies(world);
        std::vector<sensors::ShapeCaster::Shape> shapes;
        std::vector<double> friction;
        for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
            const BodyId owner = world.colliders.body[c];
            if (owner != kWorldBody && moving[owner.index]) {
                continue;
            }
            sensors::ShapeCaster::Shape shape;
            shape.type = world.colliders.shape[c];
            shape.size = world.colliders.size[c];
            shape.pose = owner == kWorldBody
                             ? world.colliders.poseInBody[c]
                             : compose(rest[owner.index], world.colliders.poseInBody[c]);
            shape.hull = shape.type == ShapeType::ConvexHull
                             ? &world.hulls.at(world.colliders.hull[c])
                             : nullptr;
            shape.field = shape.type == ShapeType::HeightField
                              ? &world.heightfields.at(world.colliders.hull[c])
                              : nullptr;
            shapes.push_back(shape);
            friction.push_back(world.colliders.friction[c]);
        }
        auto caster = std::make_shared<sensors::ShapeCaster>();
        caster->update(shapes);
        return {std::move(caster), std::move(friction)};
    }
    /// Whether the stage has radio links or CAN buses.
    bool hasComms() const {
        const WorldView world(robot.world);
        return !world.instancesOf("RadioLinkAPI_1").empty() ||
               !world.instancesOf("CanBusAPI_1").empty();
    }
    /// A scene query over the fixed scenery's shapes (not the ground plane, not what moves), for
    /// the walls between radios.
    std::shared_ptr<const services::SceneQuery> fixedScenery() const {
        const World& world = robot.world;
        const std::vector<Pose> rest = restPoses(world);
        const std::vector<bool> moving = movingBodies(world);
        std::vector<sensors::ShapeCaster::Shape> shapes;
        for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
            const BodyId owner = world.colliders.body[c];
            if (world.colliders.shape[c] == ShapeType::Plane ||
                (owner != kWorldBody && moving[owner.index])) {
                continue;
            }
            sensors::ShapeCaster::Shape shape;
            shape.type = world.colliders.shape[c];
            shape.size = world.colliders.size[c];
            shape.pose = owner == kWorldBody
                             ? world.colliders.poseInBody[c]
                             : compose(rest[owner.index], world.colliders.poseInBody[c]);
            shape.hull = shape.type == ShapeType::ConvexHull
                             ? &world.hulls.at(world.colliders.hull[c])
                             : nullptr;
            shape.field = shape.type == ShapeType::HeightField
                              ? &world.heightfields.at(world.colliders.hull[c])
                              : nullptr;
            shapes.push_back(shape);
        }
        if (shapes.empty()) {
            return nullptr;
        }
        auto caster = std::make_shared<sensors::ShapeCaster>();
        caster->update(shapes);
        return caster;
    }
    /// Whether the stage has EMI sources or victims.
    bool hasEmi() const {
        const WorldView world(robot.world);
        return !world.instancesOf("EmiSourceAPI_1").empty() ||
               !world.instancesOf("EmiSusceptibilityAPI_1").empty();
    }
    /// Whether the EMI module itself (not its zero substitute) is in the run.
    bool emiRuns() const {
        return scheduler != nullptr &&
               std::ranges::any_of(scheduler->plan(), [](const ScheduledModule& m) {
                   return std::string_view(m.label) == "emi/analytic";
               });
    }
    /// Whether the weather module (or its substitute) is in the run.
    bool weatherRuns() const {
        return scheduler != nullptr &&
               std::ranges::any_of(scheduler->plan(), [](const ScheduledModule& m) {
                   return std::string_view(m.label).starts_with("weather/");
               });
    }
    /// The ambient temperature the run uses: an applied profile's, else the command line's.
    std::optional<double> ambient() const {
        return appliedAmbientK ? appliedAmbientK : options.ambientTemperatureK;
    }
    std::optional<std::int64_t> thermalDtNs;           // from an applied profile
    std::optional<double> appliedAmbientK;             // from an applied profile
    std::optional<double> appliedAirDensity;           // from an applied profile
    std::optional<coords::Geodetic> appliedGeoOrigin;  // from an applied profile
    std::optional<double> appliedLoadRadius;    // world.streaming.load_radius of an applied profile
    std::optional<double> appliedUnloadRadius;  // world.streaming.unload_radius
    std::optional<double> appliedCollisionRadius;  // world.streaming.collision_radius
    std::map<std::string, ConfigValue, std::less<>> appliedAerodynamics;  // its wind
    std::map<std::string, ConfigValue, std::less<>> appliedRotorAero;     // its rotor settings
    std::map<std::string, ConfigValue, std::less<>> appliedWeather;       // its weather
    std::optional<std::vector<std::string>> appliedEmiCoupling;           // its EMI victims
    std::shared_ptr<mavlink::Px4Link> px4Link;  // PX4's connection, across rebuilds of the run
    std::shared_ptr<mavlink::ArduPilotLink> ardupilotLink;       // ArduPilot's, likewise
    std::optional<bool> speedPrediction;                         // from an applied profile
    std::optional<std::string> electricalFidelity;               // from an applied profile
    std::optional<std::string> rigidBodyReplay;                  // --bench, or an applied profile
    std::optional<std::int64_t> rigidBodyDtNs;                   // from an applied profile
    std::optional<std::string> rigidBodyBackend;                 // from an applied profile
    std::optional<std::string> appliedTireModel;                 // vehicle_dynamics.tire_model
    std::optional<openscenario::Scenario> scenario;              // --scenario
    std::vector<PluginModule> plugins;                           // --plugin (RFC 0008)
    std::map<std::string, std::string> appliedSubstituteTokens;  // from an applied profile
    std::vector<ProfileSettings::Plugin> appliedPlugins;         // its plugins' config
    SignalId scenarioState, scenarioTime;                        // its runner's, when it has one
    bool scenarioEnded{false};                                   // logged once
    std::optional<std::int64_t> electricalDtNs;                  // from an applied profile
    std::optional<ProfileSettings> pending;  // applied without restart: used at the next reset

    Status build() {
        DriveCommand* drive = nullptr;
        auto made = makeScheduler(active(), &drive);
        if (!made) {
            return Status::error(StatusCode::InvalidConfig, made.error());
        }
        scheduler = std::move(*made);
        driveCommand = drive;  // only now: a failed build leaves the run as it was
        scenarioState = scheduler->bus().find("scenario/state");
        scenarioTime = scheduler->bus().find("scenario/time");
        scenarioEnded = false;
        bodyPose.assign(robot.world.bodies.names.size(), SignalId{});
        for (std::size_t b = 0; b < bodyPose.size(); ++b) {
            bodyPose[b] = scheduler->bus().find("body/" + robot.world.bodies.names[b] + "/pose");
        }
        primHinge.assign(robot.scene.prims.size(), SignalId{});
        for (std::size_t p = 0; p < primHinge.size(); ++p) {
            if (!robot.scene.prims[p].hinge.empty()) {
                primHinge[p] = scheduler->bus().find(robot.scene.prims[p].hinge);
            }
        }
        captured.assign(scheduler->bus().signalCount(), Captured{});
        for (std::size_t s = 0; s < captured.size(); ++s) {
            const SignalId id = SignalId::output(static_cast<std::uint32_t>(s));
            captured[s].id = id;
            captured[s].components = componentCount(scheduler->bus().specAt(s).type,
                                                    scheduler->bus().specAt(s).arrayLength);
            captured[s].dtNs = scheduler->writerOf(id).dt.ns;
        }
        statsSimTime = SimTime{};
        (void)scheduler->takeCosts();
        if (bridge) {
            bridge->attach(robot.world, scheduler->bus());
            if (!bridge->topicProblems().empty()) {
                std::string message = "--ros-topics:";
                for (const std::string& problem : bridge->topicProblems()) {
                    message += "\n  " + problem;
                }
                return Status::error(StatusCode::InvalidConfig, message);
            }
        }
        lidarSent.assign(robot.world.sensors.names.size(), SimTime{-1});
        if (replay) {
            startReplay();
        }
        return {};
    }

    /// Wheel radius and track of a two-wheel robot, from its wheel colliders and positions.
    DiffDriveController::Settings driveSettings() const {
        DiffDriveController::Settings settings;
        settings.leftJoint = robot.actuatedJoints[0];
        settings.rightJoint = robot.actuatedJoints[1];
        const World& world = robot.world;
        std::array<std::uint32_t, 2> wheels{};
        for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
            for (std::size_t w = 0; w < 2; ++w) {
                if (world.joints.names[j] == robot.actuatedJoints[w]) {
                    wheels[w] = world.joints.child[j].index;
                }
            }
        }
        for (std::size_t c = 0; c < world.colliders.body.size(); ++c) {
            if (world.colliders.body[c].index == wheels[0] &&
                (world.colliders.shape[c] == ShapeType::Cylinder ||
                 world.colliders.shape[c] == ShapeType::Sphere)) {
                settings.wheelRadiusM = world.colliders.size[c][0];
            }
        }
        const Pose& a = robot.bodyRestPose[wheels[0]];
        const Pose& b = robot.bodyRestPose[wheels[1]];
        settings.trackWidthM =
            std::hypot(a.position[0] - b.position[0], a.position[1] - b.position[1]);
        return settings;
    }

    /// A swerve base's modules, when the robot is one (findSwerveModules).
    std::optional<SwerveController::Settings> swerveSettings() const {
        return findSwerveModules(robot);
    }

    /// The latest scan of each lidar and image of each camera to the viewers that watch it.
    void sendSensorFrames() {
        const World::Sensors& table = robot.world.sensors;
        const SignalBus& bus = scheduler->bus();
        for (std::size_t s = 0; s < table.names.size(); ++s) {
            const std::string& path = robot.scene.prims[robot.sensorPrim[s]].path;
            const bool watched = std::any_of(sessions.begin(), sessions.end(), [&](const auto& x) {
                return x->greeted && x->sensors.contains(path);
            });
            if (table.type[s] == SensorType::Camera && watched) {
                sendImage(s, path);
            }
            if (table.type[s] != SensorType::Lidar) {
                continue;
            }
            const SignalId id = bus.find("sensor/" + table.names[s] + "/ranges");
            if (!watched || !id.valid() || bus.stamp(id) == lidarSent[s]) {
                continue;
            }
            lidarSent[s] = bus.stamp(id);
            const LidarLayout layout = LidarLayout::from(table.settings[s]);
            const std::span<const double> ranges = bus.readN(id);
            std::vector<float> points;
            for (std::uint32_t i = 0; i < ranges.size() && i < layout.beams(); ++i) {
                if (!std::isfinite(ranges[i])) {
                    continue;
                }
                // Sensor frame, as the mock server sends. Intensity: no reflectivity model yet.
                const std::array<double, 3> d = layout.direction(i);
                points.insert(points.end(), {static_cast<float>(ranges[i] * d[0]),
                                             static_cast<float>(ranges[i] * d[1]),
                                             static_cast<float>(ranges[i] * d[2]), 1.0F});
            }
            v1::Envelope envelope;
            v1::SensorFrame& frame = *envelope.mutable_sensor_frame();
            frame.set_sensor(path);
            frame.mutable_sim_time()->set_ns(lidarSent[s].ns);
            v1::PointCloudData& cloud = *frame.mutable_points();
            cloud.set_count(static_cast<std::uint32_t>(points.size() / 4));
            cloud.set_stride(16);
            for (const char* field : {"x", "y", "z", "intensity"}) {
                cloud.add_fields(field);
            }
            setInline(*cloud.mutable_data(), points);
            for (auto& session : sessions) {
                if (session->greeted && session->sensors.contains(path) && !session->congested()) {
                    session->send(envelope);
                }
            }
        }
    }
    std::vector<SimTime> lidarSent;  // per sensor: stamp of the last frame sent (lidar, camera)

    /// A camera's colour image (rgba8, inline), once per new frame. An empty buffer (the
    /// ground-truth substitute, or before the first frame) is not sent.
    void sendImage(std::size_t s, const std::string& path) {
        const SignalBus& bus = scheduler->bus();
        const SignalId id = bus.find("sensor/" + robot.world.sensors.names[s] + "/image");
        if (!id.valid() || bus.stamp(id) == lidarSent[s]) {
            return;
        }
        lidarSent[s] = bus.stamp(id);
        const std::shared_ptr<const Buffer> image = bus.buffer(bus.readBuffer(id));
        if (!image || image->bytes.empty()) {
            return;
        }
        v1::Envelope envelope;
        v1::SensorFrame& frame = *envelope.mutable_sensor_frame();
        frame.set_sensor(path);
        frame.mutable_sim_time()->set_ns(lidarSent[s].ns);
        v1::ImageData& data = *frame.mutable_image();
        data.set_width(image->width);
        data.set_height(image->height);
        data.set_encoding(image->encoding);
        data.mutable_data()->set_inline_(image->bytes.data(), image->bytes.size());
        data.mutable_data()->set_size(image->bytes.size());
        for (auto& session : sessions) {
            if (session->greeted && session->sensors.contains(path) && !session->congested()) {
                session->send(envelope);
            }
        }
    }

    /// Records the samples committed in the last tick of every signal a viewer subscribes to.
    void capture() {
        const SignalBus& bus = scheduler->bus();
        for (Captured& c : captured) {
            if (c.components == 0 || !wanted(c.id.index)) {
                continue;
            }
            const SimTime stamp = bus.stamp(c.id);
            if (stamp == c.lastStamp) {
                continue;
            }
            c.lastStamp = stamp;
            const auto index =
                static_cast<std::uint64_t>(stamp.ns / std::max<std::int64_t>(c.dtNs, 1));
            if (c.count == 0 || index != c.firstIndex + c.count) {
                c.values.clear();
                c.count = 0;
                c.firstIndex = index;
            }
            const std::span<const double> values = bus.readN(c.id);
            c.values.insert(c.values.end(), values.begin(), values.end());
            ++c.count;
        }
    }

    bool wanted(std::uint32_t signal) const {
        return std::any_of(sessions.begin(), sessions.end(),
                           [signal](const auto& s) { return s->subscriptions.contains(signal); });
    }

    /// Simulation time owed to the wall clock, in ns. Whole base ticks are stepped and the
    /// remainder carries over, so the real-time factor averages exactly the target instead of
    /// rounding every call up to a tick.
    std::int64_t owedNs{0};

    Status advance(std::int64_t simNs) {
        const std::int64_t tick = scheduler->baseTick().ns;
        // Never owe more than 100 ms: a run that cannot keep up falls behind real time.
        owedNs = std::min<std::int64_t>(owedNs + simNs, 100'000'000);
        const Clock::time_point began = Clock::now();
        if (waitingForTiles) {
            if (Status status = updateTiles(false); !status.ok()) {
                return status;
            }
            if (waitingForTiles) {
                owedNs = 0;
                return {};
            }
        }
        while (owedNs >= tick) {
            owedNs -= tick;
            const Clock::time_point tickStart = Clock::now();
            if (Status status = scheduler->stepTick(); !status.ok()) {
                return status;
            }
            tickNs.push_back(nanosSince(tickStart, Clock::now()));
            capture();
            record();
            if (Status status = worldUpdate(false); !status.ok()) {
                return status;
            }
            if (waitingForTiles) {
                owedNs = 0;  // hold: the robot's ground is not in physics yet
                break;
            }
            if (scenarioState.valid() && !scenarioEnded &&
                scheduler->bus().read(scenarioState) == 1.0) {
                scenarioEnded = true;
                char line[96];
                std::snprintf(line, sizeof(line),
                              "the scenario ended at its stop trigger, t = %.2f s",
                              scheduler->bus().read(scenarioTime));
                log(v1::LOG_LEVEL_INFO, "scenario", line);
            }
            if (replay) {
                if (scheduler->now().ns % kCheckpointNs == 0) {
                    checkpoint();
                }
                if (scheduler->now() >= replay->duration) {
                    state = v1::RUN_STATE_PAUSED;  // the end of the recording
                    owedNs = 0;
                    log(v1::LOG_LEVEL_INFO, "replay", "end of the recording");
                    break;
                }
            }
            if (bridge) {
                bridge->publish(scheduler->bus(), scheduler->now());
            }
            if (nanosSince(began, Clock::now()) > kMaxStepWallNs) {
                owedNs = 0;  // fall behind real time rather than stall the viewers
                break;
            }
        }
        return {};
    }

    // --- recording (roadmap 0.2-E4) ------------------------------------------------------

    /// World poses of the moving prims, recorded as signals "scene<path>/pose" so that a replay
    /// moves every prim (visuals and sensors sit at an offset from their body).
    static constexpr std::int64_t kScenePoseNs = 20'000'000;  // 50 Hz, a viewer's frame rate
    static constexpr std::int64_t kStateHashNs = 1'000'000'000;
    std::unique_ptr<Recorder> recorder;
    std::vector<SimTime> recordedStamp;            // per bus signal
    std::vector<std::uint32_t> recordedId;         // per bus signal: id in the file, or kNoIndex
    std::uint32_t recordedSignals{0};              // bus signals in the file
    std::vector<std::uint32_t> recordedPosePrims;  // scene prims with a pose signal
    std::uint64_t nextPoseIndex{0};
    std::int64_t nextHashNs{0};

    record::v1::RunMeta runMeta() const {
        record::v1::RunMeta meta;
        meta.set_format_version(1);
        meta.set_profile_yaml(profileYaml());
        meta.set_stage_path(robot.sourcePath);
        meta.set_generator(kServerName);
        meta.set_description("recorded by " + std::string(kServerName));
        for (const std::uint32_t prim : recordedPosePrims) {
            record::v1::PoseBinding& binding = *meta.add_pose_bindings();
            binding.set_prim(prim);
            binding.set_signal("scene" + robot.scene.prims[prim].path + "/pose");
        }
        return meta;
    }

    Status startRecording(const std::string& path) {
        if (replay) {
            return Status::error(StatusCode::Unsupported,
                                 "a replay is not recorded again: copy the file instead");
        }
        auto created = Recorder::create(path);
        if (!created) {
            return Status::error(StatusCode::InvalidConfig, created.error());
        }
        recorder = std::move(*created);
        recordedPosePrims.clear();
        for (std::uint32_t p = 0; p < robot.scene.prims.size(); ++p) {
            const std::uint32_t body = robot.scene.prims[p].body;
            if (body != kNoIndex && bodyPose[body].valid()) {
                recordedPosePrims.push_back(p);
            }
        }
        // The bus signals that fit, renumbered 0..n-1, then the prim poses.
        const v1::Envelope full = catalog();
        v1::SignalCatalog signals;
        signals.set_generation(1);
        recordedId.assign(scheduler->bus().signalCount(), kNoIndex);
        std::vector<std::string> left;
        for (const v1::SignalInfo& info : full.catalog().signals()) {
            if (info.components() > options.recordMaxComponents) {
                left.push_back(info.name());
                continue;
            }
            recordedId[info.id()] = static_cast<std::uint32_t>(signals.signals_size());
            v1::SignalInfo& kept = *signals.add_signals();
            kept = info;
            kept.set_id(recordedId[info.id()]);
        }
        recordedSignals = static_cast<std::uint32_t>(signals.signals_size());
        SignalSpec pose;
        pose.type = SignalType::Pose;
        pose.unit = units::m;
        for (std::size_t k = 0; k < recordedPosePrims.size(); ++k) {
            const ScenePrim& prim = robot.scene.prims[recordedPosePrims[k]];
            v1::SignalInfo& info = *signals.add_signals();
            info.set_id(static_cast<std::uint32_t>(signals.signals_size() - 1));
            info.set_name("scene" + prim.path + "/pose");
            info.set_type(static_cast<v1::SignalType>(static_cast<int>(SignalType::Pose) + 1));
            setUnit(*info.mutable_unit(), units::m);
            info.set_components(7);
            info.set_native_dt_ns(kScenePoseNs);
            info.set_writer("recorder");
            info.set_prim(recordedPosePrims[k]);
            for (const std::string& name : componentNames(pose)) {
                info.add_component_names(name);
            }
        }
        recorder->begin(runMeta(), signals, snapshotAt(0).snapshot());
        recordedStamp.assign(scheduler->bus().signalCount(), SimTime{-1});
        nextPoseIndex = 0;
        nextHashNs = 0;
        record();  // the values at t = 0
        log(v1::LOG_LEVEL_INFO, "recorder", "recording to " + path);
        for (const std::string& name : left) {
            log(v1::LOG_LEVEL_INFO, "recorder",
                name + " is not recorded: larger than --record-max-components");
        }
        return {};
    }

    /// The samples written by the last tick, the prim poses that fall due, and a state hash a
    /// second.
    void record() {
        if (!recorder) {
            return;
        }
        const SignalBus& bus = scheduler->bus();
        for (std::uint32_t s = 0; s < bus.signalCount(); ++s) {
            const SignalId id = SignalId::output(s);
            const SimTime stamp = bus.stamp(id);
            if (recordedId[s] == kNoIndex || stamp == recordedStamp[s] || stamp.ns < 0) {
                continue;
            }
            recordedStamp[s] = stamp;
            const std::int64_t dt = std::max<std::int64_t>(scheduler->writerOf(id).dt.ns, 1);
            recorder->add(recordedId[s], static_cast<std::uint64_t>(stamp.ns / dt), bus.readN(id));
        }
        const SimTime now = scheduler->now();
        for (; static_cast<std::int64_t>(nextPoseIndex) * kScenePoseNs <= now.ns; ++nextPoseIndex) {
            for (std::size_t k = 0; k < recordedPosePrims.size(); ++k) {
                const Pose p = primPose(robot.scene.prims[recordedPosePrims[k]]);
                const std::array<double, 7> values{
                    p.position[0],    p.position[1],    p.position[2],   p.orientation[0],
                    p.orientation[1], p.orientation[2], p.orientation[3]};
                recorder->add(recordedSignals + static_cast<std::uint32_t>(k), nextPoseIndex,
                              values);
            }
        }
        if (now.ns >= nextHashNs) {
            if (const auto hash = scheduler->stateHash()) {
                recorder->stateHash(now, *hash);
            }
            nextHashNs += kStateHashNs;
        }
        recorder->flush(now);
    }

    void stopRecording(const std::string& why) {
        if (!recorder) {
            return;
        }
        const std::string path = recorder->path();
        if (auto done = recorder->finish(scheduler->now(), runMeta()); !done) {
            log(v1::LOG_LEVEL_ERROR, "recorder", done.error());
        } else {
            log(v1::LOG_LEVEL_INFO, "recorder",
                "recorded " + path + " (" + why + ") · " +
                    std::to_string(scheduler->now().ns / 1'000'000) + " ms");
        }
        recorder.reset();
    }

    void fail(const Status& status) {
        state = v1::RUN_STATE_ERROR;
        error = status.message;
        log(v1::LOG_LEVEL_ERROR, "scheduler", status.message);
        broadcast(statusEnvelope());
    }

    // --- scene -----------------------------------------------------------------------------

    Pose primPose(const ScenePrim& prim) const {
        if (!replayPose.empty()) {
            const auto index = static_cast<std::size_t>(&prim - robot.scene.prims.data());
            if (index < replayPose.size() && replayPose[index].valid()) {
                const std::span<const double> v = scheduler->bus().readN(replayPose[index]);
                return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
            }
            return prim.offset;
        }
        if (prim.body == kNoIndex) {
            return prim.offset;
        }
        Pose body = robot.bodyRestPose[prim.body];
        const SignalId id = bodyPose[prim.body];
        if (id.valid()) {
            const std::span<const double> v = scheduler->bus().readN(id);
            body = {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
        }
        const Pose placed = compose(body, prim.offset);
        const auto index = static_cast<std::size_t>(&prim - robot.scene.prims.data());
        if (index < primHinge.size() && primHinge[index].valid()) {
            // Turned on its hinge by the signal's angle.
            const double half = 0.5 * scheduler->bus().read(primHinge[index]);
            const std::array<double, 3>& a = prim.hingeAxis;
            const double s = std::sin(half);
            return compose(placed, Pose{{}, {std::cos(half), s * a[0], s * a[1], s * a[2]}});
        }
        return placed;
    }

    v1::Envelope snapshot(Session& session) {
        session.hasSnapshot = true;
        return snapshotAt(++session.sequence);
    }

    /// The region frame: where the floating origin is and how often it moved.
    [[nodiscard]] v1::LocalFrame localFrame() const {
        v1::LocalFrame frame;
        frame.set_rebase_count(robot.world.region.rebaseCount);
        for (const double v : robot.world.region.worldEnu) {
            frame.add_region_enu(v);
        }
        for (const double v : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
            frame.add_enu_from_ecef(v);
        }
        return frame;
    }

    v1::Envelope snapshotAt(std::uint64_t sequence) {
        v1::Envelope envelope;
        v1::SceneSnapshot& snap = *envelope.mutable_snapshot();
        snap.set_sequence(sequence);
        snap.mutable_sim_time()->set_ns(scheduler->now().ns);
        *snap.mutable_frame() = localFrame();
        std::vector<std::uint32_t> children(robot.scene.prims.size(), 0);
        for (const ScenePrim& prim : robot.scene.prims) {
            if (prim.parent != kNoIndex) {
                ++children[prim.parent];
            }
        }
        std::string transforms;
        for (std::uint32_t i = 0; i < robot.scene.prims.size(); ++i) {
            const ScenePrim& prim = robot.scene.prims[i];
            v1::Prim& out = *snap.add_prims();
            out.set_id(i);
            out.set_parent(prim.parent);
            out.set_name(prim.name);
            out.set_path(prim.path);
            out.set_type_name(prim.typeName);
            for (const std::string& schema : prim.apiSchemas) {
                out.add_api_schemas(schema);
            }
            out.set_mesh(prim.mesh);
            out.set_material(prim.material);
            out.set_visible(true);
            out.set_child_count(children[i]);
            appendPose(transforms, primPose(prim), true, prim.scale);
        }
        snap.mutable_transforms()->set_inline_(transforms);
        snap.mutable_transforms()->set_size(transforms.size());
        for (std::uint32_t m = 0; m < robot.scene.meshes.size(); ++m) {
            const TriMesh& mesh = robot.scene.meshes[m];
            v1::Mesh& out = *snap.add_meshes();
            out.set_id(m);
            out.set_vertex_count(static_cast<std::uint32_t>(mesh.positions.size() / 3));
            out.set_index_count(static_cast<std::uint32_t>(mesh.indices.size()));
            setInline(*out.mutable_positions(), mesh.positions);
            setInline(*out.mutable_normals(), mesh.normals);
            setInline(*out.mutable_indices(), mesh.indices);
            std::array<float, 6> box{1e30F, 1e30F, 1e30F, -1e30F, -1e30F, -1e30F};
            for (std::size_t v = 0; v + 2 < mesh.positions.size(); v += 3) {
                for (std::size_t a = 0; a < 3; ++a) {
                    box[a] = std::min(box[a], mesh.positions[v + a]);
                    box[a + 3] = std::max(box[a + 3], mesh.positions[v + a]);
                }
            }
            for (const float value : box) {
                out.add_aabb(value);
            }
        }
        colliders(snap);
        for (std::uint32_t m = 0; m < robot.scene.materials.size(); ++m) {
            const SceneMaterial& material = robot.scene.materials[m];
            v1::Material& out = *snap.add_materials();
            out.set_id(m);
            out.set_name(material.name);
            for (const float c : material.rgba) {
                out.add_base_color(c);
            }
            out.set_roughness(0.6F);
        }
        if (replay) {
            *snap.mutable_nets() = replayNets;  // the recorded power system
        } else {
            electricalNets(snap);
        }
        snap.set_topology_hash(WorldView(robot.world).topologyHash());
        snap.set_stage_path(robot.sourcePath);
        return envelope;
    }

    v1::Envelope delta(Session& session) const {
        v1::Envelope envelope;
        v1::SceneDelta& d = *envelope.mutable_delta();
        d.set_base_sequence(session.sequence);
        d.set_sequence(++session.sequence);
        d.mutable_sim_time()->set_ns(scheduler->now().ns);
        std::string data;
        for (std::uint32_t i = 0; i < robot.scene.prims.size(); ++i) {
            const ScenePrim& prim = robot.scene.prims[i];
            if (prim.body == kNoIndex || !bodyPose[prim.body].valid()) {
                continue;  // static: never changes
            }
            d.add_transform_prims(i);
            appendPose(data, primPose(prim), false, prim.scale);
        }
        d.set_transform_data(data);
        return envelope;
    }

    // --- signals ---------------------------------------------------------------------------

    v1::Envelope catalog() const {
        v1::Envelope envelope;
        v1::SignalCatalog& out = *envelope.mutable_catalog();
        out.set_generation(catalogGeneration);
        const SignalBus& bus = scheduler->bus();
        for (std::uint32_t s = 0; s < bus.signalCount(); ++s) {
            const SignalSpec& spec = bus.specAt(s);
            const SignalWriter writer = scheduler->writerOf(SignalId::output(s));
            v1::SignalInfo& info = *out.add_signals();
            info.set_id(s);
            info.set_name(spec.name);
            info.set_type(static_cast<v1::SignalType>(static_cast<int>(spec.type) + 1));
            setUnit(*info.mutable_unit(), spec.unit);
            info.set_components(
                static_cast<std::uint32_t>(componentCount(spec.type, spec.arrayLength)));
            info.set_native_dt_ns(writer.dt.ns);
            info.set_writer(std::string(writer.label));
            info.set_substituted(writer.substitute);
            info.set_prim(primOfSignal(spec.name));
            for (const std::string& name : componentNames(spec)) {
                info.add_component_names(name);
            }
        }
        return envelope;
    }

    /// The collision shapes (scene.proto Collider), for the viewport's Collision shading: one
    /// mesh per collider after the scene's meshes, placed in its body's prim (planes left out:
    /// the ground is drawn already).
    void colliders(v1::SceneSnapshot& snap) const {
        const World& world = robot.world;
        const World::Colliders& c = world.colliders;
        for (std::size_t k = 0; k < c.body.size(); ++k) {
            const TriMesh mesh = colliderMesh(
                c.shape[k], c.size[k],
                c.shape[k] == ShapeType::ConvexHull ? &world.hulls.at(c.hull[k]) : nullptr,
                c.shape[k] == ShapeType::HeightField ? &world.heightfields.at(c.hull[k]) : nullptr);
            if (mesh.indices.empty()) {
                continue;
            }
            std::uint32_t prim = kNoIndex;
            Pose local = c.poseInBody[k];
            if (c.body[k] != kWorldBody) {
                const std::size_t b = c.body[k].index;
                if (b >= robot.bodyPrim.size()) {
                    continue;
                }
                prim = robot.bodyPrim[b];
                // In the prim's frame: undo the prim's offset from its body.
                const Pose& offset = robot.scene.prims[prim].offset;
                const std::array<double, 4> back = conjugate(offset.orientation);
                const std::array<double, 3> inPrim =
                    rotate(back, {local.position[0] - offset.position[0],
                                  local.position[1] - offset.position[1],
                                  local.position[2] - offset.position[2]});
                local = {inPrim, multiply(back, local.orientation)};
            }
            v1::Mesh& out = *snap.add_meshes();
            out.set_id(static_cast<std::uint32_t>(snap.meshes_size() - 1));
            out.set_vertex_count(static_cast<std::uint32_t>(mesh.positions.size() / 3));
            out.set_index_count(static_cast<std::uint32_t>(mesh.indices.size()));
            setInline(*out.mutable_positions(), mesh.positions);
            setInline(*out.mutable_normals(), mesh.normals);
            setInline(*out.mutable_indices(), mesh.indices);
            v1::Collider& collider = *snap.add_colliders();
            collider.set_prim(prim);
            collider.set_mesh(out.id());
            for (const double v : local.position) {
                collider.add_position(v);
            }
            for (const double v : local.orientation) {
                collider.add_rotation(static_cast<float>(v));
            }
        }
    }

    /// The electrical nets for the Electrical workspace (scene.proto ElectricalNet), from the
    /// world's schemas: a net per ElectricalBusAPI prim; its terminals the battery ports,
    /// motors, loads and converter sides connected to it, each with the current the network reports
    /// for it; its limits the bus trips and the ports' current ratings.
    void electricalNets(v1::SceneSnapshot& snap) const {
        const World& world = robot.world;
        const WorldView view(world);
        const auto name = [&view](PrimId prim) {
            return std::string(view.signalInstanceName(prim));
        };
        const auto volt = [](v1::Unit& unit) { setUnit(unit, units::V); };
        const auto amp = [](v1::Unit& unit) { setUnit(unit, units::A); };
        std::uint32_t id = 0;
        for (const SchemaInstance& bus : world.schemaInstances) {
            if (bus.schema != "ElectricalBusAPI_1") {
                continue;
            }
            const std::string busName = name(bus.prim);
            v1::ElectricalNet& net = *snap.add_nets();
            net.set_id(id++);
            net.set_name(busName);
            net.set_bus_prim(scenePrimOf(bus.prim));
            net.set_voltage_signal("bus/" + busName + "/voltage");
            const auto connected = [&bus](std::span<const PrimId> targets) {
                return std::ranges::find(targets, bus.prim) != targets.end();
            };
            for (const SchemaInstance& s : world.schemaInstances) {
                const std::string component = name(s.prim);
                if (s.schema == "PortAPI_1" &&
                    connected(s.rel("elec:port:" + s.instance + ":connectedTo"))) {
                    // A battery's port: its current out of the pack.
                    const bool battery = std::ranges::any_of(
                        world.schemaInstances, [&s](const SchemaInstance& other) {
                            return other.prim == s.prim && other.schema == "BatteryAPI_1";
                        });
                    v1::NetTerminal& terminal = *net.add_terminals();
                    terminal.set_prim(scenePrimOf(s.prim));
                    terminal.set_port(s.instance);
                    if (battery) {
                        terminal.set_current_signal("battery/" + component + "/current");
                    }
                    if (const auto rating = s.real("elec:port:" + s.instance + ":maxCurrent");
                        rating && battery) {
                        v1::NetLimit& limit = *net.add_limits();
                        limit.set_label(component + " current");
                        limit.set_value(*rating);
                        amp(*limit.mutable_unit());
                        limit.set_upper(true);
                        limit.set_signal(terminal.current_signal());
                    }
                } else if ((s.schema == "ElectricalMotorAPI_1" ||
                            s.schema == "ElectricalLoadAPI_1") &&
                           connected(s.rel("elec:powerPort"))) {
                    const bool motor = s.schema == "ElectricalMotorAPI_1";
                    v1::NetTerminal& terminal = *net.add_terminals();
                    terminal.set_prim(scenePrimOf(s.prim));
                    terminal.set_port("power");
                    terminal.set_current_signal(motor ? "motor/" + component + "/bus_current"
                                                      : "load/" + component + "/current");
                } else if (s.schema == "ElectricalConverterAPI_1") {
                    // A DC-DC converter is a terminal on both its nets: a sink on its input
                    // net and a source (port named *_out, as a battery's) on its output net.
                    for (const auto& [side, port] :
                         {std::pair{"input", "dc_in"}, std::pair{"output", "dc_out"}}) {
                        if (connected(s.rel(std::string("elec:conv:") + side))) {
                            v1::NetTerminal& terminal = *net.add_terminals();
                            terminal.set_prim(scenePrimOf(s.prim));
                            terminal.set_port(port);
                            terminal.set_current_signal("converter/" + component + "/" + side +
                                                        "_current");
                        }
                    }
                }
            }
            for (const auto& [attribute, label, upper] :
                 {std::tuple{"elec:bus:overvoltageTrip", "OV trip", true},
                  std::tuple{"elec:bus:undervoltageTrip", "UV trip", false}}) {
                if (const auto value = bus.real(attribute); value && *value > 0.0) {
                    v1::NetLimit& limit = *net.add_limits();
                    limit.set_label(label);
                    limit.set_value(*value);
                    volt(*limit.mutable_unit());
                    limit.set_upper(upper);
                    limit.set_signal(net.voltage_signal());
                }
            }
        }
    }

    /// The scene prim a signal is about: body/<link>/..., joint/<joint>/..., sensor/<sensor>/...
    /// and the electrical and thermal components' motor/, battery/, load/, bus/, converter/ and
    /// thermal/ <prim>/... name it.
    std::uint32_t primOfSignal(const std::string& name) const {
        if (replay) {
            for (const recording::Recording::CatalogEntry& entry : replay->catalog) {
                if (entry.name == name) {
                    return entry.prim;
                }
            }
            return kNoIndex;
        }
        const auto field = [&](std::string_view prefix) -> std::string {
            if (name.rfind(prefix, 0) != 0) {
                return {};
            }
            const std::size_t end = name.find('/', prefix.size());
            return name.substr(prefix.size(), end - prefix.size());
        };
        const World& world = robot.world;
        if (const std::string link = field("body/"); !link.empty()) {
            for (std::size_t b = 0; b < world.bodies.names.size(); ++b) {
                if (world.bodies.names[b] == link) {
                    return robot.bodyPrim[b];
                }
            }
        }
        if (const std::string sensor = field("sensor/"); !sensor.empty()) {
            for (std::size_t s = 0; s < world.sensors.names.size(); ++s) {
                if (world.sensors.names[s] == sensor) {
                    return robot.sensorPrim[s];
                }
            }
        }
        if (const std::string joint = field("joint/"); !joint.empty()) {
            for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
                if (world.joints.names[j] == joint) {
                    return robot.jointPrim[j];
                }
            }
        }
        for (const char* domain :
             {"motor/", "battery/", "load/", "bus/", "converter/", "thermal/"}) {
            if (const std::string component = field(domain); !component.empty()) {
                const WorldView view(world);
                for (std::uint32_t p = 0; p < world.prims.paths.size(); ++p) {
                    if (view.signalInstanceName(PrimId{p}) == component) {
                        return scenePrimOf(PrimId{p});
                    }
                }
            }
        }
        return kNoIndex;
    }

    /// The runs of one subscription from what was captured since the last emit.
    void runsFor(Subscription& sub, v1::SignalBlock& block) {
        Captured& c = captured[sub.signal];
        if (c.count == 0 || c.components == 0) {
            return;
        }
        const std::size_t width = c.components;
        const auto value = [&](std::size_t k, std::size_t component) {
            return c.values[k * width + component];
        };
        v1::SignalRun* run = nullptr;
        std::string samples;
        std::uint64_t runCount = 0;
        const auto finish = [&] {
            if (run != nullptr) {
                run->set_sample_count(static_cast<std::uint32_t>(runCount));
                run->set_samples(samples);
            }
            run = nullptr;
            samples.clear();
            runCount = 0;
        };
        const auto start = [&](std::uint64_t firstIndex, std::int64_t dt, std::int64_t t0) {
            run = block.add_runs();
            run->set_signal(sub.signal);
            run->mutable_t0()->set_ns(t0);
            run->set_dt_ns(dt);
            run->set_components(static_cast<std::uint32_t>(width));
            run->set_encoding(v1::SAMPLE_ENCODING_F64_LE);
            run->set_first_index(firstIndex);
            run->set_mode(sub.mode);
        };
        const auto put = [&](double v) { samples.append(reinterpret_cast<const char*>(&v), 8); };

        const std::uint32_t n = std::max<std::uint32_t>(sub.every, 1);
        if (sub.mode == v1::SignalSubscription::MODE_MINMAX && n > 1) {
            std::int64_t lastBucket = -1;
            for (std::size_t k = 0; k < c.count; ++k) {
                const auto index = static_cast<std::int64_t>(c.firstIndex + k);
                const std::int64_t bucket = index / n;
                if (bucket != sub.bucket) {
                    sub.bucket = bucket;  // an incomplete bucket is dropped, never sent half
                    sub.inBucket = 0;
                    sub.low.assign(width, 0.0);
                    sub.high.assign(width, 0.0);
                }
                for (std::size_t i = 0; i < width; ++i) {
                    const double v = value(k, i);
                    sub.low[i] = sub.inBucket == 0 ? v : std::min(sub.low[i], v);
                    sub.high[i] = sub.inBucket == 0 ? v : std::max(sub.high[i], v);
                }
                if (++sub.inBucket == n) {
                    if (run == nullptr || bucket != lastBucket + 1) {
                        finish();
                        start(static_cast<std::uint64_t>(bucket), c.dtNs * n, bucket * n * c.dtNs);
                    }
                    for (const double v : sub.low) {
                        put(v);
                    }
                    for (const double v : sub.high) {
                        put(v);
                    }
                    ++runCount;
                    lastBucket = bucket;
                    sub.bucket = -1;
                }
            }
        } else if (sub.mode == v1::SignalSubscription::MODE_STRIDE && n > 1) {
            for (std::size_t k = 0; k < c.count; ++k) {
                const std::uint64_t index = c.firstIndex + k;
                if (index % n != 0) {
                    continue;
                }
                if (run == nullptr) {
                    start(index / n, c.dtNs * n, static_cast<std::int64_t>(index) * c.dtNs);
                }
                for (std::size_t i = 0; i < width; ++i) {
                    put(value(k, i));
                }
                ++runCount;
            }
        } else {
            start(c.firstIndex, c.dtNs, static_cast<std::int64_t>(c.firstIndex) * c.dtNs);
            for (const double v : c.values) {
                put(v);
            }
            runCount = c.count;
        }
        finish();
    }

    // --- messages ----------------------------------------------------------------------------

    v1::Envelope statusEnvelope() const {
        v1::Envelope envelope;
        v1::ServerStatus& s = *envelope.mutable_status();
        s.mutable_sim_time()->set_ns(scheduler->now().ns);
        s.set_state(state);
        s.set_real_time_factor_target(rtfTarget);
        s.set_real_time_factor(state == v1::RUN_STATE_RUNNING ? measuredRtf : 0.0);
        s.set_error(error);
        if (replay) {
            s.mutable_replay_begin()->set_ns(0);
            s.mutable_replay_end()->set_ns(replay->duration.ns);
        }
        return envelope;
    }
    double measuredRtf{0.0};

    void log(v1::LogLevel level, const std::string& source, const std::string& text) {
        v1::LogRecord record;
        record.mutable_sim_time()->set_ns(scheduler ? scheduler->now().ns : 0);
        record.set_wall_unix_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count());
        record.set_level(level);
        record.set_source(source);
        record.set_text(text);
        std::fprintf(stderr, "[%s] %s\n", source.c_str(), text.c_str());
        logs.push_back(record);
        if (logs.size() > kMaxLogRecords) {
            logs.pop_front();
        }
        v1::Envelope envelope;
        *envelope.mutable_log() = record;
        broadcast(envelope);
    }

    void broadcast(const v1::Envelope& envelope) {
        for (auto& session : sessions) {
            if (session->greeted) {
                session->send(envelope);
            }
        }
    }

    bool hasSensors() const { return !robot.world.sensors.names.empty(); }
    /// Motors to simulate: ElectricalMotorAPI_1 prims from USD or an overlay.
    /// Thermal nodes: ThermalAPI_1 prims.
    bool hasThermalNodes() const {
        return !WorldView(robot.world).instancesOf("ThermalAPI_1").empty();
    }
    bool hasRotors() const { return !WorldView(robot.world).instancesOf("RotorAPI_1").empty(); }
    bool hasMotors() const {
        return !WorldView(robot.world).instancesOf("ElectricalMotorAPI_1").empty();
    }

    /// Picks where lidars cast (ServerOptions::lidar); logs the choice.
    Status placeLidar() {
        const auto hasLidar = std::ranges::any_of(
            robot.world.sensors.type, [](SensorType t) { return t == SensorType::Lidar; });
        using Lidar = ServerOptions::Lidar;
        // Auto stays on the CPU: the exact casts, culled by Embree and spread over the cores,
        // outrun a GPU round trip for scans of tens of thousands of beams (docs/design/
        // benchmarks.md) and keep the run deterministic. --lidar gpu takes the compute kernel.
        if (!hasLidar || options.lidar != Lidar::Gpu) {
            if (hasLidar) {
                log(v1::LOG_LEVEL_INFO, "sensors",
                    "lidar on the CPU (Tier 0: exact casts, Embree culling, all cores)");
            }
            return {};
        }
        if (!robot.world.heightfields.empty() || !robot.tiles.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 "--lidar gpu does not see height fields (terrain) yet: drop "
                                 "it, the CPU lidar sees them exactly");
        }
        std::string reason;
        auto made = gpu::RayContext::create(options.gpu, reason);
        if (made && made->tier() < 1) {
            reason = made->deviceName() + " is a software device (Tier 0)";
            made.reset();
        }
        if (!made) {
            return Status::error(StatusCode::InvalidConfig,
                                 "--lidar gpu needs a Tier 1 GPU: " + reason);
        }
        rayContext = std::move(made);
        log(v1::LOG_LEVEL_INFO, "sensors",
            "lidar on the GPU: " + rayContext->deviceName() + " (Tier " +
                std::to_string(rayContext->tier()) +
                ", compute BVH, bvh_traverse.slang); the run is not deterministic");
        return {};
    }
    std::shared_ptr<gpu::RayContext> rayContext;  // null: lidars on the CPU

    /// Uploads the scene for the cameras, if the robot has any; logs the device.
    void placeCameras() {
        const auto count = std::ranges::count(robot.world.sensors.type, SensorType::Camera);
        if (count == 0) {
            return;
        }
        if (!options.cameras) {
            log(v1::LOG_LEVEL_INFO, "sensors",
                "cameras publish ground truth (empty images): rendering is off for this run");
            return;
        }
        std::string reason;
        cameraScene = sensors::createCameraScene(robot, options.gpu, reason);
        if (!cameraScene) {
            log(v1::LOG_LEVEL_WARN, "sensors",
                "cameras need a Vulkan device and none is usable (" + reason +
                    "); set the sensors domain to ground_truth to run without them");
            return;
        }
        const gpu::RasterScene& raster = *cameraScene->raster;
        log(v1::LOG_LEVEL_INFO, "sensors",
            std::to_string(count) + (count == 1 ? " camera" : " cameras") + " rendered on " +
                raster.deviceName() + " (" + std::to_string(raster.samples()) +
                " samples per pixel" + (raster.isSoftware() ? "; a software device, slow" : "") +
                "); the run is not deterministic");
    }
    std::shared_ptr<const sensors::CameraScene> cameraScene;  // null: no camera rendering

    std::int64_t fastestSensorNs() const {
        const World::Sensors& sensors = robot.world.sensors;
        std::int64_t fastest = 0;
        for (std::size_t s = 0; s < sensors.rateHz.size(); ++s) {
            if (sensors.rateHz[s] <= 0.0) {
                continue;
            }
            // Cameras step on whole milliseconds (the sensors module rounds their period).
            const std::int64_t period =
                sensors.type[s] == SensorType::Camera
                    ? std::max<std::int64_t>(std::llround(1e3 / sensors.rateHz[s]), 1) * 1'000'000
                    : std::llround(1e9 / sensors.rateHz[s]);
            fastest = fastest == 0 ? period : std::min(fastest, period);
        }
        return fastest;
    }

    /// What the sensors domain runs: the lidar's casting, else the cameras' raster.
    std::string sensorLevel() const {
        const auto has = [this](SensorType type) {
            return std::ranges::count(robot.world.sensors.type, type) != 0;
        };
        if (has(SensorType::Lidar) || !has(SensorType::Camera)) {
            return rayContext ? "raycast_gpu" : "raycast";
        }
        return "raster";
    }

    [[nodiscard]] bool substituted(std::string_view domain) const {
        return options.substituted.contains(std::string(domain));
    }

    /// The rigid-body backend the run uses: an applied profile's, else the command line's.
    std::string physicsBackend() const { return rigidBodyBackend.value_or(options.physics); }

    /// The stage as profiles name it (ServerOptions::stageName).
    std::string stageName() const {
        if (!options.stageName.empty()) {
            return options.stageName;
        }
        if (!robot.sourcePath.empty()) {
            return robot.sourcePath;
        }
        return options.worldPath.empty() ? "none" : options.worldPath;  // a run without a robot
    }

    std::string profileYaml() const {
        if (replay) {
            // A replay: every recorded domain is its replay substitute, the rest their default.
            std::string yaml =
                "# Lagrange simulation profile: a replay\nschema_version: 1\n"
                "stage: " +
                stageName() + "\n\ndomains:\n";
            for (const DomainRow& row : kDomains) {
                const bool recorded = std::ranges::any_of(
                    replay->catalog, [&row](const recording::Recording::CatalogEntry& entry) {
                        return entry.writer.starts_with(std::string(row.domain) + "/");
                    });
                yaml +=
                    "  " + std::string(row.domain) + ": {enabled: false, substitute: " +
                    (recorded ? "replay, source: " + replay->path : std::string(row.substitute)) +
                    "}\n";
            }
            return yaml;
        }
        std::string yaml =
            "# Lagrange simulation profile\nschema_version: 1\nstage: " + stageName() +
            "\n\ndomains:\n";
        for (const DomainRow& row : kDomains) {
            if (std::string_view(row.domain) == "rigid_body" && !substituted("rigid_body")) {
                char dt[48];
                std::snprintf(dt, sizeof(dt), "%.9g",
                              static_cast<double>(rigidBodyDtNs.value_or(1'000'000)) / 1e9);
                yaml += "  rigid_body: {enabled: true, backend: " + physicsBackend() +
                        ", dt: " + std::string(dt) + "}\n";
            } else if (std::string_view(row.domain) == "rigid_body" && rigidBodyReplay) {
                yaml += "  rigid_body: {enabled: false, substitute: replay, source: " +
                        *rigidBodyReplay + "}\n";
            } else if (std::string_view(row.domain) == "electrical" && hasMotors() &&
                       !substituted("electrical")) {
                char dt[48];
                std::snprintf(dt, sizeof(dt), "%.9g",
                              static_cast<double>(electricalDtNs.value_or(100'000)) / 1e9);
                yaml += "  electrical: {enabled: true, fidelity: " +
                        electricalFidelity.value_or("averaged") + ", dt: " + std::string(dt) +
                        "}\n";
            } else if (std::string_view(row.domain) == "thermal" && hasThermalNodes() &&
                       ambient()) {
                char text[96];
                if (substituted("thermal")) {
                    std::snprintf(text, sizeof(text),
                                  "{enabled: false, substitute: ambient, ambient_temperature: "
                                  "%.9g}",
                                  *ambient());
                } else {
                    std::snprintf(
                        text, sizeof(text), "{enabled: true, dt: %.9g, ambient_temperature: %.9g}",
                        static_cast<double>(thermalDtNs.value_or(100'000'000)) / 1e9, *ambient());
                }
                yaml += "  thermal: " + std::string(text) + "\n";
            } else if (std::string_view(row.domain) == "sensors" && hasSensors() &&
                       !substituted("sensors")) {
                yaml += "  sensors: {enabled: true}\n";
            } else if (std::string_view(row.domain) == "rotor_aero" && hasRotors()) {
                const std::optional<double> rho =
                    appliedAirDensity ? appliedAirDensity : options.airDensityKgM3;
                char text[160];
                std::snprintf(text, sizeof(text), "{enabled: %s, %s%s%.9g}",
                              substituted("rotor_aero") ? "false" : "true",
                              substituted("rotor_aero") ? "substitute: ideal, " : "",
                              rho ? "air_density: " : "dt: ", rho ? *rho : 0.001);
                yaml += "  rotor_aero: " + std::string(text) + "\n";
            } else if (std::string_view(row.domain) == "weather" && weatherRuns()) {
                // What the run uses: the command line's weather, then the applied profile's.
                std::map<std::string, ConfigValue, std::less<>> values;
                for (const auto& [key, value] : options.weather) {
                    values[key] = value;
                }
                for (const auto& [key, value] : appliedWeather) {
                    values[key] = value;
                }
                std::string text = substituted("weather") ? "{enabled: false, substitute: clear"
                                                          : "{enabled: true";
                for (const auto& [key, value] : values) {
                    if (const double* number = std::get_if<double>(&value)) {
                        char item[96];
                        std::snprintf(item, sizeof(item), ", %s: %.9g", key.c_str(), *number);
                        text += item;
                    }
                }
                yaml += "  weather: " + text + "}\n";
            } else if (std::string_view(row.domain) == "emi" && hasEmi() && !substituted("emi")) {
                std::string text = "{enabled: true";
                if (appliedEmiCoupling) {
                    text += ", coupling: [";
                    for (std::size_t i = 0; i < appliedEmiCoupling->size(); ++i) {
                        text += (i == 0 ? "" : ", ") + (*appliedEmiCoupling)[i];
                    }
                    text += "]";
                }
                yaml += "  emi: " + text + "}\n";
            } else if (std::string_view(row.domain) == "vehicle_dynamics" && hasVehicles()) {
                yaml += substituted("vehicle_dynamics")
                            ? "  vehicle_dynamics: {enabled: false, substitute: ideal}\n"
                            : "  vehicle_dynamics: {enabled: true, tire_model: " +
                                  appliedTireModel.value_or("pacejka") + "}\n";
            } else if (std::string_view(row.domain) == "traffic" && hasTraffic()) {
                yaml += substituted("traffic") ? "  traffic: {enabled: false, substitute: zero}\n"
                                               : "  traffic: {enabled: true, backend: builtin}\n";
            } else if (std::string_view(row.domain) == "comms" && hasComms()) {
                yaml += substituted("comms") ? "  comms: {enabled: false, substitute: ideal}\n"
                                             : "  comms: {enabled: true}\n";
            } else if (std::string_view(row.domain) == "comms" ||
                       std::string_view(row.domain) == "traffic" ||
                       std::string_view(row.domain) == "vehicle_dynamics" ||
                       std::string_view(row.domain) == "rotor_aero" ||
                       std::string_view(row.domain) == "hydrodynamics") {
                continue;  // not in the profile of a ground robot
            } else {
                yaml += "  " + std::string(row.domain) +
                        ": {enabled: false, substitute: " + row.substitute + "}\n";
            }
        }
        yaml += "\nviews: [physics_3d, waveforms]\n";
        return yaml;
    }

    v1::Envelope profileState(const std::vector<ModuleCost>& costs) const {
        v1::Envelope envelope;
        v1::ProfileState& p = *envelope.mutable_profile_state();
        p.set_yaml(profileYaml());
        p.set_schema_version(1);
        p.set_json_schema(schemaText);
        p.set_restart_required(pending.has_value());  // applied, waiting for the next reset
        // Measured cost per simulated second, summed over each domain's modules.
        std::map<std::string, double> cost;
        for (const ModuleCost& c : costs) {
            if (c.steps > 0) {
                cost[c.domain] += static_cast<double>(c.totalNs) / 1e6 /
                                  (static_cast<double>(c.steps) * c.dt.seconds());
            }
        }
        for (const DomainRow& row : kDomains) {
            v1::DomainState& d = *p.add_domains();
            d.set_domain(row.domain);
            if (replay) {
                // A replay simulates nothing: what was recorded stands in for every domain.
                const bool recorded = std::ranges::any_of(
                    replay->catalog, [&row](const recording::Recording::CatalogEntry& entry) {
                        return entry.writer.starts_with(std::string(row.domain) + "/");
                    });
                d.set_enabled(false);
                d.set_available(false);
                d.set_substituted(true);
                d.set_active_module(recorded ? "replay" : row.substitute);
                v1::SubstituteOption& option = *d.add_substitutes();
                option.set_token(recorded ? "replay" : row.substitute);
                option.set_kind(recorded ? "replay" : row.substituteKind);
                d.set_unavailable_reason("a replay of " + replay->path + ": nothing is simulated");
                continue;
            }
            const bool physics = std::string_view(row.domain) == "rigid_body";
            const bool sensing = std::string_view(row.domain) == "sensors" && hasSensors();
            const bool powered = std::string_view(row.domain) == "electrical" && hasMotors();
            const bool heated =
                std::string_view(row.domain) == "thermal" && hasThermalNodes() && ambient();
            const bool built = physics || sensing || powered || heated;  // a module exists
            const bool running = built && !substituted(row.domain);
            d.set_enabled(running);
            d.set_available(built);
            d.set_active_module(running ? (physics   ? physicsBackend()
                                           : powered ? "averaged_network"
                                           : heated  ? "lumped"
                                                     : sensorLevel())
                                        : row.substitute);
            d.set_substituted(!running);
            d.set_cpu_ms_per_sim_s(cost[row.domain]);
            if (sensing) {
                d.add_fidelities(sensorLevel());
                if (sensorLevel() != "raster" &&
                    std::ranges::count(robot.world.sensors.type, SensorType::Camera) != 0) {
                    d.add_fidelities("raster");
                }
                d.set_dt_ns(fastestSensorNs());
                d.add_depends_on("rigid_body");
            }
            if (powered) {
                d.add_fidelities("averaged");
                if (!electrical::motorFmuBindings(WorldView(robot.world)).empty()) {
                    d.add_fidelities("fmu");
                }
                d.set_dt_ns(electricalDtNs.value_or(100'000));
                d.set_min_dt_ns(10'000);
                d.set_max_dt_ns(1'000'000);
                d.add_depends_on("rigid_body");
            }
            if (heated) {
                d.add_fidelities("lumped");
                d.set_dt_ns(thermalDtNs.value_or(100'000'000));
                d.set_min_dt_ns(1'000'000);
                d.set_max_dt_ns(10'000'000'000);
                d.add_depends_on("electrical");
            }
            if (!built) {
                const std::string_view domain = row.domain;
                d.set_unavailable_reason(
                    domain == "electrical"
                        ? "no ElectricalMotorAPI prims in the stage; the ideal torque source "
                          "stands in (an overlay file adds motors to a URDF robot)"
                    : domain == "thermal" && hasThermalNodes()
                        ? "needs the site's ambient temperature: thermal.ambient_temperature in "
                          "the profile, or --ambient-temperature K"
                    : domain == "thermal"
                        ? "no ThermalAPI prims in the stage"
                        : "no module in this build yet; the substitute stands in");
            }
            v1::SubstituteOption& option = *d.add_substitutes();
            option.set_token(row.substitute);
            option.set_kind(row.substituteKind);
            if (physics) {
                v1::SubstituteOption& replayOption = *d.add_substitutes();
                replayOption.set_token("replay");
                replayOption.set_kind("replay");
            }
            if (physics) {
                d.add_backends("mujoco");
                d.add_backends("jolt");
                d.add_fidelities("articulated");
                d.set_dt_ns(1'000'000);
                d.set_min_dt_ns(100'000);
                d.set_max_dt_ns(10'000'000);
            }
        }
        return envelope;
    }

    // --- sessions ------------------------------------------------------------------------------

    bool handshake(Session& session, const v1::Envelope& envelope) {
        v1::Envelope reply;
        v1::ServerHello& hello = *reply.mutable_server_hello();
        hello.mutable_version()->set_major(kProtocolMajor);
        hello.mutable_version()->set_minor(kProtocolMinor);
        hello.set_server_kind(v1::SERVER_KIND_SIM);
        hello.set_server_name(kServerName);
        if (!envelope.has_client_hello()) {
            hello.set_refusal("the first message must be ClientHello");
            session.send(reply);
            return false;
        }
        const v1::ClientHello& client = envelope.client_hello();
        if (client.version().major() != kProtocolMajor) {
            hello.set_refusal("protocol version mismatch: the client speaks " +
                              std::to_string(client.version().major()) + "." +
                              std::to_string(client.version().minor()) + ", this server speaks " +
                              std::to_string(kProtocolMajor) + "." +
                              std::to_string(kProtocolMinor));
            session.send(reply);
            return false;
        }
        if (!options.authToken.empty() && session.connection.kind() == proto::Endpoint::Kind::Tcp &&
            !proto::sameSecret(client.auth_token(), options.authToken)) {
            hello.set_refusal(
                "authentication failed: this server needs its token "
                "(the viewer's --auth-token-file or LAGRANGE_AUTH_TOKEN)");
            session.send(reply);
            return false;
        }
        hello.set_session_id(nextSessionId++);
        v1::Capabilities& caps = *hello.mutable_capabilities();
        caps.set_can_step(true);
        caps.set_can_seek(replay.has_value());
        caps.set_can_import(!replay.has_value());  // ImportRobot spawns into the run
        caps.set_tier(0);
        session.send(reply);
        session.greeted = true;
        if (replay) {
            session.send(recordingInfo());
        }

        session.send(profileState({}));
        session.send(catalog());
        session.send(statusEnvelope());
        session.send(environmentState());
        for (const v1::LogRecord& record : logs) {
            v1::Envelope line;
            *line.mutable_log() = record;
            session.send(line);
        }
        return true;
    }

    void dispatch(Session& session, const v1::Envelope& envelope) {
        if (envelope.has_ping()) {
            v1::Envelope reply;
            reply.mutable_pong()->set_nonce(envelope.ping().nonce());
            reply.mutable_pong()->set_client_monotonic_ns(envelope.ping().client_monotonic_ns());
            reply.mutable_pong()->mutable_sim_time()->set_ns(scheduler->now().ns);
            session.send(reply);
        } else if (envelope.has_subscribe()) {
            if (envelope.subscribe().catalog_generation() != catalogGeneration) {
                return;  // stale: the viewer re-subscribes after the new catalog
            }
            for (const v1::SignalSubscription& item : envelope.subscribe().signals()) {
                if (item.signal() < captured.size()) {
                    Subscription sub;
                    sub.signal = item.signal();
                    sub.mode = item.mode() == v1::SignalSubscription::MODE_UNSPECIFIED
                                   ? v1::SignalSubscription::MODE_FULL
                                   : item.mode();
                    sub.every = std::max<std::uint32_t>(item.decimation(), 1);
                    session.subscriptions[item.signal()] = sub;
                }
            }
        } else if (envelope.has_unsubscribe()) {
            for (const std::uint32_t signal : envelope.unsubscribe().signals()) {
                session.subscriptions.erase(signal);
            }
        } else if (envelope.has_command()) {
            command(session, envelope.command());
        }
        // Anything else is ignored: newer viewers may send messages this server does not know.
    }

    void command(Session& session, const v1::Command& command) {
        v1::Envelope result;
        v1::CommandResult& r = *result.mutable_command_result();
        r.set_request_id(command.request_id());
        r.set_ok(true);
        bool statusChanged = false;
        bool resync = false;
        switch (command.command_case()) {
            case v1::Command::kPlay:
                if (state != v1::RUN_STATE_ERROR) {
                    state = v1::RUN_STATE_RUNNING;
                    stopped = false;
                    lastWall = Clock::now();
                }
                statusChanged = true;
                break;
            case v1::Command::kPause:
                if (state != v1::RUN_STATE_ERROR) {
                    state = v1::RUN_STATE_PAUSED;
                }
                statusChanged = true;
                break;
            case v1::Command::kSetRtf:
                if (!(command.set_rtf().target() >= 0.0) ||
                    !std::isfinite(command.set_rtf().target())) {
                    r.set_ok(false);
                    r.set_error("real-time factor must be a non-negative number");
                } else {
                    rtfTarget = command.set_rtf().target();
                }
                statusChanged = true;
                break;
            case v1::Command::kStep: {
                std::int64_t ns = command.step().duration_ns();
                if (ns <= 0) {
                    ns = static_cast<std::int64_t>(
                             std::max<std::uint32_t>(command.step().ticks(), 1)) *
                         kStepCommandDefaultNs;
                }
                if (state == v1::RUN_STATE_RUNNING) {
                    state = v1::RUN_STATE_PAUSED;
                }
                if (state != v1::RUN_STATE_ERROR) {
                    const SimTime target = scheduler->now() + Duration{ns};
                    while (scheduler->now() < target) {
                        if (Status status = scheduler->stepTick(); !status.ok()) {
                            fail(status);
                            break;
                        }
                        capture();
                        record();
                        if (Status status = worldUpdate(false); !status.ok()) {
                            fail(status);
                            break;
                        }
                    }
                    emit();
                }
                statusChanged = true;
                break;
            }
            case v1::Command::kReset: {
                if (Status status = pending ? adopt(*pending) : restart(); !status.ok()) {
                    r.set_ok(false);
                    r.set_error(status.message);
                }
                resync = true;
                statusChanged = true;
                break;
            }
            case v1::Command::kRequestSnapshot:
                session.send(snapshot(session));
                break;
            case v1::Command::kGetProfile:
                session.send(profileState(scheduler->takeCosts()));
                break;
            case v1::Command::kGetAttributes:
                attributes(command.get_attributes(), *r.mutable_attributes());
                break;
            case v1::Command::kSetAttribute:
                if (Status status = setAttribute(command.set_attribute()); !status.ok()) {
                    r.set_ok(false);
                    r.set_error(status.message);
                }
                break;
            case v1::Command::kOpenRecording:
                if (Status status = openRecording(command.open_recording().path()); !status.ok()) {
                    r.set_ok(false);
                    r.set_error(status.message);
                } else {
                    r.set_imported_stage_path(command.open_recording().path());
                }
                break;
            case v1::Command::kSeek:
                if (Status status = seek(command.seek().to().ns()); !status.ok()) {
                    r.set_ok(false);
                    r.set_error(status.message);
                } else {
                    resync = true;
                }
                statusChanged = true;
                break;
            case v1::Command::kSetTransform:
                r.set_ok(false);
                r.set_error("moving prims is not supported by this server yet");
                break;
            case v1::Command::kImportRobot: {
                const v1::ImportRobot& request = command.import_robot();
                if (request.path().empty()) {
                    r.set_ok(false);
                    r.set_error("name the robot file by its path on the server's machine");
                    break;
                }
                if (!request.overlay_path().empty() || !request.xacro_args().empty()) {
                    r.set_ok(false);
                    r.set_error("overlays and xacro arguments are not taken when importing yet");
                    break;
                }
                const ros2::SpawnOutcome outcome =
                    spawn(request.path(), "", true, importPose(), false);
                if (outcome.outcome.result != ros2::SimResult::Ok) {
                    r.set_ok(false);
                    r.set_error(outcome.outcome.message);
                } else {
                    r.set_imported_stage_path(request.path());
                }
                break;
            }
            case v1::Command::kSetRosGraph:
                session.wantsRosGraph = command.set_ros_graph().enabled();
                if (session.wantsRosGraph) {
                    session.send(rosGraph());
                }
                break;
            case v1::Command::kSetCameraInterest: {
                const v1::SetCameraInterest& hint = command.set_camera_interest();
                if (hint.enabled()) {
                    const std::array<double, 3>& region = robot.world.region.worldEnu;
                    cameraInterest = std::array<double, 3>{
                        hint.x() + region[0], hint.y() + region[1], hint.z() + region[2]};
                } else {
                    cameraInterest.reset();
                }
                break;
            }
            case v1::Command::kSetWorld:
                session.wantsWorld = command.set_world().enabled();
                if (session.wantsWorld) {
                    session.send(worldState());
                }
                break;
            case v1::Command::kSubscribeSensor:
                if (command.subscribe_sensor().enabled()) {
                    session.sensors.insert(command.subscribe_sensor().sensor());
                } else {
                    session.sensors.erase(command.subscribe_sensor().sensor());
                }
                break;
            case v1::Command::kSubscribeField: {
                const v1::SubscribeField& f = command.subscribe_field();
                if (f.enabled()) {
                    session.fields[f.field()] = f.height_m();
                    if (auto slice = fieldSlice(f.field(), f.height_m())) {
                        session.send(*slice);
                    }
                } else {
                    session.fields.erase(f.field());
                }
                break;
            }
            case v1::Command::kSetOverlays:
                break;  // accepted; overlays are drawn by the viewer
            case v1::Command::kValidateProfile:
                validate(command.validate_profile().yaml(), *r.mutable_validation());
                break;
            case v1::Command::kApplyProfile: {
                const v1::ApplyProfile& request = command.apply_profile();
                if (Status status = apply(request.yaml(), request.restart()); !status.ok()) {
                    r.set_ok(false);
                    r.set_error(status.message);
                } else {
                    resync = request.restart();
                }
                // Everyone sees the profile that is now in force (or pending).
                for (auto& other : sessions) {
                    if (other->greeted) {
                        other->send(profileState(scheduler->takeCosts()));
                    }
                }
                statusChanged = true;
                break;
            }
            default:
                r.set_ok(false);
                r.set_error(
                    "not supported by lagrange_server v0.2 (seek, import, scene editing "
                    "come later)");
                break;
        }
        session.send(result);
        if (resync) {
            resyncViewers();
        }
        if (statusChanged) {
            broadcast(statusEnvelope());
        }
    }

    /// Rebuilds the run at t = 0, keeping play or pause.
    // --- replay mode (roadmap 0.2-E8) --------------------------------------------------------

    std::optional<recording::Recording> replay;  // the recording's metadata; signals not kept
    std::vector<SignalId> replayPose;            // per scene prim: its recorded pose, if any
    google::protobuf::RepeatedPtrField<v1::ElectricalNet> replayNets;  // from its snapshot
    static constexpr std::int64_t kCheckpointNs = 1'000'000'000;       // a saved state a second
    static constexpr std::int64_t kRebaseCheckNs = 100'000'000;  // floating-origin check, 10 Hz
    std::map<std::int64_t, std::vector<std::byte>> checkpoints;  // by simulated time

    /// Loads the recording: its scene as the model, its domains as replay substitutes.
    Status loadReplay() {
        auto opened = recording::Recording::open(options.replayPath,
                                                 [](const std::string&, const std::string&) {
                                                     return false;  // metadata only
                                                 });
        if (!opened) {
            return Status::error(StatusCode::InvalidConfig, opened.error());
        }
        auto model = modelOfSnapshot(opened->snapshot);
        if (!model) {
            return Status::error(StatusCode::InvalidConfig,
                                 options.replayPath + ": " + model.error());
        }
        v1::SceneSnapshot recorded;
        if (recorded.ParseFromString(opened->snapshot)) {
            replayNets = recorded.nets();
        }
        replay = std::move(*opened);
        robot = std::move(*model);
        emiSourcesRead = false;  // the recording's scene
        log(v1::LOG_LEVEL_INFO, "replay",
            options.replayPath + " · " + std::to_string(replay->catalog.size()) + " signals · " +
                std::to_string(replay->duration.seconds()) + " s · stage " + robot.sourcePath);
        return {};
    }

    v1::Envelope recordingInfo() const {
        v1::Envelope info;
        v1::RecordingInfo& r = *info.mutable_recording_info();
        r.set_path(replay->path);
        r.mutable_duration()->set_ns(replay->duration.ns);
        std::error_code ec;
        r.set_size_bytes(std::filesystem::file_size(replay->path, ec));
        r.set_generator("lagrange_server");
        std::uint64_t total = 0;
        for (const recording::Recording::CatalogEntry& entry : replay->catalog) {
            v1::RecordingInfo::Channel& channel = *r.add_channels();
            channel.set_topic(entry.name);
            if (entry.dtNs > 0) {
                channel.set_rate_hz(1e9 / static_cast<double>(entry.dtNs));
                const auto messages =
                    static_cast<std::uint64_t>(replay->duration.ns / entry.dtNs) + 1;
                channel.set_messages(messages);  // samples
                total += messages;
            }
            channel.set_unit(entry.unit);
        }
        r.set_messages(total);
        return info;
    }

    /// Command OpenRecording: the server becomes a replay of the file (from the viewer's
    /// "Open recording"). Not with the ROS bridge, which drives a robot a replay does not have.
    Status openRecording(const std::string& path) {
        if (bridge) {
            return Status::error(StatusCode::Unsupported,
                                 "this server runs the ROS 2 bridge; replay with `lagrange_server "
                                 "--replay " +
                                     path + "` instead");
        }
        stopRecording("a recording was opened");
        const std::string previous = options.replayPath;
        options.replayPath = path;
        if (Status status = loadReplay(); !status.ok()) {
            options.replayPath = previous;
            return status;
        }
        if (Status status = build(); !status.ok()) {
            fail(status);
            return status;
        }
        state = v1::RUN_STATE_PAUSED;
        error.clear();
        owedNs = 0;
        ++catalogGeneration;
        for (auto& session : sessions) {
            if (!session->greeted) {
                continue;
            }
            session->subscriptions.clear();
            session->sensors.clear();
            session->send(catalog());
            session->send(recordingInfo());
            session->send(snapshot(*session));
            session->send(statusEnvelope());
        }
        return {};
    }

    /// After a build in replay mode: the prims' pose signals and the first checkpoint.
    void startReplay() {
        replayPose.assign(robot.scene.prims.size(), SignalId{});
        for (std::uint32_t p = 0; p < robot.scene.prims.size(); ++p) {
            replayPose[p] = scheduler->bus().find("scene" + robot.scene.prims[p].path + "/pose");
        }
        checkpoints.clear();
        checkpoint();
    }

    void checkpoint() {
        if (!replay || checkpoints.contains(scheduler->now().ns)) {
            return;
        }
        StateWriter writer;
        if (scheduler->saveState(writer).ok()) {
            const std::span<const std::byte> bytes = writer.bytes();
            checkpoints.emplace(scheduler->now().ns,
                                std::vector<std::byte>(bytes.begin(), bytes.end()));
        }
    }

    /// Command Seek: from the latest checkpoint at or before `to`, stepped forward to it.
    Status seek(std::int64_t to) {
        if (!replay) {
            return Status::error(StatusCode::Unsupported,
                                 "seeking needs a replay: start the server with --replay FILE");
        }
        to = std::clamp<std::int64_t>(to, 0, replay->duration.ns);
        auto it = checkpoints.upper_bound(to);
        if (it == checkpoints.begin()) {
            return Status::error(StatusCode::Internal, "no checkpoint to seek from");
        }
        --it;
        StateReader reader(it->second);
        if (Status status = scheduler->restoreState(reader); !status.ok()) {
            return status;
        }
        while (scheduler->now().ns < to) {
            if (Status status = scheduler->stepTick(); !status.ok()) {
                return status;
            }
            if (scheduler->now().ns % kCheckpointNs == 0) {
                checkpoint();
            }
        }
        owedNs = 0;
        captured.assign(captured.size(), Captured{});  // nothing captured before the seek
        for (std::size_t s = 0; s < captured.size(); ++s) {
            const SignalId id = SignalId::output(static_cast<std::uint32_t>(s));
            captured[s].id = id;
            captured[s].components = componentCount(scheduler->bus().specAt(s).type,
                                                    scheduler->bus().specAt(s).arrayLength);
            captured[s].dtNs = scheduler->writerOf(id).dt.ns;
        }
        return {};
    }

    /// Rebuilds the run from the (edited) world and carries its state over: the edit applies
    /// from the next step and the run goes on (E6). On failure the run is left as it was.
    Status rebuildKeepingState() {
        StateWriter saved;
        if (Status status = scheduler->saveState(saved); !status.ok()) {
            return status;
        }
        std::unique_ptr<Scheduler> previous = std::move(scheduler);
        DriveCommand* previousDrive = driveCommand;
        if (Status status = build(); !status.ok()) {
            scheduler = std::move(previous);
            driveCommand = previousDrive;
            if (bridge) {
                bridge->attach(robot.world, scheduler->bus());
            }
            return status;
        }
        StateReader reader(saved.bytes());
        if (Status status = scheduler->restoreState(reader); !status.ok()) {
            scheduler = std::move(previous);
            driveCommand = previousDrive;
            (void)build();  // the previous wiring, at t = 0: the old state cannot be carried
            return status;
        }
        return {};
    }

    static constexpr std::size_t kTileWorkers = 4;
    static constexpr double kDefaultLoadRadiusM = 600.0;       // world.streaming.load_radius
    static constexpr double kDefaultUnloadRadiusM = 800.0;     // world.streaming.unload_radius
    static constexpr double kDefaultCollisionRadiusM = 300.0;  // world.streaming.collision_radius
    // How far ahead a robot's ground must be in physics: this, plus half a second of travel.
    static constexpr double kGroundMarginM = 5.0;

    /// The robot's first body in world ENU.
    [[nodiscard]] std::array<double, 3> focusWorld() const {
        std::array<double, 3> p = robot.bodyRestPose.empty() ? std::array<double, 3>{}
                                                             : robot.bodyRestPose.front().position;
        if (scheduler && !bodyPose.empty() && bodyPose.front().valid()) {
            const std::span<const double> pose = scheduler->bus().readN(bodyPose.front());
            p = {pose[0], pose[1], pose[2]};
        }
        for (std::size_t i = 0; i < 3; ++i) {
            p.at(i) += robot.world.region.worldEnu.at(i);
        }
        return p;
    }

    /// Starts streaming when the loaded stages have tiles: the base extent, the selection, and
    /// the tiles around the robot's start, loaded before the run is built.
    Status startStreaming() {
        if (robot.tiles.empty()) {
            return {};
        }
        streaming::StreamingSettings settings;
        settings.loadRadius = active().streamingLoadRadius.value_or(kDefaultLoadRadiusM);
        settings.unloadRadius = active().streamingUnloadRadius.value_or(
            std::max(kDefaultUnloadRadiusM, settings.loadRadius * 4.0 / 3.0));
        settings.collisionRadius =
            active().streamingCollisionRadius.value_or(kDefaultCollisionRadiusM);
        // A run without ROS (headless, Python) is deterministic: its tiles follow the robot
        // only. A run with ROS takes viewer cameras as hints too (SetCameraInterest).
        settings.deterministic = !bridge;
        baseExtent = extentOf(robot);
        tileSelector = std::make_unique<streaming::TileSelector>(robot.tiles, settings);
        char text[160];
        std::snprintf(text, sizeof text,
                      "%zu streamed tiles (WorldTileAPI_1): load radius %.0f m, unload %.0f m, "
                      "colliders within %.0f m",
                      robot.tiles.size(), settings.loadRadius, settings.unloadRadius,
                      std::min(settings.collisionRadius, settings.loadRadius));
        log(v1::LOG_LEVEL_INFO, "streaming", text);
        return updateTiles(true);
    }

    /// Streaming again from nothing, with the radii of the profile now applied.
    Status restartStreaming() {
        if (!tileSelector) {
            return {};
        }
        for (auto& [id, load] : tileLoads) {
            (void)id;
            load.wait();
        }
        tileLoads.clear();
        tileQueue.clear();
        tileContent.clear();
        truncateModel(robot, baseExtent);
        tileSelector.reset();
        return startStreaming();
    }

    /// One streaming update: finished loads join the world, the selection runs on the robot's
    /// position, tiles leave or start loading. With `wait` (headless and deterministic runs) the
    /// loads finish here, so the tile set is a function of the robot's path; a served run takes
    /// them as they come and holds while the robot's ground is missing. When the tiles in the
    /// world changed, the run is rebuilt on them and carries its state over.
    Status updateTiles(bool wait) {
        if (!tileSelector) {
            return {};
        }
        bool changed = collectTiles(false);
        std::vector<streaming::Interest> interests{
            {streaming::Interest::Kind::Robot, focusWorld()}};
        if (cameraInterest) {
            interests.push_back({streaming::Interest::Kind::Camera, *cameraInterest});
        }
        const double now = scheduler ? scheduler->now().seconds() : 0.0;
        const streaming::StreamingPlan plan = tileSelector->update(interests, now);
        for (const std::uint32_t id : plan.unload) {
            tileContent.erase(id);
            tileSelector->unloaded(id);
            changed = true;
        }
        tileQueue.insert(tileQueue.end(), plan.load.begin(), plan.load.end());
        startTileLoads();
        if (wait) {
            while (!tileLoads.empty() || !tileQueue.empty()) {
                changed = collectTiles(true) || changed;
                startTileLoads();
            }
        }
        if (changed) {
            if (Status status = attachTiles(); !status.ok()) {
                return status;
            }
            char text[96];
            std::snprintf(text, sizeof text, "%zu tiles loaded, %zu loading", tileContent.size(),
                          tileLoads.size() + tileQueue.size());
            log(v1::LOG_LEVEL_DEBUG, "streaming", text);
        }
        const double margin = kGroundMarginM + 0.5 * robotSpeed();
        const bool ready = tileSelector->physicsReady(focusWorld(), margin);
        if (ready != !waitingForTiles) {
            waitingForTiles = !ready;
            log(v1::LOG_LEVEL_INFO, "streaming",
                ready ? "the robot's ground is loaded: the run goes on"
                      : "waiting for the tiles under the robot to load");
        }
        return {};
    }

    [[nodiscard]] double robotSpeed() const {
        if (!scheduler || robot.world.bodies.names.empty()) {
            return 0.0;
        }
        const SignalId twist =
            scheduler->bus().find("body/" + robot.world.bodies.names.front() + "/twist");
        if (!twist.valid()) {
            return 0.0;
        }
        const std::span<const double> v = scheduler->bus().readN(twist);
        return std::hypot(v[0], v[1]);
    }

    void startTileLoads() {
        while (tileLoads.size() < kTileWorkers && !tileQueue.empty()) {
            const std::uint32_t id = tileQueue.front();
            tileQueue.pop_front();
            const streaming::TileInfo& tile = tileSelector->tiles().at(id);
            tileLoads.emplace(
                id, std::async(std::launch::async, [source = tile.source, path = tile.path] {
                    stage::ReadOptions read;
                    read.addGround = false;
                    read.tile = path;
                    return stage::loadStage(source, read);
                }));
        }
    }

    /// Takes the finished loads (all of them, waiting, with `wait`). True if any joined.
    bool collectTiles(bool wait) {
        bool changed = false;
        for (auto it = tileLoads.begin(); it != tileLoads.end();) {
            if (!wait &&
                it->second.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                ++it;
                continue;
            }
            Expected<Model, std::string> loaded = it->second.get();
            if (loaded) {
                tileContent[it->first] = std::make_shared<const Model>(std::move(*loaded));
                tileSelector->loaded(it->first);
                changed = true;
            } else {
                // Left out of the world; asked for again at a later update.
                tileSelector->unloaded(it->first);
                log(v1::LOG_LEVEL_ERROR, "streaming", loaded.error());
            }
            it = tileLoads.erase(it);
        }
        return changed;
    }

    /// Puts the loaded tiles into the world tables and the scene, in tile order, after what was
    /// there before; rebuilds a running scheduler keeping its state.
    Status attachTiles() {
        truncateModel(robot, baseExtent);
        const std::vector<streaming::TileInfo>& tiles = tileSelector->tiles();
        for (const auto& [id, content] : tileContent) {
            // Loaded finer tiles over this one replace it where they are.
            std::vector<std::array<double, 4>> covered;
            for (const auto& [other, unused] : tileContent) {
                (void)unused;
                const streaming::TileInfo& t = tiles.at(other);
                if (t.level > tiles.at(id).level) {
                    covered.push_back(
                        {t.boundsMin[0], t.boundsMin[1], t.boundsMax[0], t.boundsMax[1]});
                }
            }
            appendTile(robot, *content, tiles.at(id).path, robot.world.region.worldEnu, covered);
        }
        ++tilesGeneration;
        if (!scheduler) {
            return {};
        }
        if (Status status = rebuildKeepingState(); !status.ok()) {
            return Status::error(status.code, "streaming: " + status.message);
        }
        resyncViewers();
        return {};
    }

    /// After a tick: every 100 ms of simulated time, the floating origin and the streamed
    /// tiles. A run without ROS (headless, deterministic) waits for its tile loads in the step,
    /// so its tile set depends only on the robot's path; a run with ROS takes them as they come.
    Status worldUpdate(bool headless) {
        if (scheduler->now().ns % kRebaseCheckNs != 0) {
            return {};
        }
        if (Status status = rebaseIfFar(); !status.ok()) {
            return status;
        }
        return replay ? Status{} : updateTiles(headless || !bridge);
    }

    /// Floating origin during a run (huge-worlds.md §1.2): every 100 ms of simulated time, if
    /// the robot is farther than kRebaseDistanceM from the region origin, the origin moves to
    /// its whole-metre position. The world tables shift, the run is rebuilt on them, and its
    /// state is carried over and shifted (Scheduler::shiftOrigin). The check runs at fixed
    /// simulated times, so a deterministic run stays deterministic.
    Status rebaseIfFar() {
        // A replay shows the recorded frame as it was recorded.
        if (replay || bodyPose.empty() || !bodyPose.front().valid() ||
            scheduler->now().ns % kRebaseCheckNs != 0) {
            return {};
        }
        const std::span<const double> pose = scheduler->bus().readN(bodyPose.front());
        const std::array<double, 3> offset =
            coords::rebaseShift({pose[0], pose[1], pose[2]}, kRebaseDistanceM);
        if (offset == std::array<double, 3>{0.0, 0.0, 0.0}) {
            return {};
        }
        StateWriter saved;
        if (Status status = scheduler->saveState(saved); !status.ok()) {
            return Status::error(status.code, "floating origin: " + status.message);
        }
        shiftRegion(robot, offset);
        if (Status status = build(); !status.ok()) {
            return Status::error(status.code, "floating origin: " + status.message);
        }
        StateReader reader(saved.bytes());
        if (Status status = scheduler->restoreState(reader); !status.ok()) {
            return Status::error(status.code, "floating origin: " + status.message);
        }
        scheduler->shiftOrigin(offset);
        shift = robot.world.region.worldEnu;
        lastRebase = scheduler->now();
        if (recorder) {
            recorder->frame(scheduler->now(), localFrame());
        }
        char text[200];
        std::snprintf(
            text, sizeof text,
            "floating origin moved by (%.0f, %.0f, %.0f) m at t = %.3f s: the "
            "region-local frame is now at (%.0f, %.0f, %.0f) m in world ENU (rebase %llu)",
            offset[0], offset[1], offset[2], scheduler->now().seconds(), shift[0], shift[1],
            shift[2], static_cast<unsigned long long>(robot.world.region.rebaseCount));
        log(v1::LOG_LEVEL_INFO, "stage", text);
        resyncViewers();
        return {};
    }

    /// The world prim standing for a scene prim, by path; kNoPrim if there is none.
    [[nodiscard]] std::uint32_t worldPrimOf(std::uint32_t scenePrim) const {
        if (scenePrim >= robot.scene.prims.size()) {
            return kNoIndex;
        }
        const std::string& path = robot.scene.prims[scenePrim].path;
        const auto& paths = robot.world.prims.paths;
        const auto it = std::find(paths.begin(), paths.end(), path);
        return it == paths.end() ? kNoIndex : static_cast<std::uint32_t>(it - paths.begin());
    }

    /// The scene prim of a world prim: a joint's is its child link's, a body's and a sensor's
    /// their own; else by path. kNoIndex if there is none.
    [[nodiscard]] std::uint32_t scenePrimOf(PrimId prim) const {
        const World& world = robot.world;
        for (std::size_t j = 0; j < world.joints.prim.size() && j < robot.jointPrim.size(); ++j) {
            if (world.joints.prim[j] == prim) {
                return robot.jointPrim[j];
            }
        }
        for (std::size_t b = 0; b < world.bodies.prim.size() && b < robot.bodyPrim.size(); ++b) {
            if (world.bodies.prim[b] == prim) {
                return robot.bodyPrim[b];
            }
        }
        for (std::size_t n = 0; n < world.sensors.prim.size() && n < robot.sensorPrim.size(); ++n) {
            if (world.sensors.prim[n] == prim) {
                return robot.sensorPrim[n];
            }
        }
        if (prim.index >= world.prims.paths.size()) {
            return kNoIndex;
        }
        const std::string& path = robot.world.prims.paths[prim.index];
        for (std::uint32_t p = 0; p < robot.scene.prims.size(); ++p) {
            if (robot.scene.prims[p].path == path) {
                return p;
            }
        }
        return kNoIndex;
    }

    /// Command SetAttribute: a number of one of the prim's schemas (or a body's mass), checked as
    /// the loader checks it, applied to the running simulation, and optionally saved.
    Status setAttribute(const v1::SetAttribute& request) {
        const v1::Attribute& attribute = request.attribute();
        const std::string& name = attribute.name();
        if (attribute.value_case() != v1::Attribute::kReal) {
            return Status::error(StatusCode::Unsupported,
                                 name + ": only numbers can be edited yet");
        }
        const double value = attribute.real();
        double* target = nullptr;
        std::string what;
        std::string unitText;
        if (name == "physics:mass") {
            for (std::size_t b = 0; b < robot.bodyPrim.size(); ++b) {
                if (robot.bodyPrim[b] == request.prim()) {
                    target = &robot.world.bodies.mass[b];
                    unitText = "kg";
                }
            }
            if (target != nullptr && !(value > 0.0 && std::isfinite(value))) {
                return Status::error(StatusCode::InvalidConfig, "physics:mass must be positive");
            }
        } else {
            const std::uint32_t prim = worldPrimOf(request.prim());
            for (SchemaInstance& s : robot.world.schemaInstances) {
                const auto it = s.reals.find(name);
                if (prim == kNoIndex || s.prim.index != prim || it == s.reals.end()) {
                    continue;
                }
                if (auto problem = stage::checkSchemaReal(s.schema, name, value)) {
                    return Status::error(StatusCode::InvalidConfig, *problem);
                }
                target = &it->second;
                what = s.schema;
                emiSourcesRead = false;  // an EMI source's value may change
                if (auto info = stage::schemaAttribute(s.schema, s.instance, name)) {
                    unitText = info->unit;
                }
            }
        }
        if (target == nullptr) {
            return Status::error(StatusCode::InvalidConfig,
                                 "this prim has no number " + name + " that can be edited");
        }
        const double old = *target;
        *target = value;
        if (Status status = rebuildKeepingState(); !status.ok()) {
            *target = old;
            return Status::error(status.code, name + " = " + std::to_string(value) +
                                                  " was not applied: " + status.message);
        }
        char line[256];
        std::snprintf(line, sizeof(line), "%s %s %g -> %g %s (from t = %.3f s)",
                      robot.scene.prims[request.prim()].path.c_str(), name.c_str(), old, value,
                      unitText.c_str(), scheduler->now().seconds());
        log(v1::LOG_LEVEL_INFO, "edit", line);
        broadcast(statusEnvelope());
        if (request.save_to_session_layer()) {
            return saveSession();
        }
        return {};
    }

    /// The session layer: the edited model as a USD layer that `--robot FILE --world none`
    /// reads back.
    Status saveSession() {
        std::filesystem::path path = options.sessionPath;
        if (path.empty()) {
            const char* stateHome = std::getenv("XDG_STATE_HOME");
            const char* home = std::getenv("HOME");
            const std::filesystem::path base =
                stateHome != nullptr && *stateHome != '\0'
                    ? std::filesystem::path(stateHome)
                    : std::filesystem::path(home != nullptr ? home : ".") / ".local" / "state";
            path = base / "lagrange" / (robot.name + ".session.usda");
        }
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
        stage::WriteOptions write;
        write.doc = "Session layer of lagrange_server: " + robot.sourcePath +
                    " with the edits made in the viewer. Run it with --robot this file --world "
                    "none.";
        if (auto written = stage::writeStage(robot, path.string(), write); !written) {
            return Status::error(
                StatusCode::Internal,
                "the edit applies, but the session layer was not saved: " + written.error());
        }
        log(v1::LOG_LEVEL_INFO, "edit", "session layer saved to " + path.string());
        return {};
    }

    /// A robot spawned during the run (spawnRobot): loaded again whenever the run is composed.
    struct Spawned {
        std::string name;  // its top prim's name, unique in the run
        std::string path;
        Pose spawn;             // world ENU, as --spawn
        bool primary{false};    // spawned into a run without a robot: the one ROS drives
        bool temporary{false};  // `path` was written from a resource string: removed with it
        bool human{false};      // a file of people (RFC 0009 §8): never the robot
    };
    std::vector<Spawned> spawned;
    std::optional<Model> scenarioRoads;  // the scenario's road network, when it is the world
    std::array<double, 3> rosOrigin{};   // world ENU of the ROS world frame: the first region
    bool composedMidRun{false};          // the model carries poses of a run rebuilt mid-way

    /// The robot ROS drives: the --robot file, or the one spawned into a run without one.
    [[nodiscard]] const Spawned* spawnedPrimary() const {
        for (const Spawned& candidate : spawned) {
            if (candidate.primary) {
                return &candidate;
            }
        }
        return nullptr;
    }
    [[nodiscard]] std::string primaryPath() const {
        const Spawned* primary = spawnedPrimary();
        return primary != nullptr ? primary->path : options.robotPath;
    }
    /// The robot's root body, for /odom; empty without a robot.
    [[nodiscard]] std::string baseLink() const {
        return primaryPath().empty() || robot.world.bodies.names.empty()
                   ? ""
                   : robot.world.bodies.names.front();
    }
    /// /robot_description is URDF; a robot from another format has none to publish.
    [[nodiscard]] std::string robotDescription() const {
        const std::string path = primaryPath();
        return !path.empty() && importers::formatOf(path) == importers::Format::Urdf
                   ? readFile(path)
                   : "";
    }

    /// The model of the run from its files: the robot (or the bare ground), the scenario's
    /// roads, the overlay, the other robots (--add-robot, then the spawned ones), the world,
    /// the scenario's entities and the traffic. `announce` logs what was added (at load).
    Expected<Robot, Status> composeModel(bool announce) {
        const auto invalid = [](std::string message) {
            return Unexpected{Status::error(StatusCode::InvalidConfig, std::move(message))};
        };
        Robot model;
        const Spawned* primary = spawnedPrimary();
        if (primary != nullptr) {
            auto loaded = loadModel(primary->path, {.spawn = primary->spawn});
            if (!loaded) {
                return invalid(primary->name + ": " + loaded.error());
            }
            model = std::move(*loaded);
            renameRobot(model, primary->name);
        } else if (!options.robotPath.empty()) {
            auto loaded = loadModel(options.robotPath, {.spawn = options.spawn});
            if (!loaded) {
                return invalid(loaded.error());
            }
            model = std::move(*loaded);
        } else {
            model = emptyModel();
        }
        if (scenarioRoads) {
            appendModel(model, *scenarioRoads);
        }
        if (!options.robotPath.empty() && !options.overlayPath.empty()) {
            if (auto applied = stage::applyOverlay(model, options.overlayPath); !applied) {
                return invalid(applied.error());
            }
        }
        // More robots in the run (huge-worlds.md §3: robots whose bubbles overlap share a region).
        for (const ServerOptions::ExtraRobot& extra : options.extraRobots) {
            auto other = loadModel(extra.path, {.spawn = extra.spawn});
            if (!other) {
                return invalid(extra.name + ": " + other.error());
            }
            prefixModel(*other, extra.name, true);
            appendModel(model, *other);
            if (announce) {
                log(v1::LOG_LEVEL_INFO, "stage",
                    "robot " + extra.name + " from " + extra.path +
                        " (passive: nothing drives it)");
            }
        }
        for (const Spawned& passive : spawned) {
            if (passive.primary || passive.human) {
                continue;
            }
            auto other = loadModel(passive.path, {.spawn = passive.spawn});
            if (!other) {
                return invalid(passive.name + ": " + other.error());
            }
            prefixModel(*other, passive.name, true);
            appendModel(model, *other);
        }
        // Motors' rotors turn with their joints: physics gets their reflected inertia.
        electrical::applyRotorInertia(model.world);
        // Propellers with a propeller file are drawn as their blades.
        rotor_aero::addPropellerBlades(model);
        if (!options.worldPath.empty()) {
            if (auto added = addModel(model, options.worldPath); !added) {
                return invalid(added.error());
            }
        }
        if (scenario) {
            if (auto added = openscenario::addEntities(model, *scenario, options.scenarioEgo);
                !added) {
                return invalid(options.scenarioPath + ": " + added.error());
            }
            if (announce) {
                log(v1::LOG_LEVEL_INFO, "scenario",
                    options.scenarioPath + " · " + std::to_string(scenario->entities.size()) +
                        " entities, " + options.scenarioEgo + " is the robot");
            }
        }
        // Traffic agents on the world's road lanes, away from the robots (traffic.md); not with
        // a scenario, which says who is on the road.
        if (!scenario && !WorldView(model.world).instancesOf("RoadLaneAPI_1").empty() &&
            options.trafficVehicles + options.trafficPedestrians > 0) {
            std::vector<traffic::Vec3> robots;
            for (std::size_t b = 0; b < model.world.bodies.names.size(); ++b) {
                if (model.world.bodies.parent[b] == kWorldBody) {
                    robots.push_back(model.bodyRestPose[b].position);
                }
            }
            const auto made = traffic::addAgents(
                model, {options.trafficVehicles, options.trafficPedestrians, options.trafficSeed},
                robots);
            if (announce) {
                log(v1::LOG_LEVEL_INFO, "traffic",
                    std::to_string(made[0]) + " vehicles and " + std::to_string(made[1]) +
                        " pedestrians on the road lanes (seed " +
                        std::to_string(options.trafficSeed) + ")");
            }
        }
        // People last, so that the first bodies stay the robot's (RFC 0009 §8).
        std::vector<humans::Spec> people = humans::takeSpecs(model);
        for (const Spawned& file : spawned) {
            if (!file.human) {
                continue;
            }
            auto loaded = loadModel(file.path);
            if (!loaded) {
                return invalid(file.name + ": " + loaded.error());
            }
            std::vector<humans::Spec> specs = humans::takeSpecs(*loaded);
            for (std::size_t i = 0; i < specs.size(); ++i) {
                specs[i].name = i == 0 ? file.name : file.name + "_" + std::to_string(i + 1);
                specs[i].start = compose(file.spawn, specs[i].start);
                people.push_back(std::move(specs[i]));
            }
        }
        for (humans::Spec& person : people) {
            humans::addHuman(model, std::move(person));
        }
        return model;
    }

    /// Where the run is, for a rebuild that changes its bodies: the time, the world pose of
    /// every free root body and the position of every joint, by name.
    struct Carried {
        SimTime now;
        std::map<std::string, Pose> roots;  // world ENU
        std::map<std::string, double> joints;
    };
    [[nodiscard]] Carried carry() const {
        Carried carried;
        carried.now = scheduler->now();
        const SignalBus& bus = scheduler->bus();
        const World::Bodies& bodies = robot.world.bodies;
        for (std::size_t b = 0; b < bodies.names.size() && b < bodyPose.size(); ++b) {
            if (bodies.parent[b] != kWorldBody || !bodyPose[b].valid()) {
                continue;
            }
            const std::span<const double> p = bus.readN(bodyPose[b]);
            Pose pose{{p[0], p[1], p[2]}, {p[3], p[4], p[5], p[6]}};
            for (std::size_t i = 0; i < 3; ++i) {
                pose.position.at(i) += robot.world.region.worldEnu.at(i);
            }
            carried.roots.emplace(bodies.names[b], pose);
        }
        for (const std::string& name : robot.world.joints.names) {
            if (const SignalId id = bus.find("joint/" + name + "/position"); id.valid()) {
                carried.joints.emplace(name, bus.read(id));
            }
        }
        return carried;
    }

    /// Puts a freshly composed model (world ENU, before recentre) where the run was.
    static void place(Robot& model, const Carried& carried) {
        World& world = model.world;
        const World::Bodies& bodies = world.bodies;
        std::vector<std::optional<Pose>> moves(bodies.names.size());  // per root: new ∘ old⁻¹
        for (std::size_t b = 0; b < bodies.names.size(); ++b) {
            const auto it = carried.roots.find(bodies.names[b]);
            if (bodies.parent[b] != kWorldBody || it == carried.roots.end()) {
                continue;
            }
            const Pose& old = model.bodyRestPose[b];
            const std::array<double, 4> back = conjugate(old.orientation);
            const std::array<double, 3> p = rotate(back, old.position);
            moves[b] = compose(it->second, Pose{{-p[0], -p[1], -p[2]}, back});
            world.bodies.poseInParent[b] = it->second;
        }
        for (std::size_t b = 0; b < bodies.names.size(); ++b) {
            std::size_t root = b;
            while (bodies.parent[root] != kWorldBody) {
                root = bodies.parent[root].index;
            }
            if (moves[root]) {
                model.bodyRestPose[b] = compose(*moves[root], model.bodyRestPose[b]);
            }
        }
        for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
            if (const auto it = carried.joints.find(world.joints.names[j]);
                it != carried.joints.end()) {
                world.joints.initialPosition[j] = it->second;
            }
        }
    }

    /// Makes a composed model (world ENU) the run's, built and started at `at`.
    Status install(Robot model, SimTime at) {
        for (auto& [id, load] : tileLoads) {
            (void)id;
            load.wait();
        }
        tileLoads.clear();
        tileQueue.clear();
        tileContent.clear();
        tileSelector.reset();
        robot = std::move(model);
        shift = recentre(robot);
        if (Status status = startStreaming(); !status.ok()) {
            return status;
        }
        if (Status status = placeLidar(); !status.ok()) {
            return status;
        }
        placeCameras();
        if (bridge) {
            bridge->setRobot(baseLink(), robotDescription(),
                             !WorldView(robot.world).instancesOf("VehicleAPI_1").empty());
        }
        if (Status status = build(); !status.ok()) {
            return status;
        }
        // A whole number of base ticks: the new run's tick may be longer than the old one's.
        const std::int64_t tick = scheduler->baseTick().ns;
        return scheduler->startAt(SimTime{(at.ns + tick - 1) / tick * tick});
    }

    /// Rebuilds the run from its files after the robots changed, where it was. On failure the
    /// change is undone by `undo` and the run rebuilt as it was.
    Status recompose(const std::function<void()>& undo) {
        const Carried carried = carry();
        auto composed = composeModel(false);
        Status status = composed ? Status{} : composed.error();
        if (status.ok()) {
            place(*composed, carried);
            stopRecording("the robots changed");
            status = install(std::move(*composed), carried.now);
            if (status.ok() && physicsBackend() == "jolt" && !carried.joints.empty()) {
                log(v1::LOG_LEVEL_WARN, "stage",
                    "Jolt starts joints at zero: the robots' joints were reset by the rebuild");
            }
        }
        if (!status.ok()) {
            undo();
            if (composed) {  // the run was taken down: put the previous one back
                auto previous = composeModel(false);
                if (!previous) {
                    fail(previous.error());
                    return status;
                }
                place(*previous, carried);
                if (Status back = install(std::move(*previous), carried.now); !back.ok()) {
                    fail(back);
                }
            }
            return status;
        }
        composedMidRun = true;
        error.clear();
        owedNs = 0;
        lastWall = Clock::now();
        ++catalogGeneration;
        for (auto& session : sessions) {
            if (!session->greeted) {
                continue;
            }
            session->subscriptions.clear();
            session->sensors.clear();
            session->send(catalog());
            session->send(snapshot(*session));
            session->send(profileState({}));  // its stage and domains follow the robots
        }
        broadcast(statusEnvelope());
        return {};
    }

    [[nodiscard]] static bool validName(std::string_view name) {
        return !name.empty() &&
               (std::isalpha(static_cast<unsigned char>(name[0])) != 0 || name[0] == '_') &&
               std::ranges::all_of(name, [](char c) {
                   return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
               });
    }
    [[nodiscard]] bool nameTaken(const std::string& name) const {
        return std::ranges::any_of(robot.scene.prims, [&name](const ScenePrim& prim) {
            return prim.parent == kNoIndex && prim.name == name;
        });
    }

    /// spawnRobot, with the simulation_interfaces result code.
    ros2::SpawnOutcome spawn(const std::string& path, std::string name, bool allowRenaming,
                             const Pose& at, bool temporary) {
        using ros2::SimResult;
        const auto refuse = [&](SimResult code, std::string message) {
            if (temporary) {
                std::filesystem::remove(path);
            }
            return ros2::SpawnOutcome{{code, std::move(message)}, ""};
        };
        if (replay) {
            return refuse(SimResult::Failed, "a replay shows the recorded robots only");
        }
        if (!std::filesystem::is_regular_file(path)) {
            return refuse(SimResult::ResourceParseError, "no file " + path);
        }
        if (importers::formatOf(path) == importers::Format::Unknown && !isUsd(path)) {
            return refuse(SimResult::UnsupportedFormat,
                          path + " is not URDF, SDFormat, MJCF or USD (xacro: run xacro first)");
        }
        if (!name.empty() && !validName(name)) {
            return refuse(SimResult::NameInvalid,
                          "'" + name +
                              "' is not a valid name: letters, digits and _, not first "
                              "a digit");
        }
        if (name.empty() || nameTaken(name)) {
            if (!name.empty() && !allowRenaming) {
                return refuse(SimResult::NameNotUnique, "an entity is already named " + name);
            }
            std::string base = name;
            if (base.empty()) {  // from the file name
                for (const char c : std::filesystem::path(path).stem().string()) {
                    base += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
                }
                if (!validName(base)) {
                    base = "robot_" + base;
                }
            }
            name = base;
            for (int n = 2; nameTaken(name); ++n) {
                name = base + "_" + std::to_string(n);
            }
        }
        // A file of people is not the robot, however empty the run (RFC 0009 §8).
        bool human = false;
        if (auto file = loadModel(path); file && file->world.bodies.names.empty()) {
            human = !humans::takeSpecs(*file).empty();
        }
        const bool primary = !human && primaryPath().empty();
        spawned.push_back({name, path, at, primary, temporary, human});
        if (Status status = recompose([this] {
                if (spawned.back().temporary) {
                    std::filesystem::remove(spawned.back().path);
                }
                spawned.pop_back();
            });
            !status.ok()) {
            return {{SimResult::ResourceParseError, status.message}, ""};
        }
        log(v1::LOG_LEVEL_INFO, "stage",
            "spawned " + name + " from " + path +
                (human     ? " (a person)"
                 : primary ? " (the robot: ROS drives it)"
                           : " (passive: nothing drives it)"));
        return {{}, "/" + name};
    }

    /// removeRobot, with the simulation_interfaces result code.
    ros2::SimOutcome remove(std::string name) {
        using ros2::SimResult;
        if (name.starts_with('/')) {
            name.erase(0, 1);
        }
        if (replay) {
            return {SimResult::Failed, "a replay shows the recorded robots only"};
        }
        const auto spawnedAt = std::ranges::find(spawned, name, &Spawned::name);
        const auto extraAt =
            std::ranges::find(options.extraRobots, name, &ServerOptions::ExtraRobot::name);
        const bool startupRobot = spawnedPrimary() == nullptr && !options.robotPath.empty() &&
                                  !robot.scene.prims.empty() &&
                                  robot.scene.prims.front().name == name;
        if (startupRobot || (spawnedAt != spawned.end() && spawnedAt->primary)) {
            if (controllers) {
                return {SimResult::Failed, "ros2_control drives " + name +
                                               ": it stays (start without --ros2-control)"};
            }
        }
        if (spawnedAt != spawned.end()) {
            const Spawned removed = *spawnedAt;
            const auto index = spawnedAt - spawned.begin();
            spawned.erase(spawnedAt);
            if (Status status =
                    recompose([&] { spawned.insert(spawned.begin() + index, removed); });
                !status.ok()) {
                return {SimResult::Failed, status.message};
            }
            if (removed.temporary) {
                std::filesystem::remove(removed.path);
            }
        } else if (extraAt != options.extraRobots.end()) {
            const ServerOptions::ExtraRobot removed = *extraAt;
            const auto index = extraAt - options.extraRobots.begin();
            options.extraRobots.erase(extraAt);
            if (Status status = recompose([&] {
                    options.extraRobots.insert(options.extraRobots.begin() + index, removed);
                });
                !status.ok()) {
                return {SimResult::Failed, status.message};
            }
        } else if (startupRobot) {
            const std::string removed = options.robotPath;
            options.robotPath.clear();
            if (Status status = recompose([&] { options.robotPath = removed; }); !status.ok()) {
                return {SimResult::Failed, status.message};
            }
        } else {
            return {SimResult::NotFound,
                    "no robot named " + name + " (the world's scenery cannot be deleted)"};
        }
        log(v1::LOG_LEVEL_INFO, "stage", "deleted " + name);
        return {};
    }

    /// Where the viewer's Import robot puts a robot: at the origin of an empty run, else 3 m
    /// to the side of the robots there are.
    [[nodiscard]] Pose importPose() const {
        Pose at;
        at.position = rosOrigin;
        at.position[2] = 0.0;
        double side = -std::numeric_limits<double>::infinity();
        for (std::size_t b = 0; b < robot.world.bodies.names.size(); ++b) {
            if (robot.world.bodies.parent[b] == kWorldBody && b < bodyPose.size() &&
                bodyPose[b].valid()) {
                side = std::max(side, scheduler->bus().readN(bodyPose[b])[1]);
            }
        }
        if (std::isfinite(side)) {
            at.position[1] = side + robot.world.region.worldEnu[1] + 3.0;
        }
        return at;
    }

    Status restart() {
        stopRecording("the run was reset");
        const bool wasRunning = state == v1::RUN_STATE_RUNNING;
        Status status;
        if (composedMidRun) {  // back to where the robots were authored, not where they were
            auto fresh = composeModel(false);
            status = fresh ? install(std::move(*fresh), SimTime{0}) : fresh.error();
            composedMidRun = !status.ok();
        } else {
            status = build();
        }
        if (!status.ok()) {
            fail(status);
            return status;
        }
        state = wasRunning ? v1::RUN_STATE_RUNNING : v1::RUN_STATE_PAUSED;
        error.clear();
        owedNs = 0;
        lastWall = Clock::now();
        log(v1::LOG_LEVEL_INFO, "scheduler", "reset · the run restarted at t = 0");
        broadcast(environmentState());
        return {};
    }

    void resyncViewers() {
        for (auto& session : sessions) {
            if (session->greeted) {
                session->send(snapshot(*session));
            }
        }
    }

    /// Steps `ticks` base ticks while paused, publishing to ROS after each.
    Status stepTicks(std::uint64_t ticks) {
        for (std::uint64_t i = 0; i < ticks; ++i) {
            if (Status status = scheduler->stepTick(); !status.ok()) {
                fail(status);
                return status;
            }
            capture();
            record();
            if (Status status = worldUpdate(false); !status.ok()) {
                fail(status);
                return status;
            }
            if (bridge) {
                bridge->publish(scheduler->bus(), scheduler->now());
            }
        }
        emit();
        return {};
    }

    /// What this server can do with a profile, for checkProfile.
    ProfileContext profileContext() const {
        ProfileContext context;
        context.stage = stageName();
        context.modules.insert("rigid_body");
        if (hasSensors()) {
            context.modules.insert("sensors");
        }
        if (hasMotors()) {
            context.modules.insert("electrical");
        }
        if (hasThermalNodes()) {
            context.modules.insert("thermal");
        }
        if (hasRotors()) {
            context.modules.insert("rotor_aero");
        }
        if (!WorldView(robot.world).instancesOf("AeroAPI_1").empty()) {
            context.modules.insert("aerodynamics");
        }
        context.modules.insert("weather");
        if (hasEmi()) {
            context.modules.insert("emi");
        }
        if (hasComms()) {
            context.modules.insert("comms");
        }
        if (hasVehicles()) {
            context.modules.insert("vehicle_dynamics");
        }
        if (hasTraffic()) {
            context.modules.insert("traffic");
        }
        if (hasHumans()) {
            context.modules.insert("humans");
        }
        for (const auto& [key, value] : options.weather) {
            context.weatherKnown.insert(key);
        }
        if (const auto prims = WorldView(robot.world).instancesOf("WeatherAPI_1"); !prims.empty()) {
            for (const auto& [key, attribute] :
                 {std::pair{"temperature", "weather:ambientTemperature"},
                  std::pair{"pressure", "weather:pressure"},
                  std::pair{"relative_humidity", "weather:relativeHumidity"}}) {
                if (prims.front()->real(attribute)) {
                    context.weatherKnown.insert(key);
                }
            }
        }
        context.ambientKnown = options.ambientTemperatureK.has_value();
        context.airDensityKnown = options.airDensityKgM3.has_value();
        for (const DomainRow& row : kDomains) {
            context.substitutes[row.domain] = {row.substitute, row.substituteKind};
            if (std::string_view(row.domain) == "rigid_body") {
                context.substitutes[row.domain].insert("replay");  // bench mode
            }
        }
        for (const PluginModule& module : plugins) {
            if (module.substitute) {
                context.substitutes[module.domain].insert(module.name);
            } else if (module.domain == "rigid_body") {
                context.rigidBodyBackends.insert(module.name);
            }
        }
        context.pluginLibraries.insert(options.plugins.begin(), options.plugins.end());
        return context;
    }

    /// Checks a profile and shows the wiring it would build (or the running one if it does not
    /// parse into settings).
    void validate(const std::string& yaml, v1::ProfileValidation& out) {
        const ProfileCheck check = this->check(yaml);
        out.set_valid(check.valid);
        for (const ProfileIssue& issue : check.issues) {
            v1::ProfileIssue& item = *out.add_issues();
            item.set_severity(issue.error ? v1::ProfileIssue::SEVERITY_ERROR
                                          : v1::ProfileIssue::SEVERITY_WARNING);
            item.set_json_pointer(issue.pointer);
            item.set_message(issue.message);
        }
        if (check.settings) {
            auto preview = makeScheduler(withRunningSite(*check.settings), nullptr);
            if (!preview) {
                out.set_valid(false);
                v1::ProfileIssue& item = *out.add_issues();
                item.set_severity(v1::ProfileIssue::SEVERITY_ERROR);
                item.set_json_pointer("/domains");
                item.set_message(preview.error());
                describeWiring(*scheduler, *out.mutable_wiring());
                return;
            }
            describeWiring(**preview, *out.mutable_wiring());
            return;
        }
        describeWiring(*scheduler, *out.mutable_wiring());
    }

    /// `settings` with the site parameters it leaves out (ambient temperature, air density, geo
    /// origin) taken from the running ones, as adopt keeps them: what a check wires.
    ProfileSettings withRunningSite(ProfileSettings settings) const {
        const ProfileSettings running = active();
        if (!settings.ambientTemperatureK) {
            settings.ambientTemperatureK = ambient();
        }
        if (!settings.airDensityKgM3) {
            settings.airDensityKgM3 = running.airDensityKgM3;
        }
        if (!settings.geoOrigin) {
            settings.geoOrigin = running.geoOrigin;
        }
        return settings;
    }

    /// checkProfile, then whether the settings wire (as validate, without the wiring preview).
    ProfileCheck check(const std::string& yaml) {
        if (replay) {
            // A replay runs what was recorded: its own profile is the only one, and it is valid.
            ProfileCheck check;
            check.valid = true;
            check.settings = active();
            if (yaml != profileYaml()) {
                check.issues.push_back({false, "/domains",
                                        "a replay runs what was recorded; profile changes do "
                                        "not apply to it"});
            }
            return check;
        }
        ProfileCheck check = checkProfile(yaml, schemaText, profileContext());
        if (check.settings) {
            if (auto preview = makeScheduler(withRunningSite(*check.settings), nullptr); !preview) {
                check.valid = false;
                check.issues.push_back({true, "/domains", preview.error()});
            }
        }
        return check;
    }

    /// Adopts a valid profile: rebuilds the run at t = 0 now, or at the next reset.
    Status apply(const std::string& yaml, bool restartNow) {
        if (replay) {
            return Status::error(StatusCode::Unsupported,
                                 "profile not applied: a replay runs what was recorded");
        }
        const ProfileCheck check = checkProfile(yaml, schemaText, profileContext());
        if (!check.valid || !check.settings) {
            for (const ProfileIssue& issue : check.issues) {
                if (issue.error) {
                    return Status::error(StatusCode::InvalidConfig,
                                         "profile not applied: " +
                                             (issue.pointer.empty() ? "" : issue.pointer + ": ") +
                                             issue.message);
                }
            }
            return Status::error(StatusCode::InvalidConfig, "profile not applied");
        }
        if (check.settings->realTimeFactor) {
            rtfTarget = *check.settings->realTimeFactor;  // takes effect at once
        }
        if (!restartNow) {
            pending = check.settings;
            return {};
        }
        return adopt(*check.settings);
    }

    /// Makes `settings` the run's and restarts at t = 0.
    Status adopt(const ProfileSettings& settings) {
        const ProfileSettings previous = active();
        options.substituted = settings.substituted;
        rigidBodyDtNs = settings.rigidBodyDtNs;
        rigidBodyBackend = settings.rigidBodyBackend;
        appliedTireModel = settings.tireModel;
        electricalDtNs = settings.electricalDtNs;
        thermalDtNs = settings.thermalDtNs;
        speedPrediction = settings.speedPrediction;
        electricalFidelity = settings.electricalFidelity;
        rigidBodyReplay = settings.rigidBodyReplay;
        const std::optional<double> previousAmbient = appliedAmbientK;
        if (settings.ambientTemperatureK) {
            appliedAmbientK = settings.ambientTemperatureK;
        }
        if (settings.airDensityKgM3) {
            appliedAirDensity = settings.airDensityKgM3;
        }
        if (settings.geoOrigin) {
            appliedGeoOrigin = settings.geoOrigin;
        }
        if (settings.streamingLoadRadius != appliedLoadRadius ||
            settings.streamingUnloadRadius != appliedUnloadRadius ||
            settings.streamingCollisionRadius != appliedCollisionRadius) {
            appliedLoadRadius = settings.streamingLoadRadius;
            appliedUnloadRadius = settings.streamingUnloadRadius;
            appliedCollisionRadius = settings.streamingCollisionRadius;
            if (Status status = restartStreaming(); !status.ok()) {
                return status;
            }
        }
        appliedAerodynamics = settings.aerodynamics;
        appliedRotorAero = settings.rotorAero;
        appliedWeather = settings.weather;
        appliedEmiCoupling = settings.emiCoupling;
        const auto previousTokens = appliedSubstituteTokens;
        const auto previousPlugins = appliedPlugins;
        appliedSubstituteTokens = settings.substituteTokens;
        appliedPlugins = settings.plugins;
        pending.reset();
        if (Status status = restart(); !status.ok()) {
            // Keep the run going on the profile it had.
            options.substituted = previous.substituted;
            rigidBodyDtNs = previous.rigidBodyDtNs;
            rigidBodyBackend = previous.rigidBodyBackend;
            electricalDtNs = previous.electricalDtNs;
            thermalDtNs = previous.thermalDtNs;
            speedPrediction = previous.speedPrediction;
            electricalFidelity = previous.electricalFidelity;
            rigidBodyReplay = previous.rigidBodyReplay;
            appliedAmbientK = previousAmbient;
            appliedSubstituteTokens = previousTokens;
            appliedPlugins = previousPlugins;
            (void)restart();
            return status;
        }
        log(v1::LOG_LEVEL_INFO, "profile", "applied; the run restarted at t = 0");
        return {};
    }

    /// The modules of `wired` and the wires between them, grouped by signal pattern.
    void describeWiring(const Scheduler& wired, v1::Wiring& wiring) const {
        std::int64_t smallest = 0;
        for (const ScheduledModule& module : wired.plan()) {
            const std::string_view label = module.label;
            const std::size_t slash = label.find('/');
            v1::Wiring::Node& node = *wiring.add_nodes();
            node.set_module(std::string(label.substr(0, slash)));
            node.set_domain(std::string(label.substr(0, slash)));
            node.set_detail(std::string(label.substr(slash + 1)));
            node.set_dt_ns(module.dt.ns);
            smallest = smallest == 0 ? module.dt.ns : std::min(smallest, module.dt.ns);
        }
        // One wire per (writer, reader, signal pattern): "joint/left_wheel_joint/torque" and
        // its right twin become "joint/*/torque".
        std::map<std::tuple<std::string, std::string, std::string>, const Connection*> grouped;
        const std::vector<Connection> connections = wired.connections();
        for (const Connection& c : connections) {
            std::string pattern = c.signal;
            const std::size_t first = pattern.find('/');
            const std::size_t second = pattern.find('/', first + 1);
            if (first != std::string::npos && second != std::string::npos) {
                pattern.replace(first + 1, second - first - 1, "*");
            }
            grouped.emplace(std::make_tuple(c.writer.substr(0, c.writer.find('/')),
                                            c.reader.substr(0, c.reader.find('/')), pattern),
                            &c);
        }
        const auto addOnce = [](google::protobuf::RepeatedPtrField<std::string>& list,
                                const std::string& item) {
            if (std::find(list.begin(), list.end(), item) == list.end()) {
                *list.Add() = item;
            }
        };
        for (const auto& [key, c] : grouped) {
            const std::string symbol = std::get<2>(key).substr(std::get<2>(key).rfind('/') + 1);
            for (v1::Wiring::Node& node : *wiring.mutable_nodes()) {
                if (node.module() == std::get<0>(key)) {
                    addOnce(*node.mutable_outputs(), symbol);
                }
                if (node.module() == std::get<1>(key)) {
                    addOnce(*node.mutable_inputs(), symbol);
                }
            }
            v1::Wiring::Wire& wire = *wiring.add_wires();
            wire.set_from(std::get<0>(key));
            wire.set_to(std::get<1>(key));
            wire.set_signal(std::get<2>(key));
            wire.set_symbol(std::get<2>(key).substr(std::get<2>(key).rfind('/') + 1));
            wire.set_source_unit(label(c->writerUnit));
            wire.set_sink_unit(label(c->readerUnit));
            wire.set_unit_ok(c->writerUnit == c->readerUnit);
        }
        wiring.set_base_tick_ns(wired.baseTick().ns);
        wiring.set_sync_ns(smallest);
    }

    /// What the ROS 2 workspace shows: the live graph seen by the bridge, or why there is none.
    v1::Envelope rosGraph() {
        v1::Envelope envelope;
        v1::RosGraph& graph = *envelope.mutable_ros_graph();
        graph.set_distro("lyrical");
        if (!bridge) {
            graph.set_note("this server runs without its ROS 2 bridge (started with --no-ros)");
            return envelope;
        }
        const ros2::GraphSnapshot snapshot = bridge->graph();
        for (const ros2::GraphSnapshot::Topic& source : snapshot.topics) {
            v1::RosGraph::Topic& topic = *graph.add_topics();
            topic.set_name(source.name);
            topic.set_type(source.type);
            topic.set_publishers(source.publishers);
            topic.set_subscribers(source.subscribers);
            topic.set_rate_hz(source.rateHz);
            topic.set_bandwidth_bps(source.bandwidthBps);
            topic.set_from_bridge(source.fromBridge);
            if (source.name == "/cmd_vel" && source.publishers == 0) {
                topic.set_status("no publisher: the demo drives");
            }
        }
        for (const ros2::GraphSnapshot::Node& source : snapshot.nodes) {
            graph.add_nodes(source.name);
            v1::RosGraph::Node& node = *graph.add_node_info();
            node.set_name(source.name);
            node.set_external(source.external);
            for (const std::string& topic : source.publishes) {
                node.add_publishes(topic);
            }
            for (const std::string& topic : source.subscribes) {
                node.add_subscribes(topic);
            }
            if (!source.external) {
                node.set_detail(std::string("simulator bridge · drive: ") +
                                (driveCommand != nullptr ? driveCommand->source() : "none"));
            }
        }
        graph.set_rmw(snapshot.rmw);
        if (!snapshot.distro.empty()) {
            graph.set_distro(snapshot.distro);
        }
        graph.set_domain_id(snapshot.domainId);
        graph.set_use_sim_time(true);  // what the bridge's /clock is for
        return envelope;
    }
    std::unique_ptr<ros2::Bridge> bridge;
    std::unique_ptr<ros2::SimulationHandler> simulation;  // behind the bridge's services
    DriveCommand* driveCommand{nullptr};
    std::unique_ptr<ros2_control::ControllerHost> controllers;  // with --ros2-control
    std::vector<ros2_control::ControlJoint> controlJoints;

    /// The World workspace: the robot as the one agent, at its simulated position.
    /// The weather values the run uses: the command line's, then the applied profile's.
    std::map<std::string, ConfigValue, std::less<>> weatherValues() const {
        std::map<std::string, ConfigValue, std::less<>> values;
        for (const auto& [key, value] : options.weather) {
            values[key] = value;
        }
        for (const auto& [key, value] : options.weatherFiles) {
            values[key] = value;
        }
        for (const auto& [key, value] : appliedWeather) {
            values[key] = value;
        }
        return values;
    }

    /// The run's wind (the aerodynamics domain's config), if it has one.
    std::optional<aerodynamics::Wind> runWind() const {
        const ProfileSettings settings = active();
        if (settings.aerodynamics.empty()) {
            return std::nullopt;
        }
        ModuleConfigData data;
        data.values.insert(settings.aerodynamics.begin(), settings.aerodynamics.end());
        const auto wind = aerodynamics::AeroDrag::windOf(ModuleConfig(data));
        return wind ? std::optional(*wind) : std::nullopt;
    }

    double busValue(const char* name, double fallback) const {
        if (scheduler == nullptr) {
            return fallback;
        }
        const SignalId id = scheduler->bus().find(name);
        return id.valid() ? scheduler->bus().read(id) : fallback;
    }

    /// The environment as the run has it (protocol.md: EnvironmentState), for the viewer's
    /// Environment workspace.
    v1::Envelope environmentState() const {
        v1::Envelope envelope;
        v1::EnvironmentState& env = *envelope.mutable_environment();
        const auto values = weatherValues();
        const auto number = [&](const char* key) -> std::optional<double> {
            const auto it = values.find(key);
            if (it == values.end()) {
                return std::nullopt;
            }
            const double* v = std::get_if<double>(&it->second);
            return v != nullptr ? std::optional(*v) : std::nullopt;
        };
        const auto text = [&](const char* key) -> std::string {
            const auto it = values.find(key);
            const std::string* v =
                it == values.end() ? nullptr : std::get_if<std::string>(&it->second);
            return v != nullptr ? std::filesystem::path(*v).filename().string() : std::string();
        };
        const bool weather = weatherRuns() && !substituted("weather");
        if (const auto t = ambient() ? ambient() : number("temperature")) {
            env.set_ambient_temperature_k(*t);
        }
        v1::EnvironmentState::Weather& w = *env.mutable_weather();
        if (weather) {
            constexpr double kMmphPerMps = 3.6e6;
            constexpr double kMgPerKg = 1e6;
            constexpr double kWetFriction = 0.7;  // weather::kWetFrictionScale
            w.set_rain_mm_per_h(number("rain_rate").value_or(0.0) * kMmphPerMps);
            w.set_fog_visibility_m(number("visibility").value_or(0.0));
            w.set_dust_mg_per_m3(number("dust_concentration").value_or(0.0) * kMgPerKg);
            w.set_surface_wetness(std::clamp(
                (1.0 - busValue("env/friction_scale", 1.0)) / (1.0 - kWetFriction), 0.0, 1.0));
        }
        if (const auto wind = runWind()) {
            v1::EnvironmentState::Wind& out = *env.mutable_wind();
            out.set_speed_m_per_s(wind->speed);
            // Where it blows to, from +X towards +Y: the opposite of its meteorological bearing.
            out.set_direction_deg(std::fmod(90.0 - (wind->fromDegrees + 180.0) + 720.0, 360.0));
            out.set_gust_intensity(wind->dryden ? 1.0 : 0.0);
        }
        if (weather && !text("wind_field").empty()) {
            env.mutable_wind()->set_field(text("wind_field"));
        }
        const auto field = [&](const char* name, const char* label, const char* domain,
                               const Unit& unit, std::uint32_t components, bool available,
                               const char* why) {
            v1::EnvironmentState::Field& f = *env.add_fields();
            f.set_name(name);
            f.set_label(label);
            f.set_domain(domain);
            setUnit(*f.mutable_unit(), unit);
            f.set_components(components);
            f.set_available(available);
            if (!available) {
                f.set_unavailable_reason(why);
            }
        };
        field(kFogField, "fog extinction", "weather", units::per_m, 1, weather,
              "the weather does not run: it needs the site's temperature, pressure and humidity");
        field(kWindField, "wind", "weather", units::m_per_s, 2,
              runWind().has_value() || (weather && !text("wind_field").empty()),
              "no wind: the aerodynamics domain's wind or a wind field");
        field(kFluxField, "|B|", "emi", units::T, 1, emiRuns(),
              hasEmi() ? "the EMI domain is off (zero substitute)"
                       : "the stage has no EMI sources (EmiSourceAPI_1)");
        if (emiRuns()) {
            for (const emi::Source& s : emiSources()) {
                v1::EnvironmentState::EmiSource& out = *env.add_emi_sources();
                out.set_name(s.name);
                out.set_prim(0xFFFF'FFFFU);
                out.set_frequency_hz(s.frequency);
                out.set_current_a(emiState(s).current);
                out.set_power_w(s.radiatedPower);
                static constexpr std::array<const char*, 5> kKinds{
                    "power line", "motor drive (dipole)", "transmitter", "welder (dipole)",
                    "custom"};
                out.set_detail(s.fieldMap ? "field map"
                                          : kKinds.at(static_cast<std::size_t>(s.type)));
            }
        }
        return envelope;
    }

    /// The run's EMI sources, read from the stage once.
    const std::vector<emi::Source>& emiSources() const {
        if (!emiSourcesRead) {
            auto sources = emi::sourcesOf(WorldView(robot.world));
            cachedEmiSources = sources ? std::move(*sources) : std::vector<emi::Source>{};
            emiSourcesRead = true;
        }
        return cachedEmiSources;
    }

    /// A source's pose and current now, from the bus (its body's pose, its motor's current).
    emi::SourceState emiState(const emi::Source& s) const {
        emi::SourceState now;
        now.current = s.current;
        Pose body;
        if (s.body != kWorldBody) {
            body = robot.bodyRestPose.at(s.body.index);
        }
        if (scheduler != nullptr) {
            const SignalBus& bus = scheduler->bus();
            if (s.body != kWorldBody) {
                const SignalId id =
                    bus.find("body/" + robot.world.bodies.names[s.body.index] + "/pose");
                if (id.valid()) {
                    const std::span<const double> v = bus.readN(id);
                    body = Pose{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
                }
            }
            if (!s.drivenBy.empty()) {
                const SignalId id = bus.find("motor/" + s.drivenBy + "/current_q");
                if (id.valid()) {
                    now.current = std::abs(bus.read(id));
                }
            }
        }
        now.pose = s.body == kWorldBody ? s.poseInBody : compose(body, s.poseInBody);
        return now;
    }

    /// A horizontal slice of `name` at height `height` around the robot, or nothing for a field
    /// the run does not have.
    std::optional<v1::Envelope> fieldSlice(const std::string& name, double height) const {
        if (scheduler == nullptr ||
            (name != kFogField && name != kWindField && name != kFluxField)) {
            return std::nullopt;
        }
        if (name == kFluxField && !emiRuns()) {
            return std::nullopt;
        }
        const bool fog = name == kFogField;
        const bool flux = name == kFluxField;
        const SignalBus& bus = scheduler->bus();
        const SignalId gridId = bus.find(fog ? "env/extinction/field" : "env/wind/velocity_field");
        const std::shared_ptr<const fields::Field> grid =
            gridId.valid() ? (fog ? fogFieldSignal : windFieldSignal).read(bus, gridId) : nullptr;
        const std::optional<aerodynamics::Wind> wind = runWind();
        if (fog && !bus.find("env/extinction/visible").valid()) {
            return std::nullopt;
        }
        if (!fog && !flux && !grid && !wind) {
            return std::nullopt;
        }
        std::vector<emi::SourceState> sources;
        if (flux) {
            for (const emi::Source& s : emiSources()) {
                sources.push_back(emiState(s));
            }
        }
        // Centred on the robot.
        Pose centre = robot.bodyRestPose.empty() ? Pose{} : robot.bodyRestPose[0];
        if (!bodyPose.empty() && bodyPose[0].valid()) {
            const std::span<const double> v = bus.readN(bodyPose[0]);
            centre.position = {v[0], v[1], v[2]};
        }
        v1::Envelope envelope;
        v1::FieldSlice& slice = *envelope.mutable_field_slice();
        slice.set_field(name);
        slice.mutable_sim_time()->set_ns(scheduler->now().ns);
        slice.set_width(kSliceWidth);
        slice.set_height(kSliceHeight);
        const std::uint32_t components = fog || flux ? 1 : 2;
        slice.set_components(components);
        slice.set_origin_x(centre.position[0] - (kSliceWidth - 1) * kSliceCell / 2.0);
        slice.set_origin_y(centre.position[1] - (kSliceHeight - 1) * kSliceCell / 2.0);
        slice.set_cell_m(kSliceCell);
        slice.set_height_m(height);
        std::vector<float> values;
        values.reserve(static_cast<std::size_t>(kSliceWidth) * kSliceHeight * components);
        const double uniform = fog ? bus.read(bus.find("env/extinction/visible")) : 0.0;
        double lo = std::numeric_limits<double>::infinity();
        double hi = 0.0;
        for (std::uint32_t j = 0; j < kSliceHeight; ++j) {
            for (std::uint32_t i = 0; i < kSliceWidth; ++i) {
                const fields::Vec3 at{slice.origin_x() + i * kSliceCell,
                                      slice.origin_y() + j * kSliceCell, height};
                if (flux) {
                    // |B| RMS; sources add in power (as incoherent ones would).
                    double b2 = 0.0;
                    for (std::size_t k = 0; k < sources.size(); ++k) {
                        const double b = emi::magneticFieldRms(emiSources()[k], sources[k], at).b;
                        b2 += b * b;
                    }
                    const double v = std::sqrt(b2);
                    values.push_back(static_cast<float>(v));
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                    continue;
                }
                if (fog) {
                    const double v = uniform + (grid ? grid->sampleScalar(at) : 0.0);
                    values.push_back(static_cast<float>(v));
                    lo = std::min(lo, v);
                    hi = std::max(hi, v);
                    continue;
                }
                fields::Vec3 v{};
                const auto& box = grid ? grid->bounds() : std::array<fields::Vec3, 2>{};
                if (grid && at[0] >= box[0][0] && at[0] <= box[1][0] && at[1] >= box[0][1] &&
                    at[1] <= box[1][1] && at[2] >= box[0][2] && at[2] <= box[1][2]) {
                    v = grid->sampleVector(at);
                } else if (wind) {
                    const double from = wind->fromDegrees * std::numbers::pi / 180.0;
                    const double u = aerodynamics::meanWindAt(*wind, height);
                    v = {-u * std::sin(from), -u * std::cos(from), 0.0};
                }
                values.push_back(static_cast<float>(v[0]));
                values.push_back(static_cast<float>(v[1]));
                const double m = std::hypot(v[0], v[1]);
                lo = std::min(lo, m);
                hi = std::max(hi, m);
            }
        }
        slice.set_value_min(static_cast<float>(lo));
        slice.set_value_max(static_cast<float>(hi));
        std::string bytes(reinterpret_cast<const char*>(values.data()),
                          values.size() * sizeof(float));
        slice.mutable_values()->set_size(bytes.size());
        slice.mutable_values()->set_inline_(std::move(bytes));
        return envelope;
    }

    v1::Envelope worldState() const {
        v1::Envelope envelope;
        v1::WorldState& world = *envelope.mutable_world();
        world.mutable_sim_time()->set_ns(scheduler->now().ns);
        const std::array<double, 3>& region = robot.world.region.worldEnu;
        if (const auto& geo = robot.world.geoOrigin) {
            constexpr double kDegree = 180.0 / std::numbers::pi;
            v1::WorldState::GeoOrigin& g = *world.mutable_geo_origin();
            g.set_valid(true);
            g.set_latitude_deg(geo->latitudeRad * kDegree);
            g.set_longitude_deg(geo->longitudeRad * kDegree);
            g.set_height_m(geo->heightM);
        } else {
            world.mutable_geo_origin()->set_valid(false);
        }
        streamedTiles(world);
        v1::WorldState::Agent& agent = *world.add_agents();
        agent.set_name(robot.name);
        agent.set_prim(robot.bodyPrim.empty() ? kNoIndex : robot.bodyPrim[0]);
        agent.set_kind("robot");
        const SignalBus& bus = scheduler->bus();
        if (!bodyPose.empty() && bodyPose[0].valid()) {
            const std::span<const double> pose = bus.readN(bodyPose[0]);
            agent.set_x(pose[0] + region[0]);  // world ENU (the WorldState's frame)
            agent.set_y(pose[1] + region[1]);
            const SignalId twist = bus.find("body/" + robot.world.bodies.names[0] + "/twist");
            if (twist.valid()) {
                const std::span<const double> v = bus.readN(twist);
                agent.set_speed_m_per_s(std::hypot(v[0], v[1]));
            }
        }
        agent.set_status("focus");
        agent.set_focus(true);
        if (tileSelector) {
            // Full physics around it: where its collision tiles are loaded.
            agent.set_active_radius_m(
                active().streamingCollisionRadius.value_or(kDefaultCollisionRadiusM));
        }
        v1::WorldState::FloatingOrigin& origin = *world.mutable_floating_origin();
        origin.set_anchor(robot.name);
        origin.set_recenter_threshold_m(kRebaseDistanceM);
        origin.set_x(region[0]);
        origin.set_y(region[1]);
        origin.set_z(region[2]);
        origin.set_recenters(robot.world.region.rebaseCount);
        origin.mutable_last_recenter()->set_ns(lastRebase.ns);
        return envelope;
    }

    /// The streamed tiles for the World workspace, on the grid of the finest tiles: loaded
    /// collision tiles are level of detail 0 (physics and sensors), loaded tiles without
    /// colliders level 2 (render only), where no finer loaded tile covers them.
    void streamedTiles(v1::WorldState& world) const {
        if (!tileSelector) {
            return;
        }
        const std::vector<streaming::TileInfo>& tiles = tileSelector->tiles();
        double cell = 0.0;
        for (const streaming::TileInfo& t : tiles) {
            const double size = t.boundsMax[0] - t.boundsMin[0];
            cell = cell == 0.0 ? size : std::min(cell, size);
        }
        if (!(cell > 0.0)) {
            return;
        }
        world.set_tile_size_m(cell);
        world.set_tiles_generation(tilesGeneration);
        std::map<std::pair<std::int64_t, std::int64_t>, std::pair<std::int32_t, std::uint8_t>>
            cells;
        for (const auto& [id, content] : tileContent) {
            (void)content;
            const streaming::TileInfo& t = tiles.at(id);
            const std::uint8_t lod = t.hasCollision ? 0 : 2;
            const auto x0 = static_cast<std::int64_t>(std::floor(t.boundsMin[0] / cell + 0.5));
            const auto y0 = static_cast<std::int64_t>(std::floor(t.boundsMin[1] / cell + 0.5));
            const auto x1 = static_cast<std::int64_t>(std::floor(t.boundsMax[0] / cell + 0.5));
            const auto y1 = static_cast<std::int64_t>(std::floor(t.boundsMax[1] / cell + 0.5));
            for (std::int64_t x = x0; x < x1; ++x) {
                for (std::int64_t y = y0; y < y1; ++y) {
                    auto [it, added] = cells.try_emplace({x, y}, t.level, lod);
                    if (!added && t.level > it->second.first) {
                        it->second = {t.level, lod};  // the finer tile is what is shown
                    }
                }
            }
        }
        std::string lods;
        for (const auto& [index, value] : cells) {
            world.add_tile_x(static_cast<std::int32_t>(index.first));
            world.add_tile_y(static_cast<std::int32_t>(index.second));
            lods.push_back(static_cast<char>(value.second));
        }
        world.set_tile_lod(lods);
        v1::WorldState::Streaming& streaming = *world.mutable_streaming();
        streaming.set_host_cache_bytes(tileSelector->loadedBytes());
        streaming.set_load_queue(static_cast<std::uint32_t>(tileLoads.size() + tileQueue.size()));
    }

    /// The values of the prim's schemas of ours, numbers editable (SetAttribute).
    void schemaAttributes(std::uint32_t prim, v1::PrimAttributes& item) const {
        const std::uint32_t world = worldPrimOf(prim);
        if (world == kNoIndex) {
            return;
        }
        for (const SchemaInstance& s : robot.world.schemaInstances) {
            if (s.prim.index != world) {
                continue;
            }
            const auto add = [&](const std::string& name) -> v1::Attribute& {
                v1::Attribute& a = *item.add_attributes();
                a.set_name(name);
                a.set_authored(true);
                if (auto info = stage::schemaAttribute(s.schema, s.instance, name)) {
                    a.set_doc(info->doc);
                    if (!info->unit.empty()) {
                        if (const UnitParse unit = parseUnit(info->unit); unit.unit) {
                            setUnit(*a.mutable_unit(), *unit.unit);
                            a.mutable_unit()->set_display(displayUnit(info->unit));
                        }
                    }
                }
                return a;
            };
            for (const auto& [name, value] : s.reals) {
                v1::Attribute& a = add(name);
                a.set_real(value);
                a.set_editable(true);
            }
            for (const auto& [name, value] : s.integers) {
                add(name).set_integer(value);
            }
            for (const auto& [name, value] : s.tokens) {
                add(name).set_token(value);
            }
            for (const auto& [name, value] : s.assetPaths) {
                add(name).set_asset(value);
            }
            for (const auto& [name, values] : s.realArrays) {
                v1::Attribute& a = add(name);
                for (const double v : values) {
                    a.mutable_reals()->add_v(v);
                }
            }
            for (const auto& [name, targets] : s.relationships) {
                v1::Attribute& a = add(name);
                for (const PrimId target : targets) {
                    if (const std::uint32_t p = scenePrimOf(target); p != kNoIndex) {
                        a.mutable_targets()->add_prims(p);
                    }
                }
            }
        }
    }

    void attributes(const v1::GetAttributes& request, v1::PrimAttributesList& out) const {
        const World& world = robot.world;
        for (const std::uint32_t prim : request.prims()) {
            v1::PrimAttributes& item = *out.add_items();
            item.set_prim(prim);
            for (std::size_t b = 0; b < robot.bodyPrim.size(); ++b) {
                if (robot.bodyPrim[b] != prim) {
                    continue;
                }
                v1::Attribute& mass = *item.add_attributes();
                mass.set_name("physics:mass");
                mass.set_real(world.bodies.mass[b]);
                setUnit(*mass.mutable_unit(), units::kg);
                mass.set_authored(true);
                mass.set_editable(true);
                v1::Attribute& inertia = *item.add_attributes();
                inertia.set_name("physics:diagonalInertia");
                for (std::size_t i = 0; i < 3; ++i) {
                    inertia.mutable_reals()->add_v(world.bodies.inertiaLocal[b][i]);
                }
                setUnit(*inertia.mutable_unit(), units::kg_m2);
                inertia.set_authored(true);
            }
            schemaAttributes(prim, item);
            for (std::size_t j = 0; j < robot.jointPrim.size(); ++j) {
                if (robot.jointPrim[j] != prim) {
                    continue;
                }
                v1::Attribute& name = *item.add_attributes();
                name.set_name("joint:name");
                name.set_token(world.joints.names[j]);
                v1::Attribute& actuated = *item.add_attributes();
                actuated.set_name("joint:actuated");
                actuated.set_boolean(world.joints.actuated[j]);
                if (world.joints.effortLimit[j] > 0.0) {
                    v1::Attribute& effort = *item.add_attributes();
                    effort.set_name("joint:effortLimit");
                    effort.set_real(world.joints.effortLimit[j]);
                    setUnit(*effort.mutable_unit(),
                            world.joints.type[j] == JointType::Prismatic ? units::N : units::N_m);
                }
            }
        }
    }

    // --- output --------------------------------------------------------------------------------

    /// Signal blocks and the scene of the current time to every viewer.
    void emit() {
        for (auto& session : sessions) {
            if (!session->greeted) {
                continue;
            }
            if (session->congested()) {
                // Skip this step's delta but keep the sequence: every delta carries the absolute
                // pose of every moving prim, so the next one replaces it. Breaking the sequence
                // would make the viewer ask for a snapshot, which in a large world (thousands of
                // meshes) congests the connection again: it never caught up.
                ++session->droppedBlocks;
                if (session->hasSnapshot) {
                    ++session->droppedDeltas;
                }
                continue;
            }
            if (!session->subscriptions.empty()) {
                v1::Envelope envelope;
                v1::SignalBlock& block = *envelope.mutable_signal_block();
                block.set_catalog_generation(catalogGeneration);
                for (auto& [signal, sub] : session->subscriptions) {
                    runsFor(sub, block);
                }
                if (block.runs_size() > 0) {
                    session->send(envelope);
                }
            }
            if (session->hasSnapshot) {
                session->send(delta(*session));
            }
        }
        for (Captured& c : captured) {
            c.values.clear();
            c.count = 0;
        }
        sendSensorFrames();
    }

    void sendStats(Clock::time_point now) {
        const double wallS = static_cast<double>(nanosSince(lastStats, now)) * 1e-9;
        const std::vector<ModuleCost> costs = scheduler->takeCosts();
        v1::Envelope envelope;
        v1::ServerStats& stats = *envelope.mutable_server_stats();
        stats.mutable_sim_time()->set_ns(scheduler->now().ns);
        measuredRtf = wallS > 0.0 ? (scheduler->now() - statsSimTime).seconds() / wallS : 0.0;
        stats.set_real_time_factor(measuredRtf);
        if (!tickNs.empty()) {
            std::int64_t total = 0;
            std::int64_t most = 0;
            for (const std::int64_t ns : tickNs) {
                total += ns;
                most = std::max(most, ns);
            }
            stats.set_tick_ms(static_cast<double>(total) / 1e6 /
                              static_cast<double>(tickNs.size()));
            stats.set_tick_ms_max(static_cast<double>(most) / 1e6);
        }
        stats.set_memory_bytes(residentBytes());
        stats.set_viewers(static_cast<std::uint32_t>(sessions.size()));
        stats.set_estimated(false);
        for (const ModuleCost& cost : costs) {
            v1::ServerStats::Module& m = *stats.add_modules();
            m.set_module(cost.label);
            m.set_domain(cost.domain);
            m.set_dt_ns(cost.dt.ns);
            m.set_steps_per_second(wallS > 0.0 ? static_cast<double>(cost.steps) / wallS : 0.0);
            if (cost.steps > 0) {
                m.set_cpu_ms_per_step(static_cast<double>(cost.totalNs) / 1e6 /
                                      static_cast<double>(cost.steps));
                m.set_cpu_ms_per_step_max(static_cast<double>(cost.maxNs) / 1e6);
            }
        }
        for (auto& session : sessions) {
            if (!session->greeted) {
                continue;
            }
            stats.set_bytes_per_second_out(
                wallS > 0.0 ? static_cast<double>(session->bytesOut) / wallS : 0.0);
            session->bytesOut = 0;
            session->send(envelope);
        }
        tickNs.clear();
        statsSimTime = scheduler->now();
        lastStats = now;
    }

    // --- I/O -----------------------------------------------------------------------------------

    void accept() {
        for (;;) {
            std::error_code ec;
            std::optional<proto::Connection> connection =
                listener.accept(ec, proto::kDefaultMaxFrameSize, tls);
            if (!connection) {
                return;
            }
            auto session = std::make_unique<Session>();
            session->connection = std::move(*connection);
            sessions.push_back(std::move(session));
        }
    }

    void serve(Session& session) {
        std::error_code ec;
        const proto::IoStatus received = session.connection.receive(ec);
        for (;;) {
            const proto::FrameReader::Result frame = session.connection.next();
            if (frame.status == proto::FrameReader::Status::Error) {
                session.connection.close();
                return;
            }
            if (frame.status != proto::FrameReader::Status::Frame) {
                break;
            }
            v1::Envelope envelope;
            if (!proto::parsePayload(frame.payload, envelope)) {
                continue;  // a malformed message must not take the server down
            }
            if (!session.greeted) {
                if (!handshake(session, envelope)) {
                    (void)session.connection.flush(ec);
                    session.connection.close();
                    return;
                }
            } else {
                dispatch(session, envelope);
            }
        }
        if (received == proto::IoStatus::Closed || received == proto::IoStatus::Error) {
            session.connection.close();
        }
    }
};

namespace {

/// The simulation_interfaces services (roadmap 0.2-F4) on the server's run state. Runs on the
/// server thread, from Bridge::handleRequests() in runOnce().
///
/// Entities are named by scene path: each top-level model with a body of its own (the robot),
/// and each child of a fixed model (the scenery: "/warehouse_small/pallet"), plus the ground.
/// A robot given as text (spawn_entity's resource_string) written to a file the loaders read:
/// its extension from its content (URDF, SDFormat, MJCF or USD text).
Expected<std::string, std::string> writeResource(const std::string& text) {
    std::string extension;
    if (text.starts_with("#usda")) {
        extension = ".usda";
    } else {
        tinyxml2::XMLDocument document;
        if (document.Parse(text.c_str(), text.size()) != tinyxml2::XML_SUCCESS ||
            document.RootElement() == nullptr) {
            return Unexpected{std::string("the resource string is neither XML nor USD text")};
        }
        const std::string_view root = document.RootElement()->Name();
        extension = root == "robot"    ? ".urdf"
                    : root == "sdf"    ? ".sdf"
                    : root == "mujoco" ? ".xml"
                                       : "";
        if (extension.empty()) {
            return Unexpected{"<" + std::string(root) +
                              "> is not URDF, SDFormat or MJCF (xacro: run xacro first)"};
        }
    }
    static int written = 0;
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / ("lagrange-spawn-" + std::to_string(::getpid()) +
                                                  "-" + std::to_string(++written) + extension);
    std::ofstream out(path);
    out << text;
    if (!out) {
        return Unexpected{"cannot write " + path.string()};
    }
    return path.string();
}

class SimulationControl final : public ros2::SimulationHandler {
public:
    explicit SimulationControl(Server::Impl& server) : s_(server) {}

    ros2::SimState simulationState() override {
        if (s_.state == v1::RUN_STATE_RUNNING) {
            return ros2::SimState::Playing;
        }
        // A run that failed cannot go on until it is reset: like a stopped one.
        return s_.stopped || s_.state == v1::RUN_STATE_ERROR ? ros2::SimState::Stopped
                                                             : ros2::SimState::Paused;
    }

    ros2::SimOutcome setSimulationState(ros2::SimState target) override {
        using ros2::SimResult;
        using ros2::SimState;
        const SimState current = simulationState();
        if (target == current && target != SimState::Quitting) {
            return {SimResult::AlreadyInTargetState, ""};
        }
        switch (target) {
            case SimState::Playing:
                if (s_.state == v1::RUN_STATE_ERROR) {
                    return {SimResult::IncorrectState,
                            "the run failed (" + s_.error + "); reset or stop it first"};
                }
                s_.state = v1::RUN_STATE_RUNNING;
                s_.stopped = false;
                s_.lastWall = Clock::now();
                break;
            case SimState::Paused:
                if (current == SimState::Stopped) {
                    return {SimResult::IncorrectTransition, "a stopped simulation cannot pause"};
                }
                s_.state = v1::RUN_STATE_PAUSED;
                break;
            case SimState::Stopped:
                s_.state = v1::RUN_STATE_PAUSED;
                if (Status status = s_.restart(); !status.ok()) {
                    return {SimResult::Failed, status.message};
                }
                s_.state = v1::RUN_STATE_PAUSED;
                s_.stopped = true;
                s_.resyncViewers();
                break;
            case SimState::Quitting:
                s_.quitRequested = true;
                s_.log(v1::LOG_LEVEL_INFO, "server", "quitting: asked over simulation_interfaces");
                break;
        }
        s_.broadcast(s_.statusEnvelope());
        return {};
    }

    ros2::SimOutcome step(std::uint64_t ticks) override {
        if (s_.state != v1::RUN_STATE_PAUSED) {
            return {ros2::SimResult::Failed, s_.state == v1::RUN_STATE_ERROR
                                                 ? "the run failed: " + s_.error
                                                 : "pause the simulation before stepping"};
        }
        s_.stopped = false;
        if (Status status = s_.stepTicks(ticks); !status.ok()) {
            return {ros2::SimResult::Failed, status.message};
        }
        s_.broadcast(s_.statusEnvelope());
        return {};
    }

    ros2::SimOutcome reset() override {
        if (Status status = s_.restart(); !status.ok()) {
            return {ros2::SimResult::Failed, status.message};
        }
        s_.resyncViewers();
        s_.broadcast(s_.statusEnvelope());
        return {};
    }

    std::vector<ros2::EntityDescription> entities() override {
        std::vector<ros2::EntityDescription> out;
        const std::vector<ScenePrim>& prims = s_.robot.scene.prims;
        for (std::size_t p = 0; p < prims.size(); ++p) {
            if (const auto entity = describe(p)) {
                out.push_back(*entity);
            }
        }
        return out;
    }

    ros2::SpawnOutcome spawn(const ros2::SpawnRequest& request) override {
        using ros2::SimResult;
        if (!request.entityNamespace.empty()) {
            return {{SimResult::NamespaceInvalid,
                     "namespaces are not supported: the first robot uses the root topics, the "
                     "others are passive"},
                    ""};
        }
        if (!request.frame.empty() && request.frame != "world") {
            return {{SimResult::InvalidPose,
                     "poses are in the frame 'world' (the frame of /odom's origin), not " +
                         request.frame},
                    ""};
        }
        Pose at = request.pose;
        const std::array<double, 4>& q = at.orientation;
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!std::isfinite(norm) ||
            !std::ranges::all_of(at.position, [](double v) { return std::isfinite(v); })) {
            return {{SimResult::InvalidPose, "the pose is not finite"}, ""};
        }
        if (norm < 1e-9) {
            at.orientation = {1.0, 0.0, 0.0, 0.0};  // an unset quaternion: no rotation
        } else {
            for (double& c : at.orientation) {
                c /= norm;
            }
        }
        for (std::size_t i = 0; i < 3; ++i) {
            at.position.at(i) += s_.rosOrigin.at(i);  // the ROS world frame is the first region
        }
        std::string path;
        bool temporary = false;
        if (!request.uri.empty()) {
            auto resolved =
                meshes::resolveMeshUri(request.uri, std::filesystem::current_path().string());
            if (!resolved) {
                return {{SimResult::ResourceParseError, resolved.error()}, ""};
            }
            path = *resolved;
        } else if (!request.resource.empty()) {
            auto written = writeResource(request.resource);
            if (!written) {
                return {{SimResult::UnsupportedFormat, written.error()}, ""};
            }
            path = *written;
            temporary = true;
        } else {
            return {{SimResult::NoResource, "give the robot file as uri or resource_string"}, ""};
        }
        return s_.spawn(path, request.name, request.allowRenaming || request.name.empty(), at,
                        temporary);
    }

    ros2::SimOutcome remove(const std::string& name) override { return s_.remove(name); }

    ros2::SimOutcome removeSpawned() override {
        while (!s_.spawned.empty()) {
            const ros2::SimOutcome outcome = s_.remove(s_.spawned.back().name);
            if (outcome.result != ros2::SimResult::Ok) {
                return outcome;
            }
        }
        return {};
    }

    std::optional<ros2::EntityMotion> entityMotion(const std::string& name) override {
        const std::vector<ScenePrim>& prims = s_.robot.scene.prims;
        for (std::size_t p = 0; p < prims.size(); ++p) {
            if (prims[p].path != name || !describe(p)) {
                continue;
            }
            ros2::EntityMotion motion;
            const ScenePrim& prim = prims[p];
            if (isPerson(prim.name)) {  // where the pelvis is
                const SignalId id =
                    s_.scheduler->bus().find("body/" + humans::segmentBody(prim.name, 0) + "/pose");
                if (id.valid()) {
                    const std::span<const double> v = s_.scheduler->bus().readN(id);
                    motion.pose = Pose{{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
                }
                return motion;
            }
            if (prim.body == kNoIndex) {
                motion.pose = prim.offset;  // fixed: the offset is its world pose
                return motion;
            }
            const SignalBus& bus = s_.scheduler->bus();
            const std::string& body = s_.robot.world.bodies.names[prim.body];
            const SignalId poseId = s_.bodyPose[prim.body];
            if (!poseId.valid()) {
                motion.pose = s_.robot.bodyRestPose[prim.body];
                return motion;
            }
            const std::span<const double> pose = bus.readN(poseId);
            motion.pose = Pose{{pose[0], pose[1], pose[2]}, {pose[3], pose[4], pose[5], pose[6]}};
            if (const SignalId twist = bus.find("body/" + body + "/twist"); twist.valid()) {
                const std::span<const double> v = bus.readN(twist);
                std::copy(v.begin(), v.end(), motion.twist.begin());
            }
            if (const SignalId accel = bus.find("body/" + body + "/acceleration"); accel.valid()) {
                // The bus has the proper acceleration in the body frame; ROS wants the
                // coordinate acceleration in the world: rotate, then add gravity back.
                const std::span<const double> a = bus.readN(accel);
                const std::array<double, 3> linear =
                    rotate(motion.pose.orientation, {a[0], a[1], a[2]});
                const std::array<double, 3> angular =
                    rotate(motion.pose.orientation, {a[3], a[4], a[5]});
                const std::array<double, 3>& g = s_.robot.world.gravityLocal;
                motion.acceleration = {linear[0] + g[0], linear[1] + g[1], linear[2] + g[2],
                                       angular[0],       angular[1],       angular[2]};
            }
            return motion;
        }
        return std::nullopt;
    }

private:
    /// Whether a top prim is a person: HumanAPI_1 sits on a world prim of its name.
    [[nodiscard]] bool isPerson(const std::string& name) const {
        const World& world = s_.robot.world;
        return std::ranges::any_of(world.schemaInstances, [&](const SchemaInstance& instance) {
            return instance.schema == "HumanAPI_1" &&
                   world.prims.instanceNames.at(instance.prim.index) == name;
        });
    }

    /// The entity prim `p` stands for, if it is one.
    std::optional<ros2::EntityDescription> describe(std::size_t p) const {
        const std::vector<ScenePrim>& prims = s_.robot.scene.prims;
        const ScenePrim& prim = prims[p];
        if (prim.parent == kNoIndex) {
            if (isPerson(prim.name)) {
                return ros2::EntityDescription{prim.path, ros2::EntityCategory::Human,
                                               "person " + prim.name};
            }
            if (prim.body != kNoIndex) {
                const bool driven = !s_.robot.actuatedJoints.empty();
                return ros2::EntityDescription{
                    prim.path,
                    driven ? ros2::EntityCategory::Robot : ros2::EntityCategory::DynamicObject,
                    driven ? "robot " + prim.name + " from " + s_.robot.sourcePath
                           : "free body " + prim.name};
            }
            if (prim.mesh != kNoIndex) {  // a fixed top-level shape: the ground
                return ros2::EntityDescription{prim.path, ros2::EntityCategory::StaticObject,
                                               "fixed " + prim.name};
            }
            return std::nullopt;  // a fixed model: its children are the entities
        }
        // A link of a model bolted to the world: a static body, or a moving one (a door).
        const ScenePrim& parent = prims[prim.parent];
        if (parent.parent == kNoIndex && parent.body == kNoIndex && prim.typeName == "Xform") {
            const bool moves = prim.body != kNoIndex && s_.bodyPose[prim.body].valid();
            return ros2::EntityDescription{
                prim.path,
                moves ? ros2::EntityCategory::DynamicObject : ros2::EntityCategory::StaticObject,
                (moves ? "moving part " : "fixed scenery ") + prim.name + " of " + parent.name};
        }
        return std::nullopt;
    }

    Server::Impl& s_;
};

}  // namespace

Server::Server(ServerOptions options) : impl_(std::make_unique<Impl>()) {
    impl_->options = std::move(options);
    impl_->rtfTarget = impl_->options.realTimeFactor;
    impl_->state = impl_->options.startPaused ? v1::RUN_STATE_PAUSED : v1::RUN_STATE_RUNNING;
}

Server::~Server() {
    services::setLogSink({});
    if (impl_->scheduler) {
        impl_->stopRecording("the server stopped");
    }
    for (const Impl::Spawned& robot : impl_->spawned) {
        if (robot.temporary) {
            std::error_code ignored;
            std::filesystem::remove(robot.path, ignored);
        }
    }
}

Status Server::load() {
    Impl& s = *impl_;
    if (s.scheduler) {
        return {};
    }
    // What modules and plugins log goes to the viewer's console with the server's own lines.
    services::setLogSink(
        [&s](services::LogLevel level, std::string_view source, std::string_view text) {
            static constexpr v1::LogLevel kLevels[] = {v1::LOG_LEVEL_DEBUG, v1::LOG_LEVEL_INFO,
                                                       v1::LOG_LEVEL_WARN, v1::LOG_LEVEL_ERROR};
            s.log(kLevels[static_cast<int>(level)], std::string(source), std::string(text));
        });
    // Plugins (RFC 0008): loaded once, at start, from what the person starting the server named.
    auto loaded = loadPlugins(s.options.plugins);
    if (!loaded) {
        return Status::error(StatusCode::InvalidConfig, loaded.error());
    }
    s.plugins = std::move(*loaded);
    for (const PluginModule& module : s.plugins) {
        const std::string role =
            !isBuiltInDomain(module.domain)
                ? "its own domain; --domain " + module.domain + "=off turns it off"
            : module.substitute
                ? "a substitute: domains." + module.domain + ".substitute: " + module.name
            : module.domain == "rigid_body"
                ? "a rigid body backend: --physics " + module.name
                : "not used: a built-in domain takes plugin substitutes and, for rigid_body, "
                  "backends";
        s.log(v1::LOG_LEVEL_INFO, "plugins",
              module.domain + "/" + module.name + " from " + module.path + " (" + role + ")" +
                  (module.deterministic ? "" : "; not deterministic"));
    }
    for (const std::string& domain : s.options.substituted) {
        if (!isBuiltInDomain(domain) &&
            std::ranges::none_of(s.plugins, [&domain](const PluginModule& module) {
                return module.domain == domain;
            })) {
            s.log(v1::LOG_LEVEL_WARN, "plugins",
                  "--domain " + domain + "=off: no module has that domain");
        }
    }
    if (!s.options.replayPath.empty()) {
        if (Status status = s.loadReplay(); !status.ok()) {
            return status;
        }
        if (Status status = s.build(); !status.ok()) {
            s.scheduler.reset();
            return status;
        }
        if (!s.options.profileSchemaPath.empty()) {
            s.schemaText = readFile(s.options.profileSchemaPath);
        }
        return {};
    }
    // A scenario: its roads, and where the ego starts.
    std::optional<Model> scenarioRoads;
    if (!s.options.scenarioPath.empty()) {
        auto scenario = openscenario::read(s.options.scenarioPath);
        if (!scenario) {
            return Status::error(StatusCode::InvalidConfig, scenario.error());
        }
        s.scenario = std::move(*scenario);
        if (!s.scenario->roadNetwork.empty()) {
            auto network = opendrive::read(s.scenario->roadNetwork);
            if (!network) {
                return Status::error(StatusCode::InvalidConfig, network.error());
            }
            auto roads = opendrive::toModel(*network);
            if (!roads) {
                return Status::error(StatusCode::InvalidConfig, roads.error());
            }
            const RoadLanes lanes = roadLanesOf(WorldView(roads->world));
            for (const openscenario::Action& a : s.scenario->init) {
                if (a.entity == s.options.scenarioEgo &&
                    a.kind == openscenario::Action::Kind::Teleport) {
                    if (const auto pose = openscenario::poseOf(a.position, lanes)) {
                        s.options.spawn.position = {(*pose)[0], (*pose)[1], (*pose)[2]};
                        s.options.spawn.orientation = {std::cos((*pose)[3] / 2.0), 0.0, 0.0,
                                                       std::sin((*pose)[3] / 2.0)};
                    }
                }
            }
            if (s.options.worldPath.empty()) {
                scenarioRoads = std::move(*roads);
            }
        }
        s.options.demoDrive = false;  // the ego waits for the system under test
    }
    s.scenarioRoads = std::move(scenarioRoads);
    if (!s.options.ros && !s.options.ros2ControlParams.empty()) {
        return Status::error(StatusCode::InvalidConfig,
                             "--ros2-control needs the ROS 2 bridge: drop --no-ros");
    }
    if (!s.options.ros && !s.options.rosTopicsPath.empty()) {
        return Status::error(StatusCode::InvalidConfig,
                             "--ros-topics needs the ROS 2 bridge: drop --no-ros");
    }
    auto composed = s.composeModel(true);
    if (!composed) {
        return composed.error();
    }
    s.robot = std::move(*composed);
    if (s.options.ros) {
        ros2::BridgeSettings settings;
        settings.baseLink = s.baseLink();
        settings.robotDescription = s.robotDescription();
        settings.stampedCmdVel = s.options.stampedCmdVel;
        settings.topicLinks = s.options.linkTopics;
        if (!s.options.rosTopicsPath.empty()) {
            auto maps = ros2::readTopicMaps(s.options.rosTopicsPath);
            if (!maps) {
                return Status::error(StatusCode::InvalidConfig, maps.error());
            }
            settings.signalTopics = std::move(*maps);
        }
        settings.controllersOwnState = !s.options.ros2ControlParams.empty();
        settings.ackermannCmd = !WorldView(s.robot.world).instancesOf("VehicleAPI_1").empty();
        s.bridge = std::make_unique<ros2::Bridge>(settings);
        if (!s.options.ros2ControlParams.empty()) {
            if (importers::formatOf(s.options.robotPath) != importers::Format::Urdf) {
                return Status::error(StatusCode::InvalidConfig,
                                     "--ros2-control needs a URDF robot: its <ros2_control> block "
                                     "names the hardware");
            }
            const std::string urdf = readFile(s.options.robotPath);
            auto joints = ros2_control::controlJoints(urdf);
            if (!joints) {
                return Status::error(StatusCode::InvalidConfig, joints.error());
            }
            auto host = ros2_control::ControllerHost::start(urdf, s.options.ros2ControlParams);
            if (!host) {
                return Status::error(StatusCode::InvalidConfig, host.error());
            }
            s.controlJoints = std::move(*joints);
            s.controllers = std::move(*host);
        }
        s.simulation = std::make_unique<SimulationControl>(s);
        s.bridge->serveSimulation(*s.simulation);
    }
    s.shift = recentre(s.robot);
    s.rosOrigin = s.shift;
    if (s.options.robotPath.empty()) {
        s.log(v1::LOG_LEVEL_INFO, "stage",
              "no robot: spawn one with the viewer's Import robot, `lagrange spawn FILE` or "
              "the spawn_entity service");
    }
    if (Status status = s.startStreaming(); !status.ok()) {
        return status;
    }
    if (Status status = s.placeLidar(); !status.ok()) {
        return status;
    }
    s.placeCameras();
    if (Status status = s.build(); !status.ok()) {
        s.scheduler.reset();
        return status;
    }
    if (!s.options.profileSchemaPath.empty()) {
        s.schemaText = readFile(s.options.profileSchemaPath);
    }
    return {};
}

std::string Server::wiringDot() const {
    return impl_->scheduler ? impl_->scheduler->dumpGraphDot() : std::string{};
}

ProfileCheck Server::validateProfile(const std::string& yaml) {
    return impl_->check(yaml);
}

Status Server::applyProfile(const std::string& yaml) {
    return impl_->apply(yaml, true);
}

Status Server::stepUntil(SimTime t) {
    if (!impl_->scheduler) {
        return Status::error(StatusCode::Internal, "stepUntil before load()");
    }
    // Tick by tick, so the floating origin follows the robot as in a served run.
    while (impl_->scheduler->now() < t) {
        if (Status status = impl_->scheduler->stepTick(); !status.ok()) {
            return status;
        }
        if (Status status = impl_->worldUpdate(true); !status.ok()) {
            return status;
        }
    }
    return {};
}

std::vector<Server::SignalEntry> Server::signals() const {
    std::vector<SignalEntry> out;
    if (!impl_->scheduler) {
        return out;
    }
    const SignalBus& bus = impl_->scheduler->bus();
    for (std::size_t i = 0; i < bus.signalCount(); ++i) {
        const SignalSpec& spec = bus.specAt(i);
        out.push_back({spec.name, label(spec.unit),
                       static_cast<std::uint32_t>(componentCount(spec.type, spec.arrayLength))});
    }
    return out;
}

std::vector<double> Server::read(const std::string& name) const {
    if (!impl_->scheduler) {
        return {};
    }
    const SignalBus& bus = impl_->scheduler->bus();
    const SignalId id = bus.find(name);
    if (!id.valid()) {
        return {};
    }
    const std::span<const double> values = bus.readN(id);
    return {values.begin(), values.end()};
}

Expected<std::vector<std::byte>, Status> Server::saveState() const {
    if (!impl_->scheduler) {
        return Unexpected{Status::error(StatusCode::Internal, "saveState() before load()")};
    }
    StateWriter writer;
    if (Status status = impl_->scheduler->saveState(writer); !status.ok()) {
        return Unexpected{status};
    }
    const std::span<const std::byte> bytes = writer.bytes();
    return std::vector<std::byte>(bytes.begin(), bytes.end());
}

Status Server::restoreState(std::span<const std::byte> state) {
    if (!impl_->scheduler) {
        return Status::error(StatusCode::Internal, "restoreState() before load()");
    }
    StateReader reader(state);
    return impl_->scheduler->restoreState(reader);
}

Expected<std::string, Status> Server::spawnRobot(const std::string& path, const std::string& name,
                                                 const Pose& spawn) {
    if (!impl_->scheduler) {
        return Unexpected{Status::error(StatusCode::Internal, "spawnRobot() before load()")};
    }
    const ros2::SpawnOutcome outcome = impl_->spawn(path, name, false, spawn, false);
    if (outcome.outcome.result != ros2::SimResult::Ok) {
        return Unexpected{Status::error(StatusCode::InvalidConfig, outcome.outcome.message)};
    }
    return outcome.entityName;
}

Status Server::removeRobot(const std::string& name) {
    if (!impl_->scheduler) {
        return Status::error(StatusCode::Internal, "removeRobot() before load()");
    }
    const ros2::SimOutcome outcome = impl_->remove(name);
    return outcome.result == ros2::SimResult::Ok
               ? Status{}
               : Status::error(StatusCode::InvalidConfig, outcome.message);
}

SimTime Server::now() const {
    return impl_->scheduler ? impl_->scheduler->now() : SimTime{};
}

const World& Server::world() const {
    return impl_->robot.world;
}

Expected<std::uint64_t, Status> Server::stateHash() const {
    if (!impl_->scheduler) {
        return Unexpected{Status::error(StatusCode::Internal, "stateHash before load()")};
    }
    return impl_->scheduler->stateHash();
}

Status Server::start() {
    Impl& s = *impl_;
    if (Status status = load(); !status.ok()) {
        return status;
    }
    const std::array<double, 3>& shift = s.shift;
    if (!s.options.recordPath.empty()) {
        if (Status status = s.startRecording(s.options.recordPath); !status.ok()) {
            return status;
        }
    }
    const proto::Endpoint& endpoint = s.options.endpoint;
    if (endpoint.kind == proto::Endpoint::Kind::Tcp && s.options.authToken.empty() &&
        !proto::isLoopback(endpoint)) {
        return Status::error(StatusCode::InvalidConfig,
                             "listening on " + proto::toString(endpoint) +
                                 " is open beyond this machine: give --auth-token-file FILE "
                                 "(and preferably a tls:// endpoint with --tls-cert and "
                                 "--tls-key)");
    }
    if (endpoint.tls) {
        std::string reason;
        s.tls = proto::TlsContext::server(s.options.tlsCertificate, s.options.tlsKey, reason);
        if (!s.tls) {
            return Status::error(StatusCode::InvalidConfig,
                                 "a tls:// endpoint needs --tls-cert and --tls-key: " + reason);
        }
    }
    std::error_code ec;
    s.listener = proto::Listener::listen(s.options.endpoint, ec);
    if (!s.listener.valid()) {
        return Status::error(
            StatusCode::InvalidConfig,
            "cannot listen on " + proto::toString(s.options.endpoint) + ": " + ec.message());
    }
    const World& world = s.robot.world;
    s.log(v1::LOG_LEVEL_INFO, "stage",
          "loaded " + (s.robot.sourcePath.empty() ? "no robot" : s.robot.sourcePath) + " · " +
              std::to_string(world.bodies.names.size()) + " bodies, " +
              std::to_string(world.joints.names.size()) + " joints, " +
              std::to_string(s.robot.scene.prims.size()) + " prims");
    if (s.robot.world.region.rebaseCount > 0) {
        char text[160];
        std::snprintf(text, sizeof text,
                      "floating origin: the region-local frame starts at (%.0f, %.0f, %.0f) m in "
                      "world ENU; poses, /odom, /tf and the viewer are relative to it",
                      shift[0], shift[1], shift[2]);
        s.log(v1::LOG_LEVEL_INFO, "stage", text);
    }
    for (const std::string& note : s.robot.notes) {
        s.log(v1::LOG_LEVEL_WARN, "import", note);
    }
    s.log(v1::LOG_LEVEL_INFO, "rigid_body",
          "MuJoCo 3.12 · 1 ms · elliptic friction cone · implicitfast");
    if (s.controllers) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "ros2_control: /controller_manager at " +
                  std::to_string(s.controllers->updateRateHz()) + " Hz drives " +
                  std::to_string(s.controlJoints.size()) +
                  " joints through lagrange_ros2_control/LagrangeSystem; spawn its controllers");
    } else if (s.robot.actuatedJoints.size() == 2) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "diff drive on " + s.robot.actuatedJoints[0] + " and " + s.robot.actuatedJoints[1] +
                  (!s.options.demoDrive ? "; it stands still until /cmd_vel arrives"
                   : s.bridge           ? "; a figure-eight demo drives until /cmd_vel arrives"
                                        : "; a figure-eight demo drives"));
    } else if (const auto swerve = s.swerveSettings()) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "swerve drive on " + std::to_string(swerve->modules.size()) +
                  " modules: /cmd_vel's forward, sideways (linear.y) and turn rates" +
                  (!s.options.demoDrive ? "; it stands still until /cmd_vel arrives"
                   : s.bridge           ? "; a square and a turn on the spot until /cmd_vel arrives"
                                        : "; a square and a turn on the spot"));
    } else if (s.hasVehicles()) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              std::string("car: the driver turns /cmd_vel (speed, turn rate) or /ackermann_cmd "
                          "(steering angle, speed) into throttle, brake and steering") +
                  (s.options.demoDrive ? "; until the first command it drives a 5 m circle round "
                                         "the origin"
                                       : "; it stands still until /cmd_vel arrives"));
    } else if (s.options.ardupilotPort) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "ArduPilot SITL flies it through its JSON interface on UDP port " +
                  std::to_string(*s.options.ardupilotPort) + " (lockstep once it connects)");
    } else if (s.options.px4Port) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "PX4 SITL flies it through the MAVLink bridge on TCP port " +
                  std::to_string(*s.options.px4Port) + " (lockstep once PX4 connects)");
    } else if (MulticopterController::canFly(WorldView(s.robot.world))) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              std::string("multicopter: the built-in controller takes off after 1 s and hovers 2 m "
                          "above the start") +
                  (s.bridge ? "; /cmd_vel flies it (forward, left, up m/s; yaw rate rad/s)" : ""));
    } else if (!s.primaryPath().empty()) {
        s.log(v1::LOG_LEVEL_WARN, "controller",
              "no demo driver: it needs exactly two actuated wheel joints, the robot has " +
                  std::to_string(s.robot.actuatedJoints.size()));
    }
    for (std::size_t i = 0; i < s.robot.world.sensors.names.size(); ++i) {
        const World::Sensors& sensors = s.robot.world.sensors;
        char line[160];
        constexpr std::array<const char*, 7> kKinds{"IMU",          "lidar",     "camera", "GNSS",
                                                    "magnetometer", "barometer", "radar"};
        if (sensors.type[i] == SensorType::Custom) {
            s.log(v1::LOG_LEVEL_INFO, "sensors",
                  "custom sensor " + sensors.names[i] + " (" + sensors.kind[i] + ") on " +
                      s.robot.world.bodies.names[sensors.body[i].index] +
                      (s.plugins.empty() ? ": no plugin is loaded to implement it"
                                         : ": a plugin implements it"));
            continue;
        }
        const char* kind = kKinds.at(static_cast<std::size_t>(sensors.type[i]));
        std::snprintf(line, sizeof(line), "%s %s on %s at %.0f Hz", kind, sensors.names[i].c_str(),
                      s.robot.world.bodies.names[sensors.body[i].index].c_str(), sensors.rateHz[i]);
        s.log(v1::LOG_LEVEL_INFO, "sensors", line);
    }
    if (s.bridge) {
        s.log(v1::LOG_LEVEL_INFO, "ros2",
              "node /" + s.bridge->nodeName() +
                  " publishes /clock /tf /tf_static /odom /joint_states /robot_description and "
                  "the sensors; subscribes /cmd_vel (geometry_msgs/" +
                  (s.options.stampedCmdVel ? "TwistStamped" : "Twist") +
                  "); serves the simulation_interfaces services");
    }
    s.log(v1::LOG_LEVEL_INFO, "server", "listening on " + proto::toString(s.listener.endpoint()));
    s.lastWall = s.lastStatus = s.lastStats = Clock::now();
    return {};
}

void Server::runOnce(int timeoutMs) {
    Impl& s = *impl_;
    std::vector<pollfd> fds;
    fds.push_back({s.listener.fd(), POLLIN, 0});
    for (const auto& session : s.sessions) {
        fds.push_back({session->connection.fd(), session->connection.pollEvents(), 0});
    }
    ::poll(fds.data(), fds.size(), timeoutMs);

    s.accept();
    if (s.bridge) {
        s.bridge->handleRequests();
    }
    for (auto& session : s.sessions) {
        if (session->connection.state() == proto::Connection::State::Connected) {
            s.serve(*session);
        }
    }

    const Clock::time_point now = Clock::now();
    const std::int64_t wallNs = nanosSince(s.lastWall, now);
    s.lastWall = now;
    if (s.state == v1::RUN_STATE_RUNNING) {
        // rtf 0 means as fast as possible: the step budget alone limits each iteration.
        const std::int64_t simNs =
            s.rtfTarget > 0.0 ? static_cast<std::int64_t>(static_cast<double>(wallNs) * s.rtfTarget)
                              : kMaxStepWallNs * 1000;
        if (Status status = s.advance(simNs); !status.ok()) {
            s.fail(status);
        }
        s.emit();
    }
    if (nanosSince(s.lastStatus, now) >= kStatusPeriodNs) {
        s.lastStatus = now;
        for (auto& session : s.sessions) {
            if (session->greeted) {
                v1::Envelope status = s.statusEnvelope();
                status.mutable_status()->set_dropped_deltas(session->droppedDeltas);
                status.mutable_status()->set_dropped_signal_blocks(session->droppedBlocks);
                session->send(status);
            }
        }
    }
    if (nanosSince(s.lastFields, now) >= kFieldPeriodNs) {
        s.lastFields = now;
        for (auto& session : s.sessions) {
            if (!session->greeted || session->congested()) {
                continue;
            }
            for (const auto& [name, height] : session->fields) {
                if (auto slice = s.fieldSlice(name, height)) {
                    session->send(*slice);
                }
            }
        }
    }
    if (nanosSince(s.lastStats, now) >= kStatsPeriodNs) {
        s.sendStats(now);
        for (auto& session : s.sessions) {
            if (session->greeted && session->wantsRosGraph) {
                session->send(s.rosGraph());
            }
            if (session->greeted && session->wantsWorld) {
                session->send(s.worldState());
            }
        }
    }

    for (auto& session : s.sessions) {
        std::error_code ec;
        if (session->connection.state() == proto::Connection::State::Connected) {
            const proto::IoStatus flushed = session->connection.flush(ec);
            if (flushed == proto::IoStatus::Closed || flushed == proto::IoStatus::Error) {
                session->connection.close();
            }
        }
    }
    std::erase_if(s.sessions, [](const auto& session) {
        return session->connection.state() == proto::Connection::State::Closed;
    });
}

const proto::Endpoint& Server::endpoint() const {
    return impl_->listener.endpoint();
}

SimTime Server::simTime() const {
    return impl_->scheduler ? impl_->scheduler->now() : SimTime{};
}

bool Server::running() const {
    return impl_->state == v1::RUN_STATE_RUNNING;
}

bool Server::quitRequested() const {
    return impl_->quitRequested;
}

}  // namespace lagrange::server
