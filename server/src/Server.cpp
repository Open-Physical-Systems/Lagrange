#include "Recorder.hpp"
#include "lagrange/importers/Importers.hpp"
#include "lagrange/proto/Connection.hpp"
#include "lagrange/proto/Framing.hpp"
#include "lagrange/rigid_body/FixedRobot.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/ros2/Bridge.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/server/DiffDrive.hpp"
#include "lagrange/server/Profile.hpp"
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/Server.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"
#include "lagrange/world/LidarLayout.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <poll.h>
#include <set>
#include <sstream>
#include <string>
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
constexpr char kServerName[] = "lagrange_server 0.1.0";
constexpr std::int64_t kStatusPeriodNs = 100'000'000;  // 10 Hz, as the mock server
constexpr std::int64_t kStatsPeriodNs = 1'000'000'000;
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
constexpr std::array<DomainRow, 13> kDomains{{
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
    {"vehicle_dynamics", "ideal", "ideal"},
    {"rotor_aero", "zero", "constant"},
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

    std::vector<SignalId> bodyPose;  // pose signal per body, invalid for static bodies
    std::vector<Captured> captured;  // by signal index; empty values when nobody subscribes
    std::deque<v1::LogRecord> logs;
    std::string schemaText;
    std::array<double, 3> shift{};  // world ENU of the region-local origin at load

    Clock::time_point lastWall{Clock::now()};
    Clock::time_point lastStatus{Clock::now()};
    Clock::time_point lastStats{Clock::now()};
    SimTime statsSimTime{};
    std::vector<std::int64_t> tickNs;

    // --- simulation ----------------------------------------------------------------------

    /// The modules for `settings`, wired: the run itself (build) or a profile's preview.
    Expected<std::unique_ptr<Scheduler>, std::string> makeScheduler(const ProfileSettings& settings,
                                                                    DriveCommand** driveOut) {
        const auto off = [&settings](const char* domain) {
            return settings.substituted.contains(domain);
        };
        // A run that takes commands from ROS is not deterministic: its input comes from outside.
        // So is one with the lidar on the GPU (invariant 7: the CPU is the reference).
        auto next =
            std::make_unique<Scheduler>(SchedulerOptions{.deterministic = !bridge && !rayContext});
        if (off("rigid_body")) {
            next->add(std::make_unique<rigid_body::FixedRobot>());
        } else {
            ModuleConfigData physics;
            physics.backend = "mujoco";
            if (settings.rigidBodyDtNs) {
                physics.dt = Duration{*settings.rigidBodyDtNs};
            }
            next->add(std::make_unique<rigid_body::MujocoBackend>(), ModuleConfig(physics));
        }
        if (robot.actuatedJoints.size() == 2) {
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
        }
        ModuleConfigData sensing;
        sensing.tier =
            rayContext ? static_cast<CapabilityTier>(rayContext->tier()) : CapabilityTier::Tier0;
        for (auto& sensor : off("sensors")
                                ? sensors::createSensorSubstitutes(robot.world)
                                : sensors::createSensorModules(robot.world, rayContext)) {
            next->add(std::move(sensor), ModuleConfig(sensing));
        }
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
        return settings;
    }
    std::optional<std::int64_t> rigidBodyDtNs;  // from an applied profile
    std::optional<ProfileSettings> pending;     // applied without restart: used at the next reset

    Status build() {
        auto made = makeScheduler(active(), &driveCommand);
        if (!made) {
            return Status::error(StatusCode::InvalidConfig, made.error());
        }
        scheduler = std::move(*made);
        bodyPose.assign(robot.world.bodies.names.size(), SignalId{});
        for (std::size_t b = 0; b < bodyPose.size(); ++b) {
            bodyPose[b] = scheduler->bus().find("body/" + robot.world.bodies.names[b] + "/pose");
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
        }
        lidarSent.assign(robot.world.sensors.names.size(), SimTime{-1});
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

    /// The latest scan of each lidar to the viewers that watch it.
    void sendSensorFrames() {
        const World::Sensors& table = robot.world.sensors;
        const SignalBus& bus = scheduler->bus();
        for (std::size_t s = 0; s < table.names.size(); ++s) {
            if (table.type[s] != SensorType::Lidar) {
                continue;
            }
            const std::string& path = robot.scene.prims[robot.sensorPrim[s]].path;
            const bool watched = std::any_of(sessions.begin(), sessions.end(), [&](const auto& x) {
                return x->greeted && x->sensors.contains(path);
            });
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
    std::vector<SimTime> lidarSent;

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
        while (owedNs >= tick) {
            owedNs -= tick;
            const Clock::time_point tickStart = Clock::now();
            if (Status status = scheduler->stepTick(); !status.ok()) {
                return status;
            }
            tickNs.push_back(nanosSince(tickStart, Clock::now()));
            capture();
            record();
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
        if (prim.body == kNoIndex) {
            return prim.offset;
        }
        Pose body = robot.bodyRestPose[prim.body];
        const SignalId id = bodyPose[prim.body];
        if (id.valid()) {
            const std::span<const double> v = scheduler->bus().readN(id);
            body = {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
        }
        return compose(body, prim.offset);
    }

    v1::Envelope snapshot(Session& session) {
        session.hasSnapshot = true;
        return snapshotAt(++session.sequence);
    }

    v1::Envelope snapshotAt(std::uint64_t sequence) {
        v1::Envelope envelope;
        v1::SceneSnapshot& snap = *envelope.mutable_snapshot();
        snap.set_sequence(sequence);
        snap.mutable_sim_time()->set_ns(scheduler->now().ns);
        v1::LocalFrame& frame = *snap.mutable_frame();
        frame.set_rebase_count(robot.world.region.rebaseCount);
        for (const double v : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {
            frame.add_enu_from_ecef(v);
        }
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

    /// The scene prim a signal is about: body/<link>/... and joint/<joint>/... name it.
    std::uint32_t primOfSignal(const std::string& name) const {
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

    std::int64_t fastestSensorNs() const {
        double rate = 0.0;
        for (const double r : robot.world.sensors.rateHz) {
            rate = std::max(rate, r);
        }
        return rate > 0.0 ? static_cast<std::int64_t>(std::llround(1e9 / rate)) : 0;
    }

    [[nodiscard]] bool substituted(std::string_view domain) const {
        return options.substituted.contains(std::string(domain));
    }

    /// The stage as profiles name it (ServerOptions::stageName).
    std::string stageName() const {
        return options.stageName.empty() ? robot.sourcePath : options.stageName;
    }

    std::string profileYaml() const {
        std::string yaml =
            "# Lagrange simulation profile\nschema_version: 1\nstage: " + stageName() +
            "\n\ndomains:\n";
        for (const DomainRow& row : kDomains) {
            if (std::string_view(row.domain) == "rigid_body" && !substituted("rigid_body")) {
                char dt[48];
                std::snprintf(dt, sizeof(dt), "%.9g",
                              static_cast<double>(rigidBodyDtNs.value_or(1'000'000)) / 1e9);
                yaml +=
                    "  rigid_body: {enabled: true, backend: mujoco, dt: " + std::string(dt) + "}\n";
            } else if (std::string_view(row.domain) == "sensors" && hasSensors() &&
                       !substituted("sensors")) {
                yaml += "  sensors: {enabled: true}\n";
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
            const bool physics = std::string_view(row.domain) == "rigid_body";
            const bool sensing = std::string_view(row.domain) == "sensors" && hasSensors();
            const bool built = physics || sensing;  // a module exists for it
            const bool running = built && !substituted(row.domain);
            d.set_enabled(running);
            d.set_available(built);
            d.set_active_module(
                running ? (physics ? "mujoco" : (rayContext ? "raycast_gpu" : "raycast"))
                        : row.substitute);
            d.set_substituted(!running);
            d.set_cpu_ms_per_sim_s(cost[row.domain]);
            if (sensing) {
                d.add_fidelities(rayContext ? "raycast_gpu" : "raycast");
                d.set_dt_ns(fastestSensorNs());
                d.add_depends_on("rigid_body");
            }
            if (!built) {
                d.set_unavailable_reason("no module in this build yet; the substitute stands in");
            }
            v1::SubstituteOption& option = *d.add_substitutes();
            option.set_token(row.substitute);
            option.set_kind(row.substituteKind);
            if (physics) {
                d.add_backends("mujoco");
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
        hello.set_session_id(nextSessionId++);
        v1::Capabilities& caps = *hello.mutable_capabilities();
        caps.set_can_step(true);
        caps.set_tier(0);
        session.send(reply);
        session.greeted = true;

        session.send(profileState({}));
        session.send(catalog());
        session.send(statusEnvelope());
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
            case v1::Command::kSetRosGraph:
                session.wantsRosGraph = command.set_ros_graph().enabled();
                if (session.wantsRosGraph) {
                    session.send(rosGraph());
                }
                break;
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
            case v1::Command::kSubscribeField:
            case v1::Command::kSetOverlays:
                break;  // accepted; this server has no environment fields yet
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
    Status restart() {
        stopRecording("the run was reset");
        const bool wasRunning = state == v1::RUN_STATE_RUNNING;
        if (Status status = build(); !status.ok()) {
            fail(status);
            return status;
        }
        state = wasRunning ? v1::RUN_STATE_RUNNING : v1::RUN_STATE_PAUSED;
        error.clear();
        owedNs = 0;
        lastWall = Clock::now();
        log(v1::LOG_LEVEL_INFO, "scheduler", "reset · the run restarted at t = 0");
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
        for (const DomainRow& row : kDomains) {
            context.substitutes[row.domain] = {row.substitute, row.substituteKind};
        }
        return context;
    }

    /// Checks a profile and shows the wiring it would build (or the running one if it does not
    /// parse into settings).
    void validate(const std::string& yaml, v1::ProfileValidation& out) {
        const ProfileCheck check = checkProfile(yaml, schemaText, profileContext());
        out.set_valid(check.valid);
        for (const ProfileIssue& issue : check.issues) {
            v1::ProfileIssue& item = *out.add_issues();
            item.set_severity(issue.error ? v1::ProfileIssue::SEVERITY_ERROR
                                          : v1::ProfileIssue::SEVERITY_WARNING);
            item.set_json_pointer(issue.pointer);
            item.set_message(issue.message);
        }
        if (check.settings) {
            auto preview = makeScheduler(*check.settings, nullptr);
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

    /// checkProfile, then whether the settings wire (as validate, without the wiring preview).
    ProfileCheck check(const std::string& yaml) {
        ProfileCheck check = checkProfile(yaml, schemaText, profileContext());
        if (check.settings) {
            if (auto preview = makeScheduler(*check.settings, nullptr); !preview) {
                check.valid = false;
                check.issues.push_back({true, "/domains", preview.error()});
            }
        }
        return check;
    }

    /// Adopts a valid profile: rebuilds the run at t = 0 now, or at the next reset.
    Status apply(const std::string& yaml, bool restartNow) {
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
        pending.reset();
        if (Status status = restart(); !status.ok()) {
            // Keep the run going on the profile it had.
            options.substituted = previous.substituted;
            rigidBodyDtNs = previous.rigidBodyDtNs;
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

    /// The World workspace: the robot as the one agent, at its simulated position.
    v1::Envelope worldState() const {
        v1::Envelope envelope;
        v1::WorldState& world = *envelope.mutable_world();
        world.mutable_sim_time()->set_ns(scheduler->now().ns);
        world.mutable_geo_origin()->set_valid(false);
        v1::WorldState::Agent& agent = *world.add_agents();
        agent.set_name(robot.name);
        agent.set_prim(robot.bodyPrim.empty() ? kNoIndex : robot.bodyPrim[0]);
        agent.set_kind("robot");
        const SignalBus& bus = scheduler->bus();
        if (!bodyPose.empty() && bodyPose[0].valid()) {
            const std::span<const double> pose = bus.readN(bodyPose[0]);
            agent.set_x(pose[0]);
            agent.set_y(pose[1]);
            const SignalId twist = bus.find("body/" + robot.world.bodies.names[0] + "/twist");
            if (twist.valid()) {
                const std::span<const double> v = bus.readN(twist);
                agent.set_speed_m_per_s(std::hypot(v[0], v[1]));
            }
        }
        agent.set_status("focus");
        agent.set_focus(true);
        v1::WorldState::FloatingOrigin& origin = *world.mutable_floating_origin();
        origin.set_anchor(robot.name);
        origin.set_recenter_threshold_m(1000.0);  // huge-worlds.md §1.2 default
        return envelope;
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
                v1::Attribute& inertia = *item.add_attributes();
                inertia.set_name("physics:diagonalInertia");
                for (std::size_t i = 0; i < 3; ++i) {
                    inertia.mutable_reals()->add_v(world.bodies.inertiaLocal[b][i]);
                }
                setUnit(*inertia.mutable_unit(), units::kg_m2);
                inertia.set_authored(true);
            }
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
                // Skipping a delta breaks the sequence on purpose: the viewer asks for a snapshot.
                ++session->droppedBlocks;
                if (session->hasSnapshot) {
                    ++session->droppedDeltas;
                    ++session->sequence;
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
            std::optional<proto::Connection> connection = listener.accept(ec);
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

    std::optional<ros2::EntityMotion> entityMotion(const std::string& name) override {
        const std::vector<ScenePrim>& prims = s_.robot.scene.prims;
        for (std::size_t p = 0; p < prims.size(); ++p) {
            if (prims[p].path != name || !describe(p)) {
                continue;
            }
            ros2::EntityMotion motion;
            const ScenePrim& prim = prims[p];
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
    /// The entity prim `p` stands for, if it is one.
    std::optional<ros2::EntityDescription> describe(std::size_t p) const {
        const std::vector<ScenePrim>& prims = s_.robot.scene.prims;
        const ScenePrim& prim = prims[p];
        if (prim.parent == kNoIndex) {
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
    if (impl_->scheduler) {
        impl_->stopRecording("the server stopped");
    }
}

Status Server::load() {
    Impl& s = *impl_;
    if (s.scheduler) {
        return {};
    }
    auto robot = loadModel(s.options.robotPath, {.spawn = s.options.spawn});
    if (!robot) {
        return Status::error(StatusCode::InvalidConfig, robot.error());
    }
    s.robot = std::move(robot.value());
    if (s.options.ros) {
        ros2::BridgeSettings settings;
        settings.baseLink = s.robot.world.bodies.names.empty() ? "" : s.robot.world.bodies.names[0];
        // /robot_description is URDF; a robot from another format has none to publish.
        settings.robotDescription =
            importers::formatOf(s.options.robotPath) == importers::Format::Urdf
                ? readFile(s.options.robotPath)
                : "";
        settings.stampedCmdVel = s.options.stampedCmdVel;
        s.bridge = std::make_unique<ros2::Bridge>(settings);
        s.simulation = std::make_unique<SimulationControl>(s);
        s.bridge->serveSimulation(*s.simulation);
    }
    if (!s.options.worldPath.empty()) {
        if (auto added = addModel(s.robot, s.options.worldPath); !added) {
            return Status::error(StatusCode::InvalidConfig, added.error());
        }
    }
    s.shift = recentre(s.robot);
    if (Status status = s.placeLidar(); !status.ok()) {
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

std::string Server::wiringDot() const {
    return impl_->scheduler ? impl_->scheduler->dumpGraphDot() : std::string{};
}

ProfileCheck Server::validateProfile(const std::string& yaml) {
    return impl_->check(yaml);
}

Status Server::applyProfile(const std::string& yaml) {
    return impl_->apply(yaml, true);
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
    std::error_code ec;
    s.listener = proto::Listener::listen(s.options.endpoint, ec);
    if (!s.listener.valid()) {
        return Status::error(
            StatusCode::InvalidConfig,
            "cannot listen on " + proto::toString(s.options.endpoint) + ": " + ec.message());
    }
    const World& world = s.robot.world;
    s.log(v1::LOG_LEVEL_INFO, "stage",
          "loaded " + s.robot.sourcePath + " · " + std::to_string(world.bodies.names.size()) +
              " bodies, " + std::to_string(world.joints.names.size()) + " joints, " +
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
    if (s.robot.actuatedJoints.size() == 2) {
        s.log(v1::LOG_LEVEL_INFO, "controller",
              "diff drive on " + s.robot.actuatedJoints[0] + " and " + s.robot.actuatedJoints[1] +
                  (!s.options.demoDrive ? "; it stands still until /cmd_vel arrives"
                   : s.bridge           ? "; a figure-eight demo drives until /cmd_vel arrives"
                                        : "; a figure-eight demo drives"));
    } else {
        s.log(v1::LOG_LEVEL_WARN, "controller",
              "no demo driver: it needs exactly two actuated wheel joints, the robot has " +
                  std::to_string(s.robot.actuatedJoints.size()));
    }
    for (std::size_t i = 0; i < s.robot.world.sensors.names.size(); ++i) {
        const World::Sensors& sensors = s.robot.world.sensors;
        char line[160];
        std::snprintf(line, sizeof(line), "%s %s on %s at %.0f Hz",
                      sensors.type[i] == SensorType::Lidar ? "lidar" : "IMU",
                      sensors.names[i].c_str(),
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
