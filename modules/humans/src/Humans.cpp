#include "lagrange/humans/Gait.hpp"
#include "lagrange/humans/Humans.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

namespace lagrange::humans {

namespace {

using namespace lagrange::literals;

constexpr std::uint64_t kSteerEvery = 5;  // module steps (10 ms) to one steering step
constexpr double kSteerDt = 0.05;         // s
constexpr double kRelaxation = 0.5;       // s, how fast a person takes up the wanted velocity
constexpr double kSocialA = 2.0;          // m/s² (Helbing and Molnár 1995, as traffic)
constexpr double kSocialB = 0.3;          // m
constexpr double kPersonRadius = 0.3;     // m
constexpr double kRobotRadius = 0.6;      // m, a nominal robot (bodies are not sized here)
constexpr double kStopDistance = 0.2;     // m of free gap below which nobody walks closer
constexpr double kSidestep = 1.5;         // of the walking acceleration, near an obstacle
constexpr double kSidestepReach = 2.5;    // m of gap at which people start going round
constexpr double kSidestepRange = 0.8;    // m, how the sidestep fades with the gap
constexpr double kArrived = 0.3;          // m from a target
constexpr double kBlockedAfter = 8.0;     // s nearly stopped: give up on this target
constexpr double kWallReach = 2.0;        // m, how far ahead walls are felt
constexpr double kMaxTurn = 3.0;          // rad/s
constexpr double kChest = 1.1;            // m above the ground, wall rays
constexpr double kUnderground = 100.0;    // m, where the zero substitute keeps people
constexpr std::array<double, 7> kFan{-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2};  // rad off heading

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

double wrap(double a) {
    return std::remainder(a, 2.0 * std::numbers::pi);
}

}  // namespace

Humans::Humans(bool zero, std::shared_ptr<const services::SceneQuery> scenery,
               std::shared_ptr<const services::SceneQuery> ground)
    : zero_(zero), scenery_(std::move(scenery)), ground_(std::move(ground)) {}

ModuleInfo Humans::info() const {
    ModuleInfo info{"humans", zero_ ? "zero" : "walking", 1, 0};
    info.isSubstitute = zero_;
    return info;
}

Duration Humans::preferredStep() const {
    return 10_ms;
}

Status Humans::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    people_.clear();
    robots_.clear();
    const World& raw = world.raw();
    const auto seed = static_cast<std::int64_t>(config.get<std::int64_t>("seed").value_or(0));
    for (const SchemaInstance* s : world.instancesOf("HumanAPI_1")) {
        Person p;
        p.index = static_cast<std::uint32_t>(people_.size());
        Spec& q = p.spec;
        q.name = raw.prims.instanceNames.at(s->prim.index);
        q.height = s->real("human:height").value_or(q.height);
        q.walkSpeed = s->real("human:walkSpeed").value_or(q.walkSpeed);
        q.behavior = std::string(s->token("human:behavior").value_or(q.behavior));
        const std::span<const double> way = s->realArray("human:waypoints");
        q.waypoints.assign(way.begin(), way.end());
        q.loop = s->integer("human:loop").value_or(1) != 0;
        q.dwell = s->real("human:dwell").value_or(q.dwell);
        q.wanderRadius = s->real("human:wanderRadius").value_or(q.wanderRadius);
        q.seed = s->integer("human:seed").value_or(0) + seed;
        for (std::size_t i = 0; i + 1 < q.waypoints.size(); i += 2) {  // into the local frame
            q.waypoints[i] -= raw.region.worldEnu[0];
            q.waypoints[i + 1] -= raw.region.worldEnu[1];
        }
        for (std::size_t k = 0; k < kSegments; ++k) {
            p.commands.at(k) = d.output(spec("body/" + segmentBody(q.name, k) + "/pose_command",
                                             SignalType::Pose, units::m, "world_local"));
        }
        const std::string base = "human/" + q.name + "/";
        p.pose = d.output(spec(base + "pose", SignalType::Pose, units::m, "world_local"));
        p.speedOut = d.output(spec(base + "speed", SignalType::Scalar, units::m_per_s));
        p.phaseOut = d.output(spec(base + "gait_phase", SignalType::Scalar, units::one));
        p.nearest = d.output(spec(base + "nearest_robot", SignalType::Scalar, units::m));
        SignalSpec goal = spec(base + "goal", SignalType::ScalarArray, units::one);
        goal.arrayLength = 3;
        goal.optional = true;
        p.goal = d.input(goal);
        // Lifts the right arm toward a point: (how far, 0 to 1; x; y).
        SignalSpec gesture = spec(base + "gesture", SignalType::ScalarArray, units::one);
        gesture.arrayLength = 3;
        gesture.optional = true;
        p.gesture = d.input(gesture);
        // Where the person is now: the pelvis as loaded, carried over a rebuilt run.
        for (std::size_t b = 0; b < raw.bodies.names.size(); ++b) {
            if (raw.bodies.names[b] == segmentBody(q.name, kPelvis)) {
                const Pose& pelvis = raw.bodies.poseInParent[b];
                const Proportions body(q.height);
                p.x = p.homeX = pelvis.position[0];
                p.y = p.homeY = pelvis.position[1];
                p.z = pelvis.position[2] - body.hip;
                const std::array<double, 3> ahead = rotate(pelvis.orientation, {1.0, 0.0, 0.0});
                p.heading = std::atan2(ahead[1], ahead[0]);
            }
        }
        people_.push_back(std::move(p));
    }
    for (const std::string& name : config.get<std::vector<std::string>>("robot_bodies")
                                       .value_or(std::vector<std::string>{})) {
        SignalSpec pose = spec("body/" + name + "/pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;  // physics reads these people's pose commands
        robots_.push_back(d.input(pose));
    }
    return {};
}

void Humans::place(Person& p, SignalBus& bus) const {
    const Proportions body(p.spec.height);
    const double speed = std::hypot(p.vx, p.vy);
    GaitSample g = gaitAt(body, speed, p.phase);
    constexpr double kReachShoulder = 1.35;  // rad, an arm held out
    g.posture.shoulderRight += (kReachShoulder - g.posture.shoulderRight) * p.reach;
    g.posture.elbowRight *= 1.0 - p.reach;
    Pose pelvis;
    pelvis.position = {p.x, p.y, (zero_ ? p.z - kUnderground : p.z) + g.pelvisHeight};
    pelvis.orientation = {std::cos(p.heading / 2.0), 0.0, 0.0, std::sin(p.heading / 2.0)};
    const std::array<Pose, kSegments> poses = segmentPoses(body, pelvis, g.posture);
    std::array<double, 7> values{};
    for (std::size_t k = 0; k < kSegments; ++k) {
        const Pose& pose = poses.at(k);
        std::copy(pose.position.begin(), pose.position.end(), values.begin());
        std::copy(pose.orientation.begin(), pose.orientation.end(), values.begin() + 3);
        bus.writeN(p.commands.at(k), values);
    }
    std::copy(pelvis.position.begin(), pelvis.position.end(), values.begin());
    std::copy(pelvis.orientation.begin(), pelvis.orientation.end(), values.begin() + 3);
    bus.writeN(p.pose, values);
    bus.write(p.speedOut, speed);
    bus.write(p.phaseOut, p.phase);
}

Status Humans::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    ticks_ = 0;
    for (Person& p : people_) {
        place(p, bus);
        bus.write(p.nearest, std::numeric_limits<double>::infinity());
    }
    return {};
}

void Humans::steer(Person& p, std::size_t self, const StepContext& context, SignalBus& bus) {
    // Where to: a goal command, else the behaviour.
    bool going = false;
    double tx = p.x;
    double ty = p.y;
    if (const std::span<const double> g = bus.readN(p.goal); g.size() == 3 && g[2] > 0.5) {
        p.goalX = g[0];
        p.goalY = g[1];
        p.hasGoal = 1;
    }
    const auto waypoints = p.spec.waypoints.size() / 2;
    if (p.hasGoal != 0) {
        tx = p.goalX;
        ty = p.goalY;
        going = true;
    } else if (p.dwellLeft > 0.0) {
        p.dwellLeft -= kSteerDt;
    } else if (p.spec.behavior == "waypoints" && waypoints > 0 &&
               (p.spec.loop || p.waypoint < waypoints)) {
        const std::size_t i = p.waypoint % waypoints;
        tx = p.spec.waypoints[2 * i];
        ty = p.spec.waypoints[2 * i + 1];
        going = true;
    } else if (p.spec.behavior == "wander") {
        // A target in the circle, fresh when the last is reached: (seed, draws, person).
        random::Stream rng(static_cast<std::uint64_t>(p.spec.seed), p.picks, p.index);
        const double r = p.spec.wanderRadius * std::sqrt(rng.uniform());
        const double a = 2.0 * std::numbers::pi * rng.uniform();
        tx = p.homeX + r * std::cos(a);
        ty = p.homeY + r * std::sin(a);
        going = true;
    }
    const auto next = [&p] {
        if (p.hasGoal != 0) {
            p.hasGoal = 0;
        } else {
            ++p.waypoint;
            ++p.picks;
            p.dwellLeft = p.spec.dwell;
        }
        p.blocked = 0.0;
    };
    if (going && std::hypot(tx - p.x, ty - p.y) < kArrived) {
        next();
        going = false;
    }

    // The wanted velocity, then the social force.
    double wantX = 0.0;
    double wantY = 0.0;
    if (going) {
        const double d = std::hypot(tx - p.x, ty - p.y);
        const double speed = std::min(p.spec.walkSpeed, d / kRelaxation);
        wantX = (tx - p.x) / d * speed;
        wantY = (ty - p.y) / d * speed;
    }
    double ax = (wantX - p.vx) / kRelaxation;
    double ay = (wantY - p.vy) / kRelaxation;
    const auto heading = [&] {
        const double s = std::hypot(p.vx, p.vy);
        return s > 0.05 ? std::array<double, 2>{p.vx / s, p.vy / s}
               : going  ? std::array<double, 2>{(tx - p.x), (ty - p.y)}
                        : std::array<double, 2>{std::cos(p.heading), std::sin(p.heading)};
    }();
    bool blockedAhead = false;
    const auto repel = [&](double ox, double oy, double radius) {
        if (!going) {
            return;  // someone standing stays put, whatever comes near
        }
        const double dx = p.x - ox;
        const double dy = p.y - oy;
        const double d = std::max(std::hypot(dx, dy), 1e-6);
        const double gap = d - radius;
        const double push = kSocialA * std::exp(-gap / kSocialB);
        // Mostly from what is ahead (the field of view of the model): half from behind.
        const double norm = std::max(std::hypot(heading[0], heading[1]), 1e-6);
        const double facing = -(dx * heading[0] + dy * heading[1]) / (d * norm);
        const double weight = facing > 0.0 ? 1.0 : 0.5;
        ax += push * weight * dx / d;
        ay += push * weight * dy / d;
        // Going round: a sideways push away from what is ahead (to the right when it is dead
        // ahead), so people pass an obstacle or each other instead of stopping nose to nose.
        if (going && facing > 0.5 && gap < kSidestepReach) {
            // Obstacle on the left of the heading, or dead ahead: go right; else left.
            const double left = heading[0] * -dy - heading[1] * -dx;
            const double side = left >= 0.0 ? 1.0 : -1.0;
            const double lateral = kSidestep * p.spec.walkSpeed / kRelaxation *
                                   std::exp(-std::max(gap, 0.0) / kSidestepRange);
            ax += side * lateral * heading[1] / norm;  // (hy, -hx) is the right of the heading
            ay -= side * lateral * heading[0] / norm;
        }
        if (facing > 0.9 && gap < kStopDistance) {
            blockedAhead = true;
        }
    };
    for (std::size_t o = 0; o < people_.size(); ++o) {
        if (o != self) {
            repel(people_[o].x, people_[o].y, 2.0 * kPersonRadius);
        }
    }
    double nearest = std::numeric_limits<double>::infinity();
    for (const SignalId robot : robots_) {
        const std::span<const double> pose = bus.readN(robot);
        if (pose.size() != 7) {
            continue;
        }
        repel(pose[0], pose[1], kPersonRadius + kRobotRadius);
        nearest = std::min(nearest, std::hypot(pose[0] - p.x, pose[1] - p.y));
    }
    bus.write(p.nearest, nearest);
    if (scenery_ && going) {
        const double angle = std::atan2(heading[1], heading[0]);
        const std::array<double, 3> origin{p.x, p.y, p.z + kChest};
        for (const double off : kFan) {
            const std::array<double, 3> dir{std::cos(angle + off), std::sin(angle + off), 0.0};
            const services::RayHit hit = scenery_->ray(origin, dir, kWallReach);
            if (!hit.hit()) {
                continue;
            }
            const double gap = hit.distance - kPersonRadius;
            const double push = kSocialA * std::exp(-gap / kSocialB);
            ax -= push * dir[0];
            ay -= push * dir[1];
            if (std::abs(off) < 0.5 && gap < kStopDistance / 2.0) {
                blockedAhead = true;
            }
        }
    }
    p.vx += ax * kSteerDt;
    p.vy += ay * kSteerDt;
    if (blockedAhead) {
        // Nothing closer: what is left of the velocity is along the obstacle or away.
        const double along = p.vx * heading[0] + p.vy * heading[1];
        if (along > 0.0) {
            const double norm = std::max(std::hypot(heading[0], heading[1]), 1e-6);
            p.vx -= along * heading[0] / (norm * norm);
            p.vy -= along * heading[1] / (norm * norm);
        }
    }
    const double limit = 1.3 * p.spec.walkSpeed;
    const double speed = std::hypot(p.vx, p.vy);
    if (speed > limit) {
        p.vx *= limit / speed;
        p.vy *= limit / speed;
    }
    if (speed > 0.1) {
        const double turn = std::clamp(wrap(std::atan2(p.vy, p.vx) - p.heading),
                                       -kMaxTurn * kSteerDt, kMaxTurn * kSteerDt);
        p.heading = wrap(p.heading + turn);
    }
    if (going && speed < 0.1) {
        p.blocked += kSteerDt;
        if (p.blocked > kBlockedAfter) {
            services::log(services::LogLevel::Info, "humans",
                          p.spec.name + " is blocked: skipping to the next target");
            next();
        }
    } else {
        p.blocked = 0.0;
    }
    if (ground_) {
        const services::RayHit hit = ground_->ray({p.x, p.y, p.z + 1.0}, {0.0, 0.0, -1.0}, 3.0);
        if (hit.hit()) {
            p.z += 1.0 - hit.distance;
        }
    }
    (void)context;
}

Status Humans::step(const StepContext& context, SignalBus& bus) {
    if (!zero_ && ticks_ % kSteerEvery == 0) {
        for (std::size_t i = 0; i < people_.size(); ++i) {
            steer(people_[i], i, context, bus);
        }
    }
    ++ticks_;
    const double dt = context.dt.seconds();
    for (Person& p : people_) {
        if (const std::span<const double> g = bus.readN(p.gesture); g.size() == 3) {
            p.reach = std::clamp(g[0], 0.0, 1.0);
            if (p.reach > 0.0 && !zero_) {  // turn to what it reaches for
                const double turn = wrap(std::atan2(g[2] - p.y, g[1] - p.x) - p.heading);
                p.heading = wrap(p.heading + std::clamp(turn, -kMaxTurn * dt, kMaxTurn * dt));
            }
        }
        if (!zero_) {
            const double speed = std::hypot(p.vx, p.vy);
            p.x += p.vx * dt;
            p.y += p.vy * dt;
            p.phase += speed * dt * phasePerMetre(p.spec.height, std::max(speed, kBlendSpeed));
            p.phase -= std::floor(p.phase);
        }
        place(p, bus);
    }
    return {};
}

Status Humans::saveState(StateWriter& writer) const {
    writer.put(ticks_);
    for (const Person& p : people_) {
        for (const double v : {p.x, p.y, p.z, p.heading, p.vx, p.vy, p.phase, p.dwellLeft,
                               p.blocked, p.goalX, p.goalY, p.homeX, p.homeY}) {
            writer.put(v);
        }
        writer.put(p.waypoint);
        writer.put(p.hasGoal);
        writer.put(p.picks);
    }
    return {};
}

Status Humans::restoreState(StateReader& reader) {
    bool ok = reader.get(ticks_);
    for (Person& p : people_) {
        for (double* v : {&p.x, &p.y, &p.z, &p.heading, &p.vx, &p.vy, &p.phase, &p.dwellLeft,
                          &p.blocked, &p.goalX, &p.goalY, &p.homeX, &p.homeY}) {
            ok = reader.get(*v) && ok;
        }
        ok = reader.get(p.waypoint) && ok;
        ok = reader.get(p.hasGoal) && ok;
        ok = reader.get(p.picks) && ok;
    }
    return ok ? Status{} : Status::error(StatusCode::Internal, "humans: short state");
}

void Humans::shiftOrigin(const std::array<double, 3>& shift) {
    for (Person& p : people_) {
        p.x -= shift[0];
        p.y -= shift[1];
        p.z -= shift[2];
        p.homeX -= shift[0];
        p.homeY -= shift[1];
        p.goalX -= shift[0];
        p.goalY -= shift[1];
        for (std::size_t i = 0; i + 1 < p.spec.waypoints.size(); i += 2) {
            p.spec.waypoints[i] -= shift[0];
            p.spec.waypoints[i + 1] -= shift[1];
        }
    }
}

}  // namespace lagrange::humans
