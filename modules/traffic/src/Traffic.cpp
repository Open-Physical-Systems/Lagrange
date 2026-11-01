// The traffic domain, builtin backend (Traffic.hpp, domains/traffic.md).
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/traffic/Traffic.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <numbers>
#include <random>
#include <span>
#include <tuple>

namespace lagrange::traffic {

using namespace literals;

namespace {

constexpr double kDefaultSpeed = 50.0 / 3.6;  // m/s, a lane without a limit
constexpr double kWalkSpeed = 1.34;           // m/s (Weidmann 1993)
constexpr double kSocialA = 2.0;              // m/s² (Helbing and Molnár 1995)
constexpr double kSocialB = 0.3;              // m
constexpr double kRelaxation = 0.5;           // s, from the social force to a sideways speed
constexpr double kMaxSideStep = 0.8;          // m/s, the fastest sideways step
constexpr double kLookAhead = 80.0;           // m, how far a vehicle looks for a leader
constexpr double kEgoLateral = 2.5;           // m: a robot this close to a lane is on it
constexpr double kHidden = -100.0;            // m, where the zero substitute keeps agents
constexpr double kVehicleSpacing = 12.0;      // m between vehicles when placing them
constexpr double kPedestrianSpacing = 3.0;    // m between pedestrians
constexpr double kAvoidRadius = 15.0;         // m kept free around the robots
constexpr double kEntryClearance = 15.0;      // m: an entry is free when nothing is closer
constexpr double kEntrySpeed = 8.0;           // m/s, a vehicle coming in at the network's edge
constexpr double kYieldMargin = 5.0;          // m before the line where right of way is decided
constexpr double kBoxSpeed = 2.0;             // m/s: slower than this, a car beyond is in the way
constexpr double kBoxRoom = 9.0;              // m past a junction a car needs free to enter it
constexpr double kPatience = 6.0;             // s a walker waits before pressing all the same
constexpr double kPressChance = 0.6;          // that a walker presses as soon as it arrives
constexpr double kPressTime = 1.5;            // s at the button
constexpr double kCrossChance = 0.5;          // that a walker at a corner takes a crossing
constexpr double kWalkLookAhead = 2.5;        // m, how far ahead a walker's goal is
constexpr double kWalkArrived = 0.9;          // m from a lane's end or a spot that counts as there
constexpr double kPoleRadius = 0.06;          // m
constexpr double kLampHeight = 3.4;           // m, the middle of a vehicle signal
constexpr double kWalkLampHeight = 2.5;       // m, the middle of a pedestrian signal
constexpr double kButtonHeight = 1.05;        // m
constexpr double kNominalLampMass = 0.1;      // kg; kinematic bodies are not moved by forces
constexpr std::uint32_t kAgentClassCar = 0;
constexpr std::uint32_t kAgentClassPedestrian = 1;

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

double distance2d(const Vec3& a, const Vec3& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}

std::array<double, 4> yawQuat(double yaw) {
    return {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
}

}  // namespace

/// Whether two polylines cross (or touch), in the plane.
bool crosses(const std::vector<Vec3>& a, const std::vector<Vec3>& b) {
    const auto orient = [](const Vec3& p, const Vec3& q, const Vec3& r) {
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    };
    for (std::size_t i = 0; i + 1 < a.size(); ++i) {
        for (std::size_t k = 0; k + 1 < b.size(); ++k) {
            const double d1 = orient(b[k], b[k + 1], a[i]);
            const double d2 = orient(b[k], b[k + 1], a[i + 1]);
            const double d3 = orient(a[i], a[i + 1], b[k]);
            const double d4 = orient(a[i], a[i + 1], b[k + 1]);
            if (((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))) {
                return true;
            }
        }
    }
    return false;
}

LaneGraph lanesOf(const WorldView& world) {
    LaneGraph g;
    g.lanes = roadLanesOf(world).lanes;
    // Junctions: driving lanes of a junction leave the same lane, end on the same lane, or cross.
    // (Not "the same road": on a road network a road joins two junctions.)
    // Each approach (a lane into a junction) gets a signal group by its heading at the stop line.
    g.signalGroup.assign(g.lanes.size(), -1);
    g.junctionOf.assign(g.lanes.size(), -1);
    std::vector<int> junctionOfLane(g.lanes.size(), -1);
    const auto driven = [&g](std::size_t i) { return g.lanes[i].junction && !g.lanes[i].sidewalk; };
    const auto shares = [](const std::vector<std::size_t>& a, const std::vector<std::size_t>& b) {
        return std::ranges::any_of(a, [&b](std::size_t s) { return std::ranges::count(b, s) > 0; });
    };
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (!driven(i) || junctionOfLane[i] >= 0) {
            continue;
        }
        std::vector<std::size_t> stack{i};
        junctionOfLane[i] = g.junctions;
        while (!stack.empty()) {
            const Lane& lane = g.lanes[stack.back()];
            stack.pop_back();
            for (std::size_t k = 0; k < g.lanes.size(); ++k) {
                if (driven(k) && junctionOfLane[k] < 0 &&
                    (shares(lane.predecessors, g.lanes[k].predecessors) ||
                     shares(lane.successors, g.lanes[k].successors) ||
                     crosses(lane.points, g.lanes[k].points))) {
                    junctionOfLane[k] = g.junctions;
                    stack.push_back(k);
                }
            }
        }
        ++g.junctions;
    }
    // A junction has lights when its lanes conflict; a crossing (below) gives it lights as well.
    g.signalled.assign(static_cast<std::size_t>(g.junctions), false);
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        for (std::size_t k = i + 1; k < g.lanes.size(); ++k) {
            if (driven(i) && driven(k) && junctionOfLane[i] == junctionOfLane[k] &&
                g.lanes[i].predecessors != g.lanes[k].predecessors &&
                (shares(g.lanes[i].successors, g.lanes[k].successors) ||
                 crosses(g.lanes[i].points, g.lanes[k].points))) {
                g.signalled[static_cast<std::size_t>(junctionOfLane[i])] = true;
            }
        }
    }
    // Crossings: sidewalk lanes over a junction's driving lanes.
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (!g.lanes[i].sidewalk) {
            continue;
        }
        LaneGraph::Crossing crossing;
        crossing.lane = i;
        for (std::size_t k = 0; k < g.lanes.size(); ++k) {
            if (driven(k) && crosses(g.lanes[i].points, g.lanes[k].points)) {
                crossing.junction = junctionOfLane[k];
                const Vec3& middle = g.lanes[i].points[g.lanes[i].points.size() / 2];
                crossing.over.emplace_back(k, g.lanes[k].project(middle).first);
            }
        }
        if (crossing.junction >= 0) {
            g.signalled[static_cast<std::size_t>(crossing.junction)] = true;
            g.crossings.push_back(std::move(crossing));
        }
    }
    // The walkers' network: sidewalk lanes whose ends touch.
    g.walkLinks.assign(g.lanes.size(), {});
    constexpr double kTouch = 0.5;  // m
    const auto touches = [](const Vec3& a, const Vec3& b) { return distance2d(a, b) < kTouch; };
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (!g.lanes[i].sidewalk) {
            continue;
        }
        for (std::size_t k = 0; k < g.lanes.size(); ++k) {
            if (k == i || !g.lanes[k].sidewalk) {
                continue;
            }
            const std::vector<Vec3>& a = g.lanes[i].points;
            const std::vector<Vec3>& b = g.lanes[k].points;
            if (touches(a.front(), b.front()) || touches(a.front(), b.back())) {
                g.walkLinks[i][0].push_back(k);
            }
            if (touches(a.back(), b.front()) || touches(a.back(), b.back())) {
                g.walkLinks[i][1].push_back(k);
            }
        }
    }
    for (int j = 0; j < g.junctions; ++j) {
        if (!g.signalled[static_cast<std::size_t>(j)]) {
            continue;
        }
        // Approaches into junction j and their headings at the stop line.
        std::vector<std::pair<std::size_t, double>> approaches;
        for (std::size_t i = 0; i < g.lanes.size(); ++i) {
            const Lane& lane = g.lanes[i];
            if (lane.junction || lane.sidewalk) {
                continue;
            }
            const bool into = std::ranges::any_of(
                lane.successors, [&](std::size_t s) { return junctionOfLane[s] == j; });
            if (into) {
                approaches.emplace_back(i, lane.at(lane.length()).second);
                g.junctionOf[i] = j;
            }
        }
        // Group 0: approaches heading like the first one or the opposite way; group 1: across.
        if (approaches.empty()) {
            continue;
        }
        const double reference = approaches.front().second;
        for (const auto& [lane, heading] : approaches) {
            const double d = std::abs(std::remainder(heading - reference, std::numbers::pi));
            g.signalGroup[lane] = d < std::numbers::pi / 4.0 ? 0 : 1;
        }
    }
    return g;
}

Light lightAt(int group, int groups, double t) {
    const double phase = kGreenSeconds + kYellowSeconds + kAllRedSeconds;
    const double cycle = phase * groups;
    const double within = std::fmod(t, cycle) - group * phase;
    if (within < 0.0 || within >= phase) {
        return Light::Red;
    }
    if (within < kGreenSeconds) {
        return Light::Green;
    }
    return within < kGreenSeconds + kYellowSeconds ? Light::Yellow : Light::Red;
}

double idmAcceleration(const Idm& p, double v, double v0, double gap, double leaderSpeed) {
    const double free = 1.0 - std::pow(v / std::max(v0, 0.1), 4);
    if (!std::isfinite(gap)) {
        return p.a * free;
    }
    const double desired =
        p.s0 + std::max(0.0, v * p.T + v * (v - leaderSpeed) / (2.0 * std::sqrt(p.a * p.b)));
    const double ratio = desired / std::max(gap, 0.1);
    return p.a * (free - ratio * ratio);
}

namespace {

/// A rotation by `yaw` about z, then the local x seen along it.
Vec3 along(double yaw, double forward, double left, double up) {
    return {(std::cos(yaw) * forward) - (std::sin(yaw) * left),
            (std::sin(yaw) * forward) + (std::cos(yaw) * left), up};
}

Vec3 plus(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

/// A vehicle's approach: the signal head stands on the right of the lane's end.
struct Head {
    Vec3 at;     // the housing's middle
    double yaw;  // the way it faces
};
Head vehicleHead(const Lane& lane) {
    const auto [end, yaw] = lane.at(lane.length());
    return {plus(end, along(yaw, 0.0, -(lane.width / 2.0 + 0.6), kLampHeight)),
            yaw + std::numbers::pi};
}
/// A crossing's end (0 start, 1 end): the signal there faces those about to come from the other.
Head walkHead(const Lane& lane, int end) {
    const auto [p, yaw] = lane.at(end == 0 ? 0.0 : lane.length());
    const double side = lane.width / 2.0 + 0.4;
    // Back from the kerb, onto the pavement.
    constexpr double kBack = 0.9;  // m
    return {
        plus(p, along(yaw, end == 0 ? -kBack : kBack, end == 0 ? -side : side, kWalkLampHeight)),
        end == 0 ? yaw : yaw + std::numbers::pi};
}
/// Where a walker stands to press the button at a crossing's end.
std::array<double, 2> buttonSpot(const Lane& lane, int end) {
    const auto [p, yaw] = lane.at(end == 0 ? 0.0 : lane.length());
    // Beside the pole, on the side away from the road, an arm's length off.
    const double side = lane.width / 2.0 + 0.1;
    const Vec3 at = plus(p, along(yaw, end == 0 ? -1.2 : 1.2, end == 0 ? -side : side, 0.0));
    return {at[0], at[1]};
}

void pushMesh(TriMesh& out, const TriMesh& part, const Pose& pose) {
    const auto base = static_cast<std::uint32_t>(out.positions.size() / 3);
    for (std::size_t i = 0; i + 2 < part.positions.size(); i += 3) {
        const Vec3 p = plus(pose.position,
                            rotate(pose.orientation, {static_cast<double>(part.positions[i]),
                                                      static_cast<double>(part.positions[i + 1]),
                                                      static_cast<double>(part.positions[i + 2])}));
        const auto n = rotate(pose.orientation, {static_cast<double>(part.normals[i]),
                                                 static_cast<double>(part.normals[i + 1]),
                                                 static_cast<double>(part.normals[i + 2])});
        for (int k = 0; k < 3; ++k) {
            out.positions.push_back(static_cast<float>(p.at(k)));
            out.normals.push_back(static_cast<float>(n.at(k)));
        }
    }
    for (const std::uint32_t index : part.indices) {
        out.indices.push_back(base + index);
    }
}

Pose boxAt(double x, double y, double z) {
    Pose pose;
    pose.position = {x, y, z};
    return pose;
}

/// One flat-shaded quad, wound so its normal is (b − a) × (c − a).
void quad(TriMesh& m, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    const Vec3 u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const Vec3 v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    Vec3 n{(u[1] * v[2]) - (u[2] * v[1]), (u[2] * v[0]) - (u[0] * v[2]),
           (u[0] * v[1]) - (u[1] * v[0])};
    const double length = std::max(std::hypot(n[0], n[1], n[2]), 1e-9);
    const auto base = static_cast<std::uint32_t>(m.positions.size() / 3);
    for (const Vec3* p : {&a, &b, &c, &d}) {
        for (int k = 0; k < 3; ++k) {
            m.positions.push_back(static_cast<float>(p->at(k)));
            m.normals.push_back(static_cast<float>(n.at(k) / length));
        }
    }
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

/// A side profile (x, z) extruded across the width, the top surface and both sides.
void extrude(TriMesh& m, const std::vector<std::array<double, 2>>& top, double halfWidth,
             double bottom) {
    for (std::size_t i = 0; i + 1 < top.size(); ++i) {
        const auto& p = top[i];
        const auto& q = top[i + 1];
        quad(m, {p[0], halfWidth, p[1]}, {p[0], -halfWidth, p[1]}, {q[0], -halfWidth, q[1]},
             {q[0], halfWidth, q[1]});
        quad(m, {p[0], -halfWidth, bottom}, {q[0], -halfWidth, bottom}, {q[0], -halfWidth, q[1]},
             {p[0], -halfWidth, p[1]});
        quad(m, {q[0], halfWidth, bottom}, {p[0], halfWidth, bottom}, {p[0], halfWidth, p[1]},
             {q[0], halfWidth, q[1]});
    }
}

/// How one kind of vehicle is drawn, within its box and about the box's middle: paint, glass, tyres
/// and lamps; hubs, brake lamps and indicators turn on hinges, so they are parts of their own.
struct VehicleShape {
    double belt;       // m above the middle, where the glass starts
    double roofDrop;   // m under the box's top
    double screen;     // x of the windscreen's foot
    double roofFront;  // x where the roof starts
    double rearFoot;   // x of the rear window's foot
    double rearTop;    // x where the roof ends
    double clearance;  // m under the body
    double tyre;       // m, radius
    double axle;       // m from the ends to the axles
};
const std::array<VehicleShape, 5> kShapes{{
    {-0.08, 0.08, 0.75, 0.0, -1.45, -0.75, 0.18, 0.33, 0.85},
    {-0.08, 0.0, 0.8, 0.05, -1.8, -1.45, 0.18, 0.32, 0.7},
    {0.05, 0.0, 1.2, 0.6, -2.23, -2.1, 0.24, 0.37, 0.85},
    {0.0, 0.0, 2.35, 1.7, -2.65, -2.62, 0.25, 0.37, 1.0},
    {-0.25, 0.0, 4.4, 4.15, -4.4, -4.38, 0.32, 0.5, 1.7},
}};
struct CarMeshes {
    TriMesh paint, glass, tyres, head, tail;
    TriMesh hub, brake, indicatorFront, indicatorRear;
    Vec3 brakePivot{};
    std::array<Vec3, 4> hubs{};
    std::array<Vec3, 4> indicators{};  // front left, front right, rear left, rear right
};
constexpr double kLampSink = 0.08;  // m from a lamp's hinge to the lamp: out when lit, in when not
CarMeshes carMeshes(std::size_t kind) {
    const VehicleShape& s = kShapes.at(kind);
    const double l = kVehicleSizes.at(kind)[0] / 2.0;
    const double w = kVehicleSizes.at(kind)[1] / 2.0;
    const double h = kVehicleSizes.at(kind)[2] / 2.0;
    const double floor = -h + s.clearance;
    const double roof = h - s.roofDrop;
    CarMeshes m;
    std::vector<std::array<double, 2>> body{
        {l, floor + 0.12}, {l, s.belt - 0.14}, {l - 0.2, s.belt - 0.04}, {s.screen, s.belt}};
    body.push_back({s.rearFoot, s.belt});
    if (kind == 0) {
        body.push_back({-l + 0.1, s.belt + 0.02});
    }
    body.push_back({-l, s.belt - 0.08});
    body.push_back({-l, floor + 0.1});
    // Clockwise from above seen along +y, so the top quad's normal points up.
    std::reverse(body.begin(), body.end());
    extrude(m.paint, body, w, floor);
    quad(m.paint, {l, w, floor + 0.12}, {l, w, s.belt - 0.14}, {l, -w, s.belt - 0.14},
         {l, -w, floor + 0.12});
    quad(m.paint, {-l, -w, floor + 0.1}, {-l, -w, s.belt - 0.08}, {-l, w, s.belt - 0.08},
         {-l, w, floor + 0.1});
    // The cabin: glass all round, a painted roof on it, pillars between the windows.
    const double b = w - 0.06;
    const double t = w - (kind >= 3 ? 0.06 : 0.22);
    quad(m.glass, {s.screen, b, s.belt}, {s.screen, -b, s.belt}, {s.roofFront, -t, roof},
         {s.roofFront, t, roof});
    quad(m.glass, {s.rearFoot, -b, s.belt}, {s.rearFoot, b, s.belt}, {s.rearTop, t, roof},
         {s.rearTop, -t, roof});
    quad(m.glass, {s.rearFoot, b, s.belt}, {s.screen, b, s.belt}, {s.roofFront, t, roof},
         {s.rearTop, t, roof});
    quad(m.glass, {s.screen, -b, s.belt}, {s.rearFoot, -b, s.belt}, {s.rearTop, -t, roof},
         {s.roofFront, -t, roof});
    quad(m.paint, {s.roofFront, t, roof}, {s.roofFront, -t, roof}, {s.rearTop, -t, roof},
         {s.rearTop, t, roof});
    const int pillars = kind == 4 ? 6 : kind == 3 ? 2 : 1;
    for (int p = 1; p <= pillars; ++p) {
        const double f = static_cast<double>(p) / (pillars + 1);
        const double x0 = s.rearFoot + ((s.screen - s.rearFoot) * f);
        const double x1 = s.rearTop + ((s.roofFront - s.rearTop) * f);
        for (const double side : {1.0, -1.0}) {
            const double y0 = side * (b + 0.005);
            const double y1 = side * (t + 0.005);
            const double half = 0.04;
            if (side > 0.0) {
                quad(m.paint, {x0 - half, y0, s.belt}, {x0 + half, y0, s.belt},
                     {x1 + half, y1, roof}, {x1 - half, y1, roof});
            } else {
                quad(m.paint, {x0 + half, y0, s.belt}, {x0 - half, y0, s.belt},
                     {x1 - half, y1, roof}, {x1 + half, y1, roof});
            }
        }
    }
    // Bumpers, sills and mirrors.
    pushMesh(m.tyres, boxMesh(0.08, 2.0 * w, 0.2), boxAt(l - 0.02, 0.0, floor + 0.12));
    pushMesh(m.tyres, boxMesh(0.08, 2.0 * w, 0.2), boxAt(-l + 0.02, 0.0, floor + 0.12));
    pushMesh(m.tyres, boxMesh(2.0 * l - 0.2, 2.0 * w + 0.02, 0.1), boxAt(0.0, 0.0, floor + 0.04));
    pushMesh(m.paint, boxMesh(0.12, 0.2, 0.1), boxAt(s.screen - 0.05, w + 0.08, s.belt + 0.08));
    pushMesh(m.paint, boxMesh(0.12, 0.2, 0.1), boxAt(s.screen - 0.05, -w - 0.08, s.belt + 0.08));
    // Wheels: axles along y; the hub turns, so that a wheel is seen to roll.
    Pose wheel;
    wheel.orientation = {std::cos(std::numbers::pi / 4.0), std::sin(std::numbers::pi / 4.0), 0.0,
                         0.0};
    const TriMesh tyre = cylinderMesh(s.tyre, 0.22, 20);
    wheel.position = {};
    pushMesh(m.hub, cylinderMesh(s.tyre * 0.58, 0.24, 14), wheel);
    pushMesh(m.hub, boxMesh(s.tyre * 1.25, 0.25, 0.05), Pose{});
    pushMesh(m.hub, boxMesh(0.05, 0.25, s.tyre * 1.25), Pose{});
    std::size_t at = 0;
    for (const double x : {l - s.axle, -l + s.axle}) {
        for (const double y : {w - 0.1, -w + 0.1}) {
            wheel.position = {x, y, -h + s.tyre};
            pushMesh(m.tyres, tyre, wheel);
            m.hubs.at(at++) = wheel.position;
        }
    }
    const double lampZ = std::max(s.belt - 0.26, floor + 0.2);
    const TriMesh lamp = boxMesh(0.03, 0.34, 0.1);
    const TriMesh small = boxMesh(0.03, 0.12, 0.09);
    for (const double y : {w - 0.25, -w + 0.25}) {
        pushMesh(m.head, lamp, boxAt(l + 0.005, y, lampZ));
        pushMesh(m.tail, lamp, boxAt(-l - 0.005, y, lampZ));
        pushMesh(m.brake, lamp, boxAt(-kLampSink - 0.01, y, 0.0));
    }
    m.brakePivot = {-l + kLampSink, 0.0, lampZ};
    pushMesh(m.indicatorFront, small, boxAt(kLampSink + 0.01, 0.0, 0.0));
    pushMesh(m.indicatorRear, small, boxAt(-kLampSink - 0.01, 0.0, 0.0));
    m.indicators = {Vec3{l - kLampSink, w - 0.07, lampZ}, Vec3{l - kLampSink, -w + 0.07, lampZ},
                    Vec3{-l + kLampSink, w - 0.07, lampZ}, Vec3{-l + kLampSink, -w + 0.07, lampZ}};
    return m;
}

/// A little person for a pedestrian signal, flat, facing +x: standing, or in mid stride.
TriMesh figureMesh(bool walking) {
    TriMesh m;
    const auto part = [&m](double sy, double sz, double y, double z, double lean) {
        Pose pose = boxAt(0.0, y, z);
        pose.orientation = {std::cos(lean / 2.0), std::sin(lean / 2.0), 0.0, 0.0};
        pushMesh(m, boxMesh(0.01, sy, sz), pose);
    };
    part(0.05, 0.05, 0.0, 0.09, 0.0);  // head
    part(0.06, 0.1, 0.0, 0.0, 0.0);    // trunk
    if (walking) {
        part(0.025, 0.11, 0.03, -0.1, -0.5);
        part(0.025, 0.11, -0.03, -0.1, 0.5);
        part(0.02, 0.09, 0.05, 0.0, -0.7);
        part(0.02, 0.09, -0.05, 0.0, 0.7);
    } else {
        part(0.025, 0.11, 0.015, -0.1, 0.0);
        part(0.025, 0.11, -0.015, -0.1, 0.0);
        part(0.02, 0.09, 0.05, 0.0, 0.0);
        part(0.02, 0.09, -0.05, 0.0, 0.0);
    }
    return m;
}

}  // namespace

std::size_t vehicleKind(int index) {
    if (index % 12 == 7) {
        return 4;  // a bus
    }
    if (index % 7 == 3) {
        return 3;  // a van
    }
    return static_cast<std::size_t>(index) % 3;
}

std::vector<SignalLamp> signalLamps(const LaneGraph& g) {
    std::vector<SignalLamp> lamps;
    const auto lamp = [&lamps](std::string name, LampKind kind, std::size_t lane, const Head& head,
                               double up, double front) {
        SignalLamp out;
        out.name = std::move(name);
        out.kind = kind;
        out.lane = lane;
        out.lit.position = plus(head.at, along(head.yaw, front, 0.0, up));
        out.lit.orientation = yawQuat(head.yaw);
        out.dark.position = plus(head.at, along(head.yaw, 0.0, 0.0, up));
        out.dark.orientation = out.lit.orientation;
        lamps.push_back(std::move(out));
    };
    constexpr double kSpacing = 0.33;  // m between a vehicle signal's lamps
    constexpr double kFront = 0.112;   // m, a lit lamp from the housing's middle
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.signalGroup[i] < 0) {
            continue;
        }
        const Head head = vehicleHead(g.lanes[i]);
        const std::string base = "traffic_lamp_" + std::to_string(i);
        lamp(base + "_red", LampKind::Red, i, head, kSpacing, kFront);
        lamp(base + "_yellow", LampKind::Yellow, i, head, 0.0, kFront);
        lamp(base + "_green", LampKind::Green, i, head, -kSpacing, kFront);
    }
    for (std::size_t c = 0; c < g.crossings.size(); ++c) {
        const Lane& lane = g.lanes[g.crossings[c].lane];
        for (int end = 0; end < 2; ++end) {
            const Head head = walkHead(lane, end);
            const std::string base =
                "traffic_lamp_crossing_" + std::to_string(c) + "_" + std::to_string(end);
            lamp(base + "_dont_walk", LampKind::DontWalk, c, head, 0.17, kFront);
            lamp(base + "_walk", LampKind::Walk, c, head, -0.17, kFront);
            Head button = head;
            button.at[2] = kButtonHeight;
            lamp(base + "_wait", LampKind::Wait, c, button, 0.0, 0.052);
        }
    }
    return lamps;
}

std::vector<WalkerStart> walkersFor(const WorldView& world, int count, std::uint64_t seed,
                                    const std::vector<Vec3>& avoid) {
    std::vector<WalkerStart> out;
    const LaneGraph g = lanesOf(world);
    if (g.crossings.empty() || count <= 0) {
        return out;
    }
    std::vector<std::size_t> walks;
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.lanes[i].sidewalk && !g.lanes[i].junction && g.lanes[i].length() > 6.0) {
            walks.push_back(i);
        }
    }
    std::mt19937_64 rng(seed + 0x7A1C);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < count && !walks.empty(); ++i) {
        for (int attempt = 0; attempt < 400; ++attempt) {
            const Lane& lane =
                g.lanes[walks[static_cast<std::size_t>(unit(rng) * walks.size()) % walks.size()]];
            const auto [p, yaw] = lane.at(unit(rng) * lane.length());
            const bool clear =
                std::ranges::none_of(out,
                                     [&p](const WalkerStart& o) {
                                         return distance2d(o.ground.position, p) <
                                                kPedestrianSpacing;
                                     }) &&
                std::ranges::none_of(avoid, [&p](const Vec3& q) { return distance2d(p, q) < 2.0; });
            if (!clear) {
                continue;
            }
            WalkerStart walker;
            walker.name = "walker_" + std::to_string(i);
            walker.ground.position = p;
            walker.ground.orientation = yawQuat(yaw);
            // Stature and walking speed spread as adults' do (illustrative).
            walker.height = 1.55 + (0.35 * unit(rng));
            walker.walkSpeed = 1.1 + (0.45 * unit(rng));
            walker.seed = i + 1;
            out.push_back(std::move(walker));
            break;
        }
    }
    return out;
}

std::array<int, 2> addAgents(Model& model, const Population& population,
                             const std::vector<Vec3>& avoid) {
    const LaneGraph g = lanesOf(WorldView(model.world));
    std::mt19937_64 rng(population.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<std::size_t> roads, walks;
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.lanes[i].junction || g.lanes[i].length() < 12.0) {
            continue;
        }
        (g.lanes[i].sidewalk ? walks : roads).push_back(i);
    }
    World& w = model.world;
    const auto robot = static_cast<std::uint32_t>(w.robots.size());
    w.robots.emplace_back("traffic");
    ScenePrim top;
    top.name = "traffic";
    top.path = "/traffic";
    top.typeName = "Xform";
    model.scene.prims.push_back(top);
    const auto topPrim = static_cast<std::uint32_t>(model.scene.prims.size() - 1);
    static constexpr std::array<std::array<float, 4>, 6> kPaint{{{0.8F, 0.8F, 0.82F, 1.0F},
                                                                 {0.04F, 0.045F, 0.05F, 1.0F},
                                                                 {0.06F, 0.14F, 0.42F, 1.0F},
                                                                 {0.42F, 0.43F, 0.45F, 1.0F},
                                                                 {0.48F, 0.03F, 0.03F, 1.0F},
                                                                 {0.06F, 0.22F, 0.14F, 1.0F}}};
    const auto materialFor = [&model](const std::string& name, std::array<float, 4> rgba) {
        model.scene.materials.push_back({name, rgba});
        return static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    };
    const auto addMesh = [&model](TriMesh mesh) {
        model.scene.meshes.push_back(std::move(mesh));
        return static_cast<std::uint32_t>(model.scene.meshes.size() - 1);
    };
    struct CarKind {
        std::uint32_t paint, glass, tyres, head, tail, hub, brake, front, rear;
        std::array<Vec3, 4> hubs, indicators;
        Vec3 brakePivot;
    };
    std::array<CarKind, kVehicleSizes.size()> carKinds{};
    for (std::size_t k = 0; k < carKinds.size(); ++k) {
        CarMeshes parts = carMeshes(k);
        carKinds.at(k) = {addMesh(std::move(parts.paint)),
                          addMesh(std::move(parts.glass)),
                          addMesh(std::move(parts.tyres)),
                          addMesh(std::move(parts.head)),
                          addMesh(std::move(parts.tail)),
                          addMesh(std::move(parts.hub)),
                          addMesh(std::move(parts.brake)),
                          addMesh(std::move(parts.indicatorFront)),
                          addMesh(std::move(parts.indicatorRear)),
                          parts.hubs,
                          parts.indicators,
                          parts.brakePivot};
    }
    const std::uint32_t glassMaterial = materialFor("car_glass", {0.05F, 0.07F, 0.09F, 1.0F});
    const std::uint32_t tyreMaterial = materialFor("tyre_rubber", {0.03F, 0.03F, 0.03F, 1.0F});
    const auto glowing = [&model](const std::string& name, std::array<float, 3> colour,
                                  float strength) {
        SceneMaterial material;
        material.name = name;
        material.rgba = {colour[0], colour[1], colour[2], 1.0F};
        material.look = SurfaceLook{
            0.0F, 0.3F, {colour[0] * strength, colour[1] * strength, colour[2] * strength}};
        model.scene.materials.push_back(std::move(material));
        return static_cast<std::uint32_t>(model.scene.materials.size() - 1);
    };
    const std::uint32_t headMaterial = glowing("car_headlamp", {0.95F, 0.95F, 0.85F}, 0.6F);
    const std::uint32_t tailMaterial = glowing("car_taillamp", {0.7F, 0.04F, 0.03F}, 0.4F);
    const std::uint32_t brakeMaterial = glowing("car_brake_lamp", {1.0F, 0.05F, 0.03F}, 4.0F);
    const std::uint32_t amberMaterial = glowing("car_indicator", {1.0F, 0.5F, 0.03F}, 4.0F);
    const std::uint32_t hubMaterial = materialFor("wheel_hub_steel", {0.45F, 0.46F, 0.48F, 1.0F});
    const std::uint32_t walkerMesh = static_cast<std::uint32_t>(model.scene.meshes.size());
    model.scene.meshes.push_back(cylinderMesh(kPedestrianRadius, kPedestrianHeight, 16));
    const std::uint32_t walkerMaterial = materialFor("pedestrian", {0.3F, 0.45F, 0.75F, 1.0F});

    std::vector<Vec3> placed;
    const auto place = [&](const std::vector<std::size_t>& from, double spacing, bool pedestrian,
                           int index) -> bool {
        if (from.empty()) {
            return false;
        }
        for (int attempt = 0; attempt < 400; ++attempt) {
            const Lane& lane =
                g.lanes[from[static_cast<std::size_t>(unit(rng) * from.size()) % from.size()]];
            const double s = 6.0 + unit(rng) * (lane.length() - 12.0);
            const auto [p, yaw] = lane.at(s);
            const bool clear =
                std::ranges::none_of(placed,
                                     [&](const Vec3& q) { return distance2d(p, q) < spacing; }) &&
                std::ranges::none_of(
                    avoid, [&](const Vec3& q) { return distance2d(p, q) < kAvoidRadius; });
            if (!clear) {
                continue;
            }
            placed.push_back(p);
            const std::array<double, 3> size = kVehicleSizes.at(vehicleKind(index));
            const double height = pedestrian ? kPedestrianHeight : size[2];
            Pose pose;
            pose.position = {p[0], p[1], p[2] + height / 2.0};
            pose.orientation = yawQuat(yaw);
            const std::string name =
                std::string(pedestrian ? "traffic_pedestrian_" : "traffic_vehicle_") +
                std::to_string(index);
            // Masses and inertias are nominal: kinematic bodies are not moved by forces.
            const double mass = pedestrian ? 75.0 : 1500.0;
            const BodyId body = w.addBody(name, kWorldBody, pose, mass, {}, {1, 1, 1, 0, 0, 0});
            w.bodies.robot.back() = robot;
            w.bodies.kinematic.back() = true;
            if (pedestrian) {
                w.addCollider(body, ShapeType::Capsule,
                              {kPedestrianRadius, height / 2.0 - kPedestrianRadius, 0.0}, Pose{},
                              0.8);
            } else {
                w.addCollider(body, ShapeType::Box, {size[0] / 2.0, size[1] / 2.0, size[2] / 2.0},
                              Pose{}, 0.8);
            }
            model.bodyRestPose.push_back(pose);
            ScenePrim prim;
            prim.name = name;
            prim.path = "/traffic/" + name;
            prim.parent = topPrim;
            prim.typeName = "Mesh";
            const CarKind& kind = carKinds.at(vehicleKind(index));
            prim.mesh = pedestrian ? walkerMesh : kind.paint;
            prim.material =
                pedestrian ? walkerMaterial
                           : materialFor(name + "_gloss_paint",
                                         kPaint[static_cast<std::size_t>(index) % kPaint.size()]);
            prim.body = body.index;
            model.scene.prims.push_back(prim);
            model.bodyPrim.push_back(static_cast<std::uint32_t>(model.scene.prims.size() - 1));
            if (!pedestrian) {
                for (const auto& [part, mesh, material] :
                     {std::tuple{"glass", kind.glass, glassMaterial},
                      std::tuple{"tyres", kind.tyres, tyreMaterial},
                      std::tuple{"headlamps", kind.head, headMaterial},
                      std::tuple{"taillamps", kind.tail, tailMaterial}}) {
                    ScenePrim piece = prim;
                    piece.name = part;
                    piece.path = prim.path + "/" + part;
                    piece.parent = model.bodyPrim.back();
                    piece.mesh = mesh;
                    piece.material = material;
                    model.scene.prims.push_back(std::move(piece));
                }
                // What turns: on hinges driven by the module's signals.
                const std::string signals = "traffic/vehicle/" + std::to_string(index) + "/";
                const auto hinged = [&](const std::string& part, std::uint32_t mesh,
                                        std::uint32_t material, const Vec3& at,
                                        const std::string& signal) {
                    ScenePrim piece = prim;
                    piece.name = part;
                    piece.path = prim.path + "/" + part;
                    piece.parent = model.bodyPrim.back();
                    piece.mesh = mesh;
                    piece.material = material;
                    piece.offset.position = at;
                    piece.hinge = signals + signal;
                    piece.hingeAxis = {0.0, 1.0, 0.0};
                    model.scene.prims.push_back(std::move(piece));
                };
                for (std::size_t h = 0; h < kind.hubs.size(); ++h) {
                    hinged("hub_" + std::to_string(h), kind.hub, hubMaterial, kind.hubs.at(h),
                           "wheel");
                }
                hinged("brake_lamps", kind.brake, brakeMaterial, kind.brakePivot, "brake");
                hinged("indicator_fl", kind.front, amberMaterial, kind.indicators[0], "left");
                hinged("indicator_fr", kind.front, amberMaterial, kind.indicators[1], "right");
                hinged("indicator_rl", kind.rear, amberMaterial, kind.indicators[2], "left");
                hinged("indicator_rr", kind.rear, amberMaterial, kind.indicators[3], "right");
            }
            return true;
        }
        return false;
    };
    std::array<int, 2> made{0, 0};
    for (int i = 0; i < population.vehicles; ++i) {
        made[0] += place(roads, kVehicleSpacing, false, made[0]) ? 1 : 0;
    }
    // On a world with crossings pedestrians are walkers, which the caller adds as people.
    for (int i = 0; g.crossings.empty() && i < population.pedestrians; ++i) {
        made[1] += place(walks, kPedestrianSpacing, true, made[1]) ? 1 : 0;
    }

    // The signals: poles and housings as scenery, and the lamps that the module moves.
    const std::uint32_t poleMaterial = materialFor("signal_pole_steel", {0.2F, 0.21F, 0.22F, 1.0F});
    const std::uint32_t housingMaterial =
        materialFor("signal_housing_plastic", {0.02F, 0.02F, 0.022F, 1.0F});
    const std::uint32_t housingMesh = addMesh(boxMesh(0.2, 0.32, 1.02));
    const std::uint32_t walkHousingMesh = addMesh(boxMesh(0.2, 0.26, 0.6));
    const std::uint32_t buttonMesh = addMesh(boxMesh(0.1, 0.1, 0.16));
    const std::uint32_t lensDisc = addMesh(cylinderMesh(0.105, 0.02, 20));
    const std::uint32_t waitDisc = addMesh(cylinderMesh(0.022, 0.012, 12));
    const std::uint32_t standing = addMesh(figureMesh(false));
    const std::uint32_t striding = addMesh(figureMesh(true));
    const std::uint32_t redLit = glowing("signal_red", {1.0F, 0.06F, 0.04F}, 6.0F);
    const std::uint32_t yellowLit = glowing("signal_yellow", {1.0F, 0.62F, 0.05F}, 6.0F);
    const std::uint32_t greenLit = glowing("signal_green", {0.08F, 1.0F, 0.42F}, 6.0F);
    const std::uint32_t waitLit = glowing("signal_wait", {1.0F, 0.6F, 0.1F}, 4.0F);
    const std::uint32_t redDark = materialFor("lens_dark_red", {0.1F, 0.015F, 0.012F, 1.0F});
    const std::uint32_t yellowDark = materialFor("lens_dark_amber", {0.1F, 0.06F, 0.012F, 1.0F});
    const std::uint32_t greenDark = materialFor("lens_dark_green", {0.012F, 0.08F, 0.04F, 1.0F});
    const auto scenery = [&](const std::string& name, std::uint32_t mesh, std::uint32_t material,
                             const Pose& pose) {
        ScenePrim prim;
        prim.name = name;
        prim.path = "/traffic/" + name;
        prim.parent = topPrim;
        prim.typeName = "Mesh";
        prim.mesh = mesh;
        prim.material = material;
        prim.offset = pose;
        model.scene.prims.push_back(std::move(prim));
    };
    const auto pole = [&](const std::string& name, const Vec3& at, double height) {
        Pose pose = boxAt(at[0], at[1], height / 2.0);
        scenery(name, addMesh(cylinderMesh(kPoleRadius, height, 12)), poleMaterial, pose);
        w.addCollider(kWorldBody, ShapeType::Cylinder, {kPoleRadius, height / 2.0, 0.0}, pose, 0.6);
    };
    const auto facing = [](const Head& head, double up, double front) {
        Pose pose;
        pose.position = plus(head.at, along(head.yaw, front, 0.0, up));
        pose.orientation = yawQuat(head.yaw);
        return pose;
    };
    const auto discPose = [&facing](const Head& head, double up, double front) {
        // A disc's axis is z: turned to look along x.
        Pose pose = facing(head, up, front);
        pose = compose(pose, Pose{{0.0, 0.0, 0.0},
                                  {std::cos(std::numbers::pi / 4.0), 0.0,
                                   std::sin(std::numbers::pi / 4.0), 0.0}});
        return pose;
    };
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.signalGroup[i] < 0) {
            continue;
        }
        const Head head = vehicleHead(g.lanes[i]);
        const std::string base = "signal_" + std::to_string(i);
        pole(base + "_pole", {head.at[0], head.at[1], 0.0}, kLampHeight - 0.4);
        scenery(base + "_housing", housingMesh, housingMaterial, facing(head, 0.0, 0.0));
        scenery(base + "_lens_r", lensDisc, redDark, discPose(head, 0.33, 0.105));
        scenery(base + "_lens_y", lensDisc, yellowDark, discPose(head, 0.0, 0.105));
        scenery(base + "_lens_g", lensDisc, greenDark, discPose(head, -0.33, 0.105));
    }
    for (std::size_t c = 0; c < g.crossings.size(); ++c) {
        for (int end = 0; end < 2; ++end) {
            const Head head = walkHead(g.lanes[g.crossings[c].lane], end);
            const std::string base = "crossing_" + std::to_string(c) + "_" + std::to_string(end);
            pole(base + "_pole", {head.at[0], head.at[1], 0.0}, kWalkLampHeight);
            scenery(base + "_housing", walkHousingMesh, housingMaterial, facing(head, 0.0, 0.0));
            Head button = head;
            button.at[2] = kButtonHeight;
            scenery(base + "_button", buttonMesh, poleMaterial, facing(button, 0.0, 0.0));
        }
    }
    for (const SignalLamp& lamp : signalLamps(g)) {
        const BodyId body =
            w.addBody(lamp.name, kWorldBody, lamp.dark, kNominalLampMass, {}, {1, 1, 1, 0, 0, 0});
        w.bodies.robot.back() = robot;
        w.bodies.kinematic.back() = true;
        model.bodyRestPose.push_back(lamp.dark);
        ScenePrim prim;
        prim.name = lamp.name;
        prim.path = "/traffic/" + lamp.name;
        prim.parent = topPrim;
        prim.typeName = "Mesh";
        prim.body = body.index;
        const Pose disc{
            {0.0, 0.0, 0.0},
            {std::cos(std::numbers::pi / 4.0), 0.0, std::sin(std::numbers::pi / 4.0), 0.0}};
        switch (lamp.kind) {
            case LampKind::Red:
                prim.mesh = lensDisc;
                prim.material = redLit;
                prim.offset = disc;
                break;
            case LampKind::Yellow:
                prim.mesh = lensDisc;
                prim.material = yellowLit;
                prim.offset = disc;
                break;
            case LampKind::Green:
                prim.mesh = lensDisc;
                prim.material = greenLit;
                prim.offset = disc;
                break;
            case LampKind::DontWalk:
                prim.mesh = standing;
                prim.material = redLit;
                break;
            case LampKind::Walk:
                prim.mesh = striding;
                prim.material = greenLit;
                break;
            case LampKind::Wait:
                prim.mesh = waitDisc;
                prim.material = waitLit;
                prim.offset = disc;
                break;
        }
        model.scene.prims.push_back(std::move(prim));
        model.bodyPrim.push_back(static_cast<std::uint32_t>(model.scene.prims.size() - 1));
    }
    return made;
}

const Traffic::Controller* Traffic::controllerOf(int junction) const {
    if (junction < 0) {
        return nullptr;
    }
    const int c = controllerOfJunction_[static_cast<std::size_t>(junction)];
    return c < 0 ? nullptr : &controllers_[static_cast<std::size_t>(c)];
}

Light Traffic::lightOf(std::size_t lane, double t) const {
    const int group = graph_.signalGroup[lane];
    const Controller* controller = controllerOf(graph_.junctionOf[lane]);
    if (controller == nullptr) {
        return lightAt(group, 2, t);
    }
    if (group != controller->group) {
        return Light::Red;
    }
    switch (controller->phase) {
        case Controller::Phase::Green:
            return Light::Green;
        case Controller::Phase::Yellow:
            return Light::Yellow;
        default:
            return Light::Red;
    }
}

Walk Traffic::walkOf(int junction, double /*t*/) const {
    const Controller* controller = controllerOf(junction);
    if (controller == nullptr) {
        return Walk::DontWalk;
    }
    if (controller->phase == Controller::Phase::Walk) {
        return Walk::Walk;
    }
    return controller->phase == Controller::Phase::Flash ? Walk::Flashing : Walk::DontWalk;
}

void Traffic::updateControllers(SignalBus& /*bus*/, double dt) {
    using Phase = Controller::Phase;
    for (Controller& c : controllers_) {
        c.elapsed += dt;
        const auto to = [&c](Phase phase) {
            c.phase = phase;
            c.elapsed = 0.0;
        };
        // Whether anything is still on what a crossing goes over, or on a crossing.
        const auto vehiclesIn = [&] {
            for (const LaneGraph::Crossing& crossing : graph_.crossings) {
                if (crossing.junction != c.junction) {
                    continue;
                }
                for (const auto& [lane, s] : crossing.over) {
                    const std::size_t on = lane;
                    if (std::ranges::any_of(agents_, [on](const Agent& a) {
                            return !a.pedestrian && a.lane == on;
                        })) {
                        return true;
                    }
                }
            }
            return false;
        };
        const auto peopleOn = [&] {
            for (std::size_t k = 0; k < graph_.crossings.size(); ++k) {
                if (graph_.crossings[k].junction == c.junction && onCrossing_[k] > 0) {
                    return true;
                }
            }
            return false;
        };
        const auto nextGreen = [&] {
            c.group = (c.group + 1) % c.groups;
            to(Phase::Green);
        };
        switch (c.phase) {
            case Phase::Green:
                if (c.groups == 1 ? (c.requested && c.elapsed >= kMinGreenSeconds)
                                  : (c.elapsed >= kGreenSeconds ||
                                     (c.requested && c.elapsed >= kMinGreenSeconds))) {
                    to(Phase::Yellow);
                }
                break;
            case Phase::Yellow:
                if (c.elapsed >= kYellowSeconds) {
                    to(Phase::AllRed);
                }
                break;
            case Phase::AllRed:
                if (c.elapsed >= kAllRedSeconds) {
                    if (c.requested) {
                        to(Phase::Clearing);
                    } else {
                        nextGreen();
                    }
                }
                break;
            case Phase::Clearing:
                if (!vehiclesIn() || c.elapsed >= kClearHoldSeconds) {
                    c.requested = false;
                    to(Phase::Walk);
                }
                break;
            case Phase::Walk:
                if (c.elapsed >= kWalkSeconds) {
                    to(Phase::Flash);
                }
                break;
            case Phase::Flash:
                if (c.elapsed >= c.flash) {
                    to(Phase::Emptying);
                }
                break;
            case Phase::Emptying:
                if (!peopleOn() || c.elapsed >= kClearHoldSeconds) {
                    nextGreen();
                }
                break;
        }
    }
}

void Traffic::updateWalkers(SignalBus& bus, double dt, std::size_t /*agentCount*/) {
    using State = Walker::State;
    std::ranges::fill(onCrossing_, 0);
    for (std::size_t i = 0; i < walkers_.size(); ++i) {
        Walker& w = walkers_[i];
        if (const std::span<const double> pose = bus.readN(w.pose); pose.size() == 7) {
            w.x = pose[0];
            w.y = pose[1];
        }
        w.timer += dt;
        std::array<double, 3> gesture{0.0, 0.0, 0.0};
        const auto draw = [&] {
            random::Stream rng(seed_, ++w.decisions, static_cast<std::uint32_t>(i) + 0x10000U);
            return rng.uniform();
        };
        const auto spotOf = [&](std::size_t crossing, bool forward) {
            // Where it waits: the near end of the crossing, a little spread out.
            const Lane& lane = graph_.lanes[graph_.crossings[crossing].lane];
            const auto [p, yaw] = lane.at(forward ? 0.0 : lane.length());
            return std::array<double, 2>{p[0] - (std::sin(yaw) * w.offset),
                                         p[1] + (std::cos(yaw) * w.offset)};
        };
        const auto goTo = [&](double x, double y) {
            bus.writeN(w.goal, std::array<double, 3>{x, y, 1.0});
        };
        const auto arrived = [&](double x, double y) {
            return std::hypot(x - w.x, y - w.y) < kWalkArrived;
        };
        const auto along = [&](std::size_t laneIndex, bool forward) {
            // Toward a point ahead on the lane, on the walker's own side of its middle.
            const Lane& lane = graph_.lanes[laneIndex];
            const double s = lane.project({w.x, w.y, 0.0}).first;
            const auto [p, yaw] = lane.at(
                std::clamp(s + (forward ? kWalkLookAhead : -kWalkLookAhead), 0.0, lane.length()));
            const double left = forward ? w.offset : -w.offset;
            goTo(p[0] - (std::sin(yaw) * left), p[1] + (std::cos(yaw) * left));
            return forward ? lane.length() - s : s;
        };
        // Leaves a lane's end: the next sidewalk, or a crossing; `end` 1 is the lane's end.
        const auto onward = [&](std::size_t from, int end) {
            const std::vector<std::size_t>& next = graph_.walkLinks[from][end == 1 ? 1 : 0];
            std::vector<std::size_t> plain;
            std::vector<std::size_t> crossings;
            for (std::size_t c = 0; c < graph_.crossings.size(); ++c) {
                if (std::ranges::count(next, graph_.crossings[c].lane) > 0) {
                    crossings.push_back(c);
                }
            }
            for (const std::size_t lane : next) {
                if (std::ranges::none_of(crossings, [&](std::size_t c) {
                        return graph_.crossings[c].lane == lane;
                    })) {
                    plain.push_back(lane);
                }
            }
            const Vec3 here =
                graph_.lanes[from].points[end == 1 ? graph_.lanes[from].points.size() - 1 : 0];
            if (!crossings.empty() && (plain.empty() || draw() < kCrossChance)) {
                w.crossing = crossings[static_cast<std::size_t>(
                                           draw() * static_cast<double>(crossings.size())) %
                                       crossings.size()];
                const Lane& lane = graph_.lanes[graph_.crossings[w.crossing].lane];
                w.crossingForward =
                    distance2d(lane.points.front(), here) < distance2d(lane.points.back(), here);
                w.state = State::ToCrossing;
                w.timer = 0.0;
                return;
            }
            if (plain.empty()) {
                w.forward = !w.forward;  // a dead end: turn back
                return;
            }
            w.lane = plain[static_cast<std::size_t>(draw() * static_cast<double>(plain.size())) %
                           plain.size()];
            const Lane& lane = graph_.lanes[w.lane];
            w.forward =
                distance2d(lane.points.front(), here) < distance2d(lane.points.back(), here);
            w.state = State::Along;
            w.timer = 0.0;
        };
        switch (w.state) {
            case State::Along: {
                const double left = along(w.lane, w.forward);
                if (left < kWalkArrived) {
                    onward(w.lane, w.forward ? 1 : 0);
                }
                break;
            }
            case State::ToCrossing: {
                const auto [x, y] = spotOf(w.crossing, w.crossingForward);
                goTo(x, y);
                if (arrived(x, y)) {
                    w.state = State::Waiting;
                    w.timer = 0.0;
                    const LaneGraph::Crossing& crossing = graph_.crossings[w.crossing];
                    const Controller* c = controllerOf(crossing.junction);
                    if (c != nullptr && !c->requested &&
                        walkOf(crossing.junction, 0.0) == Walk::DontWalk && draw() < kPressChance) {
                        w.state = State::ToButton;
                    }
                }
                break;
            }
            case State::ToButton: {
                const Lane& lane = graph_.lanes[graph_.crossings[w.crossing].lane];
                const auto [x, y] = buttonSpot(lane, w.crossingForward ? 0 : 1);
                goTo(x, y);
                if (arrived(x, y) || w.timer > 8.0) {
                    w.state = State::Pressing;
                    w.timer = 0.0;
                }
                break;
            }
            case State::Pressing: {
                goTo(w.x, w.y);
                constexpr double kRaise = 0.35;  // s to lift the arm
                const Lane& pressed = graph_.lanes[graph_.crossings[w.crossing].lane];
                const Head pole = walkHead(pressed, w.crossingForward ? 0 : 1);
                gesture = {std::min(w.timer / kRaise, 1.0), pole.at[0], pole.at[1]};
                if (w.timer >= kPressTime) {
                    const int c = controllerOfJunction_[static_cast<std::size_t>(
                        graph_.crossings[w.crossing].junction)];
                    controllers_[static_cast<std::size_t>(c)].requested = true;
                    w.state = State::Waiting;
                    w.timer = 0.0;
                }
                break;
            }
            case State::Waiting: {
                const LaneGraph::Crossing& crossing = graph_.crossings[w.crossing];
                const auto [x, y] = spotOf(w.crossing, w.crossingForward);
                goTo(x, y);
                const Walk walk = walkOf(crossing.junction, 0.0);
                if (walk == Walk::Walk) {
                    w.state = State::Crossing;
                    w.timer = 0.0;
                } else if (w.timer > kPatience && walk == Walk::DontWalk) {
                    const Controller* c = controllerOf(crossing.junction);
                    if (c != nullptr && !c->requested) {
                        w.state = State::ToButton;  // nobody has pressed: it does
                        w.timer = 0.0;
                    }
                }
                break;
            }
            case State::Crossing: {
                onCrossing_[w.crossing] += 1;
                const std::size_t lane = graph_.crossings[w.crossing].lane;
                if (along(lane, w.crossingForward) < kWalkArrived) {
                    onward(lane, w.crossingForward ? 1 : 0);
                }
                break;
            }
        }
        bus.writeN(w.gesture, gesture);
    }
}

void Traffic::writeLamps(SignalBus& bus, double t) {
    constexpr double kFlashPeriod = 1.0;  // s
    const bool flashOn = std::fmod(t, kFlashPeriod) < kFlashPeriod / 2.0;
    for (std::size_t l = 0; l < lamps_.size(); ++l) {
        const SignalLamp& lamp = lamps_[l];
        bool lit = false;
        if (!zero_) {
            switch (lamp.kind) {
                case LampKind::Red:
                    lit = lightOf(lamp.lane, t) == Light::Red;
                    break;
                case LampKind::Yellow:
                    lit = lightOf(lamp.lane, t) == Light::Yellow;
                    break;
                case LampKind::Green:
                    lit = lightOf(lamp.lane, t) == Light::Green;
                    break;
                default: {
                    const int junction = graph_.crossings[lamp.lane].junction;
                    const Walk walk = walkOf(junction, t);
                    if (lamp.kind == LampKind::Walk) {
                        lit = walk == Walk::Walk || (walk == Walk::Flashing && flashOn);
                    } else if (lamp.kind == LampKind::DontWalk) {
                        lit = walk == Walk::DontWalk;
                    } else {
                        const Controller* c = controllerOf(junction);
                        lit = c != nullptr && c->requested;
                    }
                }
            }
        } else {
            lit = lamp.kind == LampKind::Red || lamp.kind == LampKind::DontWalk;
        }
        bus.writeN(lampOut_[l].command, lit ? lampOut_[l].lit : lampOut_[l].dark);
    }
}

ModuleInfo Traffic::info() const {
    return zero_ ? ModuleInfo{"traffic", "zero", 0, 1, true, true, CapabilityTier::Tier0}
                 : ModuleInfo{"traffic", "builtin", 0, 1, false, true, CapabilityTier::Tier0};
}

Duration Traffic::preferredStep() const {
    return 10_ms;
}

Status Traffic::declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
    graph_ = lanesOf(world);
    roads_ = roadLanesOf(world);
    seed_ = static_cast<std::uint64_t>(config.get<std::int64_t>("seed").value_or(1));
    const World& raw = world.raw();
    start_.clear();
    ego_.clear();
    lightLanes_.clear();
    for (std::uint32_t b = 0; b < raw.bodies.names.size(); ++b) {
        const std::string& name = raw.bodies.names[b];
        const bool vehicle = name.starts_with("traffic_vehicle_");
        const bool pedestrian = name.starts_with("traffic_pedestrian_");
        if ((!vehicle && !pedestrian) || !raw.bodies.kinematic[b]) {
            continue;
        }
        Agent a;
        a.pedestrian = pedestrian;
        a.height = pedestrian ? kPedestrianHeight : kCarSize[2];
        if (vehicle) {
            const std::string number = name.substr(std::string_view("traffic_vehicle_").size());
            const int index = std::atoi(number.c_str());
            const std::size_t kind = vehicleKind(index);
            a.length = kVehicleSizes.at(kind)[0];
            a.width = kVehicleSizes.at(kind)[1];
            a.height = kVehicleSizes.at(kind)[2];
            a.tyre = kShapes.at(kind).tyre;
            const std::string base = "traffic/vehicle/" + number + "/";
            a.wheelOut = d.output(spec(base + "wheel", SignalType::Scalar, units::rad));
            a.brakeOut = d.output(spec(base + "brake", SignalType::Scalar, units::rad));
            a.leftOut = d.output(spec(base + "left", SignalType::Scalar, units::rad));
            a.rightOut = d.output(spec(base + "right", SignalType::Scalar, units::rad));
        }
        a.poseCommand = d.output(
            spec("body/" + name + "/pose_command", SignalType::Pose, units::m, "world_local"));
        // Its start: the nearest lane of its kind to where addAgents put it.
        const Vec3 at = raw.bodies.poseInParent[b].position;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
            const Lane& lane = graph_.lanes[i];
            if (lane.sidewalk != pedestrian) {
                continue;
            }
            const auto [s, lateral] = lane.project(at);
            if (std::abs(lateral) < best) {
                best = std::abs(lateral);
                a.lane = i;
                a.s = s;
            }
        }
        if (!std::isfinite(best)) {
            return Status::error(
                StatusCode::InvalidConfig,
                name + ": no " + std::string(pedestrian ? "sidewalk" : "driving") + " lane for it");
        }
        a.speed = pedestrian ? kWalkSpeed : 0.0;
        start_.push_back(a);
    }
    for (const std::string& name :
         config.get<std::vector<std::string>>("ego_bodies").value_or(std::vector<std::string>{})) {
        Ego e;
        SignalSpec pose = spec("body/" + name + "/pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        SignalSpec twist =
            spec("body/" + name + "/twist", SignalType::Wrench, units::m_per_s, "world_local");
        twist.feedback = true;
        e.pose = d.input(pose);
        e.twist = d.input(twist);
        ego_.push_back(e);
    }
    // Junction lanes in conflict: their paths cross, or they end on the same lane (a merge).
    // Lanes leaving the same lane do not conflict: their vehicles queue on it in order.
    conflicts_.assign(graph_.lanes.size(), {});
    for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
        const Lane& a = graph_.lanes[i];
        if (!a.junction) {
            continue;
        }
        for (std::size_t k = i + 1; k < graph_.lanes.size(); ++k) {
            const Lane& b = graph_.lanes[k];
            if (!b.junction || a.predecessors == b.predecessors) {
                continue;
            }
            const bool merge = std::ranges::any_of(a.successors, [&](std::size_t s) {
                return std::ranges::count(b.successors, s) > 0;
            });
            if (merge || crosses(a.points, b.points)) {
                conflicts_[i].push_back(k);
                conflicts_[k].push_back(i);
            }
        }
    }
    entries_.clear();
    for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
        if (graph_.signalGroup[i] >= 0) {
            lightLanes_.push_back(i);
        }
        const Lane& l = graph_.lanes[i];
        if (!l.junction && !l.sidewalk && l.predecessors.empty()) {
            entries_.push_back(i);
        }
    }
    // The actuated junctions: those with crossings.
    controllers_.clear();
    controllerOfJunction_.assign(static_cast<std::size_t>(graph_.junctions), -1);
    for (const LaneGraph::Crossing& crossing : graph_.crossings) {
        const auto junction = static_cast<std::size_t>(crossing.junction);
        if (controllerOfJunction_[junction] < 0) {
            Controller controller;
            controller.junction = crossing.junction;
            for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
                if (graph_.junctionOf[i] == crossing.junction) {
                    controller.groups = std::max(controller.groups, graph_.signalGroup[i] + 1);
                }
            }
            controllerOfJunction_[junction] = static_cast<int>(controllers_.size());
            controllers_.push_back(controller);
        }
        Controller& controller =
            controllers_[static_cast<std::size_t>(controllerOfJunction_[junction])];
        controller.flash =
            std::max(controller.flash, graph_.lanes[crossing.lane].length() / kClearanceSpeed);
    }
    onCrossing_.assign(graph_.crossings.size(), 0);
    // The lamps.
    lamps_ = signalLamps(graph_);
    lampOut_.clear();
    for (const SignalLamp& lamp : lamps_) {
        LampOut out;
        out.command = d.output(
            spec("body/" + lamp.name + "/pose_command", SignalType::Pose, units::m, "world_local"));
        const auto flat = [](const Pose& pose) {
            return std::array<double, 7>{
                pose.position[0],    pose.position[1],    pose.position[2],   pose.orientation[0],
                pose.orientation[1], pose.orientation[2], pose.orientation[3]};
        };
        out.lit = flat(lamp.lit);
        out.dark = flat(lamp.dark);
        lampOut_.push_back(out);
    }
    // The walkers: people named walker_<i>, moved by the humans domain, sent by their goals.
    walkersStart_.clear();
    for (const SchemaInstance* s : world.instancesOf("HumanAPI_1")) {
        const std::string& person = raw.prims.instanceNames.at(s->prim.index);
        if (!person.starts_with("walker_") || graph_.crossings.empty()) {
            continue;
        }
        Walker walker;
        const std::string base = "human/" + person + "/";
        SignalSpec goal = spec(base + "goal", SignalType::ScalarArray, units::one);
        goal.arrayLength = 3;
        walker.goal = d.output(goal);
        SignalSpec gesture = spec(base + "gesture", SignalType::ScalarArray, units::one);
        gesture.arrayLength = 3;
        walker.gesture = d.output(gesture);
        SignalSpec pose = spec(base + "pose", SignalType::Pose, units::m, "world_local");
        pose.feedback = true;
        pose.optional = true;
        walker.pose = d.input(pose);
        const std::string pelvis = "human_" + person + "_pelvis";
        for (std::size_t b = 0; b < raw.bodies.names.size(); ++b) {
            if (raw.bodies.names[b] == pelvis) {
                walker.x = raw.bodies.poseInParent[b].position[0];
                walker.y = raw.bodies.poseInParent[b].position[1];
            }
        }
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < graph_.lanes.size(); ++i) {
            const Lane& lane = graph_.lanes[i];
            if (!lane.sidewalk || lane.junction) {
                continue;
            }
            const double off = std::abs(lane.project({walker.x, walker.y, 0.0}).second);
            if (off < best) {
                best = off;
                walker.lane = i;
            }
        }
        random::Stream rng(seed_, 0, static_cast<std::uint32_t>(walkersStart_.size()) + 0x10000U);
        walker.forward = rng.uniform() < 0.5;
        walker.offset = (rng.uniform() - 0.5) * 1.6;
        walkersStart_.push_back(walker);
    }
    SignalSpec crossingTable = spec("traffic/crossings", SignalType::ScalarArray, units::one);
    crossingTable.arrayLength =
        static_cast<std::uint32_t>(std::max<std::size_t>(graph_.crossings.size(), 1) * 3);
    crossingsOut_ = d.output(crossingTable);
    count_ = d.output(spec("traffic/agent_count", SignalType::Int, units::one));
    laneChangesOut_ = d.output(spec("traffic/lane_changes", SignalType::Int, units::one));
    SignalSpec grip = spec("env/friction_scale", SignalType::Scalar, units::one);
    grip.optional = true;
    grip.defaultValue = 1.0;
    grip.feedback = true;
    friction_ = d.input(grip);
    SignalSpec table = spec("traffic/agents", SignalType::ScalarArray, units::one);
    table.arrayLength = static_cast<std::uint32_t>(std::max<std::size_t>(start_.size(), 1) * 8);
    table_ = d.output(table);
    SignalSpec lights = spec("traffic/lights", SignalType::ScalarArray, units::one);
    lights.arrayLength =
        static_cast<std::uint32_t>(std::max<std::size_t>(lightLanes_.size(), 1) * 2);
    lights_ = d.output(lights);
    return {};
}

Status Traffic::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    agents_ = start_;
    walkers_ = walkersStart_;
    laneChanges_ = 0;
    for (Controller& controller : controllers_) {
        controller.phase = Controller::Phase::Green;
        controller.group = 0;
        controller.elapsed = 0.0;
        controller.requested = false;
    }
    for (Agent& a : agents_) {
        place(a);
    }
    write(bus, 0.0);
    return {};
}

void Traffic::place(Agent& a) {
    const Lane& lane = graph_.lanes[a.lane];
    auto [p, yaw] = lane.at(a.s);
    // The offset is to the left of the lane's direction, whichever way a walker goes.
    p[0] -= std::sin(yaw) * a.offset;
    p[1] += std::cos(yaw) * a.offset;
    if (a.pedestrian) {
        if (a.speed < 0.0) {
            yaw += std::numbers::pi;
        }
    } else {
        yaw += std::atan2(a.sideRate, std::max(a.speed, 1.0));  // pointing where it is going
    }
    a.position = {p[0], p[1], p[2] + a.height / 2.0};
    a.yaw = yaw;
}

bool Traffic::respawn(Agent& a, std::size_t agentIndex) {
    std::vector<std::size_t> entries;
    for (const std::size_t lane : entries_) {
        const Lane& l = graph_.lanes[lane];
        const bool clear = std::ranges::none_of(agents_, [&](const Agent& o) {
            return !o.pedestrian && o.lane == lane && o.s < kEntryClearance;
        });
        if (clear && std::ranges::none_of(egos_, [&](const Vec3& p) {
                return std::hypot(p[0] - l.points.front()[0], p[1] - l.points.front()[1]) <
                       kEntryClearance;
            })) {
            entries.push_back(lane);
        }
    }
    if (entries.empty()) {
        return false;
    }
    a.lane = choose(a, agentIndex, entries);
    a.s = 0.0;
    const Lane& l = graph_.lanes[a.lane];
    a.speed = std::min(kEntrySpeed, l.speedLimit > 0.0 ? l.speedLimit : kDefaultSpeed);
    a.next.reset();
    if (!l.successors.empty()) {
        a.next = choose(a, agentIndex, l.successors);
    }
    return true;
}

std::size_t Traffic::choose(Agent& a, std::size_t agentIndex,
                            const std::vector<std::size_t>& options) {
    random::Stream rng(seed_, a.decisions++, static_cast<std::uint32_t>(agentIndex));
    return options[static_cast<std::size_t>(rng.uniform() * static_cast<double>(options.size())) %
                   options.size()];
}

void Traffic::considerLaneChange(Agent& a, std::size_t index, double gap, double leaderSpeed,
                                 double v0) {
    constexpr double kFromStart = 6.0;      // m: not while coming out of a junction
    constexpr double kFromLine = 15.0;      // m: not at the line
    constexpr double kHeldUpWithin = 40.0;  // m of gap
    constexpr double kBetterBy = 10.0;      // m more room
    constexpr double kMinFreeGap = 6.0;     // m, front and back
    constexpr double kSafeBraking = -3.0;  // m/s², the most it may ask of the one behind it (MOBIL)
    constexpr double kChangeEvery = 8.0;   // s
    const Lane& lane = graph_.lanes[a.lane];
    if (a.changeWait > 0.0 || lane.junction || lane.sidewalk || std::abs(a.offset) > 0.05 ||
        a.s < kFromStart || lane.length() - a.s < kFromLine || gap > kHeldUpWithin ||
        leaderSpeed > v0 - 2.0) {
        return;
    }
    for (const int side : {1, -1}) {
        const auto target = roads_.neighbour(a.lane, a.s, side);
        if (!target) {
            continue;
        }
        const auto [to, along] = *target;
        if (graph_.lanes[to].sidewalk || graph_.lanes[to].junction) {
            continue;  // beside the outer lane is the pavement
        }
        double front = std::numeric_limits<double>::infinity();
        double frontSpeed = v0;
        double back = std::numeric_limits<double>::infinity();
        double backSpeed = 0.0;
        for (const Agent& o : agents_) {
            if (o.pedestrian || o.lane != to) {
                continue;
            }
            if (o.s >= along) {
                const double room = o.s - along - (a.length + o.length) / 2.0;
                if (room < front) {
                    front = room;
                    frontSpeed = o.speed;
                }
            } else {
                const double room = along - o.s - (a.length + o.length) / 2.0;
                if (room < back) {
                    back = room;
                    backSpeed = o.speed;
                }
            }
        }
        const Lane& into = graph_.lanes[to];
        const bool robotThere = std::ranges::any_of(egos_, [&](const Vec3& p) {
            const auto [s, lateral] = into.project(p);
            return std::abs(lateral) < kEgoLateral && std::abs(s - along) < kAvoidRadius;
        });
        const bool safe = !robotThere && front > kMinFreeGap && back > kMinFreeGap &&
                          idmAcceleration(idm_, backSpeed, v0, back, a.speed) > kSafeBraking &&
                          idmAcceleration(idm_, a.speed, v0, front, frontSpeed) > kSafeBraking;
        if (!safe || front < gap + kBetterBy) {
            continue;
        }
        a.offset = (side > 0 ? -1.0 : 1.0) * (lane.width + into.width) / 2.0;
        a.lane = to;
        a.s = along;
        a.next.reset();
        if (!into.successors.empty()) {
            a.next = choose(a, index, into.successors);
        }
        a.changeWait = kChangeEvery;
        ++laneChanges_;
        return;
    }
}

Status Traffic::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    const double t = context.t.seconds() + dt;
    if (zero_) {
        write(bus, t);
        return {};
    }
    updateWalkers(bus, dt, agents_.size());
    updateControllers(bus, dt);
    // The robots, as obstacles and leaders.
    std::vector<std::pair<Vec3, double>> egos;  // position, speed
    egos_.clear();
    for (const Ego& e : ego_) {
        const std::span<const double> p = bus.readN(e.pose);
        const std::span<const double> v = bus.readN(e.twist);
        egos.push_back({{p[0], p[1], p[2]}, std::hypot(v[0], v[1])});
        egos_.push_back({p[0], p[1], p[2]});
    }
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        const Lane* lanePtr = &graph_.lanes[a.lane];
        if (a.pedestrian) {
            const Lane& lane = *lanePtr;
            // Social force across the sidewalk from the other walkers and the robots.
            double push = 0.0;
            const double sideX = -std::sin(a.yaw);
            const double sideY = std::cos(a.yaw);
            const auto repel = [&](const Vec3& q) {
                const double dx = a.position[0] - q[0];
                const double dy = a.position[1] - q[1];
                const double d = std::hypot(dx, dy);
                if (d < 1e-6 || d > 4.0) {
                    return;
                }
                push += kSocialA * std::exp(-(d - 2.0 * kPedestrianRadius) / kSocialB) *
                        (dx * sideX + dy * sideY) / d;
            };
            for (std::size_t k = 0; k < agents_.size(); ++k) {
                if (k != i && agents_[k].pedestrian) {
                    repel(agents_[k].position);
                }
            }
            for (const auto& [p, v] : egos) {
                repel(p);
            }
            // Sideways: the force's lateral part over a relaxation time, drifting back to the
            // lane's middle; the walker's left is the lane's left when walking along it.
            const double side = a.speed < 0.0 ? -1.0 : 1.0;
            const double limit = std::max(0.0, lane.width / 2.0 - kPedestrianRadius);
            const double lateral =
                std::clamp(side * push * kRelaxation - 0.2 * a.offset, -kMaxSideStep, kMaxSideStep);
            a.offset = std::clamp(a.offset + lateral * dt, -limit, limit);
            a.s += a.speed * dt;
            if (a.s > lane.length() || a.s < 0.0) {
                const bool forward = a.s > lane.length();
                const std::vector<std::size_t>& next =
                    forward ? lane.successors : lane.predecessors;
                if (!next.empty()) {
                    const std::size_t to = choose(a, i, next);
                    a.s = forward ? a.s - lane.length() : graph_.lanes[to].length() + a.s;
                    a.lane = to;
                } else {
                    a.s = std::clamp(a.s, 0.0, lane.length());
                    a.speed = -a.speed;  // turn back
                }
            }
            place(a);
            continue;
        }
        // Vehicles: choose the next lane early, so a leader there is seen.
        const Lane* current = lanePtr;
        if (!a.next && !current->successors.empty()) {
            a.next = choose(a, i, current->successors);
        }
        const double v0 = current->speedLimit > 0.0 ? current->speedLimit : kDefaultSpeed;
        // Slower on what is slippery: down to half speed on ice (illustrative), and gentler.
        const double grip = std::clamp(bus.read(friction_), 0.05, 1.0);
        const double v0Now = v0 * (0.5 + (0.5 * grip));
        Idm idm = idm_;
        idm.a *= std::max(grip, 0.3);
        idm.b *= std::max(grip, 0.3);
        idm.T /= std::max(grip, 0.5);
        double gap = std::numeric_limits<double>::infinity();
        double leaderSpeed = 0.0;
        const auto consider = [&](double g, double v) {
            if (g < gap) {
                gap = g;
                leaderSpeed = v;
            }
        };
        const double half = a.length / 2.0;
        // Anyone still on a crossing ahead, whatever the lights say.
        for (std::size_t k = 0; k < graph_.crossings.size(); ++k) {
            if (onCrossing_[k] == 0) {
                continue;
            }
            for (const auto& [over, s] : graph_.crossings[k].over) {
                const double ahead = over == a.lane              ? s - a.s
                                     : a.next && over == *a.next ? current->length() - a.s + s
                                                                 : -1.0;
                constexpr double kFromCrossing = 2.5;  // m before its middle
                if (ahead > half) {
                    consider(std::max(ahead - half - kFromCrossing, 0.0), 0.0);
                }
            }
        }
        for (std::size_t k = 0; k < agents_.size(); ++k) {
            const Agent& o = agents_[k];
            if (k == i || o.pedestrian) {
                continue;
            }
            if (o.lane == a.lane && o.s > a.s) {
                consider(o.s - a.s - half - o.length / 2.0, o.speed);
            } else if (a.next && o.lane == *a.next) {
                consider(current->length() - a.s + o.s - half - o.length / 2.0, o.speed);
            }
        }
        for (const auto& [p, v] : egos) {
            for (const std::size_t l : {a.lane, a.next.value_or(a.lane)}) {
                const auto [s, lateral] = graph_.lanes[l].project(p);
                if (std::abs(lateral) > kEgoLateral) {
                    continue;
                }
                const double ahead = l == a.lane ? s - a.s : current->length() - a.s + s;
                if (ahead > 0.0 && ahead < kLookAhead) {
                    consider(ahead - half - 2.5, v);  // the robot's half length, about a car's
                }
            }
        }
        // Into a free lane when held up by a vehicle or a robot, not by lights or right of way.
        a.changeWait = std::max(a.changeWait - dt, 0.0);
        const std::size_t laneBefore = a.lane;
        considerLaneChange(a, i, gap, leaderSpeed, v0);
        if (a.lane != laneBefore) {
            gap = std::numeric_limits<double>::infinity();  // this step's leader is left behind
            current = &graph_.lanes[a.lane];
        }
        // Right of way into the junction: wait at the line while a conflicting junction lane is
        // taken, or while a vehicle nearer its own line is about to take one.
        if (a.next && graph_.lanes[*a.next].junction && !current->junction) {
            const double toLine = current->length() - a.s;
            const double reach = a.speed * a.speed / (2.0 * idm_.b) + kYieldMargin;
            if (toLine < reach + kYieldMargin) {
                const std::vector<std::size_t>& against = conflicts_[*a.next];
                bool yield = false;
                for (std::size_t k = 0; k < agents_.size() && !yield; ++k) {
                    const Agent& o = agents_[k];
                    if (k == i || o.pedestrian) {
                        continue;
                    }
                    if (std::ranges::count(against, o.lane) > 0) {
                        yield = true;
                    } else if (o.next && std::ranges::count(against, *o.next) > 0 &&
                               !graph_.lanes[o.lane].junction &&
                               (graph_.signalGroup[o.lane] < 0 ||
                                lightOf(o.lane, t) == Light::Green)) {
                        const double theirs = graph_.lanes[o.lane].length() - o.s;
                        yield =
                            theirs < toLine - 0.5 || (std::abs(theirs - toLine) <= 0.5 && k < i);
                        yield = yield && theirs < reach + kYieldMargin;
                    }
                }
                // Nor into the box when there is no room beyond it: a car waiting in a junction
                // holds up everyone across it, and on a road grid that closes into a gridlock.
                const Lane& through = graph_.lanes[*a.next];
                for (std::size_t k = 0; k < agents_.size() && !yield; ++k) {
                    const Agent& o = agents_[k];
                    if (k == i || o.pedestrian || o.speed > kBoxSpeed ||
                        through.successors.empty()) {
                        continue;
                    }
                    const std::size_t exit = through.successors.front();
                    const bool onExit = o.lane == exit && o.s < a.length + kBoxRoom - kCarSize[0];
                    const bool toExit =
                        graph_.lanes[o.lane].junction &&
                        std::ranges::count(graph_.lanes[o.lane].successors, exit) > 0;
                    yield = onExit || toExit;
                }
                if (yield) {
                    consider(std::max(toLine - half, 0.0), 0.0);
                }
            }
        }
        // Lights: stop at the line on red or yellow when there is room to.
        if (graph_.signalGroup[a.lane] >= 0) {
            const Light light = lightOf(a.lane, t);
            const double toLine = current->length() - a.s - half;
            if (light != Light::Green && toLine > a.speed * a.speed / (2.0 * idm_.b) - 1.0) {
                consider(std::max(toLine, 0.0), 0.0);
            }
        }
        // Over to the middle of a lane changed into.
        constexpr double kSideSpeed = 1.1;  // m/s (a lane in about 3 s, illustrative)
        a.sideRate = std::clamp(-a.offset / dt, -kSideSpeed, kSideSpeed);
        a.offset += a.sideRate * dt;
        const double accel = std::max(idmAcceleration(idm, a.speed, v0Now, gap, leaderSpeed), -8.0);
        a.speed = std::max(0.0, a.speed + accel * dt);
        a.s += a.speed * dt;
        a.wheel = std::fmod(a.wheel - (a.speed * dt / a.tyre), 2.0 * std::numbers::pi);
        constexpr double kBrakeLampAt = -0.4;  // m/s²
        a.braking = accel < kBrakeLampAt || a.speed < 0.05;
        // Indicate before a turn: on the turning lane, or nearing it.
        constexpr double kIndicateFrom = 35.0;  // m before the line
        constexpr double kTurnAtLeast = 0.35;   // rad
        const std::optional<std::size_t> turn =
            current->junction ? std::optional(a.lane)
            : (a.next && graph_.lanes[*a.next].junction && current->length() - a.s < kIndicateFrom)
                ? a.next
                : std::nullopt;
        a.turning = 0;
        if (turn) {
            const Lane& through = graph_.lanes[*turn];
            const double change =
                std::remainder(through.at(through.length()).second - through.at(0.0).second,
                               2.0 * std::numbers::pi);
            a.turning = change > kTurnAtLeast ? 1 : change < -kTurnAtLeast ? -1 : 0;
        }
        if (std::abs(a.offset) > 0.2) {
            a.turning =
                a.offset < 0.0 ? 1 : -1;  // still moving over: left when right of the middle
        }
        while (a.s > graph_.lanes[a.lane].length()) {
            const Lane& done = graph_.lanes[a.lane];
            if (!a.next) {
                // A dead end (the network's edge): come back in at a free entry.
                if (!respawn(a, i)) {
                    a.s = done.length();  // none free: wait
                    a.speed = 0.0;
                }
                break;
            }
            a.s -= done.length();
            a.lane = *a.next;
            a.next.reset();
            const Lane& now = graph_.lanes[a.lane];
            if (!now.successors.empty()) {
                a.next = choose(a, i, now.successors);
            }
        }
        place(a);
    }
    write(bus, t);
    return {};
}

void Traffic::write(SignalBus& bus, double t) {
    std::vector<double> table;
    table.reserve(agents_.size() * 8);
    for (const Agent& a : agents_) {
        std::array<double, 7> pose{};
        if (zero_) {
            pose = {a.position[0], a.position[1], kHidden, 1.0, 0.0, 0.0, 0.0};
        } else {
            const auto q = yawQuat(a.yaw);
            pose = {a.position[0], a.position[1], a.position[2], q[0], q[1], q[2], q[3]};
        }
        bus.writeN(a.poseCommand, pose);
        table.insert(table.end(),
                     {static_cast<double>(a.pedestrian ? kAgentClassPedestrian : kAgentClassCar),
                      a.position[0], a.position[1], pose[2], a.yaw, std::abs(a.speed),
                      a.pedestrian ? 2.0 * kPedestrianRadius : a.length,
                      a.pedestrian ? 2.0 * kPedestrianRadius : a.width});
        if (!a.pedestrian) {
            // Lamps turn on their hinges: 0 out and lit, half a turn into the body.
            constexpr double kBlinkPeriod = 0.7;  // s
            const bool blink = std::fmod(t, kBlinkPeriod) < kBlinkPeriod / 2.0;
            const double in = std::numbers::pi;
            bus.write(a.wheelOut, a.wheel);
            bus.write(a.brakeOut, a.braking && !zero_ ? 0.0 : in);
            bus.write(a.leftOut, a.turning > 0 && blink && !zero_ ? 0.0 : in);
            bus.write(a.rightOut, a.turning < 0 && blink && !zero_ ? 0.0 : in);
        }
    }
    table.resize(std::max<std::size_t>(agents_.size(), 1) * 8, 0.0);
    bus.writeN(table_, table);
    bus.write(count_, zero_ ? 0.0 : static_cast<double>(agents_.size()));
    bus.write(laneChangesOut_, static_cast<double>(laneChanges_));
    std::vector<double> lights;
    for (const std::size_t lane : lightLanes_) {
        lights.push_back(graph_.lanes[lane].road);
        lights.push_back(zero_ ? 0.0 : static_cast<double>(lightOf(lane, t)));
    }
    lights.resize(std::max<std::size_t>(lightLanes_.size(), 1) * 2, 0.0);
    bus.writeN(lights_, lights);
    std::vector<double> crossings;
    for (const LaneGraph::Crossing& crossing : graph_.crossings) {
        const Controller* c = controllerOf(crossing.junction);
        crossings.push_back(graph_.lanes[crossing.lane].road);
        crossings.push_back(zero_ ? 0.0 : static_cast<double>(walkOf(crossing.junction, t)));
        crossings.push_back(!zero_ && c != nullptr && c->requested ? 1.0 : 0.0);
    }
    crossings.resize(std::max<std::size_t>(graph_.crossings.size(), 1) * 3, 0.0);
    bus.writeN(crossingsOut_, crossings);
    writeLamps(bus, t);
}

Status Traffic::saveState(StateWriter& writer) const {
    for (const Agent& a : agents_) {
        writer.put(a.lane);
        writer.put(a.s);
        writer.put(a.speed);
        writer.put(a.offset);
        writer.put(a.next.has_value());
        writer.put(a.next.value_or(0));
        writer.put(a.decisions);
        writer.put(a.wheel);
        writer.put(a.changeWait);
    }
    writer.put(laneChanges_);
    for (const Controller& c : controllers_) {
        writer.put(c.phase);
        writer.put(c.group);
        writer.put(c.elapsed);
        writer.put(c.requested);
    }
    for (const Walker& w : walkers_) {
        writer.put(w.state);
        writer.put(w.lane);
        writer.put(w.forward);
        writer.put(w.crossing);
        writer.put(w.crossingForward);
        writer.put(w.offset);
        writer.put(w.timer);
        writer.put(w.decisions);
    }
    return {};
}

Status Traffic::restoreState(StateReader& reader) {
    for (Agent& a : agents_) {
        bool hasNext = false;
        std::size_t next = 0;
        if (!reader.get(a.lane) || !reader.get(a.s) || !reader.get(a.speed) ||
            !reader.get(a.offset) || !reader.get(hasNext) || !reader.get(next) ||
            !reader.get(a.decisions) || !reader.get(a.wheel) || !reader.get(a.changeWait)) {
            return Status::error(StatusCode::Internal, "traffic: truncated state");
        }
        a.next = hasNext ? std::optional(next) : std::nullopt;
        place(a);
    }
    if (!reader.get(laneChanges_)) {
        return Status::error(StatusCode::Internal, "traffic: truncated state");
    }
    for (Controller& c : controllers_) {
        if (!reader.get(c.phase) || !reader.get(c.group) || !reader.get(c.elapsed) ||
            !reader.get(c.requested)) {
            return Status::error(StatusCode::Internal, "traffic: truncated state");
        }
    }
    for (Walker& w : walkers_) {
        if (!reader.get(w.state) || !reader.get(w.lane) || !reader.get(w.forward) ||
            !reader.get(w.crossing) || !reader.get(w.crossingForward) || !reader.get(w.offset) ||
            !reader.get(w.timer) || !reader.get(w.decisions)) {
            return Status::error(StatusCode::Internal, "traffic: truncated state");
        }
    }
    return {};
}

}  // namespace lagrange::traffic
