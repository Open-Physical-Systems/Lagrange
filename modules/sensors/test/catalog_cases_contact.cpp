// test_sensor_catalog's worlds for the contact and motion kinds: bumper, cliff, wheel_odometry,
// safety_scanner, force_torque, joint_torque, gripper_sensor, proximity, tpms. Each is the
// kind's own test world (test_kinds_<kind>.cpp), set so that its readings change within 1 s.
#include "catalog_rig.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::sensors::catalogtest {
namespace {

constexpr double kGravity = 9.80665;  // m/s², standard gravity (CGPM 1901)

Pose at(double x, double y, double z) {
    return Pose{{x, y, z}};
}

/// Rotation by `angle` about the unit axis (x, y, z).
Pose turned(double angle, double x, double y, double z) {
    const double s = std::sin(angle / 2.0);
    return Pose{{0.0, 0.0, 0.0}, {std::cos(angle / 2.0), s * x, s * y, s * z}};
}

std::array<double, 6> diagonal(double v) {
    return {v, v, v, 0.0, 0.0, 0.0};
}

Feed constantScalar(std::string name, Unit unit, double value) {
    return {specOf(std::move(name), SignalType::Scalar, unit),
            [value](double) { return std::vector<double>{value}; }};
}

/// body/<name>/pose at a position of time (no turn).
Feed poseFeed(const std::string& name, std::function<std::array<double, 3>(double)> where) {
    return {specOf("body/" + name + "/pose", SignalType::Pose, units::m, "world_local"),
            [where = std::move(where)](double t) {
                const auto p = where(t);
                return std::vector<double>{p[0], p[1], p[2], 1.0, 0.0, 0.0, 0.0};
            }};
}

/// body/<name>/twist, a constant velocity.
Feed twistFeed(const std::string& name, std::array<double, 3> v) {
    return {specOf("body/" + name + "/twist", SignalType::Wrench, units::m_per_s, "world_local"),
            [v](double) { return std::vector<double>{v[0], v[1], v[2], 0.0, 0.0, 0.0}; }};
}

// bumper (test_kinds_bumper): a 10 kg box on the floor, pushed by 30 N towards a wall from
// 0.2 s: the floor's friction, then the wall, push back on it.
Setup bumper() {
    constexpr double kFriction = 0.1;
    Setup s;
    World& w = s.world;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, kFriction);
    w.addCollider(kWorldBody, ShapeType::Box, {0.05, 1.0, 0.5}, at(0.5, 0.0, 0.5), kFriction);
    const BodyId robot =
        w.addBody("robot", kWorldBody, at(0.0, 0.0, 0.1), 10.0, {}, {0.11, 0.17, 0.24, 0, 0, 0});
    w.addJoint("robot_free", JointType::Free, robot, {0, 0, 1}, false);
    w.addCollider(robot, ShapeType::Box, {0.2, 0.15, 0.1}, Pose{}, kFriction);
    s.body = robot;
    s.mount = at(0.2, 0.0, 0.0);
    s.rateHz = 100.0;
    s.physics = true;
    s.feeds.push_back(
        {specOf("body/robot/external_wrench", SignalType::Wrench, units::N, "world_local"),
         [](double t) { return std::vector<double>{t >= 0.2 ? 30.0 : 0.0, 0, 0, 0, 0, 0}; }});
    return s;
}

// cliff (test_kinds_cliff): a robot carried over a table's edge (x = 1 m, the floor half a
// metre below): the range reads the table, then +inf and a cliff from 0.5 s.
Setup cliff() {
    Setup s;
    World& w = s.world;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, at(0, 0, -0.5), 1.0);
    w.addCollider(kWorldBody, ShapeType::Box, {1.0, 1.0, 0.05}, at(0.0, 0.0, -0.05), 1.0);
    const BodyId robot = w.addBody("robot", kWorldBody, at(0.0, 0.0, 0.1), 5.0, {}, diagonal(0.1));
    w.addJoint("robot_free", JointType::Free, robot, {0, 0, 1}, false);
    w.addCollider(robot, ShapeType::Box, {1.5, 0.2, 0.08}, Pose{}, 1.0);
    s.body = robot;
    s.mount = at(0.5, 0.0, -0.06);
    s.rateHz = 100.0;
    s.feeds.push_back(poseFeed("robot", [](double t) { return std::array{t, 0.0, 0.1}; }));
    return s;
}

// wheel_odometry (test_kinds_wheel_odometry): two driven wheels and two casters on MuJoCo,
// the right wheel pushed less than the left: it drives and turns.
Setup wheelOdometry() {
    constexpr double kRadius = 0.1;  // m
    constexpr double kTrack = 0.4;   // m
    Setup s;
    World& w = s.world;
    w.addCollider(kWorldBody, ShapeType::Plane, {0, 0, 1}, Pose{}, 1.0);
    const BodyId base =
        w.addBody("base", kWorldBody, at(0, 0, kRadius), 10.0, {}, {0.1, 0.15, 0.2, 0, 0, 0});
    w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
    w.addCollider(base, ShapeType::Box, {0.2, 0.15, 0.03}, Pose{}, 1.0);
    const Pose upright = turned(std::numbers::pi / 2.0, 1, 0, 0);
    for (const auto& [name, y] :
         {std::pair{"left_wheel", kTrack / 2.0}, std::pair{"right_wheel", -kTrack / 2.0}}) {
        const BodyId wheel =
            w.addBody(name, base, at(0, y, 0), 1.0, {}, {0.0026, 0.005, 0.0026, 0, 0, 0});
        w.addJoint(std::string(name) + "_joint", JointType::Continuous, wheel, {0, 1, 0}, true);
        w.addCollider(wheel, ShapeType::Cylinder, {kRadius, 0.02, 0}, upright, 1.0);
    }
    for (const auto& [name, x] :
         {std::pair{"caster_front", 0.15}, std::pair{"caster_rear", -0.15}}) {
        const BodyId ball = w.addBody(name, base, at(x, 0, -0.05), 0.1, {}, diagonal(1e-4));
        w.addJoint(std::string(name) + "_joint", JointType::Spherical, ball, {0, 0, 1}, false);
        w.addCollider(ball, ShapeType::Sphere, {0.05, 0, 0}, Pose{}, 1.0);
    }
    s.body = base;
    s.rateHz = 100.0;
    s.physics = true;
    s.feeds.push_back(constantScalar("joint/left_wheel_joint/torque", units::N_m, 0.3));
    s.feeds.push_back(constantScalar("joint/right_wheel_joint/torque", units::N_m, 0.2));
    return s;
}

// safety_scanner (test_kinds_safety_scanner): a plate driven at 1 m/s towards the scanner,
// its face from 1.5 m to 0.5 m: it enters the 1 m protective rectangle at 0.5 s.
Setup safetyScanner() {
    constexpr double kStart = 1.55;  // m, the plate's centre
    Setup s;
    World& w = s.world;
    const BodyId mount = w.addBody("mount", kWorldBody, at(0, 0, 0.2), 1.0, {}, diagonal(0.01));
    w.addCollider(mount, ShapeType::Cylinder, {0.06, 0.08, 0}, Pose{}, 1.0);
    const BodyId plate = w.addBody("plate", kWorldBody, at(kStart, 0, 0.2), 10.0, {}, diagonal(1));
    w.bodies.kinematic.back() = true;
    w.addCollider(plate, ShapeType::Box, {0.05, 0.5, 0.3}, Pose{}, 1.0);
    s.body = mount;
    s.rateHz = 25.0;
    s.physics = true;
    s.numbers = {{"protective/x_max", 1.0}, {"protective/y_min", -0.5}, {"protective/y_max", 0.5}};
    s.feeds.push_back(
        {specOf("body/plate/pose_command", SignalType::Pose, units::m, "world_local"),
         [](double t) { return std::vector<double>{kStart - t, 0, 0.2, 1, 0, 0, 0}; }});
    return s;
}

// force_torque and joint_torque (test_kinds_force_torque): a 2 kg arm on a hinge whose motor
// holds only half its weight: it swings down, the reaction changing as it goes.
Setup swingingArm() {
    constexpr double kMass = 2.0;  // kg
    constexpr double kArm = 0.5;   // m
    Setup s;
    World& w = s.world;
    const BodyId link = w.addBody("arm", kWorldBody, at(0, 0, 1), kMass, {kArm, 0, 0},
                                  {0.001, 0.01, 0.01, 0, 0, 0});
    w.addJoint("shoulder", JointType::Revolute, link, {0, 1, 0}, true);
    w.joints.lowerLimit.back() = -3.0;
    w.joints.upperLimit.back() = 3.0;
    s.body = link;
    s.rateHz = 1000.0;
    s.physics = true;
    s.feeds.push_back(
        constantScalar("joint/shoulder/torque", units::N_m, -0.5 * kMass * kGravity * kArm));
    return s;
}

// gripper_sensor (test_kinds_gripper_sensor): two prismatic fingers round a 4 cm cube, opening
// from 2 to 3 cm over the second while pressing on it with 5 N.
Setup gripperSensor() {
    Setup s;
    World& w = s.world;
    const BodyId hand = w.addBody("hand", kWorldBody, Pose{}, 1.0, {}, diagonal(1.0));
    w.bodies.kinematic.back() = true;
    const BodyId fl = w.addBody("finger_l", hand, at(0.0, 0.027, 0.0), 0.1, {}, diagonal(1e-4));
    w.addJoint("finger_l_joint", JointType::Prismatic, fl, {0.0, 1.0, 0.0}, true);
    const BodyId fr = w.addBody("finger_r", hand, at(0.0, -0.027, 0.0), 0.1, {}, diagonal(1e-4));
    w.addJoint("finger_r_joint", JointType::Prismatic, fr, {0.0, -1.0, 0.0}, true);
    w.addCollider(fl, ShapeType::Box, {0.005, 0.005, 0.02}, Pose{}, 1.0);
    w.addCollider(fr, ShapeType::Box, {0.005, 0.005, 0.02}, Pose{}, 1.0);
    const BodyId cube = w.addBody("cube", kWorldBody, Pose{}, 1.0, {}, diagonal(1.0));
    w.bodies.kinematic.back() = true;
    w.addCollider(cube, ShapeType::Box, {0.02, 0.02, 0.02}, Pose{}, 1.0);
    s.body = hand;
    s.rateHz = 100.0;
    s.text = {{"fingers", "finger_l_joint finger_r_joint"}};
    const auto still = [](std::array<double, 3> p) { return [p](double) { return p; }; };
    for (const auto& [name, where] : {std::pair{"hand", std::array{0.0, 0.0, 0.0}},
                                      std::pair{"finger_l", std::array{0.0, 0.027, 0.0}},
                                      std::pair{"finger_r", std::array{0.0, -0.027, 0.0}},
                                      std::pair{"cube", std::array{0.0, 0.0, 0.0}}}) {
        s.feeds.push_back(poseFeed(name, still(where)));
        s.feeds.push_back(twistFeed(name, {0.0, 0.0, 0.0}));
    }
    for (const auto& [finger, sign] : {std::pair{"finger_l", 1.0}, std::pair{"finger_r", -1.0}}) {
        const std::string joint = std::string("joint/") + finger + "_joint";
        s.feeds.push_back({specOf(joint + "/position", SignalType::Scalar, units::m),
                           [](double t) { return std::vector<double>{0.02 + 0.01 * t}; }});
        s.feeds.push_back(constantScalar(joint + "/torque", units::N, -5.0));
        s.feeds.push_back({specOf(std::string("body/") + finger + "/contact_force",
                                  SignalType::Vec3, units::N, "world_local"),
                           [sign](double) { return std::vector<double>{0.0, sign * 5.0, 0.0}; }});
    }
    return s;
}

// proximity (test_kinds_proximity): a skin carried towards a wall whose face is at x = 0.3 m:
// out of reach at first, then nearer and nearer.
Setup proximity() {
    Setup s;
    World& w = s.world;
    const BodyId robot = w.addBody("robot", kWorldBody, Pose{}, 1.0, {}, diagonal(1.0));
    w.bodies.kinematic.back() = true;
    w.addCollider(robot, ShapeType::Box, {0.1, 0.1, 0.1}, Pose{}, 0.8);  // its own body
    const BodyId wall = w.addBody("wall", kWorldBody, at(0.35, 0.0, 1.0), 1.0, {}, diagonal(1.0));
    w.addCollider(wall, ShapeType::Box, {0.05, 2.0, 2.0}, Pose{}, 0.8);
    s.body = robot;
    s.rateHz = 50.0;
    s.feeds.push_back(poseFeed("robot", [](double t) { return std::array{0.2 * t, 0.0, 1.0}; }));
    return s;
}

// tpms (test_kinds_tpms): a car with four WheelAPI_1 wheels while the air warms from 0 °C.
Setup tpms() {
    Setup s;
    World& w = s.world;
    const BodyId chassis = w.addBody("chassis", kWorldBody, Pose{}, 1.0, {}, diagonal(1.0));
    w.bodies.kinematic.back() = true;
    for (const char* name : {"fl", "fr", "rl", "rr"}) {
        const BodyId wheel =
            w.addBody(std::string(name) + "_wheel", chassis, Pose{}, 10.0, {}, diagonal(0.5));
        const JointId j = w.addJoint(name, JointType::Revolute, wheel, {0.0, 1.0, 0.0}, true);
        SchemaInstance wheelApi;
        wheelApi.schema = "WheelAPI_1";
        wheelApi.prim = w.joints.prim[j.index];
        w.schemaInstances.push_back(wheelApi);
    }
    s.body = chassis;
    s.rateHz = 10.0;
    s.feeds.push_back({specOf("env/air_temperature", SignalType::Scalar, units::K),
                       [](double t) { return std::vector<double>{273.15 + 20.0 * t}; }});
    s.feeds.push_back(constantScalar("env/pressure", units::Pa, 101325.0));
    return s;
}

}  // namespace

void addContactCases(Cases& cases) {
    cases["bumper"] = {bumper};
    cases["cliff"] = {cliff};
    cases["wheel_odometry"] = {wheelOdometry};
    cases["safety_scanner"] = {safetyScanner};
    cases["force_torque"] = {swingingArm};
    cases["joint_torque"] = {swingingArm};
    cases["gripper_sensor"] = {gripperSensor};
    cases["proximity"] = {proximity};
    cases["tpms"] = {tpms};
}

}  // namespace lagrange::sensors::catalogtest
