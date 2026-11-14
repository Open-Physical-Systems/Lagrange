// test_sensor_catalog's worlds for the scene and radio kinds (sensors.md §9): landing_target,
// line_sensor, tag_reader, wifi_scanner, uwb, particle_sensor, adsb_receiver, v2x, vehicle_can.
// Each is modelled on the kind's own test (test_kinds_<kind>.cpp) with something moving, so its
// signals change over the second. detections has no case: without the camera scene (a GPU path)
// it has no objects and reads nothing.
#include "catalog_rig.hpp"
#include "lagrange/world/Model.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::sensors::catalogtest {

namespace {

using Where = std::function<std::array<double, 3>(double)>;

/// A body that moves by its pose signal (kinematic), at the origin.
BodyId kinematicBody(World& w, const std::string& name) {
    const BodyId b = w.addBody(name, kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    w.bodies.kinematic.back() = true;
    return b;
}

/// body/<name>/pose at `where(t)` with orientation `q` (w, x, y, z), and /twist with `velocity`.
void bodyFeeds(std::vector<Feed>& feeds, const std::string& name, Where where,
               std::array<double, 3> velocity = {0, 0, 0},
               std::array<double, 4> q = {1.0, 0.0, 0.0, 0.0}) {
    feeds.push_back({specOf("body/" + name + "/pose", SignalType::Pose, units::m, "world_local"),
                     [where, q](double t) {
                         const auto p = where(t);
                         return std::vector<double>{p[0], p[1], p[2], q[0], q[1], q[2], q[3]};
                     }});
    feeds.push_back(
        {specOf("body/" + name + "/twist", SignalType::Wrench, units::m_per_s, "world_local"),
         [velocity](double) {
             return std::vector<double>{velocity[0], velocity[1], velocity[2], 0.0, 0.0, 0.0};
         }});
}

Feed constantFeed(std::string name, SignalType type, Unit unit, std::vector<double> value,
                  std::uint32_t length = 0) {
    return {specOf(std::move(name), type, unit, {}, length),
            [value = std::move(value)](double) { return value; }};
}

ScenePrim meshPrim(std::string name, std::string path, std::uint32_t mesh, const Pose& at) {
    ScenePrim p;
    p.name = std::move(name);
    p.path = std::move(path);
    p.mesh = mesh;
    p.offset = at;
    return p;
}

// landing_target: a downward camera on a drone 5 m up flying over a 2 m pad (as
// test_kinds_landing_target), so the target's angles change.
Setup landingTarget() {
    Setup s;
    World& w = s.world;
    s.body = kinematicBody(w, "drone");
    const double h = std::numbers::sqrt2 / 2.0;
    s.mount = Pose{{0, 0, 0}, {h, 0, h, 0}};  // pitched 90°: x down
    s.rateHz = 20.0;
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    constexpr double kPadSize = 2.0;         // m
    constexpr double kPadThickness = 0.005;  // m
    s.scene.emplace();
    s.scene->meshes.push_back(boxMesh(kPadSize, kPadSize, kPadThickness));
    ScenePrim field;
    field.name = "field";
    field.path = "/field";
    s.scene->prims.push_back(field);
    ScenePrim pad =
        meshPrim("landing_pad", "/field/landing_pad", 0, Pose{{1.0, 0.5, kPadThickness / 2.0}});
    pad.parent = 0;
    s.scene->prims.push_back(pad);
    bodyFeeds(s.feeds, "drone", [](double t) { return std::array<double, 3>{0.5 + t, 0.0, 5.0}; });
    return s;
}

// line_sensor: an array 5 cm over the floor sliding sideways across a tape (as
// test_kinds_line_sensor), so the line's offset changes.
Setup lineSensor() {
    Setup s;
    World& w = s.world;
    s.body = kinematicBody(w, "agv");
    s.rateHz = 50.0;
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    s.scene.emplace();
    s.scene->meshes.push_back(boxMesh(2.0, 0.03, 0.0005));
    s.scene->prims.push_back(
        meshPrim("tape_main", "/warehouse/tape_main", 0, Pose{{0.0, 0.023, 0.00025}}));
    bodyFeeds(s.feeds, "agv",
              [](double t) { return std::array<double, 3>{0.0, -0.05 + 0.1 * t, 0.05}; });
    return s;
}

// tag_reader, mode qr: a reader 20 cm up driving over a QR tag turned 30° (as
// test_kinds_tag_reader), so it finds the tag and its offset changes.
Setup tagReader() {
    Setup s;
    World& w = s.world;
    s.body = kinematicBody(w, "agv");
    s.rateHz = 20.0;
    s.text["mode"] = "qr";
    w.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 1.0);
    constexpr double kThickness = 0.001;  // m
    constexpr double kYaw = std::numbers::pi / 6.0;
    s.scene.emplace();
    s.scene->meshes.push_back(boxMesh(0.05, 0.05, kThickness));
    s.scene->prims.push_back(meshPrim(
        "qr_0007", "/warehouse/qr_0007", 0,
        Pose{{0.3, 0.1, kThickness / 2.0}, {std::cos(kYaw / 2.0), 0, 0, std::sin(kYaw / 2.0)}}));
    s.scene->prims.push_back(
        meshPrim("rfid_12", "/warehouse/rfid_12", 0, Pose{{2.0, 0.0, kThickness / 2.0}}));
    bodyFeeds(s.feeds, "agv", [](double t) { return std::array<double, 3>{0.6 * t, 0.1, 0.2}; });
    return s;
}

// wifi_scanner: a robot driving away from two access points (as test_kinds_wifi_scanner). Its
// first scan, at the first step, turns the readings from NaN to levels; the default scan time
// (2 s) holds them after that.
Setup wifiScanner() {
    Setup s;
    s.body = kinematicBody(s.world, "robot");
    s.rateHz = 10.0;
    s.text["access_points"] = "0 0 1; 0 30 1";
    bodyFeeds(s.feeds, "robot",
              [](double t) { return std::array<double, 3>{1.0 + 9.0 * t, 0, 1}; });
    return s;
}

// uwb: a tag driving between two anchors (as test_kinds_uwb), so its ranges change.
Setup uwb() {
    Setup s;
    s.body = kinematicBody(s.world, "robot");
    s.rateHz = 10.0;
    s.text["anchors"] = "0 0 3; 11 2 1";
    bodyFeeds(s.feeds, "robot",
              [](double t) { return std::array<double, 3>{1.0 + 5.0 * t, 2, 1}; });
    return s;
}

// particle_sensor: weather dust that thickens over the second and lifted dust (as
// test_kinds_particle_sensor), read through the lag.
Setup particleSensor() {
    Setup s;
    s.body = kinematicBody(s.world, "robot");
    s.rateHz = 10.0;
    constexpr double kUg = 1e-9;  // kg/m³ per µg/m³
    s.feeds.push_back({specOf("env/dust/concentration", SignalType::Scalar, units::kg_per_m3),
                       [](double t) { return std::vector<double>{(50.0 + 200.0 * t) * kUg}; }});
    s.feeds.push_back(
        constantFeed("env/dust/median_diameter", SignalType::Scalar, units::m, {2e-6}));
    s.feeds.push_back(
        constantFeed("env/dust/downwash", SignalType::Scalar, units::kg_per_m3, {30.0 * kUg}));
    return s;
}

// adsb_receiver: a station at the origin hearing a quadcopter (robot 1, a RotorAPI_1 joint)
// flying past 1 km away (as test_kinds_adsb_receiver).
Setup adsbReceiver() {
    Setup s;
    World& w = s.world;
    s.rateHz = 10.0;
    w.robots = {"station", "quad"};
    s.body = w.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("frame_free", JointType::Free, s.body, {0, 0, 1}, false);
    const BodyId drone =
        w.addBody("quad_base", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("quad_free", JointType::Free, drone, {0, 0, 1}, false);
    w.bodies.robot.back() = 1;
    const BodyId prop =
        w.addBody("quad_prop", drone, Pose{}, 0.01, {}, {1e-4, 1e-4, 1e-4, 0, 0, 0});
    w.addJoint("rotor_0", JointType::Revolute, prop, {0, 0, 1}, true);
    w.bodies.robot.back() = 1;
    SchemaInstance rotor;
    rotor.schema = "RotorAPI_1";
    rotor.prim = w.joints.prim.back();
    w.schemaInstances.push_back(rotor);
    bodyFeeds(s.feeds, "frame", [](double) { return std::array<double, 3>{0, 0, 0}; });
    constexpr double kSpeed = 10.0;  // m/s
    bodyFeeds(s.feeds, "quad_base",
              [](double t) { return std::array<double, 3>{1000.0 + kSpeed * t, 0.0, 100.0}; },
              {kSpeed, 0.0, 0.0});
    return s;
}

// v2x: a unit at the origin with a moving car, a pedestrian and two signal heads (as
// test_kinds_v2x), its messages lost at random with distance.
Setup v2x() {
    Setup s;
    World& w = s.world;
    s.rateHz = 10.0;
    s.body = w.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    w.addJoint("frame_free", JointType::Free, s.body, {0, 0, 1}, false);
    const auto kinematic = [&w](const std::string& name, std::array<double, 3> at) {
        Pose p;
        p.position = at;
        w.addBody(name, kWorldBody, p, 0.1, {}, {1, 1, 1, 0, 0, 0});
        w.bodies.kinematic.back() = true;
    };
    kinematic("traffic_vehicle_0", {50, 0, 0});
    kinematic("traffic_pedestrian_1", {0, 300, 0});
    kinematic("traffic_lamp_7_red", {100, 5, 3.4});
    kinematic("traffic_lamp_7_green", {100, 5, 3.0});
    kinematic("traffic_lamp_3_red", {20, -5, 3.4});
    bodyFeeds(s.feeds, "frame", [](double) { return std::array<double, 3>{0, 0, 0}; });
    // class, x, y, z, yaw, speed, length, width per agent; the car drives on at 8 m/s.
    s.feeds.push_back(
        {specOf("traffic/agents", SignalType::ScalarArray, units::one, {}, 16), [](double t) {
             return std::vector<double>{0, 50.0 + 8.0 * t, 0,    0,   1.5, 8.0, 4.5, 1.8, 1, 0, 300,
                                        0, -1.0,           1.34, 0.5, 0.5};
         }});
    // per lane with a light, in lane order: road id, state; lane 3 turns from green to yellow.
    s.feeds.push_back(
        {specOf("traffic/lights", SignalType::ScalarArray, units::one, {}, 4),
         [](double t) { return std::vector<double>{12, t < 0.5 ? 2.0 : 1.0, 31, 0}; }});
    return s;
}

// The car of vehicle_can's world.
constexpr double kRadius = 0.3;     // m, wheel radius
constexpr double kL = 2.8;          // m, wheelbase
constexpr double kHalfTrack = 0.8;  // m
constexpr double kV = 10.0;         // m/s
constexpr double kOmega = 0.2;      // rad/s
constexpr double kDelta = 0.05;     // rad, bicycle steering angle

// vehicle_can: a car (VehicleAPI_1 chassis, four WheelAPI_1 wheels, steered front) driving a
// circle at 10 m/s (as test_kinds_vehicle_can).
Setup vehicleCan() {
    Setup s;
    World& w = s.world;
    s.rateHz = 100.0;
    w.robots = {"car"};
    const BodyId chassis = w.addBody("chassis", kWorldBody, Pose{}, 1500.0, {}, {1, 1, 1, 0, 0, 0});
    w.addJoint("chassis_free", JointType::Free, chassis, {0, 0, 1}, false);
    s.body = chassis;
    SchemaInstance v;
    v.schema = "VehicleAPI_1";
    v.prim = w.bodies.prim.back();
    w.schemaInstances.push_back(v);
    const auto wheel = [&w, chassis](const std::string& side, double x, double y, bool front) {
        Pose at;
        at.position = {x, y, 0.0};
        BodyId parent = chassis;
        std::optional<PrimId> steer;
        if (front) {
            parent = w.addBody(side + "_knuckle", chassis, at, 5.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
            w.addJoint(side + "_steer", JointType::Revolute, parent, {0, 0, 1}, true);
            steer = w.joints.prim.back();
            at = Pose{};
        }
        const BodyId body =
            w.addBody(side + "_wheel", parent, at, 10.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
        w.addJoint(side + "_spin", JointType::Continuous, body, {0, 1, 0}, !front);
        SchemaInstance inst;
        inst.schema = "WheelAPI_1";
        inst.prim = w.joints.prim.back();
        inst.reals = {{"wheel:radius", kRadius}};
        inst.tokens = {{"wheel:axle", front ? "front" : "rear"}};
        inst.integers = {{"wheel:steered", front ? 1 : 0}};
        if (steer) {
            inst.relationships = {{"wheel:steerJoint", {*steer}}};
        }
        w.schemaInstances.push_back(inst);
    };
    wheel("fl", kL / 2.0, kHalfTrack, true);
    wheel("fr", kL / 2.0, -kHalfTrack, true);
    wheel("rl", -kL / 2.0, kHalfTrack, false);
    wheel("rr", -kL / 2.0, -kHalfTrack, false);
    s.feeds.push_back(
        {specOf("body/chassis/pose", SignalType::Pose, units::m, "world_local"), [](double t) {
             const double yaw = kOmega * t;
             return std::vector<double>{kV / kOmega * std::sin(yaw),
                                        kV / kOmega * (1.0 - std::cos(yaw)),
                                        0.0,
                                        std::cos(yaw / 2),
                                        0,
                                        0,
                                        std::sin(yaw / 2)};
         }});
    s.feeds.push_back(
        {specOf("body/chassis/twist", SignalType::Wrench, units::m_per_s, "world_local"),
         [](double t) {
             return std::vector<double>{
                 kV * std::cos(kOmega * t), kV * std::sin(kOmega * t), 0.0, 0.0, 0.0, kOmega};
         }});
    for (const char* j : {"fl_spin", "fr_spin", "rl_spin", "rr_spin"}) {
        s.feeds.push_back(
            {specOf(std::string("joint/") + j + "/position", SignalType::Scalar, units::rad),
             [](double t) { return std::vector<double>{kV / kRadius * t}; }});
        s.feeds.push_back(constantFeed(std::string("joint/") + j + "/velocity", SignalType::Scalar,
                                       units::rad_per_s, {kV / kRadius}));
    }
    const auto ackermann = [](double delta, double y) {
        const double t = std::tan(delta);
        return std::atan(kL * t / (kL - y * t));
    };
    s.feeds.push_back(constantFeed("joint/fl_steer/position", SignalType::Scalar, units::rad,
                                   {ackermann(kDelta, kHalfTrack)}));
    s.feeds.push_back(constantFeed("joint/fr_steer/position", SignalType::Scalar, units::rad,
                                   {ackermann(kDelta, -kHalfTrack)}));
    return s;
}

}  // namespace

void addSceneAndRadioCases(Cases& cases) {
    cases["landing_target"] = {landingTarget};
    cases["line_sensor"] = {lineSensor};
    cases["tag_reader"] = {tagReader};
    cases["wifi_scanner"] = {wifiScanner};
    cases["uwb"] = {uwb};
    cases["particle_sensor"] = {particleSensor};
    cases["adsb_receiver"] = {adsbReceiver};
    cases["v2x"] = {v2x};
    cases["vehicle_can"] = {vehicleCan};
}

}  // namespace lagrange::sensors::catalogtest
