// test_sensor_catalog's worlds for the environment kinds (sensors.md §9): anemometer, airspeed,
// rangefinder, light_sensor, sun_sensor, rain_sensor, humidity_sensor, visibility_sensor, sonar.
// Each world varies what the kind reads over the second, as its own test_kinds_<kind>.cpp does.
#include "catalog_rig.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

namespace lagrange::sensors::catalogtest {
namespace {

constexpr double kPi = std::numbers::pi;

/// A free body "frame" carrying the sensor, moved by `pose` and `twist` feeds.
void freeFrame(Setup& s, std::function<std::vector<double>(double)> pose,
               std::function<std::vector<double>(double)> twist) {
    s.body = s.world.addBody("frame", kWorldBody, Pose{}, 1.0, {}, {0.1, 0.1, 0.1, 0, 0, 0});
    s.world.addJoint("frame_free", JointType::Free, s.body, {0, 0, 1}, false);
    s.feeds.push_back(
        {specOf("body/frame/pose", SignalType::Pose, units::m, "world_local"), std::move(pose)});
    s.feeds.push_back(
        {specOf("body/frame/twist", SignalType::Wrench, units::m_per_s, "world_local"),
         std::move(twist)});
}

/// A frame held at the origin, at rest.
void stillFrame(Setup& s) {
    freeFrame(
        s, [](double) { return std::vector<double>{0, 0, 0, 1, 0, 0, 0}; },
        [](double) { return std::vector<double>{0, 0, 0, 0, 0, 0}; });
}

Feed scalar(std::string name, Unit unit, std::function<double(double)> f) {
    return {specOf(std::move(name), SignalType::Scalar, unit),
            [f = std::move(f)](double t) { return std::vector<double>{f(t)}; }};
}

/// The weather's sun at elevation `degrees(t)`, from the south (ENU, world_local).
Feed sunAt(std::function<double(double)> degrees) {
    return {specOf("env/sun_direction", SignalType::Vec3, units::one, "world_local"),
            [degrees = std::move(degrees)](double t) {
                const double e = degrees(t) * kPi / 180.0;
                return std::vector<double>{0.0, -std::cos(e), std::sin(e)};
            }};
}

/// The sensor's +x turned to +z (up).
Pose facingUp() {
    return {{0, 0, 0}, {std::sqrt(0.5), 0.0, -std::sqrt(0.5), 0.0}};
}

/// The sensor's +x turned to −z (down).
Pose facingDown() {
    return {{0, 0, 0}, {std::cos(kPi / 4.0), 0.0, std::sin(kPi / 4.0), 0.0}};
}

}  // namespace

void addEnvironmentCases(Cases& cases) {
    // The wind turns and strengthens; the anemometer stands still.
    cases["anemometer"] = {[] {
        Setup s;
        stillFrame(s);
        s.rateHz = 50.0;
        s.feeds.push_back(
            {specOf("env/wind/velocity", SignalType::Vec3, units::m_per_s, "world_local"),
             [](double t) { return std::vector<double>{3.0 + 2.0 * t, 1.0 - t, 0.2}; }});
        return s;
    }};
    // A probe accelerating through still air at the site's density.
    cases["airspeed"] = {[] {
        Setup s;
        freeFrame(
            s, [](double) { return std::vector<double>{0, 0, 0, 1, 0, 0, 0}; },
            [](double t) { return std::vector<double>{10.0 + 5.0 * t, 0, 0, 0, 0, 0}; });
        s.rateHz = 50.0;
        s.config.values["air_density"] = 1.2;
        s.config.values["ambient_temperature"] = 288.15;
        return s;
    }};
    // Driving at 0.5 m/s toward a wall 5 m ahead.
    cases["rangefinder"] = {[] {
        Setup s;
        freeFrame(
            s, [](double t) { return std::vector<double>{0.5 * t, 0, 0, 1, 0, 0, 0}; },
            [](double) { return std::vector<double>{0.5, 0, 0, 0, 0, 0}; });
        s.world.addCollider(s.body, ShapeType::Sphere, {0.2, 0, 0}, Pose{}, 1.0);
        s.world.addCollider(kWorldBody, ShapeType::Box, {0.05, 2.0, 2.0}, Pose{{5.0, 0, 0}}, 1.0);
        s.rateHz = 100.0;
        return s;
    }};
    // Facing up under a low sun that rises from 8° to 16° (the weather's sun direction).
    cases["light_sensor"] = {[] {
        Setup s;
        stillFrame(s);
        s.mount = facingUp();
        s.rateHz = 10.0;
        s.feeds.push_back(sunAt([](double t) { return 8.0 + 8.0 * t; }));
        return s;
    }};
    // Facing up, the sun crossing its field of view from 40° to 50° of elevation.
    cases["sun_sensor"] = {[] {
        Setup s;
        stillFrame(s);
        s.mount = facingUp();
        s.rateHz = 20.0;
        s.feeds.push_back(sunAt([](double t) { return 40.0 + 10.0 * t; }));
        return s;
    }};
    // Rain getting heavier.
    cases["rain_sensor"] = {[] {
        Setup s;
        stillFrame(s);
        s.rateHz = 10.0;
        s.feeds.push_back(
            scalar("env/rain_rate", units::m_per_s, [](double t) { return 1e-6 * (1.0 + t); }));
        return s;
    }};
    // Damper and warmer air.
    cases["humidity_sensor"] = {[] {
        Setup s;
        stillFrame(s);
        s.rateHz = 10.0;
        s.feeds.push_back(
            scalar("env/relative_humidity", units::one, [](double t) { return 0.5 + 0.2 * t; }));
        s.feeds.push_back(
            scalar("env/air_temperature", units::K, [](double t) { return 290.0 + 2.0 * t; }));
        return s;
    }};
    // Fog thickening.
    cases["visibility_sensor"] = {[] {
        Setup s;
        stillFrame(s);
        s.rateHz = 10.0;
        s.feeds.push_back(scalar("env/extinction/visible", units::per_m,
                                 [](double t) { return 0.01 * (1.0 + t); }));
        return s;
    }};
    // Looking down from 5 m under the surface of a 20 m deep pond, sinking 2 m in the second.
    cases["sonar"] = {[] {
        Setup s;
        freeFrame(
            s, [](double t) { return std::vector<double>{0, 0, -5.0 - 2.0 * t, 1, 0, 0, 0}; },
            [](double) { return std::vector<double>{0, 0, -2.0, 0, 0, 0}; });
        s.mount = facingDown();
        s.rateHz = 10.0;
        const PrimId prim = s.world.addPrim("/pond", "pond");
        SchemaInstance water;
        water.schema = "WaterAPI_1";
        water.prim = prim;
        water.reals = {{"water:sizeX", 100.0}, {"water:sizeY", 100.0}, {"water:depth", 20.0}};
        s.world.schemaInstances.push_back(water);
        return s;
    }};
}

}  // namespace lagrange::sensors::catalogtest
