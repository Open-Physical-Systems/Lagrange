// test_sensor_catalog's worlds for the power kinds, optical_flow and imaging_radar: each kind gets
// what it reads, changing over the second (plan R10). event_camera and stereo_camera render, so
// the test skips them. Modelled on test_kinds_<kind>.cpp.
#include "catalog_rig.hpp"

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

namespace lagrange::sensors::catalogtest {
namespace {

/// A feed of one scalar, f(t).
Feed scalar(std::string name, Unit unit, std::function<double(double)> f) {
    return {specOf(std::move(name), SignalType::Scalar, unit),
            [f = std::move(f)](double t) { return std::vector<double>{f(t)}; }};
}

/// A prim `/robot/<name>` carrying `schema`; returns the instance (valid until the next push).
SchemaInstance& addSchema(World& world, const std::string& name, const std::string& schema) {
    SchemaInstance s;
    s.schema = schema;
    s.prim = world.addPrim("/robot/" + name, name);
    world.schemaInstances.push_back(std::move(s));
    return world.schemaInstances.back();
}

BodyId addBase(World& world) {
    return world.addBody("base", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
}

/// A kinematic body's pose (moving along x from `x0` at `vx`, height `z`) and twist feeds.
std::vector<Feed> movingAlongX(const std::string& body, double x0, double vx, double z,
                               std::function<double(double)> extraVx = nullptr) {
    const auto speed = [vx, extraVx](double t) { return vx + (extraVx ? extraVx(t) : 0.0); };
    return {
        Feed{specOf("body/" + body + "/pose", SignalType::Pose, units::m, "world_local"),
             [x0, vx, z, extraVx](double t) {
                 // The extra speed is a ramp a·t: its distance a·t²/2.
                 const double extra = extraVx ? extraVx(t) * t / 2.0 : 0.0;
                 return std::vector<double>{x0 + vx * t + extra, 0.0, z, 1.0, 0.0, 0.0, 0.0};
             }},
        Feed{specOf("body/" + body + "/twist", SignalType::Wrench, units::m_per_s, "world_local"),
             [speed](double t) { return std::vector<double>{speed(t), 0.0, 0.0, 0.0, 0.0, 0.0}; }},
    };
}

}  // namespace

void addPowerAndCameraCases(Cases& cases) {
    // The pack's fuel gauge on a draining 4S battery that warms.
    cases["battery_monitor"] = {[] {
        Setup s;
        s.body = addBase(s.world);
        addSchema(s.world, "pack", "BatteryAPI_1").reals["elec:battery:capacity"] = 18000.0;
        s.rateHz = 10.0;
        s.feeds = {
            scalar("battery/pack/terminal_voltage", units::V,
                   [](double t) { return 16.4 - 0.7 * t; }),
            scalar("battery/pack/current", units::A, [](double t) { return 3.0 + 4.0 * t; }),
            scalar("battery/pack/soc", units::one, [](double t) { return 0.9 - 0.3 * t; }),
            scalar("thermal/pack/temperature", units::K, [](double t) { return 300.0 + 2.0 * t; }),
        };
        return s;
    }};

    // An INA226 on a motor's bus current, which rises and swings.
    cases["current_sensor"] = {[] {
        Setup s;
        s.body = addBase(s.world);
        s.rateHz = 10.0;
        s.feeds = {scalar("motor/m1/bus_current", units::A, [](double t) {
            return 2.0 + 6.0 * t + std::sin(2.0 * std::numbers::pi * 3.0 * t);
        })};
        s.text = {{"signal", "motor/m1/bus_current"}};
        return s;
    }};

    // An INA226 on a sagging bus.
    cases["voltage_sensor"] = {[] {
        Setup s;
        s.body = addBase(s.world);
        s.rateHz = 10.0;
        s.feeds = {scalar("bus/main/voltage", units::V, [](double t) { return 16.8 - 1.5 * t; })};
        s.text = {{"signal", "bus/main/voltage"}};
        return s;
    }};

    // KISS telemetry of a 7-pole-pair motor spinning up on its bus.
    cases["esc_telemetry"] = {[] {
        Setup s;
        s.body = addBase(s.world);
        const PrimId joint = s.world.addPrim("/robot/rotor1", "rotor1");
        const PrimId bus = addSchema(s.world, "main", "ElectricalBusAPI_1").prim;
        SchemaInstance& m = addSchema(s.world, "m1", "ElectricalMotorAPI_1");
        m.integers["elec:polePairs"] = 7;
        m.relationships["elec:joint"] = {joint};
        m.relationships["elec:powerPort"] = {bus};
        s.rateHz = 100.0;
        s.feeds = {
            scalar("joint/rotor1/velocity", units::rad_per_s,
                   [](double t) { return 300.0 + 900.0 * t; }),
            scalar("motor/m1/bus_current", units::A, [](double t) { return 2.0 + 20.0 * t; }),
            scalar("bus/main/voltage", units::V, [](double t) { return 16.2 - 0.8 * t; }),
            scalar("thermal/m1/temperature", units::K, [](double t) { return 310.0 + 15.0 * t; }),
        };
        s.text = {{"joint", "rotor1"}};
        return s;
    }};

    // A load cell at the body end of a tether that is pulled up and let go (both branches).
    cases["tension_sensor"] = {[] {
        Setup s;
        s.body = addBase(s.world);
        addSchema(s.world, "leash", "TetherAPI_1");
        s.rateHz = 80.0;
        const auto load = [](double t) { return t < 0.5 ? 600.0 * t : 600.0 * (1.0 - t); };
        s.feeds = {scalar("tether/leash/tension_attach", units::N, load),
                   scalar("tether/leash/tension_anchor", units::N, load)};
        return s;
    }};

    // A downward flow sensor on a drone 2 m over the ground, speeding up along x.
    cases["optical_flow"] = {[] {
        Setup s;
        s.body = s.world.addBody("drone", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
        s.world.bodies.kinematic.back() = true;
        s.world.addCollider(kWorldBody, ShapeType::Plane, {}, Pose{}, 1.0);
        const double h = std::numbers::sqrt2 / 2.0;
        s.mount = Pose{{0, 0, 0}, {h, 0, h, 0}};  // its x pitched to −z
        s.rateHz = 100.0;
        s.feeds = movingAlongX("drone", 0.0, 0.5, 2.0, [](double t) { return 2.0 * t; });
        return s;
    }};

    // A 4D radar 1 m up looking along +x at a plate coming towards it from 20 m.
    cases["imaging_radar"] = {[] {
        Setup s;
        Pose mountPose;
        mountPose.position = {0.0, 0.0, 1.0};
        s.body = s.world.addBody("mount", kWorldBody, mountPose, 1.0, {}, {1, 1, 1, 0, 0, 0});
        Pose platePose;
        platePose.position = {20.0, 0.0, 1.0};
        const BodyId plate =
            s.world.addBody("target", kWorldBody, platePose, 10.0, {}, {1, 1, 1, 0, 0, 0});
        s.world.bodies.kinematic.back() = true;
        s.world.addCollider(plate, ShapeType::Box, {0.01, 1.0, 0.5}, Pose{}, 0.8);
        s.rateHz = 20.0;
        s.feeds = movingAlongX("target", 20.0, -5.0, 1.0);
        // The radar's required settings (no defaults): range, fields of view, power, noise floor.
        s.numbers = {{"max_range", 100.0},
                     {"azimuth_fov", 0.5},
                     {"elevation_fov", 0.2},
                     {"tx_power", 0.01},
                     {"noise_floor", 1e-18}};
        return s;
    }};
}

}  // namespace lagrange::sensors::catalogtest
