#pragma once
// Private to lagrange_sensors: what every built-in custom sensor kind shares (sensors.md §9).
// A kind is a custom sensor (`SensorType::Custom`, URDF <sensor type="KIND">, USD
// CustomSensorAPI_1 `sensor:custom:kind`) that Lagrange simulates itself, with no plugin. Each kind
// lives in its own file in this directory and is listed once in Kinds.cpp.

#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/module/IModule.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace lagrange::sensors::kinds {

/// What a kind's factory may use besides the world.
struct KindContext {
    std::shared_ptr<gpu::RayContext> rayContext;     // null: cast on the CPU
    std::shared_ptr<const CameraScene> cameraScene;  // null: no camera rendering
    const Scene* scene{nullptr};  // the model's scene, only while the factory runs; may be null
};

inline SignalSpec kindSpec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

/// A number parameter of the sensor (World::Sensors::settings, SI units), or `fallback`.
inline double kindSetting(const World& world, std::uint32_t sensor, std::string_view key,
                          double fallback) {
    const auto& map = world.sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

/// A text parameter of the sensor (World::Sensors::text), or `fallback`.
inline std::string kindText(const World& world, std::uint32_t sensor, std::string_view key,
                            std::string fallback = {}) {
    const auto& map = world.sensors.text[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

// --- names in a run of several robots -----------------------------------------------------
// A robot added to a run (spawned, --add-robot) has every body, joint, sensor and component
// renamed "<robot>_<name>" (prefixModel), while its sensors' text parameters still give the
// names of its file. Read as they are, they named the first robot's parts: a spawned copy's
// wheel_odometry read the first robot's wheels, its battery_monitor refused "several batteries".

inline constexpr std::uint32_t kNoRobot = 0xFFFF'FFFFU;

/// The robot a body belongs to (World::robots index), kNoRobot for the world.
inline std::uint32_t robotOfBody(const World& world, BodyId body) {
    return body == kWorldBody || body.index >= world.bodies.robot.size()
               ? kNoRobot
               : world.bodies.robot[body.index];
}
inline std::uint32_t robotOfSensor(const World& world, std::uint32_t sensor) {
    return robotOfBody(world, world.sensors.body[sensor]);
}
/// Whether `prim` is in the sensor's robot's tree: under its top prim ("/quad_x500", "/q2" for
/// a copy added as q2; World::robots holds the top prims' names). A component such as a
/// battery need not sit under a body.
inline bool onSensorsRobot(const World& world, std::uint32_t sensor, PrimId prim) {
    const std::uint32_t robot = robotOfSensor(world, sensor);
    if (robot == kNoRobot || robot >= world.robots.size() ||
        prim.index >= world.prims.paths.size()) {
        return false;
    }
    const std::string_view path = world.prims.paths[prim.index];
    return path.substr(0, path.find('/', 1)) == "/" + world.robots[robot];
}

/// `name` as the sensor's own robot calls it: "<robot>_<name>" when the sensor's robot is a
/// prefixed one and `exists` knows that name, else `name` unchanged.
template <typename Exists>
std::string ownName(const World& world, std::uint32_t sensor, const std::string& name,
                    const Exists& exists) {
    const std::uint32_t robot = robotOfSensor(world, sensor);
    if (name.empty() || robot == kNoRobot || robot >= world.robots.size()) {
        return name;
    }
    const std::string prefix = world.robots[robot] + "_";
    std::string prefixed = prefix + name;
    return !name.starts_with(prefix) && exists(prefixed) ? prefixed : name;
}
/// A joint, body, component or other prim by its signal name (World::prims::instanceNames).
inline std::string ownPrimName(const World& world, std::uint32_t sensor, const std::string& name) {
    return ownName(world, sensor, name, [&world](const std::string& n) {
        return std::ranges::find(world.prims.instanceNames, n) != world.prims.instanceNames.end();
    });
}
inline std::string ownJointName(const World& world, std::uint32_t sensor, const std::string& name) {
    return ownName(world, sensor, name, [&world](const std::string& n) {
        return std::ranges::find(world.joints.names, n) != world.joints.names.end();
    });
}
inline std::string ownBodyName(const World& world, std::uint32_t sensor, const std::string& name) {
    return ownName(world, sensor, name, [&world](const std::string& n) {
        return std::ranges::find(world.bodies.names, n) != world.bodies.names.end();
    });
}
inline std::string ownSensorName(const World& world, std::uint32_t sensor,
                                 const std::string& name) {
    return ownName(world, sensor, name, [&world](const std::string& n) {
        return std::ranges::find(world.sensors.names, n) != world.sensors.names.end();
    });
}
/// A signal name "family/<instance>/quantity" with its instance as the sensor's robot names it.
inline std::string ownSignalName(const World& world, std::uint32_t sensor,
                                 const std::string& signal) {
    const std::size_t first = signal.find('/');
    const std::size_t second = first == std::string::npos ? first : signal.find('/', first + 1);
    if (second == std::string::npos) {
        return signal;
    }
    const std::string instance = signal.substr(first + 1, second - first - 1);
    return signal.substr(0, first + 1) + ownPrimName(world, sensor, instance) +
           signal.substr(second);
}

/// A kind's (and a built-in sensor's) step. A rate whose period is a whole number of
/// nanoseconds (100 Hz, 400 Hz: 2.5 ms, 128 Hz: 7.8125 ms) keeps it exactly, as before. Any
/// other (30 Hz: 33'333'333.3 ns) shares no tick with the other modules and the scheduler
/// refused the run (base tick below 1 µs): it goes to whole milliseconds from 1 ms up, as the
/// cameras' (30 Hz runs every 33 ms), whole 10 µs below. `up`: never shorter than `ns` (a bus
/// that cannot send faster). `coarse`: whole milliseconds even for a whole number of ns (a
/// device's own period, 2.2 ms, beside a 1 Hz sensor would repeat only after 11 s).
inline Duration kindStep(double ns, bool up = false, bool coarse = false) {
    constexpr double kMs = 1e6;
    constexpr double kFine = 1e4;  // 10 µs
    if (!coarse && ns >= kFine && std::abs(ns - std::round(ns)) < 1e-3) {
        return Duration{static_cast<std::int64_t>(std::round(ns))};
    }
    const double grain = ns >= kMs ? kMs : kFine;
    const double units = up ? std::ceil(ns / grain - 1e-9) : std::round(ns / grain);
    return Duration{static_cast<std::int64_t>(std::max(units, 1.0) * grain)};
}

inline Duration kindPeriod(double rateHz) {
    return kindStep(1e9 / std::max(rateHz, 1e-3));
}

/// Common to all kinds: its mount, its rate, its name. Module domain "sensors", label
/// "<kind>_<sensor name>"; under the sensors domain's ground_truth substitute `groundTruth_` is
/// true and the kind must read exactly (no noise, no faults, no rendering).
class KindModule : public IModule {
public:
    KindModule(const World& world, std::uint32_t sensor, bool groundTruth)
        : groundTruth_(groundTruth)
        , sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , kind_(world.sensors.kind[sensor])
        , label_(world.sensors.kind[sensor] + "_" + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(kindPeriod(world.sensors.rateHz[sensor])) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", label_, 0, 1, groundTruth_, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

protected:
    [[nodiscard]] std::string base() const { return "sensor/" + name_; }

    bool groundTruth_;
    std::uint32_t sensor_;
    std::string name_;
    std::string kind_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
};

/// A kind's factory: the module for sensor `sensor`, or null when it has nothing to do.
using KindFactory = std::unique_ptr<IModule> (*)(const World& world, std::uint32_t sensor,
                                                 bool groundTruth, const KindContext& context);

}  // namespace lagrange::sensors::kinds
