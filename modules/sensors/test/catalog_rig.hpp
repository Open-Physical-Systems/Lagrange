#pragma once
// test_sensor_catalog's rig: for each built-in kind, a world in which the kind has something to
// read (plan R10), run twice over 1 s, once with no parameters and once with every catalog
// default written explicitly. Each kind's world comes from a Case in catalog_cases_*.cpp.

#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::sensors::catalogtest {

/// A signal another domain would write, as a function of the time at the end of each 1 ms step.
struct Feed {
    SignalSpec spec;
    std::function<std::vector<double>(double t)> value;
};

inline SignalSpec specOf(std::string name, SignalType type, Unit unit, std::string frame = {},
                         std::uint32_t arrayLength = 0) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    if (arrayLength != 0) {
        s.arrayLength = arrayLength;
    }
    return s;
}

/// One kind's run: the world without the sensor, where the sensor sits, and what feeds it.
struct Setup {
    World world;
    BodyId body{kWorldBody};  ///< the sensor's body; addCustomSensor is called by the test
    Pose mount;               ///< the sensor's pose in its body
    double rateHz{50.0};
    std::vector<Feed> feeds;     ///< written every 1 ms by a source module
    bool physics{false};         ///< MuJoCo runs the world (contacts, joints, wheels)
    ModuleConfigData config;     ///< the kind's module config (site air density, ...)
    std::optional<Scene> scene;  ///< the model's scene, for kinds that read it (KindContext::scene)
    /// Parameters this world needs that have no default (a joint's name, a signal's name), or a
    /// mode: present in both runs. Catalog defaults are written for every other parameter.
    std::map<std::string, double, std::less<>> numbers;
    std::map<std::string, std::string, std::less<>> text;
    /// Other modules the kind needs beside the source (rarely: another sensor's module).
    std::function<void(Scheduler&, const World&)> extraModules;
};

/// The name the test gives the sensor under test: its signals are sensor/probe/...
inline constexpr const char* kProbe = "probe";

struct Case {
    std::function<Setup()> make;
};

using Cases = std::map<std::string, Case, std::less<>>;

// One per file, each filling the cases of its kinds.
void addPowerAndCameraCases(Cases& cases);
void addEnvironmentCases(Cases& cases);
void addContactCases(Cases& cases);
void addSceneAndRadioCases(Cases& cases);

}  // namespace lagrange::sensors::catalogtest
