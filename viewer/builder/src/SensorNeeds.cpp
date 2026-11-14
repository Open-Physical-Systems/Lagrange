// What a sensor needs before the server will start it (RFC 0016 QA F5), and sensor rates the
// scheduler cannot line up (QA F10), from the sensor catalog's data: its `required` parameters,
// its `requires` items and the sonar's `step_at_least`. validate() calls both.
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

namespace {

std::string inQuotes(std::string_view name) {
    return "'" + std::string(name) + "'";
}

/// The sensor's text for `path`, trimmed; "" when not given.
std::string given(const Sensor& sensor, std::string_view path) {
    const auto it = sensor.params.find(path);
    if (it == sensor.params.end()) {
        return {};
    }
    std::string text = it->second;
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return {};
    }
    text.erase(0, first);
    text.erase(text.find_last_not_of(" \t") + 1);
    return text;
}

std::vector<std::string> wordsOf(const std::string& text) {
    std::istringstream in(text);
    std::vector<std::string> out;
    for (std::string word; in >> word;) {
        out.push_back(word);
    }
    return out;
}

const CatalogParam* paramOf(const CatalogEntry& entry, std::string_view path) {
    const auto it =
        std::ranges::find_if(entry.params, [&](const CatalogParam& p) { return p.path == path; });
    return it == entry.params.end() ? nullptr : &*it;
}

bool cameraType(std::string_view type) {
    return type == "camera" || type == "depth" || type == "depth_camera" || type == "rgbd_camera" ||
           type == "rgbd";
}

/// A URDF `<lagrange><schematic>` block (formats.md §9.2) brings batteries and motors the
/// builder does not model: their needs are then the server's to judge.
bool hasSchematicBlock(const Document& document) {
    return document.keptXml.find("<lagrange") != std::string::npos;
}

/// The joint above `link` that moves, through fixed joints (a massless sensor frame merges into
/// its parent); nullptr when there is none up to the root.
const Joint* movingJointAbove(const Document& document, std::string link) {
    std::set<std::string> seen;
    while (seen.insert(link).second) {
        const auto it =
            std::ranges::find_if(document.joints, [&](const Joint& j) { return j.child == link; });
        if (it == document.joints.end()) {
            return nullptr;
        }
        if (isMoving(it->type)) {
            return &*it;
        }
        link = it->parent;
    }
    return nullptr;
}

/// Whether the document names `name`: a link, joint or sensor.
bool documentNames(const Document& document, std::string_view name) {
    return findLink(document, name) != nullptr || findJoint(document, name) != nullptr ||
           findSensor(document, name) != nullptr;
}

}  // namespace

std::vector<Problem> checkSensorNeeds(const Document& document, const Sensor& sensor,
                                      const CatalogEntry& entry) {
    std::vector<Problem> out;
    const std::string name = "sensor " + inQuotes(sensor.name) + " (" + entry.name + ")";
    const auto add = [&](bool world, std::string message) {
        out.push_back({world ? Level::Warning : Level::Error, sensor.name, std::move(message)});
    };
    // Required parameters, and what a given one names.
    for (const CatalogParam& param : entry.params) {
        const std::string text = given(sensor, param.path);
        if (text.empty()) {
            if (param.required) {
                add(false,
                    name + " needs " + param.path +
                        (param.meaning.empty()
                             ? std::string()
                             : " (" + param.meaning.substr(0, param.meaning.find(';')) + ")") +
                        ": it has no default and the server refuses the sensor without it");
            }
            continue;
        }
        if (param.refersTo == "joint") {
            for (const std::string& joint : wordsOf(text)) {
                if (findJoint(document, joint) == nullptr) {
                    add(false, name + ": " + param.path + " names joint " + inQuotes(joint) +
                                   ", which the robot does not have");
                }
            }
        } else if (param.refersTo == "camera") {
            const Sensor* camera = findSensor(document, text);
            if (camera == nullptr || !cameraType(camera->type)) {
                add(false, name + ": " + param.path + " names " + inQuotes(text) +
                               ", which is not a camera sensor of the robot");
            }
        }
    }
    // What it needs beyond its parameters.
    for (const CatalogRequirement& need : entry.needs) {
        if (!need.unless.empty() && !given(sensor, need.unless).empty()) {
            continue;
        }
        const std::string what = name + " needs " + need.why;
        const std::string world = need.world ? "; Test on the empty ground stops on it, so test "
                                               "it Into a connected run whose world has it"
                                             : "";
        if (!need.oneOf.empty()) {
            if (std::ranges::all_of(
                    need.oneOf, [&](const std::string& p) { return given(sensor, p).empty(); })) {
                add(need.world, what + world);
            }
        } else if (!need.schema.empty()) {
            if (entriesWith(document, need.schema).empty() && !hasSchematicBlock(document)) {
                add(need.world, what + (need.world ? world
                                                   : "; the robot has none (add it in the power "
                                                     "system or the overlay)"));
            }
        } else if (!need.prim.empty()) {
            std::string target = given(sensor, need.prim);
            if (target.empty()) {
                const CatalogParam* param = paramOf(entry, need.prim);
                target = param != nullptr ? param->defaultText : std::string();
            }
            const std::string leaf = target.substr(target.find_last_of('/') + 1);
            if (!documentNames(document, leaf)) {
                add(need.world, what + " (" + inQuotes(leaf) + ")" + world);
            }
        } else if (!need.movingJoint.empty()) {
            const std::string joint = given(sensor, need.movingJoint);
            if (joint.empty()) {
                if (findLink(document, sensor.link) != nullptr &&
                    movingJointAbove(document, sensor.link) == nullptr) {
                    add(false, what + "; its link " + inQuotes(sensor.link) +
                                   " hangs from no hinge or slide (the root, or fixed joints "
                                   "only): mount it on a moving joint's child link or name one "
                                   "in " +
                                   need.movingJoint);
                }
            } else if (const Joint* j = findJoint(document, joint);
                       j != nullptr && !isMoving(j->type)) {
                add(false, what + "; joint " + inQuotes(joint) + " is fixed");
            }
        } else if (!need.wheelPair.empty()) {
            // The server's detection: hinges off the sensor's body whose child carries a
            // cylinder or sphere; two driven ones, else two of any. Which are driven is the
            // server's to know, so only fewer than two is certain.
            const auto wheels = std::ranges::count_if(document.joints, [&](const Joint& j) {
                const Link* child = findLink(document, j.child);
                return j.parent == sensor.link &&
                       (j.type == Joint::Type::Revolute || j.type == Joint::Type::Continuous) &&
                       child != nullptr && std::ranges::any_of(child->shapes, [](const Shape& s) {
                           return s.collision && (s.kind == Shape::Kind::Cylinder ||
                                                  s.kind == Shape::Kind::Sphere);
                       });
            });
            if (given(sensor, need.wheelPair).empty() && wheels < 2) {
                add(false, what + "; link " + inQuotes(sensor.link) + " has " +
                               std::to_string(wheels) + " such hinge" + (wheels == 1 ? "" : "s"));
            }
        } else if (!need.motorOn.empty()) {
            const std::string joint = given(sensor, need.motorOn);
            if (!joint.empty() && motorOf(document, joint) < 0 && !hasSchematicBlock(document)) {
                add(false, what + "; no motor of the power system drives joint " + inQuotes(joint));
            }
        }
    }
    return out;
}

namespace {

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kFine = 10'000;                    // 10 µs
constexpr std::int64_t kMaxHyperperiod = 10'000'000'000;  // 10 s, the scheduler's limit
constexpr std::int64_t kPhysicsStep = kMs;                // the rigid body's 1 ms
constexpr std::int64_t kForever = std::numeric_limits<std::int64_t>::max();

/// The server's period for a rate (modules/sensors kindPeriod): exact when it is a whole number
/// of ns (at least 10 µs), else on whole milliseconds (10 µs below 1 ms); a camera's always on
/// whole milliseconds.
std::int64_t periodOf(double rateHz, bool camera) {
    const double ns = 1e9 / std::max(rateHz, 1e-3);
    if (camera) {
        return std::max<std::int64_t>(std::llround(ns / 1e6), 1) * kMs;
    }
    if (ns >= static_cast<double>(kFine) && std::abs(ns - std::round(ns)) < 1e-3) {
        return std::llround(ns);
    }
    const double grain = ns >= static_cast<double>(kMs) ? kMs : kFine;
    return static_cast<std::int64_t>(std::max(std::round(ns / grain), 1.0) * grain);
}

/// A parameter as a number: the sensor's text, else the catalog's default.
std::optional<double> numberOf(const Sensor& sensor, const CatalogEntry& entry,
                               std::string_view path) {
    std::string text = given(sensor, path);
    if (text.empty()) {
        if (const CatalogParam* param = paramOf(entry, path)) {
            text = param->defaultText;
        }
    }
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (text.empty() || end != text.c_str() + text.size() || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::string duration(std::int64_t ns) {
    char text[48];
    if (ns % kMs == 0) {
        std::snprintf(text, sizeof(text), "%lld ms", static_cast<long long>(ns / kMs));
    } else {
        std::snprintf(text, sizeof(text), "%.6g ms", static_cast<double>(ns) / kMs);
    }
    return text;
}

std::int64_t together(std::int64_t a, std::int64_t b) {
    const std::int64_t factor = b / std::gcd(a, b);
    return a > kForever / factor ? kForever : a * factor;
}

}  // namespace

std::vector<Problem> checkSensorRates(const Document& document, const SensorCatalog& catalog) {
    struct Step {
        const Sensor* sensor;
        std::int64_t ns;
        std::string why;  // what set it, when not the rate
    };
    std::vector<Step> steps;
    for (const Sensor& sensor : document.sensors) {
        const CatalogEntry* entry = catalog.find(sensor.type);
        if (!(sensor.updateRate > 0.0) || !std::isfinite(sensor.updateRate) || entry == nullptr ||
            entry->overlayOnly) {
            continue;
        }
        Step step{&sensor, periodOf(sensor.updateRate, cameraType(sensor.type)), {}};
        if (!entry->stepRoundTrip.empty()) {
            const auto range = numberOf(sensor, *entry, entry->stepRoundTrip);
            const auto speed = numberOf(sensor, *entry, entry->stepSpeed);
            if (range && speed && *range > 0.0 && *speed > 0.0) {
                const auto ping = static_cast<std::int64_t>(
                    std::ceil(2.0 * *range / *speed * 1e3 - 1e-9) * static_cast<double>(kMs));
                if (ping > step.ns) {
                    step.ns = ping;
                    step.why = ", its ping: 2 × " + entry->stepRoundTrip + " ÷ " + entry->stepSpeed;
                }
            }
        }
        steps.push_back(std::move(step));
    }
    std::vector<Problem> out;
    // Every pair (and each against the 1 ms physics step, which divides any whole-ms step).
    std::set<const Sensor*> named;
    for (std::size_t i = 0; i < steps.size(); ++i) {
        for (std::size_t j = i + 1; j < steps.size(); ++j) {
            const Step& a = steps[i];
            const Step& b = steps[j];
            const std::int64_t both = together(together(a.ns, b.ns), kPhysicsStep);
            if (both <= kMaxHyperperiod || named.contains(a.sensor) || named.contains(b.sensor)) {
                continue;
            }
            named.insert(a.sensor);
            named.insert(b.sensor);
            char every[48];
            std::snprintf(every, sizeof(every), "%.6g s", static_cast<double>(both) * 1e-9);
            out.push_back(
                {Level::Warning, b.sensor->name,
                 "sensors " + inQuotes(a.sensor->name) + " (every " + duration(a.ns) + a.why +
                     ") and " + inQuotes(b.sensor->name) + " (every " + duration(b.ns) + b.why +
                     ") line up only every " +
                     (both == kForever ? std::string("few centuries") : std::string(every)) +
                     "; the server refuses steps that repeat after more than 10 s. Change one "
                     "rate so that one period divides the other (or both are multiples of a "
                     "common one, such as 20 ms)"});
        }
    }
    return out;
}

}  // namespace lagrange::viewer::builder
