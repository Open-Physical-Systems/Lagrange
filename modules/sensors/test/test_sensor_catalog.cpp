// The sensor catalog (RFC 0016 §5, modules/sensors/catalog/sensor_catalog.yaml) against the code:
// its kinds are exactly sensors::builtinKinds(), and for every kind a sensor with every catalog
// default written explicitly gives bit-identical signals over 1 s of CPU run to one with none
// (a wrong default in the catalog fails here). Each kind runs in a world that gives it something
// to read (catalog_cases_*.cpp), and a signal must change over the second, so that a kind which
// reads nothing cannot pass by giving the same nothing twice (plan R10).
#include "catalog_rig.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace lagrange::sensors::catalogtest {
namespace {

YAML::Node catalog() {
    static const YAML::Node node = YAML::LoadFile(LAGRANGE_SENSOR_CATALOG);
    return node;
}

const Cases& cases() {
    static const Cases all = [] {
        Cases c;
        addPowerAndCameraCases(c);
        addEnvironmentCases(c);
        addContactCases(c);
        addSceneAndRadioCases(c);
        return c;
    }();
    return all;
}

std::vector<std::string> catalogKinds() {
    std::vector<std::string> out;
    for (const auto& entry : catalog()["kinds"]) {
        out.push_back(entry.first.as<std::string>());
    }
    return out;
}

TEST(SensorCatalog, KindsAreExactlyBuiltinKinds) {
    std::vector<std::string> inFile = catalogKinds();
    std::vector<std::string> inCode;
    for (std::string_view k : builtinKinds()) {
        inCode.emplace_back(k);
    }
    std::ranges::sort(inFile);
    std::ranges::sort(inCode);
    EXPECT_EQ(inFile, inCode);
    for (const auto& item : catalog()["kinds"]) {
        const auto kind = item.first.as<std::string>();
        EXPECT_EQ(item.second["renders"].as<bool>(false), builtinKindRenders(kind)) << kind;
    }
}

// Kinds that are not rendering kinds but see nothing without the rendered camera scene.
const std::map<std::string, std::string, std::less<>> kNeedsTheCameraScene{
    {"detections",
     "its objects are the camera scene's parts and its occlusion is rendered (a Vulkan device, a "
     "GPU path, not bit-reproducible, invariant 7); without a scene it reports nothing either "
     "way, which would test nothing (plan R10)"},
};

TEST(SensorCatalog, EveryKindHasAWorldToRead) {
    for (const std::string& kind : catalogKinds()) {
        if (!builtinKindRenders(kind) && !kNeedsTheCameraScene.contains(kind)) {
            EXPECT_TRUE(cases().contains(kind)) << kind << " has no case in catalog_cases_*.cpp";
        }
    }
}

/// Whether the parameter is read in the world's mode: its `only_when` text parameters match.
bool readInMode(const YAML::Node& param, const YAML::Node& params, const catalogtest::Setup& s) {
    const YAML::Node when = param["only_when"];
    if (!when) {
        return true;
    }
    for (const auto& item : when) {
        const YAML::Node& value = item.second;
        // The mode's text parameter, or its default when the world leaves it out.
        const std::string mode = item.first.as<std::string>();
        std::string current;
        if (const auto it = s.text.find(mode); it != s.text.end()) {
            current = it->second;
        } else {
            for (const auto& other : params) {
                if (other["path"].as<std::string>() == mode && other["default"]) {
                    current = other["default"].as<std::string>();
                }
            }
        }
        // "qr|rfid": any of them.
        std::istringstream choices(value.as<std::string>());
        bool any = false;
        for (std::string choice; std::getline(choices, choice, '|');) {
            any = any || choice == current;
        }
        if (!any) {
            return false;
        }
    }
    return true;
}

struct Trace {
    std::vector<std::string> names;
    std::vector<std::size_t> counts;                  // values per signal
    std::vector<std::vector<std::uint64_t>> samples;  // per 1 ms, every value's bits
    std::string error;
};

/// Runs the kind for 1 s in its world with `numbers` and `text`, sampling every number signal
/// sensor/probe/... every millisecond.
Trace run(const std::string& kind, std::map<std::string, double, std::less<>> numbers,
          std::map<std::string, std::string, std::less<>> text, int steps = 1000) {
    Trace trace;
    Setup s = cases().at(kind).make();
    s.world.addCustomSensor(kProbe, kind, s.body, s.mount, s.rateHz, std::move(numbers),
                            std::move(text));
    const auto index = static_cast<std::uint32_t>(s.world.sensors.names.size() - 1);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    if (s.physics) {
        scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    }
    if (!s.feeds.empty()) {
        class Source final : public IModule {
        public:
            explicit Source(std::vector<Feed> feeds) : feeds_(std::move(feeds)) {}
            [[nodiscard]] ModuleInfo info() const override {
                return {"test", "catalog_source", 0, 1, false, true, CapabilityTier::Tier0};
            }
            [[nodiscard]] Duration preferredStep() const override { return Duration{1'000'000}; }
            Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
                ids_.clear();
                for (const Feed& f : feeds_) {
                    ids_.push_back(d.output(f.spec));
                }
                return {};
            }
            Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
                write(bus, 0.0);
                return {};
            }
            Status step(const StepContext& c, SignalBus& bus) override {
                write(bus, (c.t + c.dt).seconds());
                return {};
            }
            Status saveState(StateWriter&) const override { return {}; }
            Status restoreState(StateReader&) override { return {}; }

        private:
            void write(SignalBus& bus, double t) {
                for (std::size_t i = 0; i < feeds_.size(); ++i) {
                    bus.writeN(ids_[i], feeds_[i].value(t));
                }
            }
            std::vector<Feed> feeds_;
            std::vector<SignalId> ids_;
        };
        scheduler.add(std::make_unique<Source>(std::move(s.feeds)));
    }
    if (s.extraModules) {
        s.extraModules(scheduler, s.world);
    }
    auto module =
        createBuiltinKind(s.world, index, false, nullptr, nullptr, s.scene ? &*s.scene : nullptr);
    if (!module) {
        trace.error = "createBuiltinKind gave nothing";
        return trace;
    }
    scheduler.add(std::move(module), ModuleConfig(s.config));
    if (auto wired = scheduler.wire(WorldView(s.world)); !wired) {
        trace.error = "wiring: " + to_string(wired.error().front());
        return trace;
    }
    const SignalBus& bus = scheduler.bus();
    const std::string prefix = std::string("sensor/") + kProbe + "/";
    std::vector<SignalId> ids;
    for (std::size_t i = 0; i < bus.signalCount(); ++i) {
        const SignalSpec& spec = bus.specAt(i);
        if (spec.name.starts_with(prefix) && componentCount(spec.type, spec.arrayLength) > 0) {
            trace.names.push_back(spec.name);
            trace.counts.push_back(componentCount(spec.type, spec.arrayLength));
            ids.push_back(bus.find(spec.name));
        }
    }
    for (int k = 1; k <= steps; ++k) {  // 1 ms each
        if (Status ok = scheduler.stepUntil(SimTime{std::int64_t{k} * 1'000'000}); !ok.ok()) {
            trace.error = "step: " + ok.message;
            return trace;
        }
        std::vector<std::uint64_t> row;
        for (const SignalId id : ids) {
            for (const double v : bus.readN(id)) {
                row.push_back(std::bit_cast<std::uint64_t>(v));
            }
        }
        trace.samples.push_back(std::move(row));
    }
    return trace;
}

/// The kinds that start without a Vulkan device (detections, refused without its camera
/// before it looks for the scene, is one).
std::vector<std::string> allKindsForBareRobot() {
    std::vector<std::string> out;
    for (std::string_view k : builtinKinds()) {
        if (!builtinKindRenders(std::string(k))) {
            out.emplace_back(k);
        }
    }
    return out;
}

class ExplicitDefaultsChangeNothing : public testing::TestWithParam<std::string> {};

TEST_P(ExplicitDefaultsChangeNothing, OverOneSecond) {
    const std::string kind = GetParam();
    if (builtinKindRenders(kind)) {
        GTEST_SKIP() << kind
                     << " renders the camera scene: it needs a Vulkan device and is a GPU path, "
                        "not bit-reproducible (invariant 7); its defaults are checked by "
                        "check_sensor_catalog.py against sensors.md only";
    }
    if (const auto skip = kNeedsTheCameraScene.find(kind); skip != kNeedsTheCameraScene.end()) {
        GTEST_SKIP() << kind << ": " << skip->second;
    }
    ASSERT_TRUE(cases().contains(kind)) << kind << " has no case in catalog_cases_*.cpp";
    const catalogtest::Setup world = cases().at(kind).make();
    std::map<std::string, double, std::less<>> numbers = world.numbers;
    std::map<std::string, std::string, std::less<>> text = world.text;
    std::vector<std::string> written;
    const YAML::Node kindEntry = catalog()["kinds"][kind];
    for (const auto& param : kindEntry["params"]) {
        const std::string path = param["path"].as<std::string>();
        if (!param["default"] || numbers.contains(path) || text.contains(path) ||
            !readInMode(param, kindEntry["params"], world)) {
            continue;
        }
        const std::string type = param["type"].as<std::string>();
        if (type == "text" || type == "enum") {
            text[path] = param["default"].as<std::string>();
        } else {
            const std::string value = param["default"].as<std::string>();
            numbers[path] = value == "true"    ? 1.0
                            : value == "false" ? 0.0
                            : value == ".inf"  ? std::numeric_limits<double>::infinity()
                            : value == "-.inf" ? -std::numeric_limits<double>::infinity()
                                               : param["default"].as<double>();
        }
        written.push_back(path);
    }
    ASSERT_FALSE(written.empty()) << kind << ": no default to write";
    const Trace bare = run(kind, world.numbers, world.text);
    ASSERT_TRUE(bare.error.empty()) << kind << " without parameters: " << bare.error;
    const Trace full = run(kind, numbers, text);
    ASSERT_TRUE(full.error.empty()) << kind << " with its defaults: " << full.error;
    ASSERT_EQ(bare.names, full.names);
    ASSERT_EQ(bare.samples.size(), full.samples.size());

    // Something must change over the second (R10).
    bool changes = false;
    for (std::size_t k = 1; k < bare.samples.size() && !changes; ++k) {
        changes = bare.samples[k] != bare.samples[0];
    }
    EXPECT_TRUE(changes) << kind << ": no signal of sensor/probe changes over 1 s, so this run "
                         << "tests nothing; give its case a world it reads";

    std::ostringstream params;
    for (const std::string& p : written) {
        params << " " << p;
    }
    for (std::size_t k = 0; k < bare.samples.size(); ++k) {
        if (bare.samples[k] != full.samples[k]) {
            std::size_t c = 0;
            while (c < bare.samples[k].size() && bare.samples[k][c] == full.samples[k][c]) {
                ++c;
            }
            // Which signal holds value c.
            std::size_t first = 0;
            std::size_t which = 0;
            while (which + 1 < bare.counts.size() && first + bare.counts[which] <= c) {
                first += bare.counts[which];
                ++which;
            }
            const std::string signal = bare.names.empty() ? "?" : bare.names[which];
            FAIL() << kind << ": with its catalog defaults written (" << params.str()
                   << " ) the signals differ at " << (k + 1) << " ms, " << signal << "["
                   << (c - first) << "]: " << std::bit_cast<double>(bare.samples[k][c])
                   << " without, " << std::bit_cast<double>(full.samples[k][c]) << " with";
        }
    }
}

/// What the catalog says a kind needs beyond its defaults: its `required` parameters and its
/// `requires` items, as words; empty when it starts with nothing given.
std::vector<std::string> catalogNeeds(const std::string& kind) {
    std::vector<std::string> needs;
    const YAML::Node entry = catalog()["kinds"][kind];
    for (const auto& param : entry["params"]) {
        if (param["required"].as<bool>(false)) {
            needs.push_back("parameter " + param["path"].as<std::string>());
        }
    }
    for (const auto& item : entry["requires"]) {
        needs.push_back(item["why"].as<std::string>(""));
    }
    return needs;
}

// RFC 0016 QA F5: a sensor the builder adds with its catalog defaults on a bare robot (one free
// link, as a URDF's root) either starts, or the catalog says what it needs (`required`
// parameters, `requires`), from which the builder reports it before Test. Both ways: a kind the
// catalog marks as needing something must refuse here, so the builder never refuses what runs.
TEST(SensorCatalog, AKindWithItsDefaultsStartsOrTheCatalogSaysWhatItNeeds) {
    for (const std::string& kind : allKindsForBareRobot()) {
        World world;
        world.robots.push_back("robot");
        const BodyId base =
            world.addBody("base_link", kWorldBody, Pose{}, 1.0, {}, {0.01, 0.01, 0.01, 0, 0, 0});
        world.addJoint("base_link_floating", JointType::Free, base, {0, 0, 1}, false);
        world.addCustomSensor(kProbe, kind, base, Pose{}, 10.0, {}, {});
        const auto index = static_cast<std::uint32_t>(world.sensors.names.size() - 1);
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
        std::string refusal;
        auto module = createBuiltinKind(world, index, false, nullptr, nullptr, nullptr);
        if (!module) {
            refusal = "createBuiltinKind gave nothing";
        } else {
            scheduler.add(std::move(module));
            if (auto wired = scheduler.wire(WorldView(world)); !wired) {
                refusal = to_string(wired.error().front());
            }
        }
        const std::vector<std::string> needs = catalogNeeds(kind);
        std::string said;
        for (const std::string& n : needs) {
            said += (said.empty() ? "" : "; ") + n;
        }
        if (needs.empty()) {
            EXPECT_TRUE(refusal.empty())
                << kind << " with its defaults on a bare robot is refused (" << refusal
                << ") but the catalog marks nothing it needs: add `required: true` to the "
                   "parameter or a `requires` item, so the builder says so before Test";
        } else {
            EXPECT_FALSE(refusal.empty())
                << kind << ": the catalog says it needs " << said
                << " but it starts without; the builder would refuse a robot that runs";
        }
    }
}

/// Values just outside a parameter's catalog range: below `min`, at `min_exclusive`, above
/// `max`, at `max_exclusive`.
std::vector<double> outOfRange(const YAML::Node& param) {
    std::vector<double> out;
    const bool integer = param["type"].as<std::string>() == "int";
    const auto beyond = [integer](double edge, double sign) {
        return integer ? edge + sign : edge + (sign * std::max(1e-3, std::abs(edge) * 0.5));
    };
    if (param["min"]) {
        out.push_back(beyond(param["min"].as<double>(), -1.0));
    }
    if (param["min_exclusive"]) {
        out.push_back(param["min_exclusive"].as<double>());
    }
    if (param["max"]) {
        out.push_back(beyond(param["max"].as<double>(), 1.0));
    }
    if (param["max_exclusive"]) {
        out.push_back(param["max_exclusive"].as<double>());
    }
    return out;
}

/// A bare robot (one free link) carrying sensor `probe` of `kind` with `numbers`: why the run
/// refuses to start, or empty. For the kinds that need a Vulkan device, whose range refusal
/// comes before they look for one.
std::string bareRefusal(const std::string& kind,
                        std::map<std::string, double, std::less<>> numbers) {
    World world;
    world.robots.push_back("robot");
    const BodyId base =
        world.addBody("base_link", kWorldBody, Pose{}, 1.0, {}, {0.01, 0.01, 0.01, 0, 0, 0});
    world.addJoint("base_link_floating", JointType::Free, base, {0, 0, 1}, false);
    world.addCustomSensor(kProbe, kind, base, Pose{}, 10.0, std::move(numbers), {});
    const auto index = static_cast<std::uint32_t>(world.sensors.names.size() - 1);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto module = createBuiltinKind(world, index, false, nullptr, nullptr, nullptr);
    if (!module) {
        return "createBuiltinKind gave nothing";
    }
    scheduler.add(std::move(module));
    if (auto wired = scheduler.wire(WorldView(world)); !wired) {
        return to_string(wired.error().front());
    }
    return {};
}

class OutOfRangeIsRefused : public testing::TestWithParam<std::string> {};

/// Inclusive edges a kind takes only with another parameter moved too (kind/path, and why).
const std::set<std::string> kEdgeNeedsOthers = {};

// Sensors QA F2 (2026-11-15): every number parameter outside the catalog's range is refused by
// the kind, with a message naming the parameter, so the builder's form and the run agree and no
// value is silently clamped or ignored. Parameters of another mode run in that mode.
TEST_P(OutOfRangeIsRefused, EveryCatalogRange) {
    const std::string kind = GetParam();
    const bool bare = builtinKindRenders(kind) || kNeedsTheCameraScene.contains(kind);
    ASSERT_TRUE(bare || cases().contains(kind));
    const YAML::Node params = catalog()["kinds"][kind]["params"];
    for (const auto& param : params) {
        const std::string type = param["type"].as<std::string>();
        if (type != "number" && type != "int") {
            continue;
        }
        const std::string path = param["path"].as<std::string>();
        catalogtest::Setup world = bare ? catalogtest::Setup{} : cases().at(kind).make();
        // Its mode: the first choice of each `only_when` the world does not already give.
        if (!bare && !readInMode(param, params, world)) {
            for (const auto& item : param["only_when"]) {
                const std::string choice = item.second.as<std::string>();
                world.text[item.first.as<std::string>()] = choice.substr(0, choice.find('|'));
            }
        }
        for (const double bad : outOfRange(param)) {
            auto numbers = world.numbers;
            numbers[path] = bad;
            const std::string error =
                bare ? bareRefusal(kind, numbers) : run(kind, numbers, world.text, 2).error;
            EXPECT_FALSE(error.empty()) << kind << ": " << path << " = " << bad
                                        << " is outside the catalog's range but the kind starts";
            EXPECT_NE(error.find(path + " = "), std::string::npos)
                << kind << ": " << path << " = " << bad << " refused without naming it: " << error;
        }
    }
}

class OnTheEdgeIsTaken : public testing::TestWithParam<std::string> {};

// Review L4 (2026-11-15): the catalog's table and the kind's own checks agree at each bound: a
// value on an inclusive edge starts the kind (its own checks refuse nothing the table allows),
// and every value at an exclusive edge or past an edge is refused (EveryCatalogRange). For the
// kinds that need a Vulkan device, only a range refusal counts (the device comes after).
TEST_P(OnTheEdgeIsTaken, EveryInclusiveBound) {
    const std::string kind = GetParam();
    const bool bare = builtinKindRenders(kind) || kNeedsTheCameraScene.contains(kind);
    const YAML::Node params = catalog()["kinds"][kind]["params"];
    for (const auto& param : params) {
        const std::string type = param["type"].as<std::string>();
        if (type != "number" && type != "int") {
            continue;
        }
        const std::string path = param["path"].as<std::string>();
        catalogtest::Setup world = bare ? catalogtest::Setup{} : cases().at(kind).make();
        if (!bare && !readInMode(param, params, world)) {
            for (const auto& item : param["only_when"]) {
                const std::string choice = item.second.as<std::string>();
                world.text[item.first.as<std::string>()] = choice.substr(0, choice.find('|'));
            }
        }
        std::vector<double> edges;
        if (param["min"]) {
            edges.push_back(param["min"].as<double>());
        }
        if (param["max"]) {
            edges.push_back(param["max"].as<double>());
        }
        for (const double edge : edges) {
            if (kEdgeNeedsOthers.contains(kind + "/" + path)) {
                continue;
            }
            auto numbers = world.numbers;
            numbers[path] = edge;
            const std::string error =
                bare ? bareRefusal(kind, numbers) : run(kind, numbers, world.text, 2).error;
            const bool rangeRefusal = error.find("out of range") != std::string::npos ||
                                      error.find(path + " = ") != std::string::npos;
            EXPECT_TRUE(bare ? !rangeRefusal : error.empty())
                << kind << ": " << path << " = " << edge
                << " is on the catalog's inclusive edge but the kind refuses it: " << error;
        }
    }
}

// The code's range table (ParamRanges.cpp) is the catalog's, row for row, and a value on an
// inclusive edge is taken.
TEST(SensorCatalog, TheRangeTableIsTheCatalogs) {
    std::set<std::string> inCatalog;
    std::set<std::string> inCode;
    const auto row = [](std::string_view kind, std::string_view path, double lo, double hi,
                        bool loEx, bool hiEx) {
        std::ostringstream out;
        out << std::setprecision(17) << kind << " " << path << " " << lo << (loEx ? ")" : "]")
            << " " << hi << (hiEx ? "(" : "[");
        return out.str();
    };
    constexpr double kInf = std::numeric_limits<double>::infinity();
    for (const auto& item : catalog()["kinds"]) {
        const auto kind = item.first.as<std::string>();
        for (const auto& param : item.second["params"]) {
            const auto type = param["type"].as<std::string>();
            if (type != "number" && type != "int") {
                continue;
            }
            const bool loEx = static_cast<bool>(param["min_exclusive"]);
            const bool hiEx = static_cast<bool>(param["max_exclusive"]);
            const double lo = param["min"]             ? param["min"].as<double>()
                              : param["min_exclusive"] ? param["min_exclusive"].as<double>()
                                                       : -kInf;
            const double hi = param["max"]             ? param["max"].as<double>()
                              : param["max_exclusive"] ? param["max_exclusive"].as<double>()
                                                       : kInf;
            if (std::isfinite(lo) || std::isfinite(hi)) {
                inCatalog.insert(row(kind, param["path"].as<std::string>(), lo, hi, loEx, hiEx));
            }
        }
    }
    for (const KindParamRange& r : builtinKindParamRanges()) {
        inCode.insert(row(r.kind, r.path, r.min, r.max, r.minExclusive, r.maxExclusive));
        EXPECT_TRUE(isBuiltinKind(r.kind)) << r.kind;
        if (!r.minExclusive && std::isfinite(r.min)) {
            EXPECT_TRUE(r.contains(r.min)) << r.kind << " " << r.path;
        }
        if (!r.maxExclusive && std::isfinite(r.max)) {
            EXPECT_TRUE(r.contains(r.max)) << r.kind << " " << r.path;
        }
    }
    EXPECT_EQ(inCatalog, inCode);
}

std::vector<std::string> allKinds() {
    std::vector<std::string> out;
    for (std::string_view k : builtinKinds()) {
        out.emplace_back(k);
    }
    return out;
}

INSTANTIATE_TEST_SUITE_P(SensorCatalog, ExplicitDefaultsChangeNothing,
                         testing::ValuesIn(allKinds()),
                         [](const testing::TestParamInfo<std::string>& p) { return p.param; });
INSTANTIATE_TEST_SUITE_P(SensorCatalog, OutOfRangeIsRefused, testing::ValuesIn(allKinds()),
                         [](const testing::TestParamInfo<std::string>& p) { return p.param; });
INSTANTIATE_TEST_SUITE_P(SensorCatalog, OnTheEdgeIsTaken, testing::ValuesIn(allKinds()),
                         [](const testing::TestParamInfo<std::string>& p) { return p.param; });

}  // namespace
}  // namespace lagrange::sensors::catalogtest
