// The sensor catalog loader (RFC 0016 §5, plan WP-C): errors with file and line, plugin files,
// duplicate kinds, and the real catalog files of the repository.
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/Edits.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::viewer::builder {
namespace {

namespace fs = std::filesystem;

const fs::path kBuiltin = LAGRANGE_SENSOR_CATALOG_YAML;
const fs::path kPlugins = LAGRANGE_PLUGIN_CATALOG_DIR;

std::vector<fs::path> realFiles() {
    return {kBuiltin, kPlugins / "tof.yaml", kPlugins / "ultrasonic.yaml",
            kPlugins / "thermistor.yaml", kPlugins / "ir_thermometer.yaml"};
}

class CatalogLoader : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = fs::temp_directory_path() /
               ("lagrange_catalog_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(dir_); }

    fs::path write(const std::string& name, const std::string& text) {
        const fs::path path = dir_ / name;
        std::ofstream(path) << text;
        return path;
    }

    fs::path dir_;
};

constexpr const char* kSmall = R"(schema_version: 1
types:
  lidar:
    category: range_and_contact
    urdf_types: [lidar, gpu_lidar, ray]
    description: A scanning lidar.
    params:
      - path: range/max
        type: number
        unit: m
        default: 30.0
        min: 0
kinds:
  rangefinder:
    category: range_and_contact
    description: One beam.
    params:
      - path: range/max
        type: number
        unit: m
        default: 4
        min_exclusive: 0
    signals:
      - name: range
        type: scalar
        unit: m
    ros:
      - topic: range
        type: sensor_msgs/msg/Range
)";

constexpr const char* kPlugin = R"(schema_version: 1
plugin: lagrange_tof
kinds:
  tof:
    category: range_and_contact
    description: One ray.
    params:
      - path: fov
        type: number
        unit: rad
        default: 0
        min: 0
)";

const SensorCatalog& loaded(const std::variant<SensorCatalog, SensorCatalog::LoadError>& result) {
    if (const auto* error = std::get_if<SensorCatalog::LoadError>(&result)) {
        ADD_FAILURE() << error->file << ":" << error->line << ": " << error->message;
        static const SensorCatalog kEmpty;
        return kEmpty;
    }
    return std::get<SensorCatalog>(result);
}

TEST_F(CatalogLoader, MalformedFileGivesFileAndLine) {
    // A YAML syntax error on line 4 (an unclosed flow sequence).
    const fs::path syntax =
        write("syntax.yaml", "schema_version: 1\nkinds:\n  a:\n    urdf_types: [a, b\n");
    auto result = SensorCatalog::load({syntax});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    auto error = std::get<SensorCatalog::LoadError>(result);
    EXPECT_EQ(error.file, syntax.string());
    EXPECT_GE(error.line, 4);
    EXPECT_FALSE(error.message.empty());

    // Well-formed YAML with a parameter of an unknown type on line 7.
    const fs::path type = write("type.yaml", R"(schema_version: 1
kinds:
  sonar:
    category: range_and_contact
    params:
      - path: range/max
        type: distance
)");
    result = SensorCatalog::load({type});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    error = std::get<SensorCatalog::LoadError>(result);
    EXPECT_EQ(error.file, type.string());
    EXPECT_EQ(error.line, 7);
    EXPECT_NE(error.message.find("distance"), std::string::npos) << error.message;

    // A default outside its own bounds, on line 8.
    const fs::path bounds = write("bounds.yaml", R"(schema_version: 1
kinds:
  sonar:
    params:
      - path: range/max
        type: number
        min: 0
        default: -1
)");
    result = SensorCatalog::load({bounds});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    EXPECT_EQ(std::get<SensorCatalog::LoadError>(result).line, 8);

    // A missing file names the file, at no line.
    result = SensorCatalog::load({dir_ / "missing.yaml"});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    EXPECT_EQ(std::get<SensorCatalog::LoadError>(result).line, 0);
    EXPECT_EQ(std::get<SensorCatalog::LoadError>(result).file, (dir_ / "missing.yaml").string());
}

TEST_F(CatalogLoader, PluginFileAddsKinds) {
    const fs::path base = write("base.yaml", kSmall);
    const fs::path plugin = write("tof.yaml", kPlugin);
    const SensorCatalog& alone = loaded(SensorCatalog::load({base}));
    EXPECT_EQ(alone.entries().size(), 2U);
    EXPECT_EQ(alone.find("tof"), nullptr);

    const auto result = SensorCatalog::load({base, plugin});
    const SensorCatalog& both = loaded(result);
    ASSERT_EQ(both.entries().size(), 3U);
    const CatalogEntry* tof = both.find("tof");
    ASSERT_NE(tof, nullptr);
    EXPECT_FALSE(tof->builtinType);
    EXPECT_EQ(tof->group, "plugins");
    EXPECT_EQ(tof->source, plugin.string());
    EXPECT_EQ(&both.entries().back(), tof);  // plugins' kinds come last
    // Types by any URDF word, kinds by name; the built-in types first.
    EXPECT_EQ(both.find("gpu_lidar"), both.find("lidar"));
    EXPECT_EQ(both.entries().front().name, "lidar");
    EXPECT_EQ(both.entries().front().group, "built_in_types");
    // Numbers are held as the form writes them; an exclusive bound refuses itself.
    const CatalogEntry* rangefinder = both.find("rangefinder");
    ASSERT_NE(rangefinder, nullptr);
    ASSERT_EQ(rangefinder->params.size(), 1U);
    EXPECT_EQ(rangefinder->params[0].defaultText, "4");
    EXPECT_TRUE(rangefinder->params[0].minExclusive);
    EXPECT_FALSE(checkParam(rangefinder->params[0], "0").empty());
    EXPECT_TRUE(checkParam(rangefinder->params[0], "0.01").empty());
    ASSERT_EQ(rangefinder->rosTopics.size(), 1U);
    EXPECT_EQ(rangefinder->rosTopics[0].name, "range");

    // A plugin's file adds kinds only.
    const fs::path types = write("types.yaml", "schema_version: 1\nplugin: x\ntypes:\n  cam: {}\n");
    EXPECT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(SensorCatalog::load({types})));
}

TEST_F(CatalogLoader, DuplicateKindIsAnError) {
    const fs::path base = write("base.yaml", kSmall);
    const fs::path again = write("again.yaml", R"(schema_version: 1
plugin: lagrange_other
kinds:
  # a comment line
  rangefinder:
    params: []
)");
    auto result = SensorCatalog::load({base, again});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    auto error = std::get<SensorCatalog::LoadError>(result);
    EXPECT_EQ(error.file, again.string());
    EXPECT_EQ(error.line, 5);
    EXPECT_NE(error.message.find("rangefinder"), std::string::npos) << error.message;
    EXPECT_NE(error.message.find(base.string()), std::string::npos) << error.message;

    // A kind named like a built-in type's URDF word clashes too.
    const fs::path ray = write("ray.yaml", "schema_version: 1\nplugin: p\nkinds:\n  ray: {}\n");
    result = SensorCatalog::load({base, ray});
    ASSERT_TRUE(std::holds_alternative<SensorCatalog::LoadError>(result));
    EXPECT_EQ(std::get<SensorCatalog::LoadError>(result).line, 4);
}

TEST_F(CatalogLoader, CatalogFilesFollowTheAmentPrefixes) {
    // An overlay prefix with the viewer's copy and a plugin index entry, over an underlay with
    // the same package's entry (the overlay's wins, as in ament_index).
    const fs::path overlay = dir_ / "overlay";
    const fs::path underlay = dir_ / "underlay";
    fs::create_directories(overlay / "share/lagrange_viewer/catalog");
    fs::create_directories(overlay / "share/ament_index/resource_index/lagrange_sensor_catalog");
    fs::create_directories(underlay / "share/ament_index/resource_index/lagrange_sensor_catalog");
    std::ofstream(overlay / "share/lagrange_viewer/catalog/sensor_catalog.yaml") << kSmall;
    std::ofstream(overlay / "share/ament_index/resource_index/lagrange_sensor_catalog/lagrange_tof")
        << "share/p/tof.yaml";
    std::ofstream(underlay /
                  "share/ament_index/resource_index/lagrange_sensor_catalog/lagrange_tof")
        << "share/p/old_tof.yaml";
    const char* before = std::getenv("AMENT_PREFIX_PATH");
    const std::string saved = before != nullptr ? before : "";
    ::setenv("AMENT_PREFIX_PATH", (overlay.string() + ":" + underlay.string()).c_str(), 1);
    const std::vector<fs::path> files = catalogFiles();
    ::setenv("AMENT_PREFIX_PATH", saved.c_str(), 1);
    ASSERT_EQ(files.size(), 2U);
    EXPECT_EQ(files[0], overlay / "share/lagrange_viewer/catalog/sensor_catalog.yaml");
    EXPECT_EQ(files[1], overlay / "share/p/tof.yaml");
}

TEST(CatalogLoaderReal, EveryEntryOfTheRealCatalogLoads) {
    const auto result = SensorCatalog::load(realFiles());
    const SensorCatalog& catalog = loaded(result);
    // 3 URDF types, 4 overlay-only types, the 37 built-in kinds and the 4 sample plugins' kinds.
    EXPECT_EQ(catalog.entries().size(), 3U + 4U + 37U + 4U);
    std::size_t overlay = 0;
    for (const CatalogEntry& entry : catalog.entries()) {
        SCOPED_TRACE(entry.name);
        overlay += entry.overlayOnly ? 1 : 0;
        EXPECT_EQ(catalog.find(entry.name), &entry);
        EXPECT_FALSE(entry.group.empty());
        EXPECT_FALSE(entry.summary.empty());
        for (const CatalogParam& param : entry.params) {
            SCOPED_TRACE(param.path);
            EXPECT_TRUE(checkParam(param, param.defaultText).empty());
        }
    }
    EXPECT_EQ(overlay, 4U);
    // The groups are the SPEC's, built-in types first and the plugins last.
    const std::vector<std::string> spec{
        "built_in_types",   "range_and_contact", "vision", "air_and_weather", "power",
        "force_and_motion", "radio_and_vehicle", "plugins"};
    std::vector<std::string> groups;
    for (const CatalogEntry& entry : catalog.entries()) {
        if (groups.empty() || groups.back() != entry.group) {
            groups.push_back(entry.group);
        }
    }
    EXPECT_EQ(groups, spec);
    for (const char* name :
         {"camera", "lidar", "imu", "depth", "gpu_lidar", "rangefinder", "tof", "thermistor"}) {
        EXPECT_NE(catalog.find(name), nullptr) << name;
    }
    const CatalogEntry* imu = catalog.find("imu");
    ASSERT_NE(imu, nullptr);
    EXPECT_TRUE(imu->builtinType);
    EXPECT_EQ(imu->group, "built_in_types");
}

TEST(CatalogLoaderReal, EveryEntryAtItsDefaultsValidates) {
    const auto result = SensorCatalog::load(realFiles());
    const SensorCatalog& catalog = loaded(result);
    for (const CatalogEntry& entry : catalog.entries()) {
        SCOPED_TRACE(entry.name);
        Sensor sensor;
        sensor.name = "s";
        sensor.link = "base";
        sensor.type = entry.name;
        for (const CatalogParam& param : entry.params) {
            if (!param.defaultText.empty()) {
                sensor.params[param.path] = param.defaultText;  // every default, written
            }
        }
        for (const Problem& problem : checkSensorParams(sensor, entry)) {
            ADD_FAILURE() << problem.message;
        }
    }
    // An out-of-range value is an error that names the parameter and the bound.
    const CatalogEntry* rangefinder = catalog.find("rangefinder");
    ASSERT_NE(rangefinder, nullptr);
    Sensor sensor;
    sensor.name = "front";
    sensor.type = "rangefinder";
    sensor.params["range/min"] = "-1";
    const std::vector<Problem> problems = checkSensorParams(sensor, *rangefinder);
    ASSERT_FALSE(problems.empty());
    EXPECT_EQ(problems[0].level, Level::Error);
    EXPECT_NE(problems[0].message.find("range/min"), std::string::npos) << problems[0].message;
}

}  // namespace
}  // namespace lagrange::viewer::builder
