// The sensor catalog loader (RFC 0016 §5, plan WP-C): the built-in catalog
// (modules/sensors/catalog/sensor_catalog.yaml, installed as a copy in
// share/lagrange_viewer/catalog/) and plugin catalogs named by the ament resource
// `lagrange_sensor_catalog`. The file format is described at the top of sensor_catalog.yaml.
#include "lagrange/viewer/builder/Edits.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace lagrange::viewer::builder {

struct SensorCatalog::Data {
    std::vector<CatalogEntry> entries;
    std::map<std::string, std::size_t, std::less<>> byName;  // names and urdf types
};

SensorCatalog::SensorCatalog() : data_(std::make_shared<const Data>()) {}

namespace {

/// The catalog panel's groups in the SPEC's order (12-builder SPEC "Sensor catalog"); the
/// files' `category` words map onto them. Unknown categories follow in the order they appear.
constexpr std::string_view kBuiltinGroup = "built_in_types";
constexpr std::string_view kPluginGroup = "plugins";
constexpr std::array<std::string_view, 6> kGroups{"range_and_contact", "vision",
                                                  "air_and_weather",   "power",
                                                  "force_and_motion",  "radio_and_vehicle"};

std::string groupOf(const std::string& category) {
    static const std::map<std::string, std::string, std::less<>> kMap{
        {"range_and_contact", "range_and_contact"},
        {"camera", "vision"},
        {"perception", "vision"},
        {"vision", "vision"},
        {"environment", "air_and_weather"},
        {"air_and_weather", "air_and_weather"},
        {"power", "power"},
        {"motion_and_force", "force_and_motion"},
        {"force_and_motion", "force_and_motion"},
        {"radio", "radio_and_vehicle"},
        {"vehicle", "radio_and_vehicle"},
        {"radio_and_vehicle", "radio_and_vehicle"},
    };
    const auto it = kMap.find(category);
    return it != kMap.end() ? it->second : category;
}

/// A load error at a node: yaml-cpp's marks are 0-based, the error's line is 1-based.
struct Failure {
    SensorCatalog::LoadError error;
};

[[noreturn]] void fail(const std::string& file, const YAML::Node& node, const std::string& what) {
    const YAML::Mark mark = node.Mark();
    throw Failure{{file, mark.line >= 0 ? mark.line + 1 : 0, what}};
}

class Reader {
public:
    explicit Reader(std::string file) : file_(std::move(file)) {}

    [[nodiscard]] const std::string& file() const { return file_; }

    [[noreturn]] void fail(const YAML::Node& node, const std::string& what) const {
        builder::fail(file_, node, what);
    }

    /// `node[key]` as text; "" when absent. A map or list where text belongs is an error.
    std::string text(const YAML::Node& node, const char* key, bool required = false) const {
        const YAML::Node value = node[key];
        if (!value || value.IsNull()) {
            if (required) {
                fail(node, std::string("missing '") + key + "'");
            }
            return {};
        }
        if (!value.IsScalar()) {
            fail(value, std::string("'") + key + "' must be a single value");
        }
        return value.Scalar();
    }

    std::optional<double> number(const YAML::Node& node, const char* key) const {
        const std::string value = text(node, key);
        if (value.empty()) {
            return std::nullopt;
        }
        // YAML's infinities (.inf, -.inf, +.Inf, ...): "no limit".
        static const std::set<std::string, std::less<>> kInfinity{".inf",  ".Inf",  ".INF",
                                                                  "+.inf", "+.Inf", "+.INF"};
        if (kInfinity.contains(value)) {
            return std::numeric_limits<double>::infinity();
        }
        if (value.starts_with("-.") && kInfinity.contains(std::string_view(value).substr(1))) {
            return -std::numeric_limits<double>::infinity();
        }
        char* end = nullptr;
        const double parsed = std::strtod(value.c_str(), &end);
        if (end != value.c_str() + value.size() || !std::isfinite(parsed)) {
            fail(node[key], std::string("'") + key + "' = '" + value + "' is not a number");
        }
        return parsed;
    }

    bool flag(const YAML::Node& node, const char* key) const {
        const std::string value = text(node, key);
        if (value.empty() || value == "false") {
            return false;
        }
        if (value == "true") {
            return true;
        }
        fail(node[key], std::string("'") + key + "' must be true or false");
    }

    std::vector<std::string> words(const YAML::Node& node, const char* key) const {
        const YAML::Node value = node[key];
        std::vector<std::string> out;
        if (!value || value.IsNull()) {
            return out;
        }
        if (value.IsScalar()) {
            out.push_back(value.Scalar());
            return out;
        }
        if (!value.IsSequence()) {
            fail(value, std::string("'") + key + "' must be a list");
        }
        for (const YAML::Node& item : value) {
            if (!item.IsScalar()) {
                fail(item, std::string("each of '") + key + "' must be a single value");
            }
            out.push_back(item.Scalar());
        }
        return out;
    }

    CatalogParam param(const YAML::Node& node) const {
        if (!node.IsMap()) {
            fail(node, "a parameter must be a map with 'path' and 'type'");
        }
        CatalogParam param;
        param.path = text(node, "path", true);
        const std::string type = text(node, "type", true);
        if (type == "number") {
            param.type = CatalogParam::Type::Number;
        } else if (type == "int") {
            param.type = CatalogParam::Type::Int;
        } else if (type == "bool") {
            param.type = CatalogParam::Type::Bool;
        } else if (type == "text" || type == "pose") {
            param.type = CatalogParam::Type::Text;
        } else if (type == "enum") {
            param.type = CatalogParam::Type::Enum;
        } else if (type == "list") {
            param.type = CatalogParam::Type::List;
        } else {
            fail(node["type"], "parameter " + param.path + ": unknown type '" + type +
                                   "' (number, int, bool, text, enum, list)");
        }
        param.unit = text(node, "unit");
        const auto finite = [](std::optional<double> v) {
            return v && std::isfinite(*v) ? v : std::nullopt;  // an infinite bound is none
        };
        param.min = finite(number(node, "min"));
        param.max = finite(number(node, "max"));
        // An exclusive bound is held as the bound with equality refused (the code's `> 0`).
        if (const auto bound = finite(number(node, "min_exclusive"))) {
            if (!param.min || *bound >= *param.min) {
                param.min = bound;
                param.minExclusive = true;
            }
        }
        if (const auto bound = finite(number(node, "max_exclusive"))) {
            if (!param.max || *bound <= *param.max) {
                param.max = bound;
                param.maxExclusive = true;
            }
        }
        param.options = words(node, "choices");
        if (param.type == CatalogParam::Type::Enum && param.options.empty()) {
            fail(node, "parameter " + param.path + ": an enum needs 'choices'");
        }
        param.meaning = text(node, "meaning");
        const std::string defaultFrom = text(node, "default_from");
        if (!defaultFrom.empty()) {
            param.meaning += (param.meaning.empty() ? "" : "; ") + ("default: " + defaultFrom);
        }
        const std::string onlyWhen = describeOnlyWhen(node);
        if (!onlyWhen.empty()) {
            param.meaning += (param.meaning.empty() ? "" : "; ") + ("read when " + onlyWhen);
        }
        param.defaultText = text(node, "default");
        if (!param.defaultText.empty()) {
            // Numbers as the form writes them ("4" for 4.0), so "differs from the catalog"
            // compares like with like; the default must pass its own checks.
            if (param.type == CatalogParam::Type::Number || param.type == CatalogParam::Type::Int) {
                const auto value = number(node, "default");
                if (std::isfinite(*value)) {
                    param.defaultText = formatNumber(*value, 15);
                } else {
                    // No text the form can hold: the field stays empty, which is the default.
                    param.defaultText.clear();
                    param.meaning += (param.meaning.empty() ? "" : "; ") +
                                     std::string(*value > 0 ? "default: none (∞)" : "default: −∞");
                }
            }
            const std::string error = checkParam(param, param.defaultText);
            if (!param.defaultText.empty() && !error.empty()) {
                fail(node["default"], "parameter " + param.path + ": the default " +
                                          param.defaultText + " is not valid (" + error + ")");
            }
        }
        return param;
    }

    std::string describeOnlyWhen(const YAML::Node& node) const {
        const YAML::Node when = node["only_when"];
        if (!when || !when.IsMap()) {
            return {};
        }
        std::string out;
        for (const auto& item : when) {
            if (item.first.IsScalar() && item.second.IsScalar()) {
                out +=
                    (out.empty() ? "" : ", ") + item.first.Scalar() + " = " + item.second.Scalar();
            }
        }
        return out;
    }

    std::vector<CatalogOutput> outputs(const YAML::Node& node, const char* key,
                                       const char* nameKey) const {
        std::vector<CatalogOutput> out;
        const YAML::Node list = node[key];
        if (!list || list.IsNull()) {
            return out;
        }
        if (!list.IsSequence()) {
            fail(list, std::string("'") + key + "' must be a list");
        }
        for (const YAML::Node& item : list) {
            if (!item.IsMap()) {
                fail(item, std::string("each of '") + key + "' must be a map");
            }
            out.push_back(
                CatalogOutput{text(item, nameKey, true), text(item, "unit"), text(item, "type")});
        }
        return out;
    }

    CatalogEntry entry(const std::string& key, const YAML::Node& node, bool type) const {
        if (!node.IsMap()) {
            fail(node, "sensor '" + key + "' must be a map");
        }
        CatalogEntry entry;
        entry.name = key;
        entry.builtinType = type;
        entry.urdfTypes = words(node, "urdf_types");
        const std::vector<std::string> elements = words(node, "element");
        entry.element = elements.empty() ? std::string{} : elements.front();
        entry.overlayOnly = flag(node, "overlay_only");
        entry.group = type && !entry.overlayOnly ? std::string(kBuiltinGroup)
                                                 : groupOf(text(node, "category"));
        entry.summary = text(node, "description");
        if (entry.summary.empty()) {
            entry.summary = text(node, "name");
        }
        entry.doc = text(node, "doc");
        entry.source = file_;
        const YAML::Node params = node["params"];
        if (params && !params.IsNull()) {
            if (!params.IsSequence()) {
                fail(params, "sensor '" + key + "': 'params' must be a list");
            }
            std::set<std::string, std::less<>> paths;
            for (const YAML::Node& item : params) {
                CatalogParam param = this->param(item);
                if (!paths.insert(param.path).second) {
                    fail(item,
                         "sensor '" + key + "': parameter " + param.path + " is listed twice");
                }
                entry.params.push_back(std::move(param));
            }
        }
        entry.signals = outputs(node, "signals", "name");
        entry.rosTopics = outputs(node, "ros", "topic");
        return entry;
    }

private:
    std::string file_;
};

YAML::Node parse(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw Failure{{path.string(), 0, "cannot be read"}};
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    try {
        return YAML::Load(buffer.str());
    } catch (const YAML::Exception& e) {
        throw Failure{{path.string(), e.mark.line >= 0 ? e.mark.line + 1 : 0, e.msg}};
    }
}

}  // namespace

std::variant<SensorCatalog, SensorCatalog::LoadError> SensorCatalog::load(
    const std::vector<std::filesystem::path>& files) {
    struct Loaded {
        CatalogEntry entry;
        bool plugin{false};
        int line{0};
    };
    std::vector<Loaded> loaded;
    std::map<std::string, std::pair<std::string, int>, std::less<>> names;  // → file, line
    try {
        for (const std::filesystem::path& path : files) {
            const YAML::Node root = parse(path);
            const Reader reader(path.string());
            if (!root.IsMap()) {
                reader.fail(root, "a catalog file is a map with schema_version and kinds");
            }
            const YAML::Node version = root["schema_version"];
            if (!version) {
                reader.fail(root, "missing 'schema_version'");
            }
            if (reader.text(root, "schema_version") != "1") {
                reader.fail(version, "schema_version " + version.Scalar() +
                                         " is not one this viewer reads (1)");
            }
            const bool plugin = root["plugin"].IsDefined() && !root["plugin"].IsNull();
            if (plugin && root["types"]) {
                reader.fail(root["types"], "a plugin's catalog adds kinds only, not 'types'");
            }
            const auto section = [&](const char* key, bool type) {
                const YAML::Node map = root[key];
                if (!map || map.IsNull()) {
                    return;
                }
                if (!map.IsMap()) {
                    reader.fail(map, std::string("'") + key + "' must be a map of sensors");
                }
                for (const auto& item : map) {
                    if (!item.first.IsScalar()) {
                        reader.fail(item.first, "a sensor's name must be a single word");
                    }
                    const std::string name = item.first.Scalar();
                    if (!validName(name)) {
                        reader.fail(item.first, "'" + name + "' is not a valid sensor kind name");
                    }
                    CatalogEntry entry = reader.entry(name, item.second, type);
                    const int line = item.first.Mark().line + 1;
                    std::set<std::string, std::less<>> keys{name};
                    keys.insert(entry.urdfTypes.begin(), entry.urdfTypes.end());
                    for (const std::string& k : keys) {
                        if (const auto it = names.find(k); it != names.end()) {
                            reader.fail(item.first, "kind '" + k + "' is already in " +
                                                        it->second.first + ":" +
                                                        std::to_string(it->second.second));
                        }
                    }
                    for (const std::string& k : keys) {
                        names.emplace(k, std::make_pair(path.string(), line));
                    }
                    loaded.push_back(Loaded{std::move(entry), plugin, line});
                }
            };
            section("types", true);
            section("kinds", false);
        }
    } catch (const Failure& failure) {
        return failure.error;
    }

    // Order (SensorCatalog::entries): built-in types, then the SPEC's groups, then any other
    // group, then plugins' kinds; the file's order within each.
    std::vector<std::string> order{std::string(kBuiltinGroup)};
    order.insert(order.end(), kGroups.begin(), kGroups.end());
    for (const Loaded& item : loaded) {
        if (!item.plugin && std::ranges::find(order, item.entry.group) == order.end()) {
            order.push_back(item.entry.group);
        }
    }
    order.emplace_back(kPluginGroup);
    for (Loaded& item : loaded) {
        if (item.plugin) {
            item.entry.group = std::string(kPluginGroup);
        }
    }
    const auto rank = [&](const Loaded& item) {
        return std::ranges::find(order, item.entry.group) - order.begin();
    };
    std::ranges::stable_sort(loaded, [&](const Loaded& a, const Loaded& b) {
        if (rank(a) != rank(b)) {
            return rank(a) < rank(b);
        }
        return a.entry.builtinType && !b.entry.builtinType;  // a group's types first
    });

    auto data = std::make_shared<Data>();
    for (Loaded& item : loaded) {
        const std::size_t index = data->entries.size();
        data->byName.emplace(item.entry.name, index);
        for (const std::string& type : item.entry.urdfTypes) {
            data->byName.emplace(type, index);
        }
        data->entries.push_back(std::move(item.entry));
    }
    SensorCatalog catalog;
    catalog.data_ = std::move(data);
    return catalog;
}

const CatalogEntry* SensorCatalog::find(std::string_view typeOrKind) const {
    const auto it = data_->byName.find(typeOrKind);
    return it != data_->byName.end() ? &data_->entries[it->second] : nullptr;
}

std::span<const CatalogEntry> SensorCatalog::entries() const {
    return data_->entries;
}

namespace {

std::vector<std::filesystem::path> prefixes() {
    std::vector<std::filesystem::path> out;
    const char* path = std::getenv("AMENT_PREFIX_PATH");
    if (path == nullptr) {
        return out;
    }
    std::string_view rest(path);
    while (!rest.empty()) {
        const std::size_t colon = rest.find(':');
        const std::string_view prefix = rest.substr(0, colon);
        if (!prefix.empty()) {
            out.emplace_back(prefix);
        }
        rest = colon == std::string_view::npos ? std::string_view{} : rest.substr(colon + 1);
    }
    return out;
}

}  // namespace

std::vector<std::filesystem::path> catalogFiles() {
    std::error_code error;
    std::vector<std::filesystem::path> files;
    const std::vector<std::filesystem::path> roots = prefixes();
    // The built-in catalog: the viewer's own copy (plan R11), else lagrange_sensors' file.
    for (const char* share : {"share/lagrange_viewer/catalog/sensor_catalog.yaml",
                              "share/lagrange_sensors/catalog/sensor_catalog.yaml"}) {
        for (const std::filesystem::path& prefix : roots) {
            if (std::filesystem::is_regular_file(prefix / share, error)) {
                files.push_back(prefix / share);
                break;
            }
        }
        if (!files.empty()) {
            break;
        }
    }
    // Plugin catalogs: each package's index entry under the first prefix that has it (as
    // ament_index does), one file per line, relative to that prefix.
    std::set<std::string, std::less<>> packages;
    for (const std::filesystem::path& prefix : roots) {
        const std::filesystem::path index =
            prefix / "share/ament_index/resource_index/lagrange_sensor_catalog";
        if (!std::filesystem::is_directory(index, error)) {
            continue;
        }
        std::vector<std::filesystem::path> entries;
        for (const auto& item : std::filesystem::directory_iterator(index, error)) {
            entries.push_back(item.path());
        }
        std::ranges::sort(entries);
        for (const std::filesystem::path& entry : entries) {
            if (!packages.insert(entry.filename().string()).second) {
                continue;
            }
            std::ifstream in(entry);
            std::string line;
            while (std::getline(in, line)) {
                while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
                    line.pop_back();
                }
                if (!line.empty()) {
                    files.push_back(std::filesystem::path(line).is_absolute()
                                        ? std::filesystem::path(line)
                                        : prefix / line);
                }
            }
        }
    }
    return files;
}

}  // namespace lagrange::viewer::builder
