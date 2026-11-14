#pragma once

// The sensor catalog (RFC 0016 §5): the built-in URDF sensor types and the built-in kinds, with
// their parameters, units, defaults and ranges, read from YAML files. Owner: WP-C.
//
// Files: modules/sensors/catalog/sensor_catalog.yaml (installed by lagrange_sensors to
// share/lagrange_sensors/catalog/, and copied by the viewer to share/lagrange_viewer/catalog/,
// R11), plus plugin catalogs under the ament resource `lagrange_sensor_catalog`.

#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace lagrange::viewer::builder {

/// One parameter of a type or kind. `path` is the kind's own element path ("range/max" is
/// `<range><max>`), so the writer turns the form back into the XML the server reads.
struct CatalogParam {
    std::string path;
    enum class Type { Number, Int, Bool, Text, Enum, List } type{Type::Number};
    std::string unit;         ///< SI as the file holds it ("m", "rad", "Hz", "1"); empty: none
    std::string defaultText;  ///< the default as text, in `unit`; empty: no default
    std::optional<double> min, max;    ///< inclusive bounds in `unit`, for Number and Int
    std::vector<std::string> options;  ///< Enum: the allowed values
    std::string meaning;               ///< one line for the form's tooltip
};

/// A signal or ROS topic the sensor will have (Properties → Output).
struct CatalogOutput {
    std::string name;  ///< signal "range" (under sensor/<name>/) or topic "range" (under /<name>/)
    std::string unit;  ///< signal unit; empty for topics
    std::string type;  ///< "bool", "Float64", "sensor_msgs/msg/Range", ...
};

struct CatalogEntry {
    std::string name;                    ///< "lidar" (a type) or "rangefinder" (a kind)
    bool builtinType{false};             ///< a built-in URDF type (camera, lidar, imu), not a kind
    std::vector<std::string> urdfTypes;  ///< the `type=` words the reader takes for it
    std::string element;                 ///< wrapper element the writer writes (plan decision 6)
    std::string group;        ///< catalog panel group: "range_and_contact", "vision", ...
    std::string summary;      ///< one line for the catalog row
    std::string doc;          ///< "docs/design/domains/sensors.md#rangefinder"
    bool overlayOnly{false};  ///< GNSS, magnetometer, barometer, radar: need the overlay (ph. 3)
    std::string source;       ///< the file it came from (built-in or a plugin's)
    std::vector<CatalogParam> params;
    std::vector<CatalogOutput> signals;
    std::vector<CatalogOutput> rosTopics;
};

class SensorCatalog {
public:
    struct LoadError {
        std::string file;
        int line{0};  ///< 1-based; 0 when the error is not at a line (file missing)
        std::string message;
    };

    SensorCatalog();  ///< empty: no entries; every sensor gets a free key/value form
    /// Loads and merges `files` in order. A malformed file, or a kind named twice across the
    /// files, is an error naming the file and line.
    [[nodiscard]] static std::variant<SensorCatalog, LoadError> load(
        const std::vector<std::filesystem::path>& files);

    /// The entry for a sensor's type or kind: a type by any of its urdfTypes ("gpu_lidar" finds
    /// "lidar"), a kind by its name. nullptr when unknown.
    [[nodiscard]] const CatalogEntry* find(std::string_view typeOrKind) const;
    /// Built-in types first, then kinds by group in the file's order, then plugin kinds.
    [[nodiscard]] std::span<const CatalogEntry> entries() const;

private:
    struct Data;
    std::shared_ptr<const Data> data_;  // shared: a catalog is immutable once loaded
};

/// The catalog files on this machine: the viewer's own copy (share/lagrange_viewer/catalog/)
/// and every file named by the ament resource `lagrange_sensor_catalog`, found through
/// $AMENT_PREFIX_PATH (no ament_index_cpp link needed). Empty when none is installed.
[[nodiscard]] std::vector<std::filesystem::path> catalogFiles();

}  // namespace lagrange::viewer::builder
