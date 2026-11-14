// Stub (RFC 0016 WP0): WP-C replaces this file with the yaml-cpp loader of RFC 0016 §5.
#include "lagrange/viewer/builder/SensorCatalog.hpp"

namespace lagrange::viewer::builder {

struct SensorCatalog::Data {
    std::vector<CatalogEntry> entries;
};

SensorCatalog::SensorCatalog() : data_(std::make_shared<const Data>()) {}

std::variant<SensorCatalog, SensorCatalog::LoadError> SensorCatalog::load(
    const std::vector<std::filesystem::path>& files) {
    if (files.empty()) {
        return SensorCatalog{};  // nothing installed: an empty catalog, not an error
    }
    return LoadError{files.front().string(), 0,
                     "the sensor catalog loader is not implemented yet (RFC 0016 WP-C)"};
}

const CatalogEntry* SensorCatalog::find(std::string_view /*typeOrKind*/) const {
    return nullptr;
}

std::span<const CatalogEntry> SensorCatalog::entries() const {
    return data_->entries;
}

std::vector<std::filesystem::path> catalogFiles() {
    return {};  // not implemented yet (WP-C)
}

}  // namespace lagrange::viewer::builder
