#include "lagrange/importers/Importers.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <tinyxml2.h>

namespace lagrange::importers {

Format formatOf(const std::string& path) {
    const std::string extension = std::filesystem::path(path).extension().string();
    if (extension == ".usd" || extension == ".usda" || extension == ".usdc") {
        return Format::Usd;
    }
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return Format::Unknown;
    }
    const tinyxml2::XMLElement* root = document.RootElement();
    const std::string_view name = root != nullptr ? root->Name() : "";
    if (name == "robot") {
        return Format::Urdf;
    }
    if (name == "sdf") {
        return Format::Sdf;
    }
    if (name == "mujoco") {
        return Format::Mjcf;
    }
    return Format::Unknown;
}

}  // namespace lagrange::importers
