#pragma once

#include "lagrange/module/Expected.hpp"

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace lagrange::stage {

/// The overlay (applyOverlay's YAML) for a SPICE netlist's text; rules in formats.md §9.1.
/// Whatever it cannot simulate is refused, every problem together, never left out. `extra` are
/// more `*lagrange` lines, from outside the file.
[[nodiscard]] Expected<YAML::Node, std::string> overlayOfNetlist(
    const std::string& text, const std::vector<std::string>& extra = {});

/// Whether a path's extension says it is a netlist.
[[nodiscard]] bool isNetlistPath(const std::string& path);

}  // namespace lagrange::stage
