#pragma once

// Where the schematic's nodes were dragged, per stage (04-electrical/SPEC.md: "the layout is
// stored per stage"). A convenience: a file that cannot be read is an empty layout.

#include <map>
#include <string>

namespace lagrange::viewer {

/// How far a node sits from its automatic place, in logical pixels.
struct NodeOffset {
    float x{0.0F};
    float y{0.0F};
};

/// By stage path, then by prim name.
using SchematicLayouts = std::map<std::string, std::map<std::string, NodeOffset>>;

[[nodiscard]] SchematicLayouts loadSchematicLayouts(const std::string& path);
/// Best effort: returns false when the file could not be written.
bool saveSchematicLayouts(const std::string& path, const SchematicLayouts& layouts);

}  // namespace lagrange::viewer
