// See SchematicLayout.hpp. One node per line: stage, prim, x, y, separated by tabs.
#include "lagrange/viewer/SchematicLayout.hpp"

#include <exception>
#include <fstream>
#include <sstream>
#include <vector>

namespace lagrange::viewer {

SchematicLayouts loadSchematicLayouts(const std::string& path) {
    SchematicLayouts layouts;
    std::ifstream in(path);
    for (std::string line; std::getline(in, line);) {
        std::vector<std::string> fields;
        std::istringstream row(line);
        for (std::string field; std::getline(row, field, '\t');) {
            fields.push_back(field);
        }
        if (fields.size() != 4) {
            continue;
        }
        try {
            layouts[fields[0]][fields[1]] = {std::stof(fields[2]), std::stof(fields[3])};
        } catch (const std::exception&) {
            continue;  // a damaged line is a node in its automatic place
        }
    }
    return layouts;
}

bool saveSchematicLayouts(const std::string& path, const SchematicLayouts& layouts) {
    std::ofstream out(path);
    for (const auto& [stage, nodes] : layouts) {
        for (const auto& [prim, offset] : nodes) {
            out << stage << '\t' << prim << '\t' << offset.x << '\t' << offset.y << '\n';
        }
    }
    out.close();
    return static_cast<bool>(out);
}

}  // namespace lagrange::viewer
