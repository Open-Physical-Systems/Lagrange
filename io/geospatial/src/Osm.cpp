// OpenStreetMap XML (Osm.hpp).
#include "lagrange/geospatial/Osm.hpp"

#include <charconv>
#include <cmath>
#include <map>
#include <numbers>
#include <optional>
#include <string_view>
#include <tinyxml2.h>

namespace lagrange::geospatial {

namespace {

constexpr double kRadian = std::numbers::pi / 180.0;
constexpr double kStoreyM = 3.0;           // a storey, for building:levels
constexpr double kDefaultBuildingM = 9.0;  // three storeys when the map says nothing

/// A number at the start of a tag value ("12", "12.5 m"); empty if there is none.
std::optional<double> leadingNumber(std::string_view text) {
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end == text.data() || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

/// Carriageway width by kind when the map gives none: typical values, not a standard.
double defaultWidth(std::string_view kind) {
    if (kind == "motorway" || kind == "trunk") {
        return 12.0;
    }
    if (kind == "primary" || kind == "secondary") {
        return 8.0;
    }
    if (kind == "tertiary" || kind == "residential" || kind == "unclassified" ||
        kind == "service") {
        return 6.0;
    }
    return 2.0;  // footways, cycleways, paths, tracks
}

}  // namespace

Expected<OsmData, std::string> readOsm(const std::string& path) {
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{"cannot read " + path +
                          " as OSM XML: " + std::string(document.ErrorStr())};
    }
    const tinyxml2::XMLElement* root = document.FirstChildElement("osm");
    if (root == nullptr) {
        return Unexpected{path + ": not an OSM XML file (no <osm> element)"};
    }
    std::map<std::string, std::array<double, 2>> nodes;
    for (const auto* n = root->FirstChildElement("node"); n != nullptr;
         n = n->NextSiblingElement("node")) {
        const char* id = n->Attribute("id");
        if (id != nullptr) {
            nodes[id] = {n->DoubleAttribute("lat") * kRadian, n->DoubleAttribute("lon") * kRadian};
        }
    }
    OsmData out;
    std::size_t relations = 0;
    for (const auto* r = root->FirstChildElement("relation"); r != nullptr;
         r = r->NextSiblingElement("relation")) {
        ++relations;
    }
    if (relations > 0) {
        out.notes.push_back(std::to_string(relations) +
                            " relations left out (multipolygon buildings are not read yet)");
    }
    std::size_t missing = 0;
    for (const auto* w = root->FirstChildElement("way"); w != nullptr;
         w = w->NextSiblingElement("way")) {
        std::map<std::string, std::string, std::less<>> tags;
        for (const auto* t = w->FirstChildElement("tag"); t != nullptr;
             t = t->NextSiblingElement("tag")) {
            const char* k = t->Attribute("k");
            const char* v = t->Attribute("v");
            if (k != nullptr && v != nullptr) {
                tags[k] = v;
            }
        }
        std::vector<std::string> refs;
        for (const auto* nd = w->FirstChildElement("nd"); nd != nullptr;
             nd = nd->NextSiblingElement("nd")) {
            if (const char* ref = nd->Attribute("ref")) {
                refs.emplace_back(ref);
            }
        }
        std::vector<std::array<double, 2>> points;
        bool complete = true;
        for (const std::string& ref : refs) {
            const auto it = nodes.find(ref);
            if (it == nodes.end()) {
                complete = false;
                break;
            }
            points.push_back(it->second);
        }
        if (!complete) {
            ++missing;
            continue;
        }
        const std::string id = std::string("way/") + (w->Attribute("id") ? w->Attribute("id") : "");
        if (tags.contains("building") && points.size() >= 4 && refs.front() == refs.back()) {
            OsmBuilding b;
            points.pop_back();  // the ring's closing point
            b.footprint = std::move(points);
            b.id = id;
            if (const auto it = tags.find("height"); it != tags.end()) {
                b.height = leadingNumber(it->second).value_or(0.0);
            }
            if (b.height <= 0.0) {
                if (const auto it = tags.find("building:levels"); it != tags.end()) {
                    b.height = leadingNumber(it->second).value_or(0.0) * kStoreyM;
                }
            }
            if (b.height <= 0.0) {
                b.height = kDefaultBuildingM;
            }
            out.buildings.push_back(std::move(b));
        } else if (const auto highway = tags.find("highway");
                   highway != tags.end() && points.size() >= 2) {
            OsmRoad road;
            road.line = std::move(points);
            road.kind = highway->second;
            road.width = defaultWidth(road.kind);
            if (const auto it = tags.find("width"); it != tags.end()) {
                road.width = leadingNumber(it->second).value_or(road.width);
            }
            out.roads.push_back(std::move(road));
        }
    }
    if (missing > 0) {
        out.notes.push_back(std::to_string(missing) +
                            " ways left out: their nodes are not in the file (cut at its edge)");
    }
    return out;
}

}  // namespace lagrange::geospatial
