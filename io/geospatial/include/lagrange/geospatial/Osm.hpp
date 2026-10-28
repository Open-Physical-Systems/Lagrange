#pragma once

#include "lagrange/module/Expected.hpp"

#include <array>
#include <string>
#include <vector>

namespace lagrange::geospatial {

/// What a world takes from OpenStreetMap (huge-worlds.md §4): buildings and roads. OSM data is
/// ODbL: a world made from it carries the attribution, and its tiles are not committed to the
/// repository.
struct OsmBuilding {
    std::vector<std::array<double, 2>> footprint;  ///< latitude, longitude (rad), open ring
    double height{0.0};                            ///< m above the ground
    std::string id;                                ///< "way/123"
};

struct OsmRoad {
    std::vector<std::array<double, 2>> line;  ///< latitude, longitude (rad)
    std::string kind;                         ///< the highway tag: "primary", "footway", ...
    double width{0.0};                        ///< m
};

struct OsmData {
    std::vector<OsmBuilding> buildings;
    std::vector<OsmRoad> roads;
    std::vector<std::string> notes;  ///< what was left out
};

/// Reads an OSM XML file (.osm, as the editing API and Overpass write it). Buildings are closed
/// ways tagged building=*, their height from `height` (m), else `building:levels` x 3 m, else
/// 9 m. Roads are ways tagged highway=*, their width from `width`, else by kind. Multipolygon
/// relations are left out (noted).
[[nodiscard]] Expected<OsmData, std::string> readOsm(const std::string& path);

}  // namespace lagrange::geospatial
