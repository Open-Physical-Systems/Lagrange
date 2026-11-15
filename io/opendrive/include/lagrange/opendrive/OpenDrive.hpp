#pragma once

// OpenDRIVE road networks (formats.md: roads → USD; domains/traffic.md): an own reader for the
// subset that describes geometry and lanes (ASAM OpenDRIVE 1.4–1.8: planView, elevation,
// superelevation, lane offset, lane sections with width polynomials, lane heights and levels,
// lane and road links, junction connections), and its conversion to a Model: road surface
// meshes, road marks, RoadLaneAPI_1 lanes, and colliders for roads that leave the ground plane.

#include "lagrange/module/Expected.hpp"
#include "lagrange/world/Model.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::opendrive {

/// a + b·ds + c·ds² + d·ds³ from s0 on.
struct Poly3 {
    double s0{0.0};
    double a{0.0}, b{0.0}, c{0.0}, d{0.0};
    [[nodiscard]] double at(double s) const {
        const double ds = s - s0;
        return a + ds * (b + ds * (c + ds * d));
    }
    [[nodiscard]] double slope(double s) const {
        const double ds = s - s0;
        return b + ds * (2.0 * c + 3.0 * ds * d);
    }
    [[nodiscard]] bool zero() const { return a == 0.0 && b == 0.0 && c == 0.0 && d == 0.0; }
};

/// The piecewise polynomial in effect at s: the last record starting at or before it.
[[nodiscard]] double piecewise(const std::vector<Poly3>& records, double s);
/// Its derivative along s.
[[nodiscard]] double piecewiseSlope(const std::vector<Poly3>& records, double s);

/// One planView geometry record.
struct Geometry {
    enum class Kind { Line, Arc, Spiral, Poly3, ParamPoly3 };
    Kind kind{Kind::Line};
    double s{0.0}, x{0.0}, y{0.0}, hdg{0.0}, length{0.0};
    double curvature{0.0};                // arc
    double curvStart{0.0}, curvEnd{0.0};  // spiral
    std::array<double, 4> u{}, v{};       // poly3: v only (a..d); paramPoly3: aU..dU, aV..dV
    bool normalized{true};                // paramPoly3 pRange
    /// Arc length → curve parameter, for poly3 and paramPoly3 (built at load).
    std::vector<std::pair<double, double>> table;
};

/// A point of a reference line: position and heading.
struct Frame {
    double x{0.0}, y{0.0}, hdg{0.0};
};

/// The reference line's frame at `ds` metres into `g` (0 ≤ ds ≤ g.length).
[[nodiscard]] Frame evaluate(const Geometry& g, double ds);

struct RoadMark {
    double sOffset{0.0};
    std::string type;   // "solid", "broken", "solid solid", "none", ...
    std::string color;  // "standard" (white), "yellow", ...
    double width{0.12};
};

/// A lane's <height>: its surface raised along the road's h axis (kerbs, sidewalks), from
/// sOffset (from the section start) until the next record; inner and outer at its borders,
/// linear across the lane between them.
struct LaneHeight {
    double sOffset{0.0};
    double inner{0.0}, outer{0.0};  // m
};

struct Lane {
    int id{0};
    std::string type;          // "driving", "sidewalk", "shoulder", "none", ...
    bool level{false};         // @level="true": kept out of the superelevation
    std::vector<Poly3> width;  // s0 is the record's sOffset (from the section start)
    std::vector<LaneHeight> heights;
    std::vector<RoadMark> marks;
    std::optional<int> predecessor, successor;
};

struct LaneSection {
    double s{0.0};
    std::vector<Lane> left, right;  // left: ids 1, 2, …; right: −1, −2, … (inner first)
    std::vector<RoadMark> centreMarks;
};

/// A road's link: the road or junction before or after it, and which end of it touches.
struct Link {
    enum class Type { None, Road, Junction };
    Type type{Type::None};
    int id{0};
    bool contactEnd{false};  // contactPoint="end" (roads only)
};

struct Road {
    int id{0};
    std::string name;
    double length{0.0};
    int junction{-1};
    Link predecessor, successor;
    double speedLimit{0.0};  // m/s; 0: none
    std::vector<Geometry> planView;
    std::vector<Poly3> elevation;
    /// lateralProfile superelevation (rad): the cross section's roll about the reference line,
    /// positive falling to the right (ASAM OpenDRIVE 1.8 §10.5.2).
    std::vector<Poly3> superelevation;
    std::vector<Poly3> laneOffset;
    std::vector<LaneSection> sections;
};

struct Connection {
    int incomingRoad{0};
    int connectingRoad{0};
    bool contactEnd{false};
    std::vector<std::pair<int, int>> laneLinks;  // from (incoming lane), to (connecting lane)
};

struct Junction {
    int id{0};
    std::vector<Connection> connections;
};

struct Network {
    std::vector<Road> roads;
    std::vector<Junction> junctions;
    [[nodiscard]] const Road* road(int id) const;
};

/// Reads an .xodr file; an error naming the first thing that cannot be used.
[[nodiscard]] Expected<Network, std::string> read(const std::string& path);
[[nodiscard]] Expected<Network, std::string> parse(const std::string& xml);

/// The reference line's frame at s along a road (clamped to the road).
[[nodiscard]] Frame referenceAt(const Road& road, double s);
/// The superelevation at s (rad, positive falling to the right).
[[nodiscard]] double superelevationAt(const Road& road, double s);
/// A point at s and lateral offset t (positive left of the reference line) on the road's
/// cross section: the elevation, and t measured along the superelevated t axis, so that the
/// point is t·cos φ to the side and t·sin φ up of the reference line (φ the superelevation).
[[nodiscard]] std::array<double, 3> pointAt(const Road& road, double s, double t);
/// The lateral offsets of lane `id`'s inner and outer borders at s, within its section.
[[nodiscard]] std::pair<double, double> laneBorders(const Road& road, const LaneSection& section,
                                                    int id, double s);
/// A point at s and lateral offset t on lane `id`'s surface (t may reach past its borders, the
/// surface extended): pointAt, or for a lane with @level="true" the level plane from the inner
/// border of the first level lane outwards, raised along h by the lane's height, linear from
/// inner to outer border. Lane 0 (the centre) is pointAt.
[[nodiscard]] std::array<double, 3> lanePoint(const Road& road, const LaneSection& section, int id,
                                              double s, double t);
/// Whether a road leaves the ground plane: any elevation, superelevation or lane height.
[[nodiscard]] bool raised(const Road& road);

struct ConvertOptions {
    std::string root{"roads"};   // the top prim of the result
    double step{0.5};            // m, the meshes' and centre lines' spacing along s
    double surfaceHeight{0.01};  // m, the surfaces above the ground plane (no z-fighting)
    /// Raised roads (raised()) also collide: a convex slab per drawn lane every colliderStep
    /// along s, its top the drawn surface, colliderDepth thick.
    double colliderStep{2.0};   // m
    double colliderDepth{0.3};  // m
    double friction{1.0};       // the slabs' Coulomb coefficient (dry asphalt, illustrative)
};

/// A Model of the network: road surface meshes per lane type, kerb faces and road marks
/// (scene; roads on the ground plane need no colliders, raised ones get slabs), and a
/// RoadLaneAPI_1 prim per driving,
/// biking, parking or sidewalk lane and lane section, linked to the lanes it continues into
/// across sections, roads and junctions.
[[nodiscard]] Expected<Model, std::string> toModel(const Network& network,
                                                   const ConvertOptions& options = {});

}  // namespace lagrange::opendrive
