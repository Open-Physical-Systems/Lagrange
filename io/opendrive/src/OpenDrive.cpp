// OpenDRIVE road networks (OpenDrive.hpp).
#include "lagrange/opendrive/OpenDrive.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <tinyxml2.h>
#include <tuple>

namespace lagrange::opendrive {

namespace {

constexpr int kTableSteps = 4000;       // arc-length tables of poly3 and paramPoly3
constexpr double kSpiralPanel = 0.5;    // m, Gauss–Legendre panels along a spiral
constexpr double kBrokenOn = 3.0;       // m, a broken road mark's painted length (illustrative;
constexpr double kBrokenOff = 6.0;      // m  OpenDRIVE leaves it to <type> elements)
constexpr double kMarkLift = 0.003;     // m, road marks above the surface
constexpr double kKerbMin = 0.005;      // m, a step between lanes drawn as a kerb face
constexpr double kMinSlabWidth = 0.05;  // m, narrower lanes get no collider there

double attr(const tinyxml2::XMLElement* e, const char* name, double fallback = 0.0) {
    return e->DoubleAttribute(name, fallback);
}

std::string text(const tinyxml2::XMLElement* e, const char* name) {
    const char* v = e->Attribute(name);
    return v != nullptr ? std::string(v) : std::string();
}

Poly3 poly(const tinyxml2::XMLElement* e, const char* sName) {
    return {attr(e, sName), attr(e, "a"), attr(e, "b"), attr(e, "c"), attr(e, "d")};
}

/// 5-point Gauss–Legendre on [a, b].
template <class F>
double gauss(F f, double a, double b) {
    static constexpr std::array<double, 5> kX{0.0, -0.5384693101056831, 0.5384693101056831,
                                              -0.9061798459386640, 0.9061798459386640};
    static constexpr std::array<double, 5> kW{0.5688888888888889, 0.4786286704993665,
                                              0.4786286704993665, 0.2369268850561891,
                                              0.2369268850561891};
    const double half = 0.5 * (b - a);
    const double mid = 0.5 * (a + b);
    double sum = 0.0;
    for (std::size_t i = 0; i < 5; ++i) {
        sum += kW[i] * f(mid + half * kX[i]);
    }
    return sum * half;
}

/// The curve parameter at arc length s, from the table (linear between entries).
double parameterAt(const Geometry& g, double s) {
    const auto& t = g.table;
    if (t.empty()) {
        return s;
    }
    const auto it = std::lower_bound(t.begin(), t.end(), s,
                                     [](const auto& e, double v) { return e.first < v; });
    if (it == t.begin()) {
        return t.front().second;
    }
    if (it == t.end()) {
        return t.back().second;
    }
    const auto& [s1, p1] = *it;
    const auto& [s0, p0] = *std::prev(it);
    return s1 > s0 ? p0 + (p1 - p0) * (s - s0) / (s1 - s0) : p0;
}

/// Builds the arc-length table of a poly3 or paramPoly3: (s, parameter) pairs.
void buildTable(Geometry& g) {
    std::function<double(double)> speed;
    double end = 0.0;
    if (g.kind == Geometry::Kind::Poly3) {
        speed = [&g](double u) {
            const double dv = g.v[1] + u * (2.0 * g.v[2] + 3.0 * u * g.v[3]);
            return std::sqrt(1.0 + dv * dv);
        };
        end = g.length;  // u never exceeds the arc length
    } else {
        speed = [&g](double p) {
            const double du = g.u[1] + p * (2.0 * g.u[2] + 3.0 * p * g.u[3]);
            const double dv = g.v[1] + p * (2.0 * g.v[2] + 3.0 * p * g.v[3]);
            return std::hypot(du, dv);
        };
        end = g.normalized ? 1.0 : g.length;
    }
    g.table.clear();
    g.table.emplace_back(0.0, 0.0);
    double s = 0.0;
    const double dp = end / kTableSteps;
    for (int i = 0; i < kTableSteps; ++i) {
        const double p0 = i * dp;
        s += gauss(speed, p0, p0 + dp);
        g.table.emplace_back(s, p0 + dp);
        if (g.kind == Geometry::Kind::Poly3 && s >= g.length) {
            break;
        }
    }
}

Lane readLane(const tinyxml2::XMLElement* e) {
    Lane lane;
    lane.id = e->IntAttribute("id");
    lane.type = text(e, "type");
    const std::string level = text(e, "level");
    lane.level = level == "true" || level == "1";
    for (auto* w = e->FirstChildElement("width"); w != nullptr;
         w = w->NextSiblingElement("width")) {
        lane.width.push_back(poly(w, "sOffset"));
    }
    for (auto* h = e->FirstChildElement("height"); h != nullptr;
         h = h->NextSiblingElement("height")) {
        lane.heights.push_back({attr(h, "sOffset"), attr(h, "inner"), attr(h, "outer")});
    }
    std::stable_sort(
        lane.heights.begin(), lane.heights.end(),
        [](const LaneHeight& a, const LaneHeight& b) { return a.sOffset < b.sOffset; });
    for (auto* m = e->FirstChildElement("roadMark"); m != nullptr;
         m = m->NextSiblingElement("roadMark")) {
        lane.marks.push_back(
            {attr(m, "sOffset"), text(m, "type"), text(m, "color"), attr(m, "width", 0.12)});
    }
    if (const auto* link = e->FirstChildElement("link")) {
        if (const auto* p = link->FirstChildElement("predecessor")) {
            lane.predecessor = p->IntAttribute("id");
        }
        if (const auto* s = link->FirstChildElement("successor")) {
            lane.successor = s->IntAttribute("id");
        }
    }
    return lane;
}

Link readLink(const tinyxml2::XMLElement* e) {
    Link link;
    if (e == nullptr) {
        return link;
    }
    const std::string type = text(e, "elementType");
    link.type = type == "junction" ? Link::Type::Junction : Link::Type::Road;
    link.id = e->IntAttribute("elementId");
    link.contactEnd = text(e, "contactPoint") == "end";
    return link;
}

double speedOf(const tinyxml2::XMLElement* speed) {
    const double max = attr(speed, "max");
    const std::string unit = text(speed, "unit");
    if (unit == "km/h") {
        return max / 3.6;
    }
    if (unit == "mph") {
        return max * 0.44704;  // m/s per mph (exact)
    }
    return max;  // m/s (the default)
}

}  // namespace

double piecewise(const std::vector<Poly3>& records, double s) {
    const Poly3* in = nullptr;
    for (const Poly3& r : records) {
        if (r.s0 <= s + 1e-12) {
            in = &r;
        }
    }
    return in != nullptr ? in->at(s) : 0.0;
}

double piecewiseSlope(const std::vector<Poly3>& records, double s) {
    const Poly3* in = nullptr;
    for (const Poly3& r : records) {
        if (r.s0 <= s + 1e-12) {
            in = &r;
        }
    }
    return in != nullptr ? in->slope(s) : 0.0;
}

Frame evaluate(const Geometry& g, double ds) {
    ds = std::clamp(ds, 0.0, g.length);
    const double c = std::cos(g.hdg);
    const double sn = std::sin(g.hdg);
    switch (g.kind) {
        case Geometry::Kind::Line:
            return {g.x + ds * c, g.y + ds * sn, g.hdg};
        case Geometry::Kind::Arc: {
            const double k = g.curvature;
            if (std::abs(k) < 1e-12) {
                return {g.x + ds * c, g.y + ds * sn, g.hdg};
            }
            const double h = g.hdg + k * ds;
            return {g.x + (std::sin(h) - sn) / k, g.y - (std::cos(h) - c) / k, h};
        }
        case Geometry::Kind::Spiral: {
            const double rate = g.length > 0.0 ? (g.curvEnd - g.curvStart) / g.length : 0.0;
            const auto heading = [&](double s) {
                return g.hdg + g.curvStart * s + 0.5 * rate * s * s;
            };
            double x = g.x;
            double y = g.y;
            const int panels = std::max(1, static_cast<int>(std::ceil(ds / kSpiralPanel)));
            for (int i = 0; i < panels; ++i) {
                const double a = ds * i / panels;
                const double b = ds * (i + 1) / panels;
                x += gauss([&](double s) { return std::cos(heading(s)); }, a, b);
                y += gauss([&](double s) { return std::sin(heading(s)); }, a, b);
            }
            return {x, y, heading(ds)};
        }
        case Geometry::Kind::Poly3: {
            const double u = parameterAt(g, ds);
            const double v = g.v[0] + u * (g.v[1] + u * (g.v[2] + u * g.v[3]));
            const double dv = g.v[1] + u * (2.0 * g.v[2] + 3.0 * u * g.v[3]);
            return {g.x + u * c - v * sn, g.y + u * sn + v * c, g.hdg + std::atan(dv)};
        }
        case Geometry::Kind::ParamPoly3: {
            const double p = parameterAt(g, ds);
            const double u = g.u[0] + p * (g.u[1] + p * (g.u[2] + p * g.u[3]));
            const double v = g.v[0] + p * (g.v[1] + p * (g.v[2] + p * g.v[3]));
            const double du = g.u[1] + p * (2.0 * g.u[2] + 3.0 * p * g.u[3]);
            const double dv = g.v[1] + p * (2.0 * g.v[2] + 3.0 * p * g.v[3]);
            return {g.x + u * c - v * sn, g.y + u * sn + v * c, g.hdg + std::atan2(dv, du)};
        }
    }
    return {g.x, g.y, g.hdg};
}

const Road* Network::road(int id) const {
    for (const Road& r : roads) {
        if (r.id == id) {
            return &r;
        }
    }
    return nullptr;
}

Expected<Network, std::string> parse(const std::string& xml) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{std::string("not XML: ") + doc.ErrorStr()};
    }
    const auto* top = doc.FirstChildElement("OpenDRIVE");
    if (top == nullptr) {
        return Unexpected{std::string("no <OpenDRIVE> element")};
    }
    Network n;
    for (auto* r = top->FirstChildElement("road"); r != nullptr;
         r = r->NextSiblingElement("road")) {
        Road road;
        road.id = r->IntAttribute("id");
        road.name = text(r, "name");
        road.length = attr(r, "length");
        road.junction = r->IntAttribute("junction", -1);
        const std::string where = "road " + std::to_string(road.id) + ": ";
        if (const auto* link = r->FirstChildElement("link")) {
            road.predecessor = readLink(link->FirstChildElement("predecessor"));
            road.successor = readLink(link->FirstChildElement("successor"));
        }
        for (auto* t = r->FirstChildElement("type"); t != nullptr;
             t = t->NextSiblingElement("type")) {
            if (const auto* speed = t->FirstChildElement("speed")) {
                road.speedLimit = speedOf(speed);
            }
        }
        const auto* plan = r->FirstChildElement("planView");
        if (plan == nullptr) {
            return Unexpected{where + "no planView"};
        }
        for (auto* g = plan->FirstChildElement("geometry"); g != nullptr;
             g = g->NextSiblingElement("geometry")) {
            Geometry geo;
            geo.s = attr(g, "s");
            geo.x = attr(g, "x");
            geo.y = attr(g, "y");
            geo.hdg = attr(g, "hdg");
            geo.length = attr(g, "length");
            if (g->FirstChildElement("line") != nullptr) {
                geo.kind = Geometry::Kind::Line;
            } else if (const auto* a = g->FirstChildElement("arc")) {
                geo.kind = Geometry::Kind::Arc;
                geo.curvature = attr(a, "curvature");
            } else if (const auto* sp = g->FirstChildElement("spiral")) {
                geo.kind = Geometry::Kind::Spiral;
                geo.curvStart = attr(sp, "curvStart");
                geo.curvEnd = attr(sp, "curvEnd");
            } else if (const auto* p = g->FirstChildElement("poly3")) {
                geo.kind = Geometry::Kind::Poly3;
                geo.v = {attr(p, "a"), attr(p, "b"), attr(p, "c"), attr(p, "d")};
                buildTable(geo);
            } else if (const auto* pp = g->FirstChildElement("paramPoly3")) {
                geo.kind = Geometry::Kind::ParamPoly3;
                geo.u = {attr(pp, "aU"), attr(pp, "bU"), attr(pp, "cU"), attr(pp, "dU")};
                geo.v = {attr(pp, "aV"), attr(pp, "bV"), attr(pp, "cV"), attr(pp, "dV")};
                geo.normalized = text(pp, "pRange") != "arcLength";
                buildTable(geo);
            } else {
                return Unexpected{where +
                                  "a geometry without line, arc, spiral, poly3 or "
                                  "paramPoly3"};
            }
            road.planView.push_back(std::move(geo));
        }
        if (road.planView.empty()) {
            return Unexpected{where + "an empty planView"};
        }
        if (const auto* e = r->FirstChildElement("elevationProfile")) {
            for (auto* el = e->FirstChildElement("elevation"); el != nullptr;
                 el = el->NextSiblingElement("elevation")) {
                road.elevation.push_back(poly(el, "s"));
            }
        }
        if (const auto* lp = r->FirstChildElement("lateralProfile")) {
            for (auto* se = lp->FirstChildElement("superelevation"); se != nullptr;
                 se = se->NextSiblingElement("superelevation")) {
                road.superelevation.push_back(poly(se, "s"));
            }
        }
        const auto* lanes = r->FirstChildElement("lanes");
        if (lanes == nullptr) {
            return Unexpected{where + "no lanes"};
        }
        for (auto* o = lanes->FirstChildElement("laneOffset"); o != nullptr;
             o = o->NextSiblingElement("laneOffset")) {
            road.laneOffset.push_back(poly(o, "s"));
        }
        for (auto* sec = lanes->FirstChildElement("laneSection"); sec != nullptr;
             sec = sec->NextSiblingElement("laneSection")) {
            LaneSection section;
            section.s = attr(sec, "s");
            if (const auto* left = sec->FirstChildElement("left")) {
                for (auto* l = left->FirstChildElement("lane"); l != nullptr;
                     l = l->NextSiblingElement("lane")) {
                    section.left.push_back(readLane(l));
                }
            }
            if (const auto* right = sec->FirstChildElement("right")) {
                for (auto* l = right->FirstChildElement("lane"); l != nullptr;
                     l = l->NextSiblingElement("lane")) {
                    section.right.push_back(readLane(l));
                }
            }
            if (const auto* centre = sec->FirstChildElement("center")) {
                if (const auto* l = centre->FirstChildElement("lane")) {
                    section.centreMarks = readLane(l).marks;
                }
            }
            std::sort(section.left.begin(), section.left.end(),
                      [](const Lane& a, const Lane& b) { return a.id < b.id; });
            std::sort(section.right.begin(), section.right.end(),
                      [](const Lane& a, const Lane& b) { return a.id > b.id; });
            for (std::size_t i = 0; i < section.left.size(); ++i) {
                if (section.left[i].id != static_cast<int>(i) + 1) {
                    return Unexpected{where + "left lanes must be numbered 1, 2, ..."};
                }
            }
            for (std::size_t i = 0; i < section.right.size(); ++i) {
                if (section.right[i].id != -static_cast<int>(i) - 1) {
                    return Unexpected{where + "right lanes must be numbered -1, -2, ..."};
                }
            }
            road.sections.push_back(std::move(section));
        }
        if (road.sections.empty()) {
            return Unexpected{where + "no laneSection"};
        }
        n.roads.push_back(std::move(road));
    }
    for (auto* j = top->FirstChildElement("junction"); j != nullptr;
         j = j->NextSiblingElement("junction")) {
        Junction junction;
        junction.id = j->IntAttribute("id");
        for (auto* c = j->FirstChildElement("connection"); c != nullptr;
             c = c->NextSiblingElement("connection")) {
            Connection connection;
            connection.incomingRoad = c->IntAttribute("incomingRoad");
            connection.connectingRoad = c->IntAttribute("connectingRoad");
            connection.contactEnd = text(c, "contactPoint") == "end";
            for (auto* l = c->FirstChildElement("laneLink"); l != nullptr;
                 l = l->NextSiblingElement("laneLink")) {
                connection.laneLinks.emplace_back(l->IntAttribute("from"), l->IntAttribute("to"));
            }
            junction.connections.push_back(std::move(connection));
        }
        n.junctions.push_back(std::move(junction));
    }
    if (n.roads.empty()) {
        return Unexpected{std::string("no roads")};
    }
    return n;
}

Expected<Network, std::string> read(const std::string& path) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        return Unexpected{path + ": " + doc.ErrorStr()};
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    auto network = parse(printer.CStr());
    if (!network) {
        return Unexpected{path + ": " + network.error()};
    }
    return network;
}

Frame referenceAt(const Road& road, double s) {
    s = std::clamp(s, 0.0, road.length);
    const Geometry* g = &road.planView.front();
    for (const Geometry& geo : road.planView) {
        if (geo.s <= s + 1e-9) {
            g = &geo;
        }
    }
    return evaluate(*g, s - g->s);
}

double superelevationAt(const Road& road, double s) {
    return piecewise(road.superelevation, s);
}

std::array<double, 3> pointAt(const Road& road, double s, double t) {
    const Frame f = referenceAt(road, s);
    const double z = piecewise(road.elevation, s);
    if (road.superelevation.empty()) {
        return {f.x - t * std::sin(f.hdg), f.y + t * std::cos(f.hdg), z};
    }
    const double phi = superelevationAt(road, s);
    const double side = t * std::cos(phi);
    return {f.x - side * std::sin(f.hdg), f.y + side * std::cos(f.hdg), z + t * std::sin(phi)};
}

std::pair<double, double> laneBorders(const Road& road, const LaneSection& section, int id,
                                      double s) {
    const double offset = piecewise(road.laneOffset, s);
    const double ds = s - section.s;
    double inner = offset;
    const std::vector<Lane>& side = id > 0 ? section.left : section.right;
    for (const Lane& lane : side) {
        const double w = std::max(0.0, piecewise(lane.width, ds));
        const double outer = id > 0 ? inner + w : inner - w;
        if (lane.id == id) {
            return {inner, outer};
        }
        inner = outer;
    }
    return {inner, inner};
}

std::array<double, 3> lanePoint(const Road& road, const LaneSection& section, int id, double s,
                                double t) {
    const std::vector<Lane>& side = id > 0 ? section.left : section.right;
    const auto it =
        std::find_if(side.begin(), side.end(), [id](const Lane& l) { return l.id == id; });
    if (id == 0 || it == side.end()) {
        return pointAt(road, s, t);
    }
    const Lane& lane = *it;
    const double ds = s - section.s;
    // The lane's height at t: linear from its inner to its outer border.
    double h = 0.0;
    const auto [inner, outer] = laneBorders(road, section, id, s);
    if (!lane.heights.empty()) {
        const LaneHeight* in = &lane.heights.front();
        for (const LaneHeight& r : lane.heights) {
            if (r.sOffset <= ds + 1e-12) {
                in = &r;
            }
        }
        const double u = outer != inner ? (t - inner) / (outer - inner) : 0.0;
        h = in->inner + (in->outer - in->inner) * u;
    }
    const double phi = superelevationAt(road, s);
    const Frame f = referenceAt(road, s);
    const double z = piecewise(road.elevation, s);
    double across = 0.0;  // to the left of the reference line, level
    double up = z;
    if (lane.level) {
        // Level from the inner border of the first level lane (every lane outside it is level).
        auto first = it;
        while (first != side.begin() && std::prev(first)->level) {
            --first;
        }
        const double tb = laneBorders(road, section, first->id, s).first;
        across = tb * std::cos(phi) + (t - tb);
        up = z + tb * std::sin(phi) + h;
    } else {
        // Along the superelevated t axis, then h along its normal.
        across = t * std::cos(phi) - h * std::sin(phi);
        up = z + t * std::sin(phi) + h * std::cos(phi);
    }
    return {f.x - across * std::sin(f.hdg), f.y + across * std::cos(f.hdg), up};
}

bool raised(const Road& road) {
    const auto any = [](const std::vector<Poly3>& p) {
        return std::ranges::any_of(p, [](const Poly3& r) { return !r.zero(); });
    };
    if (any(road.elevation) || any(road.superelevation)) {
        return true;
    }
    for (const LaneSection& sec : road.sections) {
        for (const std::vector<Lane>* side : {&sec.left, &sec.right}) {
            for (const Lane& lane : *side) {
                for (const LaneHeight& h : lane.heights) {
                    if (h.inner != 0.0 || h.outer != 0.0) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

namespace {

/// Builds the scene's meshes and the lanes' schema instances.
class Builder {
public:
    Builder(const Network& n, const ConvertOptions& o) : n_(n), o_(o) {}

    Expected<Model, std::string> build() {
        m_.name = o_.root;
        ScenePrim top;
        top.name = o_.root;
        top.path = "/" + o_.root;
        top.typeName = "Xform";
        m_.scene.prims.push_back(top);
        for (const Road& road : n_.roads) {
            buildRoad(road);
        }
        if (auto status = linkLanes(); !status) {
            return Unexpected{status.error()};
        }
        return std::move(m_);
    }

private:
    struct LaneKey {
        int road;
        std::size_t section;
        int lane;
        friend bool operator<(const LaneKey& a, const LaneKey& b) {
            return std::tie(a.road, a.section, a.lane) < std::tie(b.road, b.section, b.lane);
        }
    };

    std::uint32_t material(const std::string& name, std::array<float, 4> rgba) {
        for (std::size_t i = 0; i < m_.scene.materials.size(); ++i) {
            if (m_.scene.materials[i].name == name) {
                return static_cast<std::uint32_t>(i);
            }
        }
        m_.scene.materials.push_back({name, rgba});
        return static_cast<std::uint32_t>(m_.scene.materials.size() - 1);
    }

    /// Material of a lane type, or none for lanes that are not drawn.
    std::optional<std::uint32_t> laneMaterial(const std::string& type) {
        if (type == "driving" || type == "entry" || type == "exit" || type == "onRamp" ||
            type == "offRamp" || type == "connectingRamp" || type == "bidirectional") {
            return material("asphalt", {0.2F, 0.2F, 0.21F, 1.0F});
        }
        if (type == "sidewalk" || type == "walking" || type == "curb") {
            return material("pavement", {0.62F, 0.6F, 0.56F, 1.0F});
        }
        if (type == "parking") {
            return material("parking", {0.25F, 0.25F, 0.27F, 1.0F});
        }
        if (type == "biking") {
            return material("cycle_lane", {0.45F, 0.22F, 0.2F, 1.0F});
        }
        if (type == "shoulder" || type == "border" || type == "stop" || type == "restricted" ||
            type == "median") {
            return material("shoulder", {0.33F, 0.33F, 0.32F, 1.0F});
        }
        return std::nullopt;
    }

    static std::string laneToken(const std::string& type) {
        if (type == "sidewalk" || type == "walking") {
            return "sidewalk";
        }
        if (type == "biking" || type == "parking") {
            return type;
        }
        if (type == "driving" || type == "entry" || type == "exit" || type == "onRamp" ||
            type == "offRamp" || type == "connectingRamp" || type == "bidirectional") {
            return "driving";
        }
        return "";
    }

    std::vector<double> samples(double from, double to) const {
        std::vector<double> s;
        const int n = std::max(1, static_cast<int>(std::ceil((to - from) / o_.step)));
        for (int i = 0; i <= n; ++i) {
            s.push_back(from + (to - from) * i / n);
        }
        return s;
    }

    /// Appends a strip between two polylines (left and right as seen along them) to `mesh`, its
    /// normals across each row and along the strip, turned to the side of `facing` (up for a
    /// surface). A strip that lies level has normals exactly (0, 0, 1).
    static void strip(TriMesh& mesh, const std::vector<std::array<double, 3>>& a,
                      const std::vector<std::array<double, 3>>& b,
                      const std::array<double, 3>& facing = {0.0, 0.0, 1.0}) {
        const auto base = static_cast<std::uint32_t>(mesh.positions.size() / 3);
        const auto mid = [&](std::size_t i, std::size_t k) { return 0.5 * (a[i][k] + b[i][k]); };
        for (std::size_t i = 0; i < a.size(); ++i) {
            const std::size_t i0 = i > 0 ? i - 1 : 0;
            const std::size_t i1 = i + 1 < a.size() ? i + 1 : i;
            const std::array<double, 3> along{mid(i1, 0) - mid(i0, 0), mid(i1, 1) - mid(i0, 1),
                                              mid(i1, 2) - mid(i0, 2)};
            const std::array<double, 3> across{b[i][0] - a[i][0], b[i][1] - a[i][1],
                                               b[i][2] - a[i][2]};
            std::array<double, 3> n{along[1] * across[2] - along[2] * across[1],
                                    along[2] * across[0] - along[0] * across[2],
                                    along[0] * across[1] - along[1] * across[0]};
            const double len = std::hypot(n[0], n[1], n[2]);
            if (!(len > 1e-12)) {
                n = facing;
            } else {
                const double sign =
                    n[0] * facing[0] + n[1] * facing[1] + n[2] * facing[2] < 0.0 ? -1.0 : 1.0;
                for (double& c : n) {
                    c = sign * c / len + 0.0;  // + 0.0: no negative zeros
                }
            }
            for (const auto* p : {&a[i], &b[i]}) {
                for (const double c : *p) {
                    mesh.positions.push_back(static_cast<float>(c));
                }
                mesh.normals.insert(
                    mesh.normals.end(),
                    {static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2])});
            }
        }
        for (std::uint32_t i = 0; i + 1 < a.size(); ++i) {
            const std::uint32_t p = base + 2 * i;
            mesh.indices.insert(mesh.indices.end(), {p, p + 1, p + 2, p + 1, p + 3, p + 2});
        }
    }

    void addMesh(const std::string& name, TriMesh mesh, std::uint32_t mat) {
        if (mesh.indices.empty()) {
            return;
        }
        // Wind every triangle counter-clockwise seen from where its normal points (the strips'
        // sides depend on the lane's side).
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const auto p = [&](std::size_t k) {
                const std::uint32_t i = mesh.indices[t + k];
                return std::array<float, 3>{mesh.positions[3 * i], mesh.positions[3 * i + 1],
                                            mesh.positions[3 * i + 2]};
            };
            const auto a = p(0);
            const auto b = p(1);
            const auto c = p(2);
            const std::uint32_t first = mesh.indices[t];
            const float nx = mesh.normals[3 * first];
            const float ny = mesh.normals[3 * first + 1];
            const float nz = mesh.normals[3 * first + 2];
            const float cz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            const float cx = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]);
            const float cy = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]);
            const float facing = nx == 0.0F && ny == 0.0F ? cz * nz : cx * nx + cy * ny + cz * nz;
            if (facing < 0.0F) {
                std::swap(mesh.indices[t + 1], mesh.indices[t + 2]);
            }
        }
        ScenePrim prim;
        prim.name = name;
        prim.path = "/" + o_.root + "/" + name;
        prim.parent = 0;
        prim.typeName = "Mesh";
        prim.mesh = static_cast<std::uint32_t>(m_.scene.meshes.size());
        prim.material = mat;
        m_.scene.meshes.push_back(std::move(mesh));
        m_.scene.prims.push_back(std::move(prim));
    }

    /// A road mark along the border at lateral offset `border(s)` from s0 to s1, on the
    /// surface `at(s, t)` of the lane it belongs to.
    void mark(TriMesh& mesh, const RoadMark& mark, double s0, double s1,
              const std::function<double(double)>& border,
              const std::function<std::array<double, 3>(double, double)>& at) {
        if (mark.type.empty() || mark.type == "none") {
            return;
        }
        const double w = mark.width > 0.0 ? mark.width : 0.12;
        const bool broken = mark.type.find("broken") != std::string::npos;
        const bool doubled = mark.type.find(' ') != std::string::npos;  // "solid solid", ...
        const double lift = o_.surfaceHeight + kMarkLift;
        for (const double shift : doubled ? std::vector<double>{-w, w} : std::vector<double>{0.0}) {
            double from = s0;
            while (from < s1 - 1e-6) {
                const double to = broken ? std::min(s1, from + kBrokenOn) : s1;
                std::vector<std::array<double, 3>> a, b;
                for (const double s : samples(from, to)) {
                    const double t = border(s) + shift;
                    auto pa = at(s, t + w / 2.0);
                    auto pb = at(s, t - w / 2.0);
                    pa[2] += lift;
                    pb[2] += lift;
                    a.push_back(pa);
                    b.push_back(pb);
                }
                strip(mesh, a, b);
                from = broken ? to + kBrokenOff : s1;
            }
        }
    }

    void buildRoad(const Road& road) {
        std::map<std::uint32_t, TriMesh> surfaces;  // by material
        TriMesh white, yellow;
        const std::string roadName = "road_" + std::to_string(road.id);
        const bool solid = raised(road);
        for (std::size_t k = 0; k < road.sections.size(); ++k) {
            const LaneSection& sec = road.sections[k];
            const double end = k + 1 < road.sections.size() ? road.sections[k + 1].s : road.length;
            if (end <= sec.s) {
                continue;
            }
            const std::vector<double> ss = samples(sec.s, end);
            for (const std::vector<Lane>* side : {&sec.left, &sec.right}) {
                for (const Lane& lane : *side) {
                    const auto at = [&](double s, double t) {
                        return lanePoint(road, sec, lane.id, s, t);
                    };
                    std::vector<std::array<double, 3>> inner, outer, centre;
                    std::vector<double> widths;
                    for (const double s : ss) {
                        const auto [ti, to] = laneBorders(road, sec, lane.id, s);
                        auto pi = at(s, ti);
                        auto po = at(s, to);
                        pi[2] += o_.surfaceHeight;
                        po[2] += o_.surfaceHeight;
                        inner.push_back(pi);
                        outer.push_back(po);
                        centre.push_back(at(s, 0.5 * (ti + to)));
                        widths.push_back(std::abs(to - ti));
                    }
                    // A sidewalk lane inside a junction is a path across what is drawn there.
                    const bool path = road.junction >= 0 && laneToken(lane.type) == "sidewalk";
                    if (const auto mat = laneMaterial(lane.type); mat && !path) {
                        strip(surfaces[*mat], inner, outer);
                        kerb(surfaces[*mat], road, sec, lane, ss, inner);
                        if (solid) {
                            slabs(road, sec, lane, sec.s, end);
                        }
                    }
                    // Road marks on the lane's outer border.
                    for (std::size_t r = 0; r < lane.marks.size(); ++r) {
                        const RoadMark& rm = lane.marks[r];
                        const double from = sec.s + rm.sOffset;
                        const double to =
                            r + 1 < lane.marks.size() ? sec.s + lane.marks[r + 1].sOffset : end;
                        mark(
                            rm.color == "yellow" ? yellow : white, rm, from, to,
                            [&](double s) { return laneBorders(road, sec, lane.id, s).second; },
                            at);
                    }
                    const std::string token = laneToken(lane.type);
                    if (token.empty()) {
                        continue;
                    }
                    // Right lanes run along s, left ones against it.
                    if (lane.id > 0) {
                        std::reverse(centre.begin(), centre.end());
                        std::reverse(widths.begin(), widths.end());
                    }
                    addLane(road, k, lane, token, centre, widths, sec.s, end);
                }
            }
            for (std::size_t r = 0; r < sec.centreMarks.size(); ++r) {
                const RoadMark& rm = sec.centreMarks[r];
                const double from = sec.s + rm.sOffset;
                const double to =
                    r + 1 < sec.centreMarks.size() ? sec.s + sec.centreMarks[r + 1].sOffset : end;
                mark(
                    rm.color == "yellow" ? yellow : white, rm, from, to,
                    [&](double s) { return piecewise(road.laneOffset, s); },
                    [&](double s, double t) { return pointAt(road, s, t); });
            }
        }
        for (auto& [mat, mesh] : surfaces) {
            addMesh(roadName + "_" + m_.scene.materials[mat].name, std::move(mesh), mat);
        }
        addMesh(roadName + "_marks", std::move(white),
                material("mark_white", {0.92F, 0.92F, 0.88F, 1.0F}));
        addMesh(roadName + "_marks_yellow", std::move(yellow),
                material("mark_yellow", {0.95F, 0.78F, 0.2F, 1.0F}));
    }

    /// The kerb face where a lane's inner border stands above or below its inner neighbour's
    /// outer border (lane heights, a level lane), drawn facing the lower side.
    void kerb(TriMesh& mesh, const Road& road, const LaneSection& sec, const Lane& lane,
              const std::vector<double>& ss, const std::vector<std::array<double, 3>>& inner) {
        const int neighbour = lane.id > 0 ? lane.id - 1 : lane.id + 1;  // 0: the centre
        std::vector<std::array<double, 3>> low;
        double most = 0.0;
        double sum = 0.0;
        for (std::size_t i = 0; i < ss.size(); ++i) {
            const double t = laneBorders(road, sec, lane.id, ss[i]).first;
            auto p = lanePoint(road, sec, neighbour, ss[i], t);
            p[2] += o_.surfaceHeight;
            most = std::max(most, std::abs(inner[i][2] - p[2]));
            sum += inner[i][2] - p[2];
            low.push_back(p);
        }
        if (most < kKerbMin) {
            return;
        }
        // Facing across the road towards the neighbour when this lane stands higher.
        const Frame f = referenceAt(road, 0.5 * (ss.front() + ss.back()));
        const double out = (lane.id > 0 ? 1.0 : -1.0) * (sum >= 0.0 ? -1.0 : 1.0);
        strip(mesh, inner, low, {-out * std::sin(f.hdg), out * std::cos(f.hdg), 0.0});
    }

    /// The colliders of a raised road's lane: convex slabs colliderStep long, their top the
    /// drawn surface, colliderDepth thick, each posed at its centre.
    void slabs(const Road& road, const LaneSection& sec, const Lane& lane, double from, double to) {
        const int n = std::max(1, static_cast<int>(std::ceil((to - from) / o_.colliderStep)));
        for (int i = 0; i < n; ++i) {
            const double s0 = from + (to - from) * i / n;
            const double s1 = from + (to - from) * (i + 1) / n;
            TriMesh box;
            std::array<double, 3> c{};
            std::vector<std::array<double, 3>> top;
            for (const double s : {s0, s1}) {
                const auto [ti, tout] = laneBorders(road, sec, lane.id, s);
                if (std::abs(tout - ti) < kMinSlabWidth) {
                    continue;
                }
                for (const double t : {ti, tout}) {
                    auto p = lanePoint(road, sec, lane.id, s, t);
                    p[2] += o_.surfaceHeight;
                    top.push_back(p);
                }
            }
            if (top.size() < 4) {
                continue;  // a lane of no width here
            }
            for (const auto& p : top) {
                for (std::size_t k = 0; k < 3; ++k) {
                    c[k] += p[k] / static_cast<double>(top.size());
                }
            }
            c[2] -= o_.colliderDepth / 2.0;
            for (const auto& p : top) {
                for (const double dz : {0.0, -o_.colliderDepth}) {
                    box.positions.push_back(static_cast<float>(p[0] - c[0]));
                    box.positions.push_back(static_cast<float>(p[1] - c[1]));
                    box.positions.push_back(static_cast<float>(p[2] + dz - c[2]));
                }
            }
            auto hull = convexHullOf(box);
            if (!hull) {
                continue;
            }
            Pose pose;
            pose.position = c;
            m_.world.addHullCollider(kWorldBody, std::move(*hull), pose, o_.friction);
        }
    }

    void addLane(const Road& road, std::size_t section, const Lane& lane, const std::string& type,
                 const std::vector<std::array<double, 3>>& centre,
                 const std::vector<double>& widths, double sStart, double sEnd) {
        const std::string name = "road_" + std::to_string(road.id) + "_s" +
                                 std::to_string(section) + "_lane_" + (lane.id < 0 ? "m" : "p") +
                                 std::to_string(std::abs(lane.id));
        const PrimId prim = m_.world.addPrim("/" + o_.root + "/lanes/" + name, name);
        SchemaInstance s;
        s.schema = "RoadLaneAPI_1";
        s.prim = prim;
        std::vector<double> points;
        for (const auto& p : centre) {
            points.insert(points.end(), p.begin(), p.end());
        }
        s.realArrays = {{"lane:centreLine", points}, {"lane:width", widths}};
        s.tokens = {{"lane:type", type}};
        s.reals = {
            {"lane:speedLimit", road.speedLimit}, {"lane:sStart", sStart}, {"lane:sEnd", sEnd}};
        s.integers = {{"lane:road", road.id},
                      {"lane:lane", lane.id},
                      {"lane:junction", road.junction >= 0 ? 1 : 0}};
        lanes_[{road.id, section, lane.id}] = m_.world.schemaInstances.size();
        m_.world.schemaInstances.push_back(std::move(s));
    }

    /// The lane `id` at one end of `road` (its first or last section).
    std::optional<std::size_t> laneAtEnd(int roadId, bool end, int id) const {
        const Road* r = n_.road(roadId);
        if (r == nullptr) {
            return std::nullopt;
        }
        const std::size_t section = end ? r->sections.size() - 1 : 0;
        const auto it = lanes_.find({roadId, section, id});
        return it == lanes_.end() ? std::nullopt : std::optional(it->second);
    }

    /// The lanes a lane continues into, in its direction of travel.
    std::vector<std::size_t> next(const Road& road, std::size_t k, const Lane& lane) const {
        std::vector<std::size_t> out;
        const bool forward = lane.id < 0;  // right lanes run along s
        const std::optional<int> linkId = forward ? lane.successor : lane.predecessor;
        const bool atRoadEnd = forward ? k + 1 == road.sections.size() : k == 0;
        if (!atRoadEnd) {
            const std::size_t other = forward ? k + 1 : k - 1;
            const int id = linkId.value_or(lane.id);
            if (const auto it = lanes_.find({road.id, other, id}); it != lanes_.end()) {
                out.push_back(it->second);
            }
            return out;
        }
        const Link& link = forward ? road.successor : road.predecessor;
        if (link.type == Link::Type::Road && linkId) {
            if (const auto l = laneAtEnd(link.id, link.contactEnd, *linkId)) {
                out.push_back(*l);
            }
        } else if (link.type == Link::Type::Junction) {
            for (const Junction& j : n_.junctions) {
                if (j.id != link.id) {
                    continue;
                }
                for (const Connection& c : j.connections) {
                    if (c.incomingRoad != road.id) {
                        continue;
                    }
                    for (const auto& [from, to] : c.laneLinks) {
                        if (from == lane.id) {
                            if (const auto l = laneAtEnd(c.connectingRoad, c.contactEnd, to)) {
                                out.push_back(*l);
                            }
                        }
                    }
                }
            }
        }
        return out;
    }

    Expected<void, std::string> linkLanes() {
        std::map<std::size_t, std::vector<PrimId>> successors, predecessors;
        for (const Road& road : n_.roads) {
            for (std::size_t k = 0; k < road.sections.size(); ++k) {
                for (const std::vector<Lane>* side :
                     {&road.sections[k].left, &road.sections[k].right}) {
                    for (const Lane& lane : *side) {
                        const auto it = lanes_.find({road.id, k, lane.id});
                        if (it == lanes_.end()) {
                            continue;
                        }
                        for (const std::size_t to : next(road, k, lane)) {
                            successors[it->second].push_back(m_.world.schemaInstances[to].prim);
                            predecessors[to].push_back(m_.world.schemaInstances[it->second].prim);
                        }
                    }
                }
            }
        }
        for (auto& [index, prims] : successors) {
            m_.world.schemaInstances[index].relationships["lane:successors"] = prims;
        }
        for (auto& [index, prims] : predecessors) {
            m_.world.schemaInstances[index].relationships["lane:predecessors"] = prims;
        }
        return {};
    }

    const Network& n_;
    const ConvertOptions& o_;
    Model m_;
    std::map<LaneKey, std::size_t> lanes_;  // → index in world.schemaInstances
};

}  // namespace

Expected<Model, std::string> toModel(const Network& network, const ConvertOptions& options) {
    return Builder(network, options).build();
}

}  // namespace lagrange::opendrive
