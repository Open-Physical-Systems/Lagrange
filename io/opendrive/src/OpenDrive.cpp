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

constexpr int kTableSteps = 4000;     // arc-length tables of poly3 and paramPoly3
constexpr double kSpiralPanel = 0.5;  // m, Gauss–Legendre panels along a spiral
constexpr double kBrokenOn = 3.0;     // m, a broken road mark's painted length (illustrative;
constexpr double kBrokenOff = 6.0;    // m  OpenDRIVE leaves it to <type> elements)
constexpr double kMarkLift = 0.003;   // m, road marks above the surface

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
    for (auto* w = e->FirstChildElement("width"); w != nullptr;
         w = w->NextSiblingElement("width")) {
        lane.width.push_back(poly(w, "sOffset"));
    }
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

std::array<double, 3> pointAt(const Road& road, double s, double t) {
    const Frame f = referenceAt(road, s);
    return {f.x - t * std::sin(f.hdg), f.y + t * std::cos(f.hdg), piecewise(road.elevation, s)};
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

    /// Appends a strip between two polylines (left and right as seen along them) to `mesh`.
    static void strip(TriMesh& mesh, const std::vector<std::array<double, 3>>& a,
                      const std::vector<std::array<double, 3>>& b) {
        const auto base = static_cast<std::uint32_t>(mesh.positions.size() / 3);
        for (std::size_t i = 0; i < a.size(); ++i) {
            for (const auto* p : {&a[i], &b[i]}) {
                for (const double c : *p) {
                    mesh.positions.push_back(static_cast<float>(c));
                }
                mesh.normals.insert(mesh.normals.end(), {0.0F, 0.0F, 1.0F});
            }
        }
        for (std::uint32_t i = 0; i + 1 < a.size(); ++i) {
            const std::uint32_t p = base + 2 * i;
            // Counter-clockwise seen from above whichever side a is on.
            mesh.indices.insert(mesh.indices.end(), {p, p + 1, p + 2, p + 1, p + 3, p + 2});
        }
    }

    void addMesh(const std::string& name, TriMesh mesh, std::uint32_t mat) {
        if (mesh.indices.empty()) {
            return;
        }
        // Make every triangle face up (the strips' sides depend on the lane's side).
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const auto p = [&](std::size_t k) {
                const std::uint32_t i = mesh.indices[t + k];
                return std::array<float, 2>{mesh.positions[3 * i], mesh.positions[3 * i + 1]};
            };
            const auto a = p(0);
            const auto b = p(1);
            const auto c = p(2);
            if ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0.0F) {
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

    /// A road mark along the border at lateral offset `border(s)` from s0 to s1.
    void mark(TriMesh& mesh, const Road& road, const RoadMark& mark, double s0, double s1,
              const std::function<double(double)>& border) {
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
                    auto pa = pointAt(road, s, t + w / 2.0);
                    auto pb = pointAt(road, s, t - w / 2.0);
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
        for (std::size_t k = 0; k < road.sections.size(); ++k) {
            const LaneSection& sec = road.sections[k];
            const double end = k + 1 < road.sections.size() ? road.sections[k + 1].s : road.length;
            if (end <= sec.s) {
                continue;
            }
            const std::vector<double> ss = samples(sec.s, end);
            for (const std::vector<Lane>* side : {&sec.left, &sec.right}) {
                for (const Lane& lane : *side) {
                    std::vector<std::array<double, 3>> inner, outer, centre;
                    std::vector<double> widths;
                    for (const double s : ss) {
                        const auto [ti, to] = laneBorders(road, sec, lane.id, s);
                        auto pi = pointAt(road, s, ti);
                        auto po = pointAt(road, s, to);
                        pi[2] += o_.surfaceHeight;
                        po[2] += o_.surfaceHeight;
                        inner.push_back(pi);
                        outer.push_back(po);
                        centre.push_back(pointAt(road, s, 0.5 * (ti + to)));
                        widths.push_back(std::abs(to - ti));
                    }
                    if (const auto mat = laneMaterial(lane.type)) {
                        strip(surfaces[*mat], inner, outer);
                    }
                    // Road marks on the lane's outer border.
                    for (std::size_t r = 0; r < lane.marks.size(); ++r) {
                        const RoadMark& rm = lane.marks[r];
                        const double from = sec.s + rm.sOffset;
                        const double to =
                            r + 1 < lane.marks.size() ? sec.s + lane.marks[r + 1].sOffset : end;
                        mark(rm.color == "yellow" ? yellow : white, road, rm, from, to,
                             [&](double s) { return laneBorders(road, sec, lane.id, s).second; });
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
                mark(rm.color == "yellow" ? yellow : white, road, rm, from, to,
                     [&](double s) { return piecewise(road.laneOffset, s); });
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
