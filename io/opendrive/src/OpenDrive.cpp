// OpenDRIVE road networks (OpenDrive.hpp).
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
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
// Objects and signals (illustrative sizes where the file gives none).
constexpr double kThinObject = 0.1;       // m, an object's length or width when it gives none
constexpr double kFlatObject = 1e-3;      // m, objects no taller than this are flat (marks)
constexpr int kCylinderSides = 12;        // a round object drawn as a prism of this many sides
constexpr double kSignPoleRadius = 0.04;  // m
constexpr double kSignSize = 0.6;         // m, a sign board's width and height when not given
constexpr double kSignDepth = 0.03;       // m, a sign board's thickness when not given
constexpr double kHeadWidth = 0.35;       // m, a traffic light head's width when not given
constexpr double kHeadHeight = 1.0;       // m, and its height
constexpr double kHeadDepth = 0.3;        // m, and its depth
constexpr double kStripe = 0.5;           // m, a crosswalk's stripes and gaps
constexpr double kOutlineMark = 0.1;      // m, a parking space's painted border

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

double attrOr(const tinyxml2::XMLElement* e, const char* name, double fallback) {
    return e->Attribute(name) != nullptr ? attr(e, name) : fallback;
}

bool yes(const tinyxml2::XMLElement* e, const char* name) {
    const std::string v = text(e, name);
    return v == "yes" || v == "true" || v == "1";
}

Outline readOutline(const tinyxml2::XMLElement* e) {
    Outline o;
    const std::string closed = text(e, "closed");
    o.closed = closed == "true" || closed == "1";
    for (auto* c = e->FirstChildElement(); c != nullptr; c = c->NextSiblingElement()) {
        const std::string tag = c->Name();
        if (tag == "cornerRoad") {
            o.corners.push_back({attr(c, "s"), attr(c, "t"), attr(c, "dz"), attr(c, "height")});
        } else if (tag == "cornerLocal") {
            o.local = true;
            o.corners.push_back({attr(c, "u"), attr(c, "v"), attr(c, "z"), attr(c, "height")});
        }
    }
    return o;
}

Object readObject(const tinyxml2::XMLElement* e) {
    Object o;
    o.id = text(e, "id");
    o.name = text(e, "name");
    o.type = text(e, "type");
    o.subtype = text(e, "subtype");
    o.s = attr(e, "s");
    o.t = attr(e, "t");
    o.zOffset = attr(e, "zOffset");
    o.hdg = attr(e, "hdg");
    o.pitch = attr(e, "pitch");
    o.roll = attr(e, "roll");
    o.length = attr(e, "length");
    o.width = attr(e, "width");
    o.height = attr(e, "height");
    o.radius = attr(e, "radius");
    o.dynamic = yes(e, "dynamic");
    constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    for (auto* r = e->FirstChildElement("repeat"); r != nullptr;
         r = r->NextSiblingElement("repeat")) {
        Repeat rep;
        rep.s = attr(r, "s", o.s);
        rep.length = attr(r, "length");
        rep.distance = attr(r, "distance");
        rep.tStart = attrOr(r, "tStart", o.t);
        rep.tEnd = attrOr(r, "tEnd", rep.tStart);
        rep.heightStart = attrOr(r, "heightStart", o.height);
        rep.heightEnd = attrOr(r, "heightEnd", rep.heightStart);
        rep.zOffsetStart = attrOr(r, "zOffsetStart", o.zOffset);
        rep.zOffsetEnd = attrOr(r, "zOffsetEnd", rep.zOffsetStart);
        rep.widthStart = attrOr(r, "widthStart", kNone);
        rep.widthEnd = attrOr(r, "widthEnd", rep.widthStart);
        rep.lengthStart = attrOr(r, "lengthStart", kNone);
        rep.lengthEnd = attrOr(r, "lengthEnd", rep.lengthStart);
        rep.radiusStart = attrOr(r, "radiusStart", kNone);
        rep.radiusEnd = attrOr(r, "radiusEnd", rep.radiusStart);
        rep.detach = yes(r, "detachFromReferenceLine");
        o.repeats.push_back(rep);
    }
    // 1.4's single <outline> under the object, or <outlines> (1.5 on).
    if (const auto* single = e->FirstChildElement("outline")) {
        o.outlines.push_back(readOutline(single));
    }
    if (const auto* all = e->FirstChildElement("outlines")) {
        for (auto* ol = all->FirstChildElement("outline"); ol != nullptr;
             ol = ol->NextSiblingElement("outline")) {
            o.outlines.push_back(readOutline(ol));
        }
    }
    return o;
}

Signal readSignal(const tinyxml2::XMLElement* e) {
    Signal g;
    g.id = text(e, "id");
    g.name = text(e, "name");
    g.type = text(e, "type");
    g.subtype = text(e, "subtype");
    g.country = text(e, "country");
    g.orientation = text(e, "orientation");
    g.text = text(e, "text");
    g.s = attr(e, "s");
    g.t = attr(e, "t");
    g.zOffset = attr(e, "zOffset");
    g.hOffset = attr(e, "hOffset");
    g.pitch = attr(e, "pitch");
    g.roll = attr(e, "roll");
    g.height = attr(e, "height");
    g.width = attr(e, "width");
    g.length = attr(e, "length");
    g.value = attrOr(e, "value", std::numeric_limits<double>::quiet_NaN());
    g.dynamic = yes(e, "dynamic");
    for (auto* v = e->FirstChildElement("validity"); v != nullptr;
         v = v->NextSiblingElement("validity")) {
        g.validity.emplace_back(v->IntAttribute("fromLane"), v->IntAttribute("toLane"));
    }
    return g;
}

std::optional<int> optionalInt(const tinyxml2::XMLElement* e, const char* name) {
    int value = 0;
    if (e->QueryIntAttribute(name, &value) == tinyxml2::XML_SUCCESS) {
        return value;
    }
    return std::nullopt;
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
        if (const auto* objects = r->FirstChildElement("objects")) {
            for (auto* o = objects->FirstChildElement("object"); o != nullptr;
                 o = o->NextSiblingElement("object")) {
                road.objects.push_back(readObject(o));
                for (const Outline& ol : road.objects.back().outlines) {
                    if (ol.corners.size() < 2) {
                        return Unexpected{where + "object " + road.objects.back().id +
                                          ": an outline needs two corners or more"};
                    }
                }
            }
        }
        if (const auto* signals = r->FirstChildElement("signals")) {
            for (auto* g = signals->FirstChildElement("signal"); g != nullptr;
                 g = g->NextSiblingElement("signal")) {
                road.signals.push_back(readSignal(g));
            }
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
        for (auto* c = j->FirstChildElement("controller"); c != nullptr;
             c = c->NextSiblingElement("controller")) {
            junction.controllers.push_back(
                {text(c, "id"), text(c, "type"), optionalInt(c, "sequence")});
        }
        n.junctions.push_back(std::move(junction));
    }
    // Controllers (§14.6): the signals that switch together.
    for (auto* c = top->FirstChildElement("controller"); c != nullptr;
         c = c->NextSiblingElement("controller")) {
        Controller controller;
        controller.id = text(c, "id");
        controller.name = text(c, "name");
        controller.sequence = optionalInt(c, "sequence");
        for (auto* k = c->FirstChildElement("control"); k != nullptr;
             k = k->NextSiblingElement("control")) {
            controller.signals.push_back(text(k, "signalId"));
        }
        n.controllers.push_back(std::move(controller));
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

namespace {

/// z-y'-x'': heading, then pitch, then roll (w, x, y, z).
std::array<double, 4> eulerQuat(double yaw, double pitch, double roll) {
    const std::array<double, 4> qz{std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
    const std::array<double, 4> qy{std::cos(pitch / 2.0), 0.0, std::sin(pitch / 2.0), 0.0};
    const std::array<double, 4> qx{std::cos(roll / 2.0), std::sin(roll / 2.0), 0.0, 0.0};
    return multiply(multiply(qz, qy), qx);
}

double lerp(double a, double b, double f) {
    return a + (b - a) * f;
}

/// A repeat value at fraction f: from the record, or the object's own where it gives none.
double repeated(double start, double end, double own, double f) {
    if (std::isnan(start)) {
        return own;
    }
    return lerp(start, std::isnan(end) ? start : end, f);
}

}  // namespace

std::array<double, 3> surfacePoint(const Road& road, double s, double t, double up) {
    auto p = pointAt(road, s, t);
    p[2] += up;
    return p;
}

Pose roadPose(const Road& road, double s, double t, double zOffset, double hdg, double pitch,
              double roll) {
    Pose pose;
    pose.position = surfacePoint(road, s, t, zOffset);
    pose.orientation = eulerQuat(referenceAt(road, s).hdg + hdg, pitch, roll);
    return pose;
}

std::vector<Placement> placements(const Road& road, const Object& object) {
    std::vector<Placement> out;
    if (object.repeats.empty()) {
        out.push_back({roadPose(road, object.s, object.t, object.zOffset, object.hdg, object.pitch,
                                object.roll),
                       object.length, object.width, object.height, object.radius});
        return out;
    }
    for (const Repeat& r : object.repeats) {
        if (!(r.distance > 0.0)) {
            continue;  // continuous: drawn and collided along its path
        }
        const int n = static_cast<int>(std::floor(r.length / r.distance + 1e-9));
        const auto start = surfacePoint(road, r.s, r.tStart, r.zOffsetStart);
        const auto end = surfacePoint(road, r.s + r.length, r.tEnd, r.zOffsetEnd);
        for (int i = 0; i <= n; ++i) {
            const double along = i * r.distance;
            const double f = r.length > 0.0 ? along / r.length : 0.0;
            Placement p;
            if (r.detach) {
                p.pose.position = {lerp(start[0], end[0], f), lerp(start[1], end[1], f),
                                   lerp(start[2], end[2], f)};
                const double line = std::atan2(end[1] - start[1], end[0] - start[0]);
                p.pose.orientation = eulerQuat(line + object.hdg, object.pitch, object.roll);
            } else {
                p.pose = roadPose(road, r.s + along, lerp(r.tStart, r.tEnd, f),
                                  lerp(r.zOffsetStart, r.zOffsetEnd, f), object.hdg, object.pitch,
                                  object.roll);
            }
            p.height = lerp(r.heightStart, r.heightEnd, f);
            p.width = repeated(r.widthStart, r.widthEnd, object.width, f);
            p.length = repeated(r.lengthStart, r.lengthEnd, object.length, f);
            p.radius = repeated(r.radiusStart, r.radiusEnd, object.radius, f);
            out.push_back(p);
        }
    }
    return out;
}

Pose signalPose(const Road& road, const Signal& signal) {
    // Orientation "+": for traffic along s, so the face looks back against s; hOffset turns it
    // counter-clockwise (§14.1). "-" and "none": hOffset from the reference line's direction.
    const double facing = (signal.orientation == "+" ? std::numbers::pi : 0.0) + signal.hOffset;
    return roadPose(road, signal.s, signal.t, signal.zOffset, facing, signal.pitch, signal.roll);
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
            if (!road.objects.empty() || !road.signals.empty()) {
                buildObjects(road);
            }
        }
        if (auto status = linkLanes(); !status) {
            return Unexpected{status.error()};
        }
        buildSignals();
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

    // ---- Objects and signals -------------------------------------------------------------

    using P3 = std::array<double, 3>;
    using Meshes = std::map<std::uint32_t, TriMesh>;

    /// What an object of `type` is drawn with, and whether it is paint on the road.
    std::uint32_t objectMaterial(const std::string& type) {
        if (type == "pole" || type == "streetLamp" || type == "gantry") {
            return material("object_pole_steel", {0.55F, 0.56F, 0.58F, 1.0F});
        }
        if (type == "barrier" || type == "railing") {
            return material("object_barrier_steel", {0.72F, 0.72F, 0.74F, 1.0F});
        }
        if (type == "soundBarrier" || type == "obstacle" || type == "trafficIsland") {
            return material("object_concrete", {0.62F, 0.61F, 0.58F, 1.0F});
        }
        if (type == "building") {
            return material("object_building_concrete", {0.74F, 0.7F, 0.64F, 1.0F});
        }
        if (type == "tree" || type == "vegetation") {
            return material("object_tree_foliage", {0.2F, 0.42F, 0.18F, 1.0F});
        }
        if (type == "crosswalk" || type == "roadMark" || type == "parkingSpace") {
            return material("mark_white", {0.92F, 0.92F, 0.88F, 1.0F});
        }
        if (type == "patch" || type == "roadSurface") {
            return material("asphalt", {0.2F, 0.2F, 0.21F, 1.0F});
        }
        if (type == "car" || type == "van" || type == "bus" || type == "trailer" ||
            type == "truck" || type == "motorbike" || type == "bike" || type == "train" ||
            type == "tram" || type == "pedestrian") {
            return material("object_vehicle_paint", {0.55F, 0.14F, 0.12F, 1.0F});
        }
        return material("object_other", {0.6F, 0.56F, 0.5F, 1.0F});
    }

    /// A closed convex solid from its bottom and top rings (same count, in order round it): its
    /// sides facing out from the centre, its top up and its bottom down.
    static void prism(TriMesh& mesh, const std::vector<P3>& bottom, const std::vector<P3>& top,
                      bool caps = true) {
        const std::size_t n = bottom.size();
        P3 c{};
        for (const auto* ring : {&bottom, &top}) {
            for (const P3& p : *ring) {
                for (std::size_t k = 0; k < 3; ++k) {
                    c[k] += p[k] / static_cast<double>(2 * n);
                }
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            const P3 mid{0.25 * (bottom[i][0] + bottom[j][0] + top[i][0] + top[j][0]),
                         0.25 * (bottom[i][1] + bottom[j][1] + top[i][1] + top[j][1]),
                         0.25 * (bottom[i][2] + bottom[j][2] + top[i][2] + top[j][2])};
            strip(mesh, {bottom[i], top[i]}, {bottom[j], top[j]},
                  {mid[0] - c[0], mid[1] - c[1], mid[2] - c[2]});
        }
        if (caps) {
            fan(mesh, top, {0.0, 0.0, 1.0});
            fan(mesh, bottom, {0.0, 0.0, -1.0});
        }
    }

    /// A convex polygon as a triangle fan, its normal its own, turned to `facing`.
    static void fan(TriMesh& mesh, const std::vector<P3>& ring, const P3& facing) {
        for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
            triangle(mesh, ring[0], ring[i], ring[i + 1], facing);
        }
    }

    static void triangle(TriMesh& mesh, const P3& a, const P3& b, const P3& c, const P3& facing) {
        std::array<double, 3> n{(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
                                (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
                                (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])};
        const double len = std::hypot(n[0], n[1], n[2]);
        if (!(len > 1e-12)) {
            return;
        }
        const double sign =
            n[0] * facing[0] + n[1] * facing[1] + n[2] * facing[2] < 0.0 ? -1.0 : 1.0;
        const auto base = static_cast<std::uint32_t>(mesh.positions.size() / 3);
        for (const P3* p : {&a, &b, &c}) {
            for (const double v : *p) {
                mesh.positions.push_back(static_cast<float>(v));
            }
            for (const double v : n) {
                mesh.normals.push_back(static_cast<float>(sign * v / len + 0.0));
            }
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2});
    }

    /// The ring of a box or cylinder footprint about a pose's origin, `up` along its z.
    static std::vector<P3> ring(const Pose& pose, double length, double width, double radius,
                                double up) {
        std::vector<P3> out;
        const auto at = [&](double u, double v) {
            const P3 r = rotate(pose.orientation, {u, v, up});
            return P3{pose.position[0] + r[0], pose.position[1] + r[1], pose.position[2] + r[2]};
        };
        if (radius > 0.0) {
            for (int i = 0; i < kCylinderSides; ++i) {
                const double a = 2.0 * std::numbers::pi * i / kCylinderSides;
                out.push_back(at(radius * std::cos(a), radius * std::sin(a)));
            }
        } else {
            const double l = (length > 0.0 ? length : kThinObject) / 2.0;
            const double w = (width > 0.0 ? width : kThinObject) / 2.0;
            out = {at(-l, -w), at(l, -w), at(l, w), at(-l, w)};
        }
        return out;
    }

    void collideBox(const Pose& pose, double length, double width, double height) {
        Pose centre = compose(pose, Pose{{0.0, 0.0, height / 2.0}, {1.0, 0.0, 0.0, 0.0}});
        m_.world.addCollider(kWorldBody, ShapeType::Box,
                             {(length > 0.0 ? length : kThinObject) / 2.0,
                              (width > 0.0 ? width : kThinObject) / 2.0, height / 2.0},
                             centre, o_.objectFriction);
    }

    void collideCylinder(const Pose& pose, double radius, double height) {
        Pose centre = compose(pose, Pose{{0.0, 0.0, height / 2.0}, {1.0, 0.0, 0.0, 0.0}});
        m_.world.addCollider(kWorldBody, ShapeType::Cylinder, {radius, height / 2.0, 0.0}, centre,
                             o_.objectFriction);
    }

    /// A convex hull of world points, posed at their centroid.
    void collideHull(const std::vector<P3>& points) {
        P3 c{};
        for (const P3& p : points) {
            for (std::size_t k = 0; k < 3; ++k) {
                c[k] += p[k] / static_cast<double>(points.size());
            }
        }
        TriMesh cloud;
        for (const P3& p : points) {
            for (std::size_t k = 0; k < 3; ++k) {
                cloud.positions.push_back(static_cast<float>(p[k] - c[k]));
            }
        }
        auto hull = convexHullOf(cloud);
        if (!hull) {
            return;  // flat or degenerate
        }
        Pose pose;
        pose.position = c;
        m_.world.addHullCollider(kWorldBody, std::move(*hull), pose, o_.objectFriction);
    }

    /// A flat footprint painted on the road: a crosswalk's stripes across its length, a parking
    /// space's border, else filled.
    void paint(TriMesh& mesh, const std::string& type, const Pose& pose, double length,
               double width, double radius) {
        const double lift = o_.surfaceHeight + kMarkLift;
        const auto at = [&](double u, double v) {
            const P3 r = rotate(pose.orientation, {u, v, 0.0});
            return P3{pose.position[0] + r[0], pose.position[1] + r[1],
                      pose.position[2] + r[2] + lift};
        };
        const double l = std::max(length, kThinObject) / 2.0;
        const double w = std::max(width, kThinObject) / 2.0;
        if (radius <= 0.0 && type == "crosswalk") {
            // Stripes kStripe wide every 2·kStripe along the longer side, running along the
            // shorter one (the walking direction is across the stripes' length).
            const bool alongU = l >= w;
            const double span = alongU ? l : w;
            for (double a = -span; a + kStripe <= span + 1e-9; a += 2.0 * kStripe) {
                if (alongU) {
                    fan(mesh, {at(a, -w), at(a + kStripe, -w), at(a + kStripe, w), at(a, w)},
                        {0.0, 0.0, 1.0});
                } else {
                    fan(mesh, {at(-l, a), at(l, a), at(l, a + kStripe), at(-l, a + kStripe)},
                        {0.0, 0.0, 1.0});
                }
            }
            return;
        }
        if (radius <= 0.0 && type == "parkingSpace") {
            const double m = kOutlineMark;
            fan(mesh, {at(-l, -w), at(l, -w), at(l, -w + m), at(-l, -w + m)}, {0.0, 0.0, 1.0});
            fan(mesh, {at(-l, w - m), at(l, w - m), at(l, w), at(-l, w)}, {0.0, 0.0, 1.0});
            fan(mesh, {at(-l, -w), at(-l + m, -w), at(-l + m, w), at(-l, w)}, {0.0, 0.0, 1.0});
            return;  // open at its far end (+u), where cars drive in
        }
        Pose lifted = pose;
        lifted.position[2] += lift;
        fan(mesh, ring(lifted, length, width, radius, 0.0), {0.0, 0.0, 1.0});
    }

    /// One placed object: a box or cylinder standing from its origin, colliding; flat ones
    /// painted.
    void place(Meshes& meshes, const Object& object, const Placement& p) {
        const std::uint32_t mat = objectMaterial(object.type);
        if (p.height <= kFlatObject) {
            paint(meshes[mat], object.type, p.pose, p.length, p.width, p.radius);
            return;
        }
        prism(meshes[mat], ring(p.pose, p.length, p.width, p.radius, 0.0),
              ring(p.pose, p.length, p.width, p.radius, p.height));
        if (p.radius > 0.0) {
            collideCylinder(p.pose, p.radius, p.height);
        } else {
            collideBox(p.pose, p.length, p.width, p.height);
        }
    }

    /// A repeat with distance 0: one object along its path (a guard rail, a wall), its cross
    /// section width × height, drawn every `step` and colliding as a hull every colliderStep.
    void continuous(Meshes& meshes, const Road& road, const Object& object, const Repeat& r) {
        const std::uint32_t mat = objectMaterial(object.type);
        const auto start = surfacePoint(road, r.s, r.tStart, r.zOffsetStart);
        const auto end = surfacePoint(road, r.s + r.length, r.tEnd, r.zOffsetEnd);
        // The cross section at fraction f: its four corners, bottom left/right, top right/left.
        const auto section = [&](double f) {
            P3 c;
            double hdg = 0.0;
            if (r.detach) {
                c = {lerp(start[0], end[0], f), lerp(start[1], end[1], f),
                     lerp(start[2], end[2], f)};
                hdg = std::atan2(end[1] - start[1], end[0] - start[0]);
            } else {
                const double s = r.s + f * r.length;
                c = surfacePoint(road, s, lerp(r.tStart, r.tEnd, f),
                                 lerp(r.zOffsetStart, r.zOffsetEnd, f));
                hdg = referenceAt(road, s).hdg;
            }
            double w = repeated(r.widthStart, r.widthEnd, object.width, f);
            const double radius = repeated(r.radiusStart, r.radiusEnd, object.radius, f);
            if (!(w > 0.0)) {
                w = radius > 0.0 ? 2.0 * radius : kThinObject;
            }
            const double h = lerp(r.heightStart, r.heightEnd, f);
            const double nx = -std::sin(hdg) * w / 2.0;
            const double ny = std::cos(hdg) * w / 2.0;
            return std::array<P3, 4>{P3{c[0] + nx, c[1] + ny, c[2]}, P3{c[0] - nx, c[1] - ny, c[2]},
                                     P3{c[0] - nx, c[1] - ny, c[2] + h},
                                     P3{c[0] + nx, c[1] + ny, c[2] + h}};
        };
        const bool flat = std::max(r.heightStart, r.heightEnd) <= kFlatObject;
        const int n = std::max(1, static_cast<int>(std::ceil(r.length / o_.step)));
        std::vector<std::array<P3, 4>> rows;
        for (int i = 0; i <= n; ++i) {
            rows.push_back(section(static_cast<double>(i) / n));
        }
        if (flat) {
            const double lift = o_.surfaceHeight + kMarkLift;
            std::vector<P3> a, b;
            for (auto row : rows) {
                row[0][2] += lift;
                row[1][2] += lift;
                a.push_back(row[0]);
                b.push_back(row[1]);
            }
            strip(meshes[mat], a, b);
            return;
        }
        // Four faces along the path, each facing out of the section's centre, and end caps.
        for (std::size_t k = 0; k < 4; ++k) {
            std::vector<P3> a, b;
            for (const auto& row : rows) {
                a.push_back(row[k]);
                b.push_back(row[(k + 1) % 4]);
            }
            const auto& row = rows[rows.size() / 2];
            const auto& opposite = row[(k + 2) % 4];
            const P3 out{0.5 * (row[k][0] + row[(k + 1) % 4][0]) - opposite[0],
                         0.5 * (row[k][1] + row[(k + 1) % 4][1]) - opposite[1],
                         0.5 * (row[k][2] + row[(k + 1) % 4][2]) - opposite[2]};
            strip(meshes[mat], a, b, out);
        }
        for (const bool last : {false, true}) {
            const auto& row = last ? rows.back() : rows.front();
            const auto& other = last ? rows[rows.size() - 2] : rows[1];
            const P3 out{row[0][0] - other[0][0], row[0][1] - other[0][1], row[0][2] - other[0][2]};
            fan(meshes[mat], {row[0], row[1], row[2], row[3]}, out);
        }
        const int m = std::max(1, static_cast<int>(std::ceil(r.length / o_.colliderStep)));
        for (int i = 0; i < m; ++i) {
            const auto a = section(static_cast<double>(i) / m);
            const auto b = section(static_cast<double>(i + 1) / m);
            collideHull({a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]});
        }
    }

    /// Ear clipping of a simple polygon in the xy plane: triangles as index triples.
    static std::vector<std::array<std::size_t, 3>> triangulate(const std::vector<P3>& poly) {
        std::vector<std::size_t> v(poly.size());
        for (std::size_t i = 0; i < v.size(); ++i) {
            v[i] = i;
        }
        double area = 0.0;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const P3& a = poly[i];
            const P3& b = poly[(i + 1) % poly.size()];
            area += a[0] * b[1] - b[0] * a[1];
        }
        const double turn = area >= 0.0 ? 1.0 : -1.0;  // counter-clockwise: +
        const auto cross = [&](std::size_t a, std::size_t b, std::size_t c) {
            return turn * ((poly[b][0] - poly[a][0]) * (poly[c][1] - poly[a][1]) -
                           (poly[b][1] - poly[a][1]) * (poly[c][0] - poly[a][0]));
        };
        std::vector<std::array<std::size_t, 3>> out;
        std::size_t guard = 0;
        while (v.size() > 3 && guard++ < 10 * poly.size() * poly.size()) {
            bool clipped = false;
            for (std::size_t i = 0; i < v.size(); ++i) {
                const std::size_t a = v[(i + v.size() - 1) % v.size()];
                const std::size_t b = v[i];
                const std::size_t c = v[(i + 1) % v.size()];
                if (cross(a, b, c) <= 1e-12) {
                    continue;  // reflex
                }
                bool inside = false;
                for (const std::size_t p : v) {
                    if (p != a && p != b && p != c && cross(a, b, p) >= 0.0 &&
                        cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0) {
                        inside = true;
                        break;
                    }
                }
                if (inside) {
                    continue;
                }
                out.push_back({a, b, c});
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
                break;
            }
            if (!clipped) {
                break;  // not simple: what is left is dropped
            }
        }
        if (v.size() == 3) {
            out.push_back({v[0], v[1], v[2]});
        }
        return out;
    }

    static bool convex(const std::vector<P3>& poly) {
        int sign = 0;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const P3& a = poly[i];
            const P3& b = poly[(i + 1) % poly.size()];
            const P3& c = poly[(i + 2) % poly.size()];
            const double z = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
            if (std::abs(z) < 1e-12) {
                continue;
            }
            const int s = z > 0.0 ? 1 : -1;
            if (sign != 0 && s != sign) {
                return false;
            }
            sign = s;
        }
        return true;
    }

    /// An outline: a closed one is a prism (its footprint at the corners, each corner's
    /// height up), colliding as one hull when convex, else a hull per triangle of its
    /// footprint; flat it is painted. An open one is a wall along its corners, kThinObject
    /// thick.
    void outline(Meshes& meshes, const Road& road, const Object& object, const Outline& ol) {
        const std::uint32_t mat = objectMaterial(object.type);
        const Pose origin = roadPose(road, object.s, object.t, object.zOffset, object.hdg,
                                     object.pitch, object.roll);
        std::vector<P3> bottom, top;
        for (const Corner& c : ol.corners) {
            P3 b;
            P3 up{0.0, 0.0, c.height};
            if (ol.local) {
                const P3 r = rotate(origin.orientation, {c.a, c.b, c.z});
                b = {origin.position[0] + r[0], origin.position[1] + r[1],
                     origin.position[2] + r[2]};
                up = rotate(origin.orientation, up);
            } else {
                b = surfacePoint(road, c.a, c.b, c.z);
            }
            bottom.push_back(b);
            top.push_back({b[0] + up[0], b[1] + up[1], b[2] + up[2]});
        }
        bool closed = ol.closed;
        const auto same = [](const P3& a, const P3& b) {
            return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-9;
        };
        if (bottom.size() > 2 && same(bottom.front(), bottom.back())) {
            bottom.pop_back();
            top.pop_back();
            closed = true;
        }
        const bool flat = std::ranges::all_of(
            ol.corners, [](const Corner& c) { return std::abs(c.height) <= kFlatObject; });
        if (!closed || bottom.size() < 3) {
            // A linear feature: per segment a wall, or a painted line.
            const double half = kThinObject / 2.0;
            for (std::size_t i = 0; i + 1 < bottom.size(); ++i) {
                const P3& a = bottom[i];
                const P3& b = bottom[i + 1];
                const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
                if (len < 1e-9) {
                    continue;
                }
                const double nx = -(b[1] - a[1]) / len * half;
                const double ny = (b[0] - a[0]) / len * half;
                const auto off = [&](const P3& p, double k, double up) {
                    return P3{p[0] + k * nx, p[1] + k * ny, p[2] + up};
                };
                if (flat) {
                    const double lift = o_.surfaceHeight + kMarkLift;
                    fan(meshes[mat],
                        {off(a, 1, lift), off(a, -1, lift), off(b, -1, lift), off(b, 1, lift)},
                        {0.0, 0.0, 1.0});
                    continue;
                }
                const double ha = top[i][2] - a[2];
                const double hb = top[i + 1][2] - b[2];
                const std::vector<P3> lo{off(a, 1, 0), off(a, -1, 0), off(b, -1, 0), off(b, 1, 0)};
                const std::vector<P3> hi{off(a, 1, ha), off(a, -1, ha), off(b, -1, hb),
                                         off(b, 1, hb)};
                prism(meshes[mat], lo, hi);
                std::vector<P3> all = lo;
                all.insert(all.end(), hi.begin(), hi.end());
                collideHull(all);
            }
            return;
        }
        const auto tris = triangulate(bottom);
        if (flat) {
            const double lift = o_.surfaceHeight + kMarkLift;
            if (object.type == "parkingSpace") {
                for (std::size_t i = 0; i < bottom.size(); ++i) {
                    std::vector<P3> edge{bottom[i], bottom[(i + 1) % bottom.size()]};
                    for (P3& p : edge) {
                        p[2] += lift;
                    }
                    const double len = std::hypot(edge[1][0] - edge[0][0], edge[1][1] - edge[0][1]);
                    if (len < 1e-9) {
                        continue;
                    }
                    const double nx = -(edge[1][1] - edge[0][1]) / len * kOutlineMark / 2.0;
                    const double ny = (edge[1][0] - edge[0][0]) / len * kOutlineMark / 2.0;
                    fan(meshes[mat],
                        {{edge[0][0] + nx, edge[0][1] + ny, edge[0][2]},
                         {edge[0][0] - nx, edge[0][1] - ny, edge[0][2]},
                         {edge[1][0] - nx, edge[1][1] - ny, edge[1][2]},
                         {edge[1][0] + nx, edge[1][1] + ny, edge[1][2]}},
                        {0.0, 0.0, 1.0});
                }
                return;
            }
            for (const auto& t : tris) {
                triangle(meshes[mat], {bottom[t[0]][0], bottom[t[0]][1], bottom[t[0]][2] + lift},
                         {bottom[t[1]][0], bottom[t[1]][1], bottom[t[1]][2] + lift},
                         {bottom[t[2]][0], bottom[t[2]][1], bottom[t[2]][2] + lift},
                         {0.0, 0.0, 1.0});
            }
            return;
        }
        prism(meshes[mat], bottom, top, false);
        for (const auto& t : tris) {
            triangle(meshes[mat], top[t[0]], top[t[1]], top[t[2]], {0.0, 0.0, 1.0});
            triangle(meshes[mat], bottom[t[0]], bottom[t[1]], bottom[t[2]], {0.0, 0.0, -1.0});
        }
        if (convex(bottom)) {
            std::vector<P3> all = bottom;
            all.insert(all.end(), top.begin(), top.end());
            collideHull(all);
        } else {
            for (const auto& t : tris) {
                collideHull(
                    {bottom[t[0]], bottom[t[1]], bottom[t[2]], top[t[0]], top[t[1]], top[t[2]]});
            }
        }
    }

    /// A signal: its board (a sign, or a traffic light's dark housing, whose lamps the traffic
    /// module draws and lights at its RoadSignalAPI_1 prim) on a pole from the road surface,
    /// both colliding.
    void signal(Meshes& meshes, const Road& road, const Signal& g) {
        const Pose board = signalPose(road, g);
        const double w = g.width > 0.0 ? g.width : (g.dynamic ? kHeadWidth : kSignSize);
        const double h = g.height > 0.0 ? g.height : (g.dynamic ? kHeadHeight : kSignSize);
        const double d = g.length > 0.0 ? g.length : (g.dynamic ? kHeadDepth : kSignDepth);
        const std::uint32_t face =
            g.dynamic ? material("signal_head_housing", {0.07F, 0.07F, 0.08F, 1.0F})
                      : material("sign_board", {0.9F, 0.9F, 0.88F, 1.0F});
        prism(meshes[face], ring(board, d, w, 0.0, 0.0), ring(board, d, w, 0.0, h));
        collideBox(board, d, w, h);
        // The pole: behind the board, from the road surface to the board's top.
        const P3 back = rotate(board.orientation, {-(d / 2.0 + kSignPoleRadius), 0.0, 0.0});
        const auto ground = surfacePoint(road, g.s, g.t, 0.0);
        const double top = board.position[2] + h - ground[2];
        if (g.zOffset > kFlatObject && top > kFlatObject) {
            Pose pole;
            pole.position = {board.position[0] + back[0], board.position[1] + back[1], ground[2]};
            const std::uint32_t steel = material("sign_pole_steel", {0.55F, 0.56F, 0.58F, 1.0F});
            prism(meshes[steel], ring(pole, 0.0, 0.0, kSignPoleRadius, 0.0),
                  ring(pole, 0.0, 0.0, kSignPoleRadius, top));
            collideCylinder(pole, kSignPoleRadius, top);
        }
    }

    /// A RoadSignalAPI_1 prim per dynamic signal, at the middle of its face, x out of it.
    void buildSignals() {
        std::uint32_t scope = kNoIndex;
        std::map<std::string, int> taken;
        for (const Road& road : n_.roads) {
            for (const Signal& g : road.signals) {
                if (!g.dynamic) {
                    continue;
                }
                if (scope == kNoIndex) {
                    ScenePrim prim;
                    prim.name = "signals";
                    prim.path = "/" + o_.root + "/signals";
                    prim.parent = 0;
                    prim.typeName = "Xform";
                    m_.scene.prims.push_back(prim);
                    scope = static_cast<std::uint32_t>(m_.scene.prims.size() - 1);
                }
                std::string name = "signal_";
                for (const char c : g.id) {
                    name += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
                }
                if (const int n = taken[name]++; n > 0) {
                    name += "_" + std::to_string(n);
                }
                // The face's middle: the board's origin is its bottom centre.
                const double h = g.height > 0.0 ? g.height : kHeadHeight;
                const double d = g.length > 0.0 ? g.length : kHeadDepth;
                Pose face = signalPose(road, g);
                const auto r = rotate(face.orientation, {d / 2.0, 0.0, h / 2.0});
                for (std::size_t k = 0; k < 3; ++k) {
                    face.position.at(k) += r.at(k);
                }
                ScenePrim prim;
                prim.name = name;
                prim.path = m_.scene.prims[scope].path + "/" + name;
                prim.parent = scope;
                prim.typeName = "Xform";
                prim.offset = face;
                const PrimId id = m_.world.addPrim(prim.path, name);
                m_.world.prims.poseInBody.at(id.index) = face;
                m_.scene.prims.push_back(std::move(prim));

                SchemaInstance s;
                s.schema = "RoadSignalAPI_1";
                s.prim = id;
                s.tokens = {{"signal:id", g.id},
                            {"signal:orientation", g.orientation == "+" || g.orientation == "-"
                                                       ? g.orientation
                                                       : std::string("none")},
                            {"signal:type", g.type},
                            {"signal:subtype", g.subtype}};
                s.reals = {{"signal:s", g.s}, {"signal:t", g.t}, {"signal:height", h}};
                s.integers = {{"signal:road", road.id}};
                if (!g.validity.empty()) {
                    // One range: the first validity's (several are rare, TODO(verify) whether
                    // any published network uses more than one per signal).
                    s.integers["signal:fromLane"] = g.validity.front().first;
                    s.integers["signal:toLane"] = g.validity.front().second;
                }
                // Its controller, and that controller's place in its junction.
                for (const Controller& c : n_.controllers) {
                    if (std::ranges::count(c.signals, g.id) == 0) {
                        continue;
                    }
                    s.tokens["signal:controller"] = c.id;
                    std::optional<int> sequence = c.sequence;
                    for (const Junction& j : n_.junctions) {
                        const auto it = std::ranges::find_if(
                            j.controllers,
                            [&c](const JunctionController& jc) { return jc.id == c.id; });
                        if (it != j.controllers.end()) {
                            s.integers["signal:junction"] = j.id;
                            sequence = it->sequence ? it->sequence : sequence;
                            break;
                        }
                    }
                    if (sequence) {
                        s.integers["signal:sequence"] = *sequence;
                    }
                    break;
                }
                m_.world.schemaInstances.push_back(std::move(s));
            }
        }
    }

    void buildObjects(const Road& road) {
        Meshes meshes;
        for (const Object& object : road.objects) {
            for (const Placement& p : placements(road, object)) {
                if (object.outlines.empty() || !object.repeats.empty()) {
                    place(meshes, object, p);
                }
            }
            for (const Repeat& r : object.repeats) {
                if (!(r.distance > 0.0) && r.length > 0.0) {
                    continuous(meshes, road, object, r);
                }
            }
            if (object.repeats.empty()) {
                for (const Outline& ol : object.outlines) {
                    outline(meshes, road, object, ol);
                }
            }
        }
        for (const Signal& g : road.signals) {
            signal(meshes, road, g);
        }
        const std::string roadName = "road_" + std::to_string(road.id);
        for (auto& [mat, mesh] : meshes) {
            addMesh(roadName + "_objects_" + m_.scene.materials[mat].name, std::move(mesh), mat);
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
