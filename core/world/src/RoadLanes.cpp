// A world's road lanes (RoadLanes.hpp).
#include "lagrange/world/RoadLanes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace lagrange {

std::pair<std::array<double, 3>, double> RoadLane::at(double s) const {
    if (points.size() < 2) {
        return {points.empty() ? std::array<double, 3>{} : points.front(), 0.0};
    }
    s = std::clamp(s, 0.0, length());
    const auto it = std::upper_bound(distance.begin(), distance.end(), s);
    const std::size_t i =
        std::min(static_cast<std::size_t>(std::max<std::ptrdiff_t>(it - distance.begin() - 1, 0)),
                 points.size() - 2);
    const double span = distance[i + 1] - distance[i];
    const double t = span > 0.0 ? (s - distance[i]) / span : 0.0;
    const auto& a = points[i];
    const auto& b = points[i + 1];
    return {{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])},
            std::atan2(b[1] - a[1], b[0] - a[0])};
}

std::pair<double, double> RoadLane::project(const std::array<double, 3>& p) const {
    double bestS = 0.0;
    double best = std::numeric_limits<double>::infinity();
    double side = 0.0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[i + 1];
        const double dx = b[0] - a[0];
        const double dy = b[1] - a[1];
        const double len2 = dx * dx + dy * dy;
        const double t =
            len2 > 0.0 ? std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2, 0.0, 1.0)
                       : 0.0;
        const double d = std::hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1]);
        if (d < best) {
            best = d;
            bestS = distance[i] + t * (distance[i + 1] - distance[i]);
            side = dx * (p[1] - a[1]) - dy * (p[0] - a[0]) >= 0.0 ? 1.0 : -1.0;
        }
    }
    return {bestS, side * best};
}

std::optional<std::pair<std::size_t, double>> RoadLanes::find(int road, int lane, double s) const {
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        const RoadLane& l = lanes[i];
        if (l.road != road || l.lane != lane || s < l.sStart - 1e-9 || s > l.sEnd + 1e-9) {
            continue;
        }
        // The centre line's length can differ from the reference line's (curves): scale.
        const double span = std::max(l.sEnd - l.sStart, 1e-9);
        const double fraction = std::clamp((s - l.sStart) / span, 0.0, 1.0);
        const double along = l.lane < 0 ? fraction : 1.0 - fraction;
        return std::pair{i, along * l.length()};
    }
    return std::nullopt;
}

std::optional<std::pair<std::size_t, double>> RoadLanes::neighbour(std::size_t index, double along,
                                                                   int step) const {
    const RoadLane& from = lanes.at(index);
    // Left of travel: towards the reference line for right lanes (id −2 → −1, along s), and
    // for left lanes, which run against s, towards it as well (id 2 → 1).
    const int target = from.lane < 0 ? from.lane + step : from.lane - step;
    if (target == 0 || (from.lane < 0) != (target < 0)) {
        return std::nullopt;  // across the centre line: the other direction
    }
    const double span = std::max(from.sEnd - from.sStart, 1e-9);
    const double fraction = from.length() > 0.0 ? along / from.length() : 0.0;
    const double s = from.lane < 0 ? from.sStart + fraction * span : from.sEnd - fraction * span;
    return find(from.road, target, s);
}

RoadLanes roadLanesOf(const WorldView& world) {
    RoadLanes g;
    const std::vector<const SchemaInstance*> instances = world.instancesOf("RoadLaneAPI_1");
    std::map<std::uint32_t, std::size_t> byPrim;
    for (const SchemaInstance* s : instances) {
        const std::string_view type = s->token("lane:type").value_or("driving");
        if (type != "driving" && type != "sidewalk") {
            continue;
        }
        RoadLane lane;
        lane.name = std::string(world.signalInstanceName(s->prim));
        const std::span<const double> c = s->realArray("lane:centreLine");
        for (std::size_t i = 0; i + 2 < c.size(); i += 3) {
            lane.points.push_back({c[i], c[i + 1], c[i + 2]});
        }
        if (lane.points.size() < 2) {
            continue;
        }
        lane.distance.push_back(0.0);
        for (std::size_t i = 1; i < lane.points.size(); ++i) {
            lane.distance.push_back(lane.distance.back() +
                                    std::hypot(lane.points[i][0] - lane.points[i - 1][0],
                                               lane.points[i][1] - lane.points[i - 1][1]));
        }
        const std::span<const double> w = s->realArray("lane:width");
        lane.width = w.empty() ? 3.5 : w[w.size() / 2];
        lane.speedLimit = s->real("lane:speedLimit").value_or(0.0);
        lane.sidewalk = type == "sidewalk";
        lane.junction = s->integer("lane:junction").value_or(0) != 0;
        lane.road = static_cast<int>(s->integer("lane:road").value_or(0));
        lane.lane = static_cast<int>(s->integer("lane:lane").value_or(0));
        lane.sStart = s->real("lane:sStart").value_or(0.0);
        lane.sEnd = s->real("lane:sEnd").value_or(0.0);
        byPrim[s->prim.index] = g.lanes.size();
        g.lanes.push_back(std::move(lane));
    }
    for (const SchemaInstance* s : instances) {
        const auto self = byPrim.find(s->prim.index);
        if (self == byPrim.end()) {
            continue;
        }
        for (const PrimId next : s->rel("lane:successors")) {
            if (const auto it = byPrim.find(next.index); it != byPrim.end()) {
                g.lanes[self->second].successors.push_back(it->second);
                g.lanes[it->second].predecessors.push_back(self->second);
            }
        }
    }
    return g;
}

}  // namespace lagrange
