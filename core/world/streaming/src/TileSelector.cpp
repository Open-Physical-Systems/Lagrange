// Tile selection for streamed worlds (TileSelector.hpp, huge-worlds.md §2).
#include "lagrange/world/streaming/TileSelector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace lagrange::streaming {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
// A footprint counts as covered when the finer tiles cover all of it but this fraction
// (rounding of the bounds).
constexpr double kCoverTolerance = 1e-9;
// Below this distance the error test uses it instead: a tile under the robot is judged as if
// one metre away, so the threshold stays finite.
constexpr double kMinDistanceM = 1.0;

double footprint(const TileInfo& t) {
    return std::max(0.0, t.boundsMax[0] - t.boundsMin[0]) *
           std::max(0.0, t.boundsMax[1] - t.boundsMin[1]);
}

double overlap(const TileInfo& a, const TileInfo& b) {
    const double x =
        std::min(a.boundsMax[0], b.boundsMax[0]) - std::max(a.boundsMin[0], b.boundsMin[0]);
    const double y =
        std::min(a.boundsMax[1], b.boundsMax[1]) - std::max(a.boundsMin[1], b.boundsMin[1]);
    return x > 0.0 && y > 0.0 ? x * y : 0.0;
}

}  // namespace

double horizontalDistance(const std::array<double, 3>& point, const std::array<double, 3>& lo,
                          const std::array<double, 3>& hi) {
    const double dx = std::max({lo[0] - point[0], 0.0, point[0] - hi[0]});
    const double dy = std::max({lo[1] - point[1], 0.0, point[1] - hi[1]});
    return std::hypot(dx, dy);
}

TileSelector::TileSelector(std::vector<TileInfo> tiles, StreamingSettings settings)
    : tiles_(std::move(tiles))
    , settings_(settings)
    , state_(tiles_.size(), State::Unloaded)
    , farSince_(tiles_.size(), -1.0) {
    if (settings_.unloadRadius < settings_.loadRadius) {
        settings_.unloadRadius = settings_.loadRadius;
    }
    // Children: the tiles of the next finer level present that overlap a tile's footprint.
    // Levels need not be consecutive; tiles are bucketed by level and searched per bucket.
    std::map<std::int32_t, std::vector<std::uint32_t>> byLevel;
    for (std::uint32_t i = 0; i < tiles_.size(); ++i) {
        byLevel[tiles_[i].level].push_back(i);
    }
    children_.assign(tiles_.size(), {});
    parents_.assign(tiles_.size(), {});
    for (auto it = byLevel.begin(); it != byLevel.end(); ++it) {
        const auto next = std::next(it);
        if (next == byLevel.end()) {
            break;
        }
        // A uniform grid over the finer level, cell = its largest tile, for overlap queries.
        const std::vector<std::uint32_t>& fine = next->second;
        double cell = 0.0;
        for (const std::uint32_t f : fine) {
            cell = std::max({cell, tiles_[f].boundsMax[0] - tiles_[f].boundsMin[0],
                             tiles_[f].boundsMax[1] - tiles_[f].boundsMin[1]});
        }
        if (!(cell > 0.0)) {
            continue;
        }
        std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::uint32_t>> grid;
        const auto key = [cell](double x, double y) {
            return std::pair{static_cast<std::int64_t>(std::floor(x / cell)),
                             static_cast<std::int64_t>(std::floor(y / cell))};
        };
        for (const std::uint32_t f : fine) {
            grid[key(tiles_[f].boundsMin[0], tiles_[f].boundsMin[1])].push_back(f);
        }
        for (const std::uint32_t c : it->second) {
            const TileInfo& coarse = tiles_[c];
            const auto [x0, y0] = key(coarse.boundsMin[0], coarse.boundsMin[1]);
            const auto [x1, y1] = key(coarse.boundsMax[0], coarse.boundsMax[1]);
            for (std::int64_t x = x0 - 1; x <= x1; ++x) {
                for (std::int64_t y = y0 - 1; y <= y1; ++y) {
                    const auto found = grid.find({x, y});
                    if (found == grid.end()) {
                        continue;
                    }
                    for (const std::uint32_t f : found->second) {
                        if (overlap(coarse, tiles_[f]) > 0.0) {
                            children_[c].push_back(f);
                            parents_[f].push_back(c);
                        }
                    }
                }
            }
            std::sort(children_[c].begin(), children_[c].end());
        }
    }
}

StreamingPlan TileSelector::update(std::span<const Interest> interests, double timeS) {
    const std::size_t n = tiles_.size();
    std::vector<double> nearest(n, kInfinity);  // to any interest that counts
    std::vector<double> nearestRobot(n, kInfinity);
    for (std::size_t i = 0; i < n; ++i) {
        for (const Interest& interest : interests) {
            if (interest.kind == Interest::Kind::Camera && settings_.deterministic) {
                continue;
            }
            const double d =
                horizontalDistance(interest.position, tiles_[i].boundsMin, tiles_[i].boundsMax);
            nearest[i] = std::min(nearest[i], d);
            if (interest.kind == Interest::Kind::Robot) {
                nearestRobot[i] = std::min(nearestRobot[i], d);
            }
        }
    }

    // Wanted tiles: refine from the roots (tiles with no coarser tile over them) down.
    std::vector<bool> wanted(n, false);
    const auto candidate = [&](std::uint32_t t) { return nearest[t] <= settings_.loadRadius; };
    const auto needsFiner = [&](std::uint32_t t) {
        const TileInfo& tile = tiles_[t];
        if (children_[t].empty()) {
            return false;
        }
        // A robot near a tile without colliders needs the finer ones that have them.
        if (!tile.hasCollision && nearestRobot[t] <= settings_.loadRadius) {
            return true;
        }
        // Hysteresis on the level of detail: while finer tiles are loaded, keep them until the
        // distance grows by unloadRadius / loadRadius.
        bool finerLoaded = false;
        for (const std::uint32_t c : children_[t]) {
            finerLoaded = finerLoaded || state_[c] == State::Loaded;
        }
        const double stretch = finerLoaded && settings_.loadRadius > 0.0
                                   ? settings_.unloadRadius / settings_.loadRadius
                                   : 1.0;
        return tile.geometricError >
               settings_.errorPerMetre * std::max(nearest[t], kMinDistanceM) * stretch;
    };
    std::vector<std::uint32_t> stack;
    for (std::uint32_t i = 0; i < n; ++i) {
        if (parents_[i].empty() && candidate(i)) {
            stack.push_back(i);
        }
    }
    std::vector<bool> visited(n, false);
    while (!stack.empty()) {
        const std::uint32_t t = stack.back();
        stack.pop_back();
        if (visited[t]) {
            continue;
        }
        visited[t] = true;
        if (!needsFiner(t)) {
            wanted[t] = true;
            continue;
        }
        double covered = 0.0;
        for (const std::uint32_t c : children_[t]) {
            if (candidate(c)) {
                covered += overlap(tiles_[t], tiles_[c]);
                stack.push_back(c);
            }
        }
        // Finer tiles that reach only part of it (at the edge of the radius): keep this one too.
        if (covered < footprint(tiles_[t]) * (1.0 - kCoverTolerance)) {
            wanted[t] = true;
        }
    }
    // Collision tiles a robot can reach are wanted however they were reached.
    for (std::uint32_t i = 0; i < n; ++i) {
        if (tiles_[i].hasCollision && nearestRobot[i] <= settings_.loadRadius) {
            wanted[i] = true;
        }
    }

    StreamingPlan plan;
    // Unload: beyond the unload radius (after the delay), or replaced by loaded finer tiles.
    for (std::uint32_t i = 0; i < n; ++i) {
        if (state_[i] != State::Loaded || wanted[i]) {
            farSince_[i] = -1.0;
            continue;
        }
        bool drop = false;
        if (nearest[i] > settings_.unloadRadius) {
            if (settings_.deterministic) {
                drop = true;
            } else {
                if (farSince_[i] < 0.0) {
                    farSince_[i] = timeS;
                }
                drop = timeS - farSince_[i] >= settings_.unloadDelayS;
            }
        } else if (nearest[i] <= settings_.loadRadius) {
            // Inside the radius and not wanted: finer tiles replace it once they are all loaded,
            // or a coarser tile does once it is loaded (it was too fine for the distance).
            bool replacements = false;
            bool allLoaded = true;
            for (const std::uint32_t c : children_[i]) {
                if (wanted[c]) {
                    replacements = true;
                    allLoaded = allLoaded && state_[c] == State::Loaded;
                }
            }
            if (!replacements) {
                for (const std::uint32_t p : parents_[i]) {
                    if (wanted[p]) {
                        replacements = true;
                        allLoaded = allLoaded && state_[p] == State::Loaded;
                    }
                }
            }
            drop = replacements && allLoaded;
        }
        if (drop) {
            plan.unload.push_back(i);
        }
    }
    for (std::uint32_t i = 0; i < n; ++i) {
        if (wanted[i] && state_[i] == State::Unloaded) {
            plan.load.push_back(i);
        }
    }
    std::stable_sort(plan.load.begin(), plan.load.end(),
                     [&](std::uint32_t a, std::uint32_t b) { return nearest[a] < nearest[b]; });

    // Memory budget: what stays plus what loads must fit. Farthest first, except the
    // collision tiles a robot needs.
    if (settings_.memoryBudget > 0) {
        std::uint64_t total = 0;
        for (std::uint32_t i = 0; i < n; ++i) {
            const bool leaving =
                std::find(plan.unload.begin(), plan.unload.end(), i) != plan.unload.end();
            if (state_[i] != State::Unloaded && !leaving) {
                total += tiles_[i].bytes;
            }
        }
        // The robots' ground first, whatever it costs; then the rest, nearest first, as far as
        // the budget goes.
        const auto needed = [&](std::uint32_t t) {
            return tiles_[t].hasCollision && nearestRobot[t] <= settings_.loadRadius;
        };
        std::vector<bool> keep(n, false);
        for (const std::uint32_t t : plan.load) {
            if (needed(t)) {
                keep[t] = true;
                total += tiles_[t].bytes;
            }
        }
        for (const std::uint32_t t : plan.load) {
            if (!keep[t] && total + tiles_[t].bytes <= settings_.memoryBudget) {
                keep[t] = true;
                total += tiles_[t].bytes;
            }
        }
        std::erase_if(plan.load, [&keep](std::uint32_t t) { return !keep[t]; });
    }

    for (const std::uint32_t t : plan.load) {
        state_[t] = State::Loading;
    }
    return plan;
}

void TileSelector::loaded(std::uint32_t id) {
    state_.at(id) = State::Loaded;
    farSince_.at(id) = -1.0;
}

void TileSelector::unloaded(std::uint32_t id) {
    state_.at(id) = State::Unloaded;
    farSince_.at(id) = -1.0;
}

bool TileSelector::physicsReady(const std::array<double, 3>& position, double margin) const {
    for (std::size_t i = 0; i < tiles_.size(); ++i) {
        const TileInfo& t = tiles_[i];
        if (t.hasCollision && state_[i] != State::Loaded &&
            horizontalDistance(position, t.boundsMin, t.boundsMax) <= margin) {
            return false;
        }
    }
    return true;
}

std::uint64_t TileSelector::loadedBytes() const {
    std::uint64_t total = 0;
    for (std::size_t i = 0; i < tiles_.size(); ++i) {
        if (state_[i] == State::Loaded) {
            total += tiles_[i].bytes;
        }
    }
    return total;
}

std::size_t TileSelector::loadingCount() const {
    return static_cast<std::size_t>(std::count(state_.begin(), state_.end(), State::Loading));
}

}  // namespace lagrange::streaming
