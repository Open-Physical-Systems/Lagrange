// Active regions (RegionPlanner.hpp, huge-worlds.md §3).
#include "lagrange/world/regions/RegionPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace lagrange::regions {

namespace {

double distance(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/// Union-find over robot indices.
class Groups {
public:
    explicit Groups(std::size_t n) : parent_(n) { std::iota(parent_.begin(), parent_.end(), 0U); }
    std::uint32_t find(std::uint32_t i) {
        while (parent_[i] != i) {
            parent_[i] = parent_[parent_[i]];
            i = parent_[i];
        }
        return i;
    }
    void join(std::uint32_t a, std::uint32_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);  // the lowest index leads: deterministic
        }
    }

private:
    std::vector<std::uint32_t> parent_;
};

}  // namespace

RegionPlanner::RegionPlanner(RegionSettings settings) : settings_(settings) {}

RegionAssignment RegionPlanner::plan(std::span<const std::array<double, 3>> positions,
                                     const RegionAssignment& previous) const {
    const std::size_t n = positions.size();
    const double merge = 2.0 * settings_.radius;
    const double split = merge * std::max(settings_.splitFactor, 1.0);
    Groups groups(n);
    // Robots that shared a region stay linked within the split distance.
    for (const std::vector<std::uint32_t>& region : previous) {
        for (std::size_t a = 0; a < region.size(); ++a) {
            for (std::size_t b = a + 1; b < region.size(); ++b) {
                const std::uint32_t i = region[a];
                const std::uint32_t j = region[b];
                if (i < n && j < n && distance(positions[i], positions[j]) <= split) {
                    groups.join(i, j);
                }
            }
        }
    }
    for (std::uint32_t i = 0; i < n; ++i) {
        for (std::uint32_t j = i + 1; j < n; ++j) {
            if (distance(positions[i], positions[j]) <= merge) {
                groups.join(i, j);
            }
        }
    }
    RegionAssignment out;
    std::vector<std::int64_t> slot(n, -1);
    for (std::uint32_t i = 0; i < n; ++i) {
        const std::uint32_t leader = groups.find(i);
        if (slot[leader] < 0) {
            slot[leader] = static_cast<std::int64_t>(out.size());
            out.emplace_back();
        }
        out[static_cast<std::size_t>(slot[leader])].push_back(i);
    }
    return out;
}

}  // namespace lagrange::regions
