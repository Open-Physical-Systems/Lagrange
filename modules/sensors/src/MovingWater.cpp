#include "lagrange/sensors/MovingWater.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lagrange::sensors {

bool MovingWater::prepare() {
    const auto n = static_cast<std::size_t>(cells);
    lowest = 0.0;
    highest = 0.0;
    if (cells < 2 || !(cell > 0.0) || heights.size() != n * n || !std::isfinite(corner[0]) ||
        !std::isfinite(corner[1])) {
        cells = 0;
        return false;
    }
    for (const double h : heights) {
        if (std::isfinite(h)) {
            lowest = std::min(lowest, h);
            highest = std::max(highest, h);
        }
    }
    return true;
}

double MovingWater::heightAt(double x, double y) const {
    if (cells < 2 || x < lo[0] || x > hi[0] || y < lo[1] || y > hi[1]) {
        return 0.0;
    }
    const double fx = (x - corner[0]) / cell - 0.5;
    const double fy = (y - corner[1]) / cell - 0.5;
    const auto last = static_cast<double>(cells - 1);
    if (!(fx >= 0.0 && fy >= 0.0 && fx <= last && fy <= last)) {
        return 0.0;
    }
    const auto n = static_cast<std::size_t>(cells);
    const auto col = std::min(static_cast<std::size_t>(fx), n - 2);
    const auto row = std::min(static_cast<std::size_t>(fy), n - 2);
    const double tx = fx - static_cast<double>(col);
    const double ty = fy - static_cast<double>(row);
    const auto at = [&](std::size_t c, std::size_t r) {
        const double v = heights[r * n + c];
        return std::isfinite(v) ? v : 0.0;
    };
    return (1.0 - ty) * ((1.0 - tx) * at(col, row) + tx * at(col + 1, row)) +
           ty * ((1.0 - tx) * at(col, row + 1) + tx * at(col + 1, row + 1));
}

std::array<double, 3> MovingWater::normalAt(double x, double y) const {
    if (cells < 2) {
        return {0.0, 0.0, 1.0};
    }
    const double e = cell / 2.0;
    const double sx = (heightAt(x + e, y) - heightAt(x - e, y)) / (2.0 * e);
    const double sy = (heightAt(x, y + e) - heightAt(x, y - e)) / (2.0 * e);
    const double norm = std::sqrt(sx * sx + sy * sy + 1.0);
    return {-sx / norm, -sy / norm, 1.0 / norm};
}

double rayOntoWater(const MovingWater& water, const std::array<double, 3>& origin,
                    const std::array<double, 3>& direction, double hit, bool hitIsWater,
                    bool* onWater) {
    if (onWater != nullptr) {
        *onWater = hitIsWater;
    }
    const double dz = direction[2];
    if (water.cells < 2 || !(dz < 0.0) || origin[2] <= water.level + water.lowest) {
        return hit;
    }
    // The stretch of the ray between the crests' and the troughs' heights.
    const double from = std::max((water.level + water.highest - origin[2]) / dz, 0.0);
    double to = (water.level + water.lowest - origin[2]) / dz;
    if (!hitIsWater) {
        to = std::min(to, hit);
    }
    if (!(from < to)) {
        return hit;
    }
    const auto gap = [&](double t) {
        const double x = origin[0] + direction[0] * t;
        const double y = origin[1] + direction[1] * t;
        return origin[2] + dz * t - water.level - water.heightAt(x, y);
    };
    // Half a cell sideways per step (at least a thousandth of the stretch: near vertical rays
    // cross it in one step), at most kMostSteps.
    constexpr int kMostSteps = 4096;
    const double sideways = std::hypot(direction[0], direction[1]);
    const double stretch = to - from;
    double step = sideways > 0.0 ? water.cell / 2.0 / sideways : stretch;
    step = std::clamp(step, stretch / kMostSteps, stretch);
    double a = from;
    if (gap(a) <= 0.0) {
        return hit;  // starts under the surface there: seen from below, left as it is
    }
    for (int i = 0; i < kMostSteps && a < to; ++i) {
        const double b = std::min(a + step, to);
        const double gb = gap(b);
        if (gb <= 0.0) {
            // Bisect [a, b]: 48 halvings take a metre's step under a nanometre.
            double lo = a;
            double hi = b;
            for (int k = 0; k < 48; ++k) {
                const double m = 0.5 * (lo + hi);
                (gap(m) > 0.0 ? lo : hi) = m;
            }
            const double x = origin[0] + direction[0] * hi;
            const double y = origin[1] + direction[1] * hi;
            if (x < water.lo[0] || x > water.hi[0] || y < water.lo[1] || y > water.hi[1]) {
                return hit;  // the level beside the water (its side was hit): no surface there
            }
            if (onWater != nullptr) {
                *onWater = true;
            }
            return hi;
        }
        a = b;
    }
    return hit;  // no crossing before `hit`: what it hit stands in front
}

}  // namespace lagrange::sensors
