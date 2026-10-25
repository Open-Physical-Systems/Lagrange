// The lumped thermal network (thermal.md §2).
#include "lagrange/thermal/Lumped.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lagrange::thermal {

LumpedNetwork::LumpedNetwork(std::vector<Node> nodes, std::vector<Link> links)
    : nodes_(std::move(nodes))
    , links_(std::move(links))
    , matrix_(nodes_.size() * nodes_.size(), 0.0)
    , rhs_(nodes_.size(), 0.0) {}

void LumpedNetwork::step(std::span<double> temperatures, std::span<const double> sources,
                         const Surroundings& around, double dt, std::span<double> heatOut) {
    const std::size_t n = nodes_.size();
    const double sky4 = around.sky * around.sky * around.sky * around.sky;
    std::fill(matrix_.begin(), matrix_.end(), 0.0);
    // Per node:
    //   (C/dt + hA + 4·εσA·T₀³ + Σ G)·T − Σ G·T_j
    //       = C/dt·T₀ + Q + hA·T_air + εσA·(3·T₀⁴ + T_sky⁴) + αA·E
    for (std::size_t i = 0; i < n; ++i) {
        const Node& node = nodes_[i];
        const double t0 = temperatures[i];
        const double t03 = t0 * t0 * t0;
        matrix_[i * n + i] = node.capacity / dt + node.convection + 4.0 * node.radiation * t03;
        rhs_[i] = node.capacity / dt * t0 + sources[i] + node.convection * around.air +
                  node.radiation * (3.0 * t03 * t0 + sky4) + node.solarArea * around.irradiance;
    }
    for (const Link& link : links_) {
        matrix_[link.a * n + link.a] += link.conductance;
        matrix_[link.b * n + link.b] += link.conductance;
        matrix_[link.a * n + link.b] -= link.conductance;
        matrix_[link.b * n + link.a] -= link.conductance;
    }
    // The matrix is symmetric and diagonally dominant (capacities and conductances are
    // positive): Gaussian elimination needs no pivoting.
    for (std::size_t k = 0; k < n; ++k) {
        const double pivot = matrix_[k * n + k];
        for (std::size_t i = k + 1; i < n; ++i) {
            const double factor = matrix_[i * n + k] / pivot;
            if (factor == 0.0) {
                continue;
            }
            for (std::size_t j = k; j < n; ++j) {
                matrix_[i * n + j] -= factor * matrix_[k * n + j];
            }
            rhs_[i] -= factor * rhs_[k];
        }
    }
    std::vector<double>& t = rhs_;  // back substitution in place
    for (std::size_t k = n; k-- > 0;) {
        double sum = t[k];
        for (std::size_t j = k + 1; j < n; ++j) {
            sum -= matrix_[k * n + j] * t[j];
        }
        t[k] = sum / matrix_[k * n + k];
    }
    for (std::size_t i = 0; i < n; ++i) {
        const Node& node = nodes_[i];
        const double t0 = temperatures[i];
        const double t03 = t0 * t0 * t0;
        // The flows the step used: radiation linearised around the start temperature.
        heatOut[i] = node.convection * (t[i] - around.air) +
                     node.radiation * (t03 * t0 + 4.0 * t03 * (t[i] - t0) - sky4);
        temperatures[i] = t[i];
    }
}

double derating(double t, double start, double max) {
    if (max <= start) {
        return 1.0;
    }
    return std::clamp((max - t) / (max - start), 0.0, 1.0);
}

}  // namespace lagrange::thermal
