#pragma once

// The lumped thermal network of docs/design/domains/thermal.md §2, free of the bus and the
// scheduler so that the validation tests can drive it directly. SI units throughout.
#include <cstddef>
#include <span>
#include <vector>

namespace lagrange::thermal {

/// Stefan–Boltzmann constant, W/(m²·K⁴): CODATA 2018, exact since the 2019 SI redefinition.
inline constexpr double kStefanBoltzmann = 5.670374419e-8;

struct Node {
    double capacity{0.0};    // J/K
    double convection{0.0};  // W/K: heat transfer coefficient × convection area
    double radiation{0.0};   // W/K⁴: emissivity × σ × radiative area
    double solarArea{0.0};   // m²: absorptivity × area facing the sun
};

/// A conductance between two nodes.
struct Link {
    std::size_t a{0}, b{0};
    double conductance{0.0};  // W/K
};

/// What surrounds the nodes.
struct Surroundings {
    double air{0.0};         // K
    double sky{0.0};         // K: radiative sink
    double irradiance{0.0};  // W/m², on the nodes' sun-facing areas
};

/// Nodes and links, advanced with backward Euler on the network linearised around the current
/// temperatures (radiation: T⁴ ≈ T₀⁴ + 4·T₀³·(T − T₀)): unconditionally stable, so a 100 ms
/// step is safe for small nodes, and energy balances exactly for the linearised flows.
class LumpedNetwork {
public:
    LumpedNetwork() = default;
    LumpedNetwork(std::vector<Node> nodes, std::vector<Link> links);

    [[nodiscard]] std::size_t size() const { return nodes_.size(); }

    /// Advances `temperatures` (K) by `dt` with the heat sources `sources` (W per node) held over
    /// the step. `heatOut` receives each node's heat flow to the air and the sky at the end of
    /// the step (W; positive leaving; links and the sun not included). No allocation.
    void step(std::span<double> temperatures, std::span<const double> sources,
              const Surroundings& around, double dt, std::span<double> heatOut);

private:
    std::vector<Node> nodes_;
    std::vector<Link> links_;
    std::vector<double> matrix_;  // n × n, row major
    std::vector<double> rhs_;
};

/// How much of its rating a component keeps at temperature `t`: 1 below `start`, falling
/// linearly to 0 at `max` (thermal.md §2). With no limits (max ≤ start) it is always 1.
[[nodiscard]] double derating(double t, double start, double max);

}  // namespace lagrange::thermal
