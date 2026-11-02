#pragma once

// A robot in a wind tunnel: its collision shapes voxelized in an LbmTunnel, air past them until the
// wake is established, and what a mechanical engineer looks at afterwards: drag and lift per link,
// the flow as a ParaView file and as a world of streamlines, a slice and surface pressure.

#include "lagrange/aerodynamics/Lbm.hpp"
#include "lagrange/module/Expected.hpp"
#include "lagrange/world/World.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace lagrange::gpu {
class Device;
}

namespace lagrange::aerodynamics {

struct WindTunnelOptions {
    double speed{10.0};            ///< m/s
    double yawDegrees{0.0};        ///< The robot turned about z, with the wind along +x.
    std::size_t maxCells{800000};  ///< The grid is as fine as this allows.
    double passes{3.0};            ///< How many times the air crosses the tunnel before averaging.
    double meanPasses{2.0};        ///< And how many it is averaged over: a wake sheds slowly.
    double airDensity{1.2};        ///< kg/m³
    double viscosity{1.5e-5};      ///< m²/s
    /// A rolling road under the robot's lowest point, as for a car; else free air around it.
    bool ground{false};
    /// m from the lowest point to the road. Never under two cells, or no air would pass beneath
    /// it; that is also what a negative value gives. A model's z = 0 says nothing: a loaded robot
    /// may stand raised.
    double clearance{-1.0};
    /// Pictures of the wake moving: this many PNG files in frames/ of the output, while it
    /// averages. 0: none. Speed in a slice along the body over pressure in a slice from above;
    /// frames/legend.json holds their scales. `frameSpin`: the air's spin in the first slice alone.
    std::size_t frames{0};
    bool frameSpin{false};
    /// Where to step it: null for the CPU reference.
    const gpu::Device* device{nullptr};
};

struct LinkLoad {
    std::string name;
    std::array<double, 3> force{};   ///< N, x along the wind, z up
    std::array<double, 3> torque{};  ///< N·m, about the body's frontal centre
};

struct WindTunnelResult {
    std::vector<LinkLoad> links;
    std::array<double, 3> force{};
    double frontalArea{0.0};  ///< m²
    double dragCoefficient{0.0};
    /// How much the drag coefficient moved between the averaging's windows (standard deviation):
    /// the wake sheds, so one number hides this.
    double dragSpread{0.0};
    double liftCoefficient{0.0};
    double sideCoefficient{0.0};
    double dynamicPressure{0.0};  ///< Pa
    double reynolds{0.0};         ///< On the body's largest size
    double cellSize{0.0};         ///< m
    double clearance{0.0};        ///< m to the road as run; 0 with no road
    std::array<std::size_t, 3> grid{};
    std::size_t steps{0};
    double seconds{0.0};   ///< Air time simulated
    double blockage{0.0};  ///< Frontal area over the tunnel's section
};

/// Runs it. `progress` gets a share 0..1 now and then. Refuses a robot with no collision shapes.
/// The collision shapes as they stand at rest, turned by `yawDegrees`, for another solver: one
/// ASCII STL per link in `directory` and surface.json with their names and bounds. Empty when all
/// went.
[[nodiscard]] std::string exportTunnelSurface(const World& world, double yawDegrees,
                                              const std::string& directory);

/// One grid of a grid independence study.
struct GridStudyRow {
    std::size_t cells{0};
    double dragCoefficient{0.0};
    double dragSpread{0.0};
    double liftCoefficient{0.0};
};

/// What a study says of its finest grid.
struct GridStudyVerdict {
    double dragChange{0.0};  ///< of Cd between the last two grids, as a share of the last
    double dragMoves{0.0};   ///< the last grid's spread, as a share of its Cd
    double liftChange{0.0};  ///< of Cl between the last two grids, as a share of the last Cd
    bool settled{false};     ///< all three within kGridSettledShare; says nothing of being right
};

/// A common rule of thumb for "grid independent" drag, not a standard: within 5 %.
inline constexpr double kGridSettledShare = 0.05;

/// Rows from coarse to fine; fewer than two is never settled.
[[nodiscard]] GridStudyVerdict gridStudyVerdict(const std::vector<GridStudyRow>& rows);

[[nodiscard]] Expected<WindTunnelResult, std::string> runWindTunnel(
    const World& world, const WindTunnelOptions& options, const std::string& outputDirectory,
    const std::function<void(double)>& progress = {});

}  // namespace lagrange::aerodynamics
