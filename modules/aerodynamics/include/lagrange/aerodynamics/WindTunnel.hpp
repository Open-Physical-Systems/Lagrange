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

namespace lagrange::aerodynamics {

struct WindTunnelOptions {
    double speed{10.0};            ///< m/s
    double yawDegrees{0.0};        ///< The robot turned about z, with the wind along +x.
    std::size_t maxCells{800000};  ///< The grid is as fine as this allows.
    double passes{3.0};            ///< How many times the air crosses the tunnel before averaging.
    double airDensity{1.2};        ///< kg/m³
    double viscosity{1.5e-5};      ///< m²/s
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
    double liftCoefficient{0.0};
    double sideCoefficient{0.0};
    double dynamicPressure{0.0};  ///< Pa
    double reynolds{0.0};         ///< On the body's largest size
    double cellSize{0.0};         ///< m
    std::array<std::size_t, 3> grid{};
    std::size_t steps{0};
    double seconds{0.0};   ///< Air time simulated
    double blockage{0.0};  ///< Frontal area over the tunnel's section
};

/// Runs it. `progress` gets a share 0..1 now and then. Refuses a robot with no collision shapes.
[[nodiscard]] Expected<WindTunnelResult, std::string> runWindTunnel(
    const World& world, const WindTunnelOptions& options, const std::string& outputDirectory,
    const std::function<void(double)>& progress = {});

}  // namespace lagrange::aerodynamics
