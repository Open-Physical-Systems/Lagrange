// The air box round a multicopter (LbmBox): still air stays still, a disc gets the thrust it is
// given, the air through it is what momentum theory says, and the GPU step agrees with the CPU's.
#include "lagrange/aerodynamics/LbmBox.hpp"
#include "lagrange/gpu/Device.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::aerodynamics::LbmBox;
using lagrange::aerodynamics::LbmBoxConfig;
namespace gpu = lagrange::gpu;

constexpr double kRho = 1.225;
constexpr double kSettle = 0.8;  // s, for the starting vortex to leave the box

LbmBoxConfig smallBox() {
    LbmBoxConfig config;
    config.nx = 32;
    config.ny = 32;
    config.nz = 48;
    config.cellSize = 0.04;
    config.maxSpeed = 12.0;
    return config;
}

LbmBox::Rotor oneRotor(const LbmBoxConfig& config, double thrust) {
    LbmBox::Rotor r;
    r.centre = {0.5 * static_cast<double>(config.nx) * config.cellSize,
                0.5 * static_cast<double>(config.ny) * config.cellSize,
                0.75 * static_cast<double>(config.nz) * config.cellSize};
    r.radius = 0.2;
    r.thrust = thrust;
    return r;
}

TEST(LbmBox, StillAirStaysStill) {
    LbmBox air(smallBox());
    air.step(50);
    for (std::size_t c = 0; c < air.cells(); c += 97) {
        const auto u = air.velocity(c);
        // Float rounding of the weights only.
        EXPECT_NEAR(u[0], 0.0, 1e-5);
        EXPECT_NEAR(u[1], 0.0, 1e-5);
        EXPECT_NEAR(u[2], 0.0, 1e-5);
    }
}

// The disc's hat weights summed over the cells are what its push is divided by, so the cells get
// exactly the thrust; the sum is about the disc's area in cells (one cell thick).
TEST(LbmBox, TheDiscWeightsCoverItsArea) {
    const LbmBoxConfig config = smallBox();
    LbmBox air(config);
    const LbmBox::Rotor rotor = oneRotor(config, 5.0);
    const double area = std::numbers::pi * std::pow(rotor.radius / config.cellSize, 2);
    EXPECT_NEAR(air.discWeightSum(rotor), area, 0.1 * area);
}

// Momentum theory (Leishman, Principles of Helicopter Aerodynamics, 2nd ed., eq. 2.14): a hovering
// disc's induced velocity is sqrt(T / (2 ρ A)), and the far wake moves at twice it, carrying the
// thrust as momentum. Coarse cells blur the disc's edge, so the speed at it is checked loosely;
// the momentum through a plane below (no pressure term: the jet is at ambient there) closely.
TEST(LbmBox, TheAirThroughADiscIsMomentumTheorys) {
    const LbmBoxConfig config = smallBox();
    LbmBox air(config);
    const LbmBox::Rotor rotor = oneRotor(config, 4.0);
    std::vector<LbmBox::Rotor> rotors{rotor};
    air.setRotors(rotors);
    const auto steps = static_cast<std::size_t>(std::lround(kSettle / air.timeStep()));
    air.step(steps);
    const double area = std::numbers::pi * rotor.radius * rotor.radius;
    const double induced = std::sqrt(rotor.thrust / (2.0 * kRho * area));
    const auto atDisc = air.velocityAt(rotor.centre);
    EXPECT_LT(atDisc[2], 0.0);
    EXPECT_NEAR(-atDisc[2], induced, 0.3 * induced);
    // The downward momentum through a plane 0.4 m below carries the thrust.
    const double below = rotor.centre[2] - 0.4;
    const auto z = static_cast<std::size_t>(below / config.cellSize);
    double flux = 0.0;
    for (std::size_t y = 0; y < config.ny; ++y) {
        for (std::size_t x = 0; x < config.nx; ++x) {
            const double w = air.velocity(air.index(x, y, z))[2];
            if (w < 0.0) {
                flux += kRho * w * w * config.cellSize * config.cellSize;
            }
        }
    }
    std::printf("induced %.3f (theory %.3f), momentum flux %.3f N of %.3f\n", -atDisc[2], induced,
                flux, rotor.thrust);
    EXPECT_NEAR(flux, rotor.thrust, 0.1 * rotor.thrust);
}

// The GPU step (half precision, float) and the CPU reference make the same jet.
TEST(LbmBox, TheGpuAgreesWithTheCpu) {
    std::string error;
    auto device = gpu::Device::create(gpu::DeviceDesc{}, error);
    if (!device) {
        GTEST_SKIP() << error;
    }
    const LbmBoxConfig config = smallBox();
    const auto run = [&config](const gpu::Device* on) {
        LbmBox air(config);
        if (on != nullptr) {
            air.useGpu(*on);
            EXPECT_TRUE(air.onGpu());
        }
        std::vector<LbmBox::Rotor> rotors{oneRotor(config, 4.0)};
        air.setRotors(rotors);
        air.step(400);
        air.syncVelocity();
        std::vector<double> out;
        for (const double depth : {0.0, 0.2, 0.5}) {
            auto at = rotors[0].centre;
            at[2] -= depth;
            out.push_back(air.velocityAt(at)[2]);
        }
        return out;
    };
    const std::vector<double> cpu = run(nullptr);
    const std::vector<double> gpu = run(device.get());
    for (std::size_t k = 0; k < cpu.size(); ++k) {
        EXPECT_NEAR(gpu[k], cpu[k], 0.05 * std::abs(cpu[k]) + 0.02) << k;
    }
}

}  // namespace
