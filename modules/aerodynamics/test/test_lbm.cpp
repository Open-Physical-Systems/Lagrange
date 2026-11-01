// The wind tunnel against what is known: a decaying shear wave has an exact solution, a sphere's
// drag has a published correlation (Schiller and Naumann, 1933), and a bluff body's a measured one.
#include "lagrange/aerodynamics/Lbm.hpp"
#include "lagrange/aerodynamics/WindTunnel.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/gpu/Device.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::aerodynamics {
namespace {

// A shear wave in a periodic box decays as exp(-ν k² t): the lattice's viscosity is what τ says.
TEST(Lbm, AShearWaveDecaysAtTheViscosityAsked) {
    LbmConfig config;
    config.nx = 4;
    config.ny = 64;
    config.nz = 4;
    config.periodic = true;
    config.smagorinsky = 0.0;
    config.cellSize = 1.0;
    config.windSpeed = 1.0;
    config.latticeSpeed = 1.0;  // one m/s is one cell per step: lattice units throughout
    const double nu = 0.05;
    config.kinematicViscosity = nu;
    LbmTunnel tunnel(config);
    const double k = 2.0 * std::numbers::pi / 64.0;
    const double u0 = 0.02;
    for (std::size_t z = 0; z < 4; ++z) {
        for (std::size_t y = 0; y < 64; ++y) {
            for (std::size_t x = 0; x < 4; ++x) {
                tunnel.setLatticeFlow(
                    tunnel.index(x, y, z), 1.0,
                    {u0 * std::sin(k * (static_cast<double>(y) + 0.5)), 0.0, 0.0});
            }
        }
    }
    const std::size_t steps = 800;
    tunnel.step(steps);
    const double expected = u0 * std::exp(-nu * k * k * static_cast<double>(steps));
    double measured = 0.0;
    for (std::size_t y = 0; y < 64; ++y) {
        measured = std::max(measured, tunnel.latticeVelocity(tunnel.index(0, y, 0))[0]);
    }
    EXPECT_NEAR(measured / expected, 1.0, 0.01);
}

/// The drag coefficient of a sphere of `cells` cells across at Reynolds number `re`.
double sphereDrag(double re, double cells, std::size_t length, std::size_t side) {
    LbmConfig config;
    config.nx = length;
    config.ny = side;
    config.nz = side;
    config.cellSize = 0.01;
    config.windSpeed = 1.0;
    config.latticeSpeed = 0.05;
    config.smagorinsky = 0.0;
    const double diameter = cells * config.cellSize;
    config.kinematicViscosity = config.windSpeed * diameter / re;
    LbmTunnel tunnel(config);
    const double cx = static_cast<double>(length) / 3.0;
    const double cy = static_cast<double>(side) / 2.0;
    for (std::size_t z = 0; z < side; ++z) {
        for (std::size_t y = 0; y < side; ++y) {
            for (std::size_t x = 0; x < length; ++x) {
                const double dx = static_cast<double>(x) + 0.5 - cx;
                const double dy = static_cast<double>(y) + 0.5 - cy;
                const double dz = static_cast<double>(z) + 0.5 - cy;
                if (std::sqrt(dx * dx + dy * dy + dz * dz) <= cells / 2.0) {
                    tunnel.setSolid(x, y, z, 0);
                }
            }
        }
    }
    // Long enough for the wake to form: three passes of the tunnel.
    const auto settle =
        static_cast<std::size_t>(3.0 * static_cast<double>(length) / config.latticeSpeed);
    tunnel.step(settle);
    tunnel.step(500);
    EXPECT_FALSE(tunnel.diverged());
    const double area = std::numbers::pi * diameter * diameter / 4.0;
    return tunnel.loads()[0].force[0] /
           (0.5 * config.airDensity * config.windSpeed * config.windSpeed * area);
}

// Schiller and Naumann: Cd = 24/Re (1 + 0.15 Re^0.687), good for Re below about 800. A sphere of
// 10 cells in a box 40 across has solid blockage, which raises the drag; the check allows it.
TEST(Lbm, ASpheresDragFollowsThePublishedCurve) {
    for (const double re : {20.0, 50.0}) {
        const double expected = 24.0 / re * (1.0 + 0.15 * std::pow(re, 0.687));
        const double measured = sphereDrag(re, 10.0, 90, 40);
        std::printf("Re %.0f: Cd %.3f, the curve %.3f\n", re, measured, expected);
        EXPECT_GT(measured, expected * 0.95) << "Re " << re;
        EXPECT_LT(measured, expected * 1.30) << "Re " << re;
    }
}

// Slow, by hand: --gtest_also_run_disabled_tests. A finer grid with less blockage comes closer.
TEST(Lbm, DISABLED_ASpheresDragComesCloserWithAFinerGrid) {
    for (const double re : {20.0, 50.0}) {
        const double expected = 24.0 / re * (1.0 + 0.15 * std::pow(re, 0.687));
        const double measured = sphereDrag(re, 20.0, 180, 100);
        std::printf("Re %.0f on a finer grid: Cd %.3f, the curve %.3f\n", re, measured, expected);
    }
}

// The whole tunnel on a cube in a URDF's shapes: what it writes, and a cube's size of drag.
TEST(WindTunnel, ACubeGivesItsDragAndItsFiles) {
    World world;
    const BodyId body = world.addBody("cube", kWorldBody, Pose{}, 1.0, {}, {1, 1, 1, 0, 0, 0});
    world.addCollider(body, ShapeType::Box, {0.25, 0.25, 0.25}, Pose{}, 1.0);
    const auto out = std::filesystem::temp_directory_path() / "lagrange_tunnel_test";
    WindTunnelOptions options;
    options.speed = 10.0;
    options.maxCells = 40000;
    options.passes = 2.0;
    auto result = runWindTunnel(world, options, out.string());
    ASSERT_TRUE(result) << result.error();
    // A cube face-on has a Cd of about 1.05 (Hoerner); a coarse grid with blockage reads high.
    std::printf("cube: Cd %.3f, measured 1.05\n", result->dragCoefficient);
    EXPECT_GT(result->dragCoefficient, 0.8);
    EXPECT_LT(result->dragCoefficient, 1.8);
    EXPECT_NEAR(result->frontalArea, 0.25, 0.06);
    EXPECT_NEAR(result->liftCoefficient, 0.0, 0.1);
    EXPECT_NEAR(result->links[0].force[0], result->force[0], 1e-9);
    for (const char* file : {"flow.vti", "flow.nvdb", "flow_view.usda", "report.json"}) {
        EXPECT_TRUE(std::filesystem::exists(out / file)) << file;
    }
    auto field = fields::Field::load((out / "flow.nvdb").string());
    ASSERT_TRUE(field) << field.error();
    // Far ahead of the cube, the air is at the tunnel's speed.
    const fields::Vec3 ahead = (*field)->sampleVector({-0.6, 0.6, 0.6});
    EXPECT_NEAR(ahead[0], 10.0, 1.5);
    std::filesystem::remove_all(out);
    // Nothing to meet is refused.
    World empty;
    EXPECT_FALSE(runWindTunnel(empty, options, ""));
}

// A road moving with the wind disturbs nothing: uniform flow stays uniform over it.
TEST(Lbm, ARollingRoadLeavesTheWindAlone) {
    LbmConfig config;
    config.nx = 40;
    config.ny = 12;
    config.nz = 20;
    config.cellSize = 0.05;
    config.windSpeed = 10.0;
    LbmTunnel tunnel(config);
    for (std::size_t y = 0; y < config.ny; ++y) {
        for (std::size_t x = 0; x < config.nx; ++x) {
            tunnel.setRollingFloor(x, y, 0);
            tunnel.setRollingFloor(x, y, config.nz - 1);  // and above, as sides wrap in z
        }
    }
    tunnel.step(2000);
    for (const std::size_t z : {std::size_t{1}, std::size_t{10}}) {
        const auto u = tunnel.velocity(tunnel.index(20, 6, z));
        EXPECT_NEAR(u[0], 10.0, 1e-2) << z;
        EXPECT_NEAR(u[2], 0.0, 1e-2) << z;
    }
}

// The GPU kernel is the same update as the CPU reference: the same drag on the same plate, here on
// whatever device is there (software Vulkan in CI).
TEST(Lbm, TheGpuAgreesWithTheCpu) {
    std::string error;
    auto device = gpu::Device::create(gpu::DeviceDesc{}, error);
    if (!device) {
        GTEST_SKIP() << error;
    }
    LbmConfig config;
    config.nx = 40;
    config.ny = 24;
    config.nz = 24;
    config.cellSize = 0.02;
    config.windSpeed = 5.0;
    const auto run = [&config](const gpu::Device* on) {
        LbmTunnel tunnel(config);
        for (std::size_t z = 8; z < 16; ++z) {
            for (std::size_t y = 8; y < 16; ++y) {
                tunnel.setSolid(12, y, z, 0);
            }
        }
        if (on != nullptr) {
            tunnel.useGpu(*on);
            EXPECT_TRUE(tunnel.onGpu());
        }
        tunnel.step(600);
        std::array<double, 4> out{};
        out[0] = tunnel.velocity(tunnel.index(30, 12, 12))[0];
        out[1] = tunnel.velocity(tunnel.index(14, 10, 12))[0];
        out[2] = tunnel.pressure(tunnel.index(11, 12, 12));
        tunnel.startAveraging();
        tunnel.step(200);
        out[3] = tunnel.loads()[0].force[0];
        return out;
    };
    const auto cpu = run(nullptr);
    const auto gpu = run(device.get());
    std::printf("cpu %.4f N, gpu %.4f N (a mean over samples, against over every step)\n", cpu[3],
                gpu[3]);
    // The same state, to what float adds up to over 600 steps.
    EXPECT_NEAR(gpu[0], cpu[0], 1e-3 * std::abs(cpu[0]));
    EXPECT_NEAR(gpu[1], cpu[1], 1e-3 * std::abs(cpu[1]));
    EXPECT_NEAR(gpu[2], cpu[2], 1e-2 * std::abs(cpu[2]));
    EXPECT_NEAR(gpu[3], cpu[3], 0.1 * std::abs(cpu[3]));
}

TEST(Lbm, TheSameRunGivesTheSameBits) {
    LbmConfig config;
    config.nx = 40;
    config.ny = 24;
    config.nz = 24;
    config.windSpeed = 5.0;
    config.cellSize = 0.02;
    const auto run = [&config] {
        LbmTunnel tunnel(config);
        for (std::size_t z = 8; z < 16; ++z) {
            for (std::size_t y = 8; y < 16; ++y) {
                tunnel.setSolid(12, y, z, 0);
            }
        }
        tunnel.step(100);
        return tunnel.loads()[0].force[0];
    };
    EXPECT_EQ(run(), run());
}

// A flat plate facing the wind at high Reynolds number: the measured Cd is about 1.17 (Hoerner,
// Fluid-Dynamic Drag, 1965). With the eddy viscosity and a coarse grid the check is loose: it says
// the tunnel gives the right order for a bluff body, not an engineering number.
TEST(Lbm, ABluffPlatesDragIsTheRightSizeAtHighReynolds) {
    LbmConfig config;
    config.nx = 80;
    config.ny = 40;
    config.nz = 40;
    config.cellSize = 0.02;
    config.windSpeed = 20.0;
    LbmTunnel tunnel(config);
    for (std::size_t z = 15; z < 25; ++z) {
        for (std::size_t y = 15; y < 25; ++y) {
            tunnel.setSolid(25, y, z, 0);
        }
    }
    tunnel.step(3000);
    tunnel.step(1000);
    ASSERT_FALSE(tunnel.diverged());
    const double area = 0.2 * 0.2;
    const double cd = tunnel.loads()[0].force[0] / (0.5 * 1.2 * 20.0 * 20.0 * area);
    std::printf("plate: Cd %.3f, measured 1.17\n", cd);
    EXPECT_GT(cd, 0.9);
    EXPECT_LT(cd, 1.7);
}

}  // namespace
}  // namespace lagrange::aerodynamics
