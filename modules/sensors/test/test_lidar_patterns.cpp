// Lidar scan patterns (sensors.md, "Lidar scan patterns (as built)"): a channel table, a flash
// lidar's focal-plane grid and a Livox-style rosette, against closed forms; and the lidar module
// casting them, on the CPU and, where a Vulkan device is, on the GPU.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/Model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using Settings = LidarLayout::Settings;

// --- the layout -----------------------------------------------------------------------------

/// The grid as it was before the patterns: every existing lidar must keep its beams exactly.
std::array<double, 3> oldDirection(const Settings& s, std::uint32_t index) {
    const auto get = [&s](const char* key, double fallback) {
        const auto it = s.find(key);
        return it == s.end() ? fallback : it->second;
    };
    const auto h = static_cast<std::uint32_t>(get("samples", 360.0));
    const auto v = static_cast<std::uint32_t>(std::max(1.0, get("vertical_samples", 1.0)));
    const double minA = get("min_angle", -3.14159265358979);
    const double maxA = get("max_angle", 3.14159265358979);
    const double minE = get("vertical_min_angle", 0.0);
    const double maxE = get("vertical_max_angle", 0.0);
    const std::uint32_t c = index % h;
    const std::uint32_t r = index / h;
    const double a = h > 1 ? minA + (maxA - minA) * c / (h - 1) : minA;
    const double e = v > 1 ? minE + (maxE - minE) * r / (v - 1) : minE;
    return {std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e)};
}

TEST(LidarPatterns, TheGridKeepsEveryBeamBitForBit) {
    const Settings vlp16{
        {"samples", 1800},        {"min_angle", -std::numbers::pi}, {"max_angle", std::numbers::pi},
        {"vertical_samples", 16}, {"vertical_min_angle", -0.2618},  {"vertical_max_angle", 0.2618}};
    for (const Settings& s : {vlp16, Settings{{"samples", 360}}, Settings{}}) {
        const LidarLayout layout = LidarLayout::from(s);
        EXPECT_EQ(layout.pattern, LidarLayout::Pattern::Grid);
        EXPECT_FALSE(layout.timeVarying());
        EXPECT_FALSE(layout.simultaneous());
        for (std::uint32_t i = 0; i < layout.beams(); ++i) {
            const auto d = layout.direction(i);
            const auto old = oldDirection(s, i);
            ASSERT_EQ(d, old) << i;                  // exactly
            ASSERT_EQ(layout.direction(i, 7), old);  // the scan does not matter
        }
    }
    EXPECT_FALSE(LidarLayout::from(Settings{{"samples", 360}}).is3d());
}

TEST(LidarPatterns, AChannelTableGivesEachRingItsElevationLowestFirst) {
    // Velodyne VLP-32C style: unevenly spaced channels, listed in firing order.
    const std::vector<double> table{-0.4363, 0.0262, -0.0175, 0.2618, -0.0087, -0.1396};
    Settings s{{"samples", 4}, {"min_angle", 0.0}, {"max_angle", 1.5 * std::numbers::pi}};
    s["vertical_samples"] = 2.0;  // ignored: the table has six
    for (std::size_t i = 0; i < table.size(); ++i) {
        s["vertical_angle_" + std::to_string(i)] = table[i];
    }
    const LidarLayout layout = LidarLayout::from(s);
    ASSERT_EQ(layout.vertical, 6U);
    ASSERT_EQ(layout.beams(), 24U);
    EXPECT_TRUE(layout.is3d());
    std::vector<double> sorted = table;
    std::ranges::sort(sorted);
    for (std::uint32_t ring = 0; ring < 6; ++ring) {
        EXPECT_EQ(layout.elevation(ring), sorted[ring]);
        for (std::uint32_t column = 0; column < 4; ++column) {
            const auto d = layout.direction(ring * 4 + column);
            EXPECT_NEAR(std::asin(d[2]), sorted[ring], 1e-12);
            EXPECT_NEAR(std::atan2(d[1], d[0]),
                        std::remainder(column * std::numbers::pi / 2.0, 2.0 * std::numbers::pi),
                        1e-12);
        }
    }
    EXPECT_EQ(layout.minElevation, sorted.front());
    EXPECT_EQ(layout.maxElevation, sorted.back());
}

TEST(LidarPatterns, AFlashLidarsBeamsAreAFocalPlaneGrid) {
    // A 64 x 16 array over 120° x 30°, its outer beams at the field's edges.
    const double h = 2.0 * std::numbers::pi / 3.0;
    const double v = std::numbers::pi / 6.0;
    const Settings s{{"pattern", 1},
                     {"samples", 64},
                     {"min_angle", -h / 2.0},
                     {"max_angle", h / 2.0},
                     {"vertical_samples", 16},
                     {"vertical_min_angle", -v / 2.0},
                     {"vertical_max_angle", v / 2.0}};
    const LidarLayout layout = LidarLayout::from(s);
    EXPECT_TRUE(layout.simultaneous());
    EXPECT_TRUE(layout.is3d());
    ASSERT_EQ(layout.beams(), 64U * 16U);
    // On the plane x = 1, the beams are evenly spaced in y and z: a pinhole's pixels.
    const double dy = 2.0 * std::tan(h / 2.0) / 63.0;
    const double dz = 2.0 * std::tan(v / 2.0) / 15.0;
    for (std::uint32_t ring = 0; ring < 16; ++ring) {
        for (std::uint32_t column = 0; column < 64; ++column) {
            const auto d = layout.direction(ring * 64 + column);
            EXPECT_NEAR(std::hypot(d[0], d[1], d[2]), 1.0, 1e-12);
            EXPECT_NEAR(d[1] / d[0], -std::tan(h / 2.0) + column * dy, 1e-12);
            EXPECT_NEAR(d[2] / d[0], -std::tan(v / 2.0) + ring * dz, 1e-12);
        }
    }
    // The corner beam is at the field's edge in both angles: azimuth −60°, and an elevation
    // whose tangent, on the plane x = 1, is tan 15° (the beam itself leans less, being further).
    const auto corner = layout.direction(0);
    EXPECT_NEAR(std::atan2(corner[1], corner[0]), -h / 2.0, 1e-12);
    EXPECT_NEAR(corner[2] / corner[0], -std::tan(v / 2.0), 1e-12);
    EXPECT_NEAR(layout.azimuth(63), h / 2.0, 1e-12);
    EXPECT_NEAR(layout.elevation(0), -v / 2.0, 1e-12);
}

/// The rosette's deflection of a beam: its angle off the x axis, and the way round.
std::array<double, 2> deflection(const std::array<double, 3>& d) {
    const double off = std::acos(std::clamp(d[0], -1.0, 1.0));
    const double round = std::atan2(d[2], d[1]);
    return {off * std::cos(round), off * std::sin(round)};
}

TEST(LidarPatterns, ARosetteStaysWithinItsPrismsReach) {
    // Unequal prisms: the beam sweeps the annulus |δ1 - δ2| <= off <= δ1 + δ2.
    const Settings s{{"pattern", 2}, {"rosette_fov", 0.8}, {"rosette_prism_ratio", 0.6}};
    const LidarLayout layout = LidarLayout::from(s);
    EXPECT_EQ(layout.horizontal, 10000U);  // 100 000 points/s at 10 Hz
    EXPECT_EQ(layout.vertical, 1U);
    EXPECT_TRUE(layout.is3d());
    EXPECT_TRUE(layout.timeVarying());
    EXPECT_NEAR(layout.rosette.deflection1 + layout.rosette.deflection2, 0.4, 1e-15);
    EXPECT_NEAR(layout.rosette.deflection2 / layout.rosette.deflection1, 0.6, 1e-12);
    const double outer = 0.4;
    const double inner = 0.4 * (1.0 - 0.6) / 1.6;
    double lo = 1.0;
    double hi = 0.0;
    for (std::uint64_t scan = 0; scan < 5; ++scan) {
        for (std::uint32_t i = 0; i < layout.horizontal; ++i) {
            const auto d = layout.direction(i, scan);
            ASSERT_NEAR(std::hypot(d[0], d[1], d[2]), 1.0, 1e-12);
            const double off = std::acos(std::clamp(d[0], -1.0, 1.0));
            lo = std::min(lo, off);
            hi = std::max(hi, off);
        }
    }
    EXPECT_GE(lo, inner - 1e-9);
    EXPECT_LE(hi, outer + 1e-9);
    EXPECT_NEAR(lo, inner, 2e-3);  // and it reaches both edges
    EXPECT_NEAR(hi, outer, 2e-3);
}

TEST(LidarPatterns, AnEqualPrismRosettesRadiiFollowTheClosedForm) {
    // With equal wedges δ the deflection is 2δ|cos φ|, φ = (θ1 − θ2)/2 turning steadily, so the
    // fraction of points within R of the axis is 1 − (2/π) arccos(R / 2δ).
    const LidarLayout layout = LidarLayout::from(Settings{{"pattern", 2}});
    const double reach = LidarLayout::kRosetteFov / 2.0;
    std::vector<double> radii;
    for (std::uint64_t scan = 0; scan < 20; ++scan) {
        for (std::uint32_t i = 0; i < layout.horizontal; ++i) {
            const auto d = deflection(layout.direction(i, scan));
            radii.push_back(std::hypot(d[0], d[1]));
        }
    }
    std::ranges::sort(radii);
    for (const double f : {0.1, 0.25, 0.5, 0.75, 0.9, 0.99}) {
        const double r = f * reach;
        const double measured =
            static_cast<double>(std::ranges::upper_bound(radii, r) - radii.begin()) /
            static_cast<double>(radii.size());
        const double expected = 1.0 - (2.0 / std::numbers::pi) * std::acos(r / reach);
        EXPECT_NEAR(measured, expected, 0.005) << "R = " << f << " of the reach";
    }
}

TEST(LidarPatterns, ARosettesCoverageGrowsWithIntegrationTime) {
    // 0.5° cells over the disc: each scan goes on where the last one stopped, so more of them
    // are hit the longer the points are gathered, unlike a repeating scan.
    const LidarLayout layout = LidarLayout::from(Settings{{"pattern", 2}});
    const double reach = LidarLayout::kRosetteFov / 2.0;
    const double cell = 0.5 * std::numbers::pi / 180.0;
    const int n = static_cast<int>(std::ceil(reach / cell));
    int inside = 0;
    for (int y = -n; y < n; ++y) {
        for (int z = -n; z < n; ++z) {
            inside += std::hypot((y + 0.5) * cell, (z + 0.5) * cell) <= reach ? 1 : 0;
        }
    }
    std::set<std::pair<int, int>> hit;
    std::vector<double> coverage;
    for (std::uint64_t scan = 0; scan < 10; ++scan) {
        for (std::uint32_t i = 0; i < layout.horizontal; ++i) {
            const auto d = deflection(layout.direction(i, scan));
            const int y = static_cast<int>(std::floor(d[0] / cell));
            const int z = static_cast<int>(std::floor(d[1] / cell));
            if (std::hypot((y + 0.5) * cell, (z + 0.5) * cell) <= reach) {
                hit.insert({y, z});
            }
        }
        coverage.push_back(static_cast<double>(hit.size()) / inside);
    }
    for (std::size_t k = 1; k < coverage.size(); ++k) {
        EXPECT_GT(coverage[k], coverage[k - 1]) << "after " << k + 1 << " scans";
    }
    EXPECT_LT(coverage.front(), 0.5);  // one scan (0.1 s) leaves gaps
    EXPECT_GT(coverage.back(), 0.99);  // a second fills the disc
    // Not repetitive: the next scan's beams are not this one's.
    EXPECT_NE(layout.direction(0, 0), layout.direction(0, 1));
    // Deterministic: the same scan and beam give the same direction.
    EXPECT_EQ(layout.direction(1234, 56),
              LidarLayout::from(Settings{{"pattern", 2}}).direction(1234, 56));
}

// --- the lidar module -----------------------------------------------------------------------

/// A wall at x = 5 m and a lidar on a post at the origin, 1 m up; `extra` its settings.
World wall(const Settings& extra) {
    World world;
    world.addCollider(kWorldBody, ShapeType::Box, {0.1, 50.0, 50.0}, Pose{{5.1, 0, 1}}, 1.0);
    const BodyId post = world.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    Settings s{{"min_range", 0.1}, {"max_range", 100.0}};
    s.insert(extra.begin(), extra.end());
    world.addSensor("front", SensorType::Lidar, post, Pose{{0, 0, 1}}, 10.0, s);
    return world;
}

std::shared_ptr<gpu::RayContext> gpuContext() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        auto created = gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
        std::fprintf(stderr, "GPU lidar: %s\n",
                     created ? created->deviceName().c_str() : error.c_str());
        return created;
    }();
    return made;
}

struct Scan {
    std::vector<double> ranges, directions;
};

/// Runs the lidar for `scans` scans, on the CPU or the GPU, and keeps each scan.
std::vector<Scan> run(const World& world, int scans, bool gpu) {
    Scheduler scheduler;
    ModuleConfigData config;
    config.tier = gpu ? CapabilityTier::Tier1 : CapabilityTier::Tier0;
    for (auto& module : createSensorModules(world, gpu ? gpuContext() : nullptr)) {
        scheduler.add(std::move(module), ModuleConfig(config));
    }
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    std::vector<Scan> out;
    const SignalBus& bus = scheduler.bus();
    for (int k = 0; k < scans; ++k) {
        EXPECT_TRUE(scheduler.stepTick().ok());
        while (scheduler.now().ns % 100'000'000 != 0) {
            EXPECT_TRUE(scheduler.stepTick().ok());
        }
        Scan scan;
        const auto r = bus.readN(bus.find("sensor/front/ranges"));
        scan.ranges.assign(r.begin(), r.end());
        if (const SignalId d = bus.find("sensor/front/directions"); d.valid()) {
            const auto v = bus.readN(d);
            scan.directions.assign(v.begin(), v.end());
        }
        out.push_back(std::move(scan));
    }
    return out;
}

TEST(LidarPatterns, AFlashLidarSeesAWallAsAFocalPlaneDoes) {
    const Settings s{{"pattern", 1},
                     {"samples", 32},
                     {"min_angle", -0.6},
                     {"max_angle", 0.6},
                     {"vertical_samples", 8},
                     {"vertical_min_angle", -0.3},
                     {"vertical_max_angle", 0.3}};
    const World world = wall(s);
    const LidarLayout layout = LidarLayout::from(world.sensors.settings[0]);
    for (const bool gpu : {false, true}) {
        if (gpu && !gpuContext()) {
            continue;
        }
        const std::vector<Scan> scans = run(world, 1, gpu);
        ASSERT_EQ(scans.front().ranges.size(), 256U);
        EXPECT_TRUE(scans.front().directions.empty());  // a fixed pattern: none on the bus
        for (std::uint32_t i = 0; i < 256; ++i) {
            // The wall's face is 5 m ahead: each beam at 5 / cos of its angle off the axis.
            EXPECT_NEAR(scans.front().ranges[i], 5.0 / layout.direction(i)[0], gpu ? 1e-3 : 1e-9)
                << (gpu ? "gpu" : "cpu") << " beam " << i;
        }
    }
}

TEST(LidarPatterns, ARosetteLidarPublishesItsBeamsWithEachScan) {
    const Settings s{{"pattern", 2}, {"samples", 2000}, {"rosette_point_rate", 20000}};
    const World world = wall(s);
    const LidarLayout layout = LidarLayout::from(world.sensors.settings[0]);
    std::vector<Scan> cpu;
    for (const bool gpu : {false, true}) {
        if (gpu && !gpuContext()) {
            continue;
        }
        const std::vector<Scan> scans = run(world, 3, gpu);
        ASSERT_EQ(scans.size(), 3U);
        for (std::size_t k = 0; k < scans.size(); ++k) {
            const Scan& scan = scans[k];
            ASSERT_EQ(scan.ranges.size(), 2000U);
            ASSERT_EQ(scan.directions.size(), 6000U);
            for (std::uint32_t i = 0; i < 2000; ++i) {
                // The scan's own beams, as the layout gives them for that scan.
                const auto d = layout.direction(i, k);
                ASSERT_EQ(scan.directions[3 * i], d[0]);
                ASSERT_EQ(scan.directions[3 * i + 1], d[1]);
                ASSERT_EQ(scan.directions[3 * i + 2], d[2]);
                EXPECT_NEAR(scan.ranges[i], 5.0 / d[0], gpu ? 1e-3 : 1e-9);
            }
        }
        EXPECT_NE(scans[0].directions, scans[1].directions);
        if (!gpu) {
            cpu = scans;
        }
    }
    // The CPU reference is deterministic: a second run gives the same scans, bit for bit.
    const std::vector<Scan> again = run(world, 3, false);
    for (std::size_t k = 0; k < again.size(); ++k) {
        EXPECT_EQ(again[k].ranges, cpu[k].ranges);
        EXPECT_EQ(again[k].directions, cpu[k].directions);
    }
}

TEST(LidarPatterns, AFlashLidarMustLookThroughAFocalPlane) {
    const World world =
        wall({{"pattern", 1}, {"samples", 8}, {"min_angle", -2.0}, {"max_angle", 2.0}});
    Scheduler scheduler;
    for (auto& module : createSensorModules(world)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("π/2"), std::string::npos);
}

TEST(LidarPatterns, ACountOutOfReachIsRefusedBeforeItIsAllocated) {
    // samples -1 was cast to 4294967295 columns and 1e8 columns asked for gigabytes: both died
    // at start with std::bad_alloc. 2^22 beams is the most a scan takes.
    for (const Settings& settings : {Settings{{"samples", -1.0}}, Settings{{"samples", 1e8}},
                                     Settings{{"samples", 4096.0}, {"vertical_samples", 4096.0}}}) {
        const World world = wall(settings);
        Scheduler scheduler;
        for (auto& module : createSensorModules(world)) {
            scheduler.add(std::move(module));
        }
        auto wired = scheduler.wire(WorldView(world));
        ASSERT_FALSE(wired);
        EXPECT_NE(to_string(wired.error().front()).find("lidar"), std::string::npos);
    }
}

TEST(LidarPatterns, FifteenHertzRunsOnWholeMilliseconds) {
    // 1e9/15 ns shares no tick with the 1 ms physics: a lidar at 15 Hz was refused ("base tick
    // 1 ns"). Its period is 67 ms now; a period of whole nanoseconds (10, 128 Hz) stays exact.
    for (const auto& [rate, period] :
         {std::pair{15.0, 67'000'000LL}, std::pair{10.0, 100'000'000LL},
          std::pair{128.0, 7'812'500LL}}) {
        World world = wall({{"samples", 8}});
        world.sensors.rateHz[0] = rate;
        const auto modules = createSensorModules(world);
        ASSERT_EQ(modules.size(), 1U);
        EXPECT_EQ(modules[0]->preferredStep().ns, period) << rate;
    }
}

}  // namespace
}  // namespace lagrange::sensors
