// The water's moving surface as lidars see it (docs/physics/water.md "The surface under a
// rotor"; sensors.md): a ray meets the patch at η, not at the still level. Closed forms: a
// vertical ray hits the level plus η there; any ray's hit lies on the surface and is its first
// crossing, as a fine march finds it, within a hundredth of a cell; a lidar over a rotor's dimple
// reads the dimple's depth, on the CPU reference and on the GPU path alike.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/MovingWater.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/Model.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

constexpr int kCells = 128;
constexpr double kCell = 0.03;                          // m, the server's patch
constexpr double kDimple = 0.025;                       // m deep
constexpr double kDimpleWidth = 0.25;                   // m, its Gaussian's 1/e radius
constexpr double kRipple = 0.0008;                      // m, rings round it
constexpr double kRippleLength = 0.12;                  // m
constexpr std::array<double, 2> kCorner{-1.92, -1.92};  // the patch centred on the origin

double eta(double x, double y) {
    const double r = std::hypot(x, y);
    return -kDimple * std::exp(-(r * r) / (kDimpleWidth * kDimpleWidth)) +
           kRipple * std::cos(2.0 * std::numbers::pi * r / kRippleLength) * std::exp(-r / 1.0);
}

std::vector<double> patch() {
    std::vector<double> h(static_cast<std::size_t>(kCells) * kCells);
    for (int y = 0; y < kCells; ++y) {
        for (int x = 0; x < kCells; ++x) {
            h[static_cast<std::size_t>(y) * kCells + x] =
                eta(kCorner[0] + (x + 0.5) * kCell, kCorner[1] + (y + 0.5) * kCell);
        }
    }
    return h;
}

MovingWater water(const std::vector<double>& heights, double level) {
    MovingWater w;
    w.level = level;
    w.lo = {-3.0, -3.0};
    w.hi = {3.0, 3.0};
    w.cells = kCells;
    w.cell = kCell;
    w.corner = kCorner;
    w.heights = heights;
    EXPECT_TRUE(w.prepare());
    return w;
}

TEST(MovingWater, AVerticalRayHitsTheLevelPlusEta) {
    const std::vector<double> h = patch();
    const MovingWater w = water(h, 0.5);
    EXPECT_NEAR(w.lowest, -kDimple + kRipple, 1e-3);
    for (const double x : {0.0, 0.013, 0.21, -0.47, 1.1}) {
        for (const double y : {0.0, -0.08, 0.36}) {
            const double t = rayOntoWater(w, {x, y, 2.5}, {0, 0, -1}, 2.0, true);
            EXPECT_NEAR(2.5 - t, 0.5 + w.heightAt(x, y), 1e-9) << x << ", " << y;
            // The patch's bilinear surface is the physics' η within its sampling.
            EXPECT_NEAR(w.heightAt(x, y), eta(x, y), 2e-3);
        }
    }
    // The dimple's bottom, under the middle: 25 mm below the level, less the ripple there.
    const double t = rayOntoWater(w, {0.0, 0.0, 2.5}, {0, 0, -1}, 2.0, true);
    EXPECT_NEAR(t - 2.0, kDimple - kRipple, 2e-3);
}

TEST(MovingWater, ASlantedRayStopsAtItsFirstCrossing) {
    const std::vector<double> h = patch();
    const MovingWater w = water(h, 0.0);
    int checked = 0;
    for (double azimuth = 0.0; azimuth < 2.0 * std::numbers::pi; azimuth += 0.37) {
        for (const double down : {0.15, 0.4, 0.9}) {
            const std::array<double, 3> o{-0.6 * std::cos(azimuth), -0.6 * std::sin(azimuth), 0.4};
            const std::array<double, 3> d{std::cos(azimuth) * std::cos(down),
                                          std::sin(azimuth) * std::cos(down), -std::sin(down)};
            const double flat = -o[2] / d[2];
            bool on = false;
            const double t = rayOntoWater(w, o, d, flat, true, &on);
            ASSERT_TRUE(on);
            const double x = o[0] + d[0] * t;
            const double y = o[1] + d[1] * t;
            EXPECT_NEAR(o[2] + d[2] * t, w.heightAt(x, y), 1e-9);
            // A march at a hundredth of a cell finds the same first crossing.
            const double step = kCell / 100.0;
            double first = std::numeric_limits<double>::infinity();
            for (double s = 0.0; s < 10.0; s += step) {
                if (o[2] + d[2] * s - w.heightAt(o[0] + d[0] * s, o[1] + d[1] * s) <= 0.0) {
                    first = s;
                    break;
                }
            }
            EXPECT_NEAR(t, first, step) << azimuth << " " << down;
            ++checked;
        }
    }
    EXPECT_GT(checked, 40);
}

TEST(MovingWater, ACrestInFrontOfAThingAndNothingBesideTheWater) {
    std::vector<double> h(static_cast<std::size_t>(kCells) * kCells, 0.0);
    // One raised cell row (a crest 5 mm high along x = 0).
    for (int y = 0; y < kCells; ++y) {
        h[static_cast<std::size_t>(y) * kCells + 64] = 0.005;
    }
    const MovingWater w = water(h, 0.0);
    // A grazing ray just above the level towards a thing at 1 m: the crest stops it first.
    const std::array<double, 3> o{-0.5, 0.0, 0.006};
    const std::array<double, 3> d{1.0, 0.0, -0.002};
    bool on = false;
    const double t = rayOntoWater(w, o, d, 1.0, false, &on);
    EXPECT_TRUE(on);
    EXPECT_LT(t, 0.6);
    EXPECT_GT(t, 0.4);
    // The same ray with the thing nearer than the crest: the thing.
    EXPECT_EQ(rayOntoWater(w, o, d, 0.3, false, &on), 0.3);
    EXPECT_FALSE(on);
    // From under the surface: left alone; and beside the water's extent: no surface there.
    EXPECT_EQ(rayOntoWater(w, {0.0, 0.0, -0.1}, {0, 0, -1}, 5.0, false), 5.0);
    MovingWater small = w;
    small.lo = {-0.2, -0.2};
    small.hi = {0.2, 0.2};
    EXPECT_EQ(rayOntoWater(small, {1.0, 0.0, 1.0}, {0, 0, -1}, 3.0, true), 3.0);
}

// --- a lidar over the dimple ----------------------------------------------------------------

/// Publishes water/pond/surface and surface_origin, as the hydrodynamics domain does.
class SurfaceFeed final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "surface_feed", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{100'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "water/pond/surface";
        s.type = SignalType::ScalarArray;
        s.unit = units::m;
        s.frame = "world_local";
        s.arrayLength = kCells * kCells;
        heights_ = d.output(s);
        s.name = "water/pond/surface_origin";
        s.arrayLength = 2;
        origin_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.writeN(heights_, patch());
        bus.writeN(origin_, std::vector<double>{kCorner[0], kCorner[1]});
        return {};
    }
    Status step(const StepContext&, SignalBus&) override { return {}; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId heights_, origin_;
};

/// A 6 m pond, its still surface at z = 0, and a lidar 1 m over its middle looking down: nine
/// beams over ±0.4 rad across y.
World pond() {
    World world;
    const PrimId prim = world.addPrim("/pond", "pond");
    SchemaInstance s;
    s.schema = "WaterAPI_1";
    s.prim = prim;
    s.reals = {{"water:sizeX", 6.0}, {"water:sizeY", 6.0}, {"water:depth", 1.0}};
    world.schemaInstances.push_back(s);
    const BodyId post = world.addBody("post", kWorldBody, Pose{{0.0, 0.0, 1.0}}, 0.0, {}, {});
    Pose down;  // the sensor's +x straight down, its fan across world y
    down.orientation = {std::cos(std::numbers::pi / 4.0), 0.0, std::sin(std::numbers::pi / 4.0),
                        0.0};
    world.addSensor("scanner", SensorType::Lidar, post, down, 10.0,
                    {{"samples", 9.0},
                     {"min_angle", -0.4},
                     {"max_angle", 0.4},
                     {"min_range", 0.1},
                     {"max_range", 10.0},
                     {"range_noise", 0.0}});
    return world;
}

std::vector<double> scan(bool moving, const std::shared_ptr<gpu::RayContext>& gpu) {
    const World world = pond();
    Scheduler scheduler;
    ModuleConfigData config;
    if (moving) {
        scheduler.add(std::make_unique<SurfaceFeed>());
        config.values["water_surface"] = true;
        config.values["surface_cells"] = std::int64_t{kCells};
        config.values["surface_cell"] = kCell;
    }
    config.tier = gpu ? CapabilityTier::Tier1 : CapabilityTier::Tier0;
    for (auto& module : createSensorModules(world, gpu)) {
        scheduler.add(std::move(module), ModuleConfig(config));
    }
    auto wired = scheduler.wire(WorldView(world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    if (!wired) {
        return {};
    }
    EXPECT_TRUE(scheduler.stepTick().ok());
    const std::span<const double> r =
        scheduler.bus().readN(scheduler.bus().find("sensor/scanner/ranges"));
    return {r.begin(), r.end()};
}

void expectOnTheDimple(const std::vector<double>& r, double tolerance) {
    ASSERT_EQ(r.size(), 9U);
    const std::vector<double> h = patch();
    const MovingWater w = water(h, 0.0);
    int returns = 0;
    for (std::size_t i = 0; i < r.size(); ++i) {
        // Water returns a lidar's light only near normal incidence (a mirror, sensors.md): the
        // outer beams see nothing, as over still water.
        if (!std::isfinite(r[i])) {
            continue;
        }
        ++returns;
        const double a = -0.4 + 0.1 * static_cast<double>(i);
        // Beam i leaves (0, 0, 1) at angle a from straight down, across y.
        const double y = std::sin(a) * r[i];
        const double z = 1.0 - std::cos(a) * r[i];
        EXPECT_NEAR(z, w.heightAt(0.0, y), tolerance) << "beam " << i;
    }
    EXPECT_GE(returns, 3);  // the middle one and its neighbours
    // Straight down: the dimple's depth below the level (25 mm, less the ripple at its middle).
    EXPECT_NEAR(r[4] - 1.0, kDimple - kRipple, 2e-3);
}

TEST(MovingWater, ALidarReadsTheDimpleOnTheCpuReference) {
    const std::vector<double> flat = scan(false, nullptr);
    ASSERT_EQ(flat.size(), 9U);
    EXPECT_NEAR(flat[4], 1.0, 1e-12);  // the still level without the surface
    expectOnTheDimple(scan(true, nullptr), 1e-9);
    EXPECT_EQ(scan(true, nullptr), scan(true, nullptr));  // invariant 7: bit for bit
}

std::shared_ptr<gpu::RayContext> context() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        auto created = gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
        if (!created) {
            std::fprintf(stderr, "no Vulkan device for the GPU lidar: %s\n", error.c_str());
        }
        return created;
    }();
    return made;
}

TEST(MovingWater, ALidarReadsTheDimpleOnTheGpuPath) {
    const auto gpu = context();
    if (!gpu) {
        GTEST_SKIP() << "no usable Vulkan device";
    }
    // The GPU casts the flat box in float; the move onto η is in double: within the tiers'
    // 1 mm (sensors.md §7) of the surface.
    expectOnTheDimple(scan(true, gpu), 1e-3);
}

}  // namespace
}  // namespace lagrange::sensors
