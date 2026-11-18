// The sample quadcopter's downwash on water (downwash.md §5a; water.md "The surface under a
// rotor"): the dimple its jets press into a pond against ρ_air U²/(2 ρ_water g), and the load
// grid the aerodynamics domain hands the water's surface.
#include "lagrange/aerodynamics/Downwash.hpp"
#include "lagrange/aerodynamics/WaterAir.hpp"
#include "lagrange/hydrodynamics/Surface.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::aerodynamics {
namespace {

using Vec3 = Downwash::Vec3;

constexpr double kRho = 1.225;  // kg/m³, ISA sea level
constexpr double kG = 9.80665;
constexpr double kWater = 1000.0;
constexpr double kLevel = 1.2;  // m: the pond's surface over the ground

/// The sample quadcopter (assets/robots/quad_x500): 2.0 kg, APC 10x4.7 propellers (R = 0.127 m),
/// hubs at (±0.177, ±0.177), hovering with its rotor plane `height` over the pond.
std::vector<Downwash::Rotor> sampleQuad(double height) {
    constexpr double kMass = 2.0;
    constexpr double kRadius = 0.127;
    constexpr double kHub = 0.177;
    std::vector<Downwash::Rotor> rotors;
    for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
            Downwash::Rotor r;
            r.hub = {sx * kHub, sy * kHub, kLevel + height};
            r.thrustAxis = {0.0, 0.0, 1.0};
            r.radius = kRadius;
            r.thrust = kMass * kG / 4.0;
            r.induced = std::sqrt(r.thrust / (2.0 * kRho * std::numbers::pi * kRadius * kRadius));
            rotors.push_back(r);
        }
    }
    return rotors;
}

Downwash pond() {
    Downwash d;
    d.setGround({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
    d.setWater({Downwash::WaterSurface{"pond", {0.0, 0.0, kLevel}, 3.0, 3.0}});
    return d;
}

TEST(DownwashOnWater, TheSampleQuadsDimpleIsItsJetsStagnationPressure) {
    // Over a still pond the dimple settles at the hydrostatic depth of the jet's stagnation
    // pressure, p/(ρ_w g) = ρ_air U²/(2 ρ_w g), U the jet arriving at the surface. The deepest
    // point is where the pressure is highest: under each rotor while the jets are apart, under
    // the middle once they have merged.
    for (const double height : {0.5, 1.0, 2.0}) {
        Downwash d = pond();
        d.update(sampleQuad(height));
        ASSERT_EQ(d.groups()[0].hitWater, 0);
        WaterGrid grid;  // the server's: 128 cells of 3 cm
        hydrodynamics::SurfaceParams params;
        params.cells = grid.cells;
        params.cell = grid.cell;
        params.depth = 1.2;
        params.density = kWater;
        params.gravity = kG;
        hydrodynamics::SurfacePatch s(params);
        const std::array<double, 2> corner = patchCorner(grid, {0.0, 0.0});
        s.setState(corner, {}, {});
        const int n = grid.cells;
        std::vector<double> p(static_cast<std::size_t>(n * n));
        double highest = 0.0;
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const Vec3 at{corner[0] + (x + 0.5) * grid.cell, corner[1] + (y + 0.5) * grid.cell,
                              kLevel};
                p[static_cast<std::size_t>(y * n + x)] = d.surfacePressure(at, kRho, 0);
                highest = std::max(highest, p[static_cast<std::size_t>(y * n + x)]);
            }
        }
        for (int i = 0; i < 1500; ++i) {  // 30 s
            s.step(0.02, p);
        }
        const auto [deepest, top] = std::ranges::minmax(s.heights());
        const double far = s.heightAt(1.5, 0.0);  // the water round it
        const double dimple = far - deepest;
        const double expected = highest / (kWater * kG);
        // The arriving jet's speed and the dimple it makes, for the docs:
        // 0.5 m: U = 12.4 m/s (four jets), 1 m: 10.2 (merging), 2 m: 6.5 (merged); dimples
        // 9.6, 6.4 and 2.6 mm, each within 0.5 % of the formula.
        const double u = std::sqrt(2.0 * highest / kRho);
        EXPECT_NEAR(dimple / expected, 1.0, 0.02) << height << " m, U " << u << " m/s";
        EXPECT_GT(top - far, -1e-4);  // no great ridge: the volume spreads over the patch
    }
}

TEST(DownwashOnWater, TheLoadGridFollowsTheWakeAcrossThePond) {
    // WaterAir's corner: whole cells, the wake's centre in the patch's middle half; it does not
    // move while the drone stays there.
    WaterGrid grid;
    grid.cells = 64;
    grid.cell = 0.05;
    const std::array<double, 2> a = patchCorner(grid, {0.31, -0.42});
    EXPECT_NEAR(a[0], std::floor((0.31 - 1.6) / 0.05) * 0.05, 1e-12);
    EXPECT_NEAR(std::remainder(a[0], 0.05), 0.0, 1e-9);
    EXPECT_NEAR(std::remainder(a[1], 0.05), 0.0, 1e-9);
    EXPECT_LE(a[0], 0.31 - 1.6 + 1e-12);
    EXPECT_GT(a[0], 0.31 - 1.6 - 0.05);
}

// --- Spray -----------------------------------------------------------------------------------

TEST(DownwashSpray, KelvinHelmholtzOnsetOverWater) {
    // 2·(ρa + ρw)/(ρa·ρw)·√(gσ(ρw − ρa)), Lamb §268: about 6.6 m/s for air over water.
    const double u = kelvinHelmholtzOnset(kRho, kWater, 0.07275, kG);
    EXPECT_NEAR(u, 6.6, 0.05);
    EXPECT_EQ(sprayEmission(0.99 * u, 0.05 * u, kRho, u), 0.0);
    EXPECT_GT(sprayEmission(1.5 * u, 0.075 * u, kRho, u), 0.0);
}

TEST(DownwashSpray, DropsFallAtGunnAndKinzersSpeeds) {
    // Terminal speeds measured by Gunn and Kinzer (J. Meteor. 6, 1949, table 2; TODO(verify)
    // the table's values): 0.1 mm 0.27 m/s, 0.2 mm 0.72, 0.5 mm 2.06; and Stokes's for 50 µm.
    EXPECT_NEAR(dropTerminalSpeed(1e-4, kRho, kG), 0.27, 0.03);
    EXPECT_NEAR(dropTerminalSpeed(2e-4, kRho, kG), 0.72, 0.07);
    EXPECT_NEAR(dropTerminalSpeed(5e-4, kRho, kG), 2.06, 0.2);
    const double stokes = kWater * kG * 50e-6 * 50e-6 / (18.0 * kAirViscosity);
    EXPECT_NEAR(dropTerminalSpeed(50e-6, kRho, kG) / stokes, 0.95, 0.02);  // Re 0.25: −5 %
    // Q_ext = 2: 1 g/m³ of 100 µm drops dims light by 3·W/(ρ_w D) = 0.03 /m.
    const std::array<double, 3> w{0.0, 1e-3, 0.0};
    EXPECT_NEAR(sprayExtinction(w), 3.0 * 1e-3 / (kWater * kSprayDiameters[1]), 1e-12);
}

/// The sample quadcopter hovering `height` over the pond, its downwash and spray run by WaterAir
/// as the aerodynamics domain runs it (wind still: the air is the wake).
class SprayRig final : public IModule {
public:
    explicit SprayRig(double height) : height_(height) {}
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "spray", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return Duration{10'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        downwash_ = pond();
        WaterGrid grid;
        return air.declare(downwash_.water(), grid, true, {"r0", "r1", "r2", "r3"}, kG, d);
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        air.init(bus);
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        const std::vector<Downwash::Rotor> rotors = sampleQuad(height_);
        downwash_.update(rotors);
        std::vector<Vec3> hubs;
        for (const Downwash::Rotor& r : rotors) {
            hubs.push_back(r.hub);
        }
        air.step(
            context, downwash_, kRho, hubs, [&](const Vec3& x) { return downwash_.velocityAt(x); },
            bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }
    WaterAir air;

private:
    double height_;
    Downwash downwash_;
};

struct SprayRun {
    double peak{0.0}, rate{0.0}, nearDrone{0.0}, extinction{0.0}, rotor{0.0}, ring{0.0};
    std::size_t drops{0};
};

SprayRun runSpray(double height, double seconds) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto rig = std::make_unique<SprayRig>(height);
    const SprayRig* rigPtr = rig.get();
    scheduler.add(std::move(rig));
    const World world;
    EXPECT_TRUE(scheduler.wire(WorldView(world)));
    SprayRun out;
    const auto read = [&](const char* name) {
        return scheduler.bus().read(scheduler.bus().find(name));
    };
    // The mean over the last half, the cloud having filled.
    const int steps = static_cast<int>(seconds / 0.04);
    for (int i = 1; i <= steps; ++i) {
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(i * 40'000'000LL)}).ok());
        if (i > steps / 2) {
            out.nearDrone += read("env/spray/downwash") / (steps - steps / 2);
            out.extinction += read("env/spray/extinction") / (steps - steps / 2);
            out.rotor += read("rotor/r0/spray") / (steps - steps / 2);
        }
    }
    // The ring the outwash carries the drops in: the most water round it.
    for (double r = 0.5; r <= 3.0; r += 0.1) {
        for (double z = 0.05; z <= 0.6; z += 0.05) {
            for (double a = 0.0; a < 2.0 * std::numbers::pi; a += std::numbers::pi / 4.0) {
                out.ring = std::max(
                    out.ring, rigPtr->air.sprayAt({r * std::cos(a), r * std::sin(a), kLevel + z}));
            }
        }
    }
    out.drops = rigPtr->air.drops();
    out.peak = read("water/pond/outwash_peak");
    out.rate = read("water/pond/spray_rate");
    return out;
}

TEST(DownwashSpray, NoSprayAboveTheOnsetHeightSprayBelowIt) {
    // The outwash's peak falls with height as the jet that arrives slows: for the sample
    // quadcopter it passes the 6.6 m/s onset below about 2.3 m (rotor plane over the water).
    const double onset = kelvinHelmholtzOnset(kRho, kWater, 0.07275, kG);
    double lastBelow = 0.0;
    for (double h = 0.5; h <= 4.0; h += 0.1) {
        Downwash d = pond();
        d.update(sampleQuad(h));
        double peak = 0.0;
        for (double r = 0.02; r < 2.5; r += 0.01) {
            peak = std::max(peak, std::hypot(d.surfaceOutwash({r, 0.0, kLevel}, 0)[0],
                                             d.surfaceOutwash({r, 0.0, kLevel}, 0)[1]));
        }
        if (peak > onset) {
            lastBelow = h;
        }
    }
    EXPECT_NEAR(lastBelow, 2.3, 0.25) << "the sample's spray onset height";
    const SprayRun high = runSpray(3.0, 4.0);
    EXPECT_LT(high.peak, onset);
    EXPECT_EQ(high.rate, 0.0);
    EXPECT_EQ(high.nearDrone, 0.0);
    EXPECT_EQ(high.extinction, 0.0);
    EXPECT_EQ(high.drops, 0U);
    const SprayRun low = runSpray(0.5, 4.0);
    EXPECT_GT(low.peak, onset);
    EXPECT_GT(low.rate, 0.0);
    EXPECT_GT(low.drops, 100U);
    EXPECT_GT(low.extinction, 0.0);  // round the drone, what its cameras look through
    // 0.5 m up: the outwash's peak 13.7 m/s, 5.2 g/s torn off, about 1400 parcels, 1.1 g/m³ in
    // the ring at its densest, 0.0013 /m of extinction round the drone, none at the drone.
    // The drops ride the outwash out in a ring, about a gram per cubic metre at its densest (a
    // heavy fog's water; kSprayEntrainment's behavioural calibration): 0.1 to 10 g/m³. Without
    // the recirculation the fitted wake leaves out, few come back up to the drone itself.
    EXPECT_GT(low.ring, 1e-4);
    EXPECT_LT(low.ring, 1e-2);
    EXPECT_LT(low.nearDrone, low.ring);
}

}  // namespace
}  // namespace lagrange::aerodynamics
