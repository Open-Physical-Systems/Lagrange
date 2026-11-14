// Kind "imaging_radar" (sensors.md §2 "4D imaging radar"): the radar equation's SNR and the
// detection range it gives, the virtual array's angle cells λ/(N·d) (two targets inside one merge,
// two cells apart they do not; in azimuth and in elevation), Doppler folding at the unambiguous
// velocity λ/(4·T_c), Doppler telling apart two targets in one angle cell, range folding, the
// ground's mirror ghost at |Γ|⁴, and the GPU rays against the CPU reference.
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/Radar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;
using Settings = std::map<std::string, double, std::less<>>;

constexpr double kDensity = 0.1;  // m²/m², SensorMaterialAPI_1's fallback
constexpr double kPower = 0.01;   // W
constexpr double kGain = 100.0;   // 20 dBi
constexpr double kFrequency = 77e9;
constexpr double kLambda = 299792458.0 / kFrequency;  // m
constexpr double kQuiet = 1e-18;                      // W, a noise floor far below every echo

/// Writes a kinematic body's pose and twist (world_local), as physics would.
class Mover final : public IModule {
public:
    struct Body {
        std::string name;
        std::array<double, 3> position;
        std::array<double, 3> velocity;
    };
    explicit Mover(std::vector<Body> bodies) : bodies_(std::move(bodies)) {}
    ModuleInfo info() const override {
        return {"test", "mover", 1, 0, false, true, CapabilityTier::Tier0};
    }
    Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        for (const Body& b : bodies_) {
            SignalSpec pose;
            pose.name = "body/" + b.name + "/pose";
            pose.type = SignalType::Pose;
            pose.unit = units::m;
            pose.frame = "world_local";
            SignalSpec twist = pose;
            twist.name = "body/" + b.name + "/twist";
            twist.type = SignalType::Wrench;
            twist.unit = units::m_per_s;
            ids_.push_back({d.output(pose), d.output(twist)});
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus) {
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            const Body& b = bodies_[i];
            bus.writeN(ids_[i].first, std::array<double, 7>{b.position[0], b.position[1],
                                                            b.position[2], 1.0, 0.0, 0.0, 0.0});
            bus.writeN(ids_[i].second, std::array<double, 6>{b.velocity[0], b.velocity[1],
                                                             b.velocity[2], 0.0, 0.0, 0.0});
        }
    }
    std::vector<Body> bodies_;
    std::vector<std::pair<SignalId, SignalId>> ids_;
};

/// A detection as sensor/<n>/detections carries it.
struct Detection {
    double range, azimuth, elevation, velocity, rcs, snr;
};

struct Rig {
    World world;
    std::vector<Mover::Body> movers;
    std::unique_ptr<Scheduler> scheduler;

    /// The radar `height` up at the origin looking along +x, on a fixed mount.
    explicit Rig(Settings extra, double height = 1.0) {
        Pose mountPose;
        mountPose.position = {0.0, 0.0, height};
        const BodyId mount =
            world.addBody("mount", kWorldBody, mountPose, 1.0, {}, {1, 1, 1, 0, 0, 0});
        Settings s{{"max_range", 100.0},          {"azimuth_fov", 0.5},
                   {"elevation_fov", 0.2},        {"range_resolution", 0.2},
                   {"velocity_resolution", 0.1},  {"tx_power", kPower},
                   {"antenna_gain", kGain},       {"noise_floor", kQuiet},
                   {"detection_threshold", 10.0}, {"carrier_frequency", kFrequency}};
        for (const auto& [k, v] : extra) {
            s[k] = v;
        }
        world.addCustomSensor("front_radar", "imaging_radar", mount, Pose{}, 20.0, s, {});
    }
    /// A fixed sphere of radius `r` at (x, y, z).
    void sphere(double x, double y, double z, double r) {
        Pose p;
        p.position = {x, y, z};
        world.addCollider(kWorldBody, ShapeType::Sphere, {r, 0.0, 0.0}, p, 0.8);
    }
    /// A 2 m × 1 m plate facing the radar, its centre at (x, y, z), moving at `velocity`.
    void plate(const std::string& name, double x, double y, double z,
               std::array<double, 3> velocity = {}) {
        Pose p;
        p.position = {x, y, z};
        const BodyId b = world.addBody(name, kWorldBody, p, 10.0, {}, {1, 1, 1, 0, 0, 0});
        world.bodies.kinematic.back() = true;
        world.addCollider(b, ShapeType::Box, {0.01, 1.0, 0.5}, Pose{}, 0.8);
        movers.push_back({name, {x, y, z}, velocity});
    }
    /// A moving sphere (a kinematic body) of radius `r` at (x, y, z).
    void movingSphere(const std::string& name, double x, double y, double z, double r,
                      std::array<double, 3> velocity) {
        Pose p;
        p.position = {x, y, z};
        const BodyId b = world.addBody(name, kWorldBody, p, 1.0, {}, {1, 1, 1, 0, 0, 0});
        world.bodies.kinematic.back() = true;
        world.addCollider(b, ShapeType::Sphere, {r, 0.0, 0.0}, Pose{}, 0.8);
        movers.push_back({name, {x, y, z}, velocity});
    }
    void ground() { world.addCollider(kWorldBody, ShapeType::Plane, {0.0, 0.0, 1.0}, Pose{}, 0.8); }

    bool start(const std::shared_ptr<gpu::RayContext>& rayContext = nullptr) {
        scheduler =
            std::make_unique<Scheduler>(SchedulerOptions{.deterministic = rayContext == nullptr});
        if (!movers.empty()) {
            scheduler->add(std::make_unique<Mover>(movers));
        }
        auto radar = createBuiltinKind(world, 0, false, rayContext, nullptr);
        EXPECT_TRUE(radar);
        if (!radar) {
            return false;
        }
        ModuleConfigData config;
        config.tier = rayContext ? CapabilityTier::Tier1 : CapabilityTier::Tier0;
        scheduler->add(std::move(radar), ModuleConfig(config));
        auto wired = scheduler->wire(WorldView(world));
        if (!wired) {
            std::printf("%s\n", to_string(wired.error().front()).c_str());
        }
        return static_cast<bool>(wired);
    }

    std::vector<Detection> scan(std::int64_t ns = 120'000'000) {
        EXPECT_TRUE(scheduler->stepUntil(SimTime{ns}).ok());
        const SignalBus& bus = scheduler->bus();
        const auto n = static_cast<std::size_t>(bus.read(bus.find("sensor/front_radar/count")));
        const std::span<const double> d = bus.readN(bus.find("sensor/front_radar/detections"));
        std::vector<Detection> out;
        for (std::size_t i = 0; i < n; ++i) {
            const double* f = &d[i * kRadarFields];
            out.push_back({f[0], f[1], f[2], f[3], f[4], f[5]});
        }
        return out;
    }
};

TEST(ImagingRadar, IsABuiltInKind) {
    EXPECT_TRUE(isBuiltinKind("imaging_radar"));
}

// The cell of a uniform array, 1/(N·d) in direction sine: λ/(N·d) rad at boresight (Rao, TI 2017).
TEST(ImagingRadar, AngleCellIsLambdaOverTheAperture) {
    const double d = kLambda / 2.0;  // m, half-wavelength spacing
    for (const double n : {4.0, 12.0, 48.0, 192.0}) {
        EXPECT_NEAR(arrayAngleCell(n, d / kLambda), kLambda / (n * d), 1e-15) << n;
    }
    EXPECT_NEAR(arrayAngleCell(16.0, 0.5), 0.125, 1e-15);
}

TEST(ImagingRadar, FoldsVelocityIntoTheUnambiguousInterval) {
    EXPECT_DOUBLE_EQ(foldVelocity(10.0, 6.0), -2.0);
    EXPECT_DOUBLE_EQ(foldVelocity(-10.0, 6.0), 2.0);
    EXPECT_DOUBLE_EQ(foldVelocity(5.0, 6.0), 5.0);
    EXPECT_DOUBLE_EQ(foldVelocity(6.0, 6.0), -6.0);  // [−v_max, v_max)
    EXPECT_DOUBLE_EQ(foldVelocity(25.0, 6.0), 1.0);  // two turns: 25 − 24
    EXPECT_DOUBLE_EQ(foldVelocity(25.0, 0.0), 25.0);
}

// A plate's σ = ρ·A in one cell, and its SNR by the radar equation P·G²·λ²·σ/((4π)³·R⁴·N).
TEST(ImagingRadar, ThePlatesSnrIsTheRadarEquations) {
    Rig r({{"azimuth_elements", 4.0},
           {"elevation_elements", 4.0},
           {"ray_spacing", 0.005},
           {"noise_floor", 1e-15}});
    r.plate("plate", 20.0, 0.0, 1.0);
    ASSERT_TRUE(r.start());
    const auto d = r.scan();
    ASSERT_EQ(d.size(), 1U);
    EXPECT_NEAR(d[0].rcs, kDensity * 2.0, 0.03 * kDensity * 2.0);
    EXPECT_NEAR(d[0].range, 19.99, 0.05);
    EXPECT_NEAR(d[0].azimuth, 0.0, 0.01);
    EXPECT_NEAR(d[0].elevation, 0.0, 0.01);
    EXPECT_NEAR(d[0].snr, radarSnr(kPower, kGain, kFrequency, d[0].rcs, 19.99, 1e-15),
                0.01 * d[0].snr);
}

TEST(ImagingRadar, DetectsUpToTheRangeTheEquationGives) {
    // The noise floor for which σ = 0.2 m² reaches the threshold (10) at 25 m.
    const double noise = radarSnr(kPower, kGain, kFrequency, 0.2, 25.0, 1.0) / 10.0;
    for (const auto& [range, detected] : {std::pair{22.0, true}, std::pair{28.0, false}}) {
        Rig r({{"azimuth_elements", 4.0},
               {"elevation_elements", 4.0},
               {"ray_spacing", 0.005},
               {"noise_floor", noise}});
        r.plate("plate", range, 0.0, 1.0);
        ASSERT_TRUE(r.start());
        EXPECT_EQ(r.scan().size(), detected ? 1U : 0U) << range;
    }
}

/// Two 0.15 m spheres 30 m out at azimuths ±`half`, seen by `elements` azimuth channels.
std::vector<Detection> pairInAzimuth(double half, double elements) {
    Rig r({{"azimuth_elements", elements}, {"elevation_elements", 4.0}, {"ray_spacing", 0.002}});
    for (const double az : {-half, half}) {
        r.sphere(30.0 * std::cos(az), 30.0 * std::sin(az), 1.0, 0.15);
    }
    EXPECT_TRUE(r.start());
    return r.scan();
}

// 16 channels at λ/2: cells of 0.125 in sine. Targets 0.08 apart share a cell and merge into one
// detection with both cross-sections; 0.3 apart (2.4 cells) they are two; 64 channels (cells of
// 0.031) tell the first pair apart.
TEST(ImagingRadar, TwoTargetsInsideTheAngleCellMerge) {
    const auto merged = pairInAzimuth(0.04, 16.0);
    ASSERT_EQ(merged.size(), 1U);
    EXPECT_NEAR(merged[0].azimuth, 0.0, 0.005);
    const auto apart = pairInAzimuth(0.15, 16.0);
    ASSERT_EQ(apart.size(), 2U);
    EXPECT_NEAR(merged[0].rcs, apart[0].rcs + apart[1].rcs, 0.05 * merged[0].rcs);
    EXPECT_NEAR(std::abs(apart[0].azimuth), 0.15, 0.01);
    EXPECT_NEAR(apart[0].azimuth + apart[1].azimuth, 0.0, 0.02);
    EXPECT_EQ(pairInAzimuth(0.04, 64.0).size(), 2U);
}

/// Two 0.15 m spheres 30 m out, 3.5 m above and below the radar.
std::vector<Detection> pairInElevation(double elements) {
    Rig r({{"azimuth_elements", 16.0},
           {"elevation_elements", elements},
           {"elevation_fov", 0.4},
           {"ray_spacing", 0.002}});
    r.sphere(30.0, 0.0, 1.0 + 3.5, 0.15);
    r.sphere(30.0, 0.0, 1.0 - 3.5, 0.15);
    EXPECT_TRUE(r.start());
    return r.scan();
}

// The fourth dimension: 16 elevation channels tell apart targets 0.23 rad apart in elevation
// (cells of 0.125 in sine) and place them at their heights; one elevation channel cannot.
TEST(ImagingRadar, ResolvesElevation) {
    const auto two = pairInElevation(16.0);
    ASSERT_EQ(two.size(), 2U);
    for (const Detection& d : two) {
        EXPECT_NEAR(std::abs(d.range * std::sin(d.elevation)), 3.5, 0.1);
    }
    EXPECT_LT(two[0].elevation * two[1].elevation, 0.0);
    EXPECT_EQ(pairInElevation(1.0).size(), 1U);
}

// A plate receding at 10 m/s, seen with chirps 1/24 λ apart in time: unambiguous to
// λ/(4·T_c) = 6 m/s, so it reads 10 − 12 = −2 m/s; without folding, 10 m/s.
TEST(ImagingRadar, DopplerFoldsAtTheUnambiguousVelocity) {
    for (const auto& [extra, expected] :
         {std::pair{Settings{{"chirp_period", kLambda / 24.0}}, -2.0},
          std::pair{Settings{{"max_velocity", 6.0}}, -2.0}, std::pair{Settings{}, 10.0}}) {
        Settings s{{"azimuth_elements", 4.0}, {"elevation_elements", 4.0}, {"ray_spacing", 0.005}};
        s.insert(extra.begin(), extra.end());
        Rig r(s);
        r.plate("plate", 20.0, 0.0, 1.0, {10.0, 0.0, 0.0});
        ASSERT_TRUE(r.start());
        const auto d = r.scan();
        ASSERT_EQ(d.size(), 1U);
        EXPECT_NEAR(d[0].velocity, expected, 0.02);
    }
}

// Two targets in one range and angle cell, one still and one closing at 3 m/s: two detections,
// told apart by Doppler alone.
TEST(ImagingRadar, DopplerSeparatesTargetsInOneAngleCell) {
    Rig r({{"azimuth_elements", 4.0}, {"elevation_elements", 4.0}, {"ray_spacing", 0.002}});
    r.sphere(30.0, 0.5, 1.0, 0.15);
    r.movingSphere("car", 30.0, -0.5, 1.0, 0.15, {-3.0, 0.0, 0.0});
    ASSERT_TRUE(r.start());
    auto d = r.scan();
    ASSERT_EQ(d.size(), 2U);
    std::sort(d.begin(), d.end(),
              [](const Detection& a, const Detection& b) { return a.velocity < b.velocity; });
    EXPECT_NEAR(d[0].velocity, -3.0, 0.05);
    EXPECT_NEAR(d[1].velocity, 0.0, 0.05);
}

// Range folding: a plate at 20 m read by a radar unambiguous to 15 m shows at 5 m, its SNR still
// that of 20 m.
TEST(ImagingRadar, RangesFoldAtTheUnambiguousRange) {
    Rig r({{"azimuth_elements", 4.0},
           {"elevation_elements", 4.0},
           {"ray_spacing", 0.005},
           {"unambiguous_range", 15.0},
           {"noise_floor", 1e-15}});
    r.plate("plate", 20.0, 0.0, 1.0);
    ASSERT_TRUE(r.start());
    const auto d = r.scan();
    ASSERT_EQ(d.size(), 1U);
    EXPECT_NEAR(d[0].range, 4.99, 0.05);
    EXPECT_NEAR(d[0].snr, radarSnr(kPower, kGain, kFrequency, d[0].rcs, 19.99, 1e-15),
                0.01 * d[0].snr);
}

/// The plate 20 m out and the radar both 3 m over the ground, ghosts at the ground's |Γ|²
/// `reflection`.
std::vector<Detection> overTheGround(double reflection) {
    Rig r({{"azimuth_elements", 8.0},
           {"elevation_elements", 32.0},
           {"elevation_fov", 0.7},
           {"azimuth_fov", 0.3},
           {"ray_spacing", 0.002},
           {"ghost_reflection", reflection}},
          3.0);
    r.ground();
    r.plate("plate", 20.0, 0.0, 3.0, {-4.0, 0.0, 0.0});
    EXPECT_TRUE(r.start());
    return r.scan();
}

// The ground's mirror ghost: below the horizon, at the mirror image's range √(R² + (2h)²), with
// |Γ|⁴ of the plate's cross-section and its Doppler; none without reflection.
TEST(ImagingRadar, GhostsBelowTheGroundAtGammaToTheFourth) {
    double direct = 0.0;
    double ghost = 0.0;
    for (const Detection& d : overTheGround(0.5)) {
        if (d.elevation < -0.15) {
            ghost += d.rcs;
            // The image spans 5.5..6.5 m below the radar: √(19.99² + 5.5²) .. √(19.99² + 6.5²).
            EXPECT_GT(d.range, 20.7);
            EXPECT_LT(d.range, 21.05);
            EXPECT_NEAR(d.velocity, -4.0 * std::cos(d.elevation), 0.1);
        } else {
            direct += d.rcs;
            EXPECT_NEAR(d.velocity, -4.0, 0.1);
        }
    }
    EXPECT_NEAR(direct, kDensity * 2.0, 0.03 * kDensity * 2.0);
    EXPECT_NEAR(ghost, 0.25 * direct, 0.02 * direct);
    for (const Detection& d : overTheGround(0.0)) {
        EXPECT_GT(d.elevation, -0.15);
    }
}

TEST(ImagingRadar, RefusesSettingsItCannotUse) {
    for (const Settings& bad :
         {Settings{{"ghost_reflection", 1.5}}, Settings{{"max_detections", 0.0}},
          Settings{{"element_spacing", 0.0}}, Settings{{"max_velocity", -1.0}},
          Settings{{"ray_spacing", 1e-6}}}) {
        Rig r(bad);
        EXPECT_FALSE(r.start()) << bad.begin()->first;
    }
}

/// A street: a plate closing in, spheres and a box beside it, ghosts over the ground.
std::unique_ptr<Rig> street(const std::shared_ptr<gpu::RayContext>& rayContext) {
    auto r = std::make_unique<Rig>(Settings{{"azimuth_elements", 24.0},
                                            {"elevation_elements", 12.0},
                                            {"azimuth_fov", 1.2},
                                            {"elevation_fov", 0.4},
                                            {"ray_spacing", 0.006},
                                            {"max_velocity", 8.0},
                                            {"ghost_reflection", 0.3},
                                            {"noise_floor", 1e-15}});
    r->ground();
    r->plate("plate", 20.0, 0.0, 1.0, {-5.0, 0.0, 0.0});
    r->sphere(12.0, 2.5, 1.0, 0.6);
    r->sphere(25.0, -6.0, 0.8, 0.5);
    Pose crate;
    crate.position = {30.0, -4.0, 1.2};
    crate.orientation = {std::cos(0.3), 0.0, 0.0, std::sin(0.3)};
    r->world.addCollider(kWorldBody, ShapeType::Box, {0.8, 0.8, 0.8}, crate, 0.8);
    EXPECT_TRUE(r->start(rayContext));
    return r;
}

// Deterministic: the CPU reference twice gives the same bits.
TEST(ImagingRadar, TheCpuReferenceRepeatsBitForBit) {
    auto a = street(nullptr);
    auto b = street(nullptr);
    for (const std::int64_t t : {120'000'000, 520'000'000}) {
        const auto x = a->scan(t);
        const auto y = b->scan(t);
        ASSERT_EQ(x.size(), y.size());
        ASSERT_GE(x.size(), 4U);
        EXPECT_EQ(std::memcmp(x.data(), y.data(), x.size() * sizeof(Detection)), 0);
    }
}

const std::shared_ptr<gpu::RayContext>& context() {
    static const std::shared_ptr<gpu::RayContext> made = [] {
        const char* preferred = std::getenv("LAGRANGE_TEST_GPU");
        std::string error;
        auto created = gpu::RayContext::create(preferred != nullptr ? preferred : "", error);
        if (!created) {
            std::fprintf(stderr, "no Vulkan device for the GPU radar: %s\n", error.c_str());
        }
        return created;
    }();
    return made;
}

// The GPU rays (float ranges) find what the CPU reference finds. An echo within float rounding
// of a cell's edge may fall in the next cell, so each CPU detection is matched to the nearest GPU
// one: the strong ones within 5 cm, 5 mrad, 0.1 m/s and 5 % of σ, and the totals within 1 %.
TEST(ImagingRadar, OnTheGpuAgreesWithTheCpuReference) {
    if (!context()) {
        GTEST_SKIP() << "no Vulkan device";
    }
    auto cpu = street(nullptr);
    auto gpu = street(context());
    for (const std::int64_t t : {120'000'000, 520'000'000}) {
        const auto x = cpu->scan(t);
        const auto y = gpu->scan(t);
        ASSERT_GE(x.size(), 4U);
        EXPECT_NEAR(static_cast<double>(y.size()), static_cast<double>(x.size()),
                    0.1 * static_cast<double>(x.size()) + 1.0);
        double sx = 0.0, sy = 0.0;
        for (const Detection& d : x) {
            sx += d.rcs;
        }
        for (const Detection& d : y) {
            sy += d.rcs;
        }
        EXPECT_NEAR(sy, sx, 0.01 * sx);
        for (std::size_t i = 0; i < std::min<std::size_t>(x.size(), 4); ++i) {
            const Detection& a = x[i];
            const auto near =
                std::min_element(y.begin(), y.end(), [&a](const auto& p, const auto& q) {
                    return std::hypot(p.range - a.range, 30.0 * (p.azimuth - a.azimuth),
                                      30.0 * (p.elevation - a.elevation)) <
                           std::hypot(q.range - a.range, 30.0 * (q.azimuth - a.azimuth),
                                      30.0 * (q.elevation - a.elevation));
                });
            ASSERT_NE(near, y.end());
            EXPECT_NEAR(near->range, a.range, 0.05) << i;
            EXPECT_NEAR(near->azimuth, a.azimuth, 5e-3) << i;
            EXPECT_NEAR(near->elevation, a.elevation, 5e-3) << i;
            EXPECT_NEAR(near->velocity, a.velocity, 0.1) << i;
            EXPECT_NEAR(near->rcs, a.rcs, 0.05 * a.rcs) << i;
        }
    }
}

}  // namespace
}  // namespace lagrange::sensors
