// Smoke (RFC 0022 §4, §Validation) against closed forms: the extinction of a soot density, a
// single puff's peak, a steady line of puffs in wind (a Gaussian line source), the rise at
// Heskestad's plume velocity, a ceiling, the mass kept in the field and lost from the box, the
// merging of the oldest puffs, save and restore, and the module's field on the bus.
#include "lagrange/fire/Correlations.hpp"
#include "lagrange/fire/Fires.hpp"
#include "lagrange/fire/Smoke.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::fire {
namespace {

using namespace literals;

/// The integral of the extinction field over its voxels over K_m: the soot it holds (kg).
double fieldMass(const fields::Field& f, const std::array<Vec3, 2>& box, double voxel) {
    double sum = 0.0;
    for (double x = box[0][0]; x <= box[1][0] + 1e-9; x += voxel) {
        for (double y = box[0][1]; y <= box[1][1] + 1e-9; y += voxel) {
            for (double z = box[0][2]; z <= box[1][2] + 1e-9; z += voxel) {
                sum += f.sampleScalar({x, y, z});
            }
        }
    }
    return sum * voxel * voxel * voxel / kSpecificExtinction;
}

/// One puff of `mass` at `centre`, σ across and up, through restore().
Smoke onePuff(const SmokeSettings& settings, const Vec3& centre, double mass, double sigmaH,
              double sigmaV) {
    Smoke smoke;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    smoke.configure(settings, base);
    const std::vector<double> state{0.0,    1.0,    centre[0], centre[1], centre[2], mass,
                                    sigmaH, sigmaV, 0.0,       0.0,       1e300};
    EXPECT_TRUE(smoke.restore(state));
    return smoke;
}

TEST(Smoke, ExtinctionIsTheSpecificExtinctionTimesTheSootDensity) {
    EXPECT_DOUBLE_EQ(kSpecificExtinction, 8700.0);  // m²/kg, Mulholland and Croarkin 2000
    EXPECT_DOUBLE_EQ(smokeExtinction(1e-4), 0.87);
    EXPECT_EQ(smokeExtinction(-1.0), 0.0);
}

// A single Gaussian puff: its peak σ is K_m·m/((2π)^(3/2)·σ_h²·σ_v), 1 %, and the field holds
// its mass, 0.1 %.
TEST(Smoke, ASinglePuffsPeakAndMass) {
    SmokeSettings settings;
    settings.voxel = 0.25;
    const Vec3 centre{2.0, -1.0, 4.0};
    const double mass = 2e-3;
    Smoke smoke = onePuff(settings, centre, mass, 1.5, 1.0);
    const auto field = smoke.raster();
    ASSERT_TRUE(field);
    const double peak =
        kSpecificExtinction * mass / (std::pow(2.0 * std::numbers::pi, 1.5) * 1.5 * 1.5 * 1.0);
    EXPECT_NEAR(field->sampleScalar(centre), peak, 0.01 * peak);
    // One σ across: e^(−1/2) of the peak.
    EXPECT_NEAR(field->sampleScalar({centre[0] + 1.5, centre[1], centre[2]}), peak * std::exp(-0.5),
                0.02 * peak);
    EXPECT_NEAR(fieldMass(*field, smoke.box(), settings.voxel), mass, 1e-3 * mass);
}

// A steady source in a wind u with no rise: the puffs line up downwind, and away from both ends
// the line's axis holds K_m·ṁ/(2π·u·σ_h·σ_v) (a Gaussian line source), 2 %.
TEST(Smoke, ASteadySourceInWindIsAGaussianLine) {
    SmokeSettings settings;
    settings.voxel = 0.25;
    Smoke smoke;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    smoke.configure(settings, base);
    // σ at the tip: 0.12/√(2 ln 2)·(h − z0); h chosen for σ = 1 m with z0 = 0.
    const double h = std::sqrt(2.0 * std::numbers::ln2) / 0.12;
    PlumeSource f;
    f.tip = {0.0, 0.0, h};
    f.hrr = 1e6;
    f.radiativeFraction = 0.3;
    f.sootRate = 1e-3;  // kg/s
    SmokeAir air;
    air.wind = {2.0, 0.0, 0.0};      // m/s; no temperature or density: no rise
    for (int i = 0; i < 200; ++i) {  // 20 s: the line reaches 40 m, past the box's 30 m
        smoke.step(0.1, std::span<const PlumeSource>(&f, 1), air, nullptr);
    }
    ASSERT_FALSE(smoke.puffs().empty());
    EXPECT_NEAR(smoke.puffs().back().sigmaH, 1.0, 1e-9);
    const auto field = smoke.raster();
    ASSERT_TRUE(field);
    const double expected = kSpecificExtinction * 1e-3 / (2.0 * std::numbers::pi * 2.0 * 1.0 * 1.0);
    for (const double x : {8.0, 15.0, 22.0}) {
        EXPECT_NEAR(field->sampleScalar({x, 0.0, h}), expected, 0.02 * expected) << x;
    }
    // What left the box went downwind past 30 m: all of it, and nothing else.
    EXPECT_NEAR(smoke.mass() + smoke.leftMass(), 1e-3 * 20.0, 1e-12);
    EXPECT_GT(smoke.leftMass(), 0.0);
}

// A puff rises at Heskestad's centreline velocity u0 = A·(z − z0)^(−1/3): (z − z0)^(4/3) grows
// as (4/3)·A·t. Explicit steps of 10 ms, 0.5 %.
TEST(Smoke, APuffRisesAtThePlumeVelocity) {
    SmokeSettings settings;
    settings.height = 100.0;
    Smoke smoke;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    smoke.configure(settings, base);
    PlumeSource f;
    f.hrr = 1e6;
    f.radiativeFraction = 0.3;
    f.virtualOriginHeight = virtualOrigin(1e6, 1.0);
    f.tip = {0.0, 0.0, 3.0};
    f.sootRate = 1.0;
    SmokeAir air;
    air.gravity = 9.81;
    air.temperature = 293.15;
    air.density = 1.2;
    const double dt = 0.01;
    smoke.step(dt, std::span<const PlumeSource>(&f, 1), air, nullptr);  // emits one puff at 3 m
    f.sootRate = 0.0;
    const double t = 2.0;
    for (int i = 0; i < 200; ++i) {
        smoke.step(dt, std::span<const PlumeSource>(&f, 1), air, nullptr);
    }
    ASSERT_EQ(smoke.puffs().size(), 1U);
    const double z0 = f.virtualOriginHeight;
    // A from the correlation itself at a height of 1 m above z0.
    const double a = plumeVelocity(1e6, 0.3, z0 + 1.0, z0, 9.81, 1005.0, 293.15, 1.2);
    const double z = z0 + std::pow(std::pow(3.0 - z0, 4.0 / 3.0) + (4.0 / 3.0) * a * t, 0.75);
    EXPECT_NEAR(smoke.puffs()[0].centre[2], z, 0.005 * (z - 3.0));
    // Its width follows the plume's.
    EXPECT_NEAR(smoke.puffs()[0].sigmaH,
                0.12 / std::sqrt(2.0 * std::numbers::ln2) * (smoke.puffs()[0].centre[2] - z0),
                1e-9);
}

/// A flat roof: the box from (−50, −50, top) to (50, 50, top + 0.2).
class Roof final : public services::SceneQuery {
public:
    explicit Roof(double top) : top_(top) {}
    [[nodiscard]] services::RayHit ray(const std::array<double, 3>& o,
                                       const std::array<double, 3>& d,
                                       double maxDistance) const override {
        services::RayHit hit;
        if (d[2] > 0.0 && o[2] < top_ && std::abs(o[0]) < 50.0 && std::abs(o[1]) < 50.0) {
            const double at = (top_ - o[2]) / d[2];
            if (at <= maxDistance) {
                hit.distance = at;
                hit.shape = 0;
            }
        }
        return hit;
    }
    [[nodiscard]] std::vector<std::uint32_t> overlapSphere(const std::array<double, 3>&,
                                                           double) const override {
        return {};
    }

private:
    double top_;
};

// Under a roof the puffs stop a σ below it and spread across; they keep their mass.
TEST(Smoke, PuffsStopUnderACeilingAndSpread) {
    SmokeSettings settings;
    Smoke smoke;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    smoke.configure(settings, base);
    const Roof roof(6.0);
    PlumeSource f;
    f.hrr = 1e6;
    f.radiativeFraction = 0.3;
    f.virtualOriginHeight = virtualOrigin(1e6, 1.0);
    f.tip = {0.0, 0.0, 2.7};
    f.sootRate = 1e-3;
    SmokeAir air;
    air.temperature = 293.15;
    air.density = 1.2;
    for (int i = 0; i < 100; ++i) {
        smoke.step(0.1, std::span<const PlumeSource>(&f, 1), air, &roof);
    }
    const Puff& first = smoke.puffs().front();
    EXPECT_TRUE(first.underCeiling);
    EXPECT_NEAR(first.centre[2] + first.sigmaV, 6.0, 1e-9);
    EXPECT_GT(first.sigmaH, 2.0 * first.sigmaV);  // spread sideways
    EXPECT_NEAR(smoke.mass(), 1e-3 * 10.0, 1e-12);
    EXPECT_EQ(smoke.leftMass(), 0.0);
    const auto field = smoke.raster();
    ASSERT_TRUE(field);
    // While no puff has left: the field's integral is the soot made, 2 % (RFC 0022 §Validation).
    EXPECT_NEAR(fieldMass(*field, smoke.box(), settings.voxel), 1e-2, 0.02 * 1e-2);
    EXPECT_EQ(field->sampleScalar({0.0, 0.0, 6.5}), 0.0);  // nothing above the roof
}

// Merging keeps mass, centre of mass and second moments.
TEST(Smoke, MergingTheOldestKeepsMassAndMoments) {
    SmokeSettings settings;
    settings.maxPuffs = 2;
    Smoke smoke;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    smoke.configure(settings, base);
    PlumeSource f;
    f.sootRate = 1.0;
    f.tip = {0.0, 0.0, 1.0};
    SmokeAir air;
    air.wind = {1.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        smoke.step(1.0, std::span<const PlumeSource>(&f, 1), air, nullptr);
    }
    // Puffs at x = 2, 1, 0 of 1 kg each; the two oldest are merged at x = 1.5.
    ASSERT_EQ(smoke.puffs().size(), 2U);
    const Puff& merged = smoke.puffs()[0];
    EXPECT_DOUBLE_EQ(merged.mass, 2.0);
    EXPECT_DOUBLE_EQ(merged.centre[0], 1.5);
    const double s = 0.25;  // σ floor: half a 0.5 m voxel
    EXPECT_NEAR(merged.sigmaH, std::sqrt((s * s) + (0.5 * 0.25)), 1e-12);
    EXPECT_NEAR(merged.sigmaV, s, 1e-12);
    EXPECT_DOUBLE_EQ(smoke.mass(), 3.0);
}

TEST(Smoke, SaveAndRestoreCarryOn) {
    SmokeSettings settings;
    Smoke a;
    const std::array<Vec3, 1> base{Vec3{0.0, 0.0, 0.0}};
    a.configure(settings, base);
    PlumeSource f;
    f.hrr = 5e5;
    f.radiativeFraction = 0.3;
    f.tip = {0.0, 0.0, 2.0};
    f.sootRate = 1e-3;
    SmokeAir air;
    air.wind = {0.7, -0.2, 0.0};
    air.temperature = 293.15;
    air.density = 1.2;
    for (int i = 0; i < 50; ++i) {
        a.step(0.1, std::span<const PlumeSource>(&f, 1), air, nullptr);
    }
    std::vector<double> state;
    a.save(state);
    Smoke b;
    b.configure(settings, base);
    ASSERT_TRUE(b.restore(state));
    for (int i = 0; i < 50; ++i) {
        a.step(0.1, std::span<const PlumeSource>(&f, 1), air, nullptr);
        b.step(0.1, std::span<const PlumeSource>(&f, 1), air, nullptr);
    }
    std::vector<double> sa;
    std::vector<double> sb;
    a.save(sa);
    b.save(sb);
    EXPECT_EQ(sa, sb);
    EXPECT_FALSE(b.restore(std::vector<double>{0.0, 3.0, 1.0}));
}

// --- the module ------------------------------------------------------------------------------

/// Reads the smoke field and mass every 100 ms.
class SmokeProbe final : public IModule {
public:
    std::shared_ptr<const fields::Field> field;
    double mass{0.0};
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "smoke", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 100_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec f;
        f.name = "env/smoke/extinction";
        f.type = SignalType::Field3D;
        f.unit = units::per_m;
        field_ = d.input(f);
        SignalSpec m;
        m.name = "env/smoke/mass";
        m.unit = units::kg;
        mass_ = d.input(m);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        field = reader_.read(bus, field_);
        mass = bus.read(mass_);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    SignalId field_, mass_;
    fields::FieldSignal reader_;
};

World burningPallet() {
    World w;
    w.gravityLocal = {0.0, 0.0, -9.81};
    const PrimId p = w.addPrim("/w/pallet", "pallet");
    w.prims.poseInBody.at(p.index).position = {0.0, 0.0, 0.3};
    w.addCollider(kWorldBody, ShapeType::Box, {0.5, 0.5, 0.3}, Pose{{0.0, 0.0, 0.3}, {1, 0, 0, 0}},
                  0.5);
    SchemaInstance s;
    s.schema = "FireAPI_1";
    s.prim = p;
    s.reals = {{"fire:alpha", 46.889},           {"fire:hrrPerArea", 5e5},
               {"fire:fuelMass", 500.0},         {"fire:heatOfCombustion", 1.2e7},
               {"fire:radiativeFraction", 0.3},  {"fire:sootYield", 0.015},
               {"fire:criticalWaterRate", 0.03}, {"fire:ignitionTime", 0.0}};
    w.schemaInstances.push_back(s);
    return w;
}

ModuleConfig burning(Fires::Mode mode = Fires::Mode::Simulate) {
    ModuleConfigData data;
    data.enabled = mode == Fires::Mode::Simulate;
    data.values["air_density"] = 1.2;
    data.values["ambient_temperature"] = 293.15;
    return ModuleConfig(data);
}

// The module's smoke: what the fire made, ∫ y_s·Q/ΔH_c dt, is in the air (no wind, no roof, the
// puffs still within the box), and the field at 2 Hz holds it, 2 %.
TEST(SmokeModule, TheFieldHoldsTheSootTheFireMade) {
    const World world = burningPallet();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Fires>(Fires::Mode::Constant), burning(Fires::Mode::Constant));
    auto probe = std::make_unique<SmokeProbe>();
    SmokeProbe* seen = probe.get();
    scheduler.add(std::move(probe));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000'000}).ok());
    // Constant: 500 kW from the start; soot 0.015·5e5/1.2e7 kg/s for 30 steps (the last puff
    // of the 3 s field at the probe's step is the one emitted then).
    const double rate = 0.015 * 5e5 / 1.2e7;
    EXPECT_NEAR(seen->mass, rate * 3.0, 1e-9);
    ASSERT_TRUE(seen->field);
    std::array<Vec3, 2> box{Vec3{-5.0, -5.0, -1.0}, Vec3{5.0, 5.0, 19.5}};
    EXPECT_NEAR(fieldMass(*seen->field, box, 0.5), rate * 3.0, 0.02 * rate * 3.0);
}

// The zero substitute: no smoke.
TEST(SmokeModule, TheZeroSubstituteMakesNoSmoke) {
    const World world = burningPallet();
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Fires>(Fires::Mode::Zero), burning(Fires::Mode::Zero));
    auto probe = std::make_unique<SmokeProbe>();
    SmokeProbe* seen = probe.get();
    scheduler.add(std::move(probe));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{2'000'000'000}).ok());
    EXPECT_EQ(seen->mass, 0.0);
    EXPECT_FALSE(seen->field);
}

// The same run twice, and a run restored half-way: identical puffs.
TEST(SmokeModule, DeterministicAndRestorable) {
    const World world = burningPallet();
    const auto run = [&](double seconds, std::vector<std::byte>* saveAt, double saveTime,
                         const std::vector<std::byte>* restore) {
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        auto fires = std::make_unique<Fires>();
        Fires* f = fires.get();
        scheduler.add(std::move(fires), burning());
        EXPECT_TRUE(scheduler.wire(WorldView(world)));
        if (restore != nullptr) {
            StateReader reader(*restore);
            EXPECT_TRUE(
                scheduler.stepUntil(SimTime{static_cast<std::int64_t>(saveTime * 1e9)}).ok());
            EXPECT_TRUE(f->restoreState(reader).ok());
        }
        if (saveAt != nullptr) {
            EXPECT_TRUE(
                scheduler.stepUntil(SimTime{static_cast<std::int64_t>(saveTime * 1e9)}).ok());
            StateWriter writer;
            EXPECT_TRUE(f->saveState(writer).ok());
            saveAt->assign(writer.bytes().begin(), writer.bytes().end());
        }
        EXPECT_TRUE(scheduler.stepUntil(SimTime{static_cast<std::int64_t>(seconds * 1e9)}).ok());
        std::vector<double> state;
        f->smoke().save(state);
        return state;
    };
    std::vector<std::byte> saved;
    const std::vector<double> a = run(20.0, &saved, 10.0, nullptr);
    EXPECT_EQ(run(20.0, nullptr, 0.0, nullptr), a);
    EXPECT_EQ(run(20.0, nullptr, 10.0, &saved), a);
    EXPECT_GT(a.size(), 100U);
}

}  // namespace
}  // namespace lagrange::fire
