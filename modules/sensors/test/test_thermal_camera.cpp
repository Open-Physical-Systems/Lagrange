// The thermal camera (sensors.md §2, "Thermal camera"): Planck's law over its band against the
// black-body fractions of Incropera et al. and a direct integration, Stefan-Boltzmann over all
// wavelengths, the apparent temperature of a grey surface, Swinbank's clear sky, and a run that
// sees a hot thermal node, the ambient air and the sky.
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ThermalCamera.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numbers>
#include <optional>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::sensors {
namespace {

using namespace literals;

/// Planck's B_λ, W/(m²·sr·m).
double planck(double lambda, double t) {
    const double c1 = 2.0 * kPlanck * kLightSpeed * kLightSpeed;
    return c1 / std::pow(lambda, 5) / std::expm1(kPlanck * kLightSpeed / (lambda * kBoltzmann * t));
}

/// Composite Simpson over [a, b] with n (even) intervals.
double simpson(double a, double b, int n, double t) {
    const double h = (b - a) / n;
    double sum = planck(a, t) + planck(b, t);
    for (int i = 1; i < n; ++i) {
        sum += (i % 2 == 1 ? 4.0 : 2.0) * planck(a + i * h, t);
    }
    return sum * h / 3.0;
}

TEST(ThermalCamera, TheBlackBodyFractionsAreIncroperasTable) {
    // Incropera, DeWitt, Bergman and Lavine, Fundamentals of Heat and Mass Transfer, Table 12.1,
    // F(0→λT) at λT in µm·K. The table's C2 is 14,388 µm·K against CODATA's 14,387.77, which
    // moves the sixth decimal; hence 1e-5.
    EXPECT_NEAR(blackBodyFraction(2200e-6), 0.100888, 1e-5);
    EXPECT_NEAR(blackBodyFraction(2800e-6), 0.227897, 1e-5);
    EXPECT_NEAR(blackBodyFraction(2898e-6), 0.250108, 1e-5);
    EXPECT_NEAR(blackBodyFraction(3000e-6), 0.273232, 1e-5);
    // An 8–14 µm camera sees 37.6 % of what a 300 K body radiates (the fractions at 4200 and
    // 2400 µm·K).
    const double total = kStefanBoltzmann * std::pow(300.0, 4) / std::numbers::pi;
    EXPECT_NEAR(bandRadiance(300.0, 8e-6, 14e-6) / total,
                blackBodyFraction(4200e-6) - blackBodyFraction(2400e-6), 1e-9);
    EXPECT_NEAR(bandRadiance(300.0, 8e-6, 14e-6) / total, 0.3757, 1e-4);
}

TEST(ThermalCamera, TheBandRadianceIsPlancksIntegral) {
    for (const double t : {250.0, 300.0, 373.15, 900.0}) {
        const double exact = simpson(8e-6, 14e-6, 2000, t);
        EXPECT_NEAR(bandRadiance(t, 8e-6, 14e-6) / exact, 1.0, 1e-9) << t;
    }
    // Over all wavelengths, Stefan-Boltzmann: σT⁴/π.
    for (const double t : {50.0, 300.0, 2500.0}) {
        EXPECT_NEAR(
            bandRadiance(t, 0.0, 1.0) * std::numbers::pi / (kStefanBoltzmann * t * t * t * t), 1.0,
            1e-6)
            << t;
    }
}

TEST(ThermalCamera, ItReadsTheApparentTemperatureOfAGreySurface) {
    const ThermalBand band(8e-6, 14e-6);
    for (const double t : {200.0, 293.15, 310.0, 600.0}) {
        EXPECT_NEAR(band.temperatureOf(band.radiance(t)), t, 1e-6) << t;
    }
    // A black body reads its own temperature; anything at the surroundings' reads those.
    EXPECT_NEAR(band.apparent(350.0, 1.0, 293.15), 350.0, 1e-6);
    EXPECT_NEAR(band.apparent(293.15, 0.3, 293.15), 293.15, 1e-6);
    // A grey one between: ε·B(T) + (1−ε)·B(T_surr) = B(T_app), checked by direct integration.
    const double app = band.apparent(350.0, 0.5, 293.15);
    EXPECT_GT(app, 293.15);
    EXPECT_LT(app, 350.0);
    EXPECT_NEAR(simpson(8e-6, 14e-6, 2000, app),
                0.5 * simpson(8e-6, 14e-6, 2000, 350.0) + 0.5 * simpson(8e-6, 14e-6, 2000, 293.15),
                1e-6 * simpson(8e-6, 14e-6, 2000, app));
    // Skin (Fanger, 2 met; ε 0.98) at 20 °C surroundings reads a few tenths below 32.5 °C.
    EXPECT_NEAR(kSkinTemperature, 305.66, 0.01);
    EXPECT_NEAR(band.apparent(kSkinTemperature, kSkinEmissivity, 293.15), kSkinTemperature, 0.3);
}

TEST(ThermalCamera, TheClearSkyIsSwinbanks) {
    // T_sky = 0.0552·T^1.5: 20 °C air gives a sky 16 K colder.
    EXPECT_NEAR(clearSkyTemperature(293.15), 0.0552 * std::pow(293.15, 1.5), 1e-9);
    EXPECT_NEAR(clearSkyTemperature(293.15), 277.06, 0.01);
}

TEST(ThermalCamera, ThePalettesRunFromColdToHot) {
    EXPECT_EQ(paletteColour(ThermalPalette::WhiteHot, 0.0)[0], 0);
    EXPECT_EQ(paletteColour(ThermalPalette::WhiteHot, 1.0)[0], 255);
    EXPECT_EQ(paletteColour(ThermalPalette::BlackHot, 1.0)[0], 0);
    const auto cold = paletteColour(ThermalPalette::Ironbow, 0.0);
    const auto hot = paletteColour(ThermalPalette::Ironbow, 1.0);
    EXPECT_LT(cold[0] + cold[1] + cold[2], hot[0] + hot[1] + hot[2]);
}

/// Writes a thermal node's temperature.
class Heater final : public IModule {
public:
    explicit Heater(double kelvin) : kelvin_(kelvin) {}
    [[nodiscard]] ModuleInfo info() const override { return {"thermal", "heater"}; }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec spec;
        spec.name = "thermal/heater/temperature";
        spec.unit = units::K;
        id_ = d.output(spec);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(id_, kelvin_);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(id_, kelvin_);
        return {};
    }

private:
    double kelvin_;
    SignalId id_;
};

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;
constexpr double kAmbient = 293.15;
constexpr double kHot = 350.0;
constexpr double kEmissivity = 0.95;
constexpr double kNetd = 0.05;

/// A thermal camera 1 m up looking along +x at a heater (a 1 m cube 3 m ahead with a thermal
/// node) and, to its left, a cube with none.
Model scene() {
    Model model;
    World& w = model.world;
    const BodyId post = w.addBody("post", kWorldBody, Pose{}, 0.0, {}, {});
    w.addSensor("ir", SensorType::Camera, post, {{0, 0, 1}}, 30.0,
                {{"width", kWidth},
                 {"height", kHeight},
                 {"horizontal_fov", 1.2},
                 {"thermal", 1.0},
                 {"thermal_netd", kNetd}});
    SchemaInstance node;
    node.schema = "ThermalAPI_1";
    node.prim = w.addPrim("/rig/heater", "heater");
    node.reals["thermal:emissivity"] = kEmissivity;
    node.reals["thermal:heatCapacity"] = 100.0;
    w.schemaInstances.push_back(node);
    model.scene.meshes.push_back(boxMesh(1.0, 1.0, 1.0));
    ScenePrim cube;
    cube.name = "heater";
    cube.path = "/rig/heater";
    cube.mesh = 0;
    cube.offset = {{3, 0, 1}};
    model.scene.prims.push_back(cube);
    ScenePrim left = cube;
    left.name = "crate";
    left.path = "/rig/crate";
    left.offset = {{3, 1.5, 1}};
    left.scale = {0.3F, 0.3F, 0.3F};
    model.scene.prims.push_back(left);
    return model;
}

float at(const Buffer& buffer, std::uint32_t u, std::uint32_t v) {
    float out = 0.0F;
    std::memcpy(&out, buffer.bytes.data() + (std::size_t{v} * buffer.width + u) * 4, 4);
    return out;
}

TEST(ThermalCamera, ItSeesAHotNodeTheAirAndTheSky) {
    const Model model = scene();
    std::string error;
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    const std::shared_ptr<CameraScene> cameras =
        createCameraScene(model, gpu != nullptr ? gpu : "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler{SchedulerOptions{}};
    scheduler.add(std::make_unique<Heater>(kHot));
    ModuleConfigData config;
    config.values["ambient_temperature"] = kAmbient;
    for (auto& module : createSensorModules(model.world, nullptr, cameras)) {
        scheduler.add(std::move(module), ModuleConfig(config));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    for (int i = 0; i < 40; ++i) {
        ASSERT_TRUE(scheduler.stepTick().ok());
    }
    const SignalBus& bus = scheduler.bus();
    const auto kelvin = bus.buffer(bus.readBuffer(bus.find("sensor/ir/temperature")));
    const auto image = bus.buffer(bus.readBuffer(bus.find("sensor/ir/image")));
    ASSERT_TRUE(kelvin && image);
    ASSERT_EQ(kelvin->encoding, "r32f");
    ASSERT_EQ(kelvin->bytes.size(), std::size_t{kWidth} * kHeight * 4);

    // The mean over a patch, so the noise averages out; and the noise itself.
    const auto patch = [&](std::uint32_t u0, std::uint32_t v0) {
        double sum = 0.0;
        double square = 0.0;
        int n = 0;
        for (std::uint32_t v = v0 - 3; v <= v0 + 3; ++v) {
            for (std::uint32_t u = u0 - 3; u <= u0 + 3; ++u) {
                const double t = at(*kelvin, u, v);
                sum += t;
                square += t * t;
                ++n;
            }
        }
        const double mean = sum / n;
        return std::pair{mean, std::sqrt(square / n - mean * mean)};
    };
    const ThermalBand band(8e-6, 14e-6);
    const double hot = band.apparent(kHot, kEmissivity, kAmbient);
    const auto [centre, noise] = patch(kWidth / 2, kHeight / 2);
    EXPECT_NEAR(centre, hot, 4.0 * kNetd / 7.0);
    EXPECT_NEAR(noise, kNetd, 0.4 * kNetd);
    EXPECT_GT(hot, kHot - 3.0);  // ε 0.95 reads about 2.6 K low
    // The crate on the left (u = cx − fx·1.5/2.85) is at the air's temperature.
    const double fx = kWidth / 2.0 / std::tan(0.6);
    const auto u = static_cast<std::uint32_t>(std::lround((kWidth - 1) / 2.0 - fx * 1.5 / 2.85));
    EXPECT_NEAR(patch(u, kHeight / 2).first, kAmbient, 0.1);
    // Above the heater, the clear sky.
    EXPECT_NEAR(patch(kWidth / 2, 8).first, clearSkyTemperature(kAmbient), 0.1);
    // White hot: the heater is the brightest thing in view.
    const auto red = [&](std::uint32_t uu, std::uint32_t vv) {
        return image->bytes[(std::size_t{vv} * kWidth + uu) * 4];
    };
    EXPECT_GT(static_cast<int>(red(kWidth / 2, kHeight / 2)), 200);
    EXPECT_LT(static_cast<int>(red(kWidth / 2, 8)), 50);
}

// RFC 0012: a link with a thermal material reads at its temperature (thermal/<link>/temperature,
// the conduction fidelity's mean) with its material's emissivity: bare aluminium (0.09) reads
// far colder than it is, as through a real thermal camera; painted (0.9) close to it.
TEST(ThermalCamera, ALinkWithAThermalMaterialReadsWithItsEmissivity) {
    const auto centreOf = [](std::optional<double> emissivity) {
        Model model = scene();
        World& w = model.world;
        w.schemaInstances.clear();  // the heater is a link with a material, not a node
        const BodyId block = w.addBody("heater", kWorldBody, Pose{{3, 0, 1}, {1, 0, 0, 0}}, 1.0, {},
                                       {0.1, 0.1, 0.1, 0, 0, 0});
        SchemaInstance m;
        m.schema = "ThermalMaterialAPI_1";
        m.prim = w.bodies.prim[block.index];
        m.tokens = {{"thermal:material:preset", "aluminium_6061"}};
        if (emissivity) {
            m.reals = {{"thermal:material:emissivity", *emissivity}};
        }
        w.schemaInstances.push_back(m);
        model.scene.prims[0].path = w.prims.paths[m.prim.index];
        model.scene.prims[0].body = block.index;
        model.scene.prims[0].offset = Pose{};
        std::string error;
        const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
        const std::shared_ptr<CameraScene> cameras =
            createCameraScene(model, gpu != nullptr ? gpu : "", error);
        if (!cameras) {
            return std::optional<double>{};
        }
        Scheduler scheduler{SchedulerOptions{}};
        scheduler.add(std::make_unique<Heater>(kHot));
        ModuleConfigData config;
        config.values["ambient_temperature"] = kAmbient;
        for (auto& module : createSensorModules(model.world, nullptr, cameras)) {
            scheduler.add(std::move(module), ModuleConfig(config));
        }
        auto wired = scheduler.wire(WorldView(model.world));
        EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
        for (int i = 0; i < 40; ++i) {
            EXPECT_TRUE(scheduler.stepTick().ok());
        }
        const SignalBus& bus = scheduler.bus();
        const auto kelvin = bus.buffer(bus.readBuffer(bus.find("sensor/ir/temperature")));
        double sum = 0.0;
        int n = 0;
        for (std::uint32_t v = kHeight / 2 - 3; v <= kHeight / 2 + 3; ++v) {
            for (std::uint32_t u = kWidth / 2 - 3; u <= kWidth / 2 + 3; ++u) {
                sum += at(*kelvin, u, v);
                ++n;
            }
        }
        return std::optional<double>{sum / n};
    };
    const auto bare = centreOf(std::nullopt);
    if (!bare) {
        GTEST_SKIP() << "no Vulkan device";
    }
    const auto painted = centreOf(0.9);
    const ThermalBand band(8e-6, 14e-6);
    EXPECT_NEAR(*bare, band.apparent(kHot, 0.09, kAmbient), 0.1);
    EXPECT_NEAR(*painted, band.apparent(kHot, 0.9, kAmbient), 0.1);
    EXPECT_LT(*bare, kAmbient + 10.0);  // 350 K bare aluminium reads near the room's
    EXPECT_GT(*painted, kHot - 8.0);
}

TEST(ThermalCamera, ItRefusesToStartWithoutTheSitesTemperature) {
    const Model model = scene();
    std::string error;
    const std::shared_ptr<CameraScene> cameras = createCameraScene(model, "", error);
    if (!cameras) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    Scheduler scheduler;
    for (auto& module : createSensorModules(model.world, nullptr, cameras)) {
        scheduler.add(std::move(module));
    }
    auto wired = scheduler.wire(WorldView(model.world));
    ASSERT_FALSE(wired);
    EXPECT_NE(to_string(wired.error().front()).find("ambient temperature"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::sensors
