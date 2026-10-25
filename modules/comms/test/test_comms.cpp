// The comms domain (domains/comms.md §7): Friis, the thermal noise floor, the bit-error curves,
// CAN fault confinement, and the module end to end with a wall and a noisy CAN bus.
#include "lagrange/comms/Comms.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::comms {
namespace {

double decibels(double x) {
    return 10.0 * std::log10(x);
}

TEST(CommsBudget, FriisAndTheNoiseFloor) {
    // The textbook form: FSPL(dB) = 20·log10(d/km) + 20·log10(f/MHz) + 32.45 (within 0.01 dB of
    // the exact constant, 20·log10(4π·1e9/c) = 32.4478).
    const double fspl = decibels(freeSpacePathLoss(1000.0, 2.4e9));
    EXPECT_NEAR(fspl, 20.0 * std::log10(2400.0) + 32.4478, 1e-3);
    // At λ = 4π m and 1 m the loss is 1, exactly.
    EXPECT_NEAR(freeSpacePathLoss(1.0, kSpeedOfLight / (4.0 * std::numbers::pi)), 1.0, 1e-12);
    // Log-distance: n = 3 beyond d0 = 1 m adds 30 dB per decade.
    EXPECT_NEAR(
        decibels(pathLoss(100.0, 2.4e9, 1.0, 3.0)) - decibels(pathLoss(10.0, 2.4e9, 1.0, 3.0)),
        30.0, 1e-9);
    // k·T0·B: −101 dBm in 20 MHz.
    EXPECT_EQ(thermalNoise(20e6, 1.0), 1.380649e-23 * 290.0 * 20e6);
    EXPECT_NEAR(decibels(thermalNoise(20e6, 1.0) / 1e-3), -100.97, 0.01);
}

TEST(CommsBudget, BitErrorCurves) {
    EXPECT_NEAR(gaussianTail(3.0), 1.349898e-3, 1e-9);  // tabulated Q(3)
    // BPSK at Eb/N0 = 9.6 dB: the classic 1e-5 point (Q(4.27) = 9.8e-6).
    const double ebn0 = std::pow(10.0, 0.96);
    EXPECT_NEAR(bitErrorRate(Modulation::Bpsk, ebn0), 9.8e-6, 0.3e-6);
    // QPSK: the same bit error rate at the same Eb/N0 (twice the SNR).
    EXPECT_DOUBLE_EQ(bitErrorRate(Modulation::Qpsk, 2.0 * ebn0),
                     bitErrorRate(Modulation::Bpsk, ebn0));
    // 16-QAM at Eb/N0 = 10 dB: (4/4)·(1 − 1/4)·Q(√(3·4/15·10)) = (3/4)·Q(√8) = 1.754e-3.
    EXPECT_NEAR(bitErrorRate(Modulation::Qam16, 4.0 * 10.0), 0.75 * gaussianTail(std::sqrt(8.0)),
                1e-15);
    EXPECT_NEAR(bitErrorRate(Modulation::Qam16, 4.0 * 10.0), 1.754e-3, 1e-6);
    // Denser constellations need more SNR.
    EXPECT_GT(bitErrorRate(Modulation::Qam64, 100.0), bitErrorRate(Modulation::Qam16, 100.0));
    EXPECT_NEAR(packetErrorRate(1e-5, 12000.0), 1.0 - std::pow(1.0 - 1e-5, 12000.0), 1e-12);
}

TEST(CommsCan, FaultConfinementTransitions) {
    CanFaultConfinement node;
    for (int i = 0; i < 15; ++i) {
        node.transmitError();
    }
    EXPECT_EQ(node.transmitErrorCount(), 120);
    EXPECT_EQ(node.state(), CanFaultConfinement::State::ErrorActive);
    node.transmitError();  // 128: above 127
    EXPECT_EQ(node.state(), CanFaultConfinement::State::ErrorPassive);
    node.transmitSuccess();  // 127
    EXPECT_EQ(node.state(), CanFaultConfinement::State::ErrorActive);
    for (int i = 0; i < 16; ++i) {
        node.transmitError();
    }
    EXPECT_EQ(node.transmitErrorCount(), 255);
    EXPECT_EQ(node.state(), CanFaultConfinement::State::ErrorPassive);
    node.transmitError();  // 263: above 255
    EXPECT_EQ(node.state(), CanFaultConfinement::State::BusOff);
    node.transmitSuccess();  // nothing is sent in bus off
    EXPECT_EQ(node.transmitErrorCount(), 263);
    EXPECT_EQ(node.idle(128 * 11 - 1), 0);
    EXPECT_EQ(node.state(), CanFaultConfinement::State::BusOff);
    EXPECT_EQ(node.idle(5), 4);  // 128 × 11 recessive bits: error active again
    EXPECT_EQ(node.state(), CanFaultConfinement::State::ErrorActive);
    EXPECT_EQ(node.transmitErrorCount(), 0);
}

/// A wall: the slab 20 m ≤ x ≤ 20.2 m.
class Slab final : public services::SceneQuery {
public:
    services::RayHit ray(const std::array<double, 3>& o, const std::array<double, 3>& d,
                         double maxDistance) const override {
        services::RayHit hit;
        double t = -1.0;
        if (o[0] >= kLo && o[0] <= kHi) {
            t = d[0] > 0.0 ? (kHi - o[0]) / d[0] : d[0] < 0.0 ? (kLo - o[0]) / d[0] : -1.0;
        } else if (o[0] < kLo && d[0] > 0.0) {
            t = (kLo - o[0]) / d[0];
        } else if (o[0] > kHi && d[0] < 0.0) {
            t = (kHi - o[0]) / d[0];
        }
        if (t >= 0.0 && t <= maxDistance) {
            hit.distance = t;
            hit.shape = 0;
        }
        return hit;
    }
    std::vector<std::uint32_t> overlapSphere(const std::array<double, 3>&, double) const override {
        return {};
    }

private:
    static constexpr double kLo = 20.0;
    static constexpr double kHi = 20.2;
};

TEST(CommsWalls, CountsTheWallsBetween) {
    const Slab wall;
    EXPECT_EQ(Comms::countWalls(wall, {0.0, 0.0, 1.0}, {50.0, 3.0, 1.0}), 1);
    EXPECT_EQ(Comms::countWalls(wall, {0.0, 0.0, 1.0}, {10.0, 3.0, 1.0}), 0);
    EXPECT_EQ(Comms::countWalls(wall, {50.0, 0.0, 1.0}, {0.0, 0.0, 1.0}), 1);
}

/// A site with an access point, a receiver 50 m away behind a wall, and a CAN bus.
struct Site {
    World world;
    Site() {
        const PrimId ap = world.addPrim("/site/access_point", "access_point");
        world.prims.poseInBody[ap.index].position = {0.0, 0.0, 2.0};
        const PrimId radio = world.addPrim("/site/radio", "radio");
        world.prims.poseInBody[radio.index].position = {50.0, 0.0, 2.0};
        SchemaInstance link;
        link.schema = "RadioLinkAPI_1";
        link.prim = radio;
        link.reals = {{"comms:frequency", 2.437e9},
                      {"comms:bandwidth", 20e6},
                      {"comms:txPower", 0.1},
                      {"comms:wallLoss", 10.0},
                      {"comms:noiseFigure", 4.0}};
        link.relationships = {{"comms:transmitter", {ap}}};
        world.schemaInstances.push_back(link);
        SchemaInstance can;
        can.schema = "CanBusAPI_1";
        can.prim = world.addPrim("/robot/can0", "can0");
        can.reals = {{"can:bitRate", 500000.0}};
        world.schemaInstances.push_back(can);
    }
};

/// Writes emi/can0/bit_error_rate, as the EMI domain would.
class NoisyCan final : public IModule {
public:
    explicit NoisyCan(double ber) : ber_(ber) {}
    ModuleInfo info() const override { return {"emi", "test", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{10'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec s;
        s.name = "emi/can0/bit_error_rate";
        s.unit = units::one;
        out_ = d.output(s);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        bus.write(out_, ber_);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.write(out_, ber_);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    double ber_;
    SignalId out_;
};

TEST(Comms, ALinkBudgetThroughAWall) {
    const Site site;
    for (const bool wall : {false, true}) {
        Scheduler scheduler(SchedulerOptions{.deterministic = true});
        scheduler.add(std::make_unique<Comms>(false, wall ? std::make_shared<Slab>() : nullptr));
        auto wired = scheduler.wire(WorldView(site.world));
        ASSERT_TRUE(wired) << to_string(wired.error().front());
        ASSERT_TRUE(scheduler.stepUntil(SimTime{20'000'001}).ok());
        const SignalBus& bus = scheduler.bus();
        const double expected = 0.1 / freeSpacePathLoss(50.0, 2.437e9) / (wall ? 10.0 : 1.0);
        EXPECT_NEAR(bus.read(bus.find("link/radio/rx_power")), expected, 1e-9 * expected);
        const double snr = expected / thermalNoise(20e6, 4.0);
        EXPECT_NEAR(bus.read(bus.find("link/radio/snr")), snr, 1e-9 * snr);
        EXPECT_EQ(bus.read(bus.find("link/radio/up")), 1.0);
        EXPECT_EQ(bus.read(bus.find("can/can0/state")), 0.0);
    }
}

TEST(Comms, ANoisyCanBusGoesBusOffAndRecovers) {
    const Site site;
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<NoisyCan>(1e-2));  // FER 66 %
    scheduler.add(std::make_unique<Comms>());
    auto wired = scheduler.wire(WorldView(site.world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    const SignalBus& bus = scheduler.bus();
    const SignalId state = bus.find("can/can0/state");
    bool busOff = false;
    for (int i = 1; i <= 100 && !busOff; ++i) {
        ASSERT_TRUE(scheduler.stepUntil(SimTime{i * 10'000'000LL + 1}).ok());
        busOff = bus.read(state) == 2.0;
    }
    EXPECT_TRUE(busOff);
    EXPECT_NEAR(bus.read(bus.find("can/can0/frame_error_rate")), 1.0 - std::pow(0.99, 108.0),
                1e-12);
}

TEST(Comms, TheIdealSubstituteDeliversEverything) {
    const Site site;
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Comms>(true));
    auto wired = scheduler.wire(WorldView(site.world));
    ASSERT_TRUE(wired);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{20'000'001}).ok());
    const SignalBus& bus = scheduler.bus();
    EXPECT_EQ(bus.read(bus.find("link/radio/packet_error_rate")), 0.0);
    EXPECT_EQ(bus.read(bus.find("link/radio/snr")), kIdealSnr);
    EXPECT_EQ(bus.read(bus.find("link/radio/latency")), 0.0);
}

TEST(Comms, ShadowingHasItsSpread) {
    Site site;
    site.world.schemaInstances[0].reals["comms:shadowingStd"] = 6.0;  // dB
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<Comms>());
    ASSERT_TRUE(scheduler.wire(WorldView(site.world)));
    const SignalBus& bus = scheduler.bus();
    const double mean = decibels(0.1 / freeSpacePathLoss(50.0, 2.437e9));
    double sum2 = 0.0;
    constexpr int kSteps = 2000;
    for (int i = 1; i <= kSteps; ++i) {
        ASSERT_TRUE(scheduler.stepUntil(SimTime{i * 10'000'000LL + 1}).ok());
        const double x = decibels(bus.read(bus.find("link/radio/rx_power"))) - mean;
        sum2 += x * x;
    }
    EXPECT_NEAR(std::sqrt(sum2 / kSteps), 6.0, 0.3);
}

}  // namespace
}  // namespace lagrange::comms
