#include "lagrange/random/Philox.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange {
namespace {

using namespace literals;

/// A module whose behaviour each test supplies as functions.
class TestModule final : public IModule {
public:
    TestModule(std::string domain, std::string name, Duration step)
        : domain_(std::move(domain)), name_(std::move(name)), step_(step) {}

    std::function<void(SignalDeclarer&, TestModule&)> onDeclare;
    std::function<void(SignalBus&, TestModule&)> onInit;
    std::function<Status(const StepContext&, SignalBus&, TestModule&)> onStep;
    std::vector<SignalId> ids;       // filled by onDeclare, in declaration order
    std::vector<SimTime> stepTimes;  // every step's start time
    double state{0.0};               // saved and restored
    bool deterministic{true};
    bool isSubstitute{false};
    CapabilityTier tier{CapabilityTier::Tier0};
    bool* shutdownCalled{nullptr};

    [[nodiscard]] ModuleInfo info() const override {
        return {domain_, name_, 1, 0, isSubstitute, deterministic, tier};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& declarer) override {
        if (onDeclare) {
            onDeclare(declarer, *this);
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (onInit) {
            onInit(bus, *this);
        }
        return {};
    }
    Status step(const StepContext& context, SignalBus& bus) override {
        stepTimes.push_back(context.t);
        return onStep ? onStep(context, bus, *this) : Status{};
    }
    void shutdown() override {
        if (shutdownCalled != nullptr) {
            *shutdownCalled = true;
        }
    }
    Status saveState(StateWriter& writer) const override {
        writer.put(state);
        writer.put<std::uint64_t>(stepTimes.size());
        return {};
    }
    Status restoreState(StateReader& reader) override {
        std::uint64_t steps = 0;
        reader.get(state);
        reader.get(steps);
        stepTimes.resize(steps);
        return reader.failed() ? Status::error(StatusCode::Internal, "short state") : Status{};
    }

private:
    std::string domain_;
    std::string name_;
    Duration step_;
};

SignalSpec scalar(std::string name, Unit unit, Hold hold = Hold::ZeroOrder) {
    SignalSpec spec;
    spec.name = std::move(name);
    spec.unit = unit;
    spec.hold = hold;
    return spec;
}

/// A writer that outputs f(t_end) every step, where t_end is the end of its step.
std::unique_ptr<TestModule> source(std::string domain, Duration step, std::string signal, Unit unit,
                                   std::function<double(double)> f) {
    auto module = std::make_unique<TestModule>(std::move(domain), "source", step);
    module->onDeclare = [signal, unit](SignalDeclarer& d, TestModule& m) {
        m.ids.push_back(d.output(scalar(signal, unit)));
    };
    module->onInit = [f](SignalBus& bus, TestModule& m) { bus.write(m.ids[0], f(0.0)); };
    module->onStep = [f](const StepContext& c, SignalBus& bus, TestModule& m) {
        bus.write(m.ids[0], f((c.t + c.dt).seconds()));
        return Status{};
    };
    return module;
}

/// A reader that records what it sees of one input at every step.
struct Seen {
    std::vector<double> values;
};
std::unique_ptr<TestModule> sink(std::string domain, Duration step, SignalSpec input,
                                 std::shared_ptr<Seen> seen) {
    auto module = std::make_unique<TestModule>(std::move(domain), "sink", step);
    module->onDeclare = [input](SignalDeclarer& d, TestModule& m) {
        m.ids.push_back(d.input(input));
    };
    module->onStep = [seen](const StepContext&, SignalBus& bus, TestModule& m) {
        seen->values.push_back(bus.read(m.ids[0]));
        return Status{};
    };
    return module;
}

World emptyWorld() {
    return {};
}

std::vector<WiringError> wiringErrors(Scheduler& scheduler, const World& world) {
    auto result = scheduler.wire(WorldView(world));
    return result ? std::vector<WiringError>{} : result.error();
}

bool hasError(const std::vector<WiringError>& errors, WiringError::Kind kind) {
    for (const WiringError& error : errors) {
        if (error.kind == kind) {
            return true;
        }
    }
    return false;
}

// --- wiring errors -------------------------------------------------------------------------

// v0.2 acceptance criterion 3: a unit mismatch fails at startup naming both modules, the signal
// and both units.
TEST(Wiring, UnitMismatchNamesBothModulesTheSignalAndBothUnits) {
    Scheduler scheduler;
    scheduler.add(source("electrical", 100_us, "joint/left_wheel/torque", units::N_m,
                         [](double) { return 0.0; }));
    scheduler.add(sink("rigid_body", 1_ms, scalar("joint/left_wheel/torque", units::N),
                       std::make_shared<Seen>()));
    const World world = emptyWorld();
    const std::vector<WiringError> errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 1U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::UnitMismatch);
    EXPECT_EQ(errors[0].signal, "joint/left_wheel/torque");
    EXPECT_EQ(errors[0].detail,
              "writer electrical/source: N·m (Torque); reader rigid_body/sink: N (Force)");
    EXPECT_EQ(to_string(errors[0]),
              "UnitMismatch joint/left_wheel/torque: writer electrical/source: N·m (Torque); "
              "reader rigid_body/sink: N (Force)");
}

TEST(Wiring, TorqueIsNotEnergy) {
    Scheduler scheduler;
    scheduler.add(source("a", 1_ms, "x/y/z", units::J, [](double) { return 0.0; }));
    scheduler.add(sink("b", 1_ms, scalar("x/y/z", units::N_m), std::make_shared<Seen>()));
    const World world;
    EXPECT_TRUE(hasError(wiringErrors(scheduler, world), WiringError::Kind::UnitMismatch));
}

TEST(Wiring, TypeMismatchAndHoldOnABool) {
    Scheduler scheduler;
    scheduler.add(source("a", 1_ms, "x/y/z", units::m, [](double) { return 0.0; }));
    SignalSpec vector = scalar("x/y/z", units::m);
    vector.type = SignalType::Vec3;
    scheduler.add(sink("b", 1_ms, vector, std::make_shared<Seen>()));
    auto flag = std::make_unique<TestModule>("c", "flag", 1_ms);
    flag->onDeclare = [](SignalDeclarer& d, TestModule&) {
        SignalSpec spec = scalar("c/flag/on", Unit{});
        spec.type = SignalType::Bool;
        d.output(spec);
    };
    scheduler.add(std::move(flag));
    SignalSpec averaged = scalar("c/flag/on", Unit{}, Hold::WindowAverage);
    averaged.type = SignalType::Bool;
    scheduler.add(sink("d", 1_ms, averaged, std::make_shared<Seen>()));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 2U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::TypeMismatch);
    EXPECT_EQ(errors[0].detail, "writer a/source: Scalar; reader b/sink: Vec3");
    EXPECT_EQ(errors[1].kind, WiringError::Kind::TypeMismatch);
    EXPECT_NE(errors[1].detail.find("only ZeroOrder"), std::string::npos);
}

TEST(Wiring, MissingWriterUnlessOptional) {
    Scheduler scheduler;
    scheduler.add(sink("a", 1_ms, scalar("bus/dc/voltage", units::V), std::make_shared<Seen>()));
    SignalSpec optional = scalar("env/wind/speed", units::m_per_s);
    optional.optional = true;
    optional.defaultValue = 3.5;
    auto seen = std::make_shared<Seen>();
    scheduler.add(sink("b", 1_ms, optional, seen));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 1U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::MissingWriter);
    EXPECT_EQ(errors[0].signal, "bus/dc/voltage");
}

TEST(Wiring, OptionalUnconnectedInputReadsItsDefault) {
    Scheduler scheduler;
    SignalSpec optional = scalar("env/wind/speed", units::m_per_s);
    optional.optional = true;
    optional.defaultValue = 3.5;
    auto seen = std::make_shared<Seen>();
    scheduler.add(sink("b", 1_ms, optional, seen));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000}).ok());
    EXPECT_EQ(seen->values, (std::vector<double>{3.5, 3.5, 3.5}));
}

TEST(Wiring, DuplicateWriter) {
    Scheduler scheduler;
    scheduler.add(source("a", 1_ms, "x/y/z", units::m, [](double) { return 0.0; }));
    scheduler.add(source("b", 1_ms, "x/y/z", units::m, [](double) { return 0.0; }));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 1U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::DuplicateWriter);
    EXPECT_EQ(errors[0].detail, "written by a/source and b/source; a signal has one writer");
}

std::unique_ptr<TestModule> relay(std::string domain, std::string in, std::string out,
                                  bool feedback) {
    auto module = std::make_unique<TestModule>(std::move(domain), "relay", 1_ms);
    module->onDeclare = [in, out, feedback](SignalDeclarer& d, TestModule& m) {
        SignalSpec input = scalar(in, units::m);
        input.feedback = feedback;
        m.ids.push_back(d.input(input));
        m.ids.push_back(d.output(scalar(out, units::m)));
    };
    module->onStep = [](const StepContext&, SignalBus& bus, TestModule& m) {
        bus.write(m.ids[1], bus.read(m.ids[0]) + 1.0);
        return Status{};
    };
    return module;
}

TEST(Wiring, CycleIsReportedWithItsSignals) {
    Scheduler scheduler;
    scheduler.add(relay("electrical", "joint/w/velocity", "joint/w/torque", false));
    scheduler.add(relay("rigid_body", "joint/w/torque", "joint/w/velocity", false));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 1U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::Cycle);
    EXPECT_NE(errors[0].detail.find("electrical/relay"), std::string::npos);
    EXPECT_NE(errors[0].detail.find("rigid_body/relay"), std::string::npos);
    EXPECT_NE(errors[0].detail.find("(joint/w/torque)"), std::string::npos);
    EXPECT_NE(errors[0].detail.find("(joint/w/velocity)"), std::string::npos);
    EXPECT_NE(errors[0].detail.find("feedback: true"), std::string::npos);
}

TEST(Wiring, FeedbackBreaksTheCycleWithAOneStepDelay) {
    Scheduler scheduler;
    // a reads b's output with a delay; b reads a's output in the same tick.
    scheduler.add(relay("a", "x/b/out", "x/a/out", true));
    scheduler.add(relay("b", "x/a/out", "x/b/out", false));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_EQ(scheduler.plan()[0].label, "a/relay");
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(scheduler.stepTick().ok());
    }
    // Tick k: a = b(k-1) + 1, b = a(k) + 1, starting from zeros: a = 1, 3, 5 and b = 2, 4, 6.
    EXPECT_EQ(scheduler.bus().read(scheduler.bus().find("x/a/out")), 5.0);
    EXPECT_EQ(scheduler.bus().read(scheduler.bus().find("x/b/out")), 6.0);
}

TEST(Wiring, IncommensurateSteps) {
    {
        Scheduler scheduler;
        scheduler.add(std::make_unique<TestModule>("a", "x", Duration{500}));  // 0.5 µs
        const World world;
        EXPECT_TRUE(
            hasError(wiringErrors(scheduler, world), WiringError::Kind::IncommensurateSteps));
    }
    {
        // A typo such as dt: 0.010001 next to 1 ms: the base tick is 1 µs, but the steps line up
        // again only after 10.001 s, which is refused.
        Scheduler scheduler;
        scheduler.add(std::make_unique<TestModule>("a", "x", 1_ms));
        scheduler.add(std::make_unique<TestModule>("b", "x", Duration{10'001'000}));
        const World world;
        const auto errors = wiringErrors(scheduler, world);
        ASSERT_EQ(errors.size(), 1U);
        EXPECT_NE(errors[0].detail.find("more than 10 s"), std::string::npos);
    }
    {
        Scheduler scheduler;
        scheduler.add(std::make_unique<TestModule>("a", "x", Duration{0}));
        const World world;
        EXPECT_TRUE(
            hasError(wiringErrors(scheduler, world), WiringError::Kind::IncommensurateSteps));
    }
}

TEST(Wiring, TierAndDeterminism) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    auto gpu = std::make_unique<TestModule>("sensors", "rt_lidar", 1_ms);
    gpu->tier = CapabilityTier::Tier2;
    gpu->deterministic = false;
    scheduler.add(std::move(gpu), ModuleConfig(ModuleConfigData{.tier = CapabilityTier::Tier1}));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    EXPECT_TRUE(hasError(errors, WiringError::Kind::TierUnsupported));
    EXPECT_TRUE(hasError(errors, WiringError::Kind::NotDeterministic));
}

TEST(Wiring, InvalidDeclaration) {
    Scheduler scheduler;
    auto module = std::make_unique<TestModule>("a", "x", 1_ms);
    module->onDeclare = [](SignalDeclarer& d, TestModule&) {
        d.output(scalar("joint//torque", units::N_m));
        d.output(scalar("joint/left wheel/torque", units::N_m));
        SignalSpec held = scalar("joint/w/torque", units::N_m, Hold::WindowAverage);
        d.output(held);
    };
    scheduler.add(std::move(module));
    const World world;
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 3U);
    for (const WiringError& error : errors) {
        EXPECT_EQ(error.kind, WiringError::Kind::InvalidDeclaration);
    }
}

TEST(Wiring, ContractPerSchemaInstance) {
    SubstituteRegistry registry;
    registry.addContract(
        {"electrical",
         {{"ElectricalMotorAPI_1", "joint/{instance}/torque", SignalType::Scalar, units::N_m}}});
    World world;
    const PrimId left = world.addPrim("/amr/left_wheel", "left_wheel");
    const PrimId right = world.addPrim("/amr/right_wheel", "right_wheel");
    world.schemaInstances.push_back(
        {"ElectricalMotorAPI_1", {}, left, kWorldBody, {}, {}, {}, {}, {}, {}, {}});
    world.schemaInstances.push_back(
        {"ElectricalMotorAPI_1", {}, right, kWorldBody, {}, {}, {}, {}, {}, {}, {}});

    Scheduler scheduler(SchedulerOptions{.contracts = &registry});
    // Writes the left wheel's torque only, and in the wrong unit.
    scheduler.add(source("electrical", 100_us, "joint/left_wheel/torque", units::N,
                         [](double) { return 0.0; }));
    const auto errors = wiringErrors(scheduler, world);
    ASSERT_EQ(errors.size(), 2U);
    EXPECT_EQ(errors[0].kind, WiringError::Kind::ContractUnsatisfied);
    EXPECT_EQ(errors[0].signal, "joint/left_wheel/torque");
    EXPECT_NE(errors[0].detail.find("promises Scalar N·m (Torque)"), std::string::npos);
    EXPECT_EQ(errors[1].signal, "joint/right_wheel/torque");
}

TEST(Wiring, AllErrorsAreCollectedAndNothingIsInitialised) {
    Scheduler scheduler;
    bool initialised = false;
    auto module = std::make_unique<TestModule>("a", "x", 1_ms);
    module->onDeclare = [](SignalDeclarer& d, TestModule&) {
        d.input(scalar("p/q/r", units::m));
        d.input(scalar("p/q/s", units::m));
    };
    module->onInit = [&initialised](SignalBus&, TestModule&) { initialised = true; };
    scheduler.add(std::move(module));
    const World world;
    EXPECT_EQ(wiringErrors(scheduler, world).size(), 2U);
    EXPECT_FALSE(initialised);
}

// --- execution -----------------------------------------------------------------------------

// Roadmap 0.2-A7: 100 µs / 1 ms / 100 ms modules, call counts and values seen.
TEST(Multirate, CallCountsOverOneSecond) {
    Scheduler scheduler;
    auto fast = std::make_unique<TestModule>("electrical", "x", 100_us);
    auto mid = std::make_unique<TestModule>("rigid_body", "x", 1_ms);
    auto slow = std::make_unique<TestModule>("thermal", "x", 100_ms);
    TestModule* fastPtr = fast.get();
    TestModule* midPtr = mid.get();
    TestModule* slowPtr = slow.get();
    scheduler.add(std::move(slow));
    scheduler.add(std::move(mid));
    scheduler.add(std::move(fast));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_EQ(scheduler.baseTick(), 100_us);
    EXPECT_EQ(scheduler.hyperperiod(), 100_ms);
    // Unconnected modules run in (dt, domain, name) order.
    EXPECT_EQ(scheduler.plan()[0].label, "electrical/x");
    EXPECT_EQ(scheduler.plan()[2].label, "thermal/x");
    ASSERT_TRUE(scheduler.stepUntil(SimTime{1'000'000'000}).ok());
    EXPECT_EQ(fastPtr->stepTimes.size(), 10'000U);
    EXPECT_EQ(midPtr->stepTimes.size(), 1'000U);
    EXPECT_EQ(slowPtr->stepTimes.size(), 10U);
    EXPECT_EQ(slowPtr->stepTimes[3], SimTime{300'000'000});
    EXPECT_EQ(scheduler.now(), SimTime{1'000'000'000});
}

TEST(Multirate, WriterBeforeReaderSeesTheSameTick) {
    Scheduler scheduler;
    auto seen = std::make_shared<Seen>();
    // Added reader first: the order still puts the writer first.
    scheduler.add(sink("b", 1_ms, scalar("x/a/count", Unit{}), seen));
    scheduler.add(source("a", 1_ms, "x/a/count", Unit{}, [](double t) { return t * 1000.0; }));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_EQ(scheduler.plan()[0].label, "a/source");
    EXPECT_EQ(scheduler.plan()[1].level, 1U);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000}).ok());
    ASSERT_EQ(seen->values.size(), 3U);
    EXPECT_NEAR(seen->values[0], 1.0, 1e-9);
    EXPECT_NEAR(seen->values[2], 3.0, 1e-9);
}

TEST(Multirate, ZeroOrderHoldOfASlowWriter) {
    Scheduler scheduler;
    auto seen = std::make_shared<Seen>();
    scheduler.add(source("rigid_body", 1_ms, "joint/w/velocity", units::rad_per_s,
                         [](double t) { return t; }));
    scheduler.add(sink("electrical", 100_us, scalar("joint/w/velocity", units::rad_per_s), seen));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{2'000'000}).ok());
    ASSERT_EQ(seen->values.size(), 20U);
    // The physics step at t = 0 commits its value for t = 1 ms; the electrical steps in
    // [0, 1 ms) all see it.
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(seen->values[static_cast<std::size_t>(i)], 0.001, 1e-12) << i;
    }
    EXPECT_NEAR(seen->values[10], 0.002, 1e-12);
}

TEST(Multirate, LinearExtrapolationOfARampIsExact) {
    Scheduler scheduler;
    auto seen = std::make_shared<Seen>();
    // The fast reader runs first in its tick (feedback input), so it extrapolates forward.
    scheduler.add(
        source("rigid_body", 1_ms, "joint/w/angle", units::rad, [](double t) { return 2.0 * t; }));
    SignalSpec input = scalar("joint/w/angle", units::rad, Hold::LinearExtrapolate);
    input.feedback = true;
    scheduler.add(sink("electrical", 100_us, input, seen));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{5'000'000}).ok());
    // From t = 1 ms on, two commits exist and the reader sees 2 t at its own step time.
    for (std::size_t i = 10; i < seen->values.size(); ++i) {
        const double t = static_cast<double>(i) * 1e-4;
        EXPECT_NEAR(seen->values[i], 2.0 * t, 1e-12) << i;
    }
}

TEST(Multirate, WindowAverageConservesTheImpulse) {
    Scheduler scheduler;
    auto seen = std::make_shared<Seen>();
    // Torque from the fast side is a ramp; the slow side must see each window's mean.
    scheduler.add(source("electrical", 100_us, "joint/w/torque", units::N_m,
                         [](double t) { return 1000.0 * t; }));
    scheduler.add(
        sink("rigid_body", 1_ms, scalar("joint/w/torque", units::N_m, Hold::WindowAverage), seen));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000}).ok());
    ASSERT_EQ(seen->values.size(), 3U);
    // The slow reader reads a flow from a faster writer, so it steps at the end of each window
    // (architecture.md §8): the step for [0, 1 ms) sees the ten commits of the writer's steps in
    // it, valued 0.1 ... 1.0 N·m (each at its end time): mean 0.55. Then 1.1 ... 2.0: 1.55.
    EXPECT_TRUE(scheduler.plan()[1].atWindowEnd);
    EXPECT_FALSE(scheduler.plan()[0].atWindowEnd);
    EXPECT_NEAR(seen->values[0], 0.55, 1e-12);
    EXPECT_NEAR(seen->values[1], 1.55, 1e-12);
    EXPECT_NEAR(seen->values[2], 2.55, 1e-12);
}

TEST(Execution, PublishedBuffersResolveWhileReadersCanHoldThem) {
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    // A 10 ms camera publishes an image holding its step count; a 1 ms reader holds it, once
    // directly and once as feedback.
    auto camera = std::make_unique<TestModule>("sensors", "camera", 10_ms);
    camera->onDeclare = [](SignalDeclarer& d, TestModule& m) {
        SignalSpec image;
        image.name = "sensor/cam/image";
        image.type = SignalType::Image;
        m.ids.push_back(d.output(image));
    };
    camera->onStep = [](const StepContext& c, SignalBus& bus, TestModule& m) {
        auto buffer = std::make_shared<Buffer>();
        buffer->width = 1;
        buffer->height = 1;
        buffer->encoding = "r32u";
        buffer->bytes.assign(4, static_cast<std::byte>(c.stepIndex));
        const BufferHandle handle = bus.publishBuffer(m.ids[0], std::move(buffer));
        EXPECT_EQ(handle.id & 0xFFFF'FFFFU, c.stepIndex + 1);
        return Status{};
    };
    struct Reads {
        std::vector<int> held, delayed;
    };
    auto reads = std::make_shared<Reads>();
    auto reader = std::make_unique<TestModule>("viewer", "reader", 1_ms);
    reader->onDeclare = [](SignalDeclarer& d, TestModule& m) {
        SignalSpec image;
        image.name = "sensor/cam/image";
        image.type = SignalType::Image;
        m.ids.push_back(d.input(image));
        image.feedback = true;
        m.ids.push_back(d.input(image));
    };
    reader->onStep = [reads](const StepContext&, SignalBus& bus, TestModule& m) {
        const auto value = [&](SignalId id) {
            const std::shared_ptr<const Buffer> b = bus.buffer(bus.readBuffer(id));
            return b ? static_cast<int>(b->bytes[0]) : -1;
        };
        reads->held.push_back(value(m.ids[0]));
        reads->delayed.push_back(value(m.ids[1]));
        return Status{};
    };
    scheduler.add(std::move(camera));
    scheduler.add(std::move(reader));
    ASSERT_TRUE(scheduler.wire(WorldView(World{})));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{50'000'000}).ok());
    ASSERT_EQ(reads->held.size(), 50U);
    for (std::size_t k = 0; k < 50; ++k) {
        // Held: the camera's step k / 10 commits before the reader's step in the same tick.
        EXPECT_EQ(reads->held[k], static_cast<int>(k / 10)) << k;
        // Feedback: what was committed before the tick began; nothing before the first step.
        EXPECT_EQ(reads->delayed[k], static_cast<int>(k / 10) - (k % 10 == 0 ? 1 : 0)) << k;
    }
    EXPECT_EQ(scheduler.bus().buffer(BufferHandle{}), nullptr);
    EXPECT_EQ(scheduler.bus().buffer(BufferHandle{(1ULL << 32U) | 1U}), nullptr);  // long gone
}

TEST(Execution, StepErrorsNameTheModuleAndTime) {
    Scheduler scheduler;
    auto module = std::make_unique<TestModule>("rigid_body", "mujoco", 1_ms);
    module->onStep = [](const StepContext& c, SignalBus&, TestModule&) {
        return c.stepIndex == 2 ? Status::error(StatusCode::Diverged, "NaN in qacc") : Status{};
    };
    scheduler.add(std::move(module));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const Status status = scheduler.stepUntil(SimTime{10'000'000});
    EXPECT_EQ(status.code, StatusCode::Diverged);
    EXPECT_EQ(status.message, "rigid_body/mujoco at 0.002 s: NaN in qacc");
}

TEST(Execution, WritingAnotherModulesSignalFailsTheStep) {
    Scheduler scheduler;
    scheduler.add(source("a", 1_ms, "x/a/out", units::m, [](double) { return 0.0; }));
    auto thief = std::make_unique<TestModule>("b", "thief", 1_ms);
    thief->onStep = [](const StepContext&, SignalBus& bus, TestModule&) {
        bus.write(bus.find("x/a/out"), 42.0);
        return Status{};
    };
    scheduler.add(std::move(thief));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const Status status = scheduler.stepTick();
    EXPECT_EQ(status.code, StatusCode::Internal);
    EXPECT_NE(status.message.find("does not output"), std::string::npos);
}

TEST(Execution, BatchEnvironmentsAreSeparate) {
    Scheduler scheduler;
    auto module = std::make_unique<TestModule>("a", "x", 1_ms);
    module->onDeclare = [](SignalDeclarer& d, TestModule& m) {
        SignalSpec spec = scalar("x/a/vec", units::m);
        spec.type = SignalType::Vec3;
        m.ids.push_back(d.output(spec));
    };
    module->onStep = [](const StepContext&, SignalBus& bus, TestModule& m) {
        for (std::uint32_t env = 0; env < bus.batchCount(); ++env) {
            const double base = 10.0 * env;
            const std::array<double, 3> v{base, base + 1, base + 2};
            bus.writeN(m.ids[0], v, env);
        }
        return Status{};
    };
    scheduler.add(std::move(module));
    World world;
    world.batchCount = 3;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    const SignalId id = scheduler.bus().find("x/a/vec");
    EXPECT_EQ(scheduler.bus().readN(id, 2)[1], 21.0);
    const std::span<const double> batch = scheduler.bus().readBatch(id);
    ASSERT_EQ(batch.size(), 9U);
    EXPECT_EQ(batch[3], 10.0);
}

TEST(Execution, CostsCountStepsAndReset) {
    Scheduler scheduler;
    scheduler.add(std::make_unique<TestModule>("a", "fast", 1_ms));
    scheduler.add(std::make_unique<TestModule>("b", "slow", 10_ms));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{100'000'000}).ok());
    const std::vector<ModuleCost> costs = scheduler.takeCosts();
    ASSERT_EQ(costs.size(), 2U);
    EXPECT_EQ(costs[0].label, "a/fast");
    EXPECT_EQ(costs[0].steps, 100U);
    EXPECT_EQ(costs[1].steps, 10U);
    EXPECT_GE(costs[0].maxNs, 0);
    EXPECT_EQ(scheduler.takeCosts()[0].steps, 0U);
}

TEST(Execution, WriterOfASignal) {
    Scheduler scheduler;
    scheduler.add(source("electrical", 100_us, "x/a/out", units::A, [](double) { return 0.0; }));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const SignalWriter writer = scheduler.writerOf(scheduler.bus().find("x/a/out"));
    EXPECT_EQ(writer.label, "electrical/source");
    EXPECT_EQ(writer.dt, 100_us);
    EXPECT_TRUE(scheduler.writerOf(SignalId{}).label.empty());
}

TEST(Execution, ConnectionsListWriterReaderAndUnits) {
    Scheduler scheduler;
    scheduler.add(relay("a", "x/b/out", "x/a/out", true));
    scheduler.add(relay("b", "x/a/out", "x/b/out", false));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const std::vector<Connection> wires = scheduler.connections();
    ASSERT_EQ(wires.size(), 2U);
    EXPECT_EQ(wires[0].writer, "b/relay");
    EXPECT_EQ(wires[0].reader, "a/relay");
    EXPECT_TRUE(wires[0].feedback);
    EXPECT_EQ(wires[1].signal, "x/a/out");
    EXPECT_EQ(wires[1].readerUnit, units::m);
}

TEST(Execution, ShutdownRunsOnce) {
    bool called = false;
    {
        Scheduler scheduler;
        auto module = std::make_unique<TestModule>("a", "x", 1_ms);
        module->shutdownCalled = &called;
        scheduler.add(std::move(module));
        const World world;
        ASSERT_TRUE(scheduler.wire(WorldView(world)));
    }
    EXPECT_TRUE(called);
}

// --- state and determinism -----------------------------------------------------------------

/// A noisy integrator: x += dt * (u + noise), with u from another module and noise from Philox.
std::unique_ptr<TestModule> noisyIntegrator() {
    auto module = std::make_unique<TestModule>("thermal", "noisy", 1_ms);
    module->onDeclare = [](SignalDeclarer& d, TestModule& m) {
        m.ids.push_back(d.input(scalar("x/source/u", units::m, Hold::WindowAverage)));
        m.ids.push_back(d.output(scalar("x/noisy/x", units::m)));
    };
    module->onStep = [](const StepContext& c, SignalBus& bus, TestModule& m) {
        random::Stream rng(c.seed, c.stepIndex, 0);
        m.state += c.dt.seconds() * (bus.read(m.ids[0]) + rng.normal());
        bus.write(m.ids[1], m.state);
        return Status{};
    };
    return module;
}

Scheduler& build(Scheduler& scheduler, const World& world) {
    scheduler.add(source("electrical", 100_us, "x/source/u", units::m,
                         [](double t) { return std::sin(10.0 * t); }));
    scheduler.add(noisyIntegrator());
    EXPECT_TRUE(scheduler.wire(WorldView(world)));
    return scheduler;
}

TEST(State, SameSeedSameHashOtherSeedOtherHash) {
    const World world;
    Scheduler a(SchedulerOptions{.deterministic = true, .runSeed = 7});
    Scheduler b(SchedulerOptions{.deterministic = true, .runSeed = 7});
    Scheduler c(SchedulerOptions{.deterministic = true, .runSeed = 8});
    for (Scheduler* s : {&a, &b, &c}) {
        build(*s, world);
        ASSERT_TRUE(s->stepUntil(SimTime{500'000'000}).ok());
    }
    EXPECT_EQ(a.stateHash().value(), b.stateHash().value());
    EXPECT_NE(a.stateHash().value(), c.stateHash().value());
}

// Roadmap 0.2-A9: save -> run -> restore -> run gives identical hashes.
TEST(State, SaveRunRestoreRunIsIdentical) {
    const World world;
    Scheduler scheduler(SchedulerOptions{.runSeed = 3});
    build(scheduler, world);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{100'000'000}).ok());
    StateWriter saved;
    ASSERT_TRUE(scheduler.saveState(saved).ok());

    ASSERT_TRUE(scheduler.stepUntil(SimTime{300'000'000}).ok());
    const std::uint64_t first = scheduler.stateHash().value();

    StateReader reader(saved.bytes());
    ASSERT_TRUE(scheduler.restoreState(reader).ok());
    EXPECT_EQ(scheduler.now(), SimTime{100'000'000});
    ASSERT_TRUE(scheduler.stepUntil(SimTime{300'000'000}).ok());
    EXPECT_EQ(scheduler.stateHash().value(), first);
}

TEST(State, ModulesWithoutStateCannotBeHashed) {
    class Stateless final : public IModule {
    public:
        [[nodiscard]] ModuleInfo info() const override { return {"a", "stateless"}; }
        [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
        Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer&) override {
            return {};
        }
        Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
        Status step(const StepContext&, SignalBus&) override { return {}; }
    };
    Scheduler scheduler;
    scheduler.add(std::make_unique<Stateless>());
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const auto hash = scheduler.stateHash();
    ASSERT_FALSE(hash.has_value());
    EXPECT_EQ(hash.error().code, StatusCode::Unsupported);
    EXPECT_NE(hash.error().message.find("a/stateless"), std::string::npos);
}

TEST(State, RestoreRefusesAnotherRun) {
    const World world;
    Scheduler a;
    build(a, world);
    StateWriter saved;
    ASSERT_TRUE(a.saveState(saved).ok());
    Scheduler b;
    b.add(std::make_unique<TestModule>("other", "x", 1_ms));
    ASSERT_TRUE(b.wire(WorldView(world)));
    StateReader reader(saved.bytes());
    EXPECT_FALSE(b.restoreState(reader).ok());
}

// --- substitutes and the reference example -------------------------------------------------

// CLAUDE.md §11.5: used when electrical is disabled, commanded torque passes straight through.
class IdealTorqueSource final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"electrical", "ideal_torque_source", 1, 0, true};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        for (const SchemaInstance* motor : world.instancesOf("ElectricalMotorAPI_1")) {
            const std::string joint(world.signalInstanceName(motor->prim));
            joints_.push_back({d.input(scalar("joint/" + joint + "/torque_command", units::N_m)),
                               d.output(scalar("joint/" + joint + "/torque", units::N_m))});
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus& bus) override {
        for (const Joint& j : joints_) {
            bus.write(j.torqueOut, bus.read(j.torqueCommand));
        }
        return {};
    }

private:
    struct Joint {
        SignalId torqueCommand;
        SignalId torqueOut;
    };
    std::vector<Joint> joints_;
};

TEST(Substitutes, ProfileTokensMapToKindsAndPresets) {
    EXPECT_EQ(substituteForToken("ideal")->preset, "ideal_torque");
    EXPECT_EQ(substituteForToken("zero")->kind, SubstituteKind::Constant);
    EXPECT_EQ(substituteForToken("replay")->kind, SubstituteKind::Replay);
    EXPECT_FALSE(substituteForToken("magic").has_value());
}

TEST(Substitutes, IdealTorqueSourceSatisfiesTheElectricalContract) {
    SubstituteRegistry registry;
    registry.addContract(
        {"electrical",
         {{"ElectricalMotorAPI_1", "joint/{instance}/torque", SignalType::Scalar, units::N_m}}});
    registry.add({"electrical", SubstituteKind::Ideal, "ideal_torque",
                  [] { return std::make_unique<IdealTorqueSource>(); }});
    World world;
    const PrimId wheel = world.addPrim("/amr/left_wheel", "left_wheel");
    world.schemaInstances.push_back(
        {"ElectricalMotorAPI_1", {}, wheel, kWorldBody, {}, {}, {}, {}, {}, {}, {}});

    const auto choice = substituteForToken("ideal");
    ASSERT_TRUE(choice);
    std::unique_ptr<IModule> substitute =
        registry.create("electrical", choice->kind, choice->preset);
    ASSERT_NE(substitute, nullptr);
    EXPECT_EQ(registry.create("electrical", SubstituteKind::Constant, "zero"), nullptr);

    Scheduler scheduler(SchedulerOptions{.contracts = &registry});
    scheduler.add(source("controller", 1_ms, "joint/left_wheel/torque_command", units::N_m,
                         [](double) { return 0.25; }));
    scheduler.add(std::move(substitute));
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    ASSERT_TRUE(scheduler.stepTick().ok());
    EXPECT_EQ(scheduler.bus().read(scheduler.bus().find("joint/left_wheel/torque")), 0.25);
}

TEST(Graph, DotListsModulesAndEdges) {
    Scheduler scheduler;
    scheduler.add(relay("a", "x/b/out", "x/a/out", true));
    scheduler.add(relay("b", "x/a/out", "x/b/out", false));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    const std::string dot = scheduler.dumpGraphDot();
    EXPECT_NE(dot.find("label=\"a/relay\\n1 ms\""), std::string::npos);
    EXPECT_NE(dot.find("label=\"x/b/out\", style=dashed"), std::string::npos);
}

TEST(Graph, DotMarksSubstitutes) {
    Scheduler scheduler;
    auto standIn =
        source("electrical", 1_ms, "joint/a/torque", units::N_m, [](double) { return 0.0; });
    standIn->isSubstitute = true;
    scheduler.add(std::move(standIn));
    const World world;
    ASSERT_TRUE(scheduler.wire(WorldView(world)));
    EXPECT_NE(scheduler.dumpGraphDot().find(
                  "label=\"electrical/source\\n1 ms\\nsubstitute\", style=dashed"),
              std::string::npos)
        << scheduler.dumpGraphDot();
}

}  // namespace
}  // namespace lagrange
