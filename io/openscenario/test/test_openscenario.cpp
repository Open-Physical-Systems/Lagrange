// OpenSCENARIO (roadmap §7, "conformance subset"): the sample cut-in runs to its end condition
// with its events at the moments and places its triggers and dynamics say; shapes, parameters
// and errors.
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/openscenario/Scenario.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::openscenario {
namespace {

TEST(Dynamics, Shapes) {
    EXPECT_DOUBLE_EQ(shapeAt(Dynamics::Shape::Linear, 0.25), 0.25);
    EXPECT_DOUBLE_EQ(shapeAt(Dynamics::Shape::Sinusoidal, 0.5), 0.5);
    EXPECT_DOUBLE_EQ(shapeAt(Dynamics::Shape::Cubic, 0.5), 0.5);
    EXPECT_DOUBLE_EQ(shapeAt(Dynamics::Shape::Step, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(shapeAt(Dynamics::Shape::Sinusoidal, 1.0), 1.0);
}

TEST(Parse, ParametersAndWhatIsNotSupported) {
    const auto s = read(LAGRANGE_TEST_CUT_IN);
    ASSERT_TRUE(s) << s.error();
    EXPECT_EQ(s->entities.size(), 2U);
    ASSERT_EQ(s->init.size(), 3U);
    EXPECT_DOUBLE_EQ(s->init[2].speed, 10.0);  // $TargetSpeed
    ASSERT_EQ(s->acts.size(), 1U);
    EXPECT_EQ(s->acts[0].events.size(), 2U);
    EXPECT_NE(s->roadNetwork.find("city_block.xodr"), std::string::npos);
    const auto bad = parse(R"(<OpenSCENARIO><Entities><ScenarioObject name="a">
        <CatalogReference catalogName="c" entryName="e"/></ScenarioObject></Entities>
        <Storyboard/></OpenSCENARIO>)",
                           ".");
    ASSERT_FALSE(bad);
    EXPECT_NE(bad.error().find("catalogs"), std::string::npos);
    const auto undeclared = parse(R"(<OpenSCENARIO><Storyboard><Init><Actions>
        <Private entityRef="a"><PrivateAction><LongitudinalAction><SpeedAction>
        <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time"/>
        <SpeedActionTarget><AbsoluteTargetSpeed value="$nope"/></SpeedActionTarget>
        </SpeedAction></LongitudinalAction></PrivateAction></Private></Actions></Init>
        </Storyboard></OpenSCENARIO>)",
                                  ".");
    ASSERT_FALSE(undeclared);
    EXPECT_NE(undeclared.error().find("$nope"), std::string::npos);
}

/// The ego, standing where the scenario puts it.
class StandingEgo final : public IModule {
public:
    explicit StandingEgo(std::array<double, 4> pose) : pose_(pose) {}
    ModuleInfo info() const override { return {"test", "ego", 0, 1, false, true}; }
    Duration preferredStep() const override { return Duration{10'000'000}; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec p;
        p.name = "body/ego/pose";
        p.type = SignalType::Pose;
        p.unit = units::m;
        p.frame = "world_local";
        pose_id_ = d.output(p);
        SignalSpec t;
        t.name = "body/ego/twist";
        t.type = SignalType::Wrench;
        t.unit = units::m_per_s;
        t.frame = "world_local";
        twist_ = d.output(t);
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        return step({}, bus);
    }
    Status step(const StepContext&, SignalBus& bus) override {
        bus.writeN(pose_id_,
                   std::array<double, 7>{pose_[0], pose_[1], pose_[2], std::cos(pose_[3] / 2.0),
                                         0.0, 0.0, std::sin(pose_[3] / 2.0)});
        bus.writeN(twist_, std::array<double, 6>{});
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    std::array<double, 4> pose_;
    SignalId pose_id_, twist_;
};

TEST(CutIn, RunsToItsEndWithEventsWhereTheTriggersSay) {
    auto scenario = read(LAGRANGE_TEST_CUT_IN);
    ASSERT_TRUE(scenario) << scenario.error();
    auto network = opendrive::read(scenario->roadNetwork);
    ASSERT_TRUE(network) << network.error();
    auto model = opendrive::toModel(*network);
    ASSERT_TRUE(model) << model.error();
    ASSERT_TRUE(addEntities(*model, *scenario, "Ego"));
    const RoadLanes lanes = roadLanesOf(WorldView(model->world));
    const auto egoPose = poseOf(scenario->init[0].position, lanes);
    ASSERT_TRUE(egoPose);
    EXPECT_NEAR((*egoPose)[0], 12.0 + 30.0, 1e-6);  // road 1 starts at x = 12, heading east
    EXPECT_NEAR((*egoPose)[1], -5.25, 1e-6);        // lane -2's centre

    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<StandingEgo>(*egoPose));
    auto runner = std::make_unique<ScenarioRunner>(*scenario, "Ego");
    const ScenarioRunner* view = runner.get();
    ModuleConfigData config;
    config.values["ego_body"] = std::string("ego");
    scheduler.add(std::move(runner), ModuleConfig(config));
    auto wired = scheduler.wire(WorldView(model->world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());
    const SignalBus& bus = scheduler.bus();
    const SignalId table = bus.find("scenario/entities");
    // The lane change starts when the target has travelled 35 m at 10 m/s: 3.5 s.
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'400'000'000}).ok());
    EXPECT_EQ(bus.readN(table)[7], -1.0);  // still in the inner lane
    ASSERT_TRUE(scheduler.stepUntil(SimTime{5'000'000'000}).ok());
    // Halfway (sinusoidal, 1.5 s of 3): halfway between the lanes, y = -3.5.
    EXPECT_NEAR(bus.readN(table)[1], -3.5, 0.1);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{6'600'000'000}).ok());
    EXPECT_EQ(bus.readN(table)[7], -2.0);  // in the ego's lane
    EXPECT_NEAR(bus.readN(table)[1], -5.25, 1e-6);
    EXPECT_GT(bus.readN(table)[0], (*egoPose)[0] + 5.0);  // in front of the ego
    // It slows linearly from 10 to 6 m/s over 2 s from 6.5 s: 8 m/s at 7.5 s.
    ASSERT_TRUE(scheduler.stepUntil(SimTime{7'510'000'000}).ok());
    EXPECT_NEAR(bus.readN(table)[4], 8.0, 0.05);
    ASSERT_TRUE(scheduler.stepUntil(SimTime{9'000'000'000}).ok());
    EXPECT_NEAR(bus.readN(table)[4], 6.0, 1e-9);
    // The stop trigger at 12 s ends it; its time freezes.
    ASSERT_TRUE(scheduler.stepUntil(SimTime{12'500'000'000}).ok());
    EXPECT_EQ(bus.read(bus.find("scenario/state")), 1.0);
    EXPECT_NEAR(bus.read(bus.find("scenario/time")), 12.01, 0.011);
    ASSERT_GE(view->events().size(), 4U);
    EXPECT_NE(view->events()[1].find("change_lane"), std::string::npos);
}

}  // namespace
}  // namespace lagrange::openscenario
