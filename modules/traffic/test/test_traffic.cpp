// The builtin traffic (domains/traffic.md §7): the IDM's equilibrium, the light cycle, and the
// city block's traffic on MuJoCo: vehicles move, keep their distance, never enter a junction on
// red, pedestrians stay on their sidewalks, and the same seed gives the same tracks.
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/traffic/Traffic.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::traffic {
namespace {

TEST(Idm, EquilibriumAndFreeRoad) {
    const Idm p;
    // At its desired speed on a free road it neither speeds up nor slows down.
    EXPECT_NEAR(idmAcceleration(p, 13.9, 13.9, INFINITY, 0.0), 0.0, 1e-12);
    // Following a leader at the same speed, the equilibrium gap s0 + v·T over √(1 − (v/v0)⁴)
    // gives no acceleration (Treiber and Kesting, Traffic Flow Dynamics, §11.3).
    const double v = 10.0;
    const double v0 = 13.9;
    const double gap = (p.s0 + v * p.T) / std::sqrt(1.0 - std::pow(v / v0, 4));
    EXPECT_NEAR(idmAcceleration(p, v, v0, gap, v), 0.0, 1e-12);
    // Closing in on a stopped car brakes.
    EXPECT_LT(idmAcceleration(p, 10.0, v0, 15.0, 0.0), -2.0);
}

TEST(Lights, TakeTurnsGreenYellowRed) {
    EXPECT_EQ(lightAt(0, 2, 1.0), Light::Green);
    EXPECT_EQ(lightAt(1, 2, 1.0), Light::Red);
    EXPECT_EQ(lightAt(0, 2, 21.0), Light::Yellow);
    EXPECT_EQ(lightAt(0, 2, 24.0), Light::Red);  // all red
    EXPECT_EQ(lightAt(1, 2, 24.0), Light::Red);
    EXPECT_EQ(lightAt(1, 2, 26.0), Light::Green);
    EXPECT_EQ(lightAt(0, 2, 51.0), Light::Green);  // a 50 s cycle
}

Model cityBlock(const Population& population) {
    auto network = opendrive::read(LAGRANGE_TEST_CITY_XODR);
    EXPECT_TRUE(network) << network.error();
    auto model = opendrive::toModel(*network);
    EXPECT_TRUE(model) << model.error();
    model->world.addCollider(kWorldBody, ShapeType::Plane, {100.0, 100.0, 1.0}, Pose{}, 1.0);
    const auto made = addAgents(*model, population, {});
    EXPECT_EQ(made[0], population.vehicles);
    EXPECT_EQ(made[1], population.pedestrians);
    return std::move(*model);
}

// QA 2026-11-12: --traffic vehicles=100000,pedestrians=100000 spent 40 s trying to place agents
// the lanes had no room for before the run loaded. It stops once the lanes are full.
TEST(CityBlock, AskingForMoreAgentsThanFitStopsWhenTheLanesAreFull) {
    auto network = opendrive::read(LAGRANGE_TEST_CITY_XODR);
    ASSERT_TRUE(network) << network.error();
    auto model = opendrive::toModel(*network);
    ASSERT_TRUE(model) << model.error();
    const auto started = std::chrono::steady_clock::now();
    const auto made = addAgents(*model, {1'000'000, 1'000'000, 1}, {});
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    EXPECT_GT(made[0], 0);
    EXPECT_LT(made[0], 1000);
    EXPECT_LT(made[1], 1000);
    EXPECT_LT(seconds, 5.0) << "it gave up once the lanes were full";
}

struct Run {
    Model model;
    std::unique_ptr<Scheduler> scheduler;
};

std::unique_ptr<Run> run(const Population& population, bool zero = false) {
    auto r = std::make_unique<Run>();
    r->model = cityBlock(population);
    r->scheduler = std::make_unique<Scheduler>(SchedulerOptions{.deterministic = true});
    r->scheduler->add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData config;
    config.values["seed"] = static_cast<std::int64_t>(population.seed);
    r->scheduler->add(std::make_unique<Traffic>(zero), ModuleConfig(config));
    auto wired = r->scheduler->wire(WorldView(r->model.world));
    EXPECT_TRUE(wired) << (wired ? "" : to_string(wired.error().front()));
    return wired ? std::move(r) : nullptr;
}

TEST(CityBlock, VehiclesChangeLanesWithoutComingTooClose) {
    const Population population{14, 0, 5};
    auto r = run(population);
    ASSERT_TRUE(r);
    const SignalBus& bus = r->scheduler->bus();
    const SignalId table = bus.find("traffic/agents");
    double closest = INFINITY;
    for (int step = 1; step <= 3000; ++step) {  // five minutes at 0.1 s
        ASSERT_TRUE(r->scheduler->stepUntil(SimTime{step * 100'000'000LL}).ok());
        const std::span<const double> a = bus.readN(table);
        for (int i = 0; i < population.vehicles; ++i) {
            for (int k = i + 1; k < population.vehicles; ++k) {
                closest = std::min(
                    closest, std::hypot(a[8 * i + 1] - a[8 * k + 1], a[8 * i + 2] - a[8 * k + 2]));
            }
        }
    }
    EXPECT_GT(bus.read(bus.find("traffic/lane_changes")), 0.0);
    EXPECT_GT(closest, 2.0);  // side by side in neighbouring lanes is 3.5 m, less while moving over
}

TEST(CityBlock, VehiclesFollowKeepDistanceAndStopOnRed) {
    const Population population{8, 6, 3};
    auto r = run(population);
    ASSERT_TRUE(r);
    const LaneGraph g = lanesOf(WorldView(r->model.world));
    ASSERT_EQ(g.junctions, 1);
    const SignalBus& bus = r->scheduler->bus();
    const SignalId table = bus.find("traffic/agents");
    // Which lane each vehicle is on, by projecting its position: entering a junction lane is
    // allowed only from an approach whose light is not red.
    double speedSum = 0.0;
    int speedCount = 0;
    double closest = INFINITY;
    for (int step = 1; step <= 1200; ++step) {  // two minutes at 0.1 s
        ASSERT_TRUE(r->scheduler->stepUntil(SimTime{step * 100'000'000LL}).ok());
        const std::span<const double> a = bus.readN(table);
        for (int i = 0; i < population.vehicles; ++i) {
            speedSum += a[8 * i + 5];
            ++speedCount;
            for (int k = i + 1; k < population.vehicles; ++k) {
                closest = std::min(
                    closest, std::hypot(a[8 * i + 1] - a[8 * k + 1], a[8 * i + 2] - a[8 * k + 2]));
            }
        }
        for (int i = population.vehicles; i < population.vehicles + population.pedestrians; ++i) {
            EXPECT_EQ(a[8 * i], 1.0);
            // Sidewalks run 7 to 10 m from the arms' centre lines.
            const double lateral = std::min(std::abs(a[8 * i + 1]), std::abs(a[8 * i + 2]));
            EXPECT_GT(lateral, 6.9) << "pedestrian " << i;
            EXPECT_LT(lateral, 10.1) << "pedestrian " << i;
        }
    }
    EXPECT_GT(speedSum / speedCount, 3.0);  // traffic flows (with stops at the lights)
    // Centres never closer than a car length: vehicles keep their distance (crossing paths
    // inside the junction come no closer than this either at the sample's density).
    EXPECT_GT(closest, 3.0);
}

TEST(CityBlock, NoVehicleEntersTheJunctionOnRed) {
    const Population population{10, 0, 5};
    auto r = run(population);
    ASSERT_TRUE(r);
    const LaneGraph g = lanesOf(WorldView(r->model.world));
    const SignalBus& bus = r->scheduler->bus();
    const SignalId table = bus.find("traffic/agents");
    // A vehicle is "in the junction" within 12 m of the origin on both axes.
    std::vector<bool> inside(population.vehicles, false);
    int entries = 0;
    int onRed = 0;
    for (int step = 1; step <= 1800; ++step) {  // three minutes at 0.1 s
        ASSERT_TRUE(r->scheduler->stepUntil(SimTime{step * 100'000'000LL}).ok());
        const std::span<const double> a = bus.readN(table);
        const double t = step * 0.1;
        for (int i = 0; i < population.vehicles; ++i) {
            const double x = a[8 * i + 1];
            const double y = a[8 * i + 2];
            const bool now = std::abs(x) < 12.0 && std::abs(y) < 12.0;
            if (now && !inside[i]) {
                ++entries;
                // Its approach: east-west arms are group 0 or 1 by their heading; find the
                // light of the arm it came from (the one it is nearest the edge of).
                const bool eastWest = std::abs(x) > std::abs(y);
                // The first approach in the graph decides which group is which: look it up.
                int group = -1;
                for (std::size_t l = 0; l < g.lanes.size(); ++l) {
                    if (g.signalGroup[l] < 0) {
                        continue;
                    }
                    const auto [p, h] = g.lanes[l].at(g.lanes[l].length());
                    const bool laneEastWest = std::abs(p[0]) > std::abs(p[1]);
                    if (laneEastWest == eastWest) {
                        group = g.signalGroup[l];
                        break;
                    }
                }
                ASSERT_GE(group, 0);
                // Allow one step of rounding at the line: the light a moment before.
                if (lightAt(group, 2, t - 0.2) == Light::Red &&
                    lightAt(group, 2, t) == Light::Red) {
                    ++onRed;
                }
            }
            inside[i] = now;
        }
    }
    EXPECT_GT(entries, 10);
    EXPECT_EQ(onRed, 0);
}

TEST(CityBlock, TheSameSeedGivesTheSameTracks) {
    const Population population{6, 4, 11};
    auto a = run(population);
    auto b = run(population);
    ASSERT_TRUE(a && b);
    ASSERT_TRUE(a->scheduler->stepUntil(SimTime{30'000'000'000}).ok());
    ASSERT_TRUE(b->scheduler->stepUntil(SimTime{30'000'000'000}).ok());
    const auto ta = a->scheduler->bus().readN(a->scheduler->bus().find("traffic/agents"));
    const auto tb = b->scheduler->bus().readN(b->scheduler->bus().find("traffic/agents"));
    for (std::size_t i = 0; i < ta.size(); ++i) {
        EXPECT_EQ(ta[i], tb[i]) << i;
    }
}

TEST(CityBlock, TheZeroSubstituteHidesTheAgents) {
    auto r = run({4, 2, 1}, true);
    ASSERT_TRUE(r);
    ASSERT_TRUE(r->scheduler->stepUntil(SimTime{1'000'000'000}).ok());
    const SignalBus& bus = r->scheduler->bus();
    EXPECT_EQ(bus.read(bus.find("traffic/agent_count")), 0.0);
    EXPECT_LT(bus.readN(bus.find("body/traffic_vehicle_0/pose"))[2], -50.0);
}

}  // namespace
}  // namespace lagrange::traffic
