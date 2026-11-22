// The builtin traffic (domains/traffic.md §7): the IDM's equilibrium, the light cycle, and the
// city block's traffic on MuJoCo: vehicles move, keep their distance, never enter a junction on
// red, pedestrians stay on their sidewalks, and the same seed gives the same tracks.
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/rigid_body/MujocoBackend.hpp"
#include "lagrange/scheduler/Scheduler.hpp"
#include "lagrange/traffic/Traffic.hpp"
#include "lagrange/world/PoseMath.hpp"

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

// OpenDRIVE traffic lights (io/opendrive/test/data/signals.xodr): a four-way junction whose
// arms (road 1 east, 2 north, 3 west, 4 south) each carry a light 3 m out from the junction,
// east-west on controller 1, north-south on controller 2, which the junction lists first.
Model signalJunction(const Population& population) {
    auto network = opendrive::read(LAGRANGE_TEST_SIGNALS_XODR);
    EXPECT_TRUE(network) << network.error();
    auto model = opendrive::toModel(*network);
    EXPECT_TRUE(model) << model.error();
    model->world.addCollider(kWorldBody, ShapeType::Plane, {100.0, 100.0, 1.0}, Pose{}, 1.0);
    const auto made = addAgents(*model, population, {});
    EXPECT_EQ(made[0], population.vehicles);
    return std::move(*model);
}

TEST(OpenDriveSignals, GiveTheLightsTheirLanesGroupsStopLinesAndHeads) {
    Model model = signalJunction({0, 0, 1});
    const WorldView world(model.world);
    const LaneGraph g = lanesOf(world);
    ASSERT_TRUE(g.fromSignals);
    const std::vector<RoadSignal> signals = roadSignalsOf(world, roadLanesOf(world));
    std::map<int, const RoadSignal*> signalOfRoad;
    for (const RoadSignal& s : signals) {
        signalOfRoad[s.road] = &s;
    }
    int lit = 0;
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        if (g.signalGroup[i] < 0) {
            continue;
        }
        ++lit;
        const Lane& lane = g.lanes[i];
        EXPECT_EQ(lane.lane, 1) << lane.name;  // the lane coming in
        // Controller 2 (north-south, sequence 1) first, then controller 1 (sequence 2).
        EXPECT_EQ(g.signalGroup[i], lane.road % 2 == 0 ? 0 : 1) << lane.name;
        EXPECT_EQ(g.groups[static_cast<std::size_t>(g.junctionOf[i])], 2);
        EXPECT_NEAR(g.stopAt[i], lane.length() - 3.0, 1e-6) << lane.name;
    }
    EXPECT_EQ(lit, 4);
    // Every lamp at its signal's face: lit 12 mm in front of it, a third of the 0.9 m head
    // apart; dark inside the housing behind it.
    int lamps = 0;
    for (const SignalLamp& lamp : signalLamps(g)) {
        if (lamp.kind != LampKind::Red && lamp.kind != LampKind::Yellow &&
            lamp.kind != LampKind::Green) {
            continue;
        }
        ++lamps;
        const Pose& face = signalOfRoad.at(g.lanes[lamp.lane].road)->face;
        const auto out = rotate(face.orientation, {1.0, 0.0, 0.0});
        const double up = lamp.kind == LampKind::Red     ? 0.3
                          : lamp.kind == LampKind::Green ? -0.3
                                                         : 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            const double expected = face.position.at(k) + 0.012 * out.at(k) + (k == 2 ? up : 0.0);
            EXPECT_NEAR(lamp.lit.position.at(k), expected, 1e-9) << lamp.name;
            const double behind = face.position.at(k) - 0.1 * out.at(k) + (k == 2 ? up : 0.0);
            EXPECT_NEAR(lamp.dark.position.at(k), behind, 1e-9) << lamp.name;
        }
    }
    EXPECT_EQ(lamps, 12);
}

TEST(OpenDriveSignals, ControllersAlternateAndVehiclesStopAtTheirSignals) {
    const Population population{10, 0, 7};
    Model model = signalJunction(population);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    ModuleConfigData config;
    config.values["seed"] = static_cast<std::int64_t>(population.seed);
    scheduler.add(std::make_unique<Traffic>(), ModuleConfig(config));
    ASSERT_TRUE(scheduler.wire(WorldView(model.world)));
    const SignalBus& bus = scheduler.bus();
    const SignalId table = bus.find("traffic/agents");
    const SignalId lightsId = bus.find("traffic/lights");
    // Each arm's way out from the centre; its stop line 13 m out.
    const std::map<int, std::array<double, 2>> outward{
        {1, {1.0, 0.0}}, {2, {0.0, 1.0}}, {3, {-1.0, 0.0}}, {4, {0.0, -1.0}}};
    constexpr double kStopLine = 13.0;                         // m from the centre
    std::map<int, Light> before, earlier;                      // a step and two steps ago
    std::vector<double> ahead(population.vehicles, INFINITY);  // front to its line, last step
    // Whether it has been short of its line on red since its light turned red: a vehicle that
    // went over on yellow is not waiting for it.
    std::vector<bool> waiting(population.vehicles, false);
    int bothGreen = 0;
    int crossedOnRed = 0;
    int crossings = 0;
    std::map<int, int> greens;        // per road, times it turned green
    double worstStop = INFINITY;      // the farthest a waiting vehicle stood past its line
    double farthestStop = -INFINITY;  // the farthest a waiting vehicle stood short of it
    int stops = 0;
    for (int step = 1; step <= 2400; ++step) {  // four minutes at 0.1 s
        ASSERT_TRUE(scheduler.stepUntil(SimTime{step * 100'000'000LL}).ok());
        const std::span<const double> lights = bus.readN(lightsId);
        std::map<int, Light> light;
        for (std::size_t k = 0; k + 1 < lights.size(); k += 2) {
            light[static_cast<int>(lights[k])] =
                static_cast<Light>(static_cast<int>(lights[k + 1]));
        }
        ASSERT_EQ(light.size(), 4U);
        const auto go = [&](int road) { return light.at(road) != Light::Red; };
        if ((go(1) || go(3)) && (go(2) || go(4))) {
            ++bothGreen;
        }
        EXPECT_EQ(light.at(1), light.at(3));  // one controller, one light
        EXPECT_EQ(light.at(2), light.at(4));
        if (step == 10) {
            EXPECT_EQ(light.at(2), Light::Green) << "the junction's first controller";
            EXPECT_EQ(light.at(1), Light::Red);
        }
        for (const auto& [road, now] : light) {
            if (now == Light::Green && before.contains(road) && before.at(road) != Light::Green) {
                ++greens[road];
            }
        }
        const std::span<const double> a = bus.readN(table);
        for (int i = 0; i < population.vehicles; ++i) {
            const double x = a[8 * i + 1];
            const double y = a[8 * i + 2];
            const double yaw = a[8 * i + 4];
            const double speed = a[8 * i + 5];
            const double half = a[8 * i + 6] / 2.0;
            const std::array<double, 2> front{x + std::cos(yaw) * half, y + std::sin(yaw) * half};
            // The arm it is coming in on: heading in, on the incoming lane's side.
            double toLine = INFINITY;
            int road = 0;
            for (const auto& [r, u] : outward) {
                const double inward = -(std::cos(yaw) * u[0] + std::sin(yaw) * u[1]);
                const double side = u[0] * y - u[1] * x;  // left of the outward way
                if (inward > 0.9 && side > 0.0 && x * u[0] + y * u[1] > 9.0) {
                    toLine = front[0] * u[0] + front[1] * u[1] - kStopLine;
                    road = r;
                }
            }
            if (road != 0) {
                // Red for 0.2 s: a vehicle committed on yellow may reach its line a step into
                // the red, as at the city block's junction (NoVehicleEntersTheJunctionOnRed).
                const bool red = light.at(road) == Light::Red && earlier.contains(road) &&
                                 before.at(road) == Light::Red && earlier.at(road) == Light::Red;
                if (ahead[i] > 0.0 && toLine <= 0.0) {
                    ++crossings;
                    crossedOnRed += red ? 1 : 0;
                    EXPECT_FALSE(red)
                        << "vehicle " << i << " over its line on red at step " << step << ": "
                        << ahead[i] << " -> " << toLine << " m, " << speed << " m/s";
                }
                waiting[i] = red && (waiting[i] || toLine > 0.0);
                if (waiting[i] && speed < 0.05 && toLine < 5.0) {
                    ++stops;
                    worstStop = std::min(worstStop, toLine);
                    farthestStop = std::max(farthestStop, toLine);
                }
            }
            if (road == 0) {
                waiting[i] = false;
            }
            ahead[i] = toLine;
        }
        earlier = before;
        before = light;
    }
    EXPECT_EQ(bothGreen, 0) << "the two controllers are never both open";
    for (int road = 1; road <= 4; ++road) {
        EXPECT_GE(greens[road], 4) << "road " << road << " turned green each cycle";
    }
    EXPECT_GT(crossings, 20);
    EXPECT_EQ(crossedOnRed, 0);
    // The first in a queue (the only one within 5 m of the line: the next is a car's length and
    // the jam distance back) stands at its signal's line, not 3 m on at the lane's end: within
    // half a car's length plus a braking margin of 0.5 m short of it, and never past it.
    EXPECT_GT(stops, 100);
    EXPECT_GT(worstStop, -0.05);
    EXPECT_LT(farthestStop, kCarSize[0] / 2.0 + 0.5);
    RecordProperty("crossings", crossings);
    RecordProperty("stops_on_red", stops);
    RecordProperty("greens_road_1", greens[1]);
    RecordProperty("greens_road_2", greens[2]);
    RecordProperty("closest_stop_m", std::to_string(worstStop));
    RecordProperty("farthest_stop_m", std::to_string(farthestStop));
}

// A banked road (io/opendrive/test/data/banked.xodr, road 1: 5° falling to the right of s, 2 m
// up): the lanes' heights give each vehicle its roll, the road's bank seen along its way.
TEST(BankedRoad, VehiclesLieOnTheBank) {
    constexpr double kBank = 0.0872664626;  // rad
    auto network = opendrive::read(LAGRANGE_TEST_BANKED_XODR);
    ASSERT_TRUE(network) << network.error();
    auto model = opendrive::toModel(*network);
    ASSERT_TRUE(model) << model.error();
    model->world.addCollider(kWorldBody, ShapeType::Plane, {100.0, 100.0, 1.0}, Pose{}, 1.0);
    const LaneGraph g = lanesOf(WorldView(model->world));
    ASSERT_TRUE(g.sloped);
    for (std::size_t i = 0; i < g.lanes.size(); ++i) {
        const Lane& lane = g.lanes[i];
        if (lane.road != 1 || lane.sidewalk) {
            continue;
        }
        ASSERT_TRUE(g.beside[i]);
        const Tilt tilt = tiltAt(g, i, 0.5 * lane.length());
        // Along s (lane -1) the left is up: +5°; against it (lane 1), -5°.
        EXPECT_NEAR(tilt.roll, lane.lane < 0 ? kBank : -kBank, 1e-9) << lane.name;
        EXPECT_NEAR(tilt.pitch, 0.0, 1e-12);
    }
    const auto made = addAgents(*model, {6, 0, 2}, {});
    ASSERT_GT(made[0], 0);
    Scheduler scheduler(SchedulerOptions{.deterministic = true});
    scheduler.add(std::make_unique<rigid_body::MujocoBackend>());
    scheduler.add(std::make_unique<Traffic>());
    ASSERT_TRUE(scheduler.wire(WorldView(model->world)));
    ASSERT_TRUE(scheduler.stepUntil(SimTime{3'000'000'000}).ok());
    const SignalBus& bus = scheduler.bus();
    int onBank = 0;
    for (int i = 0; i < made[0]; ++i) {
        const auto q = bus.readN(bus.find("body/traffic_vehicle_" + std::to_string(i) + "/pose"));
        const double y = q[1];
        if (std::abs(y) > 5.0) {
            continue;  // road 2
        }
        const double roll =
            std::atan2(2.0 * (q[3] * q[4] + q[5] * q[6]), 1.0 - 2.0 * (q[4] * q[4] + q[5] * q[5]));
        const double expected = y < 0.0 ? kBank : -kBank;  // lane -1 is on the right of s
        EXPECT_NEAR(roll, expected, 1e-6);
        // Its centre above the lane's centre line, which lies on the bank 2 m up.
        EXPECT_GT(q[2], 2.0 + std::abs(y) * std::tan(kBank) * (y < 0.0 ? -1.0 : 1.0));
        ++onBank;
    }
    EXPECT_GT(onBank, 0);
}

}  // namespace
}  // namespace lagrange::traffic
