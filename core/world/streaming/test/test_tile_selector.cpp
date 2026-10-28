#include "lagrange/world/streaming/TileSelector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::streaming {
namespace {

// A 8192 m square world in three levels, as huge-worlds.md §2 lays them out: 4096 m tiles
// (error 8 m), 1024 m (2 m) and 256 m tiles with colliders (0.5 m).
std::vector<TileInfo> pyramid() {
    std::vector<TileInfo> tiles;
    const std::array<double, 3> sizes{4096.0, 1024.0, 256.0};
    const std::array<double, 3> errors{8.0, 2.0, 0.5};
    for (std::int32_t level = 0; level < 3; ++level) {
        const double size = sizes.at(static_cast<std::size_t>(level));
        const int count = static_cast<int>(8192.0 / size);
        for (int x = 0; x < count; ++x) {
            for (int y = 0; y < count; ++y) {
                TileInfo t;
                t.path = "/world/L" + std::to_string(level) + "_" + std::to_string(x) + "_" +
                         std::to_string(y);
                t.level = level;
                t.boundsMin = {x * size, y * size, -10.0};
                t.boundsMax = {(x + 1) * size, (y + 1) * size, 50.0};
                t.geometricError = errors.at(static_cast<std::size_t>(level));
                t.hasCollision = level == 2;
                t.bytes = static_cast<std::uint64_t>(size);  // larger tiles cost more
                tiles.push_back(t);
            }
        }
    }
    return tiles;
}

StreamingSettings settings() {
    StreamingSettings s;
    s.loadRadius = 600.0;
    s.unloadRadius = 800.0;
    s.errorPerMetre = 0.002;
    s.unloadDelayS = 5.0;
    return s;
}

/// Loads everything a plan asks for and drops what it unloads (an instant loader).
void apply(TileSelector& selector, const StreamingPlan& plan) {
    for (const std::uint32_t t : plan.unload) {
        selector.unloaded(t);
    }
    for (const std::uint32_t t : plan.load) {
        selector.loaded(t);
    }
}

/// Runs selections with an instant loader until nothing changes.
void settle(TileSelector& selector, const std::vector<Interest>& interests, double t) {
    for (int i = 0; i < 10; ++i) {
        const StreamingPlan plan = selector.update(interests, t);
        if (plan.load.empty() && plan.unload.empty()) {
            return;
        }
        apply(selector, plan);
    }
    FAIL() << "the selection did not settle";
}

/// True when every point of a grid around `centre` within `radius` lies in a loaded tile.
bool covered(const TileSelector& selector, const std::array<double, 3>& centre, double radius) {
    const auto& tiles = selector.tiles();
    for (double dx = -radius; dx <= radius; dx += 50.0) {
        for (double dy = -radius; dy <= radius; dy += 50.0) {
            const std::array<double, 3> p{centre[0] + dx, centre[1] + dy, 0.0};
            if (std::hypot(dx, dy) > radius || p[0] < 0 || p[1] < 0 || p[0] > 8192 || p[1] > 8192) {
                continue;
            }
            bool inside = false;
            for (std::uint32_t i = 0; i < tiles.size() && !inside; ++i) {
                inside = selector.isLoaded(i) &&
                         horizontalDistance(p, tiles[i].boundsMin, tiles[i].boundsMax) == 0.0;
            }
            if (!inside) {
                return false;
            }
        }
    }
    return true;
}

TEST(TileSelector, ARobotGetsTheCollisionTilesAroundItAndNoHoles) {
    TileSelector selector(pyramid(), settings());
    const std::vector<Interest> robot{{Interest::Kind::Robot, {4000.0, 4000.0, 0.0}}};
    settle(selector, robot, 0.0);
    const auto& tiles = selector.tiles();
    std::size_t collision = 0;
    for (std::uint32_t i = 0; i < tiles.size(); ++i) {
        const double d =
            horizontalDistance(robot[0].position, tiles[i].boundsMin, tiles[i].boundsMax);
        if (tiles[i].hasCollision && d <= 600.0) {
            EXPECT_TRUE(selector.isLoaded(i)) << tiles[i].path;
            ++collision;
        }
        if (d > 800.0) {
            EXPECT_FALSE(selector.isLoaded(i)) << tiles[i].path << " is far";
        }
    }
    EXPECT_GT(collision, 16U);
    EXPECT_TRUE(covered(selector, robot[0].position, 600.0));
    EXPECT_TRUE(selector.physicsReady(robot[0].position, 5.0));
}

TEST(TileSelector, ARobotMayNotEnterATileThatIsNotLoaded) {
    TileSelector selector(pyramid(), settings());
    const std::array<double, 3> at{1000.0, 1000.0, 0.0};
    EXPECT_FALSE(selector.physicsReady(at, 5.0)) << "nothing loaded yet";
    const std::vector<Interest> here{{Interest::Kind::Robot, at}};
    const StreamingPlan plan = selector.update(here, 0.0);
    ASSERT_FALSE(plan.load.empty());
    EXPECT_FALSE(selector.physicsReady(at, 5.0)) << "requested is not loaded";
    for (const std::uint32_t t : plan.load) {
        EXPECT_TRUE(selector.isLoading(t));
    }
    apply(selector, plan);
    EXPECT_TRUE(selector.physicsReady(at, 5.0));
    // 900 m away, in a tile 792 m from the robot (outside the load radius): no ground there.
    EXPECT_FALSE(selector.physicsReady({1900.0, 1000.0, 0.0}, 5.0));
}

TEST(TileSelector, ACoarseTileStaysUntilTheFinerOnesAreLoaded) {
    TileSelector selector(pyramid(), settings());
    // A sensor 3 km out of the world's corner sees only the coarsest level well enough.
    const std::vector<Interest> far{{Interest::Kind::Sensor, {-550.0, 4000.0, 0.0}}};
    settle(selector, far, 0.0);
    std::vector<std::uint32_t> coarse;
    for (std::uint32_t i = 0; i < selector.tiles().size(); ++i) {
        if (selector.isLoaded(i)) {
            coarse.push_back(i);
        }
    }
    ASSERT_FALSE(coarse.empty());
    // A robot arrives: finer tiles are requested, and the coarse ones stay until they load.
    const std::vector<Interest> robot{{Interest::Kind::Robot, {100.0, 4000.0, 0.0}}};
    const StreamingPlan plan = selector.update(robot, 1.0);
    ASSERT_FALSE(plan.load.empty());
    for (const std::uint32_t c : coarse) {
        EXPECT_EQ(std::count(plan.unload.begin(), plan.unload.end(), c), 0)
            << selector.tiles()[c].path << " dropped before its replacements loaded";
    }
    apply(selector, plan);
    settle(selector, robot, 1.0);
    EXPECT_TRUE(covered(selector, robot[0].position, 600.0));
}

TEST(TileSelector, UnloadingWaitsForTheUnloadRadiusAndTheDelay) {
    TileSelector selector(pyramid(), settings());
    settle(selector, {{Interest::Kind::Robot, {4000.0, 4000.0, 0.0}}}, 0.0);
    std::uint32_t tile = 0;
    for (std::uint32_t i = 0; i < selector.tiles().size(); ++i) {
        if (selector.isLoaded(i) && selector.tiles()[i].path == "/world/L2_15_15") {
            tile = i;
        }
    }
    ASSERT_TRUE(selector.isLoaded(tile));
    // 700 m from it: between the radii, it stays.
    const std::vector<Interest> between{{Interest::Kind::Robot, {4540.0, 4540.0, 0.0}}};
    StreamingPlan plan = selector.update(between, 1.0);
    EXPECT_EQ(std::count(plan.unload.begin(), plan.unload.end(), tile), 0);
    apply(selector, plan);
    // Beyond the unload radius: only after 5 s.
    const std::vector<Interest> beyond{{Interest::Kind::Robot, {5000.0, 5000.0, 0.0}}};
    plan = selector.update(beyond, 2.0);
    EXPECT_EQ(std::count(plan.unload.begin(), plan.unload.end(), tile), 0);
    apply(selector, plan);
    plan = selector.update(beyond, 6.9);
    EXPECT_EQ(std::count(plan.unload.begin(), plan.unload.end(), tile), 0);
    apply(selector, plan);
    plan = selector.update(beyond, 7.0);
    EXPECT_EQ(std::count(plan.unload.begin(), plan.unload.end(), tile), 1);
}

TEST(TileSelector, DeterministicModeIgnoresCamerasAndTime) {
    StreamingSettings s = settings();
    s.deterministic = true;
    const std::vector<Interest> withCamera{{Interest::Kind::Robot, {2000.0, 2000.0, 0.0}},
                                           {Interest::Kind::Camera, {7000.0, 7000.0, 0.0}}};
    const std::vector<Interest> withoutCamera{withCamera[0]};
    TileSelector a(pyramid(), s);
    TileSelector b(pyramid(), s);
    const StreamingPlan pa = a.update(withCamera, 0.0);
    const StreamingPlan pb = b.update(withoutCamera, 123.0);
    EXPECT_EQ(pa.load, pb.load);
    EXPECT_EQ(pa.unload, pb.unload);
    // Beyond the unload radius a deterministic run drops at once.
    apply(a, pa);
    const std::vector<Interest> moved{{Interest::Kind::Robot, {7000.0, 2000.0, 0.0}}};
    const StreamingPlan away = a.update(moved, 0.0);
    EXPECT_FALSE(away.unload.empty());
}

TEST(TileSelector, TheBudgetKeepsTheTilesARobotNeeds) {
    StreamingSettings s = settings();
    s.memoryBudget = 2000;  // bytes: a handful of 256 m tiles, no coarse ones
    TileSelector selector(pyramid(), s);
    const std::vector<Interest> interests{{Interest::Kind::Robot, {4000.0, 4000.0, 0.0}},
                                          {Interest::Kind::Sensor, {600.0, 600.0, 0.0}}};
    const StreamingPlan plan = selector.update(interests, 0.0);
    std::size_t needed = 0;
    for (const std::uint32_t t : plan.load) {
        const TileInfo& tile = selector.tiles()[t];
        if (tile.hasCollision &&
            horizontalDistance(interests[0].position, tile.boundsMin, tile.boundsMax) <= 600.0) {
            ++needed;
        } else {
            ADD_FAILURE() << tile.path << " loaded past the budget";
        }
    }
    EXPECT_GT(needed, 16U) << "the robot's tiles load even over the budget";
}

TEST(TileSelector, CollisionTilesStayWithinTheCollisionRadius) {
    StreamingSettings s = settings();
    s.collisionRadius = 200.0;
    TileSelector selector(pyramid(), s);
    const std::vector<Interest> robot{{Interest::Kind::Robot, {4000.0, 4000.0, 0.0}}};
    settle(selector, robot, 0.0);
    for (std::uint32_t i = 0; i < selector.tiles().size(); ++i) {
        const TileInfo& t = selector.tiles()[i];
        const double d = horizontalDistance(robot[0].position, t.boundsMin, t.boundsMax);
        if (t.hasCollision) {
            EXPECT_EQ(selector.isLoaded(i), d <= 200.0) << t.path;
        }
    }
    EXPECT_TRUE(covered(selector, robot[0].position, 600.0)) << "coarser tiles fill the rest";
    EXPECT_TRUE(selector.physicsReady(robot[0].position, 5.0));
}

}  // namespace
}  // namespace lagrange::streaming
