#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace lagrange::streaming {

/// One tile of a tiled world (WorldTileAPI_1, huge-worlds.md §2), as the selection sees it.
struct TileInfo {
    std::string path;                   ///< The tile prim's path, for the loader.
    std::string source;                 ///< The stage file the tile belongs to.
    std::int32_t level{0};              ///< Level of detail; 0 is the coarsest.
    std::array<double, 3> boundsMin{};  ///< World ENU, m.
    std::array<double, 3> boundsMax{};
    double geometricError{0.0};  ///< m.
    bool hasCollision{false};
    std::uint64_t bytes{0};  ///< Estimated memory once loaded (its payload's size).
};

/// Where a tile is wanted from: a robot, a ranged sensor, or a viewer camera.
struct Interest {
    enum class Kind : std::uint8_t {
        Robot,   ///< Needs the tiles it can touch with colliders, whatever their error.
        Sensor,  ///< Sees to `range`; wants the error its resolution needs.
        Camera,  ///< A viewer's camera: a hint, left out of deterministic runs.
    };
    Kind kind{Kind::Robot};
    std::array<double, 3> position{};  ///< World ENU, m.
};

struct StreamingSettings {
    double loadRadius{0.0};    ///< m: tiles nearer than this to an interest are loaded. Required.
    double unloadRadius{0.0};  ///< m: loaded tiles stay until farther than this (> loadRadius).
    /// m: tiles with colliders are loaded only this near an interest (physics needs them only
    /// around the robots, and they are the heavy ones); 0: the load radius. Coarser tiles
    /// without colliders cover the rest.
    double collisionRadius{0.0};
    /// A tile is fine enough at distance d when geometricError <= errorPerMetre * d. A robot
    /// takes the finest tiles within its reach whatever this says.
    double errorPerMetre{0.002};
    double unloadDelayS{5.0};       ///< Outside the unload radius this long before dropping.
    std::uint64_t memoryBudget{0};  ///< Bytes of loaded tiles; 0: no limit.
    /// Deterministic mode (invariant 7): the tile set is a pure function of the robots' and
    /// sensors' positions. Camera hints and the time delay are ignored.
    bool deterministic{false};
};

/// What the loader should do next. Loading is asynchronous: report a finished tile with
/// TileSelector::loaded().
struct StreamingPlan {
    std::vector<std::uint32_t> load;    ///< Tiles to start loading, nearest first.
    std::vector<std::uint32_t> unload;  ///< Loaded tiles to drop now.
};

/// Chooses which tiles of a tiled world to hold (huge-worlds.md §2). It knows nothing of USD:
/// the server gives it the tiles from the stage and the interests each update, and tells it
/// when a tile has finished loading.
///
/// Selection, per update:
/// - A tile is a candidate when its bounds are within the load radius of an interest.
/// - Robots want every candidate with colliders (physics needs the finest geometry); sensors
///   and cameras want the coarsest candidate whose error is small enough for their distance.
/// - A coarse tile is not wanted where finer wanted tiles cover its whole footprint, but it
///   stays loaded until those finer tiles are, so nothing is ever left without ground.
/// - A loaded tile is dropped only beyond the unload radius (and, outside deterministic mode,
///   after unloadDelayS there), or when the memory budget needs room: farthest first, never a
///   collision tile a robot needs.
class TileSelector {
public:
    TileSelector(std::vector<TileInfo> tiles, StreamingSettings settings);

    /// One selection at simulated time `timeS`.
    [[nodiscard]] StreamingPlan update(std::span<const Interest> interests, double timeS);

    /// The loader finished tile `id` (it is now in the world and, with colliders, in physics).
    void loaded(std::uint32_t id);
    /// The tile is out of the world again (after a plan's unload).
    void unloaded(std::uint32_t id);

    /// True when every collision tile under `position` (world ENU, horizontally, with
    /// `margin` m around it) is loaded: a robot may only go where physics has the ground.
    [[nodiscard]] bool physicsReady(const std::array<double, 3>& position, double margin) const;

    [[nodiscard]] const std::vector<TileInfo>& tiles() const { return tiles_; }
    [[nodiscard]] bool isLoaded(std::uint32_t id) const { return state_.at(id) == State::Loaded; }
    [[nodiscard]] bool isLoading(std::uint32_t id) const { return state_.at(id) == State::Loading; }
    [[nodiscard]] std::uint64_t loadedBytes() const;
    [[nodiscard]] std::size_t loadingCount() const;

private:
    enum class State : std::uint8_t { Unloaded, Loading, Loaded };
    std::vector<TileInfo> tiles_;
    StreamingSettings settings_;
    std::vector<State> state_;
    std::vector<double> farSince_;  // simulated time a loaded tile left the unload radius; < 0
    std::vector<std::vector<std::uint32_t>> children_;  // overlapping tiles one level finer
    std::vector<std::vector<std::uint32_t>> parents_;   // overlapping tiles one level coarser
};

/// The shortest distance from `point` to the box [lo, hi], horizontally (east, north).
[[nodiscard]] double horizontalDistance(const std::array<double, 3>& point,
                                        const std::array<double, 3>& lo,
                                        const std::array<double, 3>& hi);

}  // namespace lagrange::streaming
