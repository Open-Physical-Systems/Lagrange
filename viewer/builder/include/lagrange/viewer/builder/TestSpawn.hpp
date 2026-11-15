#pragma once

// Where Test starts the robot (Test ▾ → Start at…): a pose per world, the sample worlds' known
// good places as defaults, remembered per world in the viewer's config folder, and passed to
// the test server as `--spawn X,Y,Z,YAW` (CommandLine.cpp: m, world ENU, heading in rad; for a
// URDF robot z is where its lowest point goes). No ImGui, no processes.

#include <filesystem>
#include <map>
#include <string>

namespace lagrange::viewer::builder {

/// A start pose as the user types it: metres in the world's frame, heading in degrees about up
/// (0: the robot's x along the world's x).
struct SpawnPose {
    double x{0.0};       // m
    double y{0.0};       // m
    double z{0.0};       // m
    double yawDeg{0.0};  // °
    bool operator==(const SpawnPose&) const = default;
    /// The world's origin, heading 0: the server's own default, so no --spawn is passed.
    [[nodiscard]] bool isOrigin() const { return *this == SpawnPose{}; }
};

/// The key a world is remembered by: "none" for the empty world, else the world file's path.
[[nodiscard]] std::string spawnKey(const std::string& world);

/// The start pose a world gets before the user chose one: for the sample worlds whose sample
/// runs start the robot elsewhere (tools/cli/lagrange's SAMPLES, the huge-worlds tutorial) that
/// place, by the world's folder name; else the origin.
///   tiled_town          1036, 1036        (docs/guide/tutorial-huge-worlds.md)
///   amr_fms_warehouse   26, 8, 0, 180°    (`lagrange swerve`)
///   town                30, -1.75         (`lagrange drive`, the lane by the junction)
[[nodiscard]] SpawnPose defaultSpawn(const std::string& world);

/// The value of `--spawn`: "X,Y,Z,YAW" with the heading in radians; "" for the origin (no flag).
[[nodiscard]] std::string spawnArgument(const SpawnPose& pose);

/// "26, 8, 0 · 180°": the pose as the Test menu shows it.
[[nodiscard]] std::string spawnLabel(const SpawnPose& pose);

/// The poses the user chose, per world, in `<config>/builder_spawns.tsv` (one line per world:
/// x, y, z, yaw in degrees, then the key; '#' starts a comment). A world without a line gets
/// defaultSpawn.
class SpawnMemory {
public:
    /// Reads `file` (missing or unreadable: nothing remembered). Lines that do not parse are
    /// skipped.
    void load(const std::filesystem::path& file);
    /// Writes every remembered pose to the file given to load(); false when it could not.
    bool save() const;
    [[nodiscard]] SpawnPose at(const std::string& world) const;
    /// Remembers `pose` for `world` and saves when a file was loaded.
    void set(const std::string& world, const SpawnPose& pose);
    /// Forgets the choice: the world gets its default again.
    void reset(const std::string& world);
    [[nodiscard]] bool loaded() const { return !file_.empty(); }

private:
    std::filesystem::path file_;
    std::map<std::string, SpawnPose> poses_;
};

}  // namespace lagrange::viewer::builder
