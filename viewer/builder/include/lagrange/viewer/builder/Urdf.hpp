#pragma once

// URDF writer and reader of the builder (RFC 0016 §6, §10). Owner: WP-B. tinyxml2 only; the
// server's loader stays the judge (Test and the round-trip tests load what this writes).
//
// What URDF has no place for is written as attributes the server's loader (urdfdom, and the
// <gazebo> reader in server/src/Robot.cpp) ignores:
// - `<link lagrange_density="2700">`: Inertial::density, kg/m³.
// - `<inertial lagrange_overrides="mass com inertia">`: for a link with a density, the parts of
//   the <inertial> that are the user's overrides; the others were computed, and are computed
//   again when the file is read. A file without it (any other URDF) has all three as overrides.
// - `<visual|collision lagrange_solid="true|false">`: Shape::solid, only when it is not the
//   default (true for a collision shape, false for a visual-only one).
// Colours are robot-level <material> definitions: the file's own, and `builder_color_<k>` for a
// colour no definition has.
//
// keptXml holds placement markers among the kept elements: comments
// `<!--lagrange-builder:link-->` (also joint, sensor in the robot's; inertial, visual, collision in
// a link's; robot: what stood before <robot>). A marker stands for "the next item of that kind in
// document order", so kept elements are written back where they were. Items without a marker
// (added in the builder) follow the last marker of their kind. keptForDisplay() hides them.
//
// Sensor::params: every leaf under <sensor> by its path, attributes as "<path>/@<name>"
// ("imu/angular_velocity/z/noise/@type"). A kind's wrapper element (<rangefinder>) is not in the
// path, as the server reads it (plan decision 6); a built-in type's is ("lidar/range/max"). An
// IMU's x, y and z with the same parameters read as the catalog's "{axis}", written on all three.

#include "lagrange/viewer/builder/Document.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

struct WriteOptions {
    /// Where the file will be saved. Absolute mesh paths are written relative to its folder;
    /// relative ones stay as the document holds them. Empty: every path as the document holds it.
    std::filesystem::path savedAt;
    /// Write every relative mesh path as an absolute one, resolved against savedAt's folder (the
    /// user's file, while the text goes to the Test copy, RFC §7.1 step 2).
    /// `lagrange-asset://` and `package://` stay as written either way.
    bool absoluteMeshes{false};
};

/// The document as URDF text: deterministic (document order, numbers through std::to_chars in
/// shortest round-trip form), kept XML (keptXml) written back in place, each sensor as
/// `<gazebo reference="LINK"><sensor name= type=>` with `<pose>`, `<update_rate>`, `<latency>`,
/// `<jitter>` and its parameters under the catalog entry's wrapper element (plan decision 6).
/// Writes whatever the document holds, valid or not (Save works on an unfinished robot).
[[nodiscard]] std::string writeUrdf(const Document& document, const WriteOptions& options = {});

struct ReadResult {
    std::optional<Document> document;  ///< nullopt on an error
    std::string error;                 ///< why not, one line; empty on success
    int errorLine{0};                  ///< 1-based line of the error in the text; 0 if none
    /// What was read but changed or dropped (one line each): an `<inertial>` kept as an
    /// override, a sensor of an unknown type kept as a free key/value form, ...
    std::vector<std::string> notes;
};

/// Reads URDF text (not xacro: expand it first, RFC §10). What the document models goes into
/// it; every other element is kept verbatim in keptXml of the robot, link or joint it was in.
[[nodiscard]] ReadResult readUrdf(std::string_view text);

/// keptXml without its placement markers, for "Kept as written" (RFC §10).
[[nodiscard]] std::string keptForDisplay(std::string_view keptXml);

/// Writes `text` to `path` atomically: a temporary file in the same folder, flushed, then
/// renamed over `path`. Refuses a path ending in `.xacro` (RFC §10). Returns "" on success,
/// else why not (one line, with the path).
[[nodiscard]] std::string saveAtomically(const std::filesystem::path& path, std::string_view text);

}  // namespace lagrange::viewer::builder
