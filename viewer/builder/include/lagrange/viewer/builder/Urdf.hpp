#pragma once

// URDF writer and reader of the builder (RFC 0016 §6, §10). Owner: WP-B. tinyxml2 only; the
// server's loader stays the judge (Test and the round-trip tests load what this writes).

#include "lagrange/viewer/builder/Document.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

struct WriteOptions {
    /// Where the file will be saved: relative mesh paths are written relative to its folder.
    /// Empty: mesh paths are written as the document holds them.
    std::filesystem::path savedAt;
    /// Write every relative mesh path as an absolute one (the Test copy, RFC §7.1 step 2).
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

/// Writes `text` to `path` atomically: a temporary file in the same folder, flushed, then
/// renamed over `path`. Refuses a path ending in `.xacro` (RFC §10). Returns "" on success,
/// else why not (one line, with the path).
[[nodiscard]] std::string saveAtomically(const std::filesystem::path& path, std::string_view text);

}  // namespace lagrange::viewer::builder
