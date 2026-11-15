#pragma once

// The builder's own files outside the document (code review of 2026-11-15, L2, L6, L7): the
// private runtime folder, the autosaves, and the name Save as… writes to. No ImGui, no
// processes: the UI thread calls these and tests call them directly.

#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

/// Makes `path` a folder of this user's own: created with mode 0700 when missing; refused (the
/// reason, one line) when it is a symlink, not a folder, or owned by another user. `exclusive`:
/// group and others lose every permission (0700); else only their write permission (a shared
/// parent such as $XDG_RUNTIME_DIR/lagrange). The parent must exist. "" on success.
[[nodiscard]] std::string makePrivateDirectory(const std::filesystem::path& path, bool exclusive);

/// `name` as a file name stem that is safe and still tells names apart: safeFileName(name), and
/// when that changed anything, '-' and 8 hex digits of a hash of `name`, so that "a/b" and
/// "a_b" do not share files. Never empty, never '.' first, never a '/'.
[[nodiscard]] std::string distinctFileName(std::string_view name);

/// 16 hex digits of a 64-bit FNV-1a hash of `text` (not a security hash: a file key).
[[nodiscard]] std::string fileKey(std::string_view text);

/// $XDG_STATE_HOME/lagrange/builder (else ~/.local/state/lagrange/builder): the autosaves
/// folder (RFC 0016 §10).
[[nodiscard]] std::filesystem::path autosaveFolder();

/// The autosave of a document (RFC 0016 §10) in `folder`, one per document and viewer:
/// `<robot>-<key>.<pid>.autosave.urdf`, where key is fileKey() of the document's absolute file
/// path, or "new" for a document that has no file yet. Two viewers never share one, and two
/// files of the same robot name neither.
[[nodiscard]] std::filesystem::path autosaveFile(const std::filesystem::path& folder,
                                                 std::string_view robot,
                                                 const std::filesystem::path& document, int pid);

/// What autosaves of a document were left in `folder` by viewers no longer running (`alive`
/// says whether a pid runs; this viewer's `pid` is never looked at).
struct LeftAutosaves {
    /// Newer than the document's file (or it has no file): offered back, newest first.
    std::vector<std::filesystem::path> recoverable;
    /// Not newer than the document's file: the file holds what they hold or more; to remove.
    std::vector<std::filesystem::path> stale;
};
[[nodiscard]] LeftAutosaves findLeftAutosaves(const std::filesystem::path& folder,
                                              std::string_view robot,
                                              const std::filesystem::path& document, int pid,
                                              const std::function<bool(int)>& alive);

/// True when a process with this pid runs (kill(pid, 0)).
[[nodiscard]] bool processAlive(int pid);

/// Save as…: the file dialog's choice with ".urdf" added when it lacks it. `confirm` is true
/// when that added the suffix and a file of the new name exists: the dialog asked about the
/// name without it, so the overwrite was never confirmed (L7).
struct SaveAsTarget {
    std::filesystem::path path;
    bool confirm{false};
};
[[nodiscard]] SaveAsTarget saveAsTarget(const std::filesystem::path& chosen);

/// Test into a sample world (SPEC "Modes": Test's menu): the worlds of lagrange_assets'
/// `worlds` folder, one per subfolder, by name: its `<name>.usda` (else .usd, .sdf, .urdf), else
/// its one world file, else its first .usda. Sorted by name; empty when the folder is missing.
struct SampleWorld {
    std::string name;  ///< "warehouse_small"
    std::filesystem::path file;
};
[[nodiscard]] std::vector<SampleWorld> sampleWorlds(const std::filesystem::path& worlds);

/// True for a file Test can run as a world: .usda, .usd, .usdc, .sdf, .world, .urdf.
[[nodiscard]] bool isWorldFile(const std::filesystem::path& path);

}  // namespace lagrange::viewer::builder
