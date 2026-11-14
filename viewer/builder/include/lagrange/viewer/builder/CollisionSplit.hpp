#pragma once

// "Split into convex parts" (RFC 0016 Addendum A, choice 2): CoACD run as `lagrange bake` runs
// it (lagrange_collision_bake's bake_urdf, same threshold and seed), as a child process polled
// with waitpid(WNOHANG), so the UI thread never waits (CLAUDE.md §8). The parts are written
// beside the robot's file in `<robot>_collision/`, as `<link>_0_<NNN>.obj`, and come back as
// absolute paths for setSplitParts (the URDF writer makes them relative on Save).
//
// CoACD is found as `lagrange bake` finds it: bake_urdf runs with the user's python3 and
// imports coacd from the system, a venv on PATH (docs/guide/install.md) or the user site. The
// child gets PYTHONUSERBASE=<the account's home>/.local when it is not set, so a user-site
// install is found even when $HOME points elsewhere (tests, sandboxes). Without coacd the split
// fails with bake_urdf's message ("No module named 'coacd'"), shown in the Collision section.

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

struct SplitRequest {
    std::string program;              ///< bake_urdf; empty: findBakeProgram()
    std::filesystem::path robotFile;  ///< the saved URDF: the parts go beside it
    std::string robot;                ///< the robot's name (the folder's)
    std::string link;                 ///< the link whose mesh is split
    std::string mesh;                 ///< the mesh as the document names it
    std::filesystem::path meshBase;   ///< the folder relative mesh paths are read against
    std::array<double, 3> scale{1, 1, 1};
    double threshold{0.05};  ///< CoACD's concavity threshold (bake.py's Settings default)
    int seed{0};             ///< CoACD's seed: the same seed gives the same parts
};

/// lib/lagrange_collision_bake/bake_urdf under $AMENT_PREFIX_PATH; nullopt when not installed.
[[nodiscard]] std::optional<std::filesystem::path> findBakeProgram();

class CollisionSplit {
public:
    enum class State { Idle, Running, Done, Failed };

    CollisionSplit();
    ~CollisionSplit();  ///< a running split is stopped (SIGTERM, then SIGKILL) and reaped
    CollisionSplit(const CollisionSplit&) = delete;
    CollisionSplit& operator=(const CollisionSplit&) = delete;

    /// Starts the split. Returns "" when started, else why not (one line): the robot is not
    /// saved, the mesh is not found, bake_urdf is not installed, one is already running.
    [[nodiscard]] std::string start(const SplitRequest& request);
    /// Non-blocking; call once a frame. On the exit of the child moves the parts into place
    /// (Done) or keeps the reason (Failed).
    State poll();
    /// SIGTERM to a running split; the next poll() reaps it (Failed, "cancelled").
    void cancel();

    [[nodiscard]] State state() const;
    [[nodiscard]] const std::string& link() const;
    /// With Done: the parts, absolute paths, in order.
    [[nodiscard]] const std::vector<std::string>& parts() const;
    /// With Failed: why (the tool's last lines).
    [[nodiscard]] const std::string& error() const;
    /// Seconds since start(), for "running · 4 s".
    [[nodiscard]] double seconds() const;
    /// Makes it Idle again (after its result was taken).
    void reset();

private:
    struct Process;
    std::unique_ptr<Process> process_;
};

}  // namespace lagrange::viewer::builder
