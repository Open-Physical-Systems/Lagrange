#pragma once

// The test server of the builder's own (RFC 0016 §7.1, R4). Owner: WP-F.
// posix_spawn, stderr to a log file, polled with waitpid(WNOHANG); never blocks the UI thread
// (CLAUDE.md §8). Not LocalServer: that one blocks in stop() and keeps no log.

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace lagrange::viewer::builder {

struct TestRequest {
    std::string program;         ///< absolute path of lagrange_server (the Launcher's lookup)
    std::filesystem::path urdf;  ///< the written test copy (absolute meshes)
    std::string world;           ///< "none", or a world file path
    std::string endpoint;        ///< "unix:<path>", plan decision 4
    std::filesystem::path log;   ///< stdout and stderr of the server go here (truncated)
};

class TestServer {
public:
    enum class State {
        Idle,      ///< never started, or exited and collected
        Running,   ///< started, not exited
        Stopping,  ///< SIGTERM sent, waiting; SIGKILL kStopGraceS after requestStop()
        Exited,    ///< exited; exitStatus() and logTail() tell how
    };
    /// Seconds between SIGTERM and SIGKILL (RFC §7.1 step 6).
    static constexpr double kStopGraceS = 3.0;

    TestServer();
    ~TestServer();  ///< stops a running server: SIGTERM, then SIGKILL after the grace
    TestServer(const TestServer&) = delete;
    TestServer& operator=(const TestServer&) = delete;

    /// `program --robot URDF --world WORLD --joint-control --listen ENDPOINT`. Returns "" when
    /// started, else why not (one line). Refused while a server is Running or Stopping.
    [[nodiscard]] std::string start(const TestRequest& request);
    /// Non-blocking: waitpid(WNOHANG); sends SIGKILL kStopGraceS after requestStop(). Call once
    /// a frame.
    State poll();
    /// SIGTERM. Returns at once; poll() finishes the stop.
    void requestStop();

    [[nodiscard]] State state() const;
    /// The wait status as waitpid gave it (WIFEXITED, ...); -1 before the server exited.
    [[nodiscard]] int exitStatus() const;
    /// The last `bytes` of the log file (a refused robot's message, RFC §7.1 step 5).
    [[nodiscard]] std::string logTail(std::size_t bytes) const;

private:
    struct Process;
    std::unique_ptr<Process> process_;
};

}  // namespace lagrange::viewer::builder
