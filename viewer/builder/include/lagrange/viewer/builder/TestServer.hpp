#pragma once

// The test server of the builder's own (RFC 0016 §7.1, R4), and the other child processes the
// builder runs (Export USD, Check loads, xacro). Owner: WP-F.
// posix_spawn, stdout and stderr to a log file, polled with waitpid(WNOHANG); never blocks the UI
// thread (CLAUDE.md §8). Not LocalServer: that one blocks in stop() and keeps no log.

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::viewer::builder {

/// One child process in a process group of its own, its output in a log file. The UI thread
/// polls it once a frame; nothing here waits.
class ChildProcess {
public:
    enum class State {
        Idle,      ///< never started
        Running,   ///< started, not exited
        Stopping,  ///< SIGTERM sent, waiting; SIGKILL kStopGraceS after requestStop()
        Exited,    ///< exited and collected; exitStatus() and logTail() tell how
    };
    /// Seconds between SIGTERM and SIGKILL (RFC §7.1 step 6).
    static constexpr double kStopGraceS = 3.0;

    ChildProcess();
    /// Stops a running child: SIGTERM, then SIGKILL after the grace. This one waits (at most
    /// kStopGraceS), as it runs only when the viewer closes.
    ~ChildProcess();
    ChildProcess(const ChildProcess&) = delete;
    ChildProcess& operator=(const ChildProcess&) = delete;

    /// Starts `argv` (argv[0] an absolute path) with stdout and stderr to `log` (truncated) and
    /// stdin from /dev/null. `environment` entries "NAME=VALUE" replace or add to the viewer's.
    /// Returns "" when started, else why not (one line). Refused while Running or Stopping.
    [[nodiscard]] std::string start(const std::vector<std::string>& argv,
                                    const std::filesystem::path& log,
                                    const std::vector<std::string>& environment = {});
    /// Non-blocking: waitpid(WNOHANG); sends SIGKILL kStopGraceS after requestStop().
    State poll();
    /// SIGTERM to the child's process group. Returns at once; poll() finishes the stop.
    void requestStop();

    [[nodiscard]] State state() const;
    /// The wait status as waitpid gave it (WIFEXITED, ...); -1 before the child exited.
    [[nodiscard]] int exitStatus() const;
    /// "exit code 3", "killed by signal 9": the exit status in words; empty before it exited.
    [[nodiscard]] std::string exitText() const;
    /// True when it exited by itself with code 0.
    [[nodiscard]] bool succeeded() const;
    /// The last `bytes` of the log file, from the start of a line where one starts in them.
    [[nodiscard]] std::string logTail(std::size_t bytes) const;
    [[nodiscard]] const std::filesystem::path& log() const;
    [[nodiscard]] int pid() const;

private:
    struct Process;
    std::unique_ptr<Process> process_;
};

struct TestRequest {
    std::string program;         ///< absolute path of lagrange_server (the Launcher's lookup)
    std::filesystem::path urdf;  ///< the written test copy (absolute meshes)
    /// The overlay's test copy (RFC 0016 phase 3: power system, overlay-only sensors); empty:
    /// the robot has none.
    std::filesystem::path overlay;
    std::string world;          ///< "none", or a world file path
    std::string endpoint;       ///< "unix:<path>", plan decision 4
    std::filesystem::path log;  ///< stdout and stderr of the server go here (truncated)
    /// ROS for the test server (plan R8): a ROS_DOMAIN_ID of its own ("69"), so a teleop on the
    /// user's domain drives only the run it was meant for; empty: `--no-ros`.
    std::string rosDomain;
};

class TestServer {
public:
    using State = ChildProcess::State;
    static constexpr double kStopGraceS = ChildProcess::kStopGraceS;

    /// `program --robot URDF --world WORLD --joint-control --listen ENDPOINT [--overlay FILE]
    /// [--no-ros]`.
    [[nodiscard]] static std::vector<std::string> arguments(const TestRequest& request);
    /// The environment the server gets on top of the viewer's: ROS_DOMAIN_ID when it has ROS.
    [[nodiscard]] static std::vector<std::string> environment(const TestRequest& request);

    /// Starts the server for `request`. Returns "" when started, else why not (one line).
    /// Refused while a server is Running or Stopping.
    [[nodiscard]] std::string start(const TestRequest& request);
    /// Starts any command as the test server would be (the tests use /bin/sleep).
    [[nodiscard]] std::string startCommand(const std::vector<std::string>& argv,
                                           const std::filesystem::path& log,
                                           const std::vector<std::string>& environment = {});
    /// Non-blocking; call once a frame.
    State poll() { return process_.poll(); }
    /// SIGTERM. Returns at once; poll() finishes the stop (SIGKILL after kStopGraceS).
    void requestStop() { process_.requestStop(); }

    [[nodiscard]] State state() const { return process_.state(); }
    [[nodiscard]] int exitStatus() const { return process_.exitStatus(); }
    [[nodiscard]] std::string exitText() const { return process_.exitText(); }
    /// The last `bytes` of the log file (a refused robot's message, RFC §7.1 step 5).
    [[nodiscard]] std::string logTail(std::size_t bytes) const { return process_.logTail(bytes); }

private:
    ChildProcess process_;
};

}  // namespace lagrange::viewer::builder
