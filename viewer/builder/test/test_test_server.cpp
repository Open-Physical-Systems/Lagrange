// The builder's child processes (RFC 0016 §7.1, plan WP-F): started, polled without blocking,
// stopped with SIGTERM and SIGKILL after the grace, and their log kept. /bin/sleep and /bin/sh
// stand in for lagrange_server.
#include "lagrange/viewer/builder/TestServer.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <signal.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>

using lagrange::viewer::builder::ChildProcess;
using lagrange::viewer::builder::TestRequest;
using lagrange::viewer::builder::TestServer;
using State = TestServer::State;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {

class Scratch {
public:
    Scratch() {
        std::string pattern = (std::filesystem::temp_directory_path() / "lgr-ts-XXXXXX").string();
        path_ = ::mkdtemp(pattern.data());
    }
    ~Scratch() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    Scratch(const Scratch&) = delete;
    Scratch& operator=(const Scratch&) = delete;
    [[nodiscard]] std::filesystem::path operator/(const char* name) const { return path_ / name; }

private:
    std::filesystem::path path_;
};

/// Polls as the frame loop does, every 10 ms, until `state` or the timeout. Returns how long.
template <typename Process>
std::chrono::duration<double> pollUntil(Process& process, State state, std::chrono::seconds limit) {
    const auto start = Clock::now();
    while (process.poll() != state && Clock::now() - start < limit) {
        std::this_thread::sleep_for(10ms);
    }
    return Clock::now() - start;
}

}  // namespace

TEST(TestServer, ArgumentsAreTheRfcCommandLine) {
    TestRequest request;
    request.program = "/opt/x/lagrange_server";
    request.urdf = "/run/user/1/lagrange/builder/7/rover.urdf";
    request.world = "none";
    request.endpoint = "unix:/run/user/1/lagrange/builder-7.sock";
    request.log = "/tmp/log";
    const std::vector<std::string> expected{"/opt/x/lagrange_server",
                                            "--robot",
                                            "/run/user/1/lagrange/builder/7/rover.urdf",
                                            "--world",
                                            "none",
                                            "--joint-control",
                                            "--listen",
                                            "unix:/run/user/1/lagrange/builder-7.sock",
                                            "--no-ros"};
    EXPECT_EQ(TestServer::arguments(request), expected);
    EXPECT_TRUE(TestServer::environment(request).empty());
    // With a ROS domain of its own the server keeps ROS, on that domain (plan R8).
    request.rosDomain = "69";
    EXPECT_EQ(TestServer::arguments(request).back(), "unix:/run/user/1/lagrange/builder-7.sock");
    EXPECT_EQ(TestServer::environment(request), std::vector<std::string>{"ROS_DOMAIN_ID=69"});
    TestServer server;
    request.program.clear();
    EXPECT_EQ(server.start(request), "lagrange_server is not installed");
    EXPECT_EQ(server.state(), State::Idle);
}

TEST(TestServer, StartsPollsAndStops) {
    Scratch scratch;
    TestServer server;
    EXPECT_EQ(server.state(), State::Idle);
    EXPECT_EQ(server.exitStatus(), -1);
    ASSERT_EQ(server.startCommand({"/bin/sleep", "30"}, scratch / "sleep.log"), "");
    EXPECT_EQ(server.poll(), State::Running);
    // A second start while one runs is refused.
    EXPECT_FALSE(server.startCommand({"/bin/sleep", "30"}, scratch / "other.log").empty());
    // poll() returns at once while the process runs: a frame is never held up.
    const auto before = Clock::now();
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(server.poll(), State::Running);
    }
    EXPECT_LT(Clock::now() - before, 100ms);
    server.requestStop();
    EXPECT_EQ(server.state(), State::Stopping);
    const auto took = pollUntil(server, State::Exited, 5s);
    ASSERT_EQ(server.state(), State::Exited);
    EXPECT_LT(took, 2s);  // sleep obeys SIGTERM: no wait for the grace
    ASSERT_TRUE(WIFSIGNALED(server.exitStatus()));
    EXPECT_EQ(WTERMSIG(server.exitStatus()), SIGTERM);
    EXPECT_NE(server.exitText().find("signal 15"), std::string::npos);
    // It can be started again once it exited.
    ASSERT_EQ(server.startCommand({"/bin/sleep", "0"}, scratch / "again.log"), "");
    (void)pollUntil(server, State::Exited, 5s);
    EXPECT_TRUE(WIFEXITED(server.exitStatus()));
}

TEST(TestServer, KillsAfterGrace) {
    Scratch scratch;
    TestServer server;
    // A server that ignores SIGTERM (an ignored signal stays ignored across exec).
    ASSERT_EQ(server.startCommand({"/bin/sh", "-c", "trap '' TERM; exec sleep 30"},
                                  scratch / "stubborn.log"),
              "");
    std::this_thread::sleep_for(200ms);  // let the shell set its trap
    server.requestStop();
    const auto took = pollUntil(server, State::Exited, 10s);
    ASSERT_EQ(server.state(), State::Exited);
    EXPECT_GE(took.count(), TestServer::kStopGraceS - 0.2);
    EXPECT_LT(took.count(), TestServer::kStopGraceS + 2.0);
    ASSERT_TRUE(WIFSIGNALED(server.exitStatus()));
    EXPECT_EQ(WTERMSIG(server.exitStatus()), SIGKILL);
}

TEST(TestServer, LogTailCaptured) {
    Scratch scratch;
    TestServer server;
    // stdout and stderr both land in the log; the tail starts at a line.
    ASSERT_EQ(server.startCommand({"/bin/sh", "-c",
                                   "i=0; while [ $i -lt 200 ]; do echo line $i; i=$((i+1)); done; "
                                   "echo 'error: the robot has two roots' >&2; exit 3"},
                                  scratch / "refused.log"),
              "");
    (void)pollUntil(server, State::Exited, 5s);
    ASSERT_EQ(server.state(), State::Exited);
    ASSERT_TRUE(WIFEXITED(server.exitStatus()));
    EXPECT_EQ(WEXITSTATUS(server.exitStatus()), 3);
    EXPECT_EQ(server.exitText(), "exit code 3");
    const std::string tail = server.logTail(64);
    EXPECT_LE(tail.size(), 64U);
    EXPECT_NE(tail.find("error: the robot has two roots"), std::string::npos);
    EXPECT_TRUE(tail.starts_with("line ")) << tail;  // whole lines only
    EXPECT_NE(server.logTail(100'000).find("line 0\n"), std::string::npos);
}

TEST(ChildProcess, EnvironmentIsTheViewersPlusItsOwn) {
    Scratch scratch;
    ::setenv("LAGRANGE_TEST_INHERITED", "kept", 1);
    ChildProcess child;
    ASSERT_EQ(child.start({"/bin/sh", "-c", "echo $LAGRANGE_TEST_INHERITED $ROS_DOMAIN_ID"},
                          scratch / "env.log", {"ROS_DOMAIN_ID=42"}),
              "");
    (void)pollUntil(child, State::Exited, 5s);
    EXPECT_TRUE(child.succeeded());
    EXPECT_EQ(child.logTail(100), "kept 42\n");
}

TEST(ChildProcess, AMissingProgramIsReported) {
    Scratch scratch;
    ChildProcess child;
    const std::string error = child.start({"/nonexistent/lagrange_server"}, scratch / "x.log");
    // posix_spawn reports it, or the child exits with 127: either way it does not "run".
    if (error.empty()) {
        (void)pollUntil(child, State::Exited, 5s);
        EXPECT_FALSE(child.succeeded());
    } else {
        EXPECT_NE(error.find("/nonexistent/lagrange_server"), std::string::npos);
    }
}
