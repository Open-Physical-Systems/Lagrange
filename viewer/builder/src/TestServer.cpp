// Child processes of the builder (RFC 0016 §7.1, plan WP-F and R4): posix_spawn with the
// output in a log file, waitpid(WNOHANG) once a frame, SIGTERM then SIGKILL after a grace.
#include "lagrange/viewer/builder/TestServer.hpp"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;  // NOLINT(readability-redundant-declaration): POSIX, for posix_spawn

namespace lagrange::viewer::builder {

namespace {
using Clock = std::chrono::steady_clock;
}  // namespace

struct ChildProcess::Process {
    State state{State::Idle};
    pid_t pid{-1};
    int status{-1};
    Clock::time_point stopAt{};
    bool killed{false};
    std::filesystem::path log;
};

ChildProcess::ChildProcess() : process_(std::make_unique<Process>()) {}

ChildProcess::~ChildProcess() {
    if (process_->state != State::Running && process_->state != State::Stopping) {
        return;
    }
    requestStop();
    // Only when the viewer closes: wait out the grace, then SIGKILL, then collect.
    const auto deadline = Clock::now() + std::chrono::duration<double>(kStopGraceS + 1.0);
    while (poll() == State::Stopping && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::string ChildProcess::start(const std::vector<std::string>& argv,
                                const std::filesystem::path& log,
                                const std::vector<std::string>& environment) {
    Process& p = *process_;
    if (p.state == State::Running || p.state == State::Stopping) {
        return "a process is still running";
    }
    if (argv.empty()) {
        return "nothing to run";
    }
    // The environment: the viewer's, with `environment` replacing or adding entries by name.
    std::map<std::string, std::string> merged;
    std::vector<std::string> order;
    const auto put = [&](const std::string& entry) {
        const std::size_t eq = entry.find('=');
        const std::string name = entry.substr(0, eq);
        if (!merged.contains(name)) {
            order.push_back(name);
        }
        merged[name] = entry;
    };
    for (char** e = environ; e != nullptr && *e != nullptr; ++e) {
        put(*e);
    }
    for (const std::string& entry : environment) {
        put(entry);
    }
    std::vector<std::string> envStrings;
    envStrings.reserve(order.size());
    for (const std::string& name : order) {
        envStrings.push_back(merged[name]);
    }
    std::vector<char*> envp;
    for (std::string& entry : envStrings) {
        envp.push_back(entry.data());
    }
    envp.push_back(nullptr);
    std::vector<std::string> args = argv;
    std::vector<char*> argp;
    for (std::string& arg : args) {
        argp.push_back(arg.data());
    }
    argp.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    // A process group of its own: the stop reaches whatever the child starts, and a Ctrl C in
    // the viewer's terminal does not reach the child.
    posix_spawnattr_setpgroup(&attributes, 0);
    // The viewer may block or ignore signals; the child starts with the defaults.
    sigset_t none;
    sigemptyset(&none);
    posix_spawnattr_setsigmask(&attributes, &none);
    sigset_t defaults;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGTERM);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGPIPE);
    posix_spawnattr_setsigdefault(&attributes, &defaults);
    posix_spawnattr_setflags(
        &attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);
    pid_t pid = -1;
    const int error =
        ::posix_spawn(&pid, args.front().c_str(), &actions, &attributes, argp.data(), envp.data());
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    if (error != 0) {
        return "could not start " + args.front() + ": " + std::strerror(error);
    }
    p = Process{};
    p.state = State::Running;
    p.pid = pid;
    p.log = log;
    return {};
}

ChildProcess::State ChildProcess::poll() {
    Process& p = *process_;
    if (p.state != State::Running && p.state != State::Stopping) {
        return p.state;
    }
    int status = 0;
    const pid_t done = ::waitpid(p.pid, &status, WNOHANG);
    if (done == p.pid) {
        p.status = status;
        p.state = State::Exited;
        p.pid = -1;
        return p.state;
    }
    if (done < 0) {  // not our child any more (ECHILD): nothing to wait for
        p.state = State::Exited;
        p.pid = -1;
        return p.state;
    }
    if (p.state == State::Stopping && !p.killed && Clock::now() >= p.stopAt) {
        ::kill(-p.pid, SIGKILL);
        p.killed = true;
    }
    return p.state;
}

void ChildProcess::requestStop() {
    Process& p = *process_;
    if (p.state != State::Running) {
        return;
    }
    ::kill(-p.pid, SIGTERM);
    p.state = State::Stopping;
    p.stopAt = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                  std::chrono::duration<double>(kStopGraceS));
}

ChildProcess::State ChildProcess::state() const {
    return process_->state;
}

int ChildProcess::exitStatus() const {
    return process_->status;
}

std::string ChildProcess::exitText() const {
    const int status = process_->status;
    if (process_->state != State::Exited || status < 0) {
        return {};
    }
    if (WIFEXITED(status)) {
        return "exit code " + std::to_string(WEXITSTATUS(status));
    }
    if (WIFSIGNALED(status)) {
        return "killed by signal " + std::to_string(WTERMSIG(status)) + " (" +
               ::strsignal(WTERMSIG(status)) + ")";
    }
    return "status " + std::to_string(status);
}

bool ChildProcess::succeeded() const {
    const int status = process_->status;
    return process_->state == State::Exited && status >= 0 && WIFEXITED(status) &&
           WEXITSTATUS(status) == 0;
}

std::string ChildProcess::logTail(std::size_t bytes) const {
    if (process_->log.empty()) {
        return {};
    }
    std::ifstream in(process_->log, std::ios::binary | std::ios::ate);
    if (!in) {
        return {};
    }
    const std::streamoff size = in.tellg();
    const std::streamoff from = size > static_cast<std::streamoff>(bytes)
                                    ? size - static_cast<std::streamoff>(bytes)
                                    : std::streamoff{0};
    in.seekg(from);
    std::string text(static_cast<std::size_t>(size - from), '\0');
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    text.resize(static_cast<std::size_t>(in.gcount()));
    if (from > 0) {
        const std::size_t line = text.find('\n');
        if (line != std::string::npos && line + 1 < text.size()) {
            text.erase(0, line + 1);
        }
    }
    return text;
}

const std::filesystem::path& ChildProcess::log() const {
    return process_->log;
}

int ChildProcess::pid() const {
    return process_->pid;
}

std::vector<std::string> TestServer::arguments(const TestRequest& request) {
    std::vector<std::string> args{request.program,
                                  "--robot",
                                  request.urdf.string(),
                                  "--world",
                                  request.world.empty() ? std::string("none") : request.world,
                                  "--joint-control",
                                  "--listen",
                                  request.endpoint};
    if (request.rosDomain.empty()) {
        args.emplace_back("--no-ros");
    }
    return args;
}

std::vector<std::string> TestServer::environment(const TestRequest& request) {
    if (request.rosDomain.empty()) {
        return {};
    }
    return {"ROS_DOMAIN_ID=" + request.rosDomain};
}

std::string TestServer::start(const TestRequest& request) {
    if (request.program.empty()) {
        return "lagrange_server is not installed";
    }
    return process_.start(arguments(request), request.log, environment(request));
}

std::string TestServer::startCommand(const std::vector<std::string>& argv,
                                     const std::filesystem::path& log,
                                     const std::vector<std::string>& environment) {
    return process_.start(argv, log, environment);
}

}  // namespace lagrange::viewer::builder
