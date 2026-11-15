// Child processes of the builder (RFC 0016 §7.1, plan WP-F and R4): fork and exec with the
// output in a log file, waitpid(WNOHANG) once a frame, SIGTERM then SIGKILL after a grace.
#include "lagrange/viewer/builder/TestServer.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;  // NOLINT(readability-redundant-declaration): POSIX, the environment

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

    // fork and exec rather than posix_spawn (QA4 L2): the child asks the kernel for SIGTERM
    // when the viewer dies (PR_SET_PDEATHSIG), so a crashed or killed viewer leaves no test
    // server behind. Everything the child needs is made before fork; between fork and exec it
    // calls async-signal-safe functions only, and reports a failure through a close-on-exec
    // pipe.
    const pid_t parent = ::getpid();
    int report[2] = {-1, -1};
    if (::pipe2(report, O_CLOEXEC) != 0) {
        return std::string("could not start ") + args.front() + ": " + std::strerror(errno);
    }
    const pid_t pid = ::fork();
    if (pid < 0) {
        const int error = errno;
        ::close(report[0]);
        ::close(report[1]);
        return "could not start " + args.front() + ": " + std::strerror(error);
    }
    if (pid == 0) {
        const auto fail = [&](int stage) {
            const int error = errno;
            const int message[2] = {stage, error};
            (void)!::write(report[1], message, sizeof(message));
            ::_exit(127);
        };
        // A process group of its own: the stop reaches whatever the child starts, and a Ctrl C
        // in the viewer's terminal does not reach the child.
        ::setpgid(0, 0);
        if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0) {
            fail(1);
        }
        if (::getppid() != parent) {
            ::_exit(0);  // the viewer died before the request took hold
        }
        // The viewer may block or catch signals; the child starts with the defaults.
        struct sigaction defaults{};
        defaults.sa_handler = SIG_DFL;
        sigemptyset(&defaults.sa_mask);
        for (const int signal : {SIGTERM, SIGINT, SIGPIPE}) {
            ::sigaction(signal, &defaults, nullptr);
        }
        sigset_t none;
        sigemptyset(&none);
        ::sigprocmask(SIG_SETMASK, &none, nullptr);
        const int in = ::open("/dev/null", O_RDONLY);
        // The log is the user's own: never written through a link planted at its name (code
        // review L2); the start then fails with ELOOP.
        const int out = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW, 0600);
        if (in < 0 || out < 0 || ::dup2(in, STDIN_FILENO) < 0 || ::dup2(out, STDOUT_FILENO) < 0 ||
            ::dup2(out, STDERR_FILENO) < 0) {
            fail(2);
        }
        ::execve(argp.front(), argp.data(), envp.data());
        fail(3);
    }
    ::close(report[1]);
    int message[2] = {0, 0};
    ssize_t got = -1;
    do {
        got = ::read(report[0], message, sizeof(message));
    } while (got < 0 && errno == EINTR);
    ::close(report[0]);
    if (got == static_cast<ssize_t>(sizeof(message))) {
        ::waitpid(pid, nullptr, 0);  // it exits at once with 127
        return "could not start " + args.front() + ": " + std::strerror(message[1]);
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
    if (!request.overlay.empty()) {
        args.emplace_back("--overlay");
        args.push_back(request.overlay.string());
    }
    if (!request.spawn.empty()) {
        args.emplace_back("--spawn");
        args.push_back(request.spawn);
    }
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
