// "Split into convex parts" as a polled child process (RFC 0016 Addendum A). See
// CollisionSplit.hpp.
//
// The split runs bake_urdf (`lagrange bake`) on a one-link URDF that names the mesh by its
// absolute path, in a work folder beside the robot's file: bake_urdf writes the parts into
// `<work>/<robot>_collision/<link>_0_<NNN>.obj` (already scaled, as lagrange_mesh_to_obj scales
// them), and on success they are moved to `<robot dir>/<robot>_collision/`.
#include "lagrange/meshes/MeshImport.hpp"
#include "lagrange/viewer/builder/CollisionSplit.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <pwd.h>
#include <regex>
#include <spawn.h>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;  // NOLINT(readability-redundant-declaration): POSIX, for posix_spawn

namespace lagrange::viewer::builder {

namespace {

using Clock = std::chrono::steady_clock;

std::string number(double value) {
    std::ostringstream out;
    out.precision(17);
    out << value;
    return out.str();
}

std::string xmlEscaped(const std::string& text) {
    std::string out;
    for (const char c : text) {
        switch (c) {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            default:
                out += c;
        }
    }
    return out;
}

std::string tail(const std::filesystem::path& file, std::size_t bytes) {
    std::ifstream in(file, std::ios::binary);
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::string out = text.size() > bytes ? text.substr(text.size() - bytes) : text;
    while (!out.empty() && (out.back() == '\n' || out.back() == ' ')) {
        out.pop_back();
    }
    return out;
}

/// The child's environment: the viewer's, plus PYTHONUSERBASE when it is not set. bake_urdf
/// imports coacd as `lagrange bake` does, with the user's Python: the system's, a venv's
/// (install.md), or the user site (`pip install --user coacd`, ~/.local). Python finds the user
/// site through $HOME, which a test or a sandbox may point elsewhere; PYTHONUSERBASE set to the
/// account's own home (getpwuid) keeps finding it there.
std::vector<std::string> childEnvironment() {
    std::vector<std::string> out;
    bool userBase = false;
    for (char** e = environ; e != nullptr && *e != nullptr; ++e) {
        out.emplace_back(*e);
        userBase = userBase || out.back().starts_with("PYTHONUSERBASE=");
    }
    if (!userBase) {
        // NOLINTNEXTLINE(concurrency-mt-unsafe): read once, on the UI thread
        if (const passwd* account = ::getpwuid(::getuid());
            account != nullptr && account->pw_dir != nullptr) {
            out.push_back(std::string("PYTHONUSERBASE=") + account->pw_dir + "/.local");
        }
    }
    return out;
}

}  // namespace

std::optional<std::filesystem::path> findBakeProgram() {
    const char* prefixes = std::getenv("AMENT_PREFIX_PATH");  // NOLINT(concurrency-mt-unsafe)
    std::istringstream list(prefixes != nullptr ? prefixes : "");
    for (std::string prefix; std::getline(list, prefix, ':');) {
        const std::filesystem::path program =
            std::filesystem::path(prefix) / "lib" / "lagrange_collision_bake" / "bake_urdf";
        std::error_code error;
        if (!prefix.empty() && std::filesystem::is_regular_file(program, error)) {
            return program;
        }
    }
    return std::nullopt;
}

struct CollisionSplit::Process {
    State state{State::Idle};
    pid_t pid{-1};
    SplitRequest request;
    std::filesystem::path work;  // the scratch folder beside the robot's file
    std::filesystem::path log;
    std::vector<std::string> parts;
    std::string error;
    Clock::time_point started;
    bool cancelled{false};

    void fail(std::string why) {
        state = State::Failed;
        error = std::move(why);
        std::error_code ignored;
        std::filesystem::remove_all(work, ignored);
    }

    void finish() {
        const std::filesystem::path from = work / (request.robot + "_collision");
        const std::filesystem::path to =
            request.robotFile.parent_path() / (request.robot + "_collision");
        std::error_code failure;
        std::filesystem::create_directories(to, failure);
        if (failure) {
            fail("cannot make " + to.string() + ": " + failure.message());
            return;
        }
        // The link's earlier parts go; another link's stay.
        const std::regex mine(request.link + R"(_\d+_\d{3}\.obj)");
        for (const auto& entry : std::filesystem::directory_iterator(to, failure)) {
            if (std::regex_match(entry.path().filename().string(), mine)) {
                std::filesystem::remove(entry.path(), failure);
            }
        }
        std::vector<std::filesystem::path> made;
        for (const auto& entry : std::filesystem::directory_iterator(from, failure)) {
            if (std::regex_match(entry.path().filename().string(), mine)) {
                made.push_back(entry.path());
            }
        }
        std::ranges::sort(made);
        if (made.empty()) {
            fail("bake_urdf wrote no parts: " + tail(log, 300));
            return;
        }
        for (const std::filesystem::path& file : made) {
            const std::filesystem::path target = to / file.filename();
            std::filesystem::rename(file, target, failure);
            if (failure) {
                std::filesystem::copy_file(
                    file, target, std::filesystem::copy_options::overwrite_existing, failure);
            }
            if (failure) {
                fail("cannot write " + target.string() + ": " + failure.message());
                return;
            }
            parts.push_back(std::filesystem::absolute(target).lexically_normal().string());
        }
        std::filesystem::remove_all(work, failure);
        state = State::Done;
    }
};

CollisionSplit::CollisionSplit() : process_(std::make_unique<Process>()) {}

CollisionSplit::~CollisionSplit() {
    if (process_->state == State::Running && process_->pid > 0) {
        ::kill(process_->pid, SIGTERM);
        for (int i = 0; i < 20; ++i) {  // up to 1 s, then SIGKILL: shutdown only
            if (::waitpid(process_->pid, nullptr, WNOHANG) == process_->pid) {
                process_->pid = -1;
                break;
            }
            ::usleep(50'000);
        }
        if (process_->pid > 0) {
            ::kill(process_->pid, SIGKILL);
            ::waitpid(process_->pid, nullptr, 0);
        }
        std::error_code ignored;
        std::filesystem::remove_all(process_->work, ignored);
    }
}

std::string CollisionSplit::start(const SplitRequest& request) {
    Process& p = *process_;
    if (p.state == State::Running) {
        return "a split is already running (link " + p.request.link + ")";
    }
    if (request.robotFile.empty()) {
        return "save the robot first: the parts are written beside its file";
    }
    std::filesystem::path program = request.program;
    if (program.empty()) {
        const auto found = findBakeProgram();
        if (!found) {
            return "lagrange_collision_bake is not installed (bake_urdf not found)";
        }
        program = *found;
    }
    const auto mesh = meshes::resolveMeshUri(request.mesh, request.meshBase.string());
    std::error_code error;
    if (!mesh || !std::filesystem::is_regular_file(*mesh, error)) {
        return "the mesh " + request.mesh + " is not found";
    }
    *process_ = Process{};
    p.request = request;
    p.request.robotFile = std::filesystem::absolute(request.robotFile).lexically_normal();
    p.work = p.request.robotFile.parent_path() / ("." + request.robot + "_split_" + request.link);
    std::filesystem::remove_all(p.work, error);
    std::filesystem::create_directories(p.work, error);
    if (error) {
        return "cannot make " + p.work.string() + ": " + error.message();
    }
    const std::filesystem::path urdf = p.work / (request.robot + ".urdf");
    {
        std::ofstream out(urdf);
        out << "<robot name=\"" << xmlEscaped(request.robot) << "\">\n  <link name=\""
            << xmlEscaped(request.link) << "\">\n    <collision>\n      <geometry><mesh filename=\""
            << xmlEscaped(std::filesystem::absolute(*mesh).string()) << "\" scale=\""
            << number(request.scale[0]) << ' ' << number(request.scale[1]) << ' '
            << number(request.scale[2])
            << "\"/></geometry>\n    </collision>\n  </link>\n</robot>\n";
        if (!out) {
            return "cannot write " + urdf.string();
        }
    }
    p.log = p.work / "bake.log";
    const std::string programText = program.string();
    const std::string urdfText = urdf.string();
    const std::string threshold = number(request.threshold);
    const std::string seed = std::to_string(request.seed);
    std::vector<std::string> args{programText,   urdfText,  "-o",     urdfText,
                                  "--threshold", threshold, "--seed", seed};
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (std::string& arg : args) {
        argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    const std::string logText = p.log.string();
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, logText.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    pid_t pid = -1;
    std::vector<std::string> environment = childEnvironment();
    std::vector<char*> envp;
    envp.reserve(environment.size() + 1);
    for (std::string& entry : environment) {
        envp.push_back(entry.data());
    }
    envp.push_back(nullptr);
    const int failed =
        posix_spawn(&pid, programText.c_str(), &actions, nullptr, argv.data(), envp.data());
    posix_spawn_file_actions_destroy(&actions);
    if (failed != 0) {
        p.fail(std::string("cannot start bake_urdf: ") + std::strerror(failed));
        return p.error;
    }
    p.pid = pid;
    p.state = State::Running;
    p.started = Clock::now();
    return {};
}

CollisionSplit::State CollisionSplit::poll() {
    Process& p = *process_;
    if (p.state != State::Running) {
        return p.state;
    }
    int status = 0;
    const pid_t done = ::waitpid(p.pid, &status, WNOHANG);
    if (done == 0) {
        return p.state;
    }
    p.pid = -1;
    if (p.cancelled) {
        p.fail("cancelled");
    } else if (done < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        p.fail("bake_urdf failed: " + tail(p.log, 400));
    } else {
        p.finish();
    }
    return p.state;
}

void CollisionSplit::cancel() {
    if (process_->state == State::Running && process_->pid > 0) {
        process_->cancelled = true;
        ::kill(process_->pid, SIGTERM);
    }
}

CollisionSplit::State CollisionSplit::state() const {
    return process_->state;
}

const std::string& CollisionSplit::link() const {
    return process_->request.link;
}

const std::vector<std::string>& CollisionSplit::parts() const {
    return process_->parts;
}

const std::string& CollisionSplit::error() const {
    return process_->error;
}

double CollisionSplit::seconds() const {
    if (process_->state != State::Running) {
        return 0.0;
    }
    return std::chrono::duration<double>(Clock::now() - process_->started).count();
}

void CollisionSplit::reset() {
    if (process_->state != State::Running) {
        *process_ = Process{};
    }
}

}  // namespace lagrange::viewer::builder
