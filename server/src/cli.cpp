// lagrange: the command-line tool (roadmap 0.2-E7).
//
//   lagrange run PROFILE [options]          the simulator with a profile applied
//   lagrange validate PROFILE... [options]  check profiles; exit 1 if any has an error
//   lagrange graph [PROFILE] [options]      the wiring as Graphviz DOT
//   lagrange determinism [PROFILE] [options] [--seconds N]
//                                           run twice, compare the state hashes
//   lagrange info [--version]               version, paths and GPUs of this installation
//
// Options are lagrange_server's. A profile's `stage` names the robot to load (URDF, SDFormat,
// MJCF or USD, found next to the profile or in lagrange_assets) unless --robot is given.
#include "CommandLine.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/services/DeterminismCheck.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <yaml-cpp/yaml.h>

#ifndef LAGRANGE_VERSION
#define LAGRANGE_VERSION "unknown"
#endif
#ifndef LAGRANGE_BUILD_TYPE
#define LAGRANGE_BUILD_TYPE "unknown"
#endif

namespace {

namespace fs = std::filesystem;
using lagrange::server::CommandLine;

int usage(const char* error) {
    if (error != nullptr) {
        std::fprintf(stderr, "lagrange: %s\n", error);
    }
    std::fprintf(stderr,
                 "usage: lagrange <command> [arguments]\n"
                 "  run PROFILE [options]          run the simulator with the profile applied\n"
                 "  validate PROFILE... [options]  check profiles against profiles/schema.json\n"
                 "                                 and what this simulator can run\n"
                 "  graph [PROFILE] [options]      print the wiring of the run as Graphviz DOT\n"
                 "  determinism [PROFILE] [options] [--seconds N]\n"
                 "                                 run twice (no ROS, CPU lidar) and compare the\n"
                 "                                 state hashes every 100 ms; exit 1 if they\n"
                 "                                 differ (default 10 s)\n"
                 "  info [--version]               version, paths and GPUs\n"
                 "A profile's stage names the robot (URDF) unless --robot is given.\n"
                 "Options (as lagrange_server):\n");
    lagrange::server::printServerOptions(stderr);
    return error != nullptr ? 2 : 0;
}

std::optional<std::string> readFile(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

/// The profile's `stage`, or "" when it has none (checkProfile reports that).
std::string stageOf(const std::string& yaml) {
    try {
        const YAML::Node root = YAML::Load(yaml);
        if (root.IsMap() && root["stage"] && root["stage"].IsScalar()) {
            return root["stage"].as<std::string>();
        }
    } catch (const YAML::Exception&) {
        // checkProfile reports the parse error with its position.
    }
    return {};
}

/// The stage as a file: as written, next to the profile, or in lagrange_assets.
std::optional<fs::path> findStage(const std::string& stage, const fs::path& profile) {
    std::error_code ec;
    for (const fs::path& candidate :
         {fs::path(stage), profile.parent_path() / stage,
          fs::path(lagrange::server::shareFile("lagrange_assets", "")) / stage}) {
        if (!candidate.empty() && fs::is_regular_file(candidate, ec)) {
            return candidate;
        }
    }
    return std::nullopt;
}

/// Points the run at the profile's stage. Returns the error, if any.
std::optional<std::string> useStage(CommandLine& line, const std::string& yaml,
                                    const fs::path& profile) {
    const std::string stage = stageOf(yaml);
    if (stage.empty()) {
        return std::nullopt;
    }
    line.options.stageName = stage;
    if (line.robotGiven) {
        return std::nullopt;
    }
    const auto found = findStage(stage, profile);
    if (!found) {
        return "stage " + stage + " not found next to " + profile.string() +
               " or in lagrange_assets; pass --robot FILE to check the profile against a robot";
    }
    line.options.robotPath = found->string();
    return std::nullopt;
}

int validate(const CommandLine& base) {
    if (base.positional.empty()) {
        return usage("validate needs a profile");
    }
    int code = 0;
    for (const std::string& path : base.positional) {
        const auto yaml = readFile(path);
        if (!yaml) {
            std::printf("%s: cannot read\n", path.c_str());
            code = 1;
            continue;
        }
        CommandLine line = base;
        if (const auto error = useStage(line, *yaml, path)) {
            std::printf("%s: %s\n", path.c_str(), error->c_str());
            code = 1;
            continue;
        }
        lagrange::server::Server server(line.options);
        if (const lagrange::Status status = server.load(); !status.ok()) {
            std::printf("%s: %s\n", path.c_str(), status.message.c_str());
            code = 1;
            continue;
        }
        const lagrange::server::ProfileCheck check = server.validateProfile(*yaml);
        std::size_t errors = 0;
        for (const auto& issue : check.issues) {
            errors += issue.error ? 1 : 0;
        }
        const std::size_t warnings = check.issues.size() - errors;
        std::printf("%s: %s (%zu error%s, %zu warning%s)\n", path.c_str(),
                    check.valid ? "valid" : "invalid", errors, errors == 1 ? "" : "s", warnings,
                    warnings == 1 ? "" : "s");
        for (const auto& issue : check.issues) {
            std::printf("  %-7s %s%s%s\n", issue.error ? "error" : "warning", issue.pointer.c_str(),
                        issue.pointer.empty() ? "" : ": ", issue.message.c_str());
        }
        code = check.valid ? code : 1;
    }
    return code;
}

int graph(CommandLine& line) {
    if (line.positional.size() > 1) {
        return usage("graph takes at most one profile");
    }
    std::optional<std::string> yaml;
    if (!line.positional.empty()) {
        yaml = readFile(line.positional[0]);
        if (!yaml) {
            std::fprintf(stderr, "lagrange: cannot read %s\n", line.positional[0].c_str());
            return 1;
        }
        if (const auto error = useStage(line, *yaml, line.positional[0])) {
            std::fprintf(stderr, "lagrange: %s\n", error->c_str());
            return 1;
        }
    }
    lagrange::server::Server server(line.options);
    lagrange::Status status = server.load();
    if (status.ok() && yaml) {
        status = server.applyProfile(*yaml);
    }
    if (!status.ok()) {
        std::fprintf(stderr, "lagrange: %s\n", status.message.c_str());
        return 1;
    }
    std::fputs(server.wiringDot().c_str(), stdout);
    return 0;
}

/// One headless run of the server for the determinism check.
class ServerRun final : public lagrange::services::CheckedRun {
public:
    explicit ServerRun(const lagrange::server::ServerOptions& options) : server_(options) {}
    lagrange::Status load() { return server_.load(); }
    lagrange::Status stepUntil(lagrange::SimTime t) override { return server_.stepUntil(t); }
    [[nodiscard]] lagrange::Expected<std::uint64_t, lagrange::Status> stateHash() const override {
        return server_.stateHash();
    }

private:
    lagrange::server::Server server_;
};

int determinism(CommandLine& line, double seconds) {
    if (line.positional.size() > 1) {
        return usage("determinism takes at most one profile");
    }
    std::optional<std::string> yaml;
    if (!line.positional.empty()) {
        yaml = readFile(line.positional[0]);
        if (!yaml) {
            std::fprintf(stderr, "lagrange: cannot read %s\n", line.positional[0].c_str());
            return 1;
        }
        if (const auto error = useStage(line, *yaml, line.positional[0])) {
            std::fprintf(stderr, "lagrange: %s\n", error->c_str());
            return 1;
        }
    }
    // A deterministic run: nothing from outside (no ROS) and the CPU lidar.
    line.options.ros = false;
    line.options.lidar = lagrange::server::ServerOptions::Lidar::Cpu;
    const auto make =
        [&]() -> lagrange::Expected<std::unique_ptr<lagrange::services::CheckedRun>, std::string> {
        auto run = std::make_unique<ServerRun>(line.options);
        if (lagrange::Status status = run->load(); !status.ok()) {
            return lagrange::Unexpected{status.message};
        }
        return std::unique_ptr<lagrange::services::CheckedRun>(std::move(run));
    };
    const auto end = lagrange::SimTime{static_cast<std::int64_t>(seconds * 1e9)};
    const lagrange::services::DeterminismReport report =
        lagrange::services::checkDeterminism(make, end, lagrange::Duration{100'000'000});
    if (!report.error.empty()) {
        std::fprintf(stderr, "lagrange: %s\n", report.error.c_str());
        return 1;
    }
    if (!report.identical) {
        std::printf(
            "NOT deterministic: the two runs differ at t = %.3f s (after %u identical "
            "comparisons)\n",
            report.firstDivergence->seconds(), report.comparisons - 1);
        return 1;
    }
    std::printf("deterministic: %u state hashes over %.1f s identical in both runs\n",
                report.comparisons, seconds);
    return 0;
}

int run(CommandLine& line, int argc, char** argv) {
    if (line.positional.size() != 1) {
        return usage("run needs one profile");
    }
    const auto yaml = readFile(line.positional[0]);
    if (!yaml) {
        std::fprintf(stderr, "lagrange: cannot read %s\n", line.positional[0].c_str());
        return 1;
    }
    if (const auto error = useStage(line, *yaml, line.positional[0])) {
        std::fprintf(stderr, "lagrange: %s\n", error->c_str());
        return 1;
    }
    return lagrange::server::serve(line, argc, argv, &*yaml, "lagrange");
}

std::string orNone(const std::string& path) {
    return path.empty() ? "not installed" : path;
}

int info(int argc, char** argv) {
    if (argc > 2 && std::string_view(argv[2]) == "--version") {
        std::printf("lagrange %s\n", LAGRANGE_VERSION);
        return 0;
    }
    if (argc > 2) {
        return usage("info takes only --version");
    }
    const CommandLine defaults = lagrange::server::defaultCommandLine();
    const char* distro = std::getenv("ROS_DISTRO");
    std::printf("lagrange %s (%s)\n", LAGRANGE_VERSION, LAGRANGE_BUILD_TYPE);
    std::printf("ROS 2          %s\n", distro != nullptr ? distro : "not sourced");
    std::printf("sample robot   %s\n", orNone(defaults.options.robotPath).c_str());
    std::printf("sample world   %s\n", orNone(defaults.options.worldPath).c_str());
    std::printf("profile schema %s\n", orNone(defaults.options.profileSchemaPath).c_str());
    std::printf("viewer socket  %s\n",
                lagrange::proto::toString(defaults.options.endpoint).c_str());

    std::string error;
    const std::unique_ptr<lagrange::gpu::Device> device =
        lagrange::gpu::Device::create(lagrange::gpu::DeviceDesc{}, error);
    if (!device) {
        std::printf("GPU            none usable (%s); capability %s\n", error.c_str(),
                    lagrange::gpu::toString(lagrange::gpu::CapabilityTier::Tier0));
        return 0;
    }
    std::size_t index = 0;
    for (const lagrange::gpu::DeviceInfo& gpu : device->allDevices()) {
        std::printf("GPU [%zu]        %s, %s%s%s\n", index++, gpu.name.c_str(),
                    lagrange::gpu::toString(gpu.tier), gpu.usable ? "" : " (not usable)",
                    gpu.name == device->info().name ? " (selected)" : "");
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return usage("no command");
    }
    const std::string_view command = argv[1];
    if (command == "--help" || command == "-h" || command == "help") {
        return usage(nullptr);
    }
    if (command == "info") {
        return info(argc, argv);
    }
    if (command != "run" && command != "validate" && command != "graph" &&
        command != "determinism") {
        return usage(("unknown command " + std::string(command)).c_str());
    }
    // --seconds belongs to `determinism` only; take it out before the server's options.
    double seconds = 10.0;
    std::vector<char*> args(argv, argv + argc);
    for (std::size_t i = 2; i + 1 < args.size(); ++i) {
        if (std::string_view(args[i]) == "--seconds") {
            char* end = nullptr;
            seconds = std::strtod(args[i + 1], &end);
            if (end == args[i + 1] || *end != '\0' || !(seconds > 0.0)) {
                return usage("--seconds needs a positive number");
            }
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i),
                       args.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            break;
        }
    }
    argc = static_cast<int>(args.size());
    argv = args.data();
    CommandLine line = lagrange::server::defaultCommandLine();
    if (const auto error = lagrange::server::parseCommandLine(argc, argv, 2, true, line)) {
        return usage(error->c_str());
    }
    if (line.help) {
        return usage(nullptr);
    }
    if (command == "run") {
        return run(line, argc, argv);
    }
    if (command == "determinism") {
        return determinism(line, seconds);
    }
    // Checking and printing need no ROS node.
    line.options.ros = false;
    return command == "validate" ? validate(line) : graph(line);
}
