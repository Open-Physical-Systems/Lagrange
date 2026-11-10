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
#include "lagrange/electrical/Export.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/services/DeterminismCheck.hpp"
#include "lagrange/thermal/ThermalConduction.hpp"
#include "lagrange/world/PoseMath.hpp"

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
    std::fprintf(
        stderr,
        "usage: lagrange <command> [arguments]\n"
        "  run PROFILE [options]          run the simulator with the profile applied\n"
        "  validate PROFILE... [options]  check profiles against profiles/schema.json\n"
        "                                 and what this simulator can run\n"
        "  graph [PROFILE] [options]      print the wiring of the run as Graphviz DOT\n"
        "  determinism [PROFILE] [options] [--seconds N]\n"
        "                                 run twice (no ROS, CPU lidar) and compare the\n"
        "                                 state hashes every 100 ms; exit 1 if they\n"
        "                                 differ (default 10 s)\n"
        "  export spice|modelica [PROFILE] [options] [--out FILE]\n"
        "                                 the robot's power system as an ngspice\n"
        "                                 netlist or a Modelica model (to stdout)\n"
        "  heat [PROFILE] [options] [--surface-coefficient W_M2K]\n"
        "                                 the steady temperatures of the links with a\n"
        "                                 thermal material, at rest, in seconds (RFC 0012)\n"
        "  export thermal [PROFILE] [options] [--out FILE.json]\n"
        "                                 the links' thermal materials and heat\n"
        "                                 sources, for lagrange foam-cht (RFC 0012)\n"
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

/// The profile's plugin libraries (RFC 0008): a profile read at start loads them.
std::vector<std::string> pluginsOf(const std::string& yaml) {
    std::vector<std::string> libraries;
    try {
        const YAML::Node root = YAML::Load(yaml);
        if (root.IsMap() && root["plugins"] && root["plugins"].IsSequence()) {
            for (const YAML::Node& entry : root["plugins"]) {
                if (entry.IsMap() && entry["library"] && entry["library"].IsScalar()) {
                    libraries.push_back(entry["library"].as<std::string>());
                }
            }
        }
    } catch (const YAML::Exception&) {
        // checkProfile reports the parse error with its position.
    }
    return libraries;
}

/// Points the run at the profile's stage and loads its plugins. Returns the error, if any.
std::optional<std::string> useStage(CommandLine& line, const std::string& yaml,
                                    const fs::path& profile) {
    for (std::string& library : pluginsOf(yaml)) {
        if (std::ranges::find(line.options.plugins, library) == line.options.plugins.end()) {
            line.options.plugins.push_back(std::move(library));
        }
    }
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
    if (!line.overlayGiven && found->filename() != "amr_diffdrive.urdf") {
        line.options.overlayPath.clear();  // the default overlay fits the sample robot only
    }
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
    // A deterministic run: nothing from outside (no ROS), the CPU lidar, no camera rendering
    // (a GPU path, invariant 7).
    line.options.ros = false;
    line.options.lidar = lagrange::server::ServerOptions::Lidar::Cpu;
    line.options.cameras = false;
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

/// `lagrange export spice|modelica`: the robot's electrical network for offline study
/// (domains/electrical.md §7).
int exportNetwork(CommandLine& line, std::string_view format, const std::string& out) {
    if (line.positional.size() > 1) {
        return usage("export takes at most one profile");
    }
    if (!line.positional.empty()) {
        const auto yaml = readFile(line.positional[0]);
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
    if (const lagrange::Status status = server.load(); !status.ok()) {
        std::fprintf(stderr, "lagrange: %s\n", status.message.c_str());
        return 1;
    }
    lagrange::electrical::ExportOptions options;
    options.title = fs::path(line.options.robotPath).stem().string() + "_power";
    const lagrange::WorldView world(server.world());
    const auto text = format == "spice" ? lagrange::electrical::spiceNetlist(world, options)
                                        : lagrange::electrical::modelicaModel(world, options);
    if (!text) {
        std::fprintf(stderr, "lagrange: %s\n", text.error().c_str());
        return 1;
    }
    if (out.empty()) {
        std::fputs(text->c_str(), stdout);
        return 0;
    }
    std::ofstream file(out);
    file << *text;
    if (!file) {
        std::fprintf(stderr, "lagrange: cannot write %s\n", out.c_str());
        return 1;
    }
    std::printf("wrote %s\n", out.c_str());
    return 0;
}

/// `lagrange heat`: the conduction fidelity (RFC 0012) to its steady state at the rest pose,
/// without physics: backward Euler steps of growing length until no cell moves by 1 mK, then
/// each link's and each heat source's temperature. A source on a signal takes no power here.
int steadyHeat(CommandLine& line, std::optional<double> filmCoefficient) {
    if (line.positional.size() > 1) {
        return usage("heat takes at most one profile");
    }
    if (!line.positional.empty()) {
        const auto yaml = readFile(line.positional[0]);
        if (!yaml) {
            std::fprintf(stderr, "lagrange: cannot read %s\n", line.positional[0].c_str());
            return 1;
        }
        if (const auto error = useStage(line, *yaml, line.positional[0])) {
            std::fprintf(stderr, "lagrange: %s\n", error->c_str());
            return 1;
        }
    }
    if (!line.options.ambientTemperatureK) {
        return usage("heat needs the site's air temperature: --ambient-temperature K");
    }
    const double ambient = *line.options.ambientTemperatureK;
    lagrange::server::Server server(line.options);
    if (const lagrange::Status status = server.load(); !status.ok()) {
        std::fprintf(stderr, "lagrange: %s\n", status.message.c_str());
        return 1;
    }
    constexpr double kCell = 0.002;  // m, as the conduction fidelity's default
    auto setup =
        lagrange::thermal::thermalSetupOf(lagrange::WorldView(server.world()), ambient, kCell);
    if (!setup) {
        std::fprintf(stderr, "lagrange: %s\n", setup.error().c_str());
        return 1;
    }
    if (setup->parts.empty()) {
        std::fprintf(stderr, "lagrange: no link has a thermal material (ThermalMaterialAPI_1)\n");
        return 1;
    }
    std::vector<std::string> notes;
    std::string error;
    auto model = lagrange::thermal::ConductionModel::build(setup->parts, 200000, notes, error);
    if (!model) {
        std::fprintf(stderr, "lagrange: %s\n", error.c_str());
        return 1;
    }
    for (const std::string& note : notes) {
        std::fprintf(stderr, "note: %s\n", note.c_str());
    }
    std::vector<double> heats;
    double total = 0.0;
    for (const auto& s : setup->sources) {
        const double q = s.powerSignal.empty() ? (1.0 - s.efficiency) * s.power : 0.0;
        heats.push_back(q);
        total += q;
    }
    std::vector<lagrange::thermal::PartInput> inputs(setup->parts.size());
    for (std::size_t p = 0; p < inputs.size(); ++p) {
        inputs[p].pose = setup->parts[p].rest;
    }
    std::vector<double> t = model->initialTemperatures();
    const lagrange::thermal::Surroundings around{ambient, ambient, 0.0};
    constexpr double kSettled = 1e-3;  // K: no cell moves by more in a step
    double dt = 1.0;
    int steps = 0;
    for (; steps < 400; ++steps) {
        const std::vector<double> before = t;
        model->step(t, inputs, heats, around, dt, filmCoefficient);
        double moved = 0.0;
        for (std::size_t i = 0; i < t.size(); ++i) {
            moved = std::max(moved, std::abs(t[i] - before[i]));
        }
        if (moved < kSettled && dt >= 100.0) {
            break;
        }
        dt = std::min(dt * 1.5, 1e4);
    }
    double out = 0.0;
    std::printf("%-24s %10s %10s %10s\n", "link", "mean K", "max K", "min K");
    for (std::size_t p = 0; p < setup->parts.size(); ++p) {
        out += model->heatOut(p);
        std::printf("%-24s %10.2f %10.2f %10.2f\n", setup->parts[p].name.c_str(), model->mean(t, p),
                    model->maximum(t, p), model->minimum(t, p));
    }
    std::vector<std::size_t> nth(setup->parts.size(), 0);
    for (std::size_t i = 0; i < setup->sources.size(); ++i) {
        const auto& s = setup->sources[i];
        std::printf("%-24s %10.2f   (%.3g W of heat%s)\n", s.name.c_str(),
                    model->sourceMean(t, s.part, nth[s.part]++), heats[i],
                    s.powerSignal.empty() ? "" : ", on a signal: none here");
    }
    std::printf("%.3f W in, %.3f W out to the air and the sky at %.2f K, after %d steps%s\n", total,
                out, ambient, steps,
                filmCoefficient ? " (one film coefficient)" : " (the correlations)");
    return 0;
}

/// `lagrange export thermal`: every link with a thermal material, as the conduction fidelity
/// reads it (RFC 0012), with a point inside each and its heat sources, at the rest pose in the
/// world frame, as JSON for lagrange foam-cht.
int exportThermal(CommandLine& line, const std::string& out) {
    if (line.positional.size() > 1) {
        return usage("export takes at most one profile");
    }
    if (!line.positional.empty()) {
        const auto yaml = readFile(line.positional[0]);
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
    if (const lagrange::Status status = server.load(); !status.ok()) {
        std::fprintf(stderr, "lagrange: %s\n", status.message.c_str());
        return 1;
    }
    constexpr double kCell = 0.002;  // m, as the conduction fidelity's default
    const double ambient = line.options.ambientTemperatureK.value_or(0.0);
    auto setup = lagrange::thermal::thermalSetupOf(lagrange::WorldView(server.world()),
                                                   ambient > 0.0 ? ambient : 293.15, kCell);
    if (!setup) {
        std::fprintf(stderr, "lagrange: %s\n", setup.error().c_str());
        return 1;
    }
    if (setup->parts.empty()) {
        std::fprintf(stderr, "lagrange: no link has a thermal material (ThermalMaterialAPI_1)\n");
        return 1;
    }
    // A point inside each part: the centre of one of its cells.
    std::vector<std::string> notes;
    std::string error;
    const auto model =
        lagrange::thermal::ConductionModel::build(setup->parts, 200000, notes, error);
    if (!model) {
        std::fprintf(stderr, "lagrange: %s\n", error.c_str());
        return 1;
    }
    std::ostringstream json;
    json.precision(10);
    const auto vec = [&](const std::array<double, 3>& v) {
        json << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    };
    const auto toWorld = [](const lagrange::Pose& pose, const std::array<double, 3>& local) {
        return lagrange::compose(pose, lagrange::Pose{local, {1.0, 0.0, 0.0, 0.0}}).position;
    };
    json << "{\n  \"ambient_temperature\": " << (ambient > 0.0 ? ambient : 0.0)
         << ",\n  \"parts\": [\n";
    for (std::size_t p = 0; p < setup->parts.size(); ++p) {
        const lagrange::thermal::SolidPart& part = setup->parts[p];
        const auto& m = part.material;
        const auto& grid = model->grid(p);
        json << "    {\"name\": \"" << part.name << "\", \"conductivity\": " << m.conductivity;
        if (m.conductivityAlong) {
            json << ", \"conductivity_along\": " << *m.conductivityAlong << ", \"axis\": ";
            vec(lagrange::rotate(part.rest.orientation, part.axis));
        }
        json << ", \"density\": " << m.density << ", \"specific_heat\": " << m.specificHeat
             << ", \"emissivity\": " << m.emissivity;
        if (part.contactConductance && part.parent) {
            json << ", \"contact_conductance\": " << *part.contactConductance << ", \"parent\": \""
                 << setup->parts[*part.parent].name << "\"";
        }
        // The cell nearest the middle of its cells: well inside, for snappyHexMesh.
        std::array<double, 3> middle{0.0, 0.0, 0.0};
        for (std::size_t c = grid.first; c < grid.first + grid.count; ++c) {
            for (std::size_t k = 0; k < 3; ++k) {
                middle[k] += model->centre(c)[k] / static_cast<double>(grid.count);
            }
        }
        std::size_t inner = grid.first;
        double best = 1e300;
        for (std::size_t c = grid.first; c < grid.first + grid.count; ++c) {
            const auto x = model->centre(c);
            const double d = (x[0] - middle[0]) * (x[0] - middle[0]) +
                             (x[1] - middle[1]) * (x[1] - middle[1]) +
                             (x[2] - middle[2]) * (x[2] - middle[2]);
            if (d < best) {
                best = d;
                inner = c;
            }
        }
        json << ", \"inside\": ";
        vec(toWorld(part.rest, model->centre(inner)));
        json << ", \"cell\": " << grid.cell << ", \"sources\": [";
        bool first = true;
        for (const auto& source : setup->sources) {
            if (source.part != p) {
                continue;
            }
            json << (first ? "" : ", ") << "{\"name\": \"" << source.name
                 << "\", \"heat\": " << (1.0 - source.efficiency) * source.power
                 << ", \"power_signal\": \"" << source.powerSignal << "\", \"centre\": ";
            vec(toWorld(part.rest, source.box.centre));
            json << ", \"size\": ";
            vec(source.box.size);
            json << ", \"axes\": [";
            for (int a = 0; a < 3; ++a) {
                std::array<double, 3> e{0.0, 0.0, 0.0};
                e[static_cast<std::size_t>(a)] = 1.0;
                if (a > 0) {
                    json << ", ";
                }
                vec(lagrange::rotate(part.rest.orientation, e));
            }
            json << "]}";
            first = false;
        }
        json << "]}" << (p + 1 < setup->parts.size() ? "," : "") << "\n";
    }
    json << "  ]\n}\n";
    if (out.empty()) {
        std::fputs(json.str().c_str(), stdout);
        return 0;
    }
    std::ofstream file(out);
    file << json.str();
    if (!file) {
        std::fprintf(stderr, "lagrange: cannot write %s\n", out.c_str());
        return 1;
    }
    std::printf("wrote %s\n", out.c_str());
    return 0;
}

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
        command != "determinism" && command != "export" && command != "heat") {
        return usage(("unknown command " + std::string(command)).c_str());
    }
    // `export` takes its format first.
    std::string_view format;
    std::vector<char*> exportArgs;
    if (command == "export") {
        if (argc < 3 ||
            (std::string_view(argv[2]) != "spice" && std::string_view(argv[2]) != "modelica" &&
             std::string_view(argv[2]) != "thermal")) {
            return usage("export needs spice, modelica or thermal");
        }
        format = argv[2];
        exportArgs.assign(argv, argv + argc);
        exportArgs.erase(exportArgs.begin() + 2);
        argc = static_cast<int>(exportArgs.size());
        argv = exportArgs.data();
    }
    // --seconds belongs to `determinism` and --out to `export`; take them out before the
    // server's options.
    double seconds = 10.0;
    std::string out;
    std::vector<char*> args(argv, argv + argc);
    for (std::size_t i = 2; i + 1 < args.size(); ++i) {
        if (std::string_view(args[i]) == "--out") {
            out = args[i + 1];
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i),
                       args.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            break;
        }
    }
    std::optional<double> filmCoefficient;
    for (std::size_t i = 2; i + 1 < args.size(); ++i) {
        if (std::string_view(args[i]) == "--surface-coefficient") {
            char* end = nullptr;
            const double h = std::strtod(args[i + 1], &end);
            if (end == args[i + 1] || *end != '\0' || !(h >= 0.0)) {
                return usage("--surface-coefficient needs W/(m²·K), 0 or more");
            }
            filmCoefficient = h;
            args.erase(args.begin() + static_cast<std::ptrdiff_t>(i),
                       args.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            break;
        }
    }
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
    if (command == "heat") {
        return steadyHeat(line, filmCoefficient);
    }
    if (command == "export" && format == "thermal") {
        return exportThermal(line, out);
    }
    if (command == "export") {
        return exportNetwork(line, format, out);
    }
    return command == "validate" ? validate(line) : graph(line);
}
