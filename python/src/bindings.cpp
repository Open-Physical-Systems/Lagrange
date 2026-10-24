// Python bindings (roadmap 0.2-H5): a simulation run headless from Python.
//
//   import lagrange
//   sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
//   sim.step_until(2.0)
//   sim.read("body/base_link/pose")
#include "lagrange/server/Server.hpp"

#include <cstdint>
#include <fstream>
#include <memory>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace {

/// One run, without ROS and without viewers, stepped only when Python asks.
class Simulation {
public:
    Simulation(const std::string& robot, const std::optional<std::string>& world,
               const std::optional<std::string>& profile, const std::string& lidar) {
        lagrange::server::ServerOptions options;
        options.robotPath = robot;
        options.worldPath = world.value_or("");
        options.ros = false;
        options.demoDrive = true;
        if (lidar == "cpu" || lidar == "auto") {
            options.lidar = lagrange::server::ServerOptions::Lidar::Cpu;
        } else if (lidar == "gpu") {
            options.lidar = lagrange::server::ServerOptions::Lidar::Gpu;
        } else {
            throw std::invalid_argument("lidar must be 'cpu' or 'gpu', not '" + lidar + "'");
        }
        std::string yaml;
        if (profile) {
            std::ifstream in(*profile);
            if (!in) {
                throw std::runtime_error("cannot read profile " + *profile);
            }
            std::ostringstream text;
            text << in.rdbuf();
            yaml = text.str();
            options.stageName = stageOf(yaml);
            options.profileSchemaPath = schemaPath();
        }
        server_ = std::make_unique<lagrange::server::Server>(options);
        if (const lagrange::Status status = server_->load(); !status.ok()) {
            throw std::runtime_error(status.message);
        }
        if (profile) {
            if (const lagrange::Status status = server_->applyProfile(yaml); !status.ok()) {
                throw std::runtime_error(status.message);
            }
        }
    }

    void stepUntil(double seconds) {
        const lagrange::Status status = [&] {
            const nb::gil_scoped_release release;  // pure C++: let other Python threads run
            return server_->stepUntil(
                lagrange::SimTime{static_cast<std::int64_t>(seconds * 1e9 + 0.5)});
        }();
        if (!status.ok()) {
            throw std::runtime_error(status.message);
        }
    }

    [[nodiscard]] double time() const { return server_->now().seconds(); }

    [[nodiscard]] std::vector<double> read(const std::string& name) const {
        std::vector<double> values = server_->read(name);
        if (values.empty()) {
            throw nb::key_error(("no signal " + name).c_str());
        }
        return values;
    }

    [[nodiscard]] nb::list signals() const {
        nb::list out;
        for (const auto& signal : server_->signals()) {
            nb::dict entry;
            entry["name"] = signal.name;
            entry["unit"] = signal.unit;
            entry["components"] = signal.components;
            out.append(entry);
        }
        return out;
    }

    [[nodiscard]] std::vector<std::string> bodies() const { return server_->world().bodies.names; }
    [[nodiscard]] std::vector<std::string> joints() const { return server_->world().joints.names; }

    [[nodiscard]] std::uint64_t stateHash() const {
        const auto hash = server_->stateHash();
        if (!hash) {
            throw std::runtime_error(hash.error().message);
        }
        return *hash;
    }

private:
    /// The profile's `stage:` line, so that the profile applies to the robot given here.
    static std::string stageOf(const std::string& yaml) {
        std::istringstream lines(yaml);
        for (std::string line; std::getline(lines, line);) {
            if (line.rfind("stage:", 0) == 0) {
                std::string value = line.substr(6);
                const std::size_t begin = value.find_first_not_of(" \t\"'");
                const std::size_t end = value.find_last_not_of(" \t\"'\r");
                return begin == std::string::npos ? "" : value.substr(begin, end - begin + 1);
            }
        }
        return {};
    }

    static std::string schemaPath() {
        nb::object share = nb::module_::import_("ament_index_python.packages")
                               .attr("get_package_share_directory")("lagrange_profiles");
        return nb::cast<std::string>(share) + "/schema.json";
    }

    std::unique_ptr<lagrange::server::Server> server_;
};

}  // namespace

NB_MODULE(_lagrange, m) {
    m.doc() = "Lagrange simulator, headless (roadmap 0.2-H5)";
    nb::class_<Simulation>(m, "Simulation",
                           "A run without ROS or viewers, stepped from Python. The robot and "
                           "world are URDF, SDFormat, MJCF or USD files.")
        .def(nb::init<const std::string&, const std::optional<std::string>&,
                      const std::optional<std::string>&, const std::string&>(),
             "robot"_a, "world"_a = nb::none(), "profile"_a = nb::none(), "lidar"_a = "cpu")
        .def("step_until", &Simulation::stepUntil, "seconds"_a,
             "Advance to this simulated time (s), as fast as possible.")
        .def_prop_ro("time", &Simulation::time, "Simulated time (s).")
        .def("read", &Simulation::read, "name"_a,
             "Latest value of a signal (all its components). KeyError for an unknown name.")
        .def("signals", &Simulation::signals,
             "Every signal on the bus: dicts with name, unit and components.")
        .def("bodies", &Simulation::bodies)
        .def("joints", &Simulation::joints)
        .def("state_hash", &Simulation::stateHash,
             "Hash of the whole state: equal hashes mean equal runs.");
}
