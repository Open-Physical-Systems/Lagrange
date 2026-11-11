// Python bindings (roadmap 0.2-H5): a simulation run headless from Python.
//
//   import lagrange
//   sim = lagrange.Simulation(lagrange.sample_robot(), world=lagrange.sample_world())
//   sim.step_until(2.0)
//   sim.read("body/base_link/pose")
#include "lagrange/server/Server.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
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
    Simulation(const std::optional<std::string>& robot, const std::optional<std::string>& world,
               const std::optional<std::string>& profile, const std::string& lidar,
               const std::optional<std::vector<double>>& spawn,
               const std::optional<std::array<int, 3>>& traffic, bool jointControl,
               const std::vector<std::string>& sensorFaults) {
        lagrange::server::ServerOptions options;
        options.jointControl = jointControl;
        options.sensorFaults = sensorFaults;     // checked by load(), as --sensor-fault
        options.robotPath = robot.value_or("");  // None: an empty run to spawn robots into
        if (spawn) {
            options.spawn = poseOf(*spawn, "spawn");
        }
        options.worldPath = world.value_or("");
        options.ros = false;
        if (traffic) {  // vehicles, pedestrians, seed
            options.trafficVehicles = (*traffic)[0];
            options.trafficPedestrians = (*traffic)[1];
            options.trafficSeed = static_cast<std::uint64_t>((*traffic)[2]);
        }
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
            options.profileFollows = true;
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
        // A time that does not fit SimTime's nanoseconds (or NaN) would convert to nonsense and
        // silently step nothing.
        constexpr double kLatestSeconds = 9.2e9;  // below INT64_MAX ns
        if (!(seconds <= kLatestSeconds)) {
            throw std::invalid_argument("step_until needs a finite time in s, at most 9.2e9");
        }
        const lagrange::Status status = [&] {
            const nb::gil_scoped_release release;  // pure C++: let other Python threads run
            return server_->stepUntil(
                lagrange::SimTime{static_cast<std::int64_t>(seconds * 1e9 + 0.5)});
        }();
        if (!status.ok()) {
            throw std::runtime_error(status.message);
        }
    }

    std::string spawnRobot(const std::string& path, const std::string& name,
                           const std::vector<double>& at) {
        auto spawned = server_->spawnRobot(path, name, poseOf(at, "at"));
        if (!spawned) {
            throw std::runtime_error(spawned.error().message);
        }
        return *spawned;
    }

    void removeRobot(const std::string& name) {
        if (const lagrange::Status status = server_->removeRobot(name); !status.ok()) {
            throw std::runtime_error(status.message);
        }
    }

    [[nodiscard]] double time() const { return server_->now().seconds(); }

    [[nodiscard]] std::vector<double> read(const std::string& name) const {
        std::vector<double> values = server_->read(name);
        if (values.empty()) {
            if (components(name)) {
                throw nb::type_error(
                    (name + " has no numbers (an image: read it with image())").c_str());
            }
            throw nb::key_error(("no signal " + name).c_str());
        }
        return values;
    }

    /// The signal's components, or nullopt for an unknown name (0 for an image).
    [[nodiscard]] std::optional<std::uint32_t> components(const std::string& name) const {
        for (const auto& signal : server_->signals()) {
            if (signal.name == name) {
                return signal.components;
            }
        }
        return std::nullopt;
    }

    /// The latest frame of an image signal as a numpy array: rgba8 (height, width, 4) uint8,
    /// rgb8 (h, w, 3) uint8, r8 (h, w) uint8, r16u (h, w) uint16, r32f (h, w) float32 (a depth
    /// in m, a temperature in K, ...), r32u (h, w) uint32. None before the first frame.
    [[nodiscard]] nb::object image(const std::string& name) const {
        const std::shared_ptr<const lagrange::Buffer> buffer = server_->readImage(name);
        if (!buffer) {
            // Why not (looked up only now: signals() lists every signal).
            const std::optional<std::uint32_t> count = components(name);
            if (!count) {
                throw nb::key_error(("no signal " + name).c_str());
            }
            if (*count != 0) {
                throw nb::type_error((name + " is not an image (read it with read())").c_str());
            }
            return nb::none();
        }
        struct Layout {
            const char* encoding;
            const char* dtype;
            std::size_t channels, bytesPerPixel;
        };
        static constexpr std::array<Layout, 6> kLayouts{{{"rgba8", "uint8", 4, 4},
                                                         {"rgb8", "uint8", 3, 3},
                                                         {"r8", "uint8", 1, 1},
                                                         {"r16u", "<u2", 1, 2},
                                                         {"r32f", "<f4", 1, 4},
                                                         {"r32u", "<u4", 1, 4}}};
        for (const Layout& layout : kLayouts) {
            if (buffer->encoding != layout.encoding) {
                continue;
            }
            const std::size_t h = buffer->height;
            const std::size_t w = buffer->width;
            if (buffer->bytes.size() != h * w * layout.bytesPerPixel) {
                throw std::runtime_error(name + ": the frame's size does not match its encoding");
            }
            const nb::module_ np = nb::module_::import_("numpy");
            const nb::bytes raw(reinterpret_cast<const char*>(buffer->bytes.data()),
                                buffer->bytes.size());
            const nb::object shape =
                layout.channels == 1 ? nb::make_tuple(h, w) : nb::make_tuple(h, w, layout.channels);
            return np.attr("frombuffer")(raw, np.attr("dtype")(layout.dtype))
                .attr("reshape")(shape)
                .attr("copy")();
        }
        throw std::runtime_error(name + ": encoding " + buffer->encoding +
                                 " has no numpy layout here");
    }

    bool commandJoint(const std::string& joint, const std::string& mode, double value) {
        return server_->commandJoint(joint, mode, value);
    }

    [[nodiscard]] nb::bytes saveState() const {
        auto state = server_->saveState();
        if (!state) {
            throw std::runtime_error(state.error().message);
        }
        return nb::bytes(reinterpret_cast<const char*>(state->data()), state->size());
    }

    void restoreState(const nb::bytes& state) {
        const std::span<const std::byte> bytes(reinterpret_cast<const std::byte*>(state.c_str()),
                                               state.size());
        if (const lagrange::Status status = server_->restoreState(bytes); !status.ok()) {
            throw std::runtime_error(status.message);
        }
    }

    void setSite(const std::string& name, double value) {
        if (const lagrange::Status status = server_->setSite(name, value); !status.ok()) {
            throw std::runtime_error(status.message);
        }
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
    [[nodiscard]] std::vector<double> regionOrigin() const {
        const auto& r = server_->world().region;
        return {r.worldEnu[0], r.worldEnu[1], r.worldEnu[2]};
    }
    [[nodiscard]] std::size_t colliders() const { return server_->world().colliders.body.size(); }
    [[nodiscard]] std::vector<std::string> joints() const { return server_->world().joints.names; }

    [[nodiscard]] std::uint64_t stateHash() const {
        const auto hash = server_->stateHash();
        if (!hash) {
            throw std::runtime_error(hash.error().message);
        }
        return *hash;
    }

private:
    /// x, y[, z[, yaw]] in world ENU, as --spawn.
    static lagrange::Pose poseOf(const std::vector<double>& xyzYaw, const char* what) {
        if (xyzYaw.size() < 2 || xyzYaw.size() > 4) {
            throw std::invalid_argument(std::string(what) + " is (x, y[, z[, yaw]])");
        }
        for (const double v : xyzYaw) {
            if (!std::isfinite(v)) {
                throw std::invalid_argument(std::string(what) + " must be finite numbers");
            }
        }
        lagrange::Pose pose;
        pose.position = {xyzYaw[0], xyzYaw[1], xyzYaw.size() > 2 ? xyzYaw[2] : 0.0};
        const double yaw = xyzYaw.size() > 3 ? xyzYaw[3] : 0.0;
        pose.orientation = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
        return pose;
    }

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
        .def(nb::init<const std::optional<std::string>&, const std::optional<std::string>&,
                      const std::optional<std::string>&, const std::string&,
                      const std::optional<std::vector<double>>&,
                      const std::optional<std::array<int, 3>>&, bool,
                      const std::vector<std::string>&>(),
             "robot"_a, "world"_a = nb::none(), "profile"_a = nb::none(), "lidar"_a = "cpu",
             "spawn"_a = nb::none(), "traffic"_a = nb::none(), "joint_control"_a = false,
             "sensor_faults"_a = std::vector<std::string>{})
        .def("spawn", &Simulation::spawnRobot, "path"_a, "name"_a = "",
             "at"_a = std::vector<double>{0.0, 0.0},
             "Add a robot to the run at (x, y[, z[, yaw]]); returns its entity name. In a run "
             "without a robot (robot=None) it is the robot; later ones are passive. Time and "
             "poses go on; velocities start from rest.")
        .def("delete", &Simulation::removeRobot, "name"_a, "Take a robot out of the run.")
        .def("step_until", &Simulation::stepUntil, "seconds"_a,
             "Advance to this simulated time (s), as fast as possible.")
        .def(
            "command_joint",
            [](Simulation& sim, const std::string& joint, const std::string& mode, double value) {
                return sim.commandJoint(joint, mode, value);
            },
            "joint"_a, "mode"_a, "value"_a,
            "With joint_control=True: hold a joint at this position, velocity or effort. False "
            "if it takes no such command.")
        .def("save_state", &Simulation::saveState,
             "The whole state as bytes: restore_state goes back to it and the run repeats.")
        .def("restore_state", &Simulation::restoreState, "state"_a)
        .def("set_site", &Simulation::setSite, "name"_a, "value"_a,
             "Change a site parameter while running: gravity, wind_speed, temperature, ...")
        .def_prop_ro("time", &Simulation::time, "Simulated time (s).")
        .def("read", &Simulation::read, "name"_a,
             "Latest value of a signal (all its components). KeyError for an unknown name.")
        .def("image", &Simulation::image, "name"_a,
             "Latest frame of an image signal (a camera's sensor/<n>/image or depth, a sensor "
             "kind's sensor/<n>/<q>) as a numpy array: rgba8 (height, width, 4) uint8, r32f "
             "(height, width) float32, r32u uint32, ... None before the first frame (the "
             "sensors domain on ground truth renders nothing). KeyError for an unknown name.")
        .def("signals", &Simulation::signals,
             "Every signal on the bus: dicts with name, unit and components.")
        .def("bodies", &Simulation::bodies)
        .def("joints", &Simulation::joints)
        .def("region_origin", &Simulation::regionOrigin,
             "World ENU of the region-local frame (the floating origin), m. Poses read from "
             "signals are relative to it.")
        .def("colliders", &Simulation::colliders,
             "How many collision shapes the world has now (streamed tiles come and go).")
        .def("state_hash", &Simulation::stateHash,
             "Hash of the whole state: equal hashes mean equal runs.");
}
