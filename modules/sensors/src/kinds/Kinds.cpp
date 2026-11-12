// The built-in custom sensor kinds (sensors.md §9): one line per kind in kKinds below. A kind
// here is simulated by lagrange_sensors with no plugin; the sample plugins' kinds (tof,
// ultrasonic, thermistor, ir_thermometer) stay plugins and are never listed here.
#include "KindModule.hpp"
#include "lagrange/sensors/Kinds.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <string>
#include <string_view>

namespace lagrange::sensors {

namespace kinds {
// Factories, one per kind, defined in their own files in this directory.
std::unique_ptr<IModule> createEventCamera(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createOpticalFlow(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createStereoCamera(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createBatteryMonitor(const World&, std::uint32_t, bool,
                                              const KindContext&);
std::unique_ptr<IModule> createCurrentSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createEscTelemetry(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createTensionSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createVoltageSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeAirspeed(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeAnemometer(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeHumiditySensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeLightSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeRainSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeRangefinder(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeSunSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeVisibilitySensor(const World&, std::uint32_t, bool,
                                              const KindContext&);
std::unique_ptr<IModule> makeBumper(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeCliff(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeForceTorque(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeJointTorque(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeSafetyScanner(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeWheelOdometry(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createDetections(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createLandingTarget(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createLineSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createTagReader(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createGripperSensor(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createParticleSensor(const World&, std::uint32_t, bool,
                                              const KindContext&);
std::unique_ptr<IModule> createProximity(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createTpms(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createUwb(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> createWifiScanner(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeAdsbReceiver(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeSonar(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeV2x(const World&, std::uint32_t, bool, const KindContext&);
std::unique_ptr<IModule> makeVehicleCan(const World&, std::uint32_t, bool, const KindContext&);
}  // namespace kinds

namespace {

struct Entry {
    std::string_view kind;
    kinds::KindFactory factory;
};

// Keep sorted by kind. Add a kind: its file in src/kinds/, its line here, its section in
// docs/design/domains/sensors.md §9 and its test in test/test_kinds_<kind>.cpp.
constexpr Entry kKinds[] = {
    {"adsb_receiver", &kinds::makeAdsbReceiver},
    {"airspeed", &kinds::makeAirspeed},
    {"anemometer", &kinds::makeAnemometer},
    {"battery_monitor", &kinds::createBatteryMonitor},
    {"bumper", &kinds::makeBumper},
    {"cliff", &kinds::makeCliff},
    {"current_sensor", &kinds::createCurrentSensor},
    {"detections", &kinds::createDetections},
    {"esc_telemetry", &kinds::createEscTelemetry},
    {"event_camera", &kinds::createEventCamera},
    {"force_torque", &kinds::makeForceTorque},
    {"gripper_sensor", &kinds::createGripperSensor},
    {"humidity_sensor", &kinds::makeHumiditySensor},
    {"joint_torque", &kinds::makeJointTorque},
    {"landing_target", &kinds::createLandingTarget},
    {"light_sensor", &kinds::makeLightSensor},
    {"line_sensor", &kinds::createLineSensor},
    {"optical_flow", &kinds::createOpticalFlow},
    {"particle_sensor", &kinds::createParticleSensor},
    {"proximity", &kinds::createProximity},
    {"rain_sensor", &kinds::makeRainSensor},
    {"rangefinder", &kinds::makeRangefinder},
    {"safety_scanner", &kinds::makeSafetyScanner},
    {"sonar", &kinds::makeSonar},
    {"stereo_camera", &kinds::createStereoCamera},
    {"sun_sensor", &kinds::makeSunSensor},
    {"tag_reader", &kinds::createTagReader},
    {"tension_sensor", &kinds::createTensionSensor},
    {"tpms", &kinds::createTpms},
    {"uwb", &kinds::createUwb},
    {"v2x", &kinds::makeV2x},
    {"vehicle_can", &kinds::makeVehicleCan},
    {"visibility_sensor", &kinds::makeVisibilitySensor},
    {"voltage_sensor", &kinds::createVoltageSensor},
    {"wheel_odometry", &kinds::makeWheelOdometry},
    {"wifi_scanner", &kinds::createWifiScanner},
};

// The kinds that draw the camera scene (gpu::RasterCamera), so need a Vulkan device.
constexpr std::array<std::string_view, 2> kRenderingKinds{"event_camera", "stereo_camera"};
/// Stands for a kind whose parameters cannot be used: the run refuses to start with `problem`.
class Refused final : public kinds::KindModule {
public:
    Refused(const World& world, std::uint32_t sensor, std::string problem)
        : KindModule(world, sensor, false), problem_(std::move(problem)) {}
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer&) override {
        return Status::error(StatusCode::InvalidConfig, problem_);
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus&) override { return {}; }
    Status step(const StepContext&, SignalBus&) override { return {}; }

private:
    std::string problem_;
};

const Entry* find(std::string_view kind) {
    for (const Entry& e : kKinds) {
        if (e.kind == kind) {
            return &e;
        }
    }
    return nullptr;
}

}  // namespace

bool isBuiltinKind(std::string_view kind) {
    return find(kind) != nullptr;
}

bool builtinKindRenders(std::string_view kind) {
    return std::ranges::find(kRenderingKinds, kind) != kRenderingKinds.end();
}

std::vector<std::string_view> builtinKinds() {
    std::vector<std::string_view> out;
    out.reserve(std::size(kKinds));
    for (const Entry& e : kKinds) {
        out.push_back(e.kind);
    }
    return out;
}

std::unique_ptr<IModule> createBuiltinKind(const World& world, std::uint32_t sensor,
                                           bool groundTruth,
                                           std::shared_ptr<gpu::RayContext> rayContext,
                                           std::shared_ptr<const CameraScene> cameraScene,
                                           const Scene* scene) {
    const Entry* e = find(world.sensors.kind[sensor]);
    if (e == nullptr) {
        return nullptr;
    }
    // A NaN parameter ("nan" parses as a number) passes every range check a kind makes (each
    // comparison with NaN is false) and then floods its outputs, or sizes its arrays from an
    // undefined cast: refused here for every kind.
    // An update rate of 0 (Gazebo's "as fast as possible") or below made a 1000 s period, and the
    // run was refused for steps that "repeat only after more than 10 s; check the profile's dt".
    if (!(world.sensors.rateHz[sensor] > 0.0) || !std::isfinite(world.sensors.rateHz[sensor])) {
        return std::make_unique<Refused>(
            world, sensor,
            world.sensors.kind[sensor] + " " + world.sensors.names[sensor] +
                ": update_rate must be a number of Hz above 0 (0, as fast as possible, is not "
                "taken: give the rate)");
    }
    for (const auto& [key, value] : world.sensors.settings[sensor]) {
        if (std::isnan(value)) {
            return std::make_unique<Refused>(world, sensor,
                                             world.sensors.kind[sensor] + " " +
                                                 world.sensors.names[sensor] + ": parameter " +
                                                 key + " is not a number");
        }
    }
    const kinds::KindContext context{std::move(rayContext), std::move(cameraScene), scene};
    return e->factory(world, sensor, groundTruth, context);
}

}  // namespace lagrange::sensors
