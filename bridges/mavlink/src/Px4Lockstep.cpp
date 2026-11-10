// PX4 SITL in lockstep over MAVLink (Px4Lockstep.hpp).
#include "Escs.hpp"
#include "lagrange/mavlink/Px4Lockstep.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <numbers>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

// The generated MAVLink C headers (MIT, the mavlink package's generator-output exception).
#include <common/mavlink.h>

namespace lagrange::mavlink {

namespace {

using Vec3 = std::array<double, 3>;

constexpr std::uint8_t kSystemId = 1;           // the simulator's MAVLink system
constexpr std::uint8_t kComponentId = 51;       // MAV_COMP_ID_USER27, unused by PX4
constexpr std::uint32_t kAccelGyro = 0b111111;  // HIL_SENSOR fields_updated bits
constexpr std::uint32_t kMag = 0b111000000;
constexpr std::uint32_t kBaro = 0b1101000000000;  // abs_pressure, pressure_alt, temperature
constexpr double kGaussPerTesla = 1e4;
constexpr double kKelvinAtZeroCelsius = 273.15;
// ICAO standard atmosphere (as the barometer, lagrange::sensors::standardAtmosphere).
constexpr double kSeaLevelPressurePa = 101325.0;
constexpr double kSeaLevelTemperatureK = 288.15;
constexpr double kLapseRateKPerM = 0.0065;
constexpr double kPressureExponent = 5.25588;

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

/// A sensor reading as the autopilot gets it: the last sample, read before this step's
/// physics (it breaks the loop sensors → autopilot → motors → physics → sensors).
SignalSpec sample(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out = spec(std::move(name), type, unit, std::move(frame));
    out.feedback = true;
    return out;
}

/// x forward, y left, z up to PX4's x forward, y right, z down.
Vec3 frd(std::span<const double> v) {
    return {v[0], -v[1], -v[2]};
}

}  // namespace

class Px4Link {
public:
    explicit Px4Link(int listener) : listener_(listener) {}
    ~Px4Link() {
        for (int fd : {client_, listener_}) {
            if (fd >= 0) {
                ::close(fd);
            }
        }
    }
    Px4Link(const Px4Link&) = delete;
    Px4Link& operator=(const Px4Link&) = delete;
    Px4Link(Px4Link&&) = delete;
    Px4Link& operator=(Px4Link&&) = delete;

    [[nodiscard]] bool connected() const { return client_ >= 0; }

    void accept() {
        client_ = ::accept(listener_, nullptr, nullptr);
        if (client_ >= 0) {
            int yes = 1;
            ::setsockopt(client_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
            answering = false;
            services::log(services::LogLevel::Info, "mavlink", "PX4 connected: lockstep");
        }
    }

    void drop() {
        services::log(services::LogLevel::Warning, "mavlink", "PX4 disconnected");
        ::close(client_);
        client_ = -1;
        answering = false;
    }

    void send(const mavlink_message_t& message) {
        std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        const std::uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &message);
        if (::send(client_, buffer.data(), length, MSG_NOSIGNAL) != length) {
            drop();
        }
    }

    /// Reads until a HIL_ACTUATOR_CONTROLS arrives (true, its controls in `controls`) or
    /// `timeoutS` of wall time passes (false); 0 only takes what has arrived already.
    bool awaitControls(double timeoutS, std::array<float, 16>& controls) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutS);
        do {
            pollfd fd{client_, POLLIN, 0};
            if (::poll(&fd, 1, timeoutS > 0.0 ? 100 : 0) <= 0) {
                continue;
            }
            std::array<std::uint8_t, 2048> buffer{};
            const ssize_t n = ::recv(client_, buffer.data(), buffer.size(), 0);
            if (n <= 0) {
                drop();
                return false;
            }
            bool got = false;
            for (ssize_t i = 0; i < n; ++i) {
                mavlink_message_t message{};
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[static_cast<std::size_t>(i)],
                                       &message, &status_) != 0 &&
                    message.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                    mavlink_hil_actuator_controls_t out{};
                    mavlink_msg_hil_actuator_controls_decode(&message, &out);
                    std::copy(std::begin(out.controls), std::end(out.controls), controls.begin());
                    got = true;
                }
            }
            if (got) {
                return true;
            }
        } while (client_ >= 0 && std::chrono::steady_clock::now() < deadline);
        return false;
    }

    bool answering{false};  // PX4 has sent outputs on this connection: lockstep is on

private:
    int listener_{-1};
    int client_{-1};
    mavlink_status_t status_{};
};

namespace {

class Px4Lockstep final : public IModule {
public:
    Px4Lockstep(Px4Settings settings, std::shared_ptr<Px4Link> link)
        : settings_(settings), link_(std::move(link)) {}
    ~Px4Lockstep() override = default;
    Px4Lockstep(const Px4Lockstep&) = delete;
    Px4Lockstep& operator=(const Px4Lockstep&) = delete;
    Px4Lockstep(Px4Lockstep&&) = delete;
    Px4Lockstep& operator=(Px4Lockstep&&) = delete;

    [[nodiscard]] ModuleInfo info() const override {
        // Its input comes from outside: not deterministic.
        return {"bridge", "mavlink_px4", 0, 1, false, false, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& sensors = world.sensors();
        const auto first = [&sensors](SensorType type) -> std::optional<std::uint32_t> {
            for (std::uint32_t s = 0; s < sensors.names.size(); ++s) {
                if (sensors.type[s] == type) {
                    return s;
                }
            }
            return std::nullopt;
        };
        const auto imu = first(SensorType::Imu);
        if (!imu) {
            return Status::error(StatusCode::InvalidConfig, "PX4 needs an IMU on the robot");
        }
        step_ = Duration{static_cast<std::int64_t>(std::llround(1e9 / sensors.rateHz[*imu]))};
        const std::string imuBase = "sensor/" + sensors.names[*imu] + "/";
        force_ = d.input(
            sample(imuBase + "specific_force", SignalType::Vec3, units::m_per_s2, "sensor"));
        rate_ =
            d.input(sample(imuBase + "angular_rate", SignalType::Vec3, units::rad_per_s, "sensor"));
        if (const auto mag = first(SensorType::Magnetometer); mag && world.raw().geoOrigin) {
            field_ = d.input(sample("sensor/" + sensors.names[*mag] + "/field", SignalType::Vec3,
                                    units::T, "sensor"));
            magEvery_ = everyOf(sensors.rateHz[*mag], sensors.rateHz[*imu]);
        }
        if (const auto baro = first(SensorType::Barometer); baro && world.raw().geoOrigin) {
            const std::string base = "sensor/" + sensors.names[*baro] + "/";
            pressure_ = d.input(sample(base + "pressure", SignalType::Scalar, units::Pa));
            temperature_ = d.input(sample(base + "temperature", SignalType::Scalar, units::K));
            baroEvery_ = everyOf(sensors.rateHz[*baro], sensors.rateHz[*imu]);
        }
        if (const auto gnss = first(SensorType::Gnss); gnss && world.raw().geoOrigin) {
            const std::string base = "sensor/" + sensors.names[*gnss] + "/";
            latitude_ = d.input(sample(base + "latitude", SignalType::Scalar, units::rad));
            longitude_ = d.input(sample(base + "longitude", SignalType::Scalar, units::rad));
            altitude_ = d.input(sample(base + "altitude", SignalType::Scalar, units::m));
            velocity_ = d.input(sample(base + "velocity", SignalType::Vec3, units::m_per_s, "enu"));
            gpsEvery_ = everyOf(sensors.rateHz[*gnss], sensors.rateHz[*imu]);
        }

        // The rotors in name order: rotor_1 is PX4's motor 1.
        return escs_.declare(world, d, settings_.airDensity, settings_.maxRotorSpeed,
                             settings_.speedGain, "mavlink");
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        escs_.stop(bus);
        controls_.fill(0.0F);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (!link_->connected()) {
            link_->accept();
        }
        if (link_->connected()) {
            sendSensors(context, bus);
            // Lockstep: once PX4 has produced outputs it answers every IMU sample with them.
            // While it boots it sends none, so until the first arrives nothing waits.
            const bool waiting = link_->answering;
            if (!link_->awaitControls(waiting ? settings_.actuatorTimeoutS : 0.0, controls_)) {
                if (waiting) {
                    services::log(services::LogLevel::Warning, "mavlink",
                                  "PX4 did not answer in time; the motors stop until it does");
                    controls_.fill(0.0F);
                    link_->answering = false;
                }
            } else {
                link_->answering = true;
            }
        }
        std::array<double, 16> outputs{};
        std::ranges::copy(controls_, outputs.begin());
        escs_.drive(bus, outputs);
        return {};
    }

    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    static std::uint32_t everyOf(double rate, double imuRate) {
        return std::max<std::uint32_t>(1, static_cast<std::uint32_t>(std::lround(imuRate / rate)));
    }

    void sendSensors(const StepContext& context, SignalBus& bus) {
        const auto usec = static_cast<std::uint64_t>(context.t.ns / 1000) + step_.ns / 1000;
        mavlink_hil_sensor_t sensor{};
        sensor.time_usec = usec;
        const Vec3 a = frd(bus.readN(force_));
        const Vec3 w = frd(bus.readN(rate_));
        sensor.xacc = static_cast<float>(a[0]);
        sensor.yacc = static_cast<float>(a[1]);
        sensor.zacc = static_cast<float>(a[2]);
        sensor.xgyro = static_cast<float>(w[0]);
        sensor.ygyro = static_cast<float>(w[1]);
        sensor.zgyro = static_cast<float>(w[2]);
        sensor.fields_updated = kAccelGyro;
        if (field_.valid() && context.stepIndex % magEvery_ == 0) {
            const Vec3 b = frd(bus.readN(field_));
            sensor.xmag = static_cast<float>(b[0] * kGaussPerTesla);
            sensor.ymag = static_cast<float>(b[1] * kGaussPerTesla);
            sensor.zmag = static_cast<float>(b[2] * kGaussPerTesla);
            sensor.fields_updated |= kMag;
        }
        if (pressure_.valid() && context.stepIndex % baroEvery_ == 0) {
            const double pa = bus.read(pressure_);
            sensor.abs_pressure = static_cast<float>(pa / 100.0);
            sensor.pressure_alt = static_cast<float>(pressureAltitude(pa));
            sensor.temperature = static_cast<float>(bus.read(temperature_) - kKelvinAtZeroCelsius);
            sensor.fields_updated |= kBaro;
        }
        sensor.id = 0;
        mavlink_message_t message{};
        mavlink_msg_hil_sensor_encode(kSystemId, kComponentId, &message, &sensor);
        link_->send(message);
        if (link_->connected() && latitude_.valid() && context.stepIndex % gpsEvery_ == 0) {
            const std::span<const double> v = bus.readN(velocity_);  // east, north, up
            mavlink_hil_gps_t gps{};
            gps.time_usec = usec;
            constexpr double kDegE7 = 1e7 * 180.0 / std::numbers::pi;
            gps.lat = static_cast<std::int32_t>(std::lround(bus.read(latitude_) * kDegE7));
            gps.lon = static_cast<std::int32_t>(std::lround(bus.read(longitude_) * kDegE7));
            gps.alt = static_cast<std::int32_t>(std::lround(bus.read(altitude_) * 1000.0));
            gps.eph = 100;  // HDOP 1.0
            gps.epv = 100;
            gps.vn = static_cast<std::int16_t>(std::lround(v[1] * 100.0));
            gps.ve = static_cast<std::int16_t>(std::lround(v[0] * 100.0));
            gps.vd = static_cast<std::int16_t>(std::lround(-v[2] * 100.0));
            gps.vel = static_cast<std::uint16_t>(std::lround(std::hypot(v[0], v[1]) * 100.0));
            double course = std::atan2(v[0], v[1]) * 180.0 / std::numbers::pi;
            course += course < 0.0 ? 360.0 : 0.0;
            gps.cog = static_cast<std::uint16_t>(std::lround(course * 100.0) % 36000);
            gps.fix_type = 3;
            gps.satellites_visible = 10;
            gps.id = 0;
            mavlink_msg_hil_gps_encode(kSystemId, kComponentId, &message, &gps);
            link_->send(message);
        }
    }

    Px4Settings settings_;
    Duration step_{4'000'000};
    Escs escs_;
    std::array<float, 16> controls_{};
    std::uint32_t magEvery_{1}, baroEvery_{1}, gpsEvery_{1};
    SignalId force_, rate_, field_, pressure_, temperature_;
    SignalId latitude_, longitude_, altitude_, velocity_;
    std::shared_ptr<Px4Link> link_;
};

}  // namespace

double pressureAltitude(double pressurePa) {
    return kSeaLevelTemperatureK / kLapseRateKPerM *
           (1.0 - std::pow(pressurePa / kSeaLevelPressurePa, 1.0 / kPressureExponent));
}

std::shared_ptr<Px4Link> openPx4Link(std::uint16_t port, std::string& error) {
    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (listener < 0 ||
        ::bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
        ::listen(listener, 1) != 0) {
        if (listener >= 0) {
            ::close(listener);
        }
        error = "cannot listen for PX4 on TCP port " + std::to_string(port);
        return nullptr;
    }
    ::fcntl(listener, F_SETFL, O_NONBLOCK);
    services::log(services::LogLevel::Info, "mavlink",
                  "waiting for PX4 SITL on TCP port " + std::to_string(port));
    return std::make_shared<Px4Link>(listener);
}

std::unique_ptr<IModule> createPx4Lockstep(const Px4Settings& settings,
                                           std::shared_ptr<Px4Link> link) {
    return std::make_unique<Px4Lockstep>(settings, std::move(link));
}

}  // namespace lagrange::mavlink
