// ArduPilot SITL through its JSON simulator interface (ArduPilotJson.hpp).
#include "Escs.hpp"
#include "lagrange/mavlink/ArduPilotJson.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace lagrange::mavlink {

namespace {

constexpr std::uint16_t kMagic16 = 18458;  // ArduPilot's 16-channel servo frame
constexpr std::size_t kFrameBytes = 2 + 2 + 4 + 2 * 16;
constexpr double kPwmMin = 1000.0;   // µs: output 0
constexpr double kPwmSpan = 1000.0;  // µs from output 0 to output 1

SignalSpec sample(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    out.feedback = true;  // the last sample, as an autopilot gets it
    return out;
}

}  // namespace

class ArduPilotLink {
public:
    explicit ArduPilotLink(int socket) : socket_(socket) {}
    ~ArduPilotLink() { ::close(socket_); }
    ArduPilotLink(const ArduPilotLink&) = delete;
    ArduPilotLink& operator=(const ArduPilotLink&) = delete;
    ArduPilotLink(ArduPilotLink&&) = delete;
    ArduPilotLink& operator=(ArduPilotLink&&) = delete;

    /// The next servo frame's PWM values (true), or false after `timeoutS` of wall time (0: only
    /// what has arrived). Remembers who sent it, to answer.
    bool receive(double timeoutS, std::array<std::uint16_t, 16>& pwm) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutS);
        do {
            pollfd fd{socket_, POLLIN, 0};
            if (::poll(&fd, 1, timeoutS > 0.0 ? 100 : 0) <= 0) {
                continue;
            }
            std::array<std::uint8_t, 256> buffer{};
            socklen_t length = sizeof(peer_);
            const ssize_t n = ::recvfrom(socket_, buffer.data(), buffer.size(), 0,
                                         reinterpret_cast<sockaddr*>(&peer_), &length);
            std::uint16_t magic = 0;
            std::memcpy(&magic, buffer.data(), sizeof(magic));
            if (n < static_cast<ssize_t>(kFrameBytes) || magic != kMagic16) {
                continue;  // not a 16-channel servo frame
            }
            std::memcpy(pwm.data(), buffer.data() + 8, sizeof(std::uint16_t) * 16);
            if (!connected) {
                connected = true;
                services::log(services::LogLevel::Info, "ardupilot",
                              "ArduPilot connected: lockstep");
            }
            return true;
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
    }

    void answer(const std::string& line) const {
        ::sendto(socket_, line.data(), line.size(), 0, reinterpret_cast<const sockaddr*>(&peer_),
                 sizeof(peer_));
    }

    bool connected{false};
    bool started{false};           // the vehicle's start, where NED is measured from
    std::array<double, 3> home{};  // region-local ENU, m

private:
    int socket_;
    sockaddr_in peer_{};
};

namespace {

class ArduPilotJson final : public IModule {
public:
    ArduPilotJson(ArduPilotSettings settings, std::shared_ptr<ArduPilotLink> link)
        : settings_(settings), link_(std::move(link)) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"bridge", "ardupilot_json", 0, 1, false, false, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override {
        return Duration{static_cast<std::int64_t>(std::llround(1e9 / settings_.rateHz))};
    }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& sensors = world.sensors();
        std::optional<std::uint32_t> imu;
        for (std::uint32_t s = 0; s < sensors.names.size() && !imu; ++s) {
            if (sensors.type[s] == SensorType::Imu) {
                imu = s;
            }
        }
        if (!imu) {
            return Status::error(StatusCode::InvalidConfig, "ArduPilot needs an IMU on the robot");
        }
        const std::string body = "body/" + world.bodies().names[sensors.body[*imu].index] + "/";
        // Proper acceleration in the body frame (what an accelerometer measures).
        force_ =
            d.input(sample(body + "acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        pose_ = d.input(sample(body + "pose", SignalType::Pose, units::m, "world_local"));
        twist_ = d.input(sample(body + "twist", SignalType::Wrench, units::m_per_s, "world_local"));
        return escs_.declare(world, d, settings_.airDensity, settings_.maxRotorSpeed,
                             settings_.speedGain);
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        escs_.stop(bus);
        outputs_.assign(escs_.count(), 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        std::array<std::uint16_t, 16> pwm{};
        // Lockstep once ArduPilot is there: its next frame, then our state for it.
        const bool waiting = link_->connected;
        if (link_->receive(waiting ? settings_.frameTimeoutS : 0.0, pwm)) {
            for (std::size_t i = 0; i < outputs_.size(); ++i) {
                outputs_[i] = pwm[i] == 0 ? 0.0 : (pwm[i] - kPwmMin) / kPwmSpan;
            }
            link_->answer(state(context, bus));
        } else if (waiting) {
            services::log(services::LogLevel::Warning, "ardupilot",
                          "ArduPilot sent no frame in time; the motors stop until it does");
            std::fill(outputs_.begin(), outputs_.end(), 0.0);
            link_->connected = false;
        }
        escs_.drive(bus, outputs_);
        return {};
    }

    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    /// The JSON line for ArduPilot: FRD body rates and specific force, NED position and
    /// velocity from where the vehicle started, roll, pitch and yaw of FRD in NED.
    std::string state(const StepContext& context, const SignalBus& bus) {
        const std::span<const double> f = bus.readN(force_);
        const std::span<const double> p = bus.readN(pose_);
        const std::span<const double> v = bus.readN(twist_);
        // The angular rate in the body frame, from the world-frame twist.
        const std::array<double, 3> w =
            rotate(conjugate(std::array<double, 4>{p[3], p[4], p[5], p[6]}), {v[3], v[4], v[5]});
        if (!link_->started) {
            link_->home = {p[0], p[1], p[2]};
            link_->started = true;
        }
        // R (world ENU from body FLU) to NED from FRD: swap the first two rows, negate the
        // third, and negate the second and third columns.
        const std::array<double, 4> q{p[3], p[4], p[5], p[6]};
        const std::array<double, 3> x = rotate(q, {1.0, 0.0, 0.0});
        const std::array<double, 3> y = rotate(q, {0.0, 1.0, 0.0});
        const std::array<double, 3> z = rotate(q, {0.0, 0.0, 1.0});
        // Columns of R are x, y, z; NED row i is (ENU row σ(i)) with σ = (1, 0, 2), sign −1 on
        // row 2; FRD column j is FLU column j with sign −1 on columns 1 and 2.
        const auto r = [&](int row, int column) {
            const std::array<double, 3>& c = column == 0 ? x : column == 1 ? y : z;
            const double enu = row == 0 ? c[1] : row == 1 ? c[0] : -c[2];
            return column == 0 ? enu : -enu;
        };
        const double roll = std::atan2(r(2, 1), r(2, 2));
        const double pitch = -std::asin(std::clamp(r(2, 0), -1.0, 1.0));
        const double yaw = std::atan2(r(1, 0), r(0, 0));
        const double t = (context.t + context.dt).seconds();
        std::array<char, 512> line{};
        std::snprintf(line.data(), line.size(),
                      "\n{\"timestamp\":%.9g,\"imu\":{\"gyro\":[%.9g,%.9g,%.9g],"
                      "\"accel_body\":[%.9g,%.9g,%.9g]},\"position\":[%.9g,%.9g,%.9g],"
                      "\"velocity\":[%.9g,%.9g,%.9g],\"attitude\":[%.9g,%.9g,%.9g]}\n",
                      t, w[0], -w[1], -w[2], f[0], -f[1], -f[2], p[1] - link_->home[1],
                      p[0] - link_->home[0], -(p[2] - link_->home[2]), v[1], v[0], -v[2], roll,
                      pitch, yaw);
        return line.data();
    }

    ArduPilotSettings settings_;
    std::shared_ptr<ArduPilotLink> link_;
    Escs escs_;
    std::vector<double> outputs_;
    SignalId force_, pose_, twist_;
};

}  // namespace

std::shared_ptr<ArduPilotLink> openArduPilotLink(std::uint16_t port, std::string& error) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (fd < 0 || ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        if (fd >= 0) {
            ::close(fd);
        }
        error = "cannot listen for ArduPilot on UDP port " + std::to_string(port);
        return nullptr;
    }
    services::log(services::LogLevel::Info, "ardupilot",
                  "waiting for ArduPilot SITL (--model JSON) on UDP port " + std::to_string(port));
    return std::make_shared<ArduPilotLink>(fd);
}

std::unique_ptr<IModule> createArduPilotJson(const ArduPilotSettings& settings,
                                             std::shared_ptr<ArduPilotLink> link) {
    return std::make_unique<ArduPilotJson>(settings, std::move(link));
}

}  // namespace lagrange::mavlink
