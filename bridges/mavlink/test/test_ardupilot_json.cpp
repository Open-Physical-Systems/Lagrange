// The ArduPilot bridge over a real socket with a stand-in for ArduPilot: it sends servo frames as
// ArduPilot's JSON backend does, checks the state line it gets back (FRD and NED frames, from the
// start) and the PWM values must come back as rotor torque commands.
#include "lagrange/mavlink/ArduPilotJson.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::mavlink {
namespace {

using namespace literals;

constexpr std::uint16_t kPort = 29002;  // not ArduPilot's, so a running SITL is not disturbed

/// Writes the base body's state as physics would: 10 m east, 20 m north, 5 m up, nose north
/// (90° yaw in ENU), moving (1, 2, 3) m/s and turning 0.5 rad/s left, accelerometer at 1 g up.
class FakeBody final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "body", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 1_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec pose;
        pose.name = "body/base/pose";
        pose.type = SignalType::Pose;
        pose.unit = units::m;
        pose.frame = "world_local";
        pose_ = d.output(pose);
        SignalSpec twist = pose;
        twist.name = "body/base/twist";
        twist.type = SignalType::Wrench;
        twist.unit = units::m_per_s;
        twist_ = d.output(twist);
        SignalSpec acceleration = twist;
        acceleration.name = "body/base/acceleration";
        acceleration.unit = units::m_per_s2;
        acceleration.frame = "body";
        acceleration_ = d.output(acceleration);
        for (int i = 1; i <= 4; ++i) {
            SignalSpec v;
            v.name = "joint/rotor_" + std::to_string(i) + "_joint/velocity";
            v.unit = units::rad_per_s;
            speeds_.push_back(d.output(v));
        }
        return {};
    }
    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status step(const StepContext&, SignalBus& bus) override {
        write(bus);
        return {};
    }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

private:
    void write(SignalBus& bus) const {
        const double h = std::sqrt(0.5);
        bus.writeN(pose_, std::array<double, 7>{10.0, 20.0, 5.0, h, 0.0, 0.0, h});
        bus.writeN(twist_, std::array<double, 6>{1.0, 2.0, 3.0, 0.0, 0.0, 0.5});
        bus.writeN(acceleration_, std::array<double, 6>{0.1, 0.2, 9.80665, 0.0, 0.0, 0.0});
        for (const SignalId id : speeds_) {
            bus.write(id, 0.0);
        }
    }
    SignalId pose_, twist_, acceleration_;
    std::vector<SignalId> speeds_;
};

World quad() {
    World w;
    const BodyId base = w.addBody("base", kWorldBody, Pose{}, 2.0, {}, {0.02, 0.02, 0.04, 0, 0, 0});
    w.addJoint("base_free", JointType::Free, base, {0, 0, 1}, false);
    w.addSensor("imu", SensorType::Imu, base, Pose{}, 250.0, {});
    for (int i = 1; i <= 4; ++i) {
        const BodyId rotor = w.addBody("rotor_" + std::to_string(i) + "_link", base, Pose{}, 0.025,
                                       {}, {4.5e-5, 4.5e-5, 9e-5, 0, 0, 0});
        w.addJoint("rotor_" + std::to_string(i) + "_joint", JointType::Continuous, rotor, {0, 0, 1},
                   true);
        w.schemaInstances.push_back({});
        SchemaInstance& s = w.schemaInstances.back();
        s.schema = "RotorAPI_1";
        s.prim = w.joints.prim.back();
        s.reals = {{"aero:rotor:radius", 0.127},
                   {"aero:rotor:thrustCoeff", 0.125},
                   {"aero:rotor:powerCoeff", 0.0507}};
        s.tokens = {{"aero:rotor:direction", i <= 2 ? "ccw" : "cw"}};
    }
    return w;
}

/// The three numbers after `"key":[` in a JSON line.
std::array<double, 3> triple(const std::string& line, const std::string& key) {
    std::array<double, 3> out{NAN, NAN, NAN};
    const std::size_t at = line.find("\"" + key + "\":[");
    if (at != std::string::npos) {
        std::sscanf(line.c_str() + at + key.size() + 4, "%lf,%lf,%lf", &out[0], &out[1], &out[2]);
    }
    return out;
}

TEST(ArduPilotJson, AnswersServoFramesWithTheStateInFrdAndNed) {
    const World world = quad();
    ArduPilotSettings settings;
    settings.port = kPort;
    settings.airDensity = 1.225;
    Scheduler scheduler;
    scheduler.add(std::make_unique<FakeBody>());
    std::string error;
    auto link = openArduPilotLink(kPort, error);
    ASSERT_TRUE(link) << error;
    scheduler.add(createArduPilotJson(settings, link));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());

    // The stand-in for ArduPilot: a servo frame, then the next one for each answer.
    std::atomic<int> answers{0};
    std::atomic<bool> done{false};
    std::string first;
    std::thread ardupilot([&] {
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        timeval timeout{0, 20'000};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(kPort);
        std::array<std::uint8_t, 40> frame{};
        const std::uint16_t magic = 18458;
        const std::uint16_t rate = 1000;
        std::memcpy(frame.data(), &magic, 2);
        std::memcpy(frame.data() + 2, &rate, 2);
        // Half throttle on motors 1 and 3; motor 2 at its minimum, motor 4 unset.
        const std::array<std::uint16_t, 4> pwm{1500, 1000, 1500, 0};
        std::memcpy(frame.data() + 8, pwm.data(), sizeof(pwm));
        for (std::uint32_t count = 0; !done; ++count) {
            std::memcpy(frame.data() + 4, &count, 4);
            ::sendto(fd, frame.data(), frame.size(), 0, reinterpret_cast<sockaddr*>(&address),
                     sizeof(address));
            std::array<char, 1024> buffer{};
            const ssize_t n = ::recv(fd, buffer.data(), buffer.size() - 1, 0);
            if (n > 0) {
                if (answers == 0) {
                    first.assign(buffer.data(), static_cast<std::size_t>(n));
                }
                ++answers;
            }
        }
        ::close(fd);
    });
    for (int k = 0; k < 5000 && answers < 5; ++k) {
        ASSERT_TRUE(scheduler.stepTick().ok());
    }
    const auto& bus = scheduler.bus();
    const double ccw = bus.read(bus.find("joint/rotor_1_joint/torque_command"));
    const double minimum = bus.read(bus.find("joint/rotor_2_joint/torque_command"));
    const double cw = bus.read(bus.find("joint/rotor_3_joint/torque_command"));
    const double unset = bus.read(bus.find("joint/rotor_4_joint/torque_command"));
    done = true;
    ardupilot.join();

    ASSERT_GE(answers.load(), 5);
    // A line of JSON, between newlines, as ArduPilot's parser wants it.
    EXPECT_EQ(first.front(), '\n');
    EXPECT_EQ(first.back(), '\n');
    EXPECT_NE(first.find("\"timestamp\":"), std::string::npos);
    const auto gyro = triple(first, "gyro");
    const auto accel = triple(first, "accel_body");
    const auto position = triple(first, "position");
    const auto velocity = triple(first, "velocity");
    const auto attitude = triple(first, "attitude");
    // Turning left in FLU is a negative yaw rate in FRD; 1 g up in FLU is −g on FRD's z.
    EXPECT_NEAR(gyro[2], -0.5, 1e-9);
    EXPECT_NEAR(gyro[0], 0.0, 1e-9);
    EXPECT_NEAR(accel[0], 0.1, 1e-9);
    EXPECT_NEAR(accel[1], -0.2, 1e-9);
    EXPECT_NEAR(accel[2], -9.80665, 1e-9);
    // Position from where it started; ENU velocity (1, 2, 3) is NED (2, 1, −3).
    EXPECT_NEAR(position[0], 0.0, 1e-9);
    EXPECT_NEAR(position[2], 0.0, 1e-9);
    EXPECT_NEAR(velocity[0], 2.0, 1e-9);
    EXPECT_NEAR(velocity[1], 1.0, 1e-9);
    EXPECT_NEAR(velocity[2], -3.0, 1e-9);
    // Nose north, level: zero roll, pitch and yaw in NED.
    EXPECT_NEAR(attitude[0], 0.0, 1e-9);
    EXPECT_NEAR(attitude[1], 0.0, 1e-9);
    EXPECT_NEAR(attitude[2], 0.0, 1e-9);
    // Half throttle turns each rotor its own way; 1000 µs and an unset channel stop it.
    EXPECT_GT(ccw, 0.0);
    EXPECT_LT(cw, 0.0);
    EXPECT_EQ(minimum, 0.0);
    EXPECT_EQ(unset, 0.0);
}

}  // namespace
}  // namespace lagrange::mavlink
