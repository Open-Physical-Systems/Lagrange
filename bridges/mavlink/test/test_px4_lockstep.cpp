// The PX4 bridge over a real socket with a stand-in for PX4: it connects as PX4 does, checks the
// HIL_SENSOR it receives (FRD frame, units, lockstep time) and answers with motor outputs, which
// must come back as rotor torque commands.
#include "lagrange/mavlink/Px4Lockstep.hpp"
#include "lagrange/scheduler/Scheduler.hpp"

#include <arpa/inet.h>
#include <atomic>
#include <cmath>
#include <common/mavlink.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>

namespace lagrange::mavlink {
namespace {

using namespace literals;

constexpr std::uint16_t kPort = 24560;  // not PX4's, so a running SITL is not disturbed

/// Writes an IMU's outputs as the sensors module would: level, at rest, 1 g up in FLU.
class FakeImu final : public IModule {
public:
    [[nodiscard]] ModuleInfo info() const override {
        return {"test", "imu", 1, 0, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return 4_ms; }
    Status declare(const WorldView&, const ModuleConfig&, SignalDeclarer& d) override {
        SignalSpec f;
        f.name = "sensor/imu/specific_force";
        f.type = SignalType::Vec3;
        f.unit = units::m_per_s2;
        f.frame = "sensor";
        force_ = d.output(f);
        SignalSpec r = f;
        r.name = "sensor/imu/angular_rate";
        r.unit = units::rad_per_s;
        rate_ = d.output(r);
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
        bus.writeN(force_, std::array<double, 3>{0.1, 0.2, 9.80665});
        bus.writeN(rate_, std::array<double, 3>{0.01, 0.02, 0.03});
        for (const SignalId id : speeds_) {
            bus.write(id, 0.0);
        }
    }
    SignalId force_, rate_;
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

TEST(Px4Lockstep, SendsHilSensorInFrdAndAppliesTheMotorOutputs) {
    const World world = quad();
    Px4Settings settings;
    settings.port = kPort;
    settings.airDensity = 1.225;
    Scheduler scheduler;
    scheduler.add(std::make_unique<FakeImu>());
    std::string error;
    auto link = openPx4Link(kPort, error);
    ASSERT_TRUE(link) << error;
    scheduler.add(createPx4Lockstep(settings, link));
    auto wired = scheduler.wire(WorldView(world));
    ASSERT_TRUE(wired) << to_string(wired.error().front());

    // The stand-in for PX4, on its own thread: connect, then answer every HIL_SENSOR.
    std::atomic<int> sensors{0};
    mavlink_hil_sensor_t first{};
    std::thread px4([&] {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(kPort);
        while (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        timeval timeout{0, 200'000};  // so that it notices when the test is done
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        mavlink_status_t status{};
        std::array<std::uint8_t, 1024> buffer{};
        while (sensors < 20) {
            const ssize_t n = ::recv(fd, buffer.data(), buffer.size(), 0);
            if (n <= 0) {
                continue;
            }
            for (ssize_t i = 0; i < n; ++i) {
                mavlink_message_t m{};
                if (mavlink_parse_char(MAVLINK_COMM_1, buffer[static_cast<std::size_t>(i)], &m,
                                       &status) == 0 ||
                    m.msgid != MAVLINK_MSG_ID_HIL_SENSOR) {
                    continue;
                }
                if (sensors == 0) {
                    mavlink_msg_hil_sensor_decode(&m, &first);
                }
                ++sensors;
                mavlink_hil_actuator_controls_t out{};
                out.controls[0] = 0.5F;
                out.controls[2] = 0.5F;
                mavlink_message_t reply{};
                mavlink_msg_hil_actuator_controls_encode(1, 1, &reply, &out);
                std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
                const auto len = mavlink_msg_to_send_buffer(bytes.data(), &reply);
                ::send(fd, bytes.data(), len, MSG_NOSIGNAL);
            }
        }
        ::close(fd);
    });
    // Step until PX4 has answered a few samples.
    for (int k = 0; k < 2000 && sensors < 5; ++k) {
        ASSERT_TRUE(scheduler.stepTick().ok());
    }
    const auto& bus = scheduler.bus();
    const double ccw = bus.read(bus.find("joint/rotor_1_joint/torque_command"));
    const double cw = bus.read(bus.find("joint/rotor_3_joint/torque_command"));
    const double off = bus.read(bus.find("joint/rotor_2_joint/torque_command"));
    sensors = 100;  // let the stand-in finish
    px4.join();

    EXPECT_GE(sensors.load(), 5);
    // FLU (0.1, 0.2, 9.81) is FRD (0.1, −0.2, −9.81): PX4's accelerometer at rest reads −g on z.
    EXPECT_NEAR(first.xacc, 0.1F, 1e-6F);
    EXPECT_NEAR(first.yacc, -0.2F, 1e-6F);
    EXPECT_NEAR(first.zacc, -9.80665F, 1e-5F);
    EXPECT_NEAR(first.zgyro, -0.03F, 1e-7F);
    EXPECT_EQ(first.fields_updated & 0b111111U, 0b111111U);
    EXPECT_GT(first.time_usec, 0U);  // the end of the first step, PX4's lockstep clock
    // Half throttle: a speed set point, turning the rotors their own way.
    EXPECT_GT(ccw, 0.0);
    EXPECT_LT(cw, 0.0);
    EXPECT_EQ(off, 0.0);
}

TEST(Px4Lockstep, PressureAltitudeInvertsTheStandardAtmosphere) {
    EXPECT_NEAR(pressureAltitude(101325.0), 0.0, 1e-9);
    EXPECT_NEAR(pressureAltitude(89876.0), 1000.0, 1.0);  // ICAO table: 898.76 hPa at 1000 m
}

}  // namespace
}  // namespace lagrange::mavlink
