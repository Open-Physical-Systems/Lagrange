#pragma once

#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace lagrange::mavlink {

/// ArduPilot SITL through its JSON simulator interface (libraries/SITL/examples/JSON/readme.md of
/// ArduPilot, read at Copter-4.7.2): ArduPilot sends binary servo frames (magic 18458, frame rate
/// and count, 16 PWM values in µs) to UDP 9002; the simulator answers each one with a line of
/// JSON: timestamp (s), gyro and accel_body (FRD body frame), position and velocity (NED, m and
/// m/s, from where the vehicle started) and attitude (roll, pitch, yaw, rad). The inertial data
/// is the IMU's body's, exact, at the exchange rate (the IMU sensor's 250 Hz is too slow for
/// ArduPilot); ArduPilot adds its own sensor noise and makes its GPS, compass and barometer from
/// that state itself.
///
/// Lockstep: once ArduPilot has sent a first frame, each step waits for the next one, applies
/// its outputs ((PWM − 1000)/1000 for the i-th rotor joint, as Px4Lockstep's ESCs) and answers
/// with the state. ArduCopter's quad X motor order is PX4's: rotor_1 front right,
/// counter-clockwise. ArduPilot is GPL-3.0 and runs as its own program; nothing of it is linked.
struct ArduPilotSettings {
    std::uint16_t port{9002};     // UDP port ArduPilot sends its servo frames to
    double rateHz{1000.0};        // the exchange rate (ArduPilot's SIM_RATE_HZ; its pre-arm
                                  // check wants gyro data at 1.8 times its 400 Hz loop)
    double maxRotorSpeed{850.0};  // rad/s at full output
    double speedGain{0.02};       // N·m per rad/s of rotor speed error
    double airDensity{0.0};       // kg/m³, for the ESCs' feed-forward
    double frameTimeoutS{5.0};    // s of wall time to wait for a frame before giving up a step
};

/// The UDP link to ArduPilot, kept across rebuilds of the run (as Px4Link).
class ArduPilotLink;
[[nodiscard]] std::shared_ptr<ArduPilotLink> openArduPilotLink(std::uint16_t port,
                                                               std::string& error);

/// The bridge as a scheduler module on `link`: the IMU's body and the world's first IMU; refuses a
/// world without an IMU or without rotors.
[[nodiscard]] std::unique_ptr<IModule> createArduPilotJson(const ArduPilotSettings& settings,
                                                           std::shared_ptr<ArduPilotLink> link);

}  // namespace lagrange::mavlink
