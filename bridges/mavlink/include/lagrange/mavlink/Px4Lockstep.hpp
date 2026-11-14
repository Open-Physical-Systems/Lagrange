#pragma once

#include "lagrange/module/IModule.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::mavlink {

/// The PX4 SITL simulator interface over MAVLink (docs/design/robot-classes.md §5), verified
/// against PX4 v1.17.0's simulator_mavlink module: PX4 connects as a TCP client to port
/// 4560 + its instance; each HIL_SENSOR whose id is 0 sets PX4's lockstep clock to its time_usec
/// and lets PX4 run up to it; PX4 then answers with HIL_ACTUATOR_CONTROLS.
///
/// The module steps at the IMU's rate. Each step it sends HIL_SENSOR (IMU specific force and
/// angular rate, magnetometer in gauss, barometer in hPa, all turned from the sensors' x forward,
/// y left, z up into PX4's FRD body frame) and, at the GNSS receiver's rate, HIL_GPS; once PX4 is
/// connected it waits for the HIL_ACTUATOR_CONTROLS that answers (lockstep: the simulation never
/// runs ahead of the autopilot). Until PX4 connects the motors are off and the run is free.
///
/// The ESCs: control i (0 to 1, PX4's motor order) is a speed set point i·top for the i-th rotor
/// joint (RotorAPI_1, in name order), held by a speed loop whose torque command, with the
/// propeller's drag as feed-forward, goes to the electrical domain like any controller's. `top`
/// is maxRotorSpeed when given, else each rotor's loaded top speed from its motor and battery
/// (rotorTopSpeed, RotorTopSpeed.hpp).
struct Px4Settings {
    std::uint16_t port{4560};             // TCP port PX4 connects to (4560 + instance)
    std::optional<double> maxRotorSpeed;  // rad/s at control 1; none: from the motors
    double thrustLimit{1.0};              // share of full thrust at control 1 (--thrust-limit)
    double speedGain{0.02};               // N·m per rad/s of rotor speed error
    double airDensity{0.0};               // kg/m³, for the feed-forward (rotor_aero's)
    double actuatorTimeoutS{5.0};         // s of wall time to wait for PX4 before giving up a step
    // The electrical domain is off (its ideal torque source): the ESCs bound each rotor's torque
    // command by its drive's and cut the speed gain to what the step allows (Escs.hpp).
    bool idealMotors{false};
};

/// The TCP link to PX4: listens on `port` from creation, and keeps PX4's connection across
/// rebuilds of the run (a profile applied, a reset), which make new bridge modules. Not thread
/// safe: one run steps it at a time.
class Px4Link;
[[nodiscard]] std::shared_ptr<Px4Link> openPx4Link(std::uint16_t port, std::string& error);

/// The bridge as a scheduler module on `link`. Its sensors are the world's first IMU,
/// magnetometer, barometer and GNSS receiver; it refuses a world without an IMU or without
/// rotors.
[[nodiscard]] std::unique_ptr<IModule> createPx4Lockstep(const Px4Settings& settings,
                                                         std::shared_ptr<Px4Link> link);

/// HIL_SENSOR's pressure altitude (m) for a static pressure (Pa): the ICAO standard atmosphere
/// solved for height, as PX4 expects it.
[[nodiscard]] double pressureAltitude(double pressurePa);

}  // namespace lagrange::mavlink
