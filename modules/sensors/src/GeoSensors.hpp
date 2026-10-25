#pragma once

// Internal to lagrange_sensors: the sensors that need the world's geo origin.

#include "lagrange/module/IModule.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::sensors {

/// The EMI victim a sensor is (an EmiSusceptibilityAPI_1 of `victimType` on its prim): the
/// victim's signal prefix "emi/<p>/", or empty. Its inputs are optional, zero without the EMI
/// domain.
[[nodiscard]] std::string emiVictim(const WorldView& world, std::uint32_t sensor,
                                    std::string_view victimType);

/// An optional input that reads zero when nothing writes it.
[[nodiscard]] SignalSpec optionalSpec(std::string name, SignalType type, Unit unit,
                                      std::string frame = {});

/// Adds a module for every GNSS receiver, magnetometer and barometer of `world`, when the world
/// has a geo origin; without one it adds none and logs why.
void createGeoSensors(const World& world, bool groundTruth,
                      std::vector<std::unique_ptr<IModule>>& modules);

}  // namespace lagrange::sensors
