#pragma once

// Internal to lagrange_sensors: the sensors that need the world's geo origin.

#include "lagrange/module/IModule.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <memory>
#include <vector>

namespace lagrange::sensors {

/// Adds a module for every GNSS receiver, magnetometer and barometer of `world`, when the world
/// has a geo origin; without one it adds none and logs why.
void createGeoSensors(const World& world, bool groundTruth,
                      std::vector<std::unique_ptr<IModule>>& modules);

}  // namespace lagrange::sensors
