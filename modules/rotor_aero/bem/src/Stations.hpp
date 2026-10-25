#pragma once

// Internal to lagrange_rotor_aero: blade stations between the propeller file's.

#include "lagrange/rotor_aero/Propeller.hpp"

#include <vector>

namespace lagrange::rotor_aero {

/// The station at radius fraction `x`, interpolated linearly; held beyond the last station.
[[nodiscard]] BladeStation stationAt(const std::vector<BladeStation>& s, double x);

}  // namespace lagrange::rotor_aero
