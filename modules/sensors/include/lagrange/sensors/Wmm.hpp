#pragma once

#include "lagrange/world/coords/Frames.hpp"

#include <array>

namespace lagrange::sensors {

/// The geomagnetic main field of the World Magnetic Model 2025 (NOAA NCEI and the British
/// Geological Survey, "The US/UK World Magnetic Model for 2025-2030: Technical Report", 2024;
/// coefficients in data/wmm2025, a U.S. Government work in the public domain), degree 12,
/// valid from 2025.0 to 2030.0.
///
/// Returns north, east and down components (X, Y, Z), nT, in the ellipsoidal frame at `where`
/// (WGS84 latitude and longitude in radians, height above the ellipsoid in metres) at
/// `decimalYear`. Exact to the report's equations; the field at the geographic poles is not
/// defined (division by cos of the geocentric latitude).
[[nodiscard]] std::array<double, 3> wmmField(const coords::Geodetic& where, double decimalYear);

/// The model's epoch and its last valid date (decimal years).
inline constexpr double kWmmValidFrom = 2025.0;
inline constexpr double kWmmValidUntil = 2030.0;

}  // namespace lagrange::sensors
