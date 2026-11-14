#pragma once

// The closed forms behind the environment sensor kinds (sensors.md §9: anemometer, airspeed,
// rangefinder, light_sensor, sun_sensor, rain_sensor, humidity_sensor, visibility_sensor). Pure
// functions, so the tests check them against their references directly.

#include <array>
#include <optional>
#include <string_view>

namespace lagrange::sensors {

// --- Air ---------------------------------------------------------------------------------------

/// Sea-level density of the ICAO standard atmosphere (ICAO Doc 7488/3, 1993, table A): the
/// density an airspeed indicator is calibrated for, so the definition of indicated airspeed, not a
/// site default.
inline constexpr double kStandardSeaLevelDensity = 1.225;  // kg/m³

/// What a pitot-static probe's differential pressure sensor reads (Pa): the dynamic pressure
/// q = ½·ρ·V² of the flow along the probe, signed with the flow (`flowAlongProbe` m/s, positive
/// into the probe's mouth), as q = ½·ρ·V·|V|. Incompressible Bernoulli: under 1 % low against the
/// compressible form below Mach 0.3 (about 100 m/s).
[[nodiscard]] double pitotPressure(double density, double flowAlongProbe);

/// The airspeed (m/s) a differential pressure `q` (Pa) gives at density `density`:
/// sign(q)·√(2·|q|/ρ). With ρ = kStandardSeaLevelDensity it is the indicated airspeed, with the
/// air's own density the true airspeed.
[[nodiscard]] double airspeedFromPressure(double q, double density);

/// The meteorological direction (rad, [0, 2π)) the wind blows from, for air moving with
/// horizontal velocity (vx, vy) in a frame whose x is the reference ("north") and whose angles
/// grow clockwise seen from above (towards −y): atan2(vy, −vx). Wind from +x gives 0, from −y
/// π/2.
[[nodiscard]] double windFromDirection(double vx, double vy);

// --- Visibility --------------------------------------------------------------------------------

/// The meteorological optical range (m) for an extinction coefficient σ (1/m): the path that
/// leaves 5 % of a beam (WMO-No. 8, 2018 edition, Vol. I, ch. 9, §9.1.2), Koschmieder's law with a
/// 5 % contrast threshold, MOR = −ln(0.05)/σ ≈ 2.996/σ. +inf for σ ≤ 0.
inline constexpr double kMorContrast = 0.05;
[[nodiscard]] double meteorologicalOpticalRange(double extinction);

// --- First-order sensor dynamics ---------------------------------------------------------------

/// One step of a first-order lag of time constant `tau` (s) towards `input` over `dt` (s), exact
/// for an input held over the step: y + (1 − e^(−dt/τ))·(u − y). τ ≤ 0 gives the input.
[[nodiscard]] double firstOrderStep(double state, double input, double dt, double tau);

// --- Sun ---------------------------------------------------------------------------------------

/// Julian day (UT) of a proleptic Gregorian date and time (Meeus, Astronomical Algorithms, 2nd ed.
/// 1998, ch. 7, eq. 7.1).
[[nodiscard]] double julianDay(int year, int month, int day, double hour, double minute,
                               double second);

/// "YYYY-MM-DDTHH:MM[:SS[.s]][Z]" (UTC, ISO 8601 basic calendar form) as a Julian day, or empty.
[[nodiscard]] std::optional<double> parseUtc(std::string_view text);

/// Where the sun is, seen from a place on the Earth.
struct SolarPosition {
    double elevation{0.0};       // rad above the horizon, refraction included
    double azimuth{0.0};         // rad, from north clockwise (east π/2)
    double declination{0.0};     // rad
    double equationOfTime{0.0};  // minutes, apparent less mean solar time
    /// Unit vector towards the sun in east, north, up.
    [[nodiscard]] std::array<double, 3> enu() const;
};

/// The NOAA solar position algorithm (NOAA Global Monitoring Laboratory's Solar Calculator, after
/// Meeus 1998, chs. 25 and 28; accuracy about 0.01° for years 1800 to 2100) at Julian day `jd`
/// (UT) for geodetic latitude and east longitude (rad), with NOAA's approximate atmospheric
/// refraction.
[[nodiscard]] SolarPosition solarPosition(double jd, double latitudeRad, double longitudeRad);

/// Relative optical air mass at true solar zenith angle `zenith` (rad): Kasten and Young (1989,
/// Appl. Opt. 28(22), 4735–4738), m = 1 / (cos z + 0.50572·(96.07995° − z)^(−1.6364)), z in
/// degrees in the bracket. Valid to the horizon (m ≈ 38).
[[nodiscard]] double airMass(double zenith);

/// Daylight on a clear or overcast sky: the model of Gillette, Pierpoint and Treado (NBS, 1982)
/// that the IES daylight availability practice took up (J. IES 13(4), 1984, 381–392; not read),
/// as Robbins and Hunter quote it (SERI/TR-254-1703, 1983, eqs. 2-16, 2-17, 3-6): extraterrestrial
/// illuminance E_xt = 127.5 klx · (1 + 0.033·cos(2π·day/365)), direct normal E_dn = E_xt·exp(−c·m)
/// and no direct beam under an overcast sky, and diffuse horizontal E_dh = A + B·sin^C(α) klx
/// (clear A = 0.8, B = 15.5, C = 0.5; overcast A = 0.3, B = 21.0, C = 1.0). c = 0.21 per air mass
/// is the rural design illuminance extinction of LBL-17727 (Lawrence Berkeley Laboratory, 1984);
/// m is Kasten–Young's air mass where the source has 1/sin α. Night (α ≤ 0): both zero, twilight
/// is not modelled.
inline constexpr double kExtraterrestrialIlluminance = 127.5e3;  // lx, IES 1984
inline constexpr double kClearSkyExtinction = 0.21;              // per air mass, LBL-17727
struct Daylight {
    double directNormal{0.0};       // lx, on a surface facing the sun
    double diffuseHorizontal{0.0};  // lx, the sky's on a level surface
};
[[nodiscard]] Daylight iesDaylight(double elevation, int dayOfYear, bool overcast);

/// Illuminance (lx) on a surface of unit normal `normal` (any frame) under the sun at unit
/// `sun` (same frame, z up): the direct beam × cos of incidence (zero from behind or when
/// `sunBlocked`), times `beamTransmission`, plus the sky's light by the isotropic sky model
/// (Liu and Jordan 1963: view factor (1 + n_z)/2) and, with ground reflectance ρ_g, the ground's
/// ρ_g·E_gh·(1 − n_z)/2, E_gh the global horizontal illuminance.
[[nodiscard]] double surfaceIlluminance(const Daylight& light, const std::array<double, 3>& sun,
                                        const std::array<double, 3>& normal, bool sunBlocked,
                                        double beamTransmission, double groundReflectance);

}  // namespace lagrange::sensors
