#pragma once

// What the weather does to sensors (domains/weather.md §2, §6): the lidar's detection range in
// an attenuating atmosphere and behind a dirty window, false returns from fog and rain, and the
// camera's veil. Pure functions, used by the lidar and camera modules.

#include <cstdint>
#include <span>

namespace lagrange::sensors {

/// The weather at a sensor, from the weather domain's signals.
struct SensorWeather {
    double extinction{0.0};     // 1/m, at the sensor's wavelength (fog, rain, dust)
    double fogExtinction{0.0};  // 1/m, the part that backscatters as fog does
    double rainRate{0.0};       // m/s
    double contamination{0.0};  // 0 clean … 1 opaque window or lens
};

/// The lidar's detection range against its reference target. The lidar equation for an extended
/// Lambertian target gives received power ∝ T²·exp(−2σr)/r² (window transmission T = 1 − c,
/// two-way Beer–Lambert); the sensor's max range R is where that power meets the threshold in
/// clear air with a clean window, so the range in weather solves r·exp(σr) = R·T (Lambert's W:
/// r = W(σ·R·T)/σ).
[[nodiscard]] double lidarDetectionRange(double maxRange, double extinction, double contamination);

/// Lidar settings for false returns (behavioural: domains/weather.md §7 asks for order of
/// magnitude and trend, not a point match; TODO(verify): calibrate against published fog- and
/// rain-chamber statistics, e.g. the DENSE project's).
struct LidarClutter {
    double pulseLength{0.75};      // m, c·τ/2 for a 5 ns pulse
    double lidarRatio{18.5};       // sr, extinction over backscatter of fog droplets (TODO(verify))
    double overlap{2.0};           // m, where the receiver sees the beam (overlap 1 − e^−(r/o)⁴)
    double reflectivity{0.1};      // the reference target's reflectivity, at max range
    double gain{0.11};             // fog backscatter detection gain, set so that half the beams of
                                   // a 30 m lidar see clutter at V = 60 m (uncalibrated)
    double speckle{0.5};           // log-normal spread of the fog return (σ of ln)
    double beamDiameter{0.005};    // m, for rain drops in the beam
    double rainRange{2.0};         // m, out to where a single drop returns enough
    double minDropDiameter{5e-4};  // m, the smallest drop that returns enough
};

/// The fog's backscatter return relative to the threshold at range r (the peak over the range
/// is where clutter would appear): g(r) = η·O(r)·(σ/S)·Δr·exp(−2σr)·T²·π·R² / (ρ_ref·r²).
[[nodiscard]] double fogReturn(const LidarClutter& c, double maxRange, double fogExtinction,
                               double contamination, double range);

/// Range of the fog return's peak, and the probability that a beam reports it (the peak times a
/// log-normal speckle factor meets the threshold).
struct Clutter {
    double range{0.0};
    double probability{0.0};
};
[[nodiscard]] Clutter fogClutter(const LidarClutter& c, double maxRange, double fogExtinction,
                                 double contamination, double minRange);

/// The probability that a beam meets a drop that returns enough: Marshall–Palmer's number of
/// drops at least minDropDiameter across, in the beam out to rainRange.
[[nodiscard]] double rainClutterProbability(const LidarClutter& c, double rainRateMps,
                                            double minRange);

/// One beam's range in weather: the true range if the target is within the detection range,
/// else no return (+inf); replaced by a nearer false return from fog or rain when the beam's
/// uniform draws (u1 for fog, u2 for rain, u3 for where) say so.
[[nodiscard]] double weatherRange(double range, double minRange, double maxRange,
                                  const SensorWeather& w, const LidarClutter& c, double u1,
                                  double u2, double u3);

/// The camera's veil, per pixel, in place: L = L0·exp(−σd) + L_air·(1 − exp(−σd)) with d the
/// depth (∞ for sky: all air light), then the dirty lens mixing in a grey haze of weight c/2.
/// `rgba` is 8-bit RGBA, `depth` metres per pixel.
void applyCameraWeather(std::span<std::uint8_t> rgba, std::span<const float> depth,
                        double extinction, double contamination);

/// The air light of the veil (overcast sky), 8-bit RGB.
inline constexpr std::uint8_t kAirLight[3]{190, 194, 200};

}  // namespace lagrange::sensors
