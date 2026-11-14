#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace lagrange {

/// Beam geometry of a lidar from its World::Sensors settings, shared by the sensor module that
/// casts the rays and by everything that turns its ranges back into points.
///
/// Settings (radians, as in SDFormat <ray><scan>): samples, min_angle, max_angle (horizontal,
/// about the sensor's z); vertical_samples (default 1: a 2D scanner), vertical_min_angle,
/// vertical_max_angle (elevation above the sensor's xy plane, default 0).
///
/// The ranges array holds one ring after another: index = ring * horizontal + column, rings
/// from the lowest elevation up, columns from min_angle to max_angle.
///
/// Opt-in patterns (docs/design/domains/sensors.md, "Lidar scan patterns (as built)"); without
/// their settings every beam is where it always was:
/// - `vertical_angle_0`, `vertical_angle_1`, ...: a channel table (Velodyne style), one
///   elevation per ring in any order; the rings are these sorted from the lowest up, and
///   vertical_samples, vertical_min_angle and vertical_max_angle are ignored.
/// - `pattern` 1, flash: a solid-state focal-plane array, all beams fired at once. Its
///   columns and rings are evenly spaced on the image plane x = 1 (a pinhole's pixels): tan of
///   the angle, not the angle, is linear in the column; min/max angles are those of the outer
///   beams and must lie within ±π/2.
/// - `pattern` 2, rosette: a Livox-style non-repetitive scan from two Risley prisms (first
///   order: the deflections of the two wedges add as vectors, Marshall 1999). `samples` points
///   per scan, fired one after another at `rosette_point_rate`; the pattern goes on from scan
///   to scan, so its coverage grows with the integration time, and direction() takes the scan.
struct LidarLayout {
    enum class Pattern : std::uint8_t { Grid = 0, Flash = 1, Rosette = 2 };

    /// Two wedge prisms turning at speed1 and speed2 (rad/s; opposite signs: counter-rotating),
    /// deflecting the beam by deflection1 and deflection2 (rad).
    struct Rosette {
        double deflection1{0.0}, deflection2{0.0};
        double speed1{0.0}, speed2{0.0};
        double pointRate{0.0};  // points per second
    };

    // Livox Mid-40's circular field of view, 38.4°, and its point rate, 100 000 points/s (Livox
    // Mid Series User Manual v1.2, 2019.08, table 1.2.1 "Point cloud specifications").
    static constexpr double kRosetteFov = 0.670206;        // rad, 38.4°
    static constexpr double kRosettePointRate = 100000.0;  // 1/s
    // Livox does not publish the Mid-40's prism speeds. These are the prototype's in Livox's
    // paper on the design (Liu, Zhang and Hong, "Low-cost retina-like robotic lidars based on
    // incommensurable scanning", arXiv:2006.11034, appendix B: two identical prisms at 7294 rpm
    // and −4664 rpm): counter-rotating, a rosette that fills the disc in about a second.
    static constexpr double kRosetteSpeed1 = 2.0 * 3.14159265358979 * 7294.0 / 60.0;   // rad/s
    static constexpr double kRosetteSpeed2 = -2.0 * 3.14159265358979 * 4664.0 / 60.0;  // rad/s
    static constexpr double kRosetteSamples = 10000.0;  // points per scan: 100 000/s at 10 Hz

    std::uint32_t horizontal{1};
    std::uint32_t vertical{1};
    double minAngle{0.0};
    double maxAngle{0.0};
    double minElevation{0.0};
    double maxElevation{0.0};
    Pattern pattern{Pattern::Grid};
    std::vector<double> ringElevations;  // a channel table, lowest first; empty: even spacing
    Rosette rosette;

    using Settings = std::map<std::string, double, std::less<>>;

    [[nodiscard]] static LidarLayout from(const Settings& settings) {
        const auto get = [&settings](const char* key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        LidarLayout layout;
        const double pattern = get("pattern", 0.0);
        layout.pattern = pattern == 1.0   ? Pattern::Flash
                         : pattern == 2.0 ? Pattern::Rosette
                                          : Pattern::Grid;
        layout.horizontal = static_cast<std::uint32_t>(
            get("samples", layout.pattern == Pattern::Rosette ? kRosetteSamples : 360.0));
        layout.vertical = static_cast<std::uint32_t>(std::max(1.0, get("vertical_samples", 1.0)));
        layout.minAngle = get("min_angle", -3.14159265358979);
        layout.maxAngle = get("max_angle", 3.14159265358979);
        layout.minElevation = get("vertical_min_angle", 0.0);
        layout.maxElevation = get("vertical_max_angle", 0.0);
        for (std::uint32_t ring = 0;; ++ring) {
            const auto it = settings.find("vertical_angle_" + std::to_string(ring));
            if (it == settings.end()) {
                break;
            }
            layout.ringElevations.push_back(it->second);
        }
        if (!layout.ringElevations.empty()) {
            std::ranges::sort(layout.ringElevations);
            layout.vertical = static_cast<std::uint32_t>(layout.ringElevations.size());
            layout.minElevation = layout.ringElevations.front();
            layout.maxElevation = layout.ringElevations.back();
        }
        if (layout.pattern == Pattern::Rosette) {
            layout.vertical = 1;
            const double half = get("rosette_fov", kRosetteFov) / 2.0;
            const double ratio = std::max(get("rosette_prism_ratio", 1.0), 0.0);  // δ2 / δ1
            layout.rosette.deflection1 = half / (1.0 + ratio);
            layout.rosette.deflection2 = half * ratio / (1.0 + ratio);
            layout.rosette.speed1 = get("rosette_speed_1", kRosetteSpeed1);
            layout.rosette.speed2 = get("rosette_speed_2", kRosetteSpeed2);
            layout.rosette.pointRate = get("rosette_point_rate", kRosettePointRate);
        }
        return layout;
    }

    [[nodiscard]] std::uint32_t beams() const { return horizontal * vertical; }
    [[nodiscard]] bool is3d() const { return vertical > 1 || pattern != Pattern::Grid; }
    /// Whether the beams change from scan to scan (the rosette): then direction() needs the scan.
    [[nodiscard]] bool timeVarying() const { return pattern == Pattern::Rosette; }
    /// Whether all beams fire at once (a flash lidar: no motion distortion within a scan).
    [[nodiscard]] bool simultaneous() const { return pattern == Pattern::Flash; }

    [[nodiscard]] double azimuth(std::uint32_t column) const {
        if (pattern == Pattern::Flash && horizontal > 1) {
            return std::atan(flashSlope(minAngle, maxAngle, column, horizontal));
        }
        return horizontal > 1 ? minAngle + (maxAngle - minAngle) * column / (horizontal - 1)
                              : minAngle;
    }
    [[nodiscard]] double elevation(std::uint32_t ring) const {
        if (!ringElevations.empty()) {
            return ringElevations[std::min<std::size_t>(ring, ringElevations.size() - 1)];
        }
        if (pattern == Pattern::Flash && vertical > 1) {
            return std::atan(flashSlope(minElevation, maxElevation, ring, vertical));
        }
        return vertical > 1 ? minElevation + (maxElevation - minElevation) * ring / (vertical - 1)
                            : minElevation;
    }
    /// Unit direction of beam `index` in the sensor frame; `scan` counts the scans from the
    /// start (only the rosette's beams depend on it).
    [[nodiscard]] std::array<double, 3> direction(std::uint32_t index,
                                                  std::uint64_t scan = 0) const {
        if (pattern == Pattern::Rosette) {
            const std::uint64_t fired = (scan * horizontal) + index;
            return rosetteDirection(static_cast<double>(fired) / rosette.pointRate);
        }
        if (pattern == Pattern::Flash) {
            // On the image plane x = 1: a column's slope across, a ring's slope up.
            const double y = horizontal > 1
                                 ? flashSlope(minAngle, maxAngle, index % horizontal, horizontal)
                                 : std::tan(minAngle);
            const double z = !ringElevations.empty() ? std::tan(elevation(index / horizontal))
                             : vertical > 1          ? flashSlope(minElevation, maxElevation,
                                                                  index / horizontal, vertical)
                                                     : std::tan(minElevation);
            const double norm = std::sqrt(1.0 + (y * y) + (z * z));
            return {1.0 / norm, y / norm, z / norm};
        }
        const double a = azimuth(index % horizontal);
        const double e = elevation(index / horizontal);
        return {std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e)};
    }

    /// The rosette's beam `seconds` after its prisms' zero angle: the two deflections, each
    /// turning with its prism, add as vectors (first-order Risley prism theory: Marshall,
    /// "Risley prism scan patterns", Proc. SPIE 3787, 1999; Liu, Zhang and Hong, "Low-cost
    /// retina-like robotic lidars based on incommensurable scanning", IEEE/ASME Trans.
    /// Mechatronics 27(1), 2022, for Livox's use of it). The deflection's size is the angle off
    /// the sensor's x axis, its direction the way round it.
    [[nodiscard]] std::array<double, 3> rosetteDirection(double seconds) const {
        const double a1 = std::fmod(rosette.speed1 * seconds, 2.0 * 3.14159265358979);
        const double a2 = std::fmod(rosette.speed2 * seconds, 2.0 * 3.14159265358979);
        const double dy =
            (rosette.deflection1 * std::cos(a1)) + (rosette.deflection2 * std::cos(a2));
        const double dz =
            (rosette.deflection1 * std::sin(a1)) + (rosette.deflection2 * std::sin(a2));
        const double off = std::sqrt((dy * dy) + (dz * dz));
        const double s = off > 1e-12 ? std::sin(off) / off : 1.0;
        return {std::cos(off), dy * s, dz * s};
    }

private:
    /// The tangent of a flash lidar's beam `i` of `n`, evenly spaced on the image plane.
    static double flashSlope(double lo, double hi, std::uint32_t i, std::uint32_t n) {
        const double a = std::tan(lo);
        const double b = std::tan(hi);
        return a + ((b - a) * i / (n - 1));
    }
};

}  // namespace lagrange
