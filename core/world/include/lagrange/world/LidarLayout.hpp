#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>

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
struct LidarLayout {
    std::uint32_t horizontal{1};
    std::uint32_t vertical{1};
    double minAngle{0.0};
    double maxAngle{0.0};
    double minElevation{0.0};
    double maxElevation{0.0};

    using Settings = std::map<std::string, double, std::less<>>;

    [[nodiscard]] static LidarLayout from(const Settings& settings) {
        const auto get = [&settings](const char* key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        LidarLayout layout;
        layout.horizontal = static_cast<std::uint32_t>(get("samples", 360.0));
        layout.vertical = static_cast<std::uint32_t>(std::max(1.0, get("vertical_samples", 1.0)));
        layout.minAngle = get("min_angle", -3.14159265358979);
        layout.maxAngle = get("max_angle", 3.14159265358979);
        layout.minElevation = get("vertical_min_angle", 0.0);
        layout.maxElevation = get("vertical_max_angle", 0.0);
        return layout;
    }

    [[nodiscard]] std::uint32_t beams() const { return horizontal * vertical; }
    [[nodiscard]] bool is3d() const { return vertical > 1; }

    [[nodiscard]] double azimuth(std::uint32_t column) const {
        return horizontal > 1 ? minAngle + (maxAngle - minAngle) * column / (horizontal - 1)
                              : minAngle;
    }
    [[nodiscard]] double elevation(std::uint32_t ring) const {
        return vertical > 1 ? minElevation + (maxElevation - minElevation) * ring / (vertical - 1)
                            : minElevation;
    }
    /// Unit direction of beam `index` in the sensor frame.
    [[nodiscard]] std::array<double, 3> direction(std::uint32_t index) const {
        const double a = azimuth(index % horizontal);
        const double e = elevation(index / horizontal);
        return {std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e)};
    }
};

}  // namespace lagrange
