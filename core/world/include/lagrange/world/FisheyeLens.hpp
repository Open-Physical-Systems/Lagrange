#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <string>

namespace lagrange {

/// A fisheye camera's projection from its World::Sensors settings, shared by the camera module
/// that draws it and by everything that publishes its calibration (docs/design/domains/sensors.md,
/// "Fisheye cameras (as built)").
///
/// The Kannala–Brandt model as OpenCV's fisheye module and ROS camera_info ("equidistant", D =
/// k1..k4) write it: a ray at angle θ off the optical axis lands at distance
/// θ_d = θ (1 + k1 θ² + k2 θ⁴ + k3 θ⁶ + k4 θ⁸) from the principal point, in units of the focal
/// length, in the ray's own direction round the axis (Kannala and Brandt, "A generic camera model
/// and calibration method for conventional, wide-angle, and fish-eye lenses", IEEE TPAMI 28(8),
/// 2006). All k = 0 is the ideal equidistant fisheye, r = f θ. Unlike a pinhole it covers rays at
/// and beyond 90° off the axis.
///
/// Settings: `projection` 1 (opt in; the camera is a pinhole otherwise), `horizontal_fov` (rad,
/// the angle across the image's width: f = (width / 2) / θ_d(fov / 2), as Gazebo's wide-angle
/// camera with scale_to_hfov), `distortion_k1`..`distortion_k4`, `fisheye_cutoff` (rad, the
/// half-angle beyond which the lens sees nothing: default the larger of fov / 2 and π / 2, as
/// Gazebo's cutoff_angle; at most π).
struct FisheyeLens {
    static constexpr double kPi = 3.14159265358979323846;

    std::array<double, 4> k{};  // k1..k4
    double cutoff{kPi / 2.0};   // rad, half-angle

    using Settings = std::map<std::string, double, std::less<>>;

    [[nodiscard]] static bool isFisheye(const Settings& settings) {
        const auto it = settings.find("projection");
        return it != settings.end() && it->second == 1.0;
    }

    [[nodiscard]] static FisheyeLens from(const Settings& settings) {
        const auto get = [&settings](const std::string& key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        FisheyeLens lens;
        for (std::size_t i = 0; i < lens.k.size(); ++i) {
            lens.k.at(i) = get("distortion_k" + std::to_string(i + 1), 0.0);
        }
        const double fov = get("horizontal_fov", 1.047);
        lens.cutoff = std::min(get("fisheye_cutoff", std::max(fov / 2.0, kPi / 2.0)), kPi);
        return lens;
    }

    /// θ_d of a ray θ off the axis.
    [[nodiscard]] double distorted(double theta) const {
        const double t2 = theta * theta;
        return theta * (1.0 + (t2 * (k[0] + (t2 * (k[1] + (t2 * (k[2] + (t2 * k[3]))))))));
    }
    /// dθ_d/dθ.
    [[nodiscard]] double slope(double theta) const {
        const double t2 = theta * theta;
        return 1.0 + (t2 * ((3.0 * k[0]) +
                            (t2 * ((5.0 * k[1]) + (t2 * ((7.0 * k[2]) + (t2 * 9.0 * k[3])))))));
    }
    /// Whether θ_d grows with θ all the way to the cutoff (a lens that folds back is refused).
    [[nodiscard]] bool monotonic() const {
        constexpr int kChecks = 1000;
        for (int i = 0; i <= kChecks; ++i) {
            if (!(slope(cutoff * i / kChecks) > 0.0)) {
                return false;
            }
        }
        return true;
    }
    /// θ of a distorted radius θ_d, by Newton's method from θ_d; nullopt past the cutoff.
    [[nodiscard]] std::optional<double> undistorted(double thetaD) const {
        constexpr int kIterations = 50;
        constexpr double kTolerance = 1e-12;
        double theta = std::min(thetaD, cutoff);
        for (int i = 0; i < kIterations; ++i) {
            const double step = (distorted(theta) - thetaD) / slope(theta);
            theta -= step;
            if (std::abs(step) < kTolerance) {
                break;
            }
        }
        if (!(theta >= 0.0) || theta > cutoff + 1e-9 ||
            std::abs(distorted(theta) - thetaD) > 1e-9) {
            return std::nullopt;
        }
        return theta;
    }

    /// The focal length (px) that spans `fov` across `width` pixels.
    [[nodiscard]] double focal(double width, double fov) const {
        return width / 2.0 / distorted(fov / 2.0);
    }

    /// Normalised image coordinates (x, y: (u - cx) / fx, (v - cy) / fy) of a direction in the
    /// optical frame (z forward, x right, y down); nullopt past the cutoff.
    [[nodiscard]] std::optional<std::array<double, 2>> project(
        const std::array<double, 3>& d) const {
        const double r = std::hypot(d[0], d[1]);
        const double theta = std::atan2(r, d[2]);
        if (theta > cutoff) {
            return std::nullopt;
        }
        if (r < 1e-15) {
            return std::array<double, 2>{0.0, 0.0};
        }
        const double scale = distorted(theta) / r;
        return std::array<double, 2>{d[0] * scale, d[1] * scale};
    }
    /// The unit direction (optical frame) of normalised image coordinates; nullopt past the
    /// cutoff.
    [[nodiscard]] std::optional<std::array<double, 3>> unproject(double x, double y) const {
        const double thetaD = std::hypot(x, y);
        const std::optional<double> theta = undistorted(thetaD);
        if (!theta) {
            return std::nullopt;
        }
        if (thetaD < 1e-15) {
            return std::array<double, 3>{0.0, 0.0, 1.0};
        }
        const double s = std::sin(*theta) / thetaD;
        return std::array<double, 3>{x * s, y * s, std::cos(*theta)};
    }
};

}  // namespace lagrange
