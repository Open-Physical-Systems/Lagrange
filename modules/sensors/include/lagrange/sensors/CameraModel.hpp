#pragma once

// The camera's sensor model (domains/sensors.md §2, the `sensor_model` level): lens distortion,
// vignetting, a rotational rolling shutter, exposure, photon shot and read noise and
// quantisation, applied on the CPU to the raster level's pinhole images. Each part takes effect
// when CameraSensorAPI_1 asks for it; without any, the raster images pass through unchanged.

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/random/Philox.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace lagrange::sensors {

/// A lens distortion model, as OpenCV and ROS camera_info write it.
struct Lens {
    enum class Model : std::uint8_t { None, PlumbBob, Equidistant };
    Model model{Model::None};
    std::array<double, 5> k{};  // plumb_bob: k1, k2, p1, p2, k3; equidistant: k1, k2, k3, k4
};

/// Normalised image coordinates of a ray (x/z, y/z) → their distorted coordinates (Brown–Conrady
/// radial and tangential terms for plumb_bob; θ_d = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸) for
/// equidistant).
[[nodiscard]] std::array<double, 2> distort(const Lens& lens, double x, double y);

/// The inverse of distort(), by fixed-point iteration (plumb_bob) or Newton's method on θ
/// (equidistant); nullopt where it does not converge or the ray points backwards.
[[nodiscard]] std::optional<std::array<double, 2>> undistort(const Lens& lens, double xd,
                                                             double yd);

/// The exposure and the pixel's electronics (the linear model of EMVA 1288).
struct Exposure {
    double time{0.0};          // s; with autoExposure, the longest exposure
    bool autoExposure{false};  // aim the mean signal at kAutoExposureTarget of full scale
    double electronRate{0.0};  // e⁻/s a pixel collects where the renderer draws linear 1
    double gain{1.0};          // analogue gain before the converter (full scale = full well)
    double readNoise{2.0};     // e⁻, standard deviation
    double fullWell{10000.0};  // e⁻
    int bitDepth{8};           // converter bits
};

/// The mean signal auto exposure aims at, as a fraction of full scale (18 % grey).
inline constexpr double kAutoExposureTarget = 0.18;

/// One pixel's digital number for a mean of `electrons` e⁻: Poisson shot noise (as a Gaussian of
/// variance `electrons`), Gaussian read noise, clipped at the full well, amplified by the gain and
/// quantised to the bit depth's full scale.
[[nodiscard]] std::uint32_t digitise(double electrons, const Exposure& exposure,
                                     random::Stream& rng);

struct CameraModelConfig {
    std::uint32_t width{0}, height{0};  // the output image
    double fx{0}, fy{0}, cx{0}, cy{0};  // its pinhole intrinsics (before distortion)
    Lens lens;
    double vignetting{0.0};          // fraction of the natural cos⁴θ fall-off applied, 0 to 1
    double rollingShutterTime{0.0};  // s from the first row's exposure to the last's; 0: global
    std::optional<Exposure> exposure;
};

/// Remaps a rendered pinhole view through the lens and the rolling shutter and turns its colour
/// into a sensor's output.
///
/// - The renderer draws a pinhole view with the output's pixel pitch, wider than the output when
///   a barrel lens or a rolling shutter looks past its edges (at most twice the size, below a
///   2.8 rad field of view); output pixels whose rays leave it stay black, at infinite depth.
/// - Rolling shutter: the last row is exposed at the frame's time, earlier rows earlier, and each
///   row sees the scene from the camera's orientation then (the angular velocity held over the
///   frame). Translation during the readout is left out (rotation dominates a moving camera's
///   rolling-shutter skew beyond a few metres).
/// - Colour: the renderer's display encoding (γ = 2.2) is undone; the linear value L is the
///   fraction of electronRate a pixel collects. Vignetting scales it by 1 − v(1 − cos⁴θ). With an
///   exposure, each channel gets e = L·electronRate·t electrons and digitise(); the result is
///   encoded for display again. Depth and instance ids follow the remap (nearest pixel), with
///   depth along the output's optical axis.
class CameraModel {
public:
    explicit CameraModel(const CameraModelConfig& config);

    /// Whether any part is on (otherwise apply() is not needed).
    [[nodiscard]] bool active() const { return active_; }
    /// The pinhole camera to render: size and intrinsics (pose and near plane left to the caller).
    [[nodiscard]] const gpu::PinholeCamera& renderCamera() const { return render_; }

    /// The sensor's images from the rendered ones; `omega` is the camera's angular velocity in its
    /// optical frame (rad/s); the noise draws one stream per row (entity `entity` + row).
    void apply(const gpu::RasterImages& rendered, const std::array<double, 3>& omega,
               std::uint64_t seed, std::uint64_t stepIndex, std::uint32_t entity,
               gpu::RasterImages& out);

    /// apply()'s per-pixel work for the GPU (gpu::RasterCamera::setEffects, the kernel
    /// gpu/kernels/camera/camera_model.slang): the same rays, vignetting and tables.
    [[nodiscard]] gpu::CameraEffects gpuEffects() const;
    /// This frame's part of it: as apply() takes `omega`, `seed`, `stepIndex` and `entity`.
    [[nodiscard]] gpu::CameraEffectsFrame gpuFrame(const std::array<double, 3>& omega,
                                                   std::uint64_t seed, std::uint64_t stepIndex,
                                                   std::uint32_t entity) const;
    /// After a frame on the GPU: auto exposure from its per-pixel `signal` (as apply() does).
    void afterGpuFrame(std::span<const float> signal);

    /// The exposure time of the next frame (s); 0 without an exposure.
    [[nodiscard]] double exposureTime() const { return exposureTime_; }
    void setExposureTime(double time) { exposureTime_ = time; }

private:
    void autoExpose(double signal, std::uint64_t count);
    CameraModelConfig config_;
    bool active_{false};
    gpu::PinholeCamera render_;
    std::vector<std::array<float, 3>> rays_;  // per output pixel, (x, y, 1) before distortion
    std::vector<float> vignette_;             // per output pixel
    std::array<float, 256> decode_{};         // display value → linear
    std::vector<std::uint8_t> encoded_;       // converter code → display value (with an exposure)
    double exposureTime_{0.0};
};

}  // namespace lagrange::sensors
