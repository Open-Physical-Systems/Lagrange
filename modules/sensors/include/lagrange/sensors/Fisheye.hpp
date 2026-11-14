#pragma once

// A fisheye camera (domains/sensors.md, "Fisheye cameras (as built)"): its image drawn from the
// faces of a cube around the camera, each a pinhole render, remapped through the Kannala–Brandt
// projection (lagrange/world/FisheyeLens.hpp). A cube covers every direction, so the lens can
// see 200° and more, which one pinhole render cannot (it stops short of 180°).

#include "lagrange/gpu/Raster.hpp"
#include "lagrange/world/FisheyeLens.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace lagrange::sensors {
inline namespace optics {

struct FisheyeConfig {
    std::uint32_t width{0}, height{0};  // the output image
    double fx{0}, fy{0}, cx{0}, cy{0};  // its intrinsics (OpenCV fisheye: u = fx x_d + cx)
    FisheyeLens lens;
    std::uint32_t maxFaceSize{2048};  // px, the largest render of one face
};

/// One face of the cube that some output pixel looks through: a pinhole render of the part of
/// the face those pixels need (pose and near plane left to the caller), and its frame.
struct FisheyeFace {
    gpu::PinholeCamera camera;
    std::array<double, 4> rotation{1, 0, 0, 0};  // optical frame from the face's frame (w x y z)
};

class FisheyeRemap {
public:
    explicit FisheyeRemap(const FisheyeConfig& config);

    /// The faces to render, in the order apply() takes their images.
    [[nodiscard]] const std::vector<FisheyeFace>& faces() const { return faces_; }

    /// The fisheye image from the faces' renders. Colour is interpolated (bilinear, in linear
    /// light: the renderer's γ = 2.2 is undone and done again); depth and instance ids are the
    /// nearest render pixel's. Depth is the range along each pixel's ray (m), since a fisheye's
    /// rays reach 90° and more off its axis where depth along the axis means nothing. Pixels past
    /// the lens's cutoff are black, at infinite range, with no instance.
    void apply(std::span<const gpu::RasterImages> rendered, gpu::RasterImages& out) const;

private:
    struct Pixel {
        std::uint8_t face{kNone};  // index into faces_
        float u{0}, v{0};          // in that face's render, pixels
        float range{0};            // range per unit of the render's depth: 1 / cos(ray, axis)
    };
    static constexpr std::uint8_t kNone = 0xFF;

    FisheyeConfig config_;
    std::vector<FisheyeFace> faces_;
    std::vector<Pixel> pixels_;
    std::array<float, 256> decode_{};  // display value → linear
};

}  // namespace optics
}  // namespace lagrange::sensors
