#include "lagrange/sensors/Fisheye.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tbb/parallel_for.h>

namespace lagrange::sensors {
inline namespace optics {

namespace {

constexpr double kDisplayGamma = 2.2;  // the raster shader's display encoding (mesh.slang)

using Vec3 = std::array<double, 3>;

/// A cube face's axes in the optical frame (z forward, x right, y down): x and y across the
/// face's image, z out through its centre. Right-handed, as the optical frame.
struct Axes {
    Vec3 x, y, z;
};
constexpr std::array<Axes, 6> kFaces{{
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},    // front
    {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}},   // right
    {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}},   // left
    {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},   // down
    {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}},   // up
    {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},  // back
}};

double dot(const Vec3& a, const Vec3& b) {
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

/// The quaternion (w x y z) of the rotation whose matrix has columns x, y, z (Shepperd's method).
std::array<double, 4> quaternionOf(const Axes& a) {
    const double m00 = a.x[0], m01 = a.y[0], m02 = a.z[0];
    const double m10 = a.x[1], m11 = a.y[1], m12 = a.z[1];
    const double m20 = a.x[2], m21 = a.y[2], m22 = a.z[2];
    const double trace = m00 + m11 + m22;
    std::array<double, 4> q{};
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = {s / 4.0, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s};
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q = {(m21 - m12) / s, s / 4.0, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q = {(m02 - m20) / s, (m01 + m10) / s, s / 4.0, (m12 + m21) / s};
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q = {(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4.0};
    }
    return q;
}

}  // namespace

FisheyeRemap::FisheyeRemap(const FisheyeConfig& config) : config_(config) {
    for (std::size_t i = 0; i < decode_.size(); ++i) {
        decode_[i] = static_cast<float>(std::pow(static_cast<double>(i) / 255.0, kDisplayGamma));
    }
    const std::uint32_t w = config.width;
    const std::uint32_t h = config.height;
    // Each pixel's ray, the face it looks through (the one most nearly along it) and where on
    // that face's plane z = 1 it lands.
    struct Hit {
        int face{-1};
        double a{0}, b{0}, cosine{1};
    };
    std::vector<Hit> hits(static_cast<std::size_t>(w) * h);
    struct Box {
        double a0{std::numeric_limits<double>::infinity()}, a1{-a0}, b0{a0}, b1{-a0};
        bool used{false};
    };
    std::array<Box, kFaces.size()> boxes{};
    for (std::uint32_t v = 0; v < h; ++v) {
        for (std::uint32_t u = 0; u < w; ++u) {
            const auto ray =
                config.lens.unproject((u - config.cx) / config.fx, (v - config.cy) / config.fy);
            if (!ray) {
                continue;
            }
            Hit& hit = hits[(static_cast<std::size_t>(v) * w) + u];
            double best = -2.0;
            for (std::size_t f = 0; f < kFaces.size(); ++f) {
                const double c = dot(*ray, kFaces[f].z);
                if (c > best) {
                    best = c;
                    hit.face = static_cast<int>(f);
                }
            }
            const Axes& axes = kFaces[static_cast<std::size_t>(hit.face)];
            hit.cosine = best;
            hit.a = dot(*ray, axes.x) / best;
            hit.b = dot(*ray, axes.y) / best;
            Box& box = boxes[static_cast<std::size_t>(hit.face)];
            box.used = true;
            box.a0 = std::min(box.a0, hit.a);
            box.a1 = std::max(box.a1, hit.a);
            box.b0 = std::min(box.b0, hit.b);
            box.b1 = std::max(box.b1, hit.b);
        }
    }
    // Every face's render: the output's focal length (the same angular pitch at the centre),
    // over the part of the face its pixels need and a pixel more for the interpolation.
    const double focal = std::min(std::max(config.fx, config.fy),
                                  (static_cast<double>(config.maxFaceSize) - 4.0) / 2.0);
    std::array<std::uint8_t, kFaces.size()> index{};
    index.fill(kNone);
    for (std::size_t f = 0; f < kFaces.size(); ++f) {
        const Box& box = boxes[f];
        if (!box.used) {
            continue;
        }
        const double u0 = std::floor(focal * box.a0) - 1.0;
        const double u1 = std::ceil(focal * box.a1) + 1.0;
        const double v0 = std::floor(focal * box.b0) - 1.0;
        const double v1 = std::ceil(focal * box.b1) + 1.0;
        FisheyeFace face;
        face.camera.width = static_cast<std::uint32_t>(u1 - u0 + 1.0);
        face.camera.height = static_cast<std::uint32_t>(v1 - v0 + 1.0);
        face.camera.fx = focal;
        face.camera.fy = focal;
        face.camera.cx = -u0;
        face.camera.cy = -v0;
        face.rotation = quaternionOf(kFaces[f]);
        index[f] = static_cast<std::uint8_t>(faces_.size());
        faces_.push_back(face);
    }
    pixels_.resize(hits.size());
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const Hit& hit = hits[i];
        if (hit.face < 0) {
            continue;
        }
        Pixel& p = pixels_[i];
        p.face = index[static_cast<std::size_t>(hit.face)];
        const gpu::PinholeCamera& camera = faces_[p.face].camera;
        p.u = static_cast<float>((focal * hit.a) + camera.cx);
        p.v = static_cast<float>((focal * hit.b) + camera.cy);
        p.range = static_cast<float>(1.0 / hit.cosine);
    }
}

void FisheyeRemap::apply(std::span<const gpu::RasterImages> rendered,
                         gpu::RasterImages& out) const {
    const std::uint32_t w = config_.width;
    const std::uint32_t h = config_.height;
    out.rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
    out.depth.assign(static_cast<std::size_t>(w) * h, std::numeric_limits<float>::infinity());
    out.instance.assign(static_cast<std::size_t>(w) * h, gpu::kNoInstance);
    const auto encode = [](double linear) {
        return static_cast<std::uint8_t>(
            std::lround(255.0 * std::pow(std::clamp(linear, 0.0, 1.0), 1.0 / kDisplayGamma)));
    };
    tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
        for (std::uint32_t u = 0; u < w; ++u) {
            const std::size_t i = (static_cast<std::size_t>(v) * w) + u;
            const Pixel& p = pixels_[i];
            if (p.face == kNone || p.face >= rendered.size()) {
                out.rgba[(i * 4) + 3] = 255;
                continue;
            }
            const gpu::RasterImages& image = rendered[p.face];
            const std::uint32_t rw = faces_[p.face].camera.width;
            const std::uint32_t rh = faces_[p.face].camera.height;
            if (image.depth.size() < static_cast<std::size_t>(rw) * rh) {
                out.rgba[(i * 4) + 3] = 255;
                continue;
            }
            const double su = std::clamp(static_cast<double>(p.u), 0.0, rw - 1.0);
            const double sv = std::clamp(static_cast<double>(p.v), 0.0, rh - 1.0);
            const auto u0 = std::min(static_cast<std::uint32_t>(su), rw - (rw > 1 ? 2 : 1));
            const auto v0 = std::min(static_cast<std::uint32_t>(sv), rh - (rh > 1 ? 2 : 1));
            const double fu = su - u0;
            const double fv = sv - v0;
            const auto at = [&](std::uint32_t uu, std::uint32_t vv, int c) {
                const std::size_t k =
                    (static_cast<std::size_t>(std::min(vv, rh - 1)) * rw) + std::min(uu, rw - 1);
                return static_cast<double>(decode_[image.rgba[(k * 4) + c]]);
            };
            for (int c = 0; c < 3; ++c) {
                const double linear =
                    ((1 - fu) * (1 - fv) * at(u0, v0, c)) + (fu * (1 - fv) * at(u0 + 1, v0, c)) +
                    ((1 - fu) * fv * at(u0, v0 + 1, c)) + (fu * fv * at(u0 + 1, v0 + 1, c));
                out.rgba[(i * 4) + c] = encode(linear);
            }
            const std::size_t nearest =
                (static_cast<std::size_t>(std::lround(sv)) * rw) + std::lround(su);
            out.depth[i] = image.depth[nearest] * p.range;
            out.instance[i] = image.instance[nearest];
            out.rgba[(i * 4) + 3] = image.rgba[(nearest * 4) + 3];
        }
    });
}

}  // namespace optics
}  // namespace lagrange::sensors
