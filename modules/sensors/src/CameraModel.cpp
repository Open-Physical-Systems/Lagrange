#include "lagrange/sensors/CameraModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <tbb/parallel_for.h>

namespace lagrange::sensors {
namespace {

constexpr double kDisplayGamma = 2.2;         // the raster shader's display encoding (mesh.slang)
constexpr int kUndistortIterations = 50;      // fixed-point steps for plumb_bob
constexpr double kUndistortTolerance = 1e-9;  // normalised units, residual of distort()
constexpr double kMaxRenderHalfFov = 1.4;     // rad: the pinhole render stays below 2.8 rad
constexpr double kRollingMargin = 1.1;        // render 10 % wider for the rolling shutter's skew

using Mat3 = std::array<std::array<double, 3>, 3>;

/// The rotation by the rotation vector `a` (Rodrigues).
Mat3 rotationOf(const std::array<double, 3>& a) {
    const double angle = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (angle < 1e-12) {
        return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    }
    const double x = a[0] / angle;
    const double y = a[1] / angle;
    const double z = a[2] / angle;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;
    return {{{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
             {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
             {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
}

}  // namespace

std::array<double, 2> distort(const Lens& lens, double x, double y) {
    const auto& k = lens.k;
    switch (lens.model) {
        case Lens::Model::None:
            return {x, y};
        case Lens::Model::PlumbBob: {
            const double r2 = x * x + y * y;
            const double radial = 1.0 + r2 * (k[0] + r2 * (k[1] + r2 * k[4]));
            return {x * radial + 2.0 * k[2] * x * y + k[3] * (r2 + 2.0 * x * x),
                    y * radial + k[2] * (r2 + 2.0 * y * y) + 2.0 * k[3] * x * y};
        }
        case Lens::Model::Equidistant: {
            const double r = std::sqrt(x * x + y * y);
            if (r < 1e-12) {
                return {x, y};
            }
            const double theta = std::atan(r);
            const double t2 = theta * theta;
            const double thetaD =
                theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
            return {x * thetaD / r, y * thetaD / r};
        }
    }
    return {x, y};
}

std::optional<std::array<double, 2>> undistort(const Lens& lens, double xd, double yd) {
    const auto& k = lens.k;
    switch (lens.model) {
        case Lens::Model::None:
            return std::array<double, 2>{xd, yd};
        case Lens::Model::PlumbBob: {
            double x = xd;
            double y = yd;
            for (int i = 0; i < kUndistortIterations; ++i) {
                const double r2 = x * x + y * y;
                const double radial = 1.0 + r2 * (k[0] + r2 * (k[1] + r2 * k[4]));
                if (radial <= 0.0) {
                    return std::nullopt;
                }
                const double dx = 2.0 * k[2] * x * y + k[3] * (r2 + 2.0 * x * x);
                const double dy = k[2] * (r2 + 2.0 * y * y) + 2.0 * k[3] * x * y;
                x = (xd - dx) / radial;
                y = (yd - dy) / radial;
            }
            const auto back = distort(lens, x, y);
            if (std::hypot(back[0] - xd, back[1] - yd) > kUndistortTolerance) {
                return std::nullopt;
            }
            return std::array<double, 2>{x, y};
        }
        case Lens::Model::Equidistant: {
            const double thetaD = std::sqrt(xd * xd + yd * yd);
            if (thetaD < 1e-12) {
                return std::array<double, 2>{xd, yd};
            }
            double theta = thetaD;
            for (int i = 0; i < kUndistortIterations; ++i) {
                const double t2 = theta * theta;
                const double f =
                    theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - thetaD;
                const double df =
                    1.0 +
                    t2 * (3.0 * k[0] + t2 * (5.0 * k[1] + t2 * (7.0 * k[2] + t2 * 9.0 * k[3])));
                if (std::abs(df) < 1e-12) {
                    return std::nullopt;
                }
                theta -= f / df;
            }
            const double t2 = theta * theta;
            const double residual =
                theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - thetaD;
            if (theta <= 0.0 || theta >= std::numbers::pi / 2.0 ||
                std::abs(residual) > kUndistortTolerance) {
                return std::nullopt;
            }
            const double scale = std::tan(theta) / thetaD;
            return std::array<double, 2>{xd * scale, yd * scale};
        }
    }
    return std::nullopt;
}

std::uint32_t digitise(double electrons, const Exposure& exposure, random::Stream& rng) {
    const double mean = std::max(electrons, 0.0);
    double e = mean + std::sqrt(mean) * rng.normal() + exposure.readNoise * rng.normal();
    e = std::clamp(e, 0.0, exposure.fullWell);
    const double fullScale = std::ldexp(1.0, exposure.bitDepth) - 1.0;
    const double signal = std::min(e * exposure.gain / exposure.fullWell, 1.0);
    return static_cast<std::uint32_t>(std::lround(signal * fullScale));
}

CameraModel::CameraModel(const CameraModelConfig& config) : config_(config) {
    const std::uint32_t w = config.width;
    const std::uint32_t h = config.height;
    active_ = config.lens.model != Lens::Model::None || config.vignetting > 0.0 ||
              config.rollingShutterTime > 0.0 || config.exposure.has_value();
    exposureTime_ = config.exposure ? config.exposure->time : 0.0;
    render_.width = w;
    render_.height = h;
    render_.fx = config.fx;
    render_.fy = config.fy;
    render_.cx = config.cx;
    render_.cy = config.cy;
    for (std::size_t i = 0; i < decode_.size(); ++i) {
        decode_[i] = static_cast<float>(std::pow(static_cast<double>(i) / 255.0, kDisplayGamma));
    }
    if (!active_) {
        return;
    }
    // Each output pixel's ray before distortion, and how far the rays reach.
    rays_.resize(static_cast<std::size_t>(w) * h);
    vignette_.resize(rays_.size());
    // Rows in parallel (each pixel is its own; a full-HD lens took ~0.3 s on one core), the reach
    // of each row kept apart and taken in order after, so the result is the same on any cores.
    std::vector<double> rowReachX(h, 0.0);
    std::vector<double> rowReachY(h, 0.0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
        double& reachX = rowReachX[v];
        double& reachY = rowReachY[v];
        for (std::uint32_t u = 0; u < w; ++u) {
            const std::size_t i = static_cast<std::size_t>(v) * w + u;
            const auto ray =
                undistort(config.lens, (u - config.cx) / config.fx, (v - config.cy) / config.fy);
            if (!ray) {
                rays_[i] = {nan, nan, nan};
                vignette_[i] = 0.0F;
                continue;
            }
            rays_[i] = {static_cast<float>((*ray)[0]), static_cast<float>((*ray)[1]), 1.0F};
            reachX = std::max(reachX, std::abs((*ray)[0]));
            reachY = std::max(reachY, std::abs((*ray)[1]));
            const double cos2 = 1.0 / (1.0 + (*ray)[0] * (*ray)[0] + (*ray)[1] * (*ray)[1]);
            vignette_[i] = static_cast<float>(1.0 - config.vignetting * (1.0 - cos2 * cos2));
        }
    });
    double reachX = 0.0;
    double reachY = 0.0;
    for (std::uint32_t v = 0; v < h; ++v) {
        reachX = std::max(reachX, rowReachX[v]);
        reachY = std::max(reachY, rowReachY[v]);
    }
    // A render wide enough for those rays, with the output's pixel pitch.
    if (config.lens.model == Lens::Model::None && config.rollingShutterTime <= 0.0) {
        return;
    }
    const double margin = config.rollingShutterTime > 0.0 ? kRollingMargin : 1.0;
    const double limit = std::tan(kMaxRenderHalfFov);
    const auto span = [&](double reach, double f, std::uint32_t size) {
        const double wanted = 2.0 * std::ceil(std::min(reach * margin, limit) * f) + 2.0;
        return static_cast<std::uint32_t>(
            std::clamp(wanted, static_cast<double>(size), 2.0 * static_cast<double>(size)));
    };
    render_.width = span(reachX, config.fx, w);
    render_.height = span(reachY, config.fy, h);
    render_.cx = config.cx + (render_.width - static_cast<double>(w)) / 2.0;
    render_.cy = config.cy + (render_.height - static_cast<double>(h)) / 2.0;
}

void CameraModel::apply(const gpu::RasterImages& rendered, const std::array<double, 3>& omega,
                        std::uint64_t seed, std::uint64_t stepIndex, std::uint32_t entity,
                        gpu::RasterImages& out) {
    const std::uint32_t w = config_.width;
    const std::uint32_t h = config_.height;
    const std::uint32_t rw = render_.width;
    const std::uint32_t rh = render_.height;
    out.rgba.assign(static_cast<std::size_t>(w) * h * 4, 0);
    out.depth.assign(static_cast<std::size_t>(w) * h, std::numeric_limits<float>::infinity());
    out.instance.assign(static_cast<std::size_t>(w) * h, gpu::kNoInstance);
    const std::optional<Exposure>& exposure = config_.exposure;
    const double fullScale = exposure ? std::ldexp(1.0, exposure->bitDepth) - 1.0 : 255.0;
    const double electronsPerUnit =
        exposure ? exposure->electronRate * exposureTime_ : 0.0;  // e⁻ at linear 1
    std::vector<double> rowSignal(h, 0.0);
    std::vector<std::uint32_t> rowCount(h, 0);
    const auto encode = [](double linear) {
        return static_cast<std::uint8_t>(
            std::lround(255.0 * std::pow(std::clamp(linear, 0.0, 1.0), 1.0 / kDisplayGamma)));
    };
    // With an exposure every channel is one of the converter's codes: encode them once (the same
    // numbers encode() gives, without a pow per channel).
    if (exposure && encoded_.size() != static_cast<std::size_t>(fullScale) + 1) {
        encoded_.resize(static_cast<std::size_t>(fullScale) + 1);
        for (std::size_t k = 0; k < encoded_.size(); ++k) {
            encoded_[k] = encode(static_cast<double>(static_cast<std::uint32_t>(k)) / fullScale);
        }
    }
    tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
        const double tau = config_.rollingShutterTime > 0.0 && h > 1
                               ? config_.rollingShutterTime * (h - 1 - v) / (h - 1)
                               : 0.0;
        // The row's camera, τ before the frame: its rays in the frame's optical frame.
        const Mat3 r = rotationOf({-omega[0] * tau, -omega[1] * tau, -omega[2] * tau});
        random::Stream rng(seed, stepIndex, entity + v);
        for (std::uint32_t u = 0; u < w; ++u) {
            const std::size_t i = static_cast<std::size_t>(v) * w + u;
            const std::array<float, 3>& ray = rays_[i];
            std::array<double, 3> linear{0.0, 0.0, 0.0};
            bool seen = false;
            if (!std::isnan(ray[0])) {
                const double dx = r[0][0] * ray[0] + r[0][1] * ray[1] + r[0][2] * ray[2];
                const double dy = r[1][0] * ray[0] + r[1][1] * ray[1] + r[1][2] * ray[2];
                const double dz = r[2][0] * ray[0] + r[2][1] * ray[1] + r[2][2] * ray[2];
                double su = dz > 1e-9 ? render_.fx * dx / dz + render_.cx : -1.0;
                double sv = dz > 1e-9 ? render_.fy * dy / dz + render_.cy : -1.0;
                // Within the render's pixels (their footprints reach half a pixel past the
                // outer centres).
                if (su >= -0.5 && sv >= -0.5 && su <= rw - 0.5 && sv <= rh - 0.5) {
                    seen = true;
                    su = std::clamp(su, 0.0, rw - 1.0);
                    sv = std::clamp(sv, 0.0, rh - 1.0);
                    const auto u0 = std::min(static_cast<std::uint32_t>(su), rw - 2 * (rw > 1));
                    const auto v0 = std::min(static_cast<std::uint32_t>(sv), rh - 2 * (rh > 1));
                    const double fu = su - u0;
                    const double fv = sv - v0;
                    const auto at = [&](std::uint32_t uu, std::uint32_t vv, int c) {
                        return static_cast<double>(
                            decode_[rendered
                                        .rgba[(static_cast<std::size_t>(std::min(vv, rh - 1)) * rw +
                                               std::min(uu, rw - 1)) *
                                                  4 +
                                              c]]);
                    };
                    for (int c = 0; c < 3; ++c) {
                        linear[c] = (1 - fu) * (1 - fv) * at(u0, v0, c) +
                                    fu * (1 - fv) * at(u0 + 1, v0, c) +
                                    (1 - fu) * fv * at(u0, v0 + 1, c) +
                                    fu * fv * at(u0 + 1, v0 + 1, c);
                    }
                    const std::size_t nearest =
                        static_cast<std::size_t>(std::lround(sv)) * rw + std::lround(su);
                    out.depth[i] = static_cast<float>(rendered.depth[nearest] / dz);
                    out.instance[i] = rendered.instance[nearest];
                    out.rgba[i * 4 + 3] = rendered.rgba[nearest * 4 + 3];
                }
            }
            if (!seen) {
                out.rgba[i * 4 + 3] = 255;
            }
            for (int c = 0; c < 3; ++c) {
                const double l = linear[c] * vignette_[i];
                if (exposure) {
                    const double electrons = l * electronsPerUnit;
                    if (seen) {
                        rowSignal[v] +=
                            std::min(electrons * exposure->gain / exposure->fullWell, 1.0);
                    }
                    out.rgba[i * 4 + c] = encoded_[digitise(electrons, *exposure, rng)];
                } else {
                    out.rgba[i * 4 + c] = encode(l);
                }
            }
            rowCount[v] += seen ? 3 : 0;
        }
    });
    // Auto exposure: the next frame's time, scaled to bring the mean signal to the target.
    if (exposure && exposure->autoExposure) {
        double signal = 0.0;
        std::uint64_t count = 0;
        for (std::uint32_t v = 0; v < h; ++v) {
            signal += rowSignal[v];
            count += rowCount[v];
        }
        autoExpose(signal, count);
    }
}

void CameraModel::autoExpose(double signal, std::uint64_t count) {
    const double mean = count > 0 ? signal / static_cast<double>(count) : 0.0;
    constexpr double kShortestExposure = 1e-6;  // s
    exposureTime_ = std::clamp(exposureTime_ * kAutoExposureTarget / std::max(mean, 1e-4),
                               kShortestExposure, config_.exposure->time);
}

gpu::CameraEffects CameraModel::gpuEffects() const {
    gpu::CameraEffects out;
    out.width = config_.width;
    out.height = config_.height;
    out.fx = render_.fx;
    out.fy = render_.fy;
    out.cx = render_.cx;
    out.cy = render_.cy;
    const std::size_t pixels = static_cast<std::size_t>(config_.width) * config_.height;
    out.rays.resize(pixels);
    for (std::size_t i = 0; i < pixels; ++i) {
        // Without any part on, apply() is not used; the rays are then the pinhole's own.
        if (rays_.empty()) {
            const auto u = static_cast<double>(i % config_.width);
            const auto v = static_cast<double>(i / config_.width);
            out.rays[i] = {static_cast<float>((u - config_.cx) / config_.fx),
                           static_cast<float>((v - config_.cy) / config_.fy), 1.0F, 0.0F};
        } else {
            out.rays[i] = {rays_[i][0], rays_[i][1], vignette_[i], 0.0F};
        }
    }
    out.decode = decode_;
    if (config_.exposure) {
        const double fullScale = std::ldexp(1.0, config_.exposure->bitDepth) - 1.0;
        out.encoded.resize(static_cast<std::size_t>(fullScale) + 1);
        for (std::size_t k = 0; k < out.encoded.size(); ++k) {
            out.encoded[k] = static_cast<std::uint8_t>(std::lround(
                255.0 * std::pow(static_cast<double>(static_cast<std::uint32_t>(k)) / fullScale,
                                 1.0 / kDisplayGamma)));
        }
    }
    return out;
}

gpu::CameraEffectsFrame CameraModel::gpuFrame(const std::array<double, 3>& omega,
                                              std::uint64_t seed, std::uint64_t stepIndex,
                                              std::uint32_t entity) const {
    gpu::CameraEffectsFrame frame;
    frame.omega = omega;
    frame.rollingShutterTime = config_.rollingShutterTime;
    if (config_.exposure) {
        frame.electronsPerUnit = config_.exposure->electronRate * exposureTime_;
        frame.gain = config_.exposure->gain;
        frame.fullWell = config_.exposure->fullWell;
        frame.readNoise = config_.exposure->readNoise;
    }
    frame.seed = seed;
    frame.stepIndex = stepIndex;
    frame.entity = entity;
    return frame;
}

void CameraModel::afterGpuFrame(std::span<const float> signal) {
    if (!config_.exposure || !config_.exposure->autoExposure) {
        return;
    }
    double sum = 0.0;
    std::uint64_t count = 0;
    for (const float s : signal) {
        if (s >= 0.0F) {
            sum += s;
            count += 3;
        }
    }
    autoExpose(sum, count);
}

}  // namespace lagrange::sensors
