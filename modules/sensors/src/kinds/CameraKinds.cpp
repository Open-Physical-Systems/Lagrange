// The models behind the camera-like built-in kinds (CameraKinds.hpp).
#include "CameraKinds.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <tbb/parallel_for.h>

namespace lagrange::sensors::kinds {
inline namespace camera_kinds {

std::array<double, 2> flowRate(const Vec3& v, const Vec3& w, double x, double y, double depth) {
    const double xDot = (-v[0] + x * v[2]) / depth + x * y * w[0] - (1.0 + x * x) * w[1] + y * w[2];
    const double yDot = (-v[1] + y * v[2]) / depth + (1.0 + y * y) * w[0] - x * y * w[1] - x * w[2];
    return {yDot, -xDot};
}

// --- event_camera ---------------------------------------------------------------------------

namespace {
// Smallest threshold a mismatched pixel may have (ln units), as v2e clips its draws.
constexpr double kMinThreshold = 0.01;
// The stream index of the per-pixel threshold draws: a step no run reaches, so the draws are
// fixed for the run and never repeat a noise draw.
constexpr std::uint64_t kThresholdStep = 0xFFFF'FFFF'FFFF'FFFFULL;
}  // namespace

float linLog(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    // sRGB decode (IEC 61966-2-1), then Rec. 709 luminance, in digital numbers.
    const auto decode = [](std::uint8_t c) {
        const double s = c / 255.0;
        return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4);
    };
    const double y = 255.0 * (0.2126 * decode(r) + 0.7152 * decode(g) + 0.0722 * decode(b));
    if (y >= kLinLogDn) {
        return static_cast<float>(std::log(y));
    }
    return static_cast<float>(y * std::log(kLinLogDn) / kLinLogDn);
}

EventGenerator::EventGenerator(const EventConfig& config) : config_(config) {
    const std::size_t n = static_cast<std::size_t>(config.width) * config.height;
    level_.assign(n, 0.0F);
    last_.assign(n, 0.0F);
    lastEvent_.assign(n, -std::numeric_limits<double>::infinity());
    on_.assign(n, static_cast<float>(config.thresholdOn));
    off_.assign(n, static_cast<float>(config.thresholdOff));
}

void EventGenerator::drawThresholds(std::uint64_t seed) {
    if (config_.thresholdSigma <= 0.0) {
        return;
    }
    for (std::uint32_t row = 0; row < config_.height; ++row) {
        random::Stream rng(seed, kThresholdStep, row);
        for (std::uint32_t col = 0; col < config_.width; ++col) {
            const std::size_t i = static_cast<std::size_t>(row) * config_.width + col;
            on_[i] = static_cast<float>(std::max(
                kMinThreshold, config_.thresholdOn + config_.thresholdSigma * rng.normal()));
            off_[i] = static_cast<float>(std::max(
                kMinThreshold, config_.thresholdOff + config_.thresholdSigma * rng.normal()));
        }
    }
}

void EventGenerator::feed(std::span<const float> frame, double t, std::uint64_t seed,
                          std::uint64_t stepIndex, std::vector<DvsEvent>& out) {
    const std::size_t n = level_.size();
    if (frame.size() != n) {
        return;
    }
    if (!started_) {
        drawThresholds(seed);
        std::copy(frame.begin(), frame.end(), level_.begin());
        std::copy(frame.begin(), frame.end(), last_.begin());
        lastT_ = t;
        started_ = true;
        return;
    }
    const double t0 = lastT_;
    const double dt = t - t0;
    if (dt <= 0.0) {
        return;
    }
    const std::size_t first = out.size();
    const std::uint32_t w = config_.width;
    // Rows in parallel into rows of their own, joined in row order (the result does not depend
    // on the threads).
    std::vector<std::vector<DvsEvent>> rows(config_.height);
    tbb::parallel_for(std::uint32_t{0}, config_.height, [&](std::uint32_t row) {
        std::vector<DvsEvent>& mine = rows[row];
        random::Stream rng(seed, stepIndex, row);
        const double pNoise = config_.noiseRate * dt;
        for (std::uint32_t col = 0; col < w; ++col) {
            const std::size_t i = static_cast<std::size_t>(row) * w + col;
            const double l0 = last_[i];
            const double l1 = frame[i];
            const double slope = (l1 - l0) / dt;
            double level = level_[i];
            const int sign = l1 > level ? 1 : -1;
            const double c = sign > 0 ? on_[i] : off_[i];
            while (sign > 0 ? level + c <= l1 : level - c >= l1) {
                const double next = level + sign * c;
                // When the line crossed the level (it may have before this frame, if a
                // refractory hold carried it over: then at the frame's start).
                const bool between = sign > 0 ? next > l0 : next < l0;
                const double tc = between ? t0 + std::clamp((next - l0) / slope, 0.0, dt) : t0;
                if (tc - lastEvent_[i] < config_.refractory) {
                    break;  // held: the change carries over
                }
                mine.push_back({col, row, tc, sign});
                lastEvent_[i] = tc;
                level = next;
            }
            level_[i] = static_cast<float>(level);
            // Draw both numbers for every pixel, so each pixel's noise does not depend on the
            // others'.
            const double u = rng.uniform();
            const double when = rng.uniform();
            if (u < pNoise) {
                mine.push_back({col, row, t0 + when * dt, u < pNoise / 2.0 ? 1 : -1});
            }
        }
    });
    for (std::vector<DvsEvent>& row : rows) {
        out.insert(out.end(), row.begin(), row.end());
    }
    std::stable_sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(),
                     [](const DvsEvent& a, const DvsEvent& b) { return a.t < b.t; });
    std::copy(frame.begin(), frame.end(), last_.begin());
    lastT_ = t;
}

void EventGenerator::save(StateWriter& writer) const {
    writer.put(started_);
    writer.put(lastT_);
    writer.putBytes(level_.data(), level_.size() * sizeof(float));
    writer.putBytes(last_.data(), last_.size() * sizeof(float));
    writer.putBytes(lastEvent_.data(), lastEvent_.size() * sizeof(double));
    writer.putBytes(on_.data(), on_.size() * sizeof(float));
    writer.putBytes(off_.data(), off_.size() * sizeof(float));
}

bool EventGenerator::restore(StateReader& reader) {
    return reader.get(started_) && reader.get(lastT_) &&
           reader.getBytes(level_.data(), level_.size() * sizeof(float)) &&
           reader.getBytes(last_.data(), last_.size() * sizeof(float)) &&
           reader.getBytes(lastEvent_.data(), lastEvent_.size() * sizeof(double)) &&
           reader.getBytes(on_.data(), on_.size() * sizeof(float)) &&
           reader.getBytes(off_.data(), off_.size() * sizeof(float));
}

// --- stereo_camera --------------------------------------------------------------------------

void stereoDepth(const StereoConfig& c, std::uint32_t width, std::uint32_t height,
                 std::span<const float> left, std::span<const float> right, bool noise,
                 std::uint64_t seed, std::uint64_t stepIndex, std::vector<float>& out) {
    constexpr float kInvalid = std::numeric_limits<float>::quiet_NaN();
    constexpr float kFar = std::numeric_limits<float>::infinity();
    const std::size_t n = static_cast<std::size_t>(width) * height;
    out.assign(n, kInvalid);
    if (left.size() != n || right.size() != n) {
        return;
    }
    const double minZ = c.fx * c.baseline / c.maxDisparity;
    tbb::parallel_for(std::uint32_t{0}, height, [&](std::uint32_t v) {
        random::Stream rng(seed, stepIndex, v);
        for (std::uint32_t u = 0; u < width; ++u) {
            const std::size_t i = static_cast<std::size_t>(v) * width + u;
            const double gauss = noise ? rng.normal() : 0.0;  // one draw per pixel, always
            const double z = left[i];
            if (!std::isfinite(z) || z > c.maxRange) {
                out[i] = kFar;
                continue;
            }
            if (z < minZ) {
                out[i] = -kFar;
                continue;
            }
            // The point in the right camera: the same depth, shifted by the disparity f b / z.
            const double disparity = c.fx * c.baseline / z;
            const double ur = u - disparity;
            const long column = std::lround(ur);
            if (column < 0 || column >= static_cast<long>(width)) {
                continue;  // the right camera does not see it: the left band
            }
            const double seen =
                right[static_cast<std::size_t>(v) * width + static_cast<std::size_t>(column)];
            if (seen < z * (1.0 - c.occlusionTolerance)) {
                continue;  // hidden from the right camera: an occlusion shadow
            }
            double measured = z + gauss * stereoDepthSigma(z, c.subpixel, c.fx, c.baseline);
            if (c.depthUnit > 0.0) {
                measured = std::round(measured / c.depthUnit) * c.depthUnit;
            }
            out[i] = static_cast<float>(measured);
        }
    });
}

// --- rendering ------------------------------------------------------------------------------

gpu::PinholeCamera pinhole(std::uint32_t width, std::uint32_t height, double fov, double nearClip) {
    gpu::PinholeCamera out;
    out.width = width;
    out.height = height;
    out.fx = width / 2.0 / std::tan(fov / 2.0);
    out.fy = out.fx;
    out.cx = (width - 1.0) / 2.0;
    out.cy = (height - 1.0) / 2.0;
    out.near = nearClip;
    return out;
}

void SceneRender::declare(const World& raw, std::uint32_t sensorBody, SignalDeclarer& d) {
    const std::vector<bool> moving = movingBodies(raw);
    rest_ = restPoses(raw);
    poses_.assign(moving.size(), SignalId{});
    const auto watch = [&](std::uint32_t b) {
        if (b < moving.size() && moving[b] && !poses_[b].valid()) {
            poses_[b] = d.input(kindSpec("body/" + raw.bodies.names[b] + "/pose", SignalType::Pose,
                                         units::m, "world_local"));
        }
    };
    watch(sensorBody);
    hinges_.clear();
    if (!scene_) {
        return;
    }
    for (const CameraScene::Drawable& drawable : scene_->drawables) {
        if (drawable.body != kNoIndex) {
            watch(drawable.body);
        }
    }
    for (const CameraScene::Drawable& drawable : scene_->drawables) {
        SignalId id;
        if (!drawable.hinge.empty()) {
            SignalSpec angle = kindSpec(drawable.hinge, SignalType::Scalar, units::rad);
            angle.feedback = true;
            angle.optional = true;
            id = d.input(angle);
        }
        hinges_.push_back(id);
    }
}

bool SceneRender::init(std::uint32_t width, std::uint32_t height) {
    if (!scene_ || !scene_->raster) {
        return false;
    }
    renderer_ = std::make_unique<gpu::RasterCamera>(scene_->raster, width, height);
    instances_.resize(scene_->drawables.size());
    return true;
}

Pose SceneRender::bodyPose(const SignalBus& bus, std::uint32_t b) const {
    if (b >= poses_.size() || !poses_[b].valid()) {
        return b < rest_.size() ? rest_[b] : Pose{};
    }
    const std::span<const double> v = bus.readN(poses_[b]);
    return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
}

void SceneRender::render(const SignalBus& bus, const gpu::PinholeCamera& camera,
                         gpu::RasterImages& out, double seconds) {
    for (std::size_t k = 0; k < instances_.size(); ++k) {
        const CameraScene::Drawable& drawable = scene_->drawables[k];
        Pose pose = drawable.body == kNoIndex
                        ? drawable.offset
                        : compose(bodyPose(bus, drawable.body), drawable.offset);
        if (k < hinges_.size() && hinges_[k].valid()) {
            const double half = bus.read(hinges_[k]) / 2.0;
            const std::array<double, 3>& a = drawable.hingeAxis;
            pose = compose(pose, Pose{{0.0, 0.0, 0.0},
                                      {std::cos(half), std::sin(half) * a[0], std::sin(half) * a[1],
                                       std::sin(half) * a[2]}});
        }
        gpu::RasterInstance& instance = instances_[k];
        instance.mesh = drawable.mesh;
        instance.position = pose.position;
        instance.rotation = pose.orientation;
        instance.scale = drawable.scale;
        instance.color = drawable.color;
        instance.id = drawable.prim;
        instance.metallic = drawable.metallic;
        instance.roughness = drawable.roughness;
        instance.pattern = drawable.pattern;
        instance.emissive = drawable.emissive;
        instance.ground = drawable.ground;
        instance.water = drawable.water;
        instance.box = drawable.box;
    }
    renderer_->render(instances_, camera, out, seconds);
}

}  // namespace camera_kinds
}  // namespace lagrange::sensors::kinds
