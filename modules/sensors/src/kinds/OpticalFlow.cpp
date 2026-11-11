// optical_flow (sensors.md §9): a downward optical flow sensor (PMW3901, PX4Flow) from its own
// motion and the distance to what it looks at, by geometry rather than image matching.
#include "CameraKinds.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

// PixArt PMW3901MB-TXQT product datasheet (v1.x): field of view 42°, working range from 80 mm,
// maximum speed 7.4 rad/s.
constexpr double kPmw3901FovRad = 42.0 * 3.14159265358979323846 / 180.0;
constexpr double kPmw3901MinRangeM = 0.08;
constexpr double kPmw3901MaxFlowRadS = 7.4;
// Illustrative, not from a datasheet (TODO(verify) against a logged PMW3901): white noise on the
// flow rate, rad/s (1σ), and the furthest ground the default sensor tracks, m.
constexpr double kDefaultFlowNoiseRadS = 0.05;
constexpr double kDefaultMaxRangeM = 30.0;
// Quality: PX4's 0..255 scale (SensorOpticalFlow.quality).
constexpr double kFullQuality = 255.0;
// Rays across the field of view, per side: a 3 × 3 grid, the centre one on the optical axis.
constexpr int kRaysPerSide = 3;

class OpticalFlow final : public KindModule {
public:
    OpticalFlow(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {
        step_ = wholeMillisecondPeriod(world.sensors.rateHz[sensor]);
    }

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = view.raw();
        fov_ = kindSetting(raw, sensor_, "fov", kPmw3901FovRad);
        minRange_ = kindSetting(raw, sensor_, "min_range", kPmw3901MinRangeM);
        maxRange_ = kindSetting(raw, sensor_, "max_range", kDefaultMaxRangeM);
        maxFlow_ = kindSetting(raw, sensor_, "max_flow_rate", kPmw3901MaxFlowRadS);
        noise_ = kindSetting(raw, sensor_, "noise", kDefaultFlowNoiseRadS);
        if (fov_ <= 0.0 || fov_ >= 3.0 || minRange_ < 0.0 || maxRange_ <= minRange_ ||
            maxFlow_ <= 0.0 || noise_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "optical_flow " + name_ +
                                     ": fov, min_range, max_range, max_flow_rate or noise out "
                                     "of range");
        }
        const std::vector<bool> moving = movingBodies(raw);
        rest_ = restPoses(raw);
        poses_.assign(moving.size(), SignalId{});
        const auto watch = [&](std::uint32_t b) {
            if (b < moving.size() && moving[b] && !poses_[b].valid()) {
                poses_[b] = d.input(kindSpec("body/" + raw.bodies.names[b] + "/pose",
                                             SignalType::Pose, units::m, "world_local"));
            }
        };
        watch(body_);
        if (body_ < moving.size() && moving[body_]) {
            twist_ = d.input(kindSpec("body/" + raw.bodies.names[body_] + "/twist",
                                      SignalType::Wrench, units::m_per_s, "world_local"));
        }
        for (const BodyId owner : raw.colliders.body) {
            if (owner != kWorldBody && owner.index != body_) {
                watch(owner.index);
            }
        }
        // Fog lowers the image contrast, so the quality (optional: the weather may be off).
        SignalSpec fog = kindSpec("env/extinction/visible", SignalType::Scalar, units::per_m);
        fog.optional = true;
        fog.feedback = true;
        extinction_ = d.input(fog);
        SignalSpec flow =
            kindSpec(base() + "/flow", SignalType::ScalarArray, units::rad, "optical");
        flow.arrayLength = 2;
        flow_ = d.output(flow);
        quality_ = d.output(kindSpec(base() + "/quality", SignalType::Scalar, units::one));
        distance_ = d.output(kindSpec(base() + "/distance", SignalType::Scalar, units::m));
        time_ = d.output(kindSpec(base() + "/integration_time", SignalType::Scalar, units::s));
        return {};
    }

    Status init(const WorldView& view, const ModuleConfig&, SignalBus& bus) override {
        const World& raw = view.raw();
        const World::Colliders& colliders = raw.colliders;
        shapes_.clear();
        owners_.clear();
        offsets_.clear();
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId owner = colliders.body[c];
            if (owner != kWorldBody && owner.index == body_) {
                continue;  // the sensor looks out of its own body
            }
            ShapeCaster::Shape shape;
            shape.type = colliders.shape[c];
            shape.size = colliders.size[c];
            shape.hull =
                shape.type == ShapeType::ConvexHull ? &raw.hulls.at(colliders.hull[c]) : nullptr;
            shape.field = shape.type == ShapeType::HeightField
                              ? &raw.heightfields.at(colliders.hull[c])
                              : nullptr;
            shape.pose = owner == kWorldBody ? colliders.poseInBody[c]
                                             : compose(rest_[owner.index], colliders.poseInBody[c]);
            shapes_.push_back(shape);
            offsets_.push_back(colliders.poseInBody[c]);
            owners_.push_back(owner == kWorldBody || !poses_[owner.index].valid()
                                  ? -1
                                  : static_cast<int>(owner.index));
        }
        // The rays, in normalised image coordinates.
        rays_.clear();
        const double half = std::tan(fov_ / 2.0);
        for (int i = 0; i < kRaysPerSide; ++i) {
            for (int j = 0; j < kRaysPerSide; ++j) {
                const double x = half * (2.0 * (i + 0.5) / kRaysPerSide - 1.0);
                const double y = half * (2.0 * (j + 0.5) / kRaysPerSide - 1.0);
                rays_.push_back({x, y});
            }
        }
        bus.writeN(flow_, std::array<double, 2>{0.0, 0.0});
        bus.write(quality_, 0.0);
        bus.write(distance_, std::numeric_limits<double>::infinity());
        bus.write(time_, 0.0);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            if (owners_[k] >= 0) {
                shapes_[k].pose =
                    compose(bodyPose(bus, static_cast<std::uint32_t>(owners_[k])), offsets_[k]);
            }
        }
        caster_.update(shapes_);
        const Pose body = bodyPose(bus, body_);
        const Pose sensor = compose(body, mount_);
        const std::array<double, 4> optical = multiply(sensor.orientation, kSensorFromOptical);
        // The sensor's velocity and angular velocity, world → optical frame.
        Vec3 v{}, w{};
        if (twist_.valid()) {
            const std::span<const double> t = bus.readN(twist_);
            const Vec3 omega{t[3], t[4], t[5]};
            const Vec3 r{sensor.position[0] - body.position[0],
                         sensor.position[1] - body.position[1],
                         sensor.position[2] - body.position[2]};
            const Vec3 vw{t[0] + omega[1] * r[2] - omega[2] * r[1],
                          t[1] + omega[2] * r[0] - omega[0] * r[2],
                          t[2] + omega[0] * r[1] - omega[1] * r[0]};
            v = rotate(conjugate(optical), vw);
            w = rotate(conjugate(optical), omega);
        }
        // The flow of every ray that sees something in range, averaged: the sensor tracks the
        // texture of its whole field of view.
        std::array<double, 2> sum{};
        int seen = 0;
        double centre = std::numeric_limits<double>::infinity();
        for (const auto& [x, y] : rays_) {
            const double norm = std::sqrt(x * x + y * y + 1.0);
            const Vec3 dir = rotate(optical, {x / norm, y / norm, 1.0 / norm});
            const double range = caster_.cast(sensor.position, dir);
            const double depth = range / norm;  // along the optical axis
            if (std::abs(x) < 1e-12 && std::abs(y) < 1e-12) {
                centre = range;
            }
            if (!std::isfinite(range) || range < minRange_ || range > maxRange_) {
                continue;
            }
            const std::array<double, 2> f = flowRate(v, w, x, y, depth);
            sum[0] += f[0];
            sum[1] += f[1];
            ++seen;
        }
        const double dt = step_.seconds();
        std::array<double, 2> rate{};
        double quality = 0.0;
        if (seen > 0) {
            rate = {sum[0] / seen, sum[1] / seen};
            // Contrast through fog falls as exp(−σ d) (Koschmieder); the quality follows it.
            const double sigma = extinction_.valid() ? std::max(bus.read(extinction_), 0.0) : 0.0;
            const double d = std::isfinite(centre) ? centre : maxRange_;
            quality = kFullQuality * static_cast<double>(seen) / static_cast<double>(rays_.size()) *
                      std::exp(-sigma * d);
            if (std::hypot(rate[0], rate[1]) > maxFlow_) {
                quality = 0.0;  // beyond the speed it can track: it loses the texture
            }
        }
        if (!groundTruth_ && quality > 0.0 && noise_ > 0.0) {
            random::Stream rng(context.seed, context.stepIndex, sensor_);
            rate[0] += noise_ * rng.normal();
            rate[1] += noise_ * rng.normal();
        }
        if (quality <= 0.0) {
            rate = {0.0, 0.0};
        }
        bus.writeN(flow_, std::array<double, 2>{rate[0] * dt, rate[1] * dt});
        bus.write(quality_, std::round(quality));
        bus.write(distance_, centre >= minRange_ && centre <= maxRange_
                                 ? centre
                                 : std::numeric_limits<double>::infinity());
        bus.write(time_, dt);
        return {};
    }

private:
    Pose bodyPose(const SignalBus& bus, std::uint32_t b) const {
        if (b >= poses_.size() || !poses_[b].valid()) {
            return b < rest_.size() ? rest_[b] : Pose{};
        }
        const std::span<const double> v = bus.readN(poses_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    double fov_{0.0}, minRange_{0.0}, maxRange_{0.0}, maxFlow_{0.0}, noise_{0.0};
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    SignalId twist_, extinction_, flow_, quality_, distance_, time_;
    std::vector<ShapeCaster::Shape> shapes_;
    std::vector<Pose> offsets_;
    std::vector<int> owners_;  // body index of a moving shape, −1 for a fixed one
    std::vector<std::pair<double, double>> rays_;
    ShapeCaster caster_;
};

}  // namespace

std::unique_ptr<IModule> createOpticalFlow(const World& world, std::uint32_t sensor,
                                           bool groundTruth, const KindContext&) {
    return std::make_unique<OpticalFlow>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
