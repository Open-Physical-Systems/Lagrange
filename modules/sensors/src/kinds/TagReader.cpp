// tag_reader (sensors.md §9): a warehouse localisation reader for tags on the floor, at the
// detector level. RFID: the nearest tag within a radius of the antenna. QR (or Data Matrix): the
// tag nearest the optical axis in a downward camera's footprint (the sensor's -z), not hidden,
// with its offset and yaw in the reader's frame. Tags are scene prims named rfid_* or qr_*.
#include "Places.hpp"
#include "lagrange/random/Philox.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDefaultRfidRange = 0.1;      // m: an HF floor tag under the antenna, illustrative
constexpr double kDefaultQrFov = 0.8;          // rad across x and y of the footprint, illustrative
constexpr double kDefaultQrMaxRange = 0.5;     // m below the reader
constexpr double kDefaultQrMinRange = 0.01;    // m
constexpr double kDefaultMaxIncidence = 0.79;  // rad (45°) off the tag's normal, illustrative
constexpr double kDefaultPositionNoise = 5e-4;  // m, 1 sigma, illustrative
constexpr double kDefaultYawNoise = 0.005;      // rad, 1 sigma, illustrative
constexpr double kOcclusionMargin = 0.01;       // m in front of the tag that may be the tag

struct Tag {
    Place place;
    double code{-1.0};  // the number its name ends with, else -1
};

class TagReader final : public KindModule {
public:
    TagReader(const World& world, std::uint32_t sensor, bool groundTruth, bool qr,
              std::vector<Tag> tags)
        : KindModule(world, sensor, groundTruth), qr_(qr), tags_(std::move(tags)) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        range_ = kindSetting(world, sensor_, qr_ ? "max_range" : "range",
                             qr_ ? kDefaultQrMaxRange : kDefaultRfidRange);
        minRange_ = kindSetting(world, sensor_, "min_range", kDefaultQrMinRange);
        fovX_ = kindSetting(world, sensor_, "horizontal_fov", kDefaultQrFov);
        fovY_ = kindSetting(world, sensor_, "vertical_fov", fovX_);
        maxIncidence_ = kindSetting(world, sensor_, "max_incidence", kDefaultMaxIncidence);
        positionNoise_ = kindSetting(world, sensor_, "position_noise", kDefaultPositionNoise);
        yawNoise_ = kindSetting(world, sensor_, "yaw_noise", kDefaultYawNoise);
        if (range_ <= 0.0 || fovX_ <= 0.0 || fovX_ >= 3.1 || fovY_ <= 0.0 || fovY_ >= 3.1 ||
            positionNoise_ < 0.0 || yawNoise_ < 0.0) {
            return Status::error(
                StatusCode::InvalidConfig,
                "tag_reader " + name_ + ": range, field of view or noise out of range");
        }
        poses_.begin(world);
        poses_.watch(body_, d);
        for (const Tag& t : tags_) {
            poses_.watch(t.place.body, d);
        }
        if (qr_) {
            colliders_.declare(world, poses_, d);
        }
        detected_ = d.output(kindSpec(base() + "/detected", SignalType::Bool, units::one));
        tag_ = d.output(kindSpec(base() + "/tag", SignalType::Int, units::one));
        code_ = d.output(kindSpec(base() + "/code", SignalType::Int, units::one));
        tagPosition_ =
            d.output(kindSpec(base() + "/tag_position", SignalType::Vec3, units::m, "world_local"));
        offset_ =
            d.output(kindSpec(base() + "/offset", SignalType::Vec3, units::m, "sensor:" + name_));
        yaw_ = d.output(kindSpec(base() + "/yaw", SignalType::Scalar, units::rad));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        miss(bus);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose sensor = compose(poses_.pose(bus, body_), mount_);
        if (qr_) {
            colliders_.update(bus, poses_);
        }
        std::size_t best = tags_.size();
        double bestScore = std::numeric_limits<double>::infinity();
        Pose bestPose;
        for (std::size_t k = 0; k < tags_.size(); ++k) {
            const Pose tag = poses_.place(bus, tags_[k].place);
            const Vec3 v = toFrame(sensor, tag.position);
            double score = 0.0;
            if (!qr_) {
                score = norm(v);
                if (score > range_) {
                    continue;
                }
            } else {
                const double depth = -v[2];  // along the reader's -z
                if (depth < minRange_ || depth > range_ ||
                    std::abs(std::atan2(v[0], depth)) > fovX_ / 2.0 ||
                    std::abs(std::atan2(v[1], depth)) > fovY_ / 2.0) {
                    continue;
                }
                const double distance = norm(v);
                const Vec3 toSensor = scaled(sub(sensor.position, tag.position), 1.0 / distance);
                const Vec3 normal = rotate(tag.orientation, {0.0, 0.0, 1.0});
                if (dot(normal, toSensor) < std::cos(maxIncidence_)) {
                    continue;
                }
                if (!colliders_.clear(tag.position, sensor.position, kOcclusionMargin, body_)) {
                    continue;
                }
                score = std::hypot(v[0], v[1]);  // nearest the optical axis
            }
            if (score < bestScore) {
                bestScore = score;
                best = k;
                bestPose = tag;
            }
        }
        if (best == tags_.size()) {
            miss(bus);
            return {};
        }
        Vec3 offset = toFrame(sensor, bestPose.position);
        double yaw = kNaN;
        if (qr_) {
            const Vec3 axis = rotate(conjugate(sensor.orientation),
                                     rotate(bestPose.orientation, {1.0, 0.0, 0.0}));
            yaw = std::atan2(axis[1], axis[0]);
            if (!groundTruth_) {
                random::Stream rng(context.seed, context.stepIndex, sensor_);
                offset[0] += positionNoise_ * rng.normal();
                offset[1] += positionNoise_ * rng.normal();
                yaw = std::remainder(yaw + yawNoise_ * rng.normal(), 2.0 * std::numbers::pi);
            }
        }
        bus.write(detected_, 1.0);
        bus.write(tag_, static_cast<double>(best));
        bus.write(code_, tags_[best].code);
        bus.writeN(tagPosition_, bestPose.position);
        bus.writeN(offset_, offset);
        bus.write(yaw_, yaw);
        return {};
    }

private:
    void miss(SignalBus& bus) {
        const std::array<double, 3> none{kNaN, kNaN, kNaN};
        bus.write(detected_, 0.0);
        bus.write(tag_, -1.0);
        bus.write(code_, -1.0);
        bus.writeN(tagPosition_, none);
        bus.writeN(offset_, none);
        bus.write(yaw_, kNaN);
    }

    bool qr_;
    std::vector<Tag> tags_;
    double range_{kDefaultRfidRange}, minRange_{kDefaultQrMinRange};
    double fovX_{kDefaultQrFov}, fovY_{kDefaultQrFov}, maxIncidence_{kDefaultMaxIncidence};
    double positionNoise_{kDefaultPositionNoise}, yawNoise_{kDefaultYawNoise};
    BodyPoses poses_;
    Colliders colliders_;
    SignalId detected_, tag_, code_, tagPosition_, offset_, yaw_;
};

/// The number a name ends with ("qr_0042" → 42), else -1.
double trailingNumber(std::string_view name) {
    std::size_t start = name.size();
    while (start > 0 && std::isdigit(static_cast<unsigned char>(name[start - 1])) != 0) {
        --start;
    }
    if (start == name.size() || name.size() - start > 15) {
        return -1.0;
    }
    return std::stod(std::string(name.substr(start)));
}

}  // namespace

std::unique_ptr<IModule> createTagReader(const World& world, std::uint32_t sensor, bool groundTruth,
                                         const KindContext& context) {
    const std::string mode = lowerCase(kindText(world, sensor, "mode", "rfid"));
    if (mode != "rfid" && mode != "qr") {
        return refuseKind(world, sensor,
                          "tag_reader " + world.sensors.names[sensor] + ": mode '" + mode +
                              "' is neither rfid nor qr");
    }
    const bool qr = mode == "qr";
    const std::string prefix = lowerCase(kindText(world, sensor, "prefix", qr ? "qr_" : "rfid_"));
    std::vector<Tag> tags;
    for (Place& p : scenePlaces(world, context.scene)) {
        if (!lowerCase(p.name).starts_with(prefix)) {
            continue;
        }
        if (p.hasBox) {
            // The tag's point is the middle of its box's top face.
            p.offset = compose(p.offset, Pose{{p.center[0], p.center[1], p.center[2] + p.half[2]}});
        }
        const double code = trailingNumber(p.name);
        tags.push_back({std::move(p), code});
    }
    std::ranges::sort(tags, [](const Tag& a, const Tag& b) { return a.place.path < b.place.path; });
    return std::make_unique<TagReader>(world, sensor, groundTruth, qr, std::move(tags));
}

}  // namespace lagrange::sensors::kinds
