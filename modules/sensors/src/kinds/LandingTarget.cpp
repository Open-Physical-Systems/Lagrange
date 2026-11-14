// landing_target (sensors.md §9): a vision landing-target detector at the detector level, with
// PX4's LANDING_TARGET meaning (MAVLink common.xml: angle_x, angle_y, distance, size_x, size_y).
// No fiducial texture is drawn and decoded: the target is a scene prim, seen when it is in the
// field of view, in range, facing the camera, not behind a collider and large enough in pixels.
#include "Places.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <limits>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDefaultFov = 1.047;            // rad, 60°: the camera kind's default
constexpr double kDefaultWidth = 640.0;          // px
constexpr double kDefaultMaxRange = 30.0;        // m
constexpr double kDefaultMinRange = 0.05;        // m
constexpr double kDefaultMinPixels = 12.0;       // px across: illustrative decode limit
constexpr double kDefaultMaxIncidence = 1.2217;  // rad, 70°: illustrative
constexpr double kDefaultPixelNoise = 0.5;       // px, 1 sigma: illustrative corner jitter
constexpr double kDefaultMargin = 0.05;          // m, what may stand in front of the target
constexpr double kDefaultSize = 1.0;             // m, a target without a mesh

class LandingTarget final : public KindModule {
public:
    LandingTarget(const World& world, std::uint32_t sensor, bool groundTruth, Place target,
                  double sizeX, double sizeY)
        : KindModule(world, sensor, groundTruth)
        , target_(std::move(target))
        , sizeX_(sizeX)
        , sizeY_(sizeY) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        hfov_ = kindSetting(world, sensor_, "horizontal_fov", kDefaultFov);
        width_ = kindSetting(world, sensor_, "width", kDefaultWidth);
        const double height = kindSetting(world, sensor_, "height", width_ * 3.0 / 4.0);
        vfov_ = kindSetting(world, sensor_, "vertical_fov",
                            2.0 * std::atan(std::tan(hfov_ / 2.0) * height / width_));
        maxRange_ = kindSetting(world, sensor_, "max_range", kDefaultMaxRange);
        minRange_ = kindSetting(world, sensor_, "min_range", kDefaultMinRange);
        minPixels_ = kindSetting(world, sensor_, "min_pixels", kDefaultMinPixels);
        maxIncidence_ = kindSetting(world, sensor_, "max_incidence", kDefaultMaxIncidence);
        pixelNoise_ = kindSetting(world, sensor_, "pixel_noise", kDefaultPixelNoise);
        margin_ = kindSetting(world, sensor_, "occlusion_margin", kDefaultMargin);
        if (hfov_ <= 0.0 || hfov_ >= 3.1 || vfov_ <= 0.0 || vfov_ >= 3.1 || width_ < 1.0 ||
            maxRange_ <= minRange_ || minPixels_ < 0.0 || pixelNoise_ < 0.0 || sizeX_ <= 0.0 ||
            sizeY_ <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "landing_target " + name_ +
                                     ": field of view, width, range, min_pixels, pixel_noise or "
                                     "target_size out of range");
        }
        focal_ = width_ / 2.0 / std::tan(hfov_ / 2.0);
        poses_.begin(world);
        poses_.watch(body_, d);
        poses_.watch(target_.body, d);
        colliders_.declare(world, poses_, d);
        detected_ = d.output(kindSpec(base() + "/detected", SignalType::Bool, units::one));
        angleX_ = d.output(kindSpec(base() + "/angle_x", SignalType::Scalar, units::rad));
        angleY_ = d.output(kindSpec(base() + "/angle_y", SignalType::Scalar, units::rad));
        distance_ = d.output(kindSpec(base() + "/distance", SignalType::Scalar, units::m));
        position_ =
            d.output(kindSpec(base() + "/position", SignalType::Vec3, units::m, "sensor:" + name_));
        angularX_ = d.output(kindSpec(base() + "/size_x", SignalType::Scalar, units::rad));
        angularY_ = d.output(kindSpec(base() + "/size_y", SignalType::Scalar, units::rad));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        miss(bus);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose sensor = compose(poses_.pose(bus, body_), mount_);
        const Pose target = poses_.place(bus, target_);
        const Vec3 point = target.position;
        const Vec3 v = toFrame(sensor, point);  // x forward, y left, z up
        const double distance = norm(v);
        // The optical frame: z forward, x right, y down.
        const double xo = -v[1];
        const double yo = -v[2];
        const double zo = v[0];
        if (zo <= 0.0 || distance < minRange_ || distance > maxRange_) {
            miss(bus);
            return {};
        }
        const double angleX = std::atan2(xo, zo);
        const double angleY = std::atan2(yo, zo);
        const double sizeX = 2.0 * std::atan(sizeX_ / 2.0 / distance);
        const double sizeY = 2.0 * std::atan(sizeY_ / 2.0 / distance);
        const double half = std::max(sizeX, sizeY) / 2.0;
        if (std::abs(angleX) + half > hfov_ / 2.0 || std::abs(angleY) + half > vfov_ / 2.0) {
            miss(bus);  // not wholly in the image
            return {};
        }
        // Facing the camera: the target's +z towards the sensor.
        const Vec3 normal = rotate(target.orientation, {0.0, 0.0, 1.0});
        const Vec3 toSensor = scaled(sub(sensor.position, point), 1.0 / distance);
        const double cosIncidence = dot(normal, toSensor);
        if (cosIncidence < std::cos(maxIncidence_)) {
            miss(bus);
            return {};
        }
        // Nothing in front of it.
        colliders_.update(bus, poses_);
        if (!colliders_.clear(point, sensor.position, margin_, body_)) {
            miss(bus);
            return {};
        }
        // Pixels across its smaller side, foreshortened: the decoder needs min_pixels.
        const double pixels =
            focal_ * std::min(sizeX_, sizeY_) * std::sqrt(cosIncidence) / distance;
        const double chance =
            minPixels_ > 0.0 ? std::clamp((pixels - minPixels_) / minPixels_, 0.0, 1.0) : 1.0;
        random::Stream rng(context.seed, context.stepIndex, sensor_);
        const double draw = rng.uniform();
        if (chance <= 0.0 || (!groundTruth_ && draw >= chance)) {
            miss(bus);
            return {};
        }
        double ax = angleX;
        double ay = angleY;
        double range = distance;
        if (!groundTruth_ && pixelNoise_ > 0.0) {
            // Corner jitter of pixel_noise moves the centre by that over f, and the range, taken
            // from the apparent size, by the same fraction of the pixels across.
            ax += pixelNoise_ / focal_ * rng.normal();
            ay += pixelNoise_ / focal_ * rng.normal();
            range += pixelNoise_ / pixels * distance * rng.normal();
        }
        const double tx = std::tan(ax);
        const double ty = std::tan(ay);
        const double k = range / std::sqrt(1.0 + tx * tx + ty * ty);
        const std::array<double, 3> position{k, -k * tx, -k * ty};  // back to x fwd, y left, z up
        bus.write(detected_, 1.0);
        bus.write(angleX_, ax);
        bus.write(angleY_, ay);
        bus.write(distance_, range);
        bus.writeN(position_, position);
        bus.write(angularX_, sizeX);
        bus.write(angularY_, sizeY);
        return {};
    }

private:
    void miss(SignalBus& bus) {
        bus.write(detected_, 0.0);
        bus.write(angleX_, kNaN);
        bus.write(angleY_, kNaN);
        bus.write(distance_, kNaN);
        const std::array<double, 3> none{kNaN, kNaN, kNaN};
        bus.writeN(position_, none);
        bus.write(angularX_, kNaN);
        bus.write(angularY_, kNaN);
    }

    Place target_;
    double sizeX_, sizeY_;
    double hfov_{kDefaultFov}, vfov_{0.0}, width_{kDefaultWidth}, focal_{1.0};
    double maxRange_{kDefaultMaxRange}, minRange_{kDefaultMinRange};
    double minPixels_{kDefaultMinPixels}, maxIncidence_{kDefaultMaxIncidence};
    double pixelNoise_{kDefaultPixelNoise}, margin_{kDefaultMargin};
    BodyPoses poses_;
    Colliders colliders_;
    SignalId detected_, angleX_, angleY_, distance_, position_, angularX_, angularY_;
};

}  // namespace

std::unique_ptr<IModule> createLandingTarget(const World& world, std::uint32_t sensor,
                                             bool groundTruth, const KindContext& context) {
    const std::string name = kindText(world, sensor, "target", "landing_pad");
    const std::vector<Place> places = scenePlaces(world, context.scene);
    const Place* found = nullptr;
    for (const Place& p : places) {
        if (namesPlace(p, name)) {
            found = &p;
            break;
        }
    }
    if (found == nullptr) {
        return refuseKind(world, sensor,
                          "landing_target " + world.sensors.names[sensor] + ": no prim named '" +
                              name + "' in the scene (parameter target)");
    }
    Place target = *found;
    double sizeX = kDefaultSize;
    double sizeY = kDefaultSize;
    if (target.hasBox) {
        // The target point is the middle of its box's top face, the side that faces up.
        target.offset =
            compose(target.offset,
                    Pose{{target.center[0], target.center[1], target.center[2] + target.half[2]}});
        sizeX = 2.0 * target.half[0];
        sizeY = 2.0 * target.half[1];
    }
    const double size = kindSetting(world, sensor, "target_size", 0.0);
    if (size > 0.0) {
        sizeX = size;
        sizeY = size;
    }
    return std::make_unique<LandingTarget>(world, sensor, groundTruth, std::move(target), sizeX,
                                           sizeY);
}

}  // namespace lagrange::sensors::kinds
