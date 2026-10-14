#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace lagrange::sensors {

namespace {

using Vec3 = std::array<double, 3>;

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// Smallest positive root of a t² + 2 b t + c = 0, or kNoHit.
double firstRoot(double a, double b, double c) {
    if (a <= 0.0) {
        return kNoHit;
    }
    const double discriminant = b * b - a * c;
    if (discriminant < 0.0) {
        return kNoHit;
    }
    const double root = std::sqrt(discriminant);
    const double near = (-b - root) / a;
    if (near > 0.0) {
        return near;
    }
    const double far = (-b + root) / a;
    return far > 0.0 ? far : kNoHit;
}

double sphere(const Vec3& o, const Vec3& d, double radius) {
    return firstRoot(dot(d, d), dot(o, d), dot(o, o) - radius * radius);
}

/// Infinite cylinder side along z, limited to |z| <= half; all hits with t > 0 considered.
double cylinderSide(const Vec3& o, const Vec3& d, double radius, double half) {
    const double a = d[0] * d[0] + d[1] * d[1];
    const double b = o[0] * d[0] + o[1] * d[1];
    const double c = o[0] * o[0] + o[1] * o[1] - radius * radius;
    if (a <= 0.0) {
        return kNoHit;
    }
    const double discriminant = b * b - a * c;
    if (discriminant < 0.0) {
        return kNoHit;
    }
    const double root = std::sqrt(discriminant);
    for (const double t : {(-b - root) / a, (-b + root) / a}) {
        if (t > 0.0 && std::abs(o[2] + t * d[2]) <= half) {
            return t;
        }
    }
    return kNoHit;
}

double cap(const Vec3& o, const Vec3& d, double radius, double z) {
    if (d[2] == 0.0) {
        return kNoHit;
    }
    const double t = (z - o[2]) / d[2];
    const double x = o[0] + t * d[0];
    const double y = o[1] + t * d[1];
    return t > 0.0 && x * x + y * y <= radius * radius ? t : kNoHit;
}

double box(const Vec3& o, const Vec3& d, const Vec3& half) {
    double enter = -kNoHit;
    double exit = kNoHit;
    for (std::size_t i = 0; i < 3; ++i) {
        if (d[i] == 0.0) {
            if (std::abs(o[i]) > half[i]) {
                return kNoHit;
            }
            continue;
        }
        double t0 = (-half[i] - o[i]) / d[i];
        double t1 = (half[i] - o[i]) / d[i];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        enter = std::max(enter, t0);
        exit = std::min(exit, t1);
    }
    if (enter > exit) {
        return kNoHit;
    }
    if (enter > 0.0) {
        return enter;
    }
    return exit > 0.0 ? exit : kNoHit;
}

}  // namespace

double raycast(ShapeType shape, const std::array<double, 3>& size, const Pose& shapePose,
               const std::array<double, 3>& origin, const std::array<double, 3>& dir) {
    // Into the shape's frame.
    const std::array<double, 4> inverse = conjugate(shapePose.orientation);
    const Vec3 o =
        rotate(inverse, {origin[0] - shapePose.position[0], origin[1] - shapePose.position[1],
                         origin[2] - shapePose.position[2]});
    const Vec3 d = rotate(inverse, dir);
    switch (shape) {
        case ShapeType::Plane: {
            if (d[2] == 0.0) {
                return kNoHit;
            }
            const double t = -o[2] / d[2];
            return t > 0.0 ? t : kNoHit;
        }
        case ShapeType::Sphere:
            return sphere(o, d, size[0]);
        case ShapeType::Box:
            return box(o, d, size);
        case ShapeType::Cylinder:
            return std::min({cylinderSide(o, d, size[0], size[1]), cap(o, d, size[0], size[1]),
                             cap(o, d, size[0], -size[1])});
        case ShapeType::Capsule:
            return std::min({cylinderSide(o, d, size[0], size[1]),
                             sphere({o[0], o[1], o[2] - size[1]}, d, size[0]),
                             sphere({o[0], o[1], o[2] + size[1]}, d, size[0])});
    }
    return kNoHit;
}

namespace {

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec out;
    out.name = std::move(name);
    out.type = type;
    out.unit = unit;
    out.frame = std::move(frame);
    return out;
}

double setting(const World::Sensors& sensors, std::uint32_t sensor, const char* key,
               double fallback) {
    const auto& map = sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

Duration periodOf(double rateHz) {
    return Duration{static_cast<std::int64_t>(std::llround(1e9 / std::max(rateHz, 1e-3)))};
}

/// Common to all sensors: the mount and how its body's pose is read.
class SensorModule : public IModule {
public:
    SensorModule(const World& world, std::uint32_t sensor, const char* kind)
        : sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , label_(std::string(kind) + "_" + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(periodOf(world.sensors.rateHz[sensor])) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", label_, 0, 1, false, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }  // sensors keep no state
    Status restoreState(StateReader&) override { return {}; }

protected:
    std::uint32_t sensor_;
    std::string name_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
};

class Lidar final : public SensorModule {
public:
    Lidar(const World& world, std::uint32_t sensor) : SensorModule(world, sensor, "lidar") {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& sensors = world.sensors();
        layout_ = LidarLayout::from(sensors.settings[sensor_]);
        samples_ = layout_.beams();
        minRange_ = setting(sensors, sensor_, "min_range", 0.1);
        maxRange_ = setting(sensors, sensor_, "max_range", 30.0);
        noise_ = setting(sensors, sensor_, "range_noise", 0.0);
        if (samples_ == 0) {
            return Status::error(StatusCode::InvalidConfig, "lidar " + name_ + " has no samples");
        }
        // Read the pose of every moving body that carries geometry, and of the sensor's body.
        const World& raw = world.raw();
        const std::vector<bool> moving = movingBodies(raw);
        rest_ = restPoses(raw);
        poses_.assign(moving.size(), SignalId{});
        const auto watch = [&](std::uint32_t b) {
            if (moving[b] && !poses_[b].valid()) {
                poses_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/pose", SignalType::Pose,
                                         units::m, "world_local"));
            }
        };
        watch(body_);
        for (const BodyId owner : raw.colliders.body) {
            if (owner != kWorldBody) {
                watch(owner.index);
            }
        }
        ranges_ = d.output([&] {
            SignalSpec out = spec("sensor/" + name_ + "/ranges", SignalType::ScalarArray, units::m);
            out.arrayLength = samples_;
            return out;
        }());
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        const World::Colliders& colliders = world.colliders();
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId owner = colliders.body[c];
            Shape shape{colliders.shape[c], colliders.size[c], colliders.poseInBody[c],
                        owner == kWorldBody ? -1 : static_cast<int>(owner.index)};
            // Geometry that never moves is placed in the world once.
            if (owner == kWorldBody) {
                shape.pose = colliders.poseInBody[c];
            } else if (!poses_[owner.index].valid()) {
                shape.pose = compose(rest_[owner.index], colliders.poseInBody[c]);
                shape.body = -1;
            }
            shapes_.push_back(shape);
        }
        placed_.resize(shapes_.size());
        buffer_.assign(samples_, kNoHit);
        bus.writeN(ranges_, buffer_);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose sensor = compose(bodyPose(bus, body_), mount_);
        // Place every shape once per scan, not once per beam.
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            const Shape& shape = shapes_[k];
            placed_[k] =
                shape.body < 0 ? shape.pose : compose(bodyPose(bus, shape.body), shape.pose);
        }
        for (std::uint32_t i = 0; i < samples_; ++i) {
            const Vec3 dir = rotate(sensor.orientation, layout_.direction(i));
            double nearest = kNoHit;
            for (std::size_t k = 0; k < shapes_.size(); ++k) {
                nearest = std::min(nearest, raycast(shapes_[k].type, shapes_[k].size, placed_[k],
                                                    sensor.position, dir));
            }
            if (nearest < minRange_ || nearest > maxRange_) {
                buffer_[i] = kNoHit;  // no valid return (sensor_msgs/LaserScan convention)
                continue;
            }
            if (noise_ > 0.0) {
                random::Stream rng(context.seed, context.stepIndex, i);
                nearest += noise_ * rng.normal();
            }
            buffer_[i] = std::clamp(nearest, minRange_, maxRange_);
        }
        bus.writeN(ranges_, buffer_);
        return {};
    }

private:
    struct Shape {
        ShapeType type;
        std::array<double, 3> size;
        Pose pose;  // world pose when body < 0, else pose in the body
        int body;
    };

    Pose bodyPose(const SignalBus& bus, std::uint32_t b) const {
        if (!poses_[b].valid()) {
            return rest_[b];
        }
        const std::span<const double> v = bus.readN(poses_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    LidarLayout layout_;
    std::uint32_t samples_{0};  // beams, all rings
    double minRange_{0.0}, maxRange_{0.0}, noise_{0.0};
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::vector<Shape> shapes_;
    std::vector<Pose> placed_;  // world pose of each shape for the current scan
    std::vector<double> buffer_;
    SignalId ranges_;
};

class Imu final : public SensorModule {
public:
    Imu(const World& world, std::uint32_t sensor) : SensorModule(world, sensor, "imu") {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        if (!movingBodies(raw)[body_]) {
            return Status::error(
                StatusCode::InvalidConfig,
                "IMU " + name_ + " is on '" + raw.bodies.names[body_] + "', which never moves");
        }
        accelNoise_ = setting(world.sensors(), sensor_, "accel_noise", 0.0);
        gyroNoise_ = setting(world.sensors(), sensor_, "gyro_noise", 0.0);
        const std::string body = "body/" + raw.bodies.names[body_];
        pose_ = d.input(spec(body + "/pose", SignalType::Pose, units::m, "world_local"));
        twist_ = d.input(spec(body + "/twist", SignalType::Wrench, units::m_per_s, "world_local"));
        acceleration_ =
            d.input(spec(body + "/acceleration", SignalType::Wrench, units::m_per_s2, "body"));
        const std::string base = "sensor/" + name_;
        force_ =
            d.output(spec(base + "/specific_force", SignalType::Vec3, units::m_per_s2, "sensor"));
        rate_ =
            d.output(spec(base + "/angular_rate", SignalType::Vec3, units::rad_per_s, "sensor"));
        orientation_ =
            d.output(spec(base + "/orientation", SignalType::Quat, Unit{}, "world_local"));
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        const std::array<double, 3> zero{0.0, 0.0, 0.0};
        bus.writeN(force_, zero);
        bus.writeN(rate_, zero);
        const std::array<double, 4> identity{1.0, 0.0, 0.0, 0.0};
        bus.writeN(orientation_, identity);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const std::span<const double> pose = bus.readN(pose_);
        const std::array<double, 4> body{pose[3], pose[4], pose[5], pose[6]};
        const std::span<const double> twist = bus.readN(twist_);
        const std::span<const double> accel = bus.readN(acceleration_);
        const std::array<double, 4> unmount = conjugate(mount_.orientation);
        // Proper acceleration is already in the body frame; the angular rate is in the world.
        Vec3 force = rotate(unmount, {accel[0], accel[1], accel[2]});
        Vec3 rate = rotate(unmount, rotate(conjugate(body), {twist[3], twist[4], twist[5]}));
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (std::size_t i = 0; i < 3; ++i) {
            force[i] += accelNoise_ * rng.normal();
            rate[i] += gyroNoise_ * rng.normal();
        }
        bus.writeN(force_, force);
        bus.writeN(rate_, rate);
        bus.writeN(orientation_, multiply(body, mount_.orientation));
        return {};
    }

private:
    double accelNoise_{0.0}, gyroNoise_{0.0};
    SignalId pose_, twist_, acceleration_, force_, rate_, orientation_;
};

}  // namespace

std::vector<std::unique_ptr<IModule>> createSensorModules(const World& world) {
    std::vector<std::unique_ptr<IModule>> modules;
    for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
        switch (world.sensors.type[s]) {
            case SensorType::Lidar:
                modules.push_back(std::make_unique<Lidar>(world, s));
                break;
            case SensorType::Imu:
                modules.push_back(std::make_unique<Imu>(world, s));
                break;
        }
    }
    return modules;
}

}  // namespace lagrange::sensors
