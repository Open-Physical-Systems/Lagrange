#include "GeoSensors.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/ImuNoise.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <tbb/parallel_for.h>
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

/// Ray against a convex polyhedron, as the intersection of its faces' half-spaces.
double convexHull(const Vec3& o, const Vec3& d, const ConvexHull& hull) {
    thread_local std::vector<std::array<double, 4>> planes;
    facePlanes(hull, planes);
    if (planes.empty()) {
        return kNoHit;
    }
    double enter = -kNoHit;
    double exit = kNoHit;
    for (const auto& plane : planes) {
        const Vec3 n{plane[0], plane[1], plane[2]};
        const double rate = dot(n, d);
        const double gap = plane[3] - dot(n, o);  // >= 0 when o is inside this face's plane
        if (rate == 0.0) {
            if (gap < 0.0) {
                return kNoHit;  // parallel and outside
            }
            continue;
        }
        const double t = gap / rate;
        if (rate < 0.0) {
            enter = std::max(enter, t);
        } else {
            exit = std::min(exit, t);
        }
        if (enter > exit) {
            return kNoHit;
        }
    }
    if (enter > 0.0) {
        return enter;
    }
    return exit > 0.0 ? exit : kNoHit;
}

}  // namespace

void facePlanes(const ConvexHull& hull, std::vector<std::array<double, 4>>& out) {
    out.clear();
    Vec3 centre{};
    for (const auto& p : hull.points) {
        for (std::size_t i = 0; i < 3; ++i) {
            centre.at(i) += p.at(i) / static_cast<double>(hull.points.size());
        }
    }
    for (const auto& t : hull.triangles) {
        const Vec3& a = hull.points.at(t[0]);
        const Vec3& b = hull.points.at(t[1]);
        const Vec3& c = hull.points.at(t[2]);
        const Vec3 u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const Vec3 v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        Vec3 n{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        const double length = std::sqrt(dot(n, n));
        if (length <= 0.0) {
            continue;  // a degenerate triangle has no plane
        }
        for (double& x : n) {
            x /= length;
        }
        double offset = dot(n, a);
        if (dot(n, centre) > offset) {
            n = {-n[0], -n[1], -n[2]};
            offset = -offset;
        }
        out.push_back({n[0], n[1], n[2], offset});
    }
}

double raycast(ShapeType shape, const std::array<double, 3>& size, const Pose& shapePose,
               const std::array<double, 3>& origin, const std::array<double, 3>& dir,
               const ConvexHull* hull) {
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
        case ShapeType::ConvexHull:
            return hull != nullptr ? convexHull(o, d, *hull) : kNoHit;
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
    SensorModule(const World& world, std::uint32_t sensor, const char* kind, bool groundTruth)
        : groundTruth_(groundTruth)
        , sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , label_(std::string(kind) + "_" + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(periodOf(world.sensors.rateHz[sensor])) {}

    [[nodiscard]] ModuleInfo info() const override {
        return {"sensors", label_, 0, 1, groundTruth_, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }  // a lidar keeps no state
    Status restoreState(StateReader&) override { return {}; }

protected:
    bool groundTruth_;  // the ground_truth substitute: no rendering, no noise
    std::uint32_t sensor_;
    std::string name_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
};

class Lidar final : public SensorModule {
public:
    Lidar(const World& world, std::uint32_t sensor, bool groundTruth,
          std::shared_ptr<gpu::RayContext> rayContext)
        : SensorModule(world, sensor, "lidar", groundTruth), rayContext_(std::move(rayContext)) {}

    [[nodiscard]] ModuleInfo info() const override {
        ModuleInfo out = SensorModule::info();
        if (rayContext_) {
            out.deterministic = false;
            out.minimumTier = CapabilityTier::Tier1;
        }
        return out;
    }

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
                        owner == kWorldBody ? -1 : static_cast<int>(owner.index),
                        colliders.shape[c] == ShapeType::ConvexHull
                            ? &world.raw().hulls.at(colliders.hull[c])
                            : nullptr};
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
        nearest_.assign(samples_, kNoHit);
        casterShapes_.resize(shapes_.size());
        if (rayContext_ && !groundTruth_) {
            hullFaces_.assign(shapes_.size(), {});
            for (std::size_t k = 0; k < shapes_.size(); ++k) {
                if (shapes_[k].hull != nullptr) {
                    facePlanes(*shapes_[k].hull, hullFaces_[k]);
                }
            }
            std::vector<std::array<float, 3>> directions(samples_);
            for (std::uint32_t i = 0; i < samples_; ++i) {
                const Vec3 d = layout_.direction(i);
                directions[i] = {static_cast<float>(d[0]), static_cast<float>(d[1]),
                                 static_cast<float>(d[2])};
            }
            scan_ = std::make_unique<gpu::RayScan>(rayContext_, directions);
            castShapes_.resize(shapes_.size());
            hits_.resize(samples_);
        }
        bus.writeN(ranges_, buffer_);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            bus.writeN(ranges_, buffer_);  // no rendering: no returns (+inf from init)
            return {};
        }
        const Pose sensor = compose(bodyPose(bus, body_), mount_);
        // Place every shape once per scan, not once per beam.
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            const Shape& shape = shapes_[k];
            placed_[k] =
                shape.body < 0 ? shape.pose : compose(bodyPose(bus, shape.body), shape.pose);
        }
        if (scan_) {
            castOnGpu(sensor);
            for (std::uint32_t i = 0; i < samples_; ++i) {
                nearest_[i] = static_cast<double>(hits_[i].range);
            }
        } else {
            castOnCpu(sensor);
        }
        for (std::uint32_t i = 0; i < samples_; ++i) {
            double nearest = nearest_[i];
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
        const ConvexHull* hull;  // in the world tables, which outlive the run
    };

    /// Tier 0: exact casts in double, culled by Embree (ShapeCaster), beams in parallel. Each
    /// beam is independent, so the result does not depend on the threads.
    void castOnCpu(const Pose& sensor) {
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            casterShapes_[k] = {shapes_[k].type, shapes_[k].size, placed_[k], shapes_[k].hull};
        }
        caster_.update(casterShapes_);
        tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
            nearest_[i] =
                caster_.cast(sensor.position, rotate(sensor.orientation, layout_.direction(i)));
        });
    }

    /// Shapes relative to the sensor, in double; the caster converts to float last
    /// (huge-worlds.md §1: only the GPU sees float positions, and only small ones).
    void castOnGpu(const Pose& sensor) {
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            gpu::CastShape& out = castShapes_[k];
            out.type = static_cast<std::uint32_t>(shapes_[k].type);
            out.size = shapes_[k].size;
            for (std::size_t i = 0; i < 3; ++i) {
                out.position[i] = placed_[k].position[i] - sensor.position[i];
            }
            out.orientation = placed_[k].orientation;
            out.faces = hullFaces_[k];
        }
        scan_->cast(castShapes_, sensor.orientation, hits_);
    }

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
    std::vector<Pose> placed_;     // world pose of each shape for the current scan
    std::vector<double> nearest_;  // per beam, before range limits and noise
    ShapeCaster caster_;
    std::vector<ShapeCaster::Shape> casterShapes_;
    std::vector<double> buffer_;
    SignalId ranges_;
    std::shared_ptr<gpu::RayContext> rayContext_;  // null: cast on the CPU
    std::unique_ptr<gpu::RayScan> scan_;
    std::vector<gpu::CastShape> castShapes_;
    std::vector<std::vector<std::array<double, 4>>> hullFaces_;  // per shape, for the GPU
    std::vector<gpu::CastHit> hits_;
};

class Imu final : public SensorModule {
public:
    Imu(const World& world, std::uint32_t sensor, bool groundTruth)
        : SensorModule(world, sensor, "imu", groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        if (!movingBodies(raw)[body_]) {
            return Status::error(
                StatusCode::InvalidConfig,
                "IMU " + name_ + " is on '" + raw.bodies.names[body_] + "', which never moves");
        }
        if (!groundTruth_) {
            const auto& settings = world.sensors().settings[sensor_];
            const double rate = world.sensors().rateHz[sensor_];
            const double dt = step_.seconds();
            accel_ = ImuChannel(ImuNoise::from(settings, "accel", rate), dt);
            gyro_ = ImuChannel(ImuNoise::from(settings, "gyro", rate), dt);
            biased_ = ImuNoise::from(settings, "accel", rate).hasBias() ||
                      ImuNoise::from(settings, "gyro", rate).hasBias();
        }
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
        if (biased_) {
            // Separate streams from the white noise, so that adding a bias term leaves the
            // white noise of a run as it was.
            if (!started_) {
                random::Stream rng(context.seed, 0, kTurnOnStream);
                accel_.start(rng);
                gyro_.start(rng);
                started_ = true;
            } else {
                random::Stream rng(context.seed, context.stepIndex, kBiasStream);
                accel_.advance(rng);
                gyro_.advance(rng);
            }
        }
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (std::size_t i = 0; i < 3; ++i) {
            force[i] = accel_.measure(i, force[i], rng.normal());
            rate[i] = gyro_.measure(i, rate[i], rng.normal());
        }
        bus.writeN(force_, force);
        bus.writeN(rate_, rate);
        bus.writeN(orientation_, multiply(body, mount_.orientation));
        return {};
    }

    Status saveState(StateWriter& writer) const override {
        writer.put(started_);
        writer.putDoubles(accel_.state());
        writer.putDoubles(gyro_.state());
        return {};
    }

    Status restoreState(StateReader& reader) override {
        std::array<double, 9> accel{}, gyro{};
        if (!reader.get(started_) || !reader.getDoubles(accel) || !reader.getDoubles(gyro)) {
            return Status::error(StatusCode::Internal, "IMU " + name_ + ": truncated state");
        }
        accel_.setState(accel);
        gyro_.setState(gyro);
        return {};
    }

private:
    static constexpr std::uint32_t kTurnOnStream = 1;  // Philox entities besides 0, the white
    static constexpr std::uint32_t kBiasStream = 2;    // noise
    ImuChannel accel_, gyro_;  // all terms off for the ground-truth substitute
    bool biased_{false};
    bool started_{false};  // turn-on bias drawn
    SignalId pose_, twist_, acceleration_, force_, rate_, orientation_;
};

/// Rounds a rate's period to whole milliseconds, so that a camera at 30 Hz lines up with the
/// other modules' steps (the scheduler refuses a base tick below 1 µs).
Duration wholeMilliseconds(double rateHz) {
    const auto ms = std::max<std::int64_t>(std::llround(1e3 / std::max(rateHz, 1e-3)), 1);
    return Duration{ms * 1'000'000};
}

class Camera final : public SensorModule {
public:
    Camera(const World& world, std::uint32_t sensor, bool groundTruth,
           std::shared_ptr<const CameraScene> scene)
        : SensorModule(world, sensor, "camera", groundTruth), scene_(std::move(scene)) {
        step_ = wholeMilliseconds(world.sensors.rateHz[sensor]);
    }

    [[nodiscard]] ModuleInfo info() const override {
        ModuleInfo out = SensorModule::info();
        out.deterministic = groundTruth_;  // rendering is a GPU path (invariant 7)
        return out;
    }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& sensors = world.sensors();
        const double width = setting(sensors, sensor_, "width", 640.0);
        const double height = setting(sensors, sensor_, "height", 480.0);
        const double fov = setting(sensors, sensor_, "horizontal_fov", 1.047);
        if (width < 1.0 || height < 1.0 || width > 16384.0 || height > 16384.0 || fov <= 0.0 ||
            fov >= 3.1) {
            return Status::error(StatusCode::InvalidConfig,
                                 "camera " + name_ + ": image size or horizontal_fov out of range");
        }
        camera_.width = static_cast<std::uint32_t>(width);
        camera_.height = static_cast<std::uint32_t>(height);
        camera_.fx = width / 2.0 / std::tan(fov / 2.0);
        camera_.fy = camera_.fx;
        camera_.cx = (width - 1.0) / 2.0;
        camera_.cy = (height - 1.0) / 2.0;
        camera_.near = setting(sensors, sensor_, "near_clip", 0.05);
        far_ = setting(sensors, sensor_, "far_clip", 100.0);

        const World& raw = world.raw();
        const std::vector<bool> moving = movingBodies(raw);
        rest_ = restPoses(raw);
        poses_.assign(moving.size(), SignalId{});
        const auto watch = [&](std::uint32_t b) {
            if (b < moving.size() && moving[b] && !poses_[b].valid()) {
                poses_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/pose", SignalType::Pose,
                                         units::m, "world_local"));
            }
        };
        watch(body_);
        if (scene_ && !groundTruth_) {
            for (const CameraScene::Drawable& drawable : scene_->drawables) {
                if (drawable.body != kNoIndex) {
                    watch(drawable.body);
                }
            }
        }
        const auto image = [&](const char* what, Unit unit) {
            SignalSpec out =
                spec("sensor/" + name_ + "/" + what, SignalType::Image, unit, "optical");
            return d.output(std::move(out));
        };
        image_ = image("image", Unit{});
        depth_ = image("depth", units::m);
        instance_ = image("instance", Unit{});
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (!groundTruth_) {
            if (!scene_ || !scene_->raster) {
                return Status::error(StatusCode::Unsupported,
                                     "camera " + name_ +
                                         " needs a Vulkan device, and none was usable (set the "
                                         "sensors domain to ground_truth to run without one)");
            }
            renderer_ =
                std::make_unique<gpu::RasterCamera>(scene_->raster, camera_.width, camera_.height);
            instances_.resize(scene_->drawables.size());
        }
        publish(bus);  // empty images until the first frame
        return {};
    }

    Status step(const StepContext&, SignalBus& bus) override {
        if (groundTruth_) {
            publish(bus);
            return {};
        }
        const Pose sensor = compose(bodyPose(bus, body_), mount_);
        camera_.position = sensor.position;
        camera_.orientation = multiply(sensor.orientation, kSensorFromOptical);
        for (std::size_t k = 0; k < instances_.size(); ++k) {
            const CameraScene::Drawable& drawable = scene_->drawables[k];
            const Pose pose = drawable.body == kNoIndex
                                  ? drawable.offset
                                  : compose(bodyPose(bus, drawable.body), drawable.offset);
            instances_[k] = {drawable.mesh,  pose.position,  pose.orientation,
                             drawable.scale, drawable.color, drawable.prim};
        }
        renderer_->render(instances_, camera_, images_);
        for (float& depth : images_.depth) {
            if (depth > far_) {
                depth = std::numeric_limits<float>::infinity();
                // The colour keeps whatever was drawn: a far clip limits the range, not the view.
            }
        }
        publish(bus);
        return {};
    }

private:
    /// Publishes the last images; empty buffers (with their size) before the first frame and
    /// for the ground-truth substitute.
    void publish(SignalBus& bus) {
        const auto buffer = [&](const char* encoding, const auto& pixels) {
            auto out = std::make_shared<Buffer>();
            out->width = camera_.width;
            out->height = camera_.height;
            out->encoding = encoding;
            const auto bytes = std::as_bytes(std::span{pixels});
            out->bytes.assign(bytes.begin(), bytes.end());
            return out;
        };
        bus.publishBuffer(image_, buffer("rgba8", images_.rgba));
        bus.publishBuffer(depth_, buffer("r32f", images_.depth));
        bus.publishBuffer(instance_, buffer("r32u", images_.instance));
    }

    Pose bodyPose(const SignalBus& bus, std::uint32_t b) const {
        if (b >= poses_.size() || !poses_[b].valid()) {
            return b < rest_.size() ? rest_[b] : Pose{};
        }
        const std::span<const double> v = bus.readN(poses_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    std::shared_ptr<const CameraScene> scene_;
    gpu::PinholeCamera camera_;
    double far_{100.0};
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::unique_ptr<gpu::RasterCamera> renderer_;
    std::vector<gpu::RasterInstance> instances_;
    gpu::RasterImages images_;
    SignalId image_, depth_, instance_;
};
}  // namespace

namespace {

std::vector<std::unique_ptr<IModule>> createModules(
    const World& world, bool groundTruth, const std::shared_ptr<gpu::RayContext>& rayContext,
    const std::shared_ptr<const CameraScene>& cameraScene, bool groundTruthCameras) {
    std::vector<std::unique_ptr<IModule>> modules;
    for (std::uint32_t s = 0; s < world.sensors.names.size(); ++s) {
        switch (world.sensors.type[s]) {
            case SensorType::Lidar:
                modules.push_back(std::make_unique<Lidar>(world, s, groundTruth, rayContext));
                break;
            case SensorType::Imu:
                modules.push_back(std::make_unique<Imu>(world, s, groundTruth));
                break;
            case SensorType::Camera:
                modules.push_back(std::make_unique<Camera>(
                    world, s, groundTruth || groundTruthCameras, cameraScene));
                break;
            case SensorType::Gnss:
            case SensorType::Magnetometer:
            case SensorType::Barometer:
                break;  // createGeoSensors, below
        }
    }
    createGeoSensors(world, groundTruth, modules);
    return modules;
}

}  // namespace

std::vector<std::unique_ptr<IModule>> createSensorModules(
    const World& world, std::shared_ptr<gpu::RayContext> rayContext,
    std::shared_ptr<const CameraScene> cameraScene, bool groundTruthCameras) {
    return createModules(world, false, rayContext, cameraScene, groundTruthCameras);
}

std::vector<std::unique_ptr<IModule>> createSensorSubstitutes(const World& world) {
    return createModules(world, true, nullptr, nullptr, true);
}

std::shared_ptr<CameraScene> createCameraScene(const Model& model,
                                               const std::string& preferredDevice,
                                               std::string& error) {
    auto scene = std::make_shared<CameraScene>();
    std::vector<gpu::MeshSource> meshes;
    meshes.reserve(model.scene.meshes.size());
    for (const TriMesh& mesh : model.scene.meshes) {
        meshes.push_back({mesh.positions, mesh.normals, mesh.indices});
    }
    for (std::uint32_t p = 0; p < model.scene.prims.size(); ++p) {
        const ScenePrim& prim = model.scene.prims[p];
        if (prim.mesh == kNoIndex) {
            continue;
        }
        CameraScene::Drawable drawable;
        drawable.mesh = prim.mesh;
        drawable.body = prim.body;
        drawable.offset = prim.offset;
        drawable.scale = prim.scale;
        if (prim.material != kNoIndex) {
            drawable.color = model.scene.materials[prim.material].rgba;
        }
        drawable.prim = p;
        scene->drawables.push_back(drawable);
    }
    scene->raster = gpu::RasterScene::create(preferredDevice, meshes, error);
    return scene->raster ? scene : nullptr;
}

}  // namespace lagrange::sensors
