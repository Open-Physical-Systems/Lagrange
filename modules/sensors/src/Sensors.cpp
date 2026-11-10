#include "GeoSensors.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/CameraModel.hpp"
#include "lagrange/sensors/ImuNoise.hpp"
#include "lagrange/sensors/Radar.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/sensors/ThermalCamera.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"
#include "lagrange/thermal/Materials.hpp"
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
    thread_local std::vector<std::array<double, 4>> computed;  // for a hull not in world tables
    if (hull.faces.empty()) {
        hullFacePlanes(hull, computed);
    }
    const std::vector<std::array<double, 4>>& planes = hull.faces.empty() ? computed : hull.faces;
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

/// Ray against one triangle (Moller-Trumbore), either side; kNoHit when it misses.
double triangle(const Vec3& o, const Vec3& d, const Vec3& a, const Vec3& b, const Vec3& c) {
    constexpr double kParallel = 1e-15;
    const Vec3 e1{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const Vec3 e2{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const Vec3 p{d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2],
                 d[0] * e2[1] - d[1] * e2[0]};
    const double det = dot(e1, p);
    if (std::abs(det) < kParallel) {
        return kNoHit;
    }
    const Vec3 s{o[0] - a[0], o[1] - a[1], o[2] - a[2]};
    const double u = dot(s, p) / det;
    if (u < 0.0 || u > 1.0) {
        return kNoHit;
    }
    const Vec3 q{s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2],
                 s[0] * e1[1] - s[1] * e1[0]};
    const double v = dot(d, q) / det;
    if (v < 0.0 || u + v > 1.0) {
        return kNoHit;
    }
    const double t = dot(e2, q) / det;
    return t > 0.0 ? t : kNoHit;
}

/// Ray against a height field: clipped to the grid's box, then the cells under the ray in
/// order (a 2D walk), two triangles each, split as HeightField says.
double heightField(const Vec3& o, const Vec3& d, const std::array<double, 3>& half,
                   const HeightField& field) {
    // The grid's box, a little taller than its heights.
    double enter = 0.0;
    double exit = kNoHit;
    const std::array<double, 3> lo{-half[0], -half[1], -half[2] - 1.0};
    const std::array<double, 3> hi{half[0], half[1], half[2] + 1.0};
    for (std::size_t i = 0; i < 3; ++i) {
        if (d.at(i) == 0.0) {
            if (o.at(i) < lo.at(i) || o.at(i) > hi.at(i)) {
                return kNoHit;
            }
            continue;
        }
        double t0 = (lo.at(i) - o.at(i)) / d.at(i);
        double t1 = (hi.at(i) - o.at(i)) / d.at(i);
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        enter = std::max(enter, t0);
        exit = std::min(exit, t1);
    }
    if (enter > exit) {
        return kNoHit;
    }
    const auto columns = static_cast<std::int64_t>(field.columns);
    const auto rows = static_cast<std::int64_t>(field.rows);
    const auto point = [&](std::int64_t c, std::int64_t r) -> Vec3 {
        return {(static_cast<double>(c) - static_cast<double>(columns - 1) / 2.0) * field.cellX,
                (static_cast<double>(r) - static_cast<double>(rows - 1) / 2.0) * field.cellY,
                field.heights[static_cast<std::size_t>(r * columns + c)]};
    };
    // Cell coordinates along the ray: u = x / cellX + (columns - 1) / 2, likewise v.
    const auto cellOf = [&](double t) {
        const double u = (o[0] + t * d[0]) / field.cellX + static_cast<double>(columns - 1) / 2.0;
        const double v = (o[1] + t * d[1]) / field.cellY + static_cast<double>(rows - 1) / 2.0;
        return std::array<std::int64_t, 2>{
            std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor(u)), 0, columns - 2),
            std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor(v)), 0, rows - 2)};
    };
    std::array<std::int64_t, 2> cell = cellOf(enter);
    const std::array<std::int64_t, 2> step{d[0] > 0.0 ? 1 : -1, d[1] > 0.0 ? 1 : -1};
    // The ray's parameter at the next cell border in x and in y, and between borders.
    const auto border = [&](std::size_t axis, std::int64_t index) {
        const double size = axis == 0 ? field.cellX : field.cellY;
        const double count = axis == 0 ? static_cast<double>(columns) : static_cast<double>(rows);
        const double at = (static_cast<double>(index) - (count - 1) / 2.0) * size;
        return d.at(axis) == 0.0 ? kNoHit : (at - o.at(axis)) / d.at(axis);
    };
    std::array<double, 2> next{border(0, cell[0] + (step[0] > 0 ? 1 : 0)),
                               border(1, cell[1] + (step[1] > 0 ? 1 : 0))};
    const std::array<double, 2> delta{d[0] == 0.0 ? kNoHit : field.cellX / std::abs(d[0]),
                                      d[1] == 0.0 ? kNoHit : field.cellY / std::abs(d[1])};
    for (std::int64_t guard = 0; guard < columns + rows; ++guard) {
        const Vec3 a = point(cell[0], cell[1]);
        const Vec3 b = point(cell[0] + 1, cell[1]);
        const Vec3 c = point(cell[0] + 1, cell[1] + 1);
        const Vec3 e = point(cell[0], cell[1] + 1);
        const double t = std::min(triangle(o, d, a, b, c), triangle(o, d, a, c, e));
        if (t < kNoHit) {
            return t;
        }
        const std::size_t axis = next[0] < next[1] ? 0 : 1;
        if (next.at(axis) > exit) {
            break;
        }
        cell.at(axis) += step.at(axis);
        if (cell[0] < 0 || cell[1] < 0 || cell[0] > columns - 2 || cell[1] > rows - 2) {
            break;
        }
        next.at(axis) += delta.at(axis);
    }
    return kNoHit;
}

}  // namespace

void facePlanes(const ConvexHull& hull, std::vector<std::array<double, 4>>& out) {
    hullFacePlanes(hull, out);
}

double raycast(ShapeType shape, const std::array<double, 3>& size, const Pose& shapePose,
               const std::array<double, 3>& origin, const std::array<double, 3>& dir,
               const ConvexHull* hull, const HeightField* field) {
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
        case ShapeType::HeightField:
            return field != nullptr ? heightField(o, d, size, *field) : kNoHit;
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

    Status declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) override {
        const World::Sensors& sensors = world.sensors();
        layout_ = LidarLayout::from(sensors.settings[sensor_]);
        samples_ = layout_.beams();
        minRange_ = setting(sensors, sensor_, "min_range", 0.1);
        maxRange_ = setting(sensors, sensor_, "max_range", 30.0);
        noise_ = setting(sensors, sensor_, "range_noise", 0.0);
        distort_ = setting(sensors, sensor_, "motion_distortion", 1.0) != 0.0 && !groundTruth_;
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
        intensities_ = d.output([&] {
            SignalSpec out =
                spec("sensor/" + name_ + "/intensities", SignalType::ScalarArray, units::one);
            out.arrayLength = samples_;
            return out;
        }());
        wavelength1550_ = setting(sensors, sensor_, "wavelength", 905e-9) > 1.2e-6;
        referenceReflectivity_ = setting(sensors, sensor_, "max_range_reflectivity", 0.1);
        // In the weather domain's air (config "weather": it runs): extinction at the lidar's
        // wavelength, the fog's share of it, the rain rate and the window's contamination.
        weather_ = config.get<bool>("weather").value_or(false) && !groundTruth_;
        if (weather_) {
            const bool infrared = setting(sensors, sensor_, "wavelength", 905e-9) > 1.2e-6;
            const auto in = [&](const std::string& name) {
                SignalSpec s = spec(name, SignalType::Scalar, units::per_m);
                s.feedback = true;  // the weather changes slowly
                return s;
            };
            extinction_ =
                d.input(in(infrared ? "env/extinction/nir_1550" : "env/extinction/nir_905"));
            fogExtinction_ =
                d.input(in(infrared ? "env/extinction/fog_1550" : "env/extinction/fog_905"));
            SignalSpec rain = in("env/rain_rate");
            rain.unit = units::m_per_s;
            rainRate_ = d.input(rain);
            SignalSpec snow = in("env/snow_rate");
            snow.unit = units::m_per_s;
            snowRate_ = d.input(snow);
            SignalSpec dirt = in("sensor/" + name_ + "/contamination");
            dirt.unit = units::one;
            contamination_ = d.input(dirt);
            SignalSpec field = in("env/extinction/field");
            field.type = SignalType::Field3D;
            fogField_ = d.input(field);
            clutter_.pulseLength = setting(sensors, sensor_, "pulse_length", clutter_.pulseLength);
            clutter_.beamDiameter =
                setting(sensors, sensor_, "beam_diameter", clutter_.beamDiameter);
            clutter_.reflectivity =
                setting(sensors, sensor_, "max_range_reflectivity", clutter_.reflectivity);
        }
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        const World::Colliders& colliders = world.colliders();
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId owner = colliders.body[c];
            Shape shape{colliders.shape[c],
                        colliders.size[c],
                        colliders.poseInBody[c],
                        owner == kWorldBody ? -1 : static_cast<int>(owner.index),
                        colliders.shape[c] == ShapeType::ConvexHull
                            ? &world.raw().hulls.at(colliders.hull[c])
                            : nullptr,
                        colliders.shape[c] == ShapeType::HeightField
                            ? &world.raw().heightfields.at(colliders.hull[c])
                            : nullptr,
                        colliders.material[c] != kNoMaterial
                            ? &world.raw().surfaceMaterials.at(colliders.material[c])
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
        // Water is no collider, but a lidar sees its surface: a mirror (or a box of depth below it,
        // from under water). What enters it is absorbed: no return from the bottom.
        waterMaterials_.clear();
        const std::vector<const SchemaInstance*> waters = world.instancesOf("WaterAPI_1");
        waterMaterials_.reserve(waters.size());
        for (const SchemaInstance* w : waters) {
            SurfaceMaterial surface;
            (void)surfaceMaterialPreset("water", surface);
            surface.reflectivity905 = surface.reflectivity1550 =
                w->real("water:reflectivity").value_or(surface.reflectivity905);
            waterMaterials_.push_back(surface);
            const double depth = w->real("water:depth").value_or(2.0);
            Pose pose = world.raw().prims.poseInBody.at(w->prim.index);
            pose.position[2] -= depth / 2.0;
            shapes_.push_back(Shape{ShapeType::Box,
                                    {w->real("water:sizeX").value_or(10.0) / 2.0,
                                     w->real("water:sizeY").value_or(10.0) / 2.0, depth / 2.0},
                                    pose,
                                    -1,
                                    nullptr,
                                    nullptr,
                                    &waterMaterials_.back()});
        }
        placed_.resize(shapes_.size());
        beamDirection_.resize(samples_);
        for (std::uint32_t i = 0; i < samples_; ++i) {
            beamDirection_[i] = layout_.direction(i);
        }
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
        hitShape_.assign(samples_, kNoShapeHit);
        intensity_.assign(samples_, 0.0);
        bus.writeN(ranges_, buffer_);
        bus.writeN(intensities_, intensity_);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            bus.writeN(ranges_, buffer_);  // no rendering: no returns (+inf from init)
            bus.writeN(intensities_, intensity_);
            return {};
        }
        const Pose sensor = compose(bodyPose(bus, body_), mount_);
        sweep(sensor);
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
                hitShape_[i] = hits_[i].shape == gpu::kNoShape ? kNoShapeHit : hits_[i].shape;
            }
        } else {
            castOnCpu(context);
        }
        // What each hit returns, from its surface; a surface too faint for the lidar drops out.
        tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
            intensity_[i] = 0.0;
            if (hitShape_[i] == kNoShapeHit || !std::isfinite(nearest_[i])) {
                return;
            }
            const Shape& shape = shapes_[hitShape_[i]];
            const Pose& from = columnPose_[i % layout_.horizontal];
            const Vec3 dir = rotate(from.orientation, beamDirection_[i]);
            const Vec3 at{from.position[0] + dir[0] * nearest_[i],
                          from.position[1] + dir[1] * nearest_[i],
                          from.position[2] + dir[2] * nearest_[i]};
            const Vec3 n = surfaceNormal(shape.type, shape.size, placed_[hitShape_[i]], at,
                                         shape.hull, shape.field);
            const double cosine = -(dir[0] * n[0] + dir[1] * n[1] + dir[2] * n[2]);
            static const SurfaceMaterial kDefault{};
            const LidarReturn back = lidarReturn(
                shape.material != nullptr ? *shape.material : kDefault, shape.material != nullptr,
                wavelength1550_, cosine, nearest_[i], maxRange_, referenceReflectivity_);
            intensity_[i] = back.intensity;
            if (!back.detected) {
                nearest_[i] = kNoHit;
            }
        });
        if (weather_) {
            // A fog field: its optical depth along each beam up to the hit, and its value at the
            // sensor for the clutter near it (taken as the extinction at the lidar's wavelength).
            const std::shared_ptr<const fields::Field> field = fogFieldSignal_.read(bus, fogField_);
            const fields::Vec3 at = sensor.position;
            const double local = field ? field->sampleScalar(at) : 0.0;
            const SensorWeather w{bus.read(extinction_), bus.read(fogExtinction_) + local,
                                  bus.read(rainRate_) + (kSnowClutter * bus.read(snowRate_)),
                                  bus.read(contamination_)};
            tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
                // Its own stream, so that the range noise below is the same with or without.
                random::Stream rng(context.seed, context.stepIndex, kWeatherStream + i);
                const double u1 = rng.uniform();
                const double u2 = rng.uniform();
                const double u3 = rng.uniform();
                double depth = 0.0;
                if (field && std::isfinite(nearest_[i])) {
                    const Pose& from = columnPose_[i % layout_.horizontal];
                    const Vec3 dir = rotate(from.orientation, beamDirection_[i]);
                    const double to = std::min(nearest_[i], maxRange_);
                    const Vec3& o = from.position;
                    depth = field->integrate(
                        o, {o[0] + dir[0] * to, o[1] + dir[1] * to, o[2] + dir[2] * to});
                }
                const double hit = nearest_[i];
                nearest_[i] =
                    weatherRange(hit, minRange_, maxRange_, w, clutter_, u1, u2, u3, depth);
                if (nearest_[i] < hit) {  // clutter, not the surface: as faint as the air is
                    const double r = std::max(nearest_[i], 1.0);
                    intensity_[i] = clutter_.reflectivity / (r * r);
                }
            });
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
        for (std::uint32_t i = 0; i < samples_; ++i) {
            if (!std::isfinite(buffer_[i])) {
                intensity_[i] = 0.0;
            }
        }
        bus.writeN(ranges_, buffer_);
        bus.writeN(intensities_, intensity_);
        scanEnd_ = sensor;
        hasScanEnd_ = true;
        return {};
    }

    // Where the last scan ended is where this one starts.
    Status saveState(StateWriter& writer) const override {
        writer.put(hasScanEnd_);
        writer.put(scanEnd_);
        return {};
    }
    Status restoreState(StateReader& reader) override {
        if (!reader.get(hasScanEnd_) || !reader.get(scanEnd_)) {
            return Status::error(StatusCode::InvalidConfig, "lidar " + name_ + ": bad state");
        }
        return {};
    }
    void shiftOrigin(const std::array<double, 3>& shift) override {
        for (std::size_t i = 0; i < 3; ++i) {
            scanEnd_.position.at(i) -= shift.at(i);
        }
    }

private:
    /// Where the sensor is as each column fires: a rotating lidar's columns fire one after another
    /// through the scan, the last at the scan's end. Between the last scan's end and this one's the
    /// sensor is taken to move evenly. Only the sensor moves: everything else stays where it is at
    /// the scan's end.
    void sweep(const Pose& sensor) {
        const std::uint32_t columns = layout_.horizontal;
        columnPose_.resize(columns);
        if (!distort_ || !hasScanEnd_) {
            std::fill(columnPose_.begin(), columnPose_.end(), sensor);
            moving_ = false;
            return;
        }
        std::array<double, 4> to = sensor.orientation;
        double dot = 0.0;
        for (std::size_t k = 0; k < 4; ++k) {
            dot += scanEnd_.orientation.at(k) * to.at(k);
        }
        if (dot < 0.0) {  // the short way round
            dot = -dot;
            for (double& v : to) {
                v = -v;
            }
        }
        const double angle = std::acos(std::min(dot, 1.0));
        moving_ = angle != 0.0 || scanEnd_.position != sensor.position;
        for (std::uint32_t c = 0; c < columns; ++c) {
            const double f = static_cast<double>(c + 1) / columns;
            Pose& pose = columnPose_[c];
            for (std::size_t k = 0; k < 3; ++k) {
                pose.position.at(k) =
                    scanEnd_.position.at(k) + (sensor.position.at(k) - scanEnd_.position.at(k)) * f;
            }
            // Slerp; nearly equal, the straight line (its sine would divide by nothing).
            const double a = angle > 1e-9 ? std::sin((1.0 - f) * angle) / std::sin(angle) : 1.0 - f;
            const double b = angle > 1e-9 ? std::sin(f * angle) / std::sin(angle) : f;
            double norm = 0.0;
            for (std::size_t k = 0; k < 4; ++k) {
                pose.orientation.at(k) = a * scanEnd_.orientation.at(k) + b * to.at(k);
                norm += pose.orientation.at(k) * pose.orientation.at(k);
            }
            norm = std::sqrt(norm);
            for (double& v : pose.orientation) {
                v /= norm;
            }
        }
    }

    struct Shape {
        ShapeType type;
        std::array<double, 3> size;
        Pose pose;  // world pose when body < 0, else pose in the body
        int body;
        const ConvexHull* hull;  // in the world tables, which outlive the run
        const HeightField* field;
        const SurfaceMaterial* material;  // in the world tables, or none
    };

    /// Tier 0: exact casts in double, culled by Embree (ShapeCaster), beams in parallel. Each
    /// beam is independent, so the result does not depend on the threads.
    void castOnCpu(const StepContext& context) {
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            casterShapes_[k] = {shapes_[k].type, shapes_[k].size, placed_[k], shapes_[k].hull,
                                shapes_[k].field};
        }
        caster_.update(casterShapes_);
        tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
            const Pose& start = columnPose_[i % layout_.horizontal];
            const Vec3 dir = rotate(start.orientation, beamDirection_[i]);
            Vec3 from = start.position;
            double travelled = 0.0;
            nearest_[i] = std::numeric_limits<double>::infinity();
            hitShape_[i] = kNoShapeHit;
            // Through what the beam passes (glass): by chance, (seed, step, beam).
            for (std::uint32_t passes = 0; passes < kMaxPasses; ++passes) {
                const services::RayHit hit = caster_.castHit(from, dir);
                if (!hit.hit()) {
                    return;
                }
                const SurfaceMaterial* m = shapes_[hit.shape].material;
                if (m != nullptr && m->transmission > 0.0) {
                    random::Stream rng(context.seed, context.stepIndex,
                                       kTransmissionStream + i * kMaxPasses + passes);
                    if (rng.uniform() < m->transmission) {
                        // Past both of its faces; a thick pane's far face comes next.
                        travelled += hit.distance + kPastSurfaceM;
                        from = {start.position[0] + dir[0] * travelled,
                                start.position[1] + dir[1] * travelled,
                                start.position[2] + dir[2] * travelled};
                        continue;
                    }
                }
                nearest_[i] = travelled + hit.distance;
                hitShape_[i] = hit.shape;
                return;
            }
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
        // The beams of a moving sensor, in the frame of its pose at the scan's end.
        if (moving_ || beamsMoved_) {
            const std::array<double, 4> back{sensor.orientation[0], -sensor.orientation[1],
                                             -sensor.orientation[2], -sensor.orientation[3]};
            beamDirections_.resize(samples_);
            beamOrigins_.resize(samples_);
            for (std::uint32_t i = 0; i < samples_; ++i) {
                const Pose& from = columnPose_[i % layout_.horizontal];
                const Vec3 world = rotate(from.orientation, beamDirection_[i]);
                const Vec3 d = rotate(back, world);
                const Vec3 o = rotate(back, {from.position[0] - sensor.position[0],
                                             from.position[1] - sensor.position[1],
                                             from.position[2] - sensor.position[2]});
                beamDirections_[i] = {static_cast<float>(d[0]), static_cast<float>(d[1]),
                                      static_cast<float>(d[2])};
                beamOrigins_[i] = {static_cast<float>(o[0]), static_cast<float>(o[1]),
                                   static_cast<float>(o[2])};
            }
            scan_->setBeams(beamDirections_, beamOrigins_);
            beamsMoved_ = moving_;
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

    static constexpr std::uint32_t kWeatherStream = 0x8000'0000U;       // + beam
    static constexpr std::uint32_t kTransmissionStream = 0x4000'0000U;  // + beam × passes + pass
    static constexpr std::uint32_t kNoShapeHit = 0xFFFF'FFFFU;
    static constexpr std::uint32_t kMaxPasses = 6;
    static constexpr double kPastSurfaceM = 1e-4;
    bool distort_{true};
    bool hasScanEnd_{false};
    bool moving_{false};      // this scan's sensor moved
    bool beamsMoved_{false};  // the GPU's beams are not the sensor's at rest
    Pose scanEnd_;
    std::vector<Pose> columnPose_;
    std::vector<Vec3> beamDirection_;  // per beam, sensor frame
    std::vector<std::array<float, 3>> beamDirections_, beamOrigins_;
    SignalId intensities_;
    std::vector<SurfaceMaterial> waterMaterials_;  // reserved before shapes_ point into it
    std::vector<std::uint32_t> hitShape_;
    std::vector<double> intensity_;
    bool wavelength1550_{false};
    double referenceReflectivity_{0.1};
    bool weather_{false};
    LidarClutter clutter_;
    SignalId extinction_, fogExtinction_, rainRate_, snowRate_, contamination_, fogField_;
    // Flakes return more false echoes than the same water as drops: taken as this much rain
    // (illustrative, of the order lidar snow studies show; sensors.md).
    static constexpr double kSnowClutter = 3.0;
    fields::FieldSignal fogFieldSignal_;
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
        if (const std::string victim = emiVictim(world, sensor_, "imu_adc");
            !victim.empty() && !groundTruth_) {
            // Noise the EMI domain adds to the ADC channels (emi.md §2), 1 σ.
            emiAccel_ =
                d.input(optionalSpec(victim + "accel_noise", SignalType::Scalar, units::m_per_s2));
            emiGyro_ =
                d.input(optionalSpec(victim + "gyro_noise", SignalType::Scalar, units::rad_per_s));
        }
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
        if (emiAccel_.valid()) {
            // Its own stream, so that the IMU's own noise is the same with or without EMI.
            random::Stream emi(context.seed, context.stepIndex, kEmiStream);
            const double a = bus.read(emiAccel_);
            const double g = bus.read(emiGyro_);
            for (std::size_t i = 0; i < 3; ++i) {
                force[i] += a * emi.normal();
                rate[i] += g * emi.normal();
            }
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
    static constexpr std::uint32_t kEmiStream = 3;
    ImuChannel accel_, gyro_;  // all terms off for the ground-truth substitute
    bool biased_{false};
    bool started_{false};  // turn-on bias drawn
    SignalId pose_, twist_, acceleration_, force_, rate_, orientation_;
    SignalId emiAccel_, emiGyro_;  // EMI noise, when the IMU is a victim
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

    Status declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) override {
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
        // The sensor model (CameraModel.hpp): lens, vignetting, rolling shutter, exposure.
        CameraModelConfig model;
        model.width = camera_.width;
        model.height = camera_.height;
        model.fx = camera_.fx;
        model.fy = camera_.fy;
        model.cx = camera_.cx;
        model.cy = camera_.cy;
        const double lens = setting(sensors, sensor_, "distortion_model", 0.0);
        model.lens.model = lens == 1.0   ? Lens::Model::PlumbBob
                           : lens == 2.0 ? Lens::Model::Equidistant
                                         : Lens::Model::None;
        for (std::size_t k = 0; k < model.lens.k.size(); ++k) {
            model.lens.k[k] =
                setting(sensors, sensor_, ("distortion_k" + std::to_string(k + 1)).c_str(), 0.0);
        }
        model.vignetting = setting(sensors, sensor_, "vignetting", 0.0);
        model.rollingShutterTime = setting(sensors, sensor_, "rolling_shutter_time", 0.0);
        if (const double time = setting(sensors, sensor_, "exposure_time", 0.0); time > 0.0) {
            Exposure e;
            e.time = time;
            e.autoExposure = setting(sensors, sensor_, "auto_exposure", 0.0) != 0.0;
            e.electronRate = setting(sensors, sensor_, "electron_rate", 0.0);
            e.gain = setting(sensors, sensor_, "gain", 1.0);
            e.readNoise = setting(sensors, sensor_, "read_noise", 2.0);
            e.fullWell = setting(sensors, sensor_, "full_well", 10000.0);
            e.bitDepth = static_cast<int>(setting(sensors, sensor_, "bit_depth", 8.0));
            if (e.electronRate <= 0.0 || e.gain <= 0.0 || e.readNoise < 0.0 || e.fullWell <= 0.0 ||
                e.bitDepth < 1 || e.bitDepth > 16) {
                return Status::error(StatusCode::InvalidConfig,
                                     "camera " + name_ + ": exposure settings out of range");
            }
            model.exposure = e;
        }
        if (model.vignetting < 0.0 || model.vignetting > 1.0 || model.rollingShutterTime < 0.0) {
            return Status::error(
                StatusCode::InvalidConfig,
                "camera " + name_ + ": vignetting or rolling shutter out of range");
        }
        model_ = std::make_unique<CameraModel>(model);
        if (!model_->active()) {
            model_.reset();
        }

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
        if (model_ && model_->renderCamera().width > 0 && model.rollingShutterTime > 0.0 &&
            body_ < moving.size() && moving[body_] && !groundTruth_) {
            twist_ = d.input(spec("body/" + raw.bodies.names[body_] + "/twist", SignalType::Wrench,
                                  units::m_per_s, "world_local"));
        }
        if (scene_ && !groundTruth_) {
            for (const CameraScene::Drawable& drawable : scene_->drawables) {
                if (drawable.body != kNoIndex) {
                    watch(drawable.body);
                }
            }
            hinges_.clear();
            for (const CameraScene::Drawable& drawable : scene_->drawables) {
                SignalId id;
                if (!drawable.hinge.empty()) {
                    SignalSpec angle = spec(drawable.hinge, SignalType::Scalar, units::rad);
                    angle.feedback = true;
                    angle.optional = true;
                    id = d.input(angle);
                }
                hinges_.push_back(id);
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
        if (setting(sensors, sensor_, "thermal", 0.0) != 0.0 && !groundTruth_) {
            if (Status ok = declareThermal(world, config, d); !ok.ok()) {
                return ok;
            }
        }
        if (model_ && model.exposure) {
            exposureOut_ =
                d.output(spec("sensor/" + name_ + "/exposure_time", SignalType::Scalar, units::s));
        }
        // The weather's veil (config "weather"): visible extinction and the lens's contamination.
        // A thermal camera sees through the visible veil; its own extinction is owed.
        weather_ = config.get<bool>("weather").value_or(false) && !groundTruth_ && !thermal_;
        if (weather_) {
            SignalSpec e = spec("env/extinction/visible", SignalType::Scalar, units::per_m);
            e.feedback = true;
            extinction_ = d.input(e);
            SignalSpec dirt =
                spec("sensor/" + name_ + "/contamination", SignalType::Scalar, units::one);
            dirt.feedback = true;
            contamination_ = d.input(dirt);
            SignalSpec field = spec("env/extinction/field", SignalType::Field3D, units::per_m);
            field.feedback = true;
            fogField_ = d.input(field);
        }
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
            const gpu::PinholeCamera& render = model_ ? model_->renderCamera() : camera_;
            renderer_ =
                std::make_unique<gpu::RasterCamera>(scene_->raster, render.width, render.height);
            instances_.resize(scene_->drawables.size());
        }
        publish(bus);  // empty images until the first frame
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            publish(bus);
            return {};
        }
        const Pose sensor = compose(bodyPose(bus, body_), mount_);
        camera_.position = sensor.position;
        camera_.orientation = multiply(sensor.orientation, kSensorFromOptical);
        for (std::size_t k = 0; k < instances_.size(); ++k) {
            const CameraScene::Drawable& drawable = scene_->drawables[k];
            Pose pose = drawable.body == kNoIndex
                            ? drawable.offset
                            : compose(bodyPose(bus, drawable.body), drawable.offset);
            if (hinges_[k].valid()) {
                const double half = bus.read(hinges_[k]) / 2.0;
                const std::array<double, 3>& a = drawable.hingeAxis;
                pose = compose(pose, Pose{{0.0, 0.0, 0.0},
                                          {std::cos(half), std::sin(half) * a[0],
                                           std::sin(half) * a[1], std::sin(half) * a[2]}});
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
        if (model_) {
            gpu::PinholeCamera render = model_->renderCamera();
            render.near = camera_.near;
            render.position = camera_.position;
            render.orientation = camera_.orientation;
            renderer_->render(instances_, render, rendered_, context.t.seconds());
            std::array<double, 3> omega{};
            if (twist_.valid()) {
                const std::span<const double> t = bus.readN(twist_);
                omega = rotate(conjugate(camera_.orientation), {t[3], t[4], t[5]});
            }
            if (exposureOut_.valid()) {
                bus.write(exposureOut_, model_->exposureTime());
            }
            model_->apply(rendered_, omega, context.seed, context.stepIndex, sensor_ << 16U,
                          images_);
        } else {
            renderer_->render(instances_, camera_, images_, context.t.seconds());
        }
        for (float& depth : images_.depth) {
            if (depth > far_) {
                depth = std::numeric_limits<float>::infinity();
                // The colour keeps whatever was drawn: a far clip limits the range, not the view.
            }
        }
        if (weather_) {
            const std::shared_ptr<const fields::Field> field = fogFieldSignal_.read(bus, fogField_);
            if (field) {
                fieldDepth(*field, sensor);
            } else {
                fieldDepth_.clear();
            }
            applyCameraWeather(images_.rgba, images_.depth, bus.read(extinction_),
                               bus.read(contamination_), fieldDepth_);
        }
        if (thermal_) {
            stepThermal(context, bus);
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
        if (thermal_) {
            bus.publishBuffer(temperatureOut_, buffer("r32f", temperatures_));
        }
    }

    /// The thermal camera (ThermalCameraAPI_1): every drawable's temperature and emissivity, from
    /// the thermal node it belongs to, a person's skin, or the ambient air.
    Status declareThermal(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
        const World::Sensors& sensors = world.sensors();
        const std::optional<double> ambient = config.get<double>("ambient_temperature");
        if (!ambient || *ambient <= 0.0) {
            return Status::error(
                StatusCode::InvalidConfig,
                "thermal camera " + name_ +
                    " needs the site's ambient temperature (--ambient-temperature, "
                    "--weather temperature=K or the profile's "
                    "thermal.ambient_temperature)");
        }
        thermal_ = true;
        ambient_ = *ambient;
        const double shortest = setting(sensors, sensor_, "thermal_band_min", 8e-6);
        const double longest = setting(sensors, sensor_, "thermal_band_max", 14e-6);
        netd_ = setting(sensors, sensor_, "thermal_netd", 0.05);
        if (shortest <= 0.0 || longest <= shortest || netd_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "thermal camera " + name_ + ": band or netd out of range");
        }
        band_ = std::make_unique<ThermalBand>(shortest, longest);
        displayMin_ = setting(sensors, sensor_, "thermal_display_min", 0.0);
        displayMax_ = setting(sensors, sensor_, "thermal_display_max", 0.0);
        const double palette = setting(sensors, sensor_, "thermal_palette", 0.0);
        palette_ = palette == 1.0   ? ThermalPalette::BlackHot
                   : palette == 2.0 ? ThermalPalette::Ironbow
                                    : ThermalPalette::WhiteHot;
        temperatureOut_ = d.output(
            spec("sensor/" + name_ + "/temperature", SignalType::Image, units::K, "optical"));
        temperatures_.assign(std::size_t{camera_.width} * camera_.height, 0.0F);

        // The thermal nodes: their prim's path and body, and their temperature signal.
        const World& raw = world.raw();
        struct Node {
            std::string path;
            BodyId body;
            bool onLink{false};  // the node is its body's own prim
            double emissivity{0.9};
            SignalId temperature;
        };
        std::vector<Node> nodes;
        for (const SchemaInstance* s : world.instancesOf("ThermalAPI_1")) {
            if (s->prim.index >= raw.prims.paths.size()) {
                continue;
            }
            Node node;
            node.path = raw.prims.paths[s->prim.index];
            node.body = raw.prims.body[s->prim.index];
            node.onLink = node.body.index < raw.bodies.prim.size() &&
                          raw.bodies.prim[node.body.index] == s->prim;
            node.emissivity = s->real("thermal:emissivity").value_or(0.9);
            SignalSpec t =
                spec("thermal/" + std::string(world.signalInstanceName(s->prim)) + "/temperature",
                     SignalType::Scalar, units::K);
            t.feedback = true;
            t.optional = true;
            node.temperature = d.input(t);
            nodes.push_back(std::move(node));
        }
        // Links with a thermal material (RFC 0012): their mean temperature, their material's
        // emissivity (bare aluminium reads cold, as it does through a real thermal camera).
        for (const SchemaInstance* s : world.instancesOf("ThermalMaterialAPI_1")) {
            if (s->prim.index >= raw.prims.paths.size()) {
                continue;
            }
            std::size_t body = raw.bodies.prim.size();
            for (std::size_t b = 0; b < raw.bodies.prim.size(); ++b) {
                if (raw.bodies.prim[b] == s->prim) {
                    body = b;
                }
            }
            if (body == raw.bodies.prim.size()) {
                continue;
            }
            Node node;
            node.path = raw.prims.paths[s->prim.index];
            node.body = BodyId{static_cast<std::uint32_t>(body)};
            node.onLink = true;
            double emissivity = 0.9;
            if (const auto preset = s->token("thermal:material:preset")) {
                if (const auto found = thermal::thermalMaterialPreset(*preset)) {
                    emissivity = found->emissivity;
                }
            }
            node.emissivity = s->real("thermal:material:emissivity").value_or(emissivity);
            SignalSpec t = spec("thermal/" + raw.bodies.names[body] + "/temperature",
                                SignalType::Scalar, units::K);
            t.feedback = true;
            t.optional = true;
            node.temperature = d.input(t);
            nodes.push_back(std::move(node));
        }
        std::vector<std::string> people;  // body name prefixes of humans
        for (const SchemaInstance* s : world.instancesOf("HumanAPI_1")) {
            people.push_back("human_" + std::string(world.signalInstanceName(s->prim)) + "_");
        }
        surfaces_.clear();
        if (!scene_) {
            return {};
        }
        std::uint32_t prims = 0;
        for (const CameraScene::Drawable& drawable : scene_->drawables) {
            prims = std::max(prims, drawable.prim + 1);
        }
        surfaces_.assign(prims, Surface{});
        for (const CameraScene::Drawable& drawable : scene_->drawables) {
            Surface& surface = surfaces_[drawable.prim];
            // The node whose prim holds this one (the longest path), else one on its body.
            const Node* best = nullptr;
            for (const Node& node : nodes) {
                const bool under = !drawable.path.empty() && drawable.path.starts_with(node.path) &&
                                   (drawable.path.size() == node.path.size() ||
                                    drawable.path[node.path.size()] == '/');
                if (under && (best == nullptr || node.path.size() > best->path.size())) {
                    best = &node;
                }
            }
            if (best == nullptr && drawable.body != kNoIndex) {
                for (const Node& node : nodes) {
                    if (node.body.index == drawable.body && (best == nullptr || node.onLink)) {
                        best = &node;
                    }
                }
            }
            if (best != nullptr) {
                surface.temperature = best->temperature;
                surface.emissivity = best->emissivity;
                continue;
            }
            if (drawable.body != kNoIndex && drawable.body < raw.bodies.names.size()) {
                const std::string& body = raw.bodies.names[drawable.body];
                for (const std::string& prefix : people) {
                    if (body.starts_with(prefix)) {
                        surface.fixed = kSkinTemperature;
                        surface.emissivity = kSkinEmissivity;
                    }
                }
            }
        }
        return {};
    }

    /// The temperature image and its palette, from the instance ids of the frame just drawn.
    void stepThermal(const StepContext& context, const SignalBus& bus) {
        // What each surface reads, once per frame: its own emission and the ambient surroundings
        // it reflects. The clear sky where nothing is hit.
        apparent_.assign(surfaces_.size(), ambient_);
        for (std::size_t p = 0; p < surfaces_.size(); ++p) {
            const Surface& surface = surfaces_[p];
            double t = surface.fixed > 0.0 ? surface.fixed : ambient_;
            if (surface.temperature.valid()) {
                const double node = bus.read(surface.temperature);
                if (std::isfinite(node) && node > 0.0) {
                    t = node;
                }
            }
            apparent_[p] = band_->apparent(t, surface.emissivity, ambient_);
        }
        const double sky = clearSkyTemperature(ambient_);
        const std::uint32_t w = camera_.width;
        const std::uint32_t h = camera_.height;
        temperatures_.resize(std::size_t{w} * h);
        tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
            random::Stream rng(context.seed, context.stepIndex, (sensor_ << 16U) + v);
            for (std::uint32_t u = 0; u < w; ++u) {
                const std::size_t i = (std::size_t{v} * w) + u;
                const std::uint32_t id =
                    i < images_.instance.size() ? images_.instance[i] : gpu::kNoInstance;
                const double t = id < apparent_.size() ? apparent_[id] : sky;
                temperatures_[i] = static_cast<float>(t + (netd_ * rng.normal()));
            }
        });
        double lo = displayMin_;
        double hi = displayMax_;
        if (!(hi > lo)) {
            // Each frame's range, but never under kThermalSpan, so a scene all at one
            // temperature shows as one colour, not as its noise stretched over the palette.
            constexpr double kThermalSpan = 4.0;  // K, a display choice
            const auto [a, b] = std::ranges::minmax(temperatures_);
            lo = a;
            hi = b;
            if (hi - lo < kThermalSpan) {
                const double mid = (lo + hi) / 2.0;
                lo = mid - kThermalSpan / 2.0;
                hi = mid + kThermalSpan / 2.0;
            }
        }
        images_.rgba.resize(temperatures_.size() * 4);
        for (std::size_t i = 0; i < temperatures_.size(); ++i) {
            const std::array<std::uint8_t, 4> c =
                paletteColour(palette_, (temperatures_[i] - lo) / (hi - lo));
            std::ranges::copy(c, images_.rgba.begin() + static_cast<std::ptrdiff_t>(i * 4));
        }
    }

    Pose bodyPose(const SignalBus& bus, std::uint32_t b) const {
        if (b >= poses_.size() || !poses_[b].valid()) {
            return b < rest_.size() ? rest_[b] : Pose{};
        }
        const std::span<const double> v = bus.readN(poses_[b]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

public:
    Status saveState(StateWriter& writer) const override {
        writer.put(model_ ? model_->exposureTime() : 0.0);
        return {};
    }

    Status restoreState(StateReader& reader) override {
        double exposure = 0.0;
        if (!reader.get(exposure)) {
            return Status::error(StatusCode::Internal, "camera " + name_ + ": truncated state");
        }
        if (model_) {
            model_->setExposureTime(exposure);
        }
        return {};
    }

private:
    std::shared_ptr<const CameraScene> scene_;
    std::vector<SignalId> hinges_;        // per drawable; invalid: rigid
    gpu::PinholeCamera camera_;           // the output's pinhole (before the sensor model)
    std::unique_ptr<CameraModel> model_;  // null when no part of the sensor model is on
    gpu::RasterImages rendered_;          // the render the sensor model remaps
    SignalId twist_, exposureOut_;
    double far_{100.0};
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::unique_ptr<gpu::RasterCamera> renderer_;
    std::vector<gpu::RasterInstance> instances_;
    gpu::RasterImages images_;
    SignalId image_, depth_, instance_;
    /// A fog field's optical depth along each pixel's ray up to its depth (the far clip for the
    /// sky), integrated once per kFieldBlock² pixels and shared across the block.
    void fieldDepth(const fields::Field& field, const Pose& sensor) {
        constexpr std::uint32_t kFieldBlock = 8;
        const std::uint32_t w = camera_.width;
        const std::uint32_t h = camera_.height;
        fieldDepth_.assign(static_cast<std::size_t>(w) * h, 0.0F);
        const std::array<double, 4> optical = multiply(sensor.orientation, kSensorFromOptical);
        const std::uint32_t rows = (h + kFieldBlock - 1) / kFieldBlock;
        const std::uint32_t columns = (w + kFieldBlock - 1) / kFieldBlock;
        tbb::parallel_for(std::uint32_t{0}, rows, [&](std::uint32_t by) {
            for (std::uint32_t bx = 0; bx < columns; ++bx) {
                const std::uint32_t u = std::min(w - 1, bx * kFieldBlock + kFieldBlock / 2);
                const std::uint32_t v = std::min(h - 1, by * kFieldBlock + kFieldBlock / 2);
                const float d = images_.depth[static_cast<std::size_t>(v) * w + u];
                const double z = std::isfinite(d) ? d : far_;
                // The pixel's ray in the optical frame (x right, y down, z forward), to depth z.
                const Vec3 ray = rotate(optical, {(u - camera_.cx) / camera_.fx * z,
                                                  (v - camera_.cy) / camera_.fy * z, z});
                const fields::Vec3 from = sensor.position;
                const auto tau = static_cast<float>(
                    field.integrate(from, {from[0] + ray[0], from[1] + ray[1], from[2] + ray[2]}));
                for (std::uint32_t y = by * kFieldBlock; y < std::min(h, (by + 1) * kFieldBlock);
                     ++y) {
                    for (std::uint32_t x = bx * kFieldBlock;
                         x < std::min(w, (bx + 1) * kFieldBlock); ++x) {
                        fieldDepth_[static_cast<std::size_t>(y) * w + x] = tau;
                    }
                }
            }
        });
    }

    bool weather_{false};
    SignalId extinction_, contamination_, fogField_;
    fields::FieldSignal fogFieldSignal_;
    std::vector<float> fieldDepth_;
    // The thermal camera.
    struct Surface {
        SignalId temperature;  // a thermal node's
        double fixed{0.0};     // K, a person's skin; 0: the ambient air
        double emissivity{0.9};
    };
    bool thermal_{false};
    double ambient_{0.0}, netd_{0.0}, displayMin_{0.0}, displayMax_{0.0};
    ThermalPalette palette_{ThermalPalette::WhiteHot};
    std::unique_ptr<ThermalBand> band_;
    std::vector<Surface> surfaces_;  // by scene prim index
    std::vector<double> apparent_;   // K, by scene prim index, this frame
    std::vector<float> temperatures_;
    SignalId temperatureOut_;
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
            case SensorType::Radar:
                modules.push_back(createRadar(world, s, groundTruth));
                break;
            case SensorType::Gnss:
            case SensorType::Magnetometer:
            case SensorType::Barometer:
                break;  // createGeoSensors, below
            case SensorType::Custom:
                break;  // a plugin implements it (RFC 0008 §3)
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
                                               std::string& error, gpu::RasterLook look) {
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
            const SceneMaterial& material = model.scene.materials[prim.material];
            drawable.color = material.rgba;
            const SurfaceLook surface = lookOf(material);
            drawable.metallic = surface.metallic;
            drawable.roughness = surface.roughness;
            drawable.emissive = surface.emissive;
            drawable.pattern = gpu::surfacePatternOf(material.name);
        }
        drawable.prim = p;
        drawable.path = prim.path;
        drawable.hinge = prim.hinge;
        drawable.hingeAxis = prim.hingeAxis;
        drawable.ground = prim.name == "ground";
        drawable.water = std::ranges::find(prim.apiSchemas, "WaterAPI_1") != prim.apiSchemas.end();
        if (drawable.water) {
            const std::vector<float>& points = model.scene.meshes[prim.mesh].positions;
            drawable.box = {1e30F, 1e30F, 1e30F, -1e30F, -1e30F, -1e30F};
            for (std::size_t i = 0; i + 2 < points.size(); i += 3) {
                for (std::size_t k = 0; k < 3; ++k) {
                    drawable.box.at(k) = std::min(drawable.box.at(k), points[i + k]);
                    drawable.box.at(k + 3) = std::max(drawable.box.at(k + 3), points[i + k]);
                }
            }
        }
        scene->drawables.push_back(drawable);
    }
    scene->raster = gpu::RasterScene::create(preferredDevice, meshes, error, look);
    return scene->raster ? scene : nullptr;
}

}  // namespace lagrange::sensors
