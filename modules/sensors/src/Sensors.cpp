#include "GeoSensors.hpp"
#include "kinds/KindModule.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/gpu/Realistic.hpp"
#include "lagrange/lights/Lamp.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/CameraModel.hpp"
#include "lagrange/sensors/Fisheye.hpp"
#include "lagrange/sensors/ImuNoise.hpp"
#include "lagrange/sensors/Kinds.hpp"
#include "lagrange/sensors/MovingWater.hpp"
#include "lagrange/sensors/Radar.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/sensors/ThermalCamera.hpp"
#include "lagrange/sensors/WeatherEffects.hpp"
#include "lagrange/services/Log.hpp"
#include "lagrange/thermal/Materials.hpp"
#include "lagrange/world/Leaves.hpp"
#include "lagrange/world/LidarLayout.hpp"
#include "lagrange/world/PoseMath.hpp"
#include "lagrange/world/TreeWood.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
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

/// The bodies of water whose surface moves (config "water_surface", with "surface_cells" and
/// "surface_cell": the hydrodynamics domain's patch, water.md), each with its inputs
/// water/<w>/surface and surface_origin (feedback, optional: still until they come), in the
/// order of world.instancesOf("WaterAPI_1").
struct MovingWaterInputs {
    std::vector<MovingWater> water;
    std::vector<SignalId> heights, origins;

    Status declare(const WorldView& world, const ModuleConfig& config, SignalDeclarer& d) {
        water.clear();
        heights.clear();
        origins.clear();
        if (!config.get<bool>("water_surface").value_or(false)) {
            return {};
        }
        const auto cells = config.get<std::int64_t>("surface_cells").value_or(0);
        const double cell = config.get<double>("surface_cell").value_or(0.0);
        if (cells < 2 || !(cell > 0.0)) {
            return {};
        }
        for (const SchemaInstance* w : world.instancesOf("WaterAPI_1")) {
            constexpr double kDefaultSize = 10.0;  // m, as the water domain reads it
            const Vec3 middle = world.raw().prims.poseInBody.at(w->prim.index).position;
            const double halfX = w->real("water:sizeX").value_or(kDefaultSize) / 2.0;
            const double halfY = w->real("water:sizeY").value_or(kDefaultSize) / 2.0;
            MovingWater m;
            m.level = middle[2];
            m.lo = {middle[0] - halfX, middle[1] - halfY};
            m.hi = {middle[0] + halfX, middle[1] + halfY};
            m.cell = cell;
            water.push_back(m);
            const std::string base =
                "water/" + world.raw().prims.instanceNames.at(w->prim.index) + "/";
            SignalSpec surface =
                spec(base + "surface", SignalType::ScalarArray, units::m, "world_local");
            surface.arrayLength = static_cast<std::uint32_t>(cells * cells);
            surface.optional = surface.feedback = true;
            SignalSpec origin = surface;
            origin.name = base + "surface_origin";
            origin.arrayLength = 2;
            origin.defaultValue = std::numeric_limits<double>::quiet_NaN();
            heights.push_back(d.input(surface));
            origins.push_back(d.input(origin));
        }
        return {};
    }
    /// Reads this step's surfaces; false when none moves.
    bool read(const SignalBus& bus) {
        bool any = false;
        for (std::size_t k = 0; k < water.size(); ++k) {
            MovingWater& m = water[k];
            m.heights = bus.readN(heights[k]);
            const std::span<const double> corner = bus.readN(origins[k]);
            m.cells = static_cast<std::uint32_t>(std::lround(std::sqrt(m.heights.size())));
            m.corner = corner.size() == 2 ? std::array<double, 2>{corner[0], corner[1]}
                                          : std::array<double, 2>{std::nan(""), std::nan("")};
            any = m.prepare() || any;
        }
        return any;
    }
};

double setting(const World::Sensors& sensors, std::uint32_t sensor, const char* key,
               double fallback) {
    const auto& map = sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

/// The most beams one lidar scan takes: 2^22, sixteen times an Ouster OS2-128's 2048 × 128.
/// Each beam holds several arrays on the bus and on the GPU.
constexpr double kMaxLidarBeams = 4'194'304.0;

/// A sensor's period: exact when it is a whole number of ns, else on the kinds' grid (a lidar at
/// 15 Hz, 66'666'666.7 ns, was refused: base tick below 1 µs).
Duration periodOf(double rateHz) {
    return kinds::kindPeriod(rateHz);
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
        // Beam counts as the file gives them, before LidarLayout casts them: a negative count
        // cast to 4294967295 columns, and 1e8 columns asked for gigabytes (std::bad_alloc).
        for (const char* key : {"samples", "vertical_samples"}) {
            const auto it = sensors.settings[sensor_].find(key);
            if (it != sensors.settings[sensor_].end() &&
                !(it->second >= 1.0 && it->second <= kMaxLidarBeams)) {
                return Status::error(StatusCode::InvalidConfig,
                                     "lidar " + name_ + ": " + key + " must be from 1 to " +
                                         std::to_string(static_cast<long>(kMaxLidarBeams)));
            }
        }
        layout_ = LidarLayout::from(sensors.settings[sensor_]);
        if (static_cast<double>(layout_.horizontal) * layout_.vertical > kMaxLidarBeams) {
            return Status::error(StatusCode::InvalidConfig,
                                 "lidar " + name_ + ": more than " +
                                     std::to_string(static_cast<long>(kMaxLidarBeams)) +
                                     " beams in a scan");
        }
        samples_ = layout_.beams();
        minRange_ = setting(sensors, sensor_, "min_range", 0.1);
        maxRange_ = setting(sensors, sensor_, "max_range", 30.0);
        noise_ = setting(sensors, sensor_, "range_noise", 0.0);
        distort_ = setting(sensors, sensor_, "motion_distortion", 1.0) != 0.0 && !groundTruth_;
        rateHz_ = sensors.rateHz[sensor_];
        if (samples_ == 0) {
            return Status::error(StatusCode::InvalidConfig, "lidar " + name_ + " has no samples");
        }
        if (std::string problem = patternProblem(sensors.settings[sensor_]); !problem.empty()) {
            return Status::error(StatusCode::InvalidConfig, "lidar " + name_ + ": " + problem);
        }
        if (Status ok = declarePattern(d); !ok.ok()) {
            return ok;
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
        // The fires' smoke (config "smoke": a fire domain runs, RFC 0022 §4): its field's σ at
        // 633 nm, scaled to the lidar's wavelength, along each beam and at the sensor.
        smoke_ = config.get<bool>("smoke").value_or(false) && !groundTruth_;
        if (smoke_) {
            SignalSpec field = spec("env/smoke/extinction", SignalType::Field3D, units::per_m);
            field.feedback = true;
            field.optional = true;
            smokeField_ = d.input(field);
            smokeScale_ = smokeBandScale(setting(sensors, sensor_, "wavelength", 905e-9));
            if (!weather_) {  // the clutter's settings, as in weather
                clutter_.pulseLength =
                    setting(sensors, sensor_, "pulse_length", clutter_.pulseLength);
                clutter_.beamDiameter =
                    setting(sensors, sensor_, "beam_diameter", clutter_.beamDiameter);
                clutter_.reflectivity =
                    setting(sensors, sensor_, "max_range_reflectivity", clutter_.reflectivity);
            }
        }
        // Plants cut by propellers (ground.md "Cutting the plants"): the vegetation domain's map,
        // optional, so a run without it (or with vegetation off) sees them uncut.
        canopyCut_ = SignalId{};
        if (!groundTruth_ && std::ranges::any_of(raw.colliders.material, [&](std::uint32_t m) {
                return m != kNoMaterial && raw.surfaceMaterials.at(m).canopyHeight > 0.0;
            })) {
            SignalSpec cut = spec(std::string(kCanopyCutSignal), SignalType::ScalarArray, units::m);
            cut.arrayLength = kCanopyCutLength;
            cut.optional = true;
            cut.feedback = true;
            canopyCut_ = d.input(cut);
        }
        // Trees (ground.md "Trees"): crowns are turbid volumes at their sway, without the cells
        // the propellers cut. Both inputs optional: without the vegetation domain, still and whole.
        trees_.clear();
        treeSway_.clear();
        treeCut_ = SignalId{};
        branchCut_ = SignalId{};
        flyingIn_ = SignalId{};
        if (!groundTruth_) {
            std::string treeError;
            trees_ = treesOf(raw, treeError);
            wood_.build(trees_);
            for (const TreeSpec& t : trees_) {
                SignalSpec sway =
                    spec("tree/" + t.name + "/sway", SignalType::Vec3, units::m, "world_local");
                sway.optional = true;
                sway.feedback = true;
                sway.defaultValue = 0.0;
                treeSway_.push_back(d.input(sway));
            }
            if (!trees_.empty()) {
                SignalSpec cut =
                    spec(std::string(kTreeCutSignal), SignalType::ScalarArray, units::one);
                cut.arrayLength = kTreeCutLength;
                cut.optional = true;
                cut.feedback = true;
                treeCut_ = d.input(cut);
                surfaceMaterialPreset("shrub", leaves_);
                // The wood in the crowns (lagrange/world/TreeWood.hpp), less what broke off.
                SignalSpec broken =
                    spec(std::string(kBranchCutSignal), SignalType::ScalarArray, units::one);
                broken.arrayLength = kBranchCutLength;
                broken.optional = true;
                broken.feedback = true;
                branchCut_ = d.input(broken);
                surfaceMaterialPreset("bark", bark_);
                SignalSpec flying = spec(std::string(kLeavesSignal), SignalType::ScalarArray,
                                         units::one, "world_local");
                flying.arrayLength = kLeavesLength;
                flying.optional = true;
                flying.feedback = true;
                flyingIn_ = d.input(flying);
            }
            sways_.assign(trees_.size(), {});
        }
        // The water's moving surface (water.md): beams meet it at η, not at the still level.
        if (!groundTruth_) {
            return movingWater_.declare(world, config, d);
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
            if (shape.material != nullptr && shape.material->canopyHeight > 0.0 &&
                (shape.type == ShapeType::Box || shape.type == ShapeType::Plane)) {
                canopyVolumes_.push_back(static_cast<std::uint32_t>(shapes_.size()));
            }
            shapes_.push_back(shape);
        }
        // Water is no collider, but a lidar sees its surface: a mirror (or a box of depth below it,
        // from under water). What enters it is absorbed: no return from the bottom.
        waterMaterials_.clear();
        firstWaterShape_ = shapes_.size();
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
        if (layout_.timeVarying()) {
            aimRosette(0, bus);
        }
        bus.writeN(ranges_, buffer_);
        bus.writeN(intensities_, intensity_);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (layout_.timeVarying()) {
            aimRosette(context.stepIndex, bus);
        }
        if (groundTruth_) {
            bus.writeN(ranges_, buffer_);  // no rendering: no returns (+inf from init)
            bus.writeN(intensities_, intensity_);
            return {};
        }
        if (canopyCut_.valid()) {
            cuts_.load(bus.readN(canopyCut_));
        }
        if (treeCut_.valid()) {
            treeCuts_.load(bus.readN(treeCut_));
        }
        if (branchCut_.valid()) {
            wood_.load(bus.readN(branchCut_));
        }
        if (flyingIn_.valid()) {
            flying_.load(bus.readN(flyingIn_));
        }
        for (std::size_t k = 0; k < treeSway_.size(); ++k) {
            const std::span<const double> w = bus.readN(treeSway_[k]);
            sways_[k] =
                w.size() == 3 ? std::array<double, 3>{w[0], w[1], w[2]} : std::array<double, 3>{};
        }
        crownIndex_.build(trees_, sways_);
        if (!wood_.empty()) {
            // The wood's broad phase: a sphere round each tree's skeleton (its top-level
            // subtrees' spheres together), at the tree's sway.
            woodSpheres_.resize(trees_.size());
            for (std::size_t k = 0; k < trees_.size(); ++k) {
                woodSpheres_[k] = k < wood_.trees()
                                      ? woodBound(trees_[k], wood_.skeleton(k), sways_[k])
                                      : std::array<double, 4>{};
            }
            woodIndex_.buildSpheres(woodSpheres_);
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
        // The water's moving surface: the casts met its box at the still level; each beam is
        // moved onto η (or onto a crest in front of what it hit), in double on either tier.
        const bool movingWater = movingWater_.read(bus);
        if (movingWater) {
            tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
                const Pose& from = columnPose_[i % layout_.horizontal];
                const Vec3 dir = rotate(from.orientation, beamDirection_[i]);
                for (std::size_t w = 0; w < movingWater_.water.size(); ++w) {
                    const std::size_t shape = firstWaterShape_ + w;
                    bool on = false;
                    const double t = rayOntoWater(movingWater_.water[w], from.position, dir,
                                                  nearest_[i], hitShape_[i] == shape, &on);
                    if (on) {
                        nearest_[i] = t;
                        hitShape_[i] = static_cast<std::uint32_t>(shape);
                    }
                }
            });
        }
        // What each hit returns, from its surface; a surface too faint for the lidar drops out.
        tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
            intensity_[i] = 0.0;
            const Pose& from = columnPose_[i % layout_.horizontal];
            const Vec3 dir = rotate(from.orientation, beamDirection_[i]);
            // Reeds and shrubs (ShapeCaster.hpp canopyVolumeSpan): a beam can stop in a canopy
            // volume before its hit, or with no hit at all; one draw per canopy crossed, in
            // collider order, from the beam's own stream. The same post-step serves the GPU tier.
            const bool hit = hitShape_[i] != kNoShapeHit && std::isfinite(nearest_[i]);
            double stop = std::numeric_limits<double>::infinity();
            std::uint32_t stopShape = kNoShapeHit;
            bool leaves = false;
            bool wood = false;
            double woodCosine = 1.0;
            if (!canopyVolumes_.empty() || !trees_.empty()) {
                random::Stream rng(context.seed, context.stepIndex, kCanopyStream + i);
                const double reach = hit ? nearest_[i] : maxRange_;
                for (const std::uint32_t k : canopyVolumes_) {
                    const Shape& canopy = shapes_[k];
                    double enter = 0.0;
                    double leave = 0.0;
                    if (!canopyVolumeSpan(canopy.type, canopy.size, placed_[k], from.position, dir,
                                          canopyTallest(*canopy.material), reach, enter, leave)) {
                        continue;
                    }
                    // The shaped canopy (ShapeCaster.hpp "The canopy's shape"): plants of
                    // varying height, gaps, a ragged border, leaf area by height.
                    const double at = canopyShapedStop(
                        *canopy.material, canopy.type, canopy.size, placed_[k], from.position, dir,
                        enter, leave, rng.uniform(), cuts_.empty() ? nullptr : &cuts_, k);
                    if (at < stop) {
                        stop = at;
                        stopShape = k;
                    }
                }
                // Crowns (ShapeCaster.hpp crownStop): one draw per crown the beam crosses, in
                // tree order, among the broad phase's candidates (CrownIndex).
                thread_local std::vector<std::uint32_t> near;
                crownIndex_.candidates(from.position, dir, std::min(reach, stop), near);
                for (const std::uint32_t k : near) {
                    const double range = std::min(reach, stop);
                    if (!crownCrossed(trees_[k], from.position, dir, range, sways_[k])) {
                        continue;
                    }
                    const double at =
                        crownStop(trees_[k], k, from.position, dir, range, sways_[k],
                                  treeCuts_.empty() ? nullptr : &treeCuts_, rng.uniform());
                    if (at < stop) {
                        stop = at;
                        leaves = true;
                    }
                }
                // The wood in the crowns (TreeWood::ray): bark, nearer than the leaves; only the
                // trees whose wood's sphere the beam passes over (woodIndex_), in tree order.
                if (!wood_.empty()) {
                    woodIndex_.candidates(from.position, dir, std::min(reach, stop), near);
                } else {
                    near.clear();
                }
                for (const std::uint32_t k : near) {
                    WoodHit w;
                    if (wood_.ray(trees_[k], k, from.position, dir, std::min(reach, stop),
                                  sways_[k], w) &&
                        w.distance < stop) {
                        stop = w.distance;
                        wood = true;
                        leaves = false;
                        stopShape = kNoShapeHit;
                        woodCosine =
                            -(dir[0] * w.normal[0] + dir[1] * w.normal[1] + dir[2] * w.normal[2]);
                    }
                }
                // Flying leaves (ShapeCaster.hpp FlyingLeaves): the first the beam meets.
                if (!flying_.empty()) {
                    const double at = flying_.stop(from.position, dir, std::min(reach, stop));
                    if (at < stop) {
                        stop = at;
                        leaves = true;
                        wood = false;
                        stopShape = kNoShapeHit;
                    }
                }
            }
            if (wood) {
                nearest_[i] = stop;
                hitShape_[i] = kNoShapeHit;  // no collider: a branch in a crown
                const LidarReturn back =
                    lidarReturn(bark_, true, wavelength1550_, std::clamp(woodCosine, 0.0, 1.0),
                                stop, maxRange_, referenceReflectivity_);
                intensity_[i] = back.intensity;
                if (!back.detected) {
                    nearest_[i] = kNoHit;
                }
                return;
            }
            if (leaves) {
                nearest_[i] = stop;
                hitShape_[i] = kNoShapeHit;  // no collider: a leaf in a crown
                const LidarReturn back =
                    lidarReturn(leaves_, true, wavelength1550_, kCanopyLeafCosine, stop, maxRange_,
                                referenceReflectivity_);
                intensity_[i] = back.intensity;
                if (!back.detected) {
                    nearest_[i] = kNoHit;
                }
                return;
            }
            static const SurfaceMaterial kDefault{};
            if (stopShape != kNoShapeHit) {
                nearest_[i] = stop;
                hitShape_[i] = stopShape;
                const LidarReturn back =
                    lidarReturn(*shapes_[stopShape].material, true, wavelength1550_,
                                kCanopyLeafCosine, stop, maxRange_, referenceReflectivity_);
                intensity_[i] = back.intensity;
                if (!back.detected) {
                    nearest_[i] = kNoHit;
                }
                return;
            }
            if (!hit) {
                return;
            }
            const Shape& shape = shapes_[hitShape_[i]];
            const Vec3 at{from.position[0] + dir[0] * nearest_[i],
                          from.position[1] + dir[1] * nearest_[i],
                          from.position[2] + dir[2] * nearest_[i]};
            const std::size_t water = hitShape_[i] - firstWaterShape_;
            const Vec3 n = movingWater && hitShape_[i] >= firstWaterShape_ &&
                                   water < movingWater_.water.size() &&
                                   movingWater_.water[water].cells >= 2
                               ? movingWater_.water[water].normalAt(at[0], at[1])
                               : surfaceNormal(shape.type, shape.size, placed_[hitShape_[i]], at,
                                               shape.hull, shape.field);
            const double cosine = -(dir[0] * n[0] + dir[1] * n[1] + dir[2] * n[2]);
            // Grass: the return comes from the blades over the soil (ShapeCaster.hpp), with the
            // surface's own leaf area index. On the GPU tier too: the hit comes back from the ray
            // scan and is shortened here, on the CPU. Boxes and planes did it above, as volumes.
            if (shape.material != nullptr && shape.material->canopyHeight > 0.0 &&
                shape.type != ShapeType::Box && shape.type != ShapeType::Plane) {
                random::Stream rng(context.seed, context.stepIndex, kCanopyStream + i);
                nearest_[i] -= canopyShapedShortening(*shape.material, at[0], at[1], cosine,
                                                      nearest_[i], rng.uniform());
            }
            const LidarReturn back = lidarReturn(
                shape.material != nullptr ? *shape.material : kDefault, shape.material != nullptr,
                wavelength1550_, cosine, nearest_[i], maxRange_, referenceReflectivity_);
            intensity_[i] = back.intensity;
            if (!back.detected) {
                nearest_[i] = kNoHit;
            }
        });
        if (weather_ || smoke_) {
            // A fog field and the smoke: their optical depth along each beam up to the hit, and
            // their value at the sensor for the clutter near it (taken as the extinction at the
            // lidar's wavelength; smoke's backscatter as fog's, RFC 0022 §4).
            const std::shared_ptr<const fields::Field> fog =
                weather_ ? fogFieldSignal_.read(bus, fogField_) : nullptr;
            const std::shared_ptr<const fields::Field> smoke =
                smoke_ ? smokeFieldSignal_.read(bus, smokeField_) : nullptr;
            const fields::Vec3 at = sensor.position;
            const double local = (fog ? fog->sampleScalar(at) : 0.0) +
                                 (smoke ? smokeScale_ * smoke->sampleScalar(at) : 0.0);
            const SensorWeather w =
                weather_ ? SensorWeather{bus.read(extinction_), bus.read(fogExtinction_) + local,
                                         bus.read(rainRate_) + (kSnowClutter * bus.read(snowRate_)),
                                         bus.read(contamination_)}
                         : SensorWeather{0.0, local, 0.0, 0.0};
            tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
                // Its own stream, so that the range noise below is the same with or without.
                random::Stream rng(context.seed, context.stepIndex, kWeatherStream + i);
                const double u1 = rng.uniform();
                const double u2 = rng.uniform();
                const double u3 = rng.uniform();
                double depth = 0.0;
                if ((fog || smoke) && std::isfinite(nearest_[i])) {
                    const Pose& from = columnPose_[i % layout_.horizontal];
                    const Vec3 dir = rotate(from.orientation, beamDirection_[i]);
                    const double to = std::min(nearest_[i], maxRange_);
                    const Vec3& o = from.position;
                    const fields::Vec3 end{o[0] + dir[0] * to, o[1] + dir[1] * to,
                                           o[2] + dir[2] * to};
                    depth = (fog ? fog->integrate(o, end) : 0.0) +
                            (smoke ? smokeScale_ * smoke->integrate(o, end) : 0.0);
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
        // Range noise, from each beam's own stream (seed, step, beam): the same on any number
        // of threads. A Gaussian draw per beam was a millisecond of a 32 000-beam scan.
        tbb::parallel_for(std::uint32_t{0}, samples_, [&](std::uint32_t i) {
            double nearest = nearest_[i];
            if (nearest < minRange_ || nearest > maxRange_) {
                buffer_[i] = kNoHit;  // no valid return (sensor_msgs/LaserScan convention)
                intensity_[i] = 0.0;
                return;
            }
            if (noise_ > 0.0) {
                random::Stream rng(context.seed, context.stepIndex, i);
                nearest += noise_ * rng.normal();
            }
            buffer_[i] = std::clamp(nearest, minRange_, maxRange_);
            if (!std::isfinite(buffer_[i])) {
                intensity_[i] = 0.0;
            }
        });
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
    /// What is wrong with the pattern's own parameters, or empty (sensors QA F7: a channel at
    /// 2.5 rad, a rosette cone of 4 rad, a negative prism ratio or prisms at one speed were
    /// taken without a word).
    [[nodiscard]] std::string patternProblem(const LidarLayout::Settings& settings) const {
        constexpr double kRightAngle = std::numbers::pi / 2.0;
        for (std::size_t ring = 0; ring < layout_.ringElevations.size(); ++ring) {
            if (!(std::abs(layout_.ringElevations[ring]) <= kRightAngle)) {
                return "channel elevations must lie within ±π/2 rad (the channel table has " +
                       std::to_string(layout_.ringElevations[ring]) + ")";
            }
        }
        if (layout_.pattern != LidarLayout::Pattern::Rosette) {
            return {};
        }
        const auto given = [&settings](const char* key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        const double fov = given("rosette_fov", LidarLayout::kRosetteFov);
        if (!(fov > 0.0 && fov <= std::numbers::pi)) {
            return "rosette fov (the full cone) must be above 0 and at most π rad, not " +
                   std::to_string(fov);
        }
        if (!(given("rosette_prism_ratio", 1.0) >= 0.0)) {
            return "rosette prism_ratio (δ2 / δ1) must be at least 0";
        }
        const double s1 = layout_.rosette.speed1;
        const double s2 = layout_.rosette.speed2;
        if (!std::isfinite(s1) || !std::isfinite(s2) || s1 == s2) {
            return "rosette speed_1 and speed_2 must be numbers that differ (at one speed the "
                   "prisms turn together and the beam draws one circle, or stands still)";
        }
        return {};
    }

    /// The opt-in scan patterns (LidarLayout): a flash lidar fires all beams at once, so its scan
    /// has no motion distortion; a rosette's beams move on every scan and go out as
    /// sensor/<name>/directions (unit vectors in the sensor frame, x y z per beam) for whoever
    /// turns its ranges into points.
    Status declarePattern(SignalDeclarer& d) {
        if (layout_.simultaneous()) {
            distort_ = false;
            constexpr double kRightAngle = 1.5707963267948966;
            for (const double a :
                 {layout_.minAngle, layout_.maxAngle, layout_.minElevation, layout_.maxElevation}) {
                if (!(std::abs(a) < kRightAngle)) {
                    return Status::error(StatusCode::InvalidConfig,
                                         "flash lidar " + name_ +
                                             ": its angles must lie within ±π/2 (a focal plane)");
                }
            }
        }
        if (layout_.timeVarying()) {
            if (!(layout_.rosette.pointRate > 0.0)) {
                return Status::error(StatusCode::InvalidConfig,
                                     "rosette lidar " + name_ + ": rosette_point_rate must be > 0");
            }
            // The scan holds the last `samples` points fired, 1 / point_rate apart: the whole
            // period only when samples = point_rate / update_rate (sensors.md, Rosette).
            const double perScan = layout_.perPeriod(rateHz_);
            if (std::abs(perScan - samples_) >= 1.0) {
                services::log(
                    services::LogLevel::Warning, "sensors",
                    "rosette lidar " + name_ + ": " + std::to_string(samples_) +
                        " samples per scan, but it fires " + std::to_string(std::llround(perScan)) +
                        " points per scan at point_rate / update_rate; " +
                        (perScan > samples_
                             ? "each scan holds only the last " + std::to_string(samples_) +
                                   " points of its period"
                             : "its scans overlap, their first points fired before the last "
                               "scan's end") +
                        " (give samples = point_rate / update_rate for the real picture)");
            }
            SignalSpec out =
                spec("sensor/" + name_ + "/directions", SignalType::ScalarArray, units::one);
            out.arrayLength = 3 * samples_;
            directions_ = d.output(out);
        }
        return {};
    }

    /// A rosette's beams for scan `scan`, on the bus and for the casts (the GPU's are uploaded
    /// again).
    void aimRosette(std::uint64_t scan, SignalBus& bus) {
        directionValues_.resize(std::size_t{3} * samples_);
        for (std::uint32_t i = 0; i < samples_; ++i) {
            beamDirection_[i] = layout_.direction(i, scan);
            for (std::size_t k = 0; k < 3; ++k) {
                directionValues_[(std::size_t{3} * i) + k] = beamDirection_[i].at(k);
            }
        }
        bus.writeN(directions_, directionValues_);
        beamsMoved_ = scan_ != nullptr;
    }

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
            // A rotating lidar's columns spread over the period; a rosette's points come
            // 1 / point_rate apart, the last at the scan's end (sensors QA F3). A point fired
            // before the last scan's end (samples > point_rate / rate) takes its pose.
            const double f = std::max(layout_.firedFraction(c, rateHz_), 0.0);
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
    static constexpr std::uint32_t kCanopyStream = 0x2000'0000U;        // + beam
    static constexpr std::uint32_t kNoShapeHit = 0xFFFF'FFFFU;
    static constexpr std::uint32_t kMaxPasses = 6;
    static constexpr double kPastSurfaceM = 1e-4;
    SignalId directions_;                  // a rosette's beams, per scan
    std::vector<double> directionValues_;  // x y z per beam
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
    std::size_t firstWaterShape_{0};               // shapes_ from here on are the water's boxes
    MovingWaterInputs movingWater_;
    std::vector<std::uint32_t> hitShape_;
    std::vector<std::uint32_t> canopyVolumes_;  // shapes_ with a canopy volume (boxes, planes)
    SignalId canopyCut_;                        // the vegetation domain's cut map, when it runs
    CanopyCuts cuts_;
    std::vector<TreeSpec> trees_;  // crowns the beams cross (ground.md "Trees")
    CrownIndex crownIndex_;        // their broad phase, per scan
    CrownIndex woodIndex_;         // the wood's, per scan
    std::vector<std::array<double, 4>> woodSpheres_;
    std::vector<SignalId> treeSway_;            // tree/<n>/sway, the vegetation domain's
    std::vector<std::array<double, 3>> sways_;  // this scan's
    SignalId treeCut_;                          // vegetation/tree_cut
    TreeCuts treeCuts_;
    TreeWood wood_;           // the crowns' branches (lagrange/world/TreeWood.hpp)
    SignalId branchCut_;      // vegetation/branch_cut
    SurfaceMaterial bark_;    // a branch's return: the bark preset's
    SignalId flyingIn_;       // vegetation/leaves
    FlyingLeaves flying_;     // this scan's
    SurfaceMaterial leaves_;  // a leaf's return: the shrub preset's
    std::vector<double> intensity_;
    bool wavelength1550_{false};
    double referenceReflectivity_{0.1};
    bool weather_{false};
    LidarClutter clutter_;
    SignalId extinction_, fogExtinction_, rainRate_, snowRate_, contamination_, fogField_;
    bool smoke_{false};
    SignalId smokeField_;
    double smokeScale_{1.0};  // the smoke field's σ (633 nm) at the lidar's wavelength
    fields::FieldSignal smokeFieldSignal_;
    // Flakes return more false echoes than the same water as drops: taken as this much rain.
    // Illustrative: no published snow-to-rain clutter ratio was found (Rasshofer, Spies and Spies,
    // Adv. Radio Sci. 9, 2011, give snow's extinction, not its false echoes).
    static constexpr double kSnowClutter = 3.0;
    fields::FieldSignal fogFieldSignal_;
    LidarLayout layout_;
    std::uint32_t samples_{0};  // beams, all rings
    double minRange_{0.0}, maxRange_{0.0}, noise_{0.0};
    double rateHz_{10.0};  // scans per second
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

class Camera final : public SensorModule, public SplitStep {
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
        // A fisheye (FisheyeLens.hpp) sees up to all round; a pinhole stops short of 180°.
        const bool fisheye = FisheyeLens::isFisheye(sensors.settings[sensor_]);
        constexpr double kMaxFisheyeFov = 2.0 * FisheyeLens::kPi;
        if (width < 1.0 || height < 1.0 || width > 16384.0 || height > 16384.0 || fov <= 0.0 ||
            fov >= (fisheye ? kMaxFisheyeFov : 3.1)) {
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
        if (fisheye) {
            if (Status ok = declareFisheye(sensors, model); !ok.ok()) {
                return ok;
            }
        } else {
            model_ = std::make_unique<CameraModel>(model);
            if (!model_->active()) {
                model_.reset();
            }
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
            // The run's lamps and the sky (RFC 0021 §4, §6): what lights the realistic look.
            std::string error;
            std::vector<lights::LampSpec> lamps = lights::lampsOf(world, error);
            if (!error.empty()) {
                return Status::error(StatusCode::InvalidConfig, "camera " + name_ + ": " + error);
            }
            lamps_.clear();
            for (lights::LampSpec& lamp : lamps) {
                SeenLamp& seen = lamps_.emplace_back();
                if (lamp.body != kWorldBody) {
                    watch(lamp.body.index);
                }
                SignalSpec intensity =
                    spec("lamp/" + lamp.name + "/intensity", SignalType::Scalar, units::cd);
                intensity.feedback = true;  // last step's light: no ordering asked of the lamps
                intensity.optional = true;
                intensity.defaultValue = 0.0;
                seen.intensity = d.input(intensity);
                if (lamp.colorTemperature > 0.0) {
                    seen.color = gpu::colorOfTemperature(lamp.colorTemperature);
                } else {
                    // lamp:color, normalised to luminance 1 (Rec. 709 weights, sRGB's Y row).
                    const std::array<double, 3>& c = lamp.color;
                    const double y = 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
                    for (std::size_t k = 0; k < 3; ++k) {
                        seen.color.at(k) = y > 0.0 ? static_cast<float>(c.at(k) / y) : 0.0F;
                    }
                }
                seen.spec = std::move(lamp);
            }
            SignalSpec sky = spec("env/sky_illuminance", SignalType::Scalar, units::lx);
            sky.feedback = true;
            sky.optional = true;
            sky.defaultValue = std::numeric_limits<double>::quiet_NaN();
            skyIlluminance_ = d.input(sky);
            // The rotors (RotorAPI_1), whose outwash bends the grass the camera sees
            // (gpu/GroundAir.hpp): rotor_aero's thrust, induced velocity and axis, last step's.
            rotors_.clear();
            const World::Joints& joints = raw.joints;
            const World::Bodies& bodies = raw.bodies;
            for (const SchemaInstance* s : world.instancesOf("RotorAPI_1")) {
                const auto j = static_cast<std::size_t>(std::ranges::find(joints.prim, s->prim) -
                                                        joints.prim.begin());
                const auto radius = s->real("aero:rotor:radius");
                if (j >= joints.prim.size() || !radius || *radius <= 0.0) {
                    continue;
                }
                SeenRotor& r = rotors_.emplace_back();
                r.body = joints.child[j].index;
                r.radius = *radius;
                std::size_t root = r.body;
                for (std::size_t guard = 0; guard < bodies.parent.size(); ++guard) {
                    const BodyId p = bodies.parent[root];
                    if (p == kWorldBody) {
                        break;
                    }
                    root = p.index;
                }
                r.group = root;
                watch(r.body);
                const std::string base =
                    "rotor/" + std::string(world.signalInstanceName(s->prim)) + "/";
                const auto optional = [&](SignalSpec in) {
                    in.feedback = true;
                    in.optional = true;
                    return d.input(in);
                };
                r.thrust = optional(spec(base + "thrust", SignalType::Scalar, units::N));
                r.induced =
                    optional(spec(base + "induced_velocity", SignalType::Scalar, units::m_per_s));
                r.axis = optional(
                    spec(base + "thrust_axis", SignalType::Vec3, units::one, "world_local"));
            }
            // The trees (ground.md "Trees"), drawn as trees by the realistic look: their sway,
            // cut crown cells and flying leaves, last step's; without the vegetation domain,
            // still and whole.
            std::string treeError;
            seenTrees_ = treesOf(raw, treeError);
            treeSway_.clear();
            treeCut_ = SignalId{};
            leaves_ = SignalId{};
            drawnTrees_.clear();
            treeIds_.clear();
            for (const TreeSpec& t : seenTrees_) {
                // Its crown's instance id: the tree prim's scene index.
                const std::string& path = t.prim.index < raw.prims.paths.size()
                                              ? raw.prims.paths[t.prim.index]
                                              : std::string();
                const auto at = std::ranges::find(scene_->primPaths, path);
                treeIds_.push_back(
                    at == scene_->primPaths.end() || path.empty()
                        ? gpu::kNoInstance
                        : static_cast<std::uint32_t>(at - scene_->primPaths.begin()));
                SignalSpec sway =
                    spec("tree/" + t.name + "/sway", SignalType::Vec3, units::m, "world_local");
                sway.optional = true;
                sway.feedback = true;
                sway.defaultValue = 0.0;
                treeSway_.push_back(d.input(sway));
                gpu::RealisticTree& drawn = drawnTrees_.emplace_back();
                drawn.foot = t.pose.position;
                drawn.rotation = {static_cast<float>(t.pose.orientation[0]),
                                  static_cast<float>(t.pose.orientation[1]),
                                  static_cast<float>(t.pose.orientation[2]),
                                  static_cast<float>(t.pose.orientation[3])};
                drawn.height = t.height;
                drawn.crownRadius = t.crownRadius;
                drawn.crownBase = t.crownBase;
                drawn.trunkDiameter = t.trunkDiameter;
                // The viewer's seed (App.cpp followTrees): the same tree in both.
                std::uint32_t seed = 2166136261U;
                for (const char c : t.name) {
                    seed = (seed ^ static_cast<unsigned char>(c)) * 16777619U;
                }
                drawn.seed = seed & 0xffffffU;
                drawn.id = treeIds_.back();
            }
            if (!seenTrees_.empty()) {
                SignalSpec cut =
                    spec(std::string(kTreeCutSignal), SignalType::ScalarArray, units::one);
                cut.arrayLength = kTreeCutLength;
                cut.optional = true;
                cut.feedback = true;
                treeCut_ = d.input(cut);
                SignalSpec leaves =
                    spec(std::string(kLeavesSignal), SignalType::ScalarArray, units::one);
                leaves.arrayLength = kLeavesLength;
                leaves.optional = true;
                leaves.feedback = true;
                leaves_ = d.input(leaves);
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
        // The water's moving surface (water.md): the depth meets it at η.
        if (!groundTruth_) {
            if (Status ok = movingWater_.declare(world, config, d); !ok.ok()) {
                return ok;
            }
        }
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
        // The fires' smoke (config "smoke", RFC 0022 §6): the field is σ in the visible, as the
        // veil takes it. A thermal camera's band is not measured for soot (TODO(verify)).
        smoke_ = config.get<bool>("smoke").value_or(false) && !groundTruth_ && !thermal_;
        if (smoke_) {
            SignalSpec field = spec("env/smoke/extinction", SignalType::Field3D, units::per_m);
            field.feedback = true;
            field.optional = true;
            smokeField_ = d.input(field);
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
            if (fisheye_) {
                faceRenderers_.clear();
                for (const FisheyeFace& face : fisheye_->faces()) {
                    faceRenderers_.push_back(std::make_unique<gpu::RasterCamera>(
                        scene_->raster, face.camera.width, face.camera.height));
                }
                faceImages_.resize(fisheye_->faces().size());
            } else {
                renderer_ = std::make_unique<gpu::RasterCamera>(scene_->raster, render.width,
                                                                render.height);
            }
            instances_.resize(scene_->drawables.size());
            // A thermal camera paints its own colour from the instance ids. A fisheye has no
            // single renderer, only its faces (they are resampled on the CPU, so the sensor
            // model stays on the CPU for it).
            for (const auto& face : faceRenderers_) {
                face->setColour(!thermal_);
            }
            if (renderer_) {
                renderer_->setColour(!thermal_);
            }
            // The sensor model on the GPU, beside the render (CameraModel::apply is the CPU
            // reference; LAGRANGE_CAMERA_MODEL=cpu keeps it).
            const char* where = std::getenv("LAGRANGE_CAMERA_MODEL");
            std::string error;
            gpuModel_ = renderer_ && model_ &&
                        (where == nullptr || std::string_view(where) != "cpu") &&
                        renderer_->setEffects(model_->gpuEffects(), error);
            // The thermal image on the GPU, after the render (stepThermal is the CPU
            // reference; LAGRANGE_THERMAL_CAMERA=cpu keeps it). Not with a sensor model.
            const char* thermalWhere = std::getenv("LAGRANGE_THERMAL_CAMERA");
            if (thermal_ && renderer_ && !model_ &&
                (thermalWhere == nullptr || std::string_view(thermalWhere) != "cpu")) {
                gpu::ThermalImageSetup setup;
                setup.prims = static_cast<std::uint32_t>(surfaces_.size());
                setup.palette = static_cast<std::uint32_t>(palette_);
                setup.netd = netd_;
                setup.displayMin = displayMin_;
                setup.displayMax = displayMax_;
                setup.span = kThermalSpan;
                gpuThermal_ = renderer_->setThermal(setup, error);
            }
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
        sensorPose_ = sensor;
        frameContext_ = context;
        light(bus, context.t.seconds());
        // The realistic look draws the water's moving surface (the first body of water's).
        if (renderer_ && !movingWater_.water.empty()) {
            gpu::RealisticWaterSurface surface;
            if (movingWater_.read(bus) && movingWater_.water.front().cells >= 2) {
                const MovingWater& w = movingWater_.water.front();
                waterHeights_.assign(w.heights.begin(), w.heights.end());
                surface.cells = w.cells;
                surface.cell = w.cell;
                surface.first = {w.corner[0] + w.cell / 2.0, w.corner[1] + w.cell / 2.0};
                surface.heights = waterHeights_;
            }
            renderer_->setWaterSurface(surface);
        }
        if (fisheye_) {
            // Each face of the cube from the camera's place, turned to look through it.
            for (std::size_t f = 0; f < faceRenderers_.size(); ++f) {
                gpu::PinholeCamera face = fisheye_->faces()[f].camera;
                face.near = camera_.near;
                face.position = camera_.position;
                face.orientation = multiply(camera_.orientation, fisheye_->faces()[f].rotation);
                faceRenderers_[f]->render(instances_, face, faceImages_[f], context.t.seconds());
            }
            fisheye_->apply(faceImages_, images_);
            return finishFrame(bus);
        }
        // The render is submitted here and waited for in finishStep (SplitStep): the scheduler
        // runs the tick's other modules meanwhile, and nothing reads the images before then.
        if (model_) {
            gpu::PinholeCamera render = model_->renderCamera();
            render.near = camera_.near;
            render.position = camera_.position;
            render.orientation = camera_.orientation;
            omega_ = {};
            if (twist_.valid()) {
                const std::span<const double> t = bus.readN(twist_);
                omega_ = rotate(conjugate(camera_.orientation), {t[3], t[4], t[5]});
            }
            if (exposureOut_.valid()) {
                bus.write(exposureOut_, model_->exposureTime());
            }
            if (gpuModel_) {
                const gpu::CameraEffectsFrame frame =
                    model_->gpuFrame(omega_, context.seed, context.stepIndex, sensor_ << 16U);
                renderer_->begin(instances_, render, &frame, context.t.seconds());
            } else {
                renderer_->begin(instances_, render, nullptr, context.t.seconds());
            }
        } else if (gpuThermal_) {
            thermalApparent(bus);
            gpu::ThermalImageFrame frame;
            frame.apparent = apparent_;
            frame.sky = clearSkyTemperature(ambient_);
            frame.seed = context.seed;
            frame.stepIndex = context.stepIndex;
            frame.entity = sensor_ << 16U;
            renderer_->beginThermal(instances_, camera_, frame, context.t.seconds());
        } else {
            renderer_->begin(instances_, camera_, nullptr, context.t.seconds());
        }
        if (!splitAllowed_) {
            return finishStep(bus);
        }
        return {};
    }

    void allowSplit(bool allowed) override { splitAllowed_ = allowed; }

    [[nodiscard]] bool stepPending() const override { return renderer_ && renderer_->pending(); }

    Status finishStep(SignalBus& bus) override {
        const StepContext& context = frameContext_;
        if (model_ && gpuModel_) {
            renderer_->finish(images_, &signal_);
            model_->afterGpuFrame(signal_);
        } else if (model_) {
            renderer_->finish(rendered_, nullptr);
            model_->apply(rendered_, omega_, context.seed, context.stepIndex, sensor_ << 16U,
                          images_);
        } else if (gpuThermal_) {
            renderer_->finish(images_, &temperatures_);
        } else {
            renderer_->finish(images_, nullptr);
        }
        return finishFrame(bus);
    }

private:
    /// What follows the render: the far clip, the weather's veil, the thermal image, and the
    /// images out.
    Status finishFrame(SignalBus& bus) {
        if (!fisheye_ && movingWater_.read(bus)) {
            waterDepth();
        }
        // The crowns, as the lidar sees them (not yet through a fisheye's faces, nor in a
        // thermal camera's image).
        if (!fisheye_ && !thermal_ && !seenTrees_.empty()) {
            crownDepth();
        }
        for (float& depth : images_.depth) {
            if (depth > far_) {
                depth = std::numeric_limits<float>::infinity();
                // The colour keeps whatever was drawn: a far clip limits the range, not the view.
            }
        }
        if (weather_ || smoke_) {
            const std::shared_ptr<const fields::Field> fog =
                weather_ ? fogFieldSignal_.read(bus, fogField_) : nullptr;
            const std::shared_ptr<const fields::Field> smoke =
                smoke_ ? smokeFieldSignal_.read(bus, smokeField_) : nullptr;
            if (fog || smoke) {
                fieldDepth(fog.get(), smoke.get(), sensorPose_);
            } else {
                fieldDepth_.clear();
            }
            applyCameraWeather(images_.rgba, images_.depth, weather_ ? bus.read(extinction_) : 0.0,
                               weather_ ? bus.read(contamination_) : 0.0, fieldDepth_);
        }
        if (thermal_ && !gpuThermal_) {
            stepThermal(frameContext_, bus);
        }
        publish(bus);
        return {};
    }

    /// A fisheye camera (FisheyeLens.hpp, Fisheye.hpp): its focal length from the angle across
    /// its width, and the cube faces to draw. The rest of the sensor model is not combined with
    /// it yet (sensors.md, "Fisheye cameras (as built)").
    Status declareFisheye(const World::Sensors& sensors, const CameraModelConfig& model) {
        const FisheyeLens lens = FisheyeLens::from(sensors.settings[sensor_]);
        const double fov = setting(sensors, sensor_, "horizontal_fov", 1.047);
        if (!lens.monotonic() || !(lens.distorted(fov / 2.0) > 0.0) || !(lens.cutoff > 0.0)) {
            return Status::error(
                StatusCode::InvalidConfig,
                "fisheye camera " + name_ + ": its k1..k4 fold the image back within the cutoff");
        }
        if (model.vignetting > 0.0 || model.rollingShutterTime > 0.0 || model.exposure) {
            return Status::error(StatusCode::InvalidConfig,
                                 "fisheye camera " + name_ +
                                     ": vignetting, rolling shutter and exposure are not "
                                     "supported with a fisheye yet");
        }
        camera_.fx = lens.focal(camera_.width, fov);
        camera_.fy = camera_.fx;
        FisheyeConfig config;
        config.width = camera_.width;
        config.height = camera_.height;
        config.fx = camera_.fx;
        config.fy = camera_.fy;
        config.cx = camera_.cx;
        config.cy = camera_.cy;
        config.lens = lens;
        fisheye_ = std::make_unique<FisheyeRemap>(config);
        return {};
    }

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

    /// What each surface reads, once per frame: its own emission and the ambient surroundings
    /// it reflects (the clear sky where nothing is hit is the caller's).
    void thermalApparent(const SignalBus& bus) {
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
            apparent_[p] = apparentOf(t, surface.emissivity);
        }
    }

    /// The temperature image and its palette, from the instance ids of the frame just drawn.
    void stepThermal(const StepContext& context, const SignalBus& bus) {
        thermalApparent(bus);
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
        tbb::parallel_for(std::size_t{0}, temperatures_.size(), [&](std::size_t i) {
            const std::array<std::uint8_t, 4> c =
                paletteColour(palette_, (temperatures_[i] - lo) / (hi - lo));
            std::ranges::copy(c, images_.rgba.begin() + static_cast<std::ptrdiff_t>(i * 4));
        });
    }

    /// The lamps and the sky of this frame, to every renderer of the camera (RFC 0021 §4, §6).
    void light(const SignalBus& bus, double t) {
        drawnLamps_.clear();
        for (const SeenLamp& seen : lamps_) {
            const double intensity = bus.read(seen.intensity);
            if (!(intensity > 0.0)) {
                continue;
            }
            const lights::LampSpec& spec = seen.spec;
            const Pose pose = spec.body == kWorldBody
                                  ? spec.poseInBody
                                  : compose(bodyPose(bus, spec.body.index), spec.poseInBody);
            gpu::RealisticLamp& lamp = drawnLamps_.emplace_back();
            lamp.position = pose.position;
            lamp.axis = rotate(pose.orientation, lights::beamAxis(spec, t));
            lamp.intensity = intensity;
            lamp.beamAngle = spec.beamAngle;
            lamp.fieldAngle = spec.fieldAngle;
            lamp.point = spec.kind == lights::LampKind::Point;
            lamp.color = seen.color;
        }
        const double sky = skyIlluminance_.valid() ? bus.read(skyIlluminance_)
                                                   : std::numeric_limits<double>::quiet_NaN();
        if (renderer_) {
            renderer_->setLighting(drawnLamps_, sky);
        }
        for (const auto& face : faceRenderers_) {
            face->setLighting(drawnLamps_, sky);
        }
        drawnRotors_.clear();
        for (const SeenRotor& seen : rotors_) {
            if (!seen.thrust.valid() || !seen.induced.valid() || !seen.axis.valid()) {
                continue;
            }
            const std::span<const double> a = bus.readN(seen.axis);
            gpu::AirRotor& r = drawnRotors_.emplace_back();
            r.hub = bodyPose(bus, seen.body).position;
            r.thrustAxis = {a[0], a[1], a[2]};
            r.radius = seen.radius;
            r.induced = bus.read(seen.induced);
            r.thrust = bus.read(seen.thrust);
            r.group = seen.group;
        }
        if (renderer_) {
            renderer_->setAir(drawnRotors_);
        }
        for (const auto& face : faceRenderers_) {
            face->setAir(drawnRotors_);
        }
        trees(bus);
        // The render is brightened as the dark asks (gpu::realisticAdaptation): the sensor
        // model takes it out again, so its electrons stay those of the scene's light.
        if (model_ && renderer_ && scene_->raster->realistic()) {
            model_->setRenderGain(gpu::realisticAdaptation(sky));
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
    std::vector<SignalId> hinges_;  // per drawable; invalid: rigid
    /// A lamp of the run as the camera draws it (RFC 0021 §4).
    struct SeenLamp {
        lights::LampSpec spec;
        SignalId intensity;  // lamp/<n>/intensity, cd
        std::array<float, 3> color{1.0F, 1.0F, 1.0F};
    };
    std::vector<SeenLamp> lamps_;
    std::vector<gpu::RealisticLamp> drawnLamps_;  // this frame's
    /// A rotor whose outwash the camera draws in the grass (gpu/GroundAir.hpp).
    struct SeenRotor {
        std::uint32_t body{0};  // its link
        double radius{0.0};     // m
        std::size_t group{0};   // its robot's root body
        SignalId thrust, induced, axis;
    };
    std::vector<SeenRotor> rotors_;
    std::vector<gpu::AirRotor> drawnRotors_;  // this frame's
    /// The trees of the run as the realistic look draws them (ground.md "Trees").
    std::vector<TreeSpec> seenTrees_;
    std::vector<SignalId> treeSway_;  // tree/<n>/sway
    SignalId treeCut_, leaves_;       // kTreeCutSignal, kLeavesSignal
    std::vector<gpu::RealisticTree> drawnTrees_;
    std::vector<gpu::RealisticTreeCut> drawnCuts_;
    std::vector<gpu::RealisticLeaf> drawnLeaves_;
    double cutVersion_{-1.0};
    std::vector<std::uint32_t> treeIds_;  // per tree: its crown's instance id (scene prim)
    TreeCuts crownCuts_;                  // the cut cells, for the crowns in depth
    static constexpr std::uint32_t kCrownStream = 0x8000'0000U;  // + (sensor << 22) + pixel

    /// The crowns in the depth and instance images, as the lidar sees them (ShapeCaster.hpp
    /// crownStop): each pixel's ray, up to what the render met, stops in a crown it crosses at
    /// an optical depth drawn from its own stream, one draw per crossed crown in order, at the
    /// tree's sway and without its cut cells; where it stops, the depth is there and the instance
    /// is the tree's prim. The colour is the look's (realistic: its trees; simple: TreeShapes.hpp).
    void crownDepth() {
        const std::uint32_t w = camera_.width;
        const std::uint32_t h = camera_.height;
        const std::size_t pixels = static_cast<std::size_t>(w) * h;
        if (images_.depth.size() != pixels || images_.instance.size() != pixels) {
            return;
        }
        // The crowns' broad phase (ShapeCaster.hpp CrownIndex), at their sway as trees() read
        // it: each ray tests only the crowns over whose cells it passes.
        std::vector<std::array<double, 3>> sways;
        for (std::size_t k = 0; k < seenTrees_.size(); ++k) {
            sways.push_back(k < drawnTrees_.size() ? drawnTrees_[k].sway : std::array<double, 3>{});
        }
        CrownIndex index;
        index.build(seenTrees_, sways);
        const Vec3 eye = camera_.position;
        const StepContext& context = frameContext_;
        const std::uint32_t stream = kCrownStream + ((sensor_ & 0x1ffU) << 22U);
        const TreeCuts* cuts = crownCuts_.empty() ? nullptr : &crownCuts_;
        tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
            for (std::uint32_t u = 0; u < w; ++u) {
                const std::size_t i = (static_cast<std::size_t>(v) * w) + u;
                // The ray in the world, z 1 in the optical frame, and its length per unit depth.
                const Vec3 ray = rotate(camera_.orientation, {(u - camera_.cx) / camera_.fx,
                                                              (v - camera_.cy) / camera_.fy, 1.0});
                const double length =
                    std::sqrt((ray[0] * ray[0]) + (ray[1] * ray[1]) + (ray[2] * ray[2]));
                const Vec3 dir{ray[0] / length, ray[1] / length, ray[2] / length};
                const float depth = images_.depth[i];
                double reach = (std::isfinite(depth) ? static_cast<double>(depth) : far_) * length;
                thread_local std::vector<std::uint32_t> near;
                index.candidates(eye, dir, reach, near);
                if (near.empty()) {
                    continue;
                }
                random::Stream rng(context.seed, context.stepIndex,
                                   stream + static_cast<std::uint32_t>(i & 0x3fffffU));
                std::uint32_t hit = kNoIndex;
                for (const std::uint32_t k : near) {
                    if (!crownCrossed(seenTrees_[k], eye, dir, reach, sways[k])) {
                        continue;  // a draw only for the crowns it crosses, as the lidar's
                    }
                    const double at =
                        crownStop(seenTrees_[k], k, eye, dir, reach, sways[k], cuts, rng.uniform());
                    if (at < reach) {
                        reach = at;
                        hit = static_cast<std::uint32_t>(k);
                    }
                }
                if (hit != kNoIndex) {
                    images_.depth[i] = static_cast<float>(reach / length);
                    images_.instance[i] = treeIds_[hit];
                }
            }
        });
    }

    /// This frame's trees, cut cells and flying leaves to every renderer of the camera.
    void trees(const SignalBus& bus) {
        if (drawnTrees_.empty()) {
            return;
        }
        for (std::size_t k = 0; k < drawnTrees_.size() && k < treeSway_.size(); ++k) {
            if (treeSway_[k].valid()) {
                const std::span<const double> v = bus.readN(treeSway_[k]);
                for (std::size_t c = 0; c < 3 && c < v.size(); ++c) {
                    drawnTrees_[k].sway.at(c) = std::isfinite(v[c]) ? v[c] : 0.0;
                }
            }
        }
        if (treeCut_.valid()) {
            const std::span<const double> map = bus.readN(treeCut_);
            crownCuts_.load(map);
            if (map.size() >= kTreeCutHeader && map[0] != cutVersion_) {
                cutVersion_ = map[0];
                drawnCuts_.clear();
                const auto count = static_cast<std::size_t>(std::max(0.0, map[1]));
                for (std::size_t k = 0; k < count; ++k) {
                    const std::size_t at = kTreeCutHeader + (4 * k);
                    if (at + 3 >= map.size()) {
                        break;
                    }
                    drawnCuts_.push_back({static_cast<std::uint32_t>(map[at]),
                                          {static_cast<std::int32_t>(map[at + 1]),
                                           static_cast<std::int32_t>(map[at + 2]),
                                           static_cast<std::int32_t>(map[at + 3])}});
                }
            }
        }
        drawnLeaves_.clear();
        if (leaves_.valid()) {
            const std::span<const double> list = bus.readN(leaves_);
            const std::size_t count =
                list.size() < kLeavesHeader
                    ? 0
                    : std::min(static_cast<std::size_t>(std::max(0.0, list[1])),
                               (list.size() - kLeavesHeader) / kLeavesFields);
            for (std::size_t k = 0; k < count; ++k) {
                const std::size_t at = kLeavesHeader + (kLeavesFields * k);
                drawnLeaves_.push_back({{list[at], list[at + 1], list[at + 2]},
                                        static_cast<float>(list[at + 3]),
                                        static_cast<float>(list[at + 4]),
                                        static_cast<float>(list[at + 5])});
            }
        }
        if (renderer_) {
            renderer_->setTrees(drawnTrees_, drawnCuts_, drawnLeaves_);
        }
        for (const auto& face : faceRenderers_) {
            face->setTrees(drawnTrees_, drawnCuts_, drawnLeaves_);
        }
    }
    SignalId skyIlluminance_;                // env/sky_illuminance, lx; NaN: the drawn day
    gpu::PinholeCamera camera_;              // the output's pinhole (before the sensor model)
    std::unique_ptr<CameraModel> model_;     // null when no part of the sensor model is on
    std::unique_ptr<FisheyeRemap> fisheye_;  // a fisheye's cube faces and remap; else null
    std::vector<std::unique_ptr<gpu::RasterCamera>> faceRenderers_;
    std::vector<gpu::RasterImages> faceImages_;
    gpu::RasterImages rendered_;  // the render the sensor model remaps
    SignalId twist_, exposureOut_;
    double far_{100.0};
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::unique_ptr<gpu::RasterCamera> renderer_;
    bool gpuModel_{false};       // the camera model runs on the GPU after the render
    std::vector<float> signal_;  // its per-pixel signal, for auto exposure
    std::vector<gpu::RasterInstance> instances_;
    gpu::RasterImages images_;
    SignalId image_, depth_, instance_;
    /// The depth image on the water's moving surface: a pixel that met the still level inside
    /// the water goes on to η, and one whose ray passes a crest first stops on it. Rays in the
    /// optical frame have z 1, so the distance along them is the depth.
    void waterDepth() {
        constexpr double kOnLevel = 2e-3;  // m: a depth this near the still level is the water's
        const std::uint32_t w = camera_.width;
        const std::uint32_t h = camera_.height;
        if (images_.depth.size() != static_cast<std::size_t>(w) * h) {
            return;
        }
        const Vec3 eye = camera_.position;
        tbb::parallel_for(std::uint32_t{0}, h, [&](std::uint32_t v) {
            for (std::uint32_t u = 0; u < w; ++u) {
                float& depth = images_.depth[static_cast<std::size_t>(v) * w + u];
                const Vec3 ray = rotate(camera_.orientation, {(u - camera_.cx) / camera_.fx,
                                                              (v - camera_.cy) / camera_.fy, 1.0});
                const double z = std::isfinite(depth) ? static_cast<double>(depth)
                                                      : std::numeric_limits<double>::infinity();
                for (const MovingWater& water : movingWater_.water) {
                    if (water.cells < 2) {
                        continue;
                    }
                    const Vec3 at{eye[0] + ray[0] * z, eye[1] + ray[1] * z, eye[2] + ray[2] * z};
                    const bool onLevel = std::isfinite(z) &&
                                         std::abs(at[2] - water.level) < kOnLevel &&
                                         at[0] >= water.lo[0] && at[0] <= water.hi[0] &&
                                         at[1] >= water.lo[1] && at[1] <= water.hi[1];
                    bool on = false;
                    const double t = rayOntoWater(water, eye, ray, z, onLevel, &on);
                    if (on) {
                        depth = static_cast<float>(t);
                    }
                }
            }
        });
    }

    /// The optical depth of a fog field and the smoke (either may be null) along each pixel's
    /// ray up to its depth (the far clip for the sky), integrated once per kFieldBlock² pixels
    /// and shared across the block.
    void fieldDepth(const fields::Field* fog, const fields::Field* smoke, const Pose& sensor) {
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
                const fields::Vec3 to{from[0] + ray[0], from[1] + ray[1], from[2] + ray[2]};
                const auto tau =
                    static_cast<float>((fog != nullptr ? fog->integrate(from, to) : 0.0) +
                                       (smoke != nullptr ? smoke->integrate(from, to) : 0.0));
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

    MovingWaterInputs movingWater_;
    std::vector<float> waterHeights_;  // the surface for the realistic look, in float
    bool weather_{false};
    SignalId extinction_, contamination_, fogField_;
    fields::FieldSignal fogFieldSignal_;
    bool smoke_{false};
    SignalId smokeField_;
    fields::FieldSignal smokeFieldSignal_;
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
    // The frame in flight between step() and finishStep().
    StepContext frameContext_{};
    Pose sensorPose_{};
    std::array<double, 3> omega_{};  // rad/s, in the optical frame
    bool splitAllowed_{false};       // SplitStep::allowSplit
    bool gpuThermal_{false};         // the thermal image runs on the GPU after the render
    static constexpr double kThermalSpan = 4.0;  // K, a display choice: a frame's least range

    /// ThermalBand::apparent against the ambient surroundings, remembered: most surfaces share
    /// a temperature and an emissivity (the ambient air's, a person's skin), and each answer
    /// costs a few hundred exponentials. The same inputs give the same bits.
    double apparentOf(double temperature, double emissivity) {
        if (!std::isfinite(temperature) || !std::isfinite(emissivity)) {
            return band_->apparent(temperature, emissivity, ambient_);
        }
        const std::pair<double, double> key{temperature, emissivity};
        if (const auto it = apparentCache_.find(key); it != apparentCache_.end()) {
            return it->second;
        }
        constexpr std::size_t kMostRemembered = 4096;  // node temperatures drift: start over
        if (apparentCache_.size() >= kMostRemembered) {
            apparentCache_.clear();
        }
        const double value = band_->apparent(temperature, emissivity, ambient_);
        apparentCache_.emplace(key, value);
        return value;
    }
    std::map<std::pair<double, double>, double> apparentCache_;
    std::vector<float> temperatures_;
    SignalId temperatureOut_;
};
}  // namespace

namespace {

std::vector<std::unique_ptr<IModule>> createModules(
    const World& world, bool groundTruth, const std::shared_ptr<gpu::RayContext>& rayContext,
    const std::shared_ptr<const CameraScene>& cameraScene, bool groundTruthCameras,
    const Scene* scene = nullptr) {
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
                modules.push_back(createRadar(world, s, groundTruth, rayContext));
                break;
            case SensorType::Gnss:
            case SensorType::Magnetometer:
            case SensorType::Barometer:
                break;  // createGeoSensors, below
            case SensorType::Custom:
                // A built-in kind (sensors.md §9), else a plugin implements it (RFC 0008 §3).
                if (auto module =
                        createBuiltinKind(world, s, groundTruth, rayContext, cameraScene, scene)) {
                    modules.push_back(std::move(module));
                }
                break;
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

std::vector<std::unique_ptr<IModule>> createSensorModules(
    const Model& model, std::shared_ptr<gpu::RayContext> rayContext,
    std::shared_ptr<const CameraScene> cameraScene, bool groundTruthCameras) {
    return createModules(model.world, false, rayContext, cameraScene, groundTruthCameras,
                         &model.scene);
}

std::vector<std::unique_ptr<IModule>> createSensorSubstitutes(const Model& model) {
    return createModules(model.world, true, nullptr, nullptr, true, &model.scene);
}

std::shared_ptr<CameraScene> createCameraScene(const Model& model,
                                               const std::string& preferredDevice,
                                               std::string& error, gpu::RasterLook look,
                                               const std::shared_ptr<const CameraScene>& reuse) {
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
    scene->primPaths.reserve(model.scene.prims.size());
    for (const ScenePrim& prim : model.scene.prims) {
        scene->primPaths.push_back(prim.path);
    }
    scene->raster = gpu::RasterScene::create(preferredDevice, meshes, error, look,
                                             reuse ? reuse->raster : nullptr);
    return scene->raster ? scene : nullptr;
}

}  // namespace lagrange::sensors
