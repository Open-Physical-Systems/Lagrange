// An automotive radar by ray launching (Radar.hpp).
#include "kinds/KindModule.hpp"
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Radar.hpp"
#include "lagrange/sensors/Sensors.hpp"
#include "lagrange/sensors/ShapeCaster.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <tbb/parallel_for.h>
#include <tuple>

namespace lagrange::sensors {

namespace {

using namespace literals;
using Vec3 = std::array<double, 3>;

constexpr double kSpeedOfLight = 299792458.0;  // m/s (exact)
constexpr double kDefaultRcsDensity = 0.1;     // m²/m², SensorMaterialAPI_1's fallback
constexpr double kNormalStep = 1e-3;           // m, finite differences for surface normals
constexpr double kBackOff = 0.01;              // m, off the surface for the normal
// The imaging radar's defaults (Radar.hpp createImagingRadar).
constexpr double kHalfWavelength = 0.5;          // element spacing, wavelengths (no grating lobes)
constexpr std::size_t kImagingDetections = 256;  // detections per scan, default
constexpr std::size_t kImagingMostDetections = 4096;  // the most max_detections may ask
constexpr std::size_t kImagingMostRays = 2'000'000;   // the most rays a scan may cast

/// The 80 neighbours of a cell in four dimensions (±1 in each, not itself).
constexpr std::array<std::array<long, 4>, 80> kNeighbours = [] {
    std::array<std::array<long, 4>, 80> out{};
    std::size_t k = 0;
    for (long a = -1; a <= 1; ++a) {
        for (long b = -1; b <= 1; ++b) {
            for (long c = -1; c <= 1; ++c) {
                for (long d = -1; d <= 1; ++d) {
                    if (a != 0 || b != 0 || c != 0 || d != 0) {
                        out[k++] = {a, b, c, d};
                    }
                }
            }
        }
    }
    return out;
}();

SignalSpec spec(std::string name, SignalType type, Unit unit, std::string frame = {}) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = type;
    s.unit = unit;
    s.frame = std::move(frame);
    return s;
}

double setting(const World::Sensors& sensors, std::uint32_t sensor, const char* key,
               double fallback) {
    const auto& map = sensors.settings[sensor];
    const auto it = map.find(key);
    return it == map.end() ? fallback : it->second;
}

Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

class Radar final : public IModule {
public:
    Radar(const World& world, std::uint32_t sensor, bool groundTruth,
          std::shared_ptr<gpu::RayContext> rayContext, bool imaging)
        : rayContext_(std::move(rayContext))
        , groundTruth_(groundTruth)
        , imaging_(imaging)
        , sensor_(sensor)
        , name_(world.sensors.names[sensor])
        , label_((imaging ? "imaging_radar_" : "radar_") + world.sensors.names[sensor])
        , body_(world.sensors.body[sensor].index)
        , mount_(world.sensors.poseInBody[sensor])
        , step_(kinds::kindPeriod(world.sensors.rateHz[sensor])) {}

    [[nodiscard]] ModuleInfo info() const override {
        if (rayContext_) {
            // The rays on the GPU (float ranges): not bit-reproducible, Tier 1 and up.
            return {"sensors", label_, 0, 1, groundTruth_, false, CapabilityTier::Tier1};
        }
        return {"sensors", label_, 0, 1, groundTruth_, true, CapabilityTier::Tier0};
    }
    [[nodiscard]] Duration preferredStep() const override { return step_; }
    Status saveState(StateWriter&) const override { return {}; }
    Status restoreState(StateReader&) override { return {}; }

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World::Sensors& s = world.sensors();
        maxRange_ = setting(s, sensor_, "max_range", 0.0);
        azimuthFov_ = setting(s, sensor_, "azimuth_fov", 0.0);
        elevationFov_ = setting(s, sensor_, "elevation_fov", 0.0);
        resolution_ = setting(s, sensor_, "azimuth_resolution", 0.035);
        rangeResolution_ = setting(s, sensor_, "range_resolution", 0.2);
        velocityResolution_ = setting(s, sensor_, "velocity_resolution", 0.1);
        txPower_ = setting(s, sensor_, "tx_power", 0.0);
        gain_ = setting(s, sensor_, "antenna_gain", 1.0);
        noiseFloor_ = setting(s, sensor_, "noise_floor", 0.0);
        threshold_ = setting(s, sensor_, "detection_threshold", 10.0);
        frequency_ = setting(s, sensor_, "carrier_frequency", 77e9);
        if (maxRange_ <= 0.0 || azimuthFov_ <= 0.0 || elevationFov_ <= 0.0 || txPower_ <= 0.0 ||
            noiseFloor_ <= 0.0 || resolution_ <= 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "radar " + name_ +
                                     " needs its range, fields of view, power, "
                                     "noise floor and resolution");
        }
        if (imaging_) {
            if (Status ok = declareImaging(s); !ok.ok()) {
                return ok;
            }
        }
        // Rays at half the resolution, over the field of view.
        const double spacing = imaging_ ? raySpacing_ : resolution_ / 2.0;
        const int columns = std::max(1, static_cast<int>(std::ceil(azimuthFov_ / spacing)));
        const int rows = std::max(1, static_cast<int>(std::ceil(elevationFov_ / spacing)));
        dAzimuth_ = azimuthFov_ / columns;
        dElevation_ = elevationFov_ / rows;
        rays_.clear();
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < columns; ++c) {
                const double az = -azimuthFov_ / 2.0 + (c + 0.5) * dAzimuth_;
                const double el = -elevationFov_ / 2.0 + (r + 0.5) * dElevation_;
                rays_.push_back(
                    {az,
                     el,
                     {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)}});
            }
        }
        const World& raw = world.raw();
        const std::vector<bool> moving = movingBodies(raw);
        rest_ = restPoses(raw);
        poses_.assign(moving.size(), SignalId{});
        twists_.assign(moving.size(), SignalId{});
        bodyTwist_.assign(moving.size(), {});
        bodyOrigin_.assign(moving.size(), {});
        const auto watch = [&](std::uint32_t b) {
            if (moving[b] && !poses_[b].valid()) {
                poses_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/pose", SignalType::Pose,
                                         units::m, "world_local"));
                twists_[b] = d.input(spec("body/" + raw.bodies.names[b] + "/twist",
                                          SignalType::Wrench, units::m_per_s, "world_local"));
            }
        };
        watch(body_);
        for (const BodyId owner : raw.colliders.body) {
            if (owner != kWorldBody) {
                watch(owner.index);
            }
        }
        // Radar cross-section densities by body (SensorMaterialAPI_1 on a body's prim).
        density_.assign(raw.bodies.names.size(), kDefaultRcsDensity);
        for (const SchemaInstance* m : world.instancesOf("SensorMaterialAPI_1")) {
            for (std::size_t b = 0; b < raw.bodies.prim.size(); ++b) {
                if (raw.bodies.prim[b] == m->prim) {
                    density_[b] =
                        m->real("sensmat:radarCrossSectionDensity").value_or(kDefaultRcsDensity);
                }
            }
        }
        SignalSpec out =
            spec("sensor/" + name_ + "/detections", SignalType::ScalarArray, units::one);
        out.arrayLength = static_cast<std::uint32_t>(maxDetections_ * kRadarFields);
        detections_ = d.output(out);
        count_ = d.output(spec("sensor/" + name_ + "/count", SignalType::Int, units::one));
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        const World::Colliders& colliders = world.colliders();
        shapes_.clear();
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId owner = colliders.body[c];
            if (colliders.shape[c] == ShapeType::HeightField) {
                continue;  // terrain is ground clutter, as the ground plane is (Radar.hpp)
            }
            Shape shape{colliders.shape[c],
                        colliders.size[c],
                        colliders.poseInBody[c],
                        owner == kWorldBody ? -1 : static_cast<int>(owner.index),
                        colliders.shape[c] == ShapeType::ConvexHull
                            ? &world.raw().hulls.at(colliders.hull[c])
                            : nullptr,
                        owner == kWorldBody ? kDefaultRcsDensity : density_[owner.index]};
            if (owner == kWorldBody) {
                shape.moving = -1;
            } else if (!poses_[owner.index].valid()) {
                shape.pose = compose(rest_[owner.index], colliders.poseInBody[c]);
                shape.moving = -1;
            } else {
                shape.moving = static_cast<int>(owner.index);
            }
            shapes_.push_back(shape);
        }
        placed_.resize(shapes_.size());
        casterShapes_.resize(shapes_.size());
        ground_ = -1;
        for (std::size_t k = 0; k < shapes_.size() && ground_ < 0; ++k) {
            if (shapes_[k].type == ShapeType::Plane && shapes_[k].moving < 0) {
                ground_ = static_cast<int>(k);  // the ghosts' mirror
            }
        }
        hits_.resize(rays_.size());
        echoes_.resize(rays_.size());
        if (rayContext_ && !groundTruth_) {
            hullFaces_.assign(shapes_.size(), {});
            for (std::size_t k = 0; k < shapes_.size(); ++k) {
                if (shapes_[k].hull != nullptr) {
                    facePlanes(*shapes_[k].hull, hullFaces_[k]);
                }
            }
            std::vector<std::array<float, 3>> directions(rays_.size());
            for (std::size_t i = 0; i < rays_.size(); ++i) {
                directions[i] = {static_cast<float>(rays_[i].direction[0]),
                                 static_cast<float>(rays_[i].direction[1]),
                                 static_cast<float>(rays_[i].direction[2])};
            }
            scan_ = std::make_unique<gpu::RayScan>(rayContext_, directions);
            castShapes_.resize(shapes_.size());
            gpuHits_.resize(rays_.size());
        }
        write(bus, {});
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            write(bus, {});
            return {};
        }
        const Pose sensorPose = compose(pose(bus, body_), mount_);
        const Vec3 sensorVelocity = pointVelocity(bus, body_, sensorPose.position);
        for (std::size_t k = 0; k < shapes_.size(); ++k) {
            placed_[k] = shapes_[k].moving < 0
                             ? shapes_[k].pose
                             : compose(pose(bus, static_cast<std::uint32_t>(shapes_[k].moving)),
                                       shapes_[k].pose);
            casterShapes_[k] = {shapes_[k].type, shapes_[k].size, placed_[k], shapes_[k].hull};
        }
        // Each ray's hit: on the GPU (float ranges), else on the CPU in parallel. Both give the
        // distance and the shape, ray by ray; what follows does not depend on the order.
        if (scan_) {
            for (std::size_t k = 0; k < shapes_.size(); ++k) {
                gpu::CastShape& out = castShapes_[k];
                out.type = static_cast<std::uint32_t>(shapes_[k].type);
                out.size = shapes_[k].size;
                for (std::size_t i = 0; i < 3; ++i) {
                    out.position[i] = placed_[k].position[i] - sensorPose.position[i];
                }
                out.orientation = placed_[k].orientation;
                out.faces = hullFaces_[k];
            }
            scan_->cast(castShapes_, sensorPose.orientation, gpuHits_);
            for (std::size_t i = 0; i < rays_.size(); ++i) {
                hits_[i] = gpuHits_[i].shape == gpu::kNoShape
                               ? services::RayHit{}
                               : services::RayHit{static_cast<double>(gpuHits_[i].range),
                                                  gpuHits_[i].shape};
            }
        } else {
            caster_.update(casterShapes_);
            tbb::parallel_for(std::size_t{0}, rays_.size(), [&](std::size_t i) {
                hits_[i] = caster_.castHit(sensorPose.position,
                                           rotate(sensorPose.orientation, rays_[i].direction));
            });
        }
        // The moving bodies' poses and twists, read once for the echoes below.
        for (std::size_t b = 0; b < poses_.size(); ++b) {
            if (twists_[b].valid()) {
                const std::span<const double> t = bus.readN(twists_[b]);
                bodyTwist_[b] = {t[0], t[1], t[2], t[3], t[4], t[5]};
                bodyOrigin_[b] = pose(bus, static_cast<std::uint32_t>(b)).position;
            }
        }
        // Each ray's echo, independently (in parallel), then summed in ray order, so the CPU
        // reference's sums are the same numbers in the same order as one loop would give.
        tbb::parallel_for(std::size_t{0}, rays_.size(), [&](std::size_t i) {
            echoes_[i] = echo(rays_[i], hits_[i], sensorPose, sensorVelocity);
        });
        if (imaging_) {
            write(bus, imagingDetections(context, sensorPose, sensorVelocity));
            return {};
        }
        // Echoes by object and range cell (an object's angular extent is one detection).
        struct Cell {
            double power{0.0};                                               // Σσ (m²)
            double range{0.0}, azimuth{0.0}, elevation{0.0}, velocity{0.0};  // σ-weighted sums
        };
        std::map<std::pair<int, long>, Cell> cells;
        for (std::size_t i = 0; i < rays_.size(); ++i) {
            const Echo& e = echoes_[i];
            if (!e.valid) {
                continue;
            }
            const Ray& ray = rays_[i];
            Cell& cell = cells[{e.object, std::lround(e.distance / rangeResolution_)}];
            cell.power += e.rcs;
            cell.range += e.rcs * e.distance;
            cell.azimuth += e.rcs * ray.azimuth;
            cell.elevation += e.rcs * ray.elevation;
            cell.velocity += e.rcs * e.radial;
        }
        std::vector<std::array<double, kRadarFields>> found;
        for (const auto& [key, cell] : cells) {
            const double range = cell.range / cell.power;
            const double snr =
                radarSnr(txPower_, gain_, frequency_, cell.power, range, noiseFloor_);
            if (snr < threshold_) {
                continue;
            }
            found.push_back({range, cell.azimuth / cell.power, cell.elevation / cell.power,
                             cell.velocity / cell.power, cell.power, snr});
        }
        // Strongest first, at most kRadarMaxDetections, with measurement noise.
        std::sort(found.begin(), found.end(),
                  [](const auto& a, const auto& b) { return a[5] > b[5]; });
        if (found.size() > kRadarMaxDetections) {
            found.resize(kRadarMaxDetections);
        }
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (auto& f : found) {
            const double spread = 1.0 / std::sqrt(2.0 * f[5]);
            f[0] += rangeResolution_ * spread * rng.normal();
            f[1] += resolution_ * spread * rng.normal();
            f[2] += resolution_ * spread * rng.normal();
            f[3] += velocityResolution_ * spread * rng.normal();
        }
        write(bus, found);
        return {};
    }

private:
    struct Ray {
        double azimuth, elevation;
        Vec3 direction;
    };
    struct Shape {
        ShapeType type;
        std::array<double, 3> size;
        Pose pose;  // world pose when not moving, else pose in its body
        int moving;
        const ConvexHull* hull;
        double density;
    };

    /// One ray's echo: its cross-section (0: none), range, radial velocity and object.
    struct Echo {
        bool valid{false};
        double rcs{0.0};
        double distance{0.0};
        double radial{0.0};
        int object{0};
        Vec3 point{};           // the hit point (world)
        Vec3 targetVelocity{};  // its velocity (world)
    };

    Echo echo(const Ray& ray, const services::RayHit& hit, const Pose& sensorPose,
              const Vec3& sensorVelocity) const {
        const Vec3 dir = rotate(sensorPose.orientation, ray.direction);
        if (!hit.hit() || hit.distance > maxRange_) {
            return {};
        }
        const Shape& shape = shapes_[hit.shape];
        // The ground plane is clutter, not a target: a real radar's CFAR and clutter map
        // remove the road surface's return, which this model does not simulate.
        if (shape.type == ShapeType::Plane) {
            return {};
        }
        const Vec3 p{sensorPose.position[0] + dir[0] * hit.distance,
                     sensorPose.position[1] + dir[1] * hit.distance,
                     sensorPose.position[2] + dir[2] * hit.distance};
        // Incidence from the surface normal: the gradient of the distance just outside.
        const Vec3 out{p[0] - dir[0] * kBackOff, p[1] - dir[1] * kBackOff,
                       p[2] - dir[2] * kBackOff};
        const ConvexHull* hull = shape.hull;
        const auto distance = [&](const Vec3& q) {
            return distanceToShape(shape.type, shape.size, placed_[hit.shape], q, hull);
        };
        Vec3 normal{distance({out[0] + kNormalStep, out[1], out[2]}) -
                        distance({out[0] - kNormalStep, out[1], out[2]}),
                    distance({out[0], out[1] + kNormalStep, out[2]}) -
                        distance({out[0], out[1] - kNormalStep, out[2]}),
                    distance({out[0], out[1], out[2] + kNormalStep}) -
                        distance({out[0], out[1], out[2] - kNormalStep})};
        const double n = std::sqrt(dot(normal, normal));
        const double cosine = n > 0.0 ? std::abs(dot(normal, dir)) / n : 1.0;
        const double solidAngle = dAzimuth_ * dElevation_ * std::cos(ray.elevation);
        const double rcs = shape.density * hit.distance * hit.distance * solidAngle * cosine;
        if (rcs <= 0.0) {
            return {};
        }
        // Radial velocity of the hit point relative to the radar (receding positive).
        const Vec3 target = shape.moving < 0
                                ? Vec3{}
                                : cachedPointVelocity(static_cast<std::uint32_t>(shape.moving), p);
        const double radial = dot(sub(target, sensorVelocity), dir);
        const int object = shape.moving >= 0 ? shape.moving : -1 - static_cast<int>(hit.shape);
        return {true, rcs, hit.distance, radial, object, p, target};
    }

    /// The imaging radar's cells, waveform and ambiguities (Radar.hpp createImagingRadar).
    Status declareImaging(const World::Sensors& s) {
        const auto& given = s.settings[sensor_];
        const auto refuse = [this](const std::string& what) {
            return Status::error(StatusCode::InvalidConfig, "imaging_radar " + name_ + ": " + what);
        };
        const double lambda = kSpeedOfLight / frequency_;
        const double spacing = setting(s, sensor_, "element_spacing", kHalfWavelength);
        const double azimuthElements = setting(s, sensor_, "azimuth_elements", 0.0);
        const double elevationElements = setting(s, sensor_, "elevation_elements", 0.0);
        const double elevationResolution = setting(s, sensor_, "elevation_resolution", resolution_);
        if (!(spacing > 0.0) || azimuthElements < 0.0 || elevationElements < 0.0 ||
            !(elevationResolution > 0.0) || !(frequency_ > 0.0)) {
            return refuse(
                "element_spacing, elevation_resolution and carrier_frequency must be above 0, "
                "azimuth_elements and elevation_elements not below 0");
        }
        const double quarterTurn = std::numbers::pi / 2.0;
        uCell_ = azimuthElements > 0.0 ? arrayAngleCell(azimuthElements, spacing)
                                       : std::sin(std::min(resolution_, quarterTurn));
        wCell_ = elevationElements > 0.0 ? arrayAngleCell(elevationElements, spacing)
                                         : std::sin(std::min(elevationResolution, quarterTurn));
        // The waveform, where given, sets what was not given directly (Rao, "Introduction to mmWave
        // Sensing: FMCW Radars", TI 2017).
        const double bandwidth = setting(s, sensor_, "bandwidth", 0.0);
        const double chirpPeriod = setting(s, sensor_, "chirp_period", 0.0);
        const double chirps = setting(s, sensor_, "chirps", 0.0);
        if (bandwidth < 0.0 || chirpPeriod < 0.0 || chirps < 0.0) {
            return refuse("bandwidth, chirp_period and chirps must not be below 0");
        }
        if (bandwidth > 0.0 && !given.contains("range_resolution")) {
            rangeResolution_ = kSpeedOfLight / (2.0 * bandwidth);  // ΔR = c / 2B
        }
        if (chirpPeriod > 0.0 && chirps > 0.0 && !given.contains("velocity_resolution")) {
            velocityResolution_ = lambda / (2.0 * chirps * chirpPeriod);  // Δv = λ / 2T_f
        }
        maxVelocity_ = setting(s, sensor_, "max_velocity",
                               chirpPeriod > 0.0 ? lambda / (4.0 * chirpPeriod) : 0.0);
        unambiguousRange_ = setting(s, sensor_, "unambiguous_range", 0.0);
        peakGrouping_ = setting(s, sensor_, "peak_grouping", 1.0) != 0.0;
        ghostReflection_ = setting(s, sensor_, "ghost_reflection", 0.0);
        const double most =
            setting(s, sensor_, "max_detections", static_cast<double>(kImagingDetections));
        if (!(rangeResolution_ > 0.0) || !(velocityResolution_ > 0.0) || maxVelocity_ < 0.0 ||
            unambiguousRange_ < 0.0 || ghostReflection_ < 0.0 || ghostReflection_ > 1.0 ||
            most < 1.0 || most > static_cast<double>(kImagingMostDetections)) {
            return refuse(
                "range_resolution and velocity_resolution must be above 0, max_velocity and "
                "unambiguous_range not below 0, ghost_reflection in 0..1, max_detections in "
                "1.." +
                std::to_string(kImagingMostDetections));
        }
        maxDetections_ = static_cast<std::size_t>(most);
        raySpacing_ = setting(
            s, sensor_, "ray_spacing",
            std::min(std::asin(std::min(uCell_, 1.0)), std::asin(std::min(wCell_, 1.0))) / 2.0);
        const double rays = std::ceil(azimuthFov_ / std::max(raySpacing_, 1e-9)) *
                            std::ceil(elevationFov_ / std::max(raySpacing_, 1e-9));
        if (!(raySpacing_ > 0.0) || rays > static_cast<double>(kImagingMostRays)) {
            return refuse("ray_spacing must be above 0 and give at most " +
                          std::to_string(kImagingMostRays) + " rays over the field of view");
        }
        return {};
    }

    /// The imaging radar's detections: every echo (and its ground ghost) in its cell of range,
    /// Doppler and direction sines; a cell over the threshold is one detection.
    std::vector<std::array<double, kRadarFields>> imagingDetections(const StepContext& context,
                                                                    const Pose& sensorPose,
                                                                    const Vec3& sensorVelocity) {
        struct Cell {
            double power{0.0};   // Σσ (m²)
            double signal{0.0};  // Σσ/R⁴ (m⁻²), at the true ranges
            double range{0.0}, azimuth{0.0}, elevation{0.0}, velocity{0.0};  // signal-weighted
        };
        std::map<std::array<long, 4>, Cell> cells;
        const auto add = [&](double rcs, double range, double azimuth, double elevation,
                             double radial) {
            if (!(rcs > 0.0) || range > maxRange_ || std::abs(azimuth) > azimuthFov_ / 2.0 ||
                std::abs(elevation) > elevationFov_ / 2.0) {
                return;
            }
            const double measured =
                unambiguousRange_ > 0.0 ? std::fmod(range, unambiguousRange_) : range;
            const double velocity = foldVelocity(radial, maxVelocity_);
            const double u = std::cos(elevation) * std::sin(azimuth);
            const double w = std::sin(elevation);
            Cell& cell = cells[{std::lround(measured / rangeResolution_),
                                std::lround(velocity / velocityResolution_),
                                std::lround(u / uCell_), std::lround(w / wCell_)}];
            const double r2 = range * range;
            const double signal = rcs / (r2 * r2);
            cell.power += rcs;
            cell.signal += signal;
            cell.range += signal * measured;
            cell.azimuth += signal * azimuth;
            cell.elevation += signal * elevation;
            cell.velocity += signal * velocity;
        };
        for (std::size_t i = 0; i < rays_.size(); ++i) {
            if (echoes_[i].valid) {
                add(echoes_[i].rcs, echoes_[i].distance, rays_[i].azimuth, rays_[i].elevation,
                    echoes_[i].radial);
            }
        }
        // Ghosts: out and back by the ground, from the target's mirror image below it.
        if (ghostReflection_ > 0.0 && ground_ >= 0) {
            const Pose& g = placed_[static_cast<std::size_t>(ground_)];
            const Vec3 n = rotate(g.orientation, {0.0, 0.0, 1.0});
            const Vec3 s = sensorPose.position;
            const double twice = ghostReflection_ * ghostReflection_;  // two bounces: |Γ|⁴
            const std::array<double, 4> toSensor = conjugate(sensorPose.orientation);
            if (dot(sub(s, g.position), n) > 0.0) {
                for (const Echo& e : echoes_) {
                    const double height = e.valid ? dot(sub(e.point, g.position), n) : 0.0;
                    if (!(height > 0.0)) {
                        continue;
                    }
                    const Vec3 image{e.point[0] - 2.0 * height * n[0],
                                     e.point[1] - 2.0 * height * n[1],
                                     e.point[2] - 2.0 * height * n[2]};
                    const Vec3 d = sub(image, s);
                    const double range = std::sqrt(dot(d, d));
                    const Vec3 dir{d[0] / range, d[1] / range, d[2] / range};
                    const Vec3 local = rotate(toSensor, dir);
                    const double along = dot(e.targetVelocity, n);
                    const Vec3 mirrored{e.targetVelocity[0] - 2.0 * along * n[0],
                                        e.targetVelocity[1] - 2.0 * along * n[1],
                                        e.targetVelocity[2] - 2.0 * along * n[2]};
                    add(e.rcs * twice, range, std::atan2(local[1], local[0]),
                        std::asin(std::clamp(local[2], -1.0, 1.0)),
                        dot(sub(mirrored, sensorVelocity), dir));
                }
            }
        }
        // Peak grouping: each cell climbs to its strongest neighbour (±1 cell in each of the
        // four dimensions) until none is stronger; a peak takes in every cell that climbs to it,
        // as one response spread over neighbouring cells by the grid is one detection.
        if (peakGrouping_ && cells.size() > 1) {
            std::vector<std::map<std::array<long, 4>, Cell>::iterator> list;
            std::map<std::array<long, 4>, std::size_t> index;
            for (auto it = cells.begin(); it != cells.end(); ++it) {
                index.emplace(it->first, list.size());
                list.push_back(it);
            }
            // A strict order: stronger, or as strong and earlier.
            const auto above = [&list](std::size_t a, std::size_t b) {
                return list[a]->second.signal > list[b]->second.signal ||
                       (list[a]->second.signal == list[b]->second.signal && a < b);
            };
            std::vector<std::size_t> up(list.size());
            for (std::size_t i = 0; i < list.size(); ++i) {
                std::size_t best = i;
                const std::array<long, 4>& key = list[i]->first;
                for (const std::array<long, 4>& off : kNeighbours) {
                    const auto n = index.find(
                        {key[0] + off[0], key[1] + off[1], key[2] + off[2], key[3] + off[3]});
                    if (n != index.end() && above(n->second, best)) {
                        best = n->second;
                    }
                }
                up[i] = best;
            }
            std::map<std::array<long, 4>, Cell> peaks;
            for (std::size_t i = 0; i < list.size(); ++i) {
                std::size_t top = i;
                while (up[top] != top) {
                    top = up[top];  // strictly stronger each time: it ends
                }
                Cell& peak = peaks[list[top]->first];
                const Cell& c = list[i]->second;
                peak.power += c.power;
                peak.signal += c.signal;
                peak.range += c.range;
                peak.azimuth += c.azimuth;
                peak.elevation += c.elevation;
                peak.velocity += c.velocity;
            }
            cells = std::move(peaks);
        }
        const double perSignal = radarSnr(txPower_, gain_, frequency_, 1.0, 1.0, noiseFloor_);
        std::vector<std::array<double, kRadarFields>> found;
        for (const auto& [key, cell] : cells) {
            const double snr = perSignal * cell.signal;
            if (snr < threshold_) {
                continue;
            }
            found.push_back({cell.range / cell.signal, cell.azimuth / cell.signal,
                             cell.elevation / cell.signal, cell.velocity / cell.signal, cell.power,
                             snr});
        }
        std::stable_sort(found.begin(), found.end(),
                         [](const auto& a, const auto& b) { return a[5] > b[5]; });
        if (found.size() > maxDetections_) {
            found.resize(maxDetections_);
        }
        // Noise of a cell / √(2·SNR); an angle cell is wider off boresight (1 / cos).
        constexpr double kLeastCosine = 0.05;
        random::Stream rng(context.seed, context.stepIndex, 0);
        for (auto& f : found) {
            const double spread = 1.0 / std::sqrt(2.0 * f[5]);
            const double cosEl = std::max(std::cos(f[2]), kLeastCosine);
            const double cosAz = std::max(std::cos(f[1]), kLeastCosine);
            f[0] += rangeResolution_ * spread * rng.normal();
            f[1] += uCell_ / (cosAz * cosEl) * spread * rng.normal();
            f[2] += wCell_ / cosEl * spread * rng.normal();
            f[3] = foldVelocity(f[3] + velocityResolution_ * spread * rng.normal(), maxVelocity_);
        }
        return found;
    }

    /// pointVelocity() from the twists and poses read at the start of the step.
    Vec3 cachedPointVelocity(std::uint32_t body, const Vec3& p) const {
        if (!twists_[body].valid()) {
            return {};
        }
        const std::array<double, 6>& t = bodyTwist_[body];
        const Vec3 w{t[3], t[4], t[5]};
        const Vec3 r = sub(p, bodyOrigin_[body]);
        const Vec3 spin = cross(w, r);
        return {t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};
    }

    Pose pose(const SignalBus& bus, std::uint32_t body) const {
        if (!poses_[body].valid()) {
            return rest_[body];
        }
        const std::span<const double> v = bus.readN(poses_[body]);
        return {{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}};
    }

    /// The velocity of point `p` (world) of body `body`: v + ω × (p − origin).
    Vec3 pointVelocity(const SignalBus& bus, std::uint32_t body, const Vec3& p) const {
        if (!twists_[body].valid()) {
            return {};
        }
        const std::span<const double> t = bus.readN(twists_[body]);
        const Pose at = pose(bus, body);
        const Vec3 w{t[3], t[4], t[5]};
        const Vec3 r = sub(p, at.position);
        const Vec3 spin = cross(w, r);
        return {t[0] + spin[0], t[1] + spin[1], t[2] + spin[2]};
    }

    void write(SignalBus& bus, const std::vector<std::array<double, kRadarFields>>& found) {
        std::vector<double> out(maxDetections_ * kRadarFields,
                                std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < found.size(); ++i) {
            std::copy(found[i].begin(), found[i].end(), out.begin() + i * kRadarFields);
        }
        bus.writeN(detections_, out);
        bus.write(count_, static_cast<double>(found.size()));
    }

    std::shared_ptr<gpu::RayContext> rayContext_;  // null: rays on the CPU
    bool groundTruth_;
    bool imaging_;  // kind imaging_radar: detections by cell, not by object
    std::size_t maxDetections_{kRadarMaxDetections};
    // The imaging radar's cells and ambiguities (declareImaging).
    double uCell_{0.0}, wCell_{0.0};  // angle cells in direction sine (azimuth, elevation)
    double raySpacing_{0.0};          // rad
    double maxVelocity_{0.0};         // m/s, unambiguous ±; 0: no folding
    double unambiguousRange_{0.0};    // m; 0: no folding
    double ghostReflection_{0.0};     // the ground's |Γ|²
    bool peakGrouping_{true};         // cells merge into their local peak
    int ground_{-1};                  // the fixed ground plane among shapes_, -1: none
    std::uint32_t sensor_;
    std::string name_;
    std::string label_;
    std::uint32_t body_;
    Pose mount_;
    Duration step_;
    double maxRange_{0.0}, azimuthFov_{0.0}, elevationFov_{0.0}, resolution_{0.0};
    double rangeResolution_{0.2}, velocityResolution_{0.1}, txPower_{0.0}, gain_{1.0};
    double noiseFloor_{0.0}, threshold_{10.0}, frequency_{77e9};
    double dAzimuth_{0.0}, dElevation_{0.0};
    std::vector<Ray> rays_;
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_, twists_;
    std::vector<double> density_;
    std::vector<Shape> shapes_;
    std::vector<Pose> placed_;
    std::vector<ShapeCaster::Shape> casterShapes_;
    ShapeCaster caster_;
    std::vector<services::RayHit> hits_;
    std::vector<Echo> echoes_;
    std::vector<std::array<double, 6>> bodyTwist_;
    std::vector<Vec3> bodyOrigin_;
    std::unique_ptr<gpu::RayScan> scan_;
    std::vector<gpu::CastShape> castShapes_;
    std::vector<gpu::CastHit> gpuHits_;
    std::vector<std::vector<std::array<double, 4>>> hullFaces_;
    SignalId detections_, count_;
};

}  // namespace

double radarSnr(double txPower, double gain, double frequency, double rcs, double range,
                double noiseFloor) {
    const double lambda = kSpeedOfLight / frequency;
    const double fourPi = 4.0 * std::numbers::pi;
    return txPower * gain * gain * lambda * lambda * rcs /
           (fourPi * fourPi * fourPi * std::pow(range, 4) * noiseFloor);
}

std::unique_ptr<IModule> createRadar(const World& world, std::uint32_t sensor, bool groundTruth,
                                     std::shared_ptr<gpu::RayContext> rayContext) {
    return std::make_unique<Radar>(world, sensor, groundTruth, std::move(rayContext), false);
}

std::unique_ptr<IModule> createImagingRadar(const World& world, std::uint32_t sensor,
                                            bool groundTruth,
                                            std::shared_ptr<gpu::RayContext> rayContext) {
    return std::make_unique<Radar>(world, sensor, groundTruth, std::move(rayContext), true);
}

double arrayAngleCell(double elements, double spacingWavelengths) {
    return 1.0 / (elements * spacingWavelengths);
}

double foldVelocity(double v, double maxVelocity) {
    if (!(maxVelocity > 0.0)) {
        return v;
    }
    const double span = 2.0 * maxVelocity;
    return v - span * std::floor((v + maxVelocity) / span);
}

}  // namespace lagrange::sensors
