// Kind "sonar" (sensors.md §9): an underwater sonar. A multibeam imaging sonar's fan of beams
// (after the Blueprint Subsea Oculus M750d in its 750 kHz mode), or with `beams` 1 a single-beam
// echosounder or altimeter (Blue Robotics Ping2). It works only under the surface of a body of
// water (WaterAPI_1, as the hydrodynamics domain and the lidars read it): each beam's first echo
// from a collider, the water's surface or its bottom, its range from the time of flight at the
// device's sound speed, and its echo level from spreading, absorption and Lambert backscatter.
#include "EnvCommon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace lagrange::sensors::kinds {

// Shared with the tests by name; in an inline namespace of their own (no clash with
// other kinds' helpers).
inline namespace extra2 {

/// Sound speed in sea water (m/s) at temperature `t` (°C), salinity `s` (PSU) and depth `d` (m):
/// Mackenzie's nine-term equation (J. Acoust. Soc. Am. 70(3), 807–812, 1981), for 0–30 °C,
/// 30–40 PSU, 0–8000 m.
double mackenzieSoundSpeed(double t, double s, double d) {
    return 1448.96 + 4.591 * t - 5.304e-2 * t * t + 2.374e-4 * t * t * t + 1.340 * (s - 35.0) +
           1.630e-2 * d + 1.675e-7 * d * d - 1.025e-2 * t * (s - 35.0) - 7.139e-13 * t * d * d * d;
}

/// Absorption of sound in sea water (dB/km) at frequency `fKHz` (kHz), temperature `t` (°C),
/// salinity `s` (PSU), depth `zKm` (km) and acidity `ph`: Ainslie and McColm's simplification of
/// Francois and Garrison (J. Acoust. Soc. Am. 103(3), 1671–1672, 1998): boric acid, magnesium
/// sulphate and pure water.
double seawaterAbsorption(double fKHz, double t, double s, double zKm, double ph) {
    const double f2 = fKHz * fKHz;
    const double f1 = 0.78 * std::sqrt(s / 35.0) * std::exp(t / 26.0);  // kHz, boric acid
    const double fm = 42.0 * std::exp(t / 17.0);                        // kHz, MgSO4
    const double boric = 0.106 * f1 * f2 / (f1 * f1 + f2) * std::exp((ph - 8.0) / 0.56);
    const double magnesium =
        0.52 * (1.0 + t / 43.0) * (s / 35.0) * fm * f2 / (fm * fm + f2) * std::exp(-zKm / 6.0);
    const double water = 0.00049 * f2 * std::exp(-(t / 27.0 + zKm / 17.0));
    return boric + magnesium + water;
}

}  // namespace extra2

namespace {

// Blueprint Subsea Oculus M750d, 750 kHz mode (Oculus M-series specifications page): 0.1 to 120 m,
// 130° by 20°, 512 beams, 4 mm range resolution, up to 40 Hz.
constexpr double kMinRange = 0.1;                                     // m
constexpr double kMaxRange = 120.0;                                   // m
constexpr double kFovH = 130.0 * std::numbers::pi / 180.0;            // rad
constexpr double kFovV = 20.0 * std::numbers::pi / 180.0;             // rad
constexpr double kBeams = 512.0;                                      // 1
constexpr double kResolution = 0.004;                                 // m
constexpr double kFrequency = 750e3;                                  // Hz
constexpr double kSingleBeamWidth = 25.0 * std::numbers::pi / 180.0;  // rad, Blue Robotics Ping2
// The nominal sound speed sonars are set to (Urick, Principles of Underwater Sound, 3rd ed.,
// 1983, §5.1: 1500 m/s is the conventional round value for sea water).
constexpr double kSoundSpeed = 1500.0;  // m/s
// Lambert's rule for seabed backscatter, 10·log10(μ) = −27 dB (Mackenzie, J. Acoust. Soc. Am.
// 33(11), 1498–1504, 1961), used for every surface here.
constexpr double kBackscatter = -27.0;  // dB
// Sea water for the absorption when not given: 10 °C, 35 PSU, pH 8 (illustrative).
constexpr double kTemperature = 10.0;  // °C
constexpr double kSalinity = 35.0;     // PSU
constexpr double kPh = 8.0;
constexpr double kNoiseDb = 1.0;       // dB, 1σ of the echo level, illustrative
constexpr double kMinLevel = -180.0;   // dB, the weakest echo detected, illustrative
constexpr double kNormalProbe = 1e-3;  // rad between the rays that find a surface's normal

/// A body of water: its surface frame (z up, origin the middle of the surface) and extent.
struct WaterBox {
    Pose surface;
    double halfX{5.0}, halfY{5.0}, depth{2.0};
};

Vec3 toFrame(const Pose& frame, const Vec3& p) {
    return rotate(conjugate(frame.orientation), sub(p, frame.position));
}

constexpr double kMinWaterTemperature = -2.0;  // °C: sea water freezes at about −1.9 °C
constexpr double kMaxWaterTemperature = 40.0;  // °C: above any natural water body
constexpr double kMaxSalinity = 45.0;          // PSU: above the Red Sea's 41

class Sonar final : public KindModule {
public:
    Sonar(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {
        // A ping waits for the farthest echo: it cannot repeat faster than 2·R/c.
        const double reach = kindSetting(world, sensor, "range/max", kMaxRange);
        const double c = kindSetting(world, sensor, "sound_speed", kSoundSpeed);
        if (reach > 0.0 && c > 0.0) {
            pingInterval_ = 2.0 * reach / c;
            step_ = std::max(step_, kindStep(pingInterval_ * 1e9, true));
        }
    }

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& w = view.raw();
        const std::string fail = "sonar " + name_ + ": ";
        beams_ = static_cast<std::uint32_t>(std::llround(kindSetting(w, sensor_, "beams", kBeams)));
        minRange_ = kindSetting(w, sensor_, "range/min", kMinRange);
        maxRange_ = kindSetting(w, sensor_, "range/max", kMaxRange);
        fovH_ = kindSetting(w, sensor_, "horizontal_fov", kFovH);
        fovV_ = kindSetting(w, sensor_, "vertical_fov", kFovV);
        beamWidth_ = kindSetting(w, sensor_, "beam_width", kSingleBeamWidth);
        rays_ = static_cast<int>(std::llround(kindSetting(w, sensor_, "vertical_rays", 5.0)));
        resolution_ = kindSetting(w, sensor_, "resolution", kResolution);
        noise_ = kindSetting(w, sensor_, "noise/stddev", resolution_);
        noiseDb_ = kindSetting(w, sensor_, "intensity_noise", kNoiseDb);
        minLevel_ = kindSetting(w, sensor_, "min_level", kMinLevel);
        backscatter_ = kindSetting(w, sensor_, "backscatter", kBackscatter);
        frequency_ = kindSetting(w, sensor_, "frequency", kFrequency);
        soundSpeed_ = kindSetting(w, sensor_, "sound_speed", kSoundSpeed);
        const auto& settings = w.sensors.settings[sensor_];
        temperatureGiven_ = settings.contains("water_temperature");
        temperature_ = kindSetting(w, sensor_, "water_temperature", kTemperature);
        salinity_ = kindSetting(w, sensor_, "salinity", kSalinity);
        ph_ = kindSetting(w, sensor_, "ph", kPh);
        bins_ = static_cast<std::uint32_t>(std::llround(kindSetting(w, sensor_, "image_bins", 0)));
        // The water's temperature is in °C (Mackenzie's equation, the absorption): one given in
        // K by §9's SI habit (283.15) made 3886 m/s of sound speed (ranges 2.6 times short).
        // Outside the water's own range it is refused.
        if (!(temperature_ >= kMinWaterTemperature && temperature_ <= kMaxWaterTemperature) ||
            !(salinity_ >= 0.0 && salinity_ <= kMaxSalinity)) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail +
                                     "water_temperature is in °C (−2 to 40) and salinity in "
                                     "PSU (0 to 45)");
        }
        if (beams_ < 1 || beams_ > 4096 || minRange_ < 0.0 || maxRange_ <= minRange_ ||
            fovH_ < 0.0 || fovV_ < 0.0 || beamWidth_ <= 0.0 || rays_ < 1 || rays_ > 64 ||
            noise_ < 0.0 || noiseDb_ < 0.0 || frequency_ <= 0.0 || soundSpeed_ <= 0.0 ||
            bins_ > 4096) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail + "beams, ranges, angles or noise out of range");
        }

        waters_.clear();
        for (const SchemaInstance* inst : view.instancesOf("WaterAPI_1")) {
            WaterBox box;
            box.surface = w.prims.poseInBody.at(inst->prim.index);
            box.halfX = inst->real("water:sizeX").value_or(10.0) / 2.0;
            box.halfY = inst->real("water:sizeY").value_or(10.0) / 2.0;
            box.depth = inst->real("water:depth").value_or(2.0);
            waters_.push_back(box);
        }
        layout();
        motion_.declare(w, body_, d, false);
        scene_.declare(w, d);
        SignalSpec ranges = kindSpec(base() + "/ranges", SignalType::ScalarArray, units::m);
        ranges.arrayLength = beams_;
        SignalSpec levels = kindSpec(base() + "/intensities", SignalType::ScalarArray, units::one);
        levels.arrayLength = beams_;
        ranges_ = d.output(ranges);
        intensities_ = d.output(levels);
        if (bins_ > 0) {
            SignalSpec image = kindSpec(base() + "/image", SignalType::ScalarArray, units::one);
            image.arrayLength = beams_ * bins_;
            image_ = d.output(image);
        }
        depth_ = d.output(kindSpec(base() + "/depth", SignalType::Scalar, units::m));
        speed_ = d.output(kindSpec(base() + "/sound_speed", SignalType::Scalar, units::m_per_s));
        interval_ = d.output(kindSpec(base() + "/ping_interval", SignalType::Scalar, units::s));
        valid_ = d.output(kindSpec(base() + "/valid", SignalType::Bool, units::one));
        return {};
    }

    Status init(const WorldView& view, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(view.raw());
        range_.assign(beams_, kInf);
        level_.assign(beams_, kNaN);
        pixels_.assign(static_cast<std::size_t>(beams_) * bins_, 0.0);
        writeAll(bus, kNaN, kNaN);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose pose = compose(motion_.pose(bus), mount_);
        const WaterBox* water = nullptr;
        double depth = kNaN;
        for (const WaterBox& box : waters_) {
            const Vec3 p = toFrame(box.surface, pose.position);
            if (std::abs(p[0]) <= box.halfX && std::abs(p[1]) <= box.halfY && p[2] < 0.0 &&
                p[2] > -box.depth) {
                water = &box;
                depth = -p[2];
                break;
            }
        }
        std::ranges::fill(range_, kInf);
        std::ranges::fill(level_, kNaN);
        std::ranges::fill(pixels_, 0.0);
        if (water == nullptr) {  // in air: a sonar hears nothing
            writeAll(bus, kNaN, kNaN);
            bus.write(valid_, 0.0);
            return {};
        }
        // The water's own sound speed: Mackenzie's from its temperature when given, else the
        // device's setting. The device turns time of flight into range with its setting.
        const double cWater =
            temperatureGiven_ ? mackenzieSoundSpeed(temperature_, salinity_, depth) : soundSpeed_;
        const double alpha =  // dB/m, one way
            seawaterAbsorption(frequency_ / 1000.0, temperature_, salinity_, depth / 1000.0, ph_) /
            1000.0;
        scene_.update(bus);
        const double scale = soundSpeed_ / cWater;
        for (std::uint32_t b = 0; b < beams_; ++b) {
            double nearest = kInf;
            double cosIncidence = 1.0;
            for (const Vec3& local : directions_[b]) {
                const Vec3 dir = rotate(pose.orientation, local);
                double n = 1.0;
                const double hit = echo(*water, pose.position, dir, n);
                if (hit < nearest) {
                    nearest = hit;
                    cosIncidence = n;
                }
            }
            if (!(nearest <= maxRange_ / scale) || nearest * scale < minRange_) {
                continue;
            }
            // Echo level re a perfect reflector at 1 m: two-way spherical spreading and
            // absorption, and Lambert backscatter μ·cos²θ (Urick 1983, §8 and §9).
            const double cos2 = std::max(cosIncidence * cosIncidence, 1e-12);
            double level = -40.0 * std::log10(std::max(nearest, 1e-3)) - 2.0 * alpha * nearest +
                           backscatter_ + 10.0 * std::log10(cos2);
            double range = nearest * scale;
            if (groundTruth_) {
                range = nearest;
            } else {
                level += noiseDb_ * gaussian(context, 2 * b + 1);
                range = quantize(range + noise_ * gaussian(context, 2 * b), resolution_);
                range = std::clamp(range, minRange_, maxRange_);
                if (level < minLevel_) {
                    continue;
                }
            }
            range_[b] = range;
            level_[b] = level;
            if (bins_ > 0) {
                const double f = (range - minRange_) / (maxRange_ - minRange_);
                const auto bin = std::min<std::uint32_t>(
                    bins_ - 1, static_cast<std::uint32_t>(std::max(0.0, f) * bins_));
                pixels_[static_cast<std::size_t>(b) * bins_ + bin] =
                    std::max(0.0, level - minLevel_);
            }
        }
        writeAll(bus, depth, cWater);
        bus.write(valid_, 1.0);
        return {};
    }

private:
    static constexpr double kInf = std::numeric_limits<double>::infinity();

    /// Each beam's ray directions in the sensor frame (x forward, y left, z up).
    void layout() {
        directions_.assign(beams_, {});
        const auto dirOf = [](double azimuth, double elevation) {
            return Vec3{std::cos(elevation) * std::cos(azimuth),
                        std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
        };
        const auto spread = [](int k, int n, double width) {
            return n == 1 ? 0.0 : -width / 2.0 + width * k / (n - 1);
        };
        if (beams_ == 1) {  // a cone of beam_width: rays on a grid inside it
            for (int i = 0; i < rays_; ++i) {
                for (int j = 0; j < rays_; ++j) {
                    const double a = spread(i, rays_, beamWidth_);
                    const double e = spread(j, rays_, beamWidth_);
                    if (std::hypot(a, e) <= beamWidth_ / 2.0 + 1e-12) {
                        directions_[0].push_back(dirOf(a, e));
                    }
                }
            }
            return;
        }
        for (std::uint32_t b = 0; b < beams_; ++b) {
            // Beam 0 on the left (+y), as the M750d's image is drawn.
            const double a = fovH_ / 2.0 - fovH_ * b / (beams_ - 1);
            for (int j = 0; j < rays_; ++j) {
                directions_[b].push_back(dirOf(a, spread(j, rays_, fovV_)));
            }
        }
    }

    /// Distance to the first echo along `dir` from `origin` inside `water`, and the cosine of the
    /// incidence there (`cosIncidence`); +inf for none (the sound leaves through a side).
    double echo(const WaterBox& water, const Vec3& origin, const Vec3& dir,
                double& cosIncidence) const {
        // The surface (z = 0) and the bottom (z = −depth) of the box, in its frame.
        const Vec3 o = toFrame(water.surface, origin);
        const Vec3 v = rotate(conjugate(water.surface.orientation), dir);
        double plane = kInf;
        if (v[2] != 0.0) {
            const double t = ((v[2] > 0.0 ? 0.0 : -water.depth) - o[2]) / v[2];
            const double x = o[0] + v[0] * t;
            const double y = o[1] + v[1] * t;
            if (t > 0.0 && std::abs(x) <= water.halfX && std::abs(y) <= water.halfY) {
                plane = t;
            }
        }
        const double hit = scene_.cast(origin, dir, body_);
        if (hit >= plane) {
            cosIncidence = std::abs(v[2]);
            return plane;
        }
        // A collider: its normal from two more rays a little apart.
        cosIncidence = 1.0;
        Vec3 u = cross(dir, {0.0, 0.0, 1.0});
        if (norm(u) < 1e-6) {
            u = cross(dir, {1.0, 0.0, 0.0});
        }
        const double un = norm(u);
        u = {u[0] / un, u[1] / un, u[2] / un};
        const Vec3 w = cross(dir, u);
        const auto tilt = [&](const Vec3& axis) {
            return Vec3{dir[0] + kNormalProbe * axis[0], dir[1] + kNormalProbe * axis[1],
                        dir[2] + kNormalProbe * axis[2]};
        };
        const auto at = [&](const Vec3& d, double t) {
            const double n = norm(d);
            return Vec3{origin[0] + d[0] / n * t, origin[1] + d[1] / n * t,
                        origin[2] + d[2] / n * t};
        };
        const Vec3 du = tilt(u), dw = tilt(w);
        const double hu = scene_.cast(origin, scaledUnit(du), body_);
        const double hw = scene_.cast(origin, scaledUnit(dw), body_);
        if (std::isfinite(hu) && std::isfinite(hw)) {
            const Vec3 p0 = at(dir, hit);
            const Vec3 n = cross(sub(at(du, hu), p0), sub(at(dw, hw), p0));
            const double nn = norm(n);
            if (nn > 0.0) {
                cosIncidence = std::min(1.0, std::abs(dot(n, dir)) / nn);
            }
        }
        return hit;
    }

    static Vec3 scaledUnit(const Vec3& d) {
        const double n = norm(d);
        return {d[0] / n, d[1] / n, d[2] / n};
    }

    void writeAll(SignalBus& bus, double depth, double speed) {
        bus.writeN(ranges_, range_);
        bus.writeN(intensities_, level_);
        if (image_.valid()) {
            bus.writeN(image_, pixels_);
        }
        bus.write(depth_, depth);
        bus.write(speed_, speed);
        bus.write(interval_, pingInterval_);
    }

    std::uint32_t beams_{512}, bins_{0};
    int rays_{5};
    double minRange_{kMinRange}, maxRange_{kMaxRange}, fovH_{kFovH}, fovV_{kFovV};
    double beamWidth_{kSingleBeamWidth}, resolution_{kResolution}, noise_{kResolution};
    double noiseDb_{kNoiseDb}, minLevel_{kMinLevel}, backscatter_{kBackscatter};
    double frequency_{kFrequency}, soundSpeed_{kSoundSpeed}, pingInterval_{0.16};
    bool temperatureGiven_{false};
    double temperature_{kTemperature}, salinity_{kSalinity}, ph_{kPh};
    std::vector<WaterBox> waters_;
    std::vector<std::vector<Vec3>> directions_;
    std::vector<double> range_, level_, pixels_;
    BodyMotion motion_;
    ColliderScene scene_;
    SignalId ranges_, intensities_, image_, depth_, speed_, interval_, valid_;
};

}  // namespace

std::unique_ptr<IModule> makeSonar(const World& world, std::uint32_t sensor, bool groundTruth,
                                   const KindContext&) {
    return std::make_unique<Sonar>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
