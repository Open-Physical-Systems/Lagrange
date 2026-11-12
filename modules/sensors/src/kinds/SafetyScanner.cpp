// Kind `safety_scanner` (sensors.md §9): a safety laser scanner in the manner of SICK's nanoScan3
// and microScan3. It scans a plane (the sensor's xy plane) over its aperture and checks every
// return against a protective field and a warning field, each a rectangle, a sector or a polygon
// in the sensor frame. Its outputs switch one response time after the scan that saw the field
// change: `protective` is what turns its OSSDs off (1: something is in the protective field).
//
// Defaults are the nanoScan3 Core I/O's (SICK data sheet NANS3-AAAZ30AN1, part 1100333): 275°
// scanning angle, 0.17° angular resolution, 70 ms response time, 40 m distance measuring range.
// Its fields (3 m protective, 10 m warning at most) are the user's: they have no default.
#include "KindScene.hpp"
#include "lagrange/random/Philox.hpp"
#include "lagrange/sensors/Sensors.hpp"

#include <cmath>
#include <deque>
#include <numbers>
#include <sstream>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kNans3Aperture = 275.0 * kDeg;   // rad, NANS3-AAAZ30AN1 "Scanning angle"
constexpr double kNans3Resolution = 0.17 * kDeg;  // rad, NANS3-AAAZ30AN1 "Angular resolution"
constexpr double kNans3ResponseTime = 0.070;      // s, NANS3-AAAZ30AN1 "Response time"
constexpr double kNans3Range = 40.0;              // m, "Distance measuring range"
constexpr std::uint32_t kMaxBeams = 20000;
// s: the longest response time taken. The delay is a ring of one state per scan, so a response
// time of 1e9 s asked for 3e10 bytes at 30 ms scans (std::bad_alloc at start); real scanners
// answer within a fraction of a second (the NANS3's 70 ms).
constexpr double kMaxResponseTime = 60.0;

/// A field in the sensor's xy plane: a polygon (a rectangle is one), or a sector of a circle
/// over the scanner's aperture.
struct Field {
    std::vector<std::array<double, 2>> polygon;
    double radius{0.0};  // a sector when > 0
    [[nodiscard]] bool empty() const { return polygon.empty() && radius <= 0.0; }
    [[nodiscard]] bool contains(double x, double y) const {
        if (radius > 0.0) {
            return x * x + y * y <= radius * radius;
        }
        // Even-odd rule.
        bool inside = false;
        for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            const auto& a = polygon[i];
            const auto& b = polygon[j];
            if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]) {
                inside = !inside;
            }
        }
        return inside;
    }
};

/// The field `name` of the sensor: `<name>/polygon` ("x,y x,y x,y ..." in m), `<name>/radius`
/// (m), or the rectangle `<name>/x_min`, `x_max`, `y_min`, `y_max` (m). Empty when none is given;
/// false when one is malformed.
bool readField(const World& world, std::uint32_t sensor, const std::string& name, Field& out) {
    const std::string polygon = kindText(world, sensor, name + "/polygon");
    if (!polygon.empty()) {
        std::string text = polygon;
        for (char& c : text) {
            if (c == ',' || c == ';') {
                c = ' ';
            }
        }
        std::istringstream in(text);
        double x = 0.0;
        double y = 0.0;
        while (in >> x >> y) {
            out.polygon.push_back({x, y});
        }
        return out.polygon.size() >= 3 && in.eof();
    }
    out.radius = kindSetting(world, sensor, name + "/radius", 0.0);
    if (out.radius > 0.0) {
        return true;
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double x0 = kindSetting(world, sensor, name + "/x_min", 0.0);
    const double x1 = kindSetting(world, sensor, name + "/x_max", nan);
    const double y0 = kindSetting(world, sensor, name + "/y_min", nan);
    const double y1 = kindSetting(world, sensor, name + "/y_max", nan);
    if (std::isnan(x1) && std::isnan(y0) && std::isnan(y1)) {
        return true;  // no field
    }
    if (!(x1 > x0) || !(y1 > y0)) {
        return false;
    }
    out.polygon = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    return true;
}

class SafetyScanner final : public KindModule {
public:
    SafetyScanner(const World& world, std::uint32_t sensor, bool groundTruth)
        : KindModule(world, sensor, groundTruth) {}

    Status declare(const WorldView& world, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = world.raw();
        const std::string fail = "safety_scanner " + name_ + ": ";
        aperture_ = kindSetting(raw, sensor_, "fov", kNans3Aperture);
        const double resolution = kindSetting(raw, sensor_, "angular_resolution", kNans3Resolution);
        minRange_ = kindSetting(raw, sensor_, "min_range", 0.0);
        maxRange_ = kindSetting(raw, sensor_, "max_range", kNans3Range);
        noise_ = kindSetting(raw, sensor_, "range_noise", 0.0);
        response_ = kindSetting(raw, sensor_, "response_time", kNans3ResponseTime);
        if (aperture_ <= 0.0 || aperture_ > 2.0 * std::numbers::pi || resolution <= 0.0 ||
            maxRange_ <= minRange_ || response_ < 0.0 || response_ > kMaxResponseTime) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail +
                                     "needs fov in (0, 2π], angular_resolution above 0, "
                                     "max_range above min_range and response_time from 0 to 60 s");
        }
        beams_ = static_cast<std::uint32_t>(std::floor(aperture_ / resolution + 1e-9)) + 1;
        if (beams_ > kMaxBeams) {
            return Status::error(StatusCode::InvalidConfig, fail + "too many beams");
        }
        if (!readField(raw, sensor_, "protective", protective_) ||
            !readField(raw, sensor_, "warning", warning_)) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail +
                                     "a field is malformed (polygon: at least three \"x,y\" "
                                     "points; rectangle: x_max > x_min, y_max > y_min)");
        }
        // Speed-dependent field switching (two monitoring cases): at or above switch_speed
        // forward (m/s, along the sensor's x), the `_fast` fields watch.
        switchSpeed_ = kindSetting(raw, sensor_, "switch_speed", 0.0);
        if (switchSpeed_ > 0.0) {
            if (!readField(raw, sensor_, "protective_fast", protectiveFast_) ||
                !readField(raw, sensor_, "warning_fast", warningFast_) || protectiveFast_.empty()) {
                return Status::error(StatusCode::InvalidConfig,
                                     fail + "switch_speed needs a protective_fast field");
            }
            if (warningFast_.empty()) {
                warningFast_ = warning_;
            }
        }
        if (protective_.empty()) {
            return Status::error(StatusCode::InvalidConfig,
                                 fail +
                                     "has no protective field (protective/x_max ..., "
                                     "protective/radius or protective/polygon)");
        }
        // The scanner's own housing does not block its window.
        scene_.declare(raw, d, body_, body_);
        if (switchSpeed_ > 0.0) {
            if (!scene_.poses().moves(body_)) {
                return Status::error(StatusCode::InvalidConfig,
                                     fail + "switch_speed on a body that never moves");
            }
            twist_ = d.input(kindSpec("body/" + raw.bodies.names[body_] + "/twist",
                                      SignalType::Wrench, units::m_per_s, "world_local"));
            case_ = d.output(kindSpec(base() + "/case", SignalType::Int, units::one));
        }
        protectiveOut_ = d.output(kindSpec(base() + "/protective", SignalType::Bool, units::one));
        warningOut_ = d.output(kindSpec(base() + "/warning", SignalType::Bool, units::one));
        minDistance_ = d.output(kindSpec(base() + "/min_distance", SignalType::Scalar, units::m));
        SignalSpec ranges = kindSpec(base() + "/ranges", SignalType::ScalarArray, units::m);
        ranges.arrayLength = beams_;
        ranges_ = d.output(ranges);
        return {};
    }

    Status init(const WorldView& world, const ModuleConfig&, SignalBus& bus) override {
        scene_.init(world.raw());
        directions_.resize(beams_);
        const double step = beams_ > 1 ? aperture_ / (beams_ - 1) : 0.0;
        for (std::uint32_t i = 0; i < beams_; ++i) {
            const double a = -aperture_ / 2.0 + i * step;
            directions_[i] = {std::cos(a), std::sin(a), 0.0};
        }
        range_.assign(beams_, kNoHit);
        // Scans until an output follows what was seen: the response time in whole scans.
        delay_ = groundTruth_
                     ? 0
                     : static_cast<std::size_t>(std::ceil(response_ / step_.seconds() - 1e-9));
        history_.assign(delay_ + 1, 0);
        bus.writeN(ranges_, range_);
        bus.write(protectiveOut_, 0.0);
        bus.write(warningOut_, 0.0);
        bus.write(minDistance_, kNoHit);
        if (case_.valid()) {
            bus.write(case_, 0.0);
        }
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        scene_.place(bus);
        const Pose sensor = compose(scene_.poses().at(bus, body_), mount_);
        // The monitoring case by the speed along the sensor's x (the body's linear velocity at
        // its origin; a scanner far off a turning body's axis sees a little more or less).
        bool fast = false;
        if (switchSpeed_ > 0.0) {
            const std::span<const double> t = bus.readN(twist_);
            const std::array<double, 3> ahead = rotate(sensor.orientation, {1.0, 0.0, 0.0});
            fast = t[0] * ahead[0] + t[1] * ahead[1] + t[2] * ahead[2] >= switchSpeed_;
            bus.write(case_, fast ? 1.0 : 0.0);
        }
        const Field& protective = fast ? protectiveFast_ : protective_;
        const Field& warning = fast ? warningFast_ : warning_;
        bool inProtective = false;
        bool inWarning = false;
        double nearest = kNoHit;
        for (std::uint32_t i = 0; i < beams_; ++i) {
            double r = scene_.cast(sensor.position, rotate(sensor.orientation, directions_[i]));
            if (!groundTruth_ && noise_ > 0.0 && std::isfinite(r)) {
                random::Stream rng(context.seed, context.stepIndex, i);
                r += noise_ * rng.normal();
            }
            if (r < minRange_ || r > maxRange_) {
                r = kNoHit;
            }
            range_[i] = r;
            if (!std::isfinite(r)) {
                continue;
            }
            nearest = std::min(nearest, r);
            const double x = r * directions_[i][0];
            const double y = r * directions_[i][1];
            inProtective = inProtective || protective.contains(x, y);
            inWarning = inWarning || (!warning.empty() && warning.contains(x, y));
        }
        // The outputs lag what this scan saw by the response time (a ring of scan states).
        const std::uint8_t now = (inProtective ? 1U : 0U) | (inWarning ? 2U : 0U);
        history_[head_] = now;
        head_ = (head_ + 1) % history_.size();
        const std::uint8_t shown = history_[head_];  // the oldest: delay_ scans ago
        bus.write(protectiveOut_, (shown & 1U) != 0 ? 1.0 : 0.0);
        bus.write(warningOut_, (shown & 2U) != 0 ? 1.0 : 0.0);
        bus.write(minDistance_, nearest);
        bus.writeN(ranges_, range_);
        return {};
    }

    Status saveState(StateWriter& w) const override {
        w.put<std::uint64_t>(head_);
        w.putBytes(history_.data(), history_.size());
        return {};
    }
    Status restoreState(StateReader& r) override {
        std::uint64_t head = 0;
        r.get(head);
        r.getBytes(history_.data(), history_.size());
        head_ = static_cast<std::size_t>(head) % history_.size();
        return r.failed() ? Status::error(StatusCode::InvalidConfig, "short state") : Status{};
    }

private:
    KindScene scene_;
    Field protective_, warning_, protectiveFast_, warningFast_;
    double switchSpeed_{0.0};
    SignalId twist_, case_;
    double aperture_{0.0}, minRange_{0.0}, maxRange_{0.0}, noise_{0.0}, response_{0.0};
    std::uint32_t beams_{0};
    std::vector<std::array<double, 3>> directions_;
    std::vector<double> range_;
    std::vector<std::uint8_t> history_;
    std::size_t head_{0}, delay_{0};
    SignalId protectiveOut_, warningOut_, minDistance_, ranges_;
};

}  // namespace

std::unique_ptr<IModule> makeSafetyScanner(const World& world, std::uint32_t sensor,
                                           bool groundTruth, const KindContext&) {
    return std::make_unique<SafetyScanner>(world, sensor, groundTruth);
}

}  // namespace lagrange::sensors::kinds
