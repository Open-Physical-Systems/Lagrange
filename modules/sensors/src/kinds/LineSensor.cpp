// line_sensor (sensors.md §9): an AGV guide sensor, a row of channels across the sensor's y that
// look down (the sensor's -z) for tape on the floor: a reflective line-follower array or a
// magnetic tape sensor, at the detector level. Tape is a scene prim whose name has the word
// "tape" or "line" (the box of its mesh), or a collider whose surface material is as bright as a
// tape (SensorMaterialAPI_1 reflectivity at 905 nm, the near infrared such arrays use).
#include "Places.hpp"
#include "lagrange/random/Philox.hpp"

#include <cmath>
#include <limits>

namespace lagrange::sensors::kinds {

namespace {

constexpr double kDefaultChannels = 16.0;
constexpr double kDefaultSpacing = 0.01;          // m between channel centres: illustrative
constexpr double kDefaultSamples = 5.0;           // rays across each channel's width
constexpr double kDefaultMaxRange = 0.1;          // m, how far below the sensor tape is seen
constexpr double kDefaultNoise = 0.02;            // activation, 1 sigma: illustrative
constexpr double kDefaultThreshold = 0.2;         // activation a channel counts from
constexpr double kDefaultTapeReflectivity = 0.8;  // white tape on a grey floor (0.5 default)
constexpr double kSurfaceTolerance = 1e-3;        // m: tape flush with the floor still shows
constexpr double kMinHalfThickness = 1e-4;        // m

struct Tape {
    Place place;
};

class LineSensor final : public KindModule {
public:
    LineSensor(const World& world, std::uint32_t sensor, bool groundTruth, std::vector<Tape> tapes)
        : KindModule(world, sensor, groundTruth), tapes_(std::move(tapes)) {}

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& world = view.raw();
        const double channels = kindSetting(world, sensor_, "channels", kDefaultChannels);
        spacing_ = kindSetting(world, sensor_, "spacing", kDefaultSpacing);
        const double samples = kindSetting(world, sensor_, "samples_per_channel", kDefaultSamples);
        maxRange_ = kindSetting(world, sensor_, "max_range", kDefaultMaxRange);
        noise_ = kindSetting(world, sensor_, "noise", kDefaultNoise);
        threshold_ = kindSetting(world, sensor_, "threshold", kDefaultThreshold);
        tapeReflectivity_ =
            kindSetting(world, sensor_, "tape_reflectivity", kDefaultTapeReflectivity);
        darkTape_ = kindSetting(world, sensor_, "dark_tape", 0.0) != 0.0;
        if (channels < 1.0 || channels > 1024.0 || spacing_ <= 0.0 || samples < 1.0 ||
            samples > 64.0 || maxRange_ <= 0.0 || noise_ < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "line_sensor " + name_ +
                                     ": channels, spacing, samples_per_channel, max_range or "
                                     "noise out of range");
        }
        channels_ = static_cast<std::uint32_t>(channels);
        samples_ = static_cast<std::uint32_t>(samples);
        materials_ = world.colliders.material;
        tapeMaterial_.assign(world.surfaceMaterials.size(), false);
        for (std::size_t m = 0; m < world.surfaceMaterials.size(); ++m) {
            const double r = world.surfaceMaterials[m].reflectivity905;
            tapeMaterial_[m] = darkTape_ ? r <= tapeReflectivity_ : r >= tapeReflectivity_;
        }
        poses_.begin(world);
        poses_.watch(body_, d);
        for (const Tape& t : tapes_) {
            poses_.watch(t.place.body, d);
        }
        colliders_.declare(world, poses_, d);
        SignalSpec out = kindSpec(base() + "/channels", SignalType::ScalarArray, units::one);
        out.arrayLength = channels_;
        channelsOut_ = d.output(std::move(out));
        offset_ = d.output(kindSpec(base() + "/offset", SignalType::Scalar, units::m));
        width_ = d.output(kindSpec(base() + "/width", SignalType::Scalar, units::m));
        detected_ = d.output(kindSpec(base() + "/detected", SignalType::Bool, units::one));
        activation_.assign(channels_, 0.0);
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        std::fill(activation_.begin(), activation_.end(), 0.0);
        write(bus);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        const Pose sensor = compose(poses_.pose(bus, body_), mount_);
        colliders_.update(bus, poses_);
        placed_.resize(tapes_.size());
        for (std::size_t k = 0; k < tapes_.size(); ++k) {
            const Place& p = tapes_[k].place;
            placed_[k] = compose(poses_.place(bus, p), Pose{p.center});
        }
        const Vec3 down = rotate(sensor.orientation, {0.0, 0.0, -1.0});
        random::Stream rng(context.seed, context.stepIndex, sensor_);
        for (std::uint32_t c = 0; c < channels_; ++c) {
            // Channel 0 is the leftmost (+y), the last the rightmost.
            const double centre = ((channels_ - 1) / 2.0 - c) * spacing_;
            std::uint32_t on = 0;
            for (std::uint32_t s = 0; s < samples_; ++s) {
                const double y = centre + ((s + 0.5) / samples_ - 0.5) * spacing_;
                const Vec3 origin = add(sensor.position, rotate(sensor.orientation, {0.0, y, 0.0}));
                if (sees(origin, down)) {
                    ++on;
                }
            }
            double a = static_cast<double>(on) / samples_;
            if (!groundTruth_ && noise_ > 0.0) {
                a = std::clamp(a + noise_ * rng.normal(), 0.0, 1.0);
            }
            activation_[c] = a;
        }
        write(bus);
        return {};
    }

private:
    /// Whether the ray from `origin` down meets tape first, within range.
    [[nodiscard]] bool sees(const Vec3& origin, const Vec3& down) const {
        const services::RayHit floor = colliders_.castPast(origin, down, body_);
        const double floorAt = floor.hit() ? floor.distance : kNoHit;
        if (floor.hit() && floorAt <= maxRange_ && floor.shape < materials_.size()) {
            const std::uint32_t m = materials_[floor.shape];
            if (m != kNoMaterial && m < tapeMaterial_.size() && tapeMaterial_[m]) {
                return true;
            }
        }
        for (std::size_t k = 0; k < tapes_.size(); ++k) {
            const double t =
                raycast(ShapeType::Box, tapes_[k].place.half, placed_[k], origin, down);
            if (t <= maxRange_ && t <= floorAt + kSurfaceTolerance) {
                return true;
            }
        }
        return false;
    }

    void write(SignalBus& bus) {
        double sum = 0.0;
        double moment = 0.0;
        double cover = 0.0;
        for (std::uint32_t c = 0; c < channels_; ++c) {
            const double a = activation_[c];
            if (a >= threshold_) {
                const double y = ((channels_ - 1) / 2.0 - c) * spacing_;
                sum += a;
                moment += a * y;
                cover += a * spacing_;
            }
        }
        bus.writeN(channelsOut_, activation_);
        const bool found = sum > 0.0;
        bus.write(detected_, found ? 1.0 : 0.0);
        bus.write(offset_, found ? moment / sum : std::numeric_limits<double>::quiet_NaN());
        bus.write(width_, cover);
    }

    std::vector<Tape> tapes_;
    std::vector<Pose> placed_;
    std::uint32_t channels_{16}, samples_{5};
    double spacing_{kDefaultSpacing}, maxRange_{kDefaultMaxRange}, noise_{kDefaultNoise};
    double threshold_{kDefaultThreshold}, tapeReflectivity_{kDefaultTapeReflectivity};
    bool darkTape_{false};
    std::vector<std::uint32_t> materials_;
    std::vector<bool> tapeMaterial_;
    std::vector<double> activation_;
    BodyPoses poses_;
    Colliders colliders_;
    SignalId channelsOut_, offset_, width_, detected_;
};

}  // namespace

std::unique_ptr<IModule> createLineSensor(const World& world, std::uint32_t sensor,
                                          bool groundTruth, const KindContext& context) {
    // Words that make a prim tape, separated by '|' (default "tape|line").
    const std::vector<std::string> wanted = [&] {
        std::vector<std::string> out;
        std::string text = kindText(world, sensor, "pattern", "tape|line");
        std::string word;
        for (const char c : text + "|") {
            if (c == '|' || c == ',') {
                if (!word.empty()) {
                    out.push_back(lowerCase(word));
                }
                word.clear();
            } else if (c != ' ') {
                word.push_back(c);
            }
        }
        return out;
    }();
    std::vector<Tape> tapes;
    for (Place& p : scenePlaces(world, context.scene)) {
        if (!p.hasBox) {
            continue;
        }
        bool match = false;
        for (const std::string& w : words(p.name)) {
            match = match || std::ranges::find(wanted, w) != wanted.end();
        }
        // A tape's mesh child ("visual") is part of it: only the named prim counts, once.
        if (match) {
            for (double& h : p.half) {
                h = std::max(h, kMinHalfThickness);  // a flat quad is still a box to a ray
            }
            tapes.push_back({std::move(p)});
        }
    }
    return std::make_unique<LineSensor>(world, sensor, groundTruth, std::move(tapes));
}

}  // namespace lagrange::sensors::kinds
