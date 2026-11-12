// event_camera (sensors.md §9): a dynamic vision sensor (DAVIS346, Prophesee) from successive
// rendered frames, as ESIM and v2e make events (CameraKinds.hpp, EventGenerator).
#include "CameraKinds.hpp"
#include "lagrange/module/StateIO.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

// iniVation DAVIS346: 346 × 260 pixels. Contrast threshold and mismatch are v2e's defaults
// (Hu, Liu and Delbruck 2021: 0.2 ln and σ 0.03 ln); the refractory period and the background
// activity rate are illustrative (TODO(verify) against a DAVIS346 datasheet or v2e's defaults).
constexpr double kDefaultWidth = 346.0;
constexpr double kDefaultHeight = 260.0;
constexpr double kDefaultFov = 1.22;  // rad, about 70°: a 4.5 mm lens on its 6.9 mm wide array
constexpr double kDefaultThreshold = 0.2;
constexpr double kDefaultThresholdSigma = 0.03;
constexpr double kDefaultRefractoryS = 1e-4;
constexpr double kDefaultNoiseRateHz = 0.1;
constexpr double kDefaultMaxEvents = 20000.0;
constexpr std::size_t kEventFields = 4;  // x, y, t, polarity
// The event image: grey where nothing happened, brighter for ON, darker for OFF.
constexpr int kEventGrey = 128;
constexpr int kEventStep = 64;

class EventCamera final : public KindModule {
public:
    EventCamera(const World& world, std::uint32_t sensor, bool groundTruth,
                std::shared_ptr<const CameraScene> scene)
        : KindModule(world, sensor, groundTruth), render_(std::move(scene)) {
        step_ = wholeMillisecondPeriod(world.sensors.rateHz[sensor]);
    }

    [[nodiscard]] ModuleInfo info() const override {
        ModuleInfo out = KindModule::info();
        out.deterministic = groundTruth_;  // rendering is a GPU path (invariant 7)
        return out;
    }

    Status declare(const WorldView& view, const ModuleConfig&, SignalDeclarer& d) override {
        const World& raw = view.raw();
        const double width = cameraSetting(raw, sensor_, "width", kDefaultWidth);
        const double height = cameraSetting(raw, sensor_, "height", kDefaultHeight);
        const double fov = cameraSetting(raw, sensor_, "horizontal_fov", kDefaultFov);
        EventConfig& c = config_;
        c.thresholdOn = kindSetting(raw, sensor_, "threshold_on", kDefaultThreshold);
        c.thresholdOff = kindSetting(raw, sensor_, "threshold_off", kDefaultThreshold);
        c.thresholdSigma = kindSetting(raw, sensor_, "threshold_sigma", kDefaultThresholdSigma);
        c.refractory = kindSetting(raw, sensor_, "refractory_period", kDefaultRefractoryS);
        c.noiseRate = kindSetting(raw, sensor_, "noise_rate", kDefaultNoiseRateHz);
        const double maxEvents = kindSetting(raw, sensor_, "max_events", kDefaultMaxEvents);
        if (width < 1.0 || height < 1.0 || width > 4096.0 || height > 4096.0 || fov <= 0.0 ||
            fov >= 3.1 || c.thresholdOn <= 0.0 || c.thresholdOff <= 0.0 || c.thresholdSigma < 0.0 ||
            c.refractory < 0.0 || c.noiseRate < 0.0 || maxEvents < 1.0 || maxEvents > 1e6) {
            return Status::error(StatusCode::InvalidConfig,
                                 "event_camera " + name_ + ": a setting is out of range");
        }
        if (groundTruth_) {
            c.thresholdSigma = 0.0;
            c.noiseRate = 0.0;
        }
        c.width = static_cast<std::uint32_t>(width);
        c.height = static_cast<std::uint32_t>(height);
        camera_ = pinhole(c.width, c.height, fov, cameraSetting(raw, sensor_, "near_clip", 0.05));
        maxEvents_ = static_cast<std::size_t>(maxEvents);
        render_.declare(raw, body_, d);
        SignalSpec events = kindSpec(base() + "/events", SignalType::ScalarArray, units::one);
        events.arrayLength = static_cast<std::uint32_t>(maxEvents_ * kEventFields);
        events_ = d.output(events);
        count_ = d.output(kindSpec(base() + "/count", SignalType::Int, units::one));
        dropped_ = d.output(kindSpec(base() + "/dropped", SignalType::Int, units::one));
        image_ = d.output(kindSpec(base() + "/image", SignalType::Image, Unit{}, "optical"));
        calibration_.declare(d, base() + "/image");
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        generator_ = std::make_unique<EventGenerator>(config_);
        if (!groundTruth_ && !render_.init(config_.width, config_.height)) {
            return Status::error(StatusCode::Unsupported,
                                 "event_camera " + name_ +
                                     " needs a Vulkan device, and none was usable (set the "
                                     "sensors domain to ground_truth to run without one)");
        }
        write(bus, {});
        calibration_.write(bus, camera_);
        bus.publishBuffer(image_, imageBuffer(config_.width, config_.height, "rgba8",
                                              std::vector<std::uint8_t>{}));
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            write(bus, {});  // the substitute renders nothing, so it sees no change
            return {};
        }
        const Pose sensor = compose(render_.bodyPose(bus, body_), mount_);
        camera_.position = sensor.position;
        camera_.orientation = multiply(sensor.orientation, kSensorFromOptical);
        // The frame at this step's time, from the poses committed by then (as the camera).
        const double t = context.t.seconds();
        render_.render(bus, camera_, frame_, t);
        const std::size_t n = static_cast<std::size_t>(config_.width) * config_.height;
        logs_.resize(n);
        for (std::size_t i = 0; i < n && i * 4 + 3 < frame_.rgba.size(); ++i) {
            logs_[i] = linLog(frame_.rgba[i * 4], frame_.rgba[i * 4 + 1], frame_.rgba[i * 4 + 2]);
        }
        frameEvents_.clear();
        generator_->feed(logs_, t, context.seed, context.stepIndex, frameEvents_);
        write(bus, frameEvents_);
        // The event image: the events of this step, accumulated per pixel.
        std::vector<int> sum(n, 0);
        for (const DvsEvent& e : frameEvents_) {
            sum[static_cast<std::size_t>(e.y) * config_.width + e.x] += e.polarity;
        }
        std::vector<std::uint8_t> rgba(n * 4, 255);
        for (std::size_t i = 0; i < n; ++i) {
            const auto g =
                static_cast<std::uint8_t>(std::clamp(kEventGrey + kEventStep * sum[i], 0, 255));
            rgba[i * 4] = g;
            rgba[i * 4 + 1] = g;
            rgba[i * 4 + 2] = g;
        }
        bus.publishBuffer(image_, imageBuffer(config_.width, config_.height, "rgba8", rgba));
        return {};
    }

    Status saveState(StateWriter& writer) const override {
        if (generator_) {
            generator_->save(writer);
        }
        return {};
    }
    Status restoreState(StateReader& reader) override {
        if (generator_ && !generator_->restore(reader)) {
            return Status::error(StatusCode::Internal,
                                 "event_camera " + name_ + ": truncated state");
        }
        return {};
    }

private:
    /// Packs the first max_events events as (x, y, t, polarity), NaN after them.
    void write(SignalBus& bus, const std::vector<DvsEvent>& events) {
        packed_.assign(maxEvents_ * kEventFields, std::numeric_limits<double>::quiet_NaN());
        const std::size_t kept = std::min(events.size(), maxEvents_);
        for (std::size_t k = 0; k < kept; ++k) {
            packed_[k * kEventFields] = events[k].x;
            packed_[k * kEventFields + 1] = events[k].y;
            packed_[k * kEventFields + 2] = events[k].t;
            packed_[k * kEventFields + 3] = events[k].polarity;
        }
        bus.writeN(events_, packed_);
        bus.write(count_, static_cast<double>(kept));
        bus.write(dropped_, static_cast<double>(events.size() - kept));
    }

    SceneRender render_;
    EventConfig config_;
    gpu::PinholeCamera camera_;
    std::size_t maxEvents_{0};
    std::unique_ptr<EventGenerator> generator_;
    gpu::RasterImages frame_;
    std::vector<float> logs_;
    std::vector<DvsEvent> frameEvents_;
    std::vector<double> packed_;
    SignalId events_, count_, dropped_, image_;
    ImageCalibration calibration_;
};

}  // namespace

std::unique_ptr<IModule> createEventCamera(const World& world, std::uint32_t sensor,
                                           bool groundTruth, const KindContext& context) {
    return std::make_unique<EventCamera>(world, sensor, groundTruth, context.cameraScene);
}

}  // namespace lagrange::sensors::kinds
