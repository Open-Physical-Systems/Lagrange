// stereo_camera (sensors.md §9): an active stereo depth camera like the RealSense D435: left and
// right infrared images at a baseline and the left camera's depth with the stereo error model
// (CameraKinds.hpp, stereoDepth). It does not match the images: the depth is the rendered depth
// with the error a matcher of the given sub-pixel accuracy makes, and the holes it leaves.
#include "CameraKinds.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <cmath>
#include <vector>

namespace lagrange::sensors::kinds {

namespace {

// RealSense D400 Series Datasheet (document 337029, March 2026), D435: table 3-49, 50 mm baseline
// and depth field of view 87° × 58°; 848 × 480 a depth mode; depth unit 1 mm. The white paper
// "Tuning depth cameras for best performance" (Grunnet-Jepsen et al., Intel RealSense): min-Z =
// f · b / 126 (the disparity search range), RMS error z² · subpixel / (f · b), plotted for
// subpixel 0.08 px. The 10 m limit is illustrative: the datasheet gives "0.2 m to over 3 m
// (varies with lighting conditions)" and the white paper up to about 10 m on a white wall; the
// error simply grows as z².
constexpr double kD435Width = 848.0;
constexpr double kD435Height = 480.0;
constexpr double kD435HorizontalFov = 87.0 * 3.14159265358979323846 / 180.0;
constexpr double kD435BaselineM = 0.050;
constexpr double kD435MaxDisparityPx = 126.0;
constexpr double kD400SubpixelPx = 0.08;
constexpr double kD400DepthUnitM = 0.001;
constexpr double kDefaultMaxRangeM = 10.0;

class StereoCamera final : public KindModule {
public:
    StereoCamera(const World& world, std::uint32_t sensor, bool groundTruth,
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
        const double width = cameraSetting(raw, sensor_, "width", kD435Width);
        const double height = cameraSetting(raw, sensor_, "height", kD435Height);
        const double fov = cameraSetting(raw, sensor_, "horizontal_fov", kD435HorizontalFov);
        StereoConfig& c = config_;
        c.baseline = kindSetting(raw, sensor_, "baseline", kD435BaselineM);
        c.subpixel = kindSetting(raw, sensor_, "subpixel", kD400SubpixelPx);
        c.maxDisparity = kindSetting(raw, sensor_, "max_disparity", kD435MaxDisparityPx);
        c.maxRange = kindSetting(raw, sensor_, "max_range", kDefaultMaxRangeM);
        c.depthUnit = kindSetting(raw, sensor_, "depth_unit", kD400DepthUnitM);
        if (width < 2.0 || height < 1.0 || width > 8192.0 || height > 8192.0 || fov <= 0.0 ||
            fov >= 3.1 || c.baseline <= 0.0 || c.subpixel < 0.0 || c.maxDisparity <= 0.0 ||
            c.maxRange <= 0.0 || c.depthUnit < 0.0) {
            return Status::error(StatusCode::InvalidConfig,
                                 "stereo_camera " + name_ + ": a setting is out of range");
        }
        camera_ = pinhole(static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height),
                          fov, cameraSetting(raw, sensor_, "near_clip", 0.05));
        c.fx = camera_.fx;
        c.cx = camera_.cx;
        c.cy = camera_.cy;
        render_.declare(raw, body_, d);
        depth_ = d.output(kindSpec(base() + "/depth", SignalType::Image, units::m, "optical"));
        left_ = d.output(kindSpec(base() + "/left", SignalType::Image, Unit{}, "optical"));
        right_ = d.output(kindSpec(base() + "/right", SignalType::Image, Unit{}, "optical"));
        depthCalibration_.declare(d, base() + "/depth");
        leftCalibration_.declare(d, base() + "/left");
        rightCalibration_.declare(d, base() + "/right");
        return {};
    }

    Status init(const WorldView&, const ModuleConfig&, SignalBus& bus) override {
        if (!groundTruth_ && !render_.init(camera_.width, camera_.height)) {
            return Status::error(StatusCode::Unsupported,
                                 "stereo_camera " + name_ +
                                     " needs a Vulkan device, and none was usable (set the "
                                     "sensors domain to ground_truth to run without one)");
        }
        depthCalibration_.write(bus, camera_);
        leftCalibration_.write(bus, camera_);
        rightCalibration_.write(bus, camera_, {config_.baseline, 0.0, 0.0});
        publish(bus);
        return {};
    }

    Status step(const StepContext& context, SignalBus& bus) override {
        if (groundTruth_) {
            publish(bus);
            return {};
        }
        const Pose sensor = compose(render_.bodyPose(bus, body_), mount_);
        const std::array<double, 4> optical = multiply(sensor.orientation, kSensorFromOptical);
        gpu::PinholeCamera view = camera_;
        view.position = sensor.position;
        view.orientation = optical;
        const double t = context.t.seconds();
        render_.render(bus, view, leftImages_, t);
        // The right imager: the baseline along the optical x (to the right in the image).
        const Vec3 shift = rotate(optical, {config_.baseline, 0.0, 0.0});
        view.position = {sensor.position[0] + shift[0], sensor.position[1] + shift[1],
                         sensor.position[2] + shift[2]};
        render_.render(bus, view, rightImages_, t);
        stereoDepth(config_, camera_.width, camera_.height, leftImages_.depth, rightImages_.depth,
                    true, context.seed, context.stepIndex, depthPixels_);
        mono(leftImages_.rgba, leftPixels_);
        mono(rightImages_.rgba, rightPixels_);
        publish(bus);
        return {};
    }

private:
    /// The rendered colour as a grey infrared-like image (rgba8, R = G = B = Rec. 709 luma).
    static void mono(const std::vector<std::uint8_t>& rgba, std::vector<std::uint8_t>& out) {
        out.resize(rgba.size());
        for (std::size_t i = 0; i + 3 < rgba.size(); i += 4) {
            const auto y = static_cast<std::uint8_t>(
                std::lround(0.2126 * rgba[i] + 0.7152 * rgba[i + 1] + 0.0722 * rgba[i + 2]));
            out[i] = y;
            out[i + 1] = y;
            out[i + 2] = y;
            out[i + 3] = 255;
        }
    }

    void publish(SignalBus& bus) {
        bus.publishBuffer(depth_, imageBuffer(camera_.width, camera_.height, "r32f", depthPixels_));
        bus.publishBuffer(left_, imageBuffer(camera_.width, camera_.height, "rgba8", leftPixels_));
        bus.publishBuffer(right_,
                          imageBuffer(camera_.width, camera_.height, "rgba8", rightPixels_));
    }

    SceneRender render_;
    StereoConfig config_;
    gpu::PinholeCamera camera_;
    gpu::RasterImages leftImages_, rightImages_;
    std::vector<float> depthPixels_;
    std::vector<std::uint8_t> leftPixels_, rightPixels_;
    SignalId depth_, left_, right_;
    ImageCalibration depthCalibration_, leftCalibration_, rightCalibration_;
};

}  // namespace

std::unique_ptr<IModule> createStereoCamera(const World& world, std::uint32_t sensor,
                                            bool groundTruth, const KindContext& context) {
    return std::make_unique<StereoCamera>(world, sensor, groundTruth, context.cameraScene);
}

}  // namespace lagrange::sensors::kinds
