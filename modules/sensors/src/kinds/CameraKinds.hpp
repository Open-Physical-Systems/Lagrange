#pragma once
// Private to lagrange_sensors: the models behind the camera-like built-in kinds (sensors.md §9):
// optical_flow, event_camera and stereo_camera. Pure functions and small classes, so the tests
// check them against closed forms without a GPU.

#include "KindModule.hpp"
#include "lagrange/gpu/Raster.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace lagrange::sensors::kinds {
// Its own inline namespace: the kinds' private headers may name helpers alike (ODR).
inline namespace camera_kinds {

using Vec3 = std::array<double, 3>;

// --- optical_flow ---------------------------------------------------------------------------

/// The flow rate (rad/s) a flow sensor measures about its optical x and y axes (optical frame:
/// z along the view, x right, y down), in PX4's `SensorOpticalFlow.pixel_flow` sign: a
/// right-handed rotation of the sensor about an axis gives positive flow about it, so the
/// gyro's delta angle is what a pure rotation reads (PX4-Autopilot msg/SensorOpticalFlow.msg;
/// ekf2 negates both before fusing them, EKF2.cpp). For a ray at normalised image coordinates
/// (x, y) = (X/Z, Y/Z) hitting a point at depth Z (m, along the optical z) the image moves by
/// the Longuet-Higgins and Prazdny (1980) motion field
///   ẋ = (−v_x + x v_z)/Z + x y ω_x − (1 + x²) ω_y + y ω_z
///   ẏ = (−v_y + y v_z)/Z + (1 + y²) ω_x − x y ω_y − x ω_z
/// with v (m/s) and ω (rad/s) the sensor's velocity and angular velocity in the optical frame;
/// in PX4's sign the flow is (ẏ, −ẋ). On the optical axis this is ω + (ẑ × v)/Z.
[[nodiscard]] std::array<double, 2> flowRate(const Vec3& velocity, const Vec3& angularVelocity,
                                             double x, double y, double depth);

// --- event_camera ---------------------------------------------------------------------------

/// One DVS event: pixel, time (s, sim time) and polarity (+1 brighter, −1 darker).
struct DvsEvent {
    std::uint32_t x{0}, y{0};
    double t{0.0};
    int polarity{0};
};

struct EventConfig {
    std::uint32_t width{0}, height{0};
    double thresholdOn{0.2};     // ln units: a log-intensity rise that fires an ON event
    double thresholdOff{0.2};    // ln units: a fall that fires an OFF event
    double thresholdSigma{0.0};  // ln units: per-pixel mismatch of both thresholds (1σ)
    double refractory{0.0};      // s: after an event the pixel stays blind this long
    double noiseRate{0.0};       // Hz per pixel: background activity (random polarity)
};

/// Events from successive log-intensity frames, as ESIM (Rebecq, Gehrig and Scaramuzza, "ESIM:
/// an Open Event Camera Simulator", CoRL 2018) and v2e (Hu, Liu and Delbruck, "v2e: From Video
/// Frames to Realistic DVS Events", CVPR Workshops 2021) make them: between two frames each
/// pixel's log intensity is linear in time; an event fires each time it crosses the pixel's
/// memorised level by its threshold, at the time of the crossing, and the level moves by the
/// threshold. A crossing inside the refractory period after the pixel's last event is held: the
/// level does not move and the change carries over to the next frame. Thresholds differ per
/// pixel by `thresholdSigma` (drawn once from the module seed), and background-activity noise
/// events fire at `noiseRate` per pixel at uniform times with random polarity.
class EventGenerator {
public:
    explicit EventGenerator(const EventConfig& config);

    /// Feeds the frame `logIntensity` (width × height, row-major) taken at time `t` (s); appends
    /// the events since the previous frame to `out`, sorted by time. The first frame only
    /// memorises the levels. Noise draws from Stream(seed, stepIndex, row).
    void feed(std::span<const float> logIntensity, double t, std::uint64_t seed,
              std::uint64_t stepIndex, std::vector<DvsEvent>& out);

    [[nodiscard]] bool started() const { return started_; }
    [[nodiscard]] double thresholdOn(std::size_t pixel) const { return on_[pixel]; }

    /// The state (levels, last frame, last event times) for save/restore.
    void save(StateWriter& writer) const;
    [[nodiscard]] bool restore(StateReader& reader);

private:
    void drawThresholds(std::uint64_t seed);

    EventConfig config_;
    bool started_{false};
    double lastT_{0.0};
    std::vector<float> level_;       // ln, the memorised level per pixel
    std::vector<float> last_;        // ln, the previous frame
    std::vector<double> lastEvent_;  // s, per pixel
    std::vector<float> on_, off_;    // ln, thresholds per pixel
};

/// A display-encoded (sRGB) RGBA8 pixel's log intensity as v2e takes it: linear luminance Y in
/// digital numbers (0..255, Rec. 709 weights), ln(Y) above kLinLogDn and a line below it that
/// meets ln at kLinLogDn, so that dark pixels do not fire on quantisation noise (v2e "lin_log").
inline constexpr double kLinLogDn = 20.0;  // DN, v2e's default linear-to-log knee
[[nodiscard]] float linLog(std::uint8_t r, std::uint8_t g, std::uint8_t b);

// --- stereo_camera --------------------------------------------------------------------------

/// The 1σ depth error (m) of a stereo pair at depth z (m): σ_z = z² σ_d / (f b), with σ_d the
/// sub-pixel disparity error (px), f the focal length (px) and b the baseline (m) (Grunnet-Jepsen
/// et al., "Tuning depth cameras for best performance", Intel RealSense D400 white paper,
/// 2019, §"RMS error").
[[nodiscard]] inline double stereoDepthSigma(double z, double subpixel, double focal,
                                             double baseline) {
    return z * z * subpixel / (focal * baseline);
}

struct StereoConfig {
    double fx{0.0}, cx{0.0}, cy{0.0};  // px, the left (depth) camera's, fy = fx
    double baseline{0.05};             // m, right camera at +baseline along optical x
    double subpixel{0.08};             // px, σ_d
    double maxDisparity{126.0};        // px, the search range: min_z = f b / maxDisparity
    double maxRange{10.0};             // m
    double depthUnit{0.001};           // m, the output quantum (D400: 1 mm by default)
    double occlusionTolerance{0.02};   // relative: a right depth this much nearer occludes
};

/// The stereo depth of the left camera: the true depth `left` (m, +inf for nothing) plus
/// Gaussian noise of σ_z, invalid where the right camera cannot see the point (outside its image
/// or behind something nearer in `right`, the true depth from the right camera) or where the
/// disparity exceeds the search range. As ROS REP 118: NaN invalid, −inf nearer than min-z, +inf
/// beyond range. `noise`: false for exact depth (ground truth checks). Rows draw from
/// Stream(seed, stepIndex, row).
void stereoDepth(const StereoConfig& config, std::uint32_t width, std::uint32_t height,
                 std::span<const float> left, std::span<const float> right, bool noise,
                 std::uint64_t seed, std::uint64_t stepIndex, std::vector<float>& out);

// --- rendering the camera scene, shared by event_camera and stereo_camera -------------------

/// The camera scene as a kind sees it: declares the body poses and hinge angles it is drawn
/// from (inputs, invariant 1) and renders pinhole views of it, as the camera does
/// (Sensors.cpp, `Camera`).
class SceneRender {
public:
    explicit SceneRender(std::shared_ptr<const CameraScene> scene) : scene_(std::move(scene)) {}

    /// Declares body/<b>/pose for `sensorBody` and every drawable's moving body, and the hinge
    /// angles. Call from declare().
    void declare(const World& world, std::uint32_t sensorBody, SignalDeclarer& d);
    /// Makes the renderer; false without a scene (no Vulkan device).
    [[nodiscard]] bool init(std::uint32_t width, std::uint32_t height);
    [[nodiscard]] Pose bodyPose(const SignalBus& bus, std::uint32_t b) const;
    /// Renders `camera` at the drawables' poses on `bus`.
    void render(const SignalBus& bus, const gpu::PinholeCamera& camera, gpu::RasterImages& out,
                double seconds);

private:
    std::shared_ptr<const CameraScene> scene_;
    std::vector<Pose> rest_;
    std::vector<SignalId> poses_;
    std::vector<SignalId> hinges_;
    std::unique_ptr<gpu::RasterCamera> renderer_;
    std::vector<gpu::RasterInstance> instances_;
};

/// A period rounded to whole milliseconds (at least 1 ms), as the camera's, so that a 30 Hz
/// sensor lines up with the other modules' steps.
[[nodiscard]] inline Duration wholeMillisecondPeriod(double rateHz) {
    const auto ms = std::max<std::int64_t>(std::llround(1e3 / std::max(rateHz, 1e-3)), 1);
    return Duration{ms * 1'000'000};
}

/// A camera-like kind's number `key`, also as SDFormat's camera block writes it
/// (`<camera><horizontal_fov>`, `<camera><image><width>`, `<camera><clip><near>`), else
/// `fallback`.
[[nodiscard]] inline double cameraSetting(const World& world, std::uint32_t sensor,
                                          const std::string& key, double fallback) {
    const auto& map = world.sensors.settings[sensor];
    for (const std::string& path :
         {key, "camera/" + key, "camera/image/" + key,
          key == "near_clip" ? std::string{"camera/clip/near"} : std::string{}}) {
        if (const auto it = map.find(path); !path.empty() && it != map.end()) {
            return it->second;
        }
    }
    return fallback;
}

/// A pinhole of `width` × `height` and horizontal field of view `fov` (rad), square pixels,
/// principal point at the centre.
[[nodiscard]] gpu::PinholeCamera pinhole(std::uint32_t width, std::uint32_t height, double fov,
                                         double nearClip);

/// The calibration of a kind's image, on the bus beside it so that every user (the ROS bridge's
/// CameraInfo, Python, recordings) reads it the same way (sensors.md §9 "Images of kinds"):
/// `sensor/<n>/<q>/intrinsics` (ScalarArray, px: fx, fy, cx, cy of the pinhole, OpenCV
/// convention, origin at the first pixel's centre) and `sensor/<n>/<q>/position` (Vec3, m, frame
/// "optical": where this image's camera centre is in the sensor's optical frame; a stereo pair's
/// right image sits at +baseline along x, which ROS writes as Tx = −fx·b in the right
/// CameraInfo's P, REP 104).
struct ImageCalibration {
    SignalId intrinsics, position;

    void declare(SignalDeclarer& d, const std::string& image) {
        SignalSpec k = kindSpec(image + "/intrinsics", SignalType::ScalarArray, units::one);
        k.arrayLength = 4;
        intrinsics = d.output(k);
        position = d.output(kindSpec(image + "/position", SignalType::Vec3, units::m, "optical"));
    }
    void write(SignalBus& bus, const gpu::PinholeCamera& camera, const Vec3& at = {}) const {
        const std::array<double, 4> k{camera.fx, camera.fy, camera.cx, camera.cy};
        bus.writeN(intrinsics, k);
        bus.writeN(position, at);
    }
};

/// An image buffer of `pixels` (empty: only the size).
template <typename T>
std::shared_ptr<Buffer> imageBuffer(std::uint32_t width, std::uint32_t height, const char* encoding,
                                    const std::vector<T>& pixels) {
    auto out = std::make_shared<Buffer>();
    out->width = width;
    out->height = height;
    out->encoding = encoding;
    const auto bytes = std::as_bytes(std::span{pixels});
    out->bytes.assign(bytes.begin(), bytes.end());
    return out;
}

}  // namespace camera_kinds
}  // namespace lagrange::sensors::kinds
