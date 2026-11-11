#pragma once

#include "lagrange/gpu/Resources.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

namespace lagrange::gpu {

class Device;

// --- the mesh pipeline shared by the viewer's viewport and the camera sensor ----------------
// (invariant 13: one shader, gpu/kernels/camera/mesh.slang, draws both)

/// Push constants of mesh.slang. Matrices are rows; positions are relative to the eye.
struct MeshPush {
    std::array<std::array<float, 4>, 4> viewProjection;
    std::array<float, 12> model;     // rows of the 3x4 model matrix
    std::array<float, 4> baseColor;  // linear RGB; alpha, or the instance id for fsSensor
};
static_assert(sizeof(MeshPush) == 128, "must match MeshPush in mesh.slang");

/// Rows of the 3x4 model matrix for a pose (rotation w, x, y, z) and scale, translated relative
/// to `eye`. The subtraction is done in double and converted last, so the GPU never sees a large
/// coordinate (invariant 6).
[[nodiscard]] std::array<float, 12> modelRows(const std::array<double, 3>& position,
                                              const std::array<double, 4>& rotation,
                                              const std::array<float, 3>& scale,
                                              const std::array<double, 3>& eye);

/// What differs between the graphics pipelines of the project. All use dynamic rendering,
/// reversed depth (D32, greater-or-equal) and a dynamic viewport.
struct PipelineSpec {
    std::span<const std::uint32_t> vertex;
    std::span<const std::uint32_t> fragment;
    VkPrimitiveTopology topology{VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST};
    std::uint32_t stride{24};  // bytes per vertex: position, then normal when hasNormal
    bool hasNormal{true};
    std::uint32_t pushSize{sizeof(MeshPush)};
    bool depthWrite{true};
    std::span<const VkFormat> colorFormats;
    bool blend{true};  // alpha blending on the first attachment
    VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
    VkPolygonMode polygonMode{VK_POLYGON_MODE_FILL};  // LINE needs Device::nonSolidFill()
    VkDescriptorSetLayout setLayout{VK_NULL_HANDLE};  // set 0, if the shaders have one
    bool depthTest{true};
};

inline constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

void buildPipeline(const Device& device, const PipelineSpec& spec, VkPipelineLayout& layout,
                   VkPipeline& pipeline);

/// Meshes on the GPU: interleaved position + normal (24 bytes), uint32 indices.
struct MeshSource {
    std::span<const float> positions;  // x, y, z per vertex
    std::span<const float> normals;    // empty: lit as if facing up
    std::span<const std::uint32_t> indices;
};

class MeshSet {
public:
    explicit MeshSet(const Device& device) : device_(device) {}
    ~MeshSet() { clear(); }
    MeshSet(const MeshSet&) = delete;
    MeshSet& operator=(const MeshSet&) = delete;

    /// Replaces the meshes. The GPU must not be using the previous ones.
    void upload(std::span<const MeshSource> meshes);
    void clear();
    [[nodiscard]] std::size_t size() const { return meshes_.size(); }
    /// Binds and draws mesh `index`; returns false for an empty one.
    bool draw(VkCommandBuffer commands, std::size_t index) const;
    /// Mesh `index`'s bounds in its own frame, min then max; null for an empty or unknown one.
    [[nodiscard]] const std::array<float, 6>* bounds(std::size_t index) const {
        return index < meshes_.size() && meshes_[index].indexCount != 0 ? &meshes_[index].box
                                                                        : nullptr;
    }

private:
    // Every mesh lives in one vertex and one index buffer, uploaded in one transfer each.
    struct GpuMesh {
        std::uint32_t firstIndex{0};
        std::int32_t vertexOffset{0};
        std::uint32_t indexCount{0};
        std::array<float, 6> box{};  // its vertices' bounds, min then max
    };
    const Device& device_;
    std::vector<GpuMesh> meshes_;
    Buffer vertices_;
    Buffer indices_;
};

// --- the camera sensor's renderer (docs/design/domains/sensors.md, camera level `raster`) ----

/// A pinhole camera in the OpenCV convention: optical frame z forward, x right, y down; pixel
/// (u, v) has its centre at integer coordinates, u = fx x / z + cx.
struct PinholeCamera {
    std::uint32_t width{0}, height{0};
    double fx{0}, fy{0}, cx{0}, cy{0};
    double near{0.05};                 // m; nothing nearer is drawn. There is no far plane.
    std::array<double, 3> position{};  // world
    std::array<double, 4> orientation{1, 0, 0, 0};  // world from optical frame, (w, x, y, z)
};

/// One thing to draw: a mesh of the scene at a world pose.
struct RasterInstance {
    std::uint32_t mesh{0};
    std::array<double, 3> position{};
    std::array<double, 4> rotation{1, 0, 0, 0};
    std::array<float, 3> scale{1, 1, 1};
    std::array<float, 4> color{0.8F, 0.8F, 0.8F, 1.0F};  // linear RGB
    std::uint32_t id{0};                                 // instance id, written to `instance`
    // For the realistic look.
    float metallic{0.0F};
    float roughness{0.6F};
    float pattern{0.0F};
    std::array<float, 3> emissive{0.0F, 0.0F, 0.0F};
    bool ground{false};
    bool water{false};
    std::array<float, 6> box{};  // water: the mesh's bounds, min then max
};

/// How a run's cameras draw colour. Depth and instance id are the same either way.
struct RasterLook {
    bool realistic{false};  // the viewport's realistic renderer; ignored on a software device
    int season{0};          // as RealisticSettings::season
};

struct RasterImages {
    std::vector<std::uint8_t> rgba;       // display-encoded RGBA8, row-major from the top
    std::vector<float> depth;             // m along the optical axis; +inf where nothing was hit
    std::vector<std::uint32_t> instance;  // RasterInstance::id; kNoInstance where nothing was hit
};
inline constexpr std::uint32_t kNoInstance = 0xFFFF'FFFFU;

/// The device, pipeline and meshes that every camera of a run shares. Rendering is serialised.
class RasterScene {
public:
    /// Creates a headless device (DeviceDesc::preferredDevice semantics; a software device on a
    /// machine without a GPU) and uploads `meshes`. Returns nullptr with a reason in `error`.
    /// With `reuse`, a scene made before (for the same preferred device), the new scene shares its
    /// device, its queue's lock and, for the same look, its realistic textures instead of making
    /// them again: a rebuilt run (a robot spawned, a profile applied) starts in a fraction of the
    /// time. The images are the same either way.
    [[nodiscard]] static std::shared_ptr<RasterScene> create(
        const std::string& preferredDevice, std::span<const MeshSource> meshes, std::string& error,
        RasterLook look = {}, const std::shared_ptr<const RasterScene>& reuse = nullptr);
    ~RasterScene();
    RasterScene(const RasterScene&) = delete;
    RasterScene& operator=(const RasterScene&) = delete;

    [[nodiscard]] const std::string& deviceName() const;
    [[nodiscard]] bool isSoftware() const;
    /// Samples per pixel of the image: the most, up to 8, the device offers. The image is their
    /// average, as a sensor pixel integrates the light over its area; depth and instance id
    /// take one sample.
    [[nodiscard]] std::uint32_t samples() const;
    /// Whether colour comes from the realistic renderer.
    [[nodiscard]] bool realistic() const;

private:
    RasterScene();
    friend class RasterCamera;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// One camera's render targets and readback buffer.
/// A camera sensor model's fixed part, for the GPU (gpu/kernels/camera/camera_model.slang): the
/// per-pixel work of modules/sensors CameraModel::apply, the CPU reference, done on the images
/// render() has just drawn, so only the output's size is read back.
struct CameraEffects {
    std::uint32_t width{0}, height{0};  // the output image; the render is this camera's size
    double fx{0}, fy{0}, cx{0}, cy{0};  // the render's intrinsics (pixels)
    /// Per output pixel: x and y of its ray at z = 1 before distortion (NaN where the lens maps
    /// none), the vignetting factor, unused.
    std::vector<std::array<float, 4>> rays;
    std::array<float, 256> decode{};    // display byte -> linear
    std::vector<std::uint8_t> encoded;  // converter code -> display byte; empty: no exposure
};

/// A frame's part of the model: motion, exposure and the noise's stream.
struct CameraEffectsFrame {
    std::array<double, 3> omega{};   // rad/s, the camera's angular velocity in its optical frame
    double rollingShutterTime{0.0};  // s, first row to last
    double electronsPerUnit{0.0};    // e- at linear 1 this frame (with an exposure)
    double gain{1.0};
    double fullWell{1.0};   // e-
    double readNoise{0.0};  // e-
    std::uint64_t seed{0};
    std::uint64_t stepIndex{0};
    std::uint32_t entity{0};  // row v draws from stream entity + v
};

/// A thermal camera's image on the GPU (thermal_camera.slang): what does not change per frame.
struct ThermalImageSetup {
    std::uint32_t prims{0};     // entries of each frame's apparent-temperature table
    std::uint32_t palette{0};   // 0 white hot, 1 black hot, 2 ironbow
    double netd{0.0};           // K, the noise's standard deviation
    double displayMin{0.0};     // K; displayMax > displayMin: a fixed display range
    double displayMax{0.0};
    double span{0.0};           // K, the least range of a frame's own display range
};

/// A thermal frame: each scene prim's apparent temperature, the sky's, the noise's stream.
struct ThermalImageFrame {
    std::span<const double> apparent;  // K, by scene prim (instance id)
    double sky{0.0};                   // K, where nothing was drawn
    std::uint64_t seed{0};
    std::uint64_t stepIndex{0};
    std::uint32_t entity{0};  // row v draws from stream entity + v
};

class RasterCamera {
public:
    RasterCamera(std::shared_ptr<RasterScene> scene, std::uint32_t width, std::uint32_t height);
    ~RasterCamera();
    RasterCamera(const RasterCamera&) = delete;
    RasterCamera& operator=(const RasterCamera&) = delete;

    /// Draws `instances` as seen by `camera` (whose size must be this camera's) and waits for
    /// the images.
    /// `seconds` is the sim time: what moves in the realistic look (water, clouds).
    void render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                RasterImages& out, double seconds = 0.0);

    /// Whether colour is drawn (the default). Without, render() draws only depth and instance
    /// ids, and the colour reads back as zeros: for a camera that needs no colour, as a thermal
    /// camera, which saves the colour passes (the realistic look's shadows and shading).
    void setColour(bool drawn);

    /// Prepares the sensor model on the GPU; false (with the reason) when its kernel cannot be
    /// built, and render() with effects is then not available.
    bool setEffects(const CameraEffects& effects, std::string& error);
    /// render(), then the model of setEffects() on the GPU: `out` at the model's output size,
    /// and `signal` per output pixel (the sum over the channels of min(e·gain/full well, 1),
    /// negative where the render was not seen) for auto exposure.
    void render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                const CameraEffectsFrame& frame, RasterImages& out, std::vector<float>& signal,
                double seconds = 0.0);

    /// render() in two halves, so the host can work while the device draws: begin() submits
    /// the draw (and the model with `frame`, as setEffects() prepared it) and returns at once;
    /// finish() waits for it and fills `out` (and `signal` with a frame) as render() does. A
    /// begin() while one is pending finishes that one first, its images discarded.
    void begin(std::span<const RasterInstance> instances, const PinholeCamera& camera,
               const CameraEffectsFrame* frame, double seconds = 0.0);
    void finish(RasterImages& out, std::vector<float>* signal = nullptr);
    /// Whether a begin() has not been finished.
    [[nodiscard]] bool pending() const;

    /// Prepares the thermal image on the GPU (a camera without colour, setColour(false)); false
    /// (with the reason) when its kernels cannot be built.
    bool setThermal(const ThermalImageSetup& setup, std::string& error);
    /// begin() with the thermal image of setThermal() after the render; finish() then gives the
    /// palette as `out.rgba` and the temperatures (K) as `signal`.
    void beginThermal(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                      const ThermalImageFrame& frame, double seconds = 0.0);

private:
    /// Draws and copies the images to the read-back buffer, then the model when `frame` is given.
    void draw(std::span<const RasterInstance> instances, const PinholeCamera& camera,
              double seconds, const CameraEffectsFrame* frame,
              const ThermalImageFrame* thermal = nullptr);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::gpu
