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

private:
    struct GpuMesh {
        Buffer vertices;
        Buffer indices;
        std::uint32_t indexCount{0};
    };
    const Device& device_;
    std::vector<GpuMesh> meshes_;
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
    [[nodiscard]] static std::shared_ptr<RasterScene> create(const std::string& preferredDevice,
                                                             std::span<const MeshSource> meshes,
                                                             std::string& error);
    ~RasterScene();
    RasterScene(const RasterScene&) = delete;
    RasterScene& operator=(const RasterScene&) = delete;

    [[nodiscard]] const std::string& deviceName() const;
    [[nodiscard]] bool isSoftware() const;
    /// Samples per pixel of the image: the most, up to 8, the device offers. The image is their
    /// average, as a sensor pixel integrates the light over its area; depth and instance id
    /// take one sample.
    [[nodiscard]] std::uint32_t samples() const;

private:
    RasterScene();
    friend class RasterCamera;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// One camera's render targets and readback buffer.
class RasterCamera {
public:
    RasterCamera(std::shared_ptr<RasterScene> scene, std::uint32_t width, std::uint32_t height);
    ~RasterCamera();
    RasterCamera(const RasterCamera&) = delete;
    RasterCamera& operator=(const RasterCamera&) = delete;

    /// Draws `instances` as seen by `camera` (whose size must be this camera's) and waits for
    /// the images.
    void render(std::span<const RasterInstance> instances, const PinholeCamera& camera,
                RasterImages& out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::gpu
