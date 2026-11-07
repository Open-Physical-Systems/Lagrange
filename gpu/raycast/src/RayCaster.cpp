#include "lagrange/gpu/Compute.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/RayCaster.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "shaders/kBvhTraverse.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <numeric>
#include <utility>

namespace lagrange::gpu {

namespace {

// The kernel's structures (std430), in gpu/kernels/lidar/bvh_traverse.slang.
struct GpuShape {
    float origin[3];  // the sensor in the shape's frame
    std::uint32_t type;
    float size[3];
    std::uint32_t pad;
    float rows[3][4];  // sensor-to-shape rotation, by rows; w of rows 0 and 1: a hull's faces
};
static_assert(sizeof(GpuShape) == 80);

struct GpuNode {
    float lo[3];
    std::int32_t left;
    float hi[3];
    std::int32_t right;
};
static_assert(sizeof(GpuNode) == 32);
static_assert(sizeof(CastHit) == 8);

struct Params {
    float rows[3][4];
    std::uint32_t rays;
    std::uint32_t planes;
    std::uint32_t nodes;
    std::uint32_t pad;
};
static_assert(sizeof(Params) == 64);

constexpr std::uint32_t kGroupSize = 64;  // numthreads of castRays
constexpr std::uint32_t kPlane = 0, kSphere = 1, kCapsule = 2, kCylinder = 3, kBox = 4,
                        kConvexHull = 5;
// Bounding boxes are grown by this much (m) plus a relative part, so that float rounding at a
// face never culls a shape the exact test would hit.
constexpr double kBoxMarginM = 1e-4;
constexpr double kBoxMarginRelative = 1e-6;

using Matrix = std::array<double, 9>;  // row-major

/// Rotation matrix of a unit quaternion (w, x, y, z).
Matrix matrixOf(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
            2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

struct Box {
    std::array<float, 3> lo, hi;
};

/// World-axis box around a shape whose world-from-shape rotation is `r`.
Box boundsOf(const CastShape& s, const Matrix& r) {
    std::array<double, 3> extent{};
    for (std::size_t i = 0; i < 3; ++i) {
        // Row i of r: world axis i in shape coordinates; column j: shape axis j in world.
        const double ax = r[3 * i], ay = r[3 * i + 1], az = r[3 * i + 2];
        switch (s.type) {
            case kSphere:
                extent[i] = s.size[0];
                break;
            case kBox:
            case kConvexHull:  // its size bounds every point's coordinates
                extent[i] =
                    std::abs(ax) * s.size[0] + std::abs(ay) * s.size[1] + std::abs(az) * s.size[2];
                break;
            case kCylinder:  // az: the axis (shape z) along world axis i
                extent[i] =
                    std::abs(az) * s.size[1] + s.size[0] * std::sqrt(std::max(0.0, 1.0 - az * az));
                break;
            case kCapsule:
                extent[i] = std::abs(az) * s.size[1] + s.size[0];
                break;
            default:
                break;
        }
    }
    Box box;
    for (std::size_t i = 0; i < 3; ++i) {
        const double margin = kBoxMarginM + kBoxMarginRelative * std::abs(s.position[i]);
        box.lo[i] = static_cast<float>(s.position[i] - extent[i] - margin);
        box.hi[i] = static_cast<float>(s.position[i] + extent[i] + margin);
    }
    return box;
}

/// Median split on the longest axis of the centres; leaves hold one shape.
class BvhBuilder {
public:
    BvhBuilder(std::span<const Box> boxes, std::span<const std::uint32_t> shapeOf)
        : boxes_(boxes), shapeOf_(shapeOf) {}

    std::vector<GpuNode> build() {
        nodes_.clear();
        std::vector<std::uint32_t> items(boxes_.size());
        std::iota(items.begin(), items.end(), 0U);
        if (!items.empty()) {
            nodes_.reserve(2 * items.size());
            add(items, 0, items.size());
        }
        return std::move(nodes_);
    }

private:
    std::int32_t add(std::vector<std::uint32_t>& items, std::size_t begin, std::size_t end) {
        const auto index = static_cast<std::int32_t>(nodes_.size());
        nodes_.emplace_back();
        Box bounds = boxes_[items[begin]];
        Box centres{bounds.lo, bounds.lo};
        for (std::size_t k = begin; k < end; ++k) {
            const Box& b = boxes_[items[k]];
            for (std::size_t i = 0; i < 3; ++i) {
                bounds.lo[i] = std::min(bounds.lo[i], b.lo[i]);
                bounds.hi[i] = std::max(bounds.hi[i], b.hi[i]);
                const float c = 0.5F * (b.lo[i] + b.hi[i]);
                centres.lo[i] = k == begin ? c : std::min(centres.lo[i], c);
                centres.hi[i] = k == begin ? c : std::max(centres.hi[i], c);
            }
        }
        GpuNode node{};
        std::copy(bounds.lo.begin(), bounds.lo.end(), node.lo);
        std::copy(bounds.hi.begin(), bounds.hi.end(), node.hi);
        if (end - begin == 1) {
            node.left = -1 - static_cast<std::int32_t>(shapeOf_[items[begin]]);
            node.right = -1;
        } else {
            std::size_t axis = 0;
            for (std::size_t i = 1; i < 3; ++i) {
                if (centres.hi[i] - centres.lo[i] > centres.hi[axis] - centres.lo[axis]) {
                    axis = i;
                }
            }
            const std::size_t middle = begin + (end - begin) / 2;
            const auto centre = [this, axis](std::uint32_t item) {
                return boxes_[item].lo[axis] + boxes_[item].hi[axis];
            };
            std::nth_element(
                items.begin() + static_cast<std::ptrdiff_t>(begin),
                items.begin() + static_cast<std::ptrdiff_t>(middle),
                items.begin() + static_cast<std::ptrdiff_t>(end),
                [&](std::uint32_t a, std::uint32_t b) { return centre(a) < centre(b); });
            node.left = add(items, begin, middle);
            node.right = add(items, middle, end);
        }
        nodes_[static_cast<std::size_t>(index)] = node;
        return index;
    }

    std::span<const Box> boxes_;
    std::span<const std::uint32_t> shapeOf_;
    std::vector<GpuNode> nodes_;
};

}  // namespace

struct RayContext::Impl {
    std::unique_ptr<Device> device;
    ComputeKernel kernel;
    std::mutex mutex;  // one cast at a time: the kernel's descriptor set and the queue
};

RayContext::RayContext() : impl_(std::make_unique<Impl>()) {}

RayContext::~RayContext() {
    if (impl_->device) {
        destroy(*impl_->device, impl_->kernel);
    }
}

std::shared_ptr<RayContext> RayContext::create(const std::string& preferredDevice,
                                               std::string& error) {
    DeviceDesc desc;
    desc.applicationName = "lagrange_lidar";
    desc.preferredDevice = preferredDevice;
    std::unique_ptr<Device> device = Device::create(desc, error);
    if (!device) {
        return nullptr;
    }
    std::shared_ptr<RayContext> context(new RayContext());
    context->impl_->kernel =
        createComputeKernel(*device, kBvhTraverse, 5, static_cast<std::uint32_t>(sizeof(Params)));
    if (context->impl_->kernel.pipeline == VK_NULL_HANDLE) {
        error = "cannot build the lidar kernel on " + device->info().name;
        return nullptr;
    }
    context->impl_->device = std::move(device);
    return context;
}

const std::string& RayContext::deviceName() const {
    return impl_->device->info().name;
}

int RayContext::tier() const {
    return static_cast<int>(impl_->device->tier());
}

struct RayScan::Impl {
    std::shared_ptr<RayContext> context;
    std::uint32_t rays{0};
    Buffer directions, shapes, nodes, hits, faces;
    std::vector<GpuShape> shapeData;
    std::vector<float> faceData;  // four per face of every hull
    std::vector<Box> boxes;
    std::vector<std::uint32_t> boundedShapes;

    [[nodiscard]] const Device& device() const { return *context->impl_->device; }

    /// A host-visible storage buffer of at least `bytes` (never empty).
    void reserve(Buffer& buffer, std::size_t bytes) {
        bytes = std::max<std::size_t>(bytes, 16);
        if (buffer.buffer != VK_NULL_HANDLE && buffer.size >= bytes) {
            return;
        }
        destroy(device(), buffer);
        buffer = createBuffer(device(), bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
    }
};

RayScan::RayScan(std::shared_ptr<RayContext> context,
                 std::span<const std::array<float, 3>> directions)
    : impl_(std::make_unique<Impl>()) {
    impl_->context = std::move(context);
    impl_->rays = static_cast<std::uint32_t>(directions.size());
    impl_->reserve(impl_->directions,
                   std::max<std::size_t>(directions.size(), 1) * 8 * sizeof(float));
    const std::vector<std::array<float, 3>> origins(directions.size(), {0.0F, 0.0F, 0.0F});
    setBeams(directions, origins);
    impl_->reserve(impl_->hits, directions.size() * sizeof(CastHit));
}

RayScan::~RayScan() {
    const Device& device = impl_->device();
    for (Buffer* buffer :
         {&impl_->directions, &impl_->shapes, &impl_->nodes, &impl_->hits, &impl_->faces}) {
        destroy(device, *buffer);
    }
}

void RayScan::setBeams(std::span<const std::array<float, 3>> directions,
                       std::span<const std::array<float, 3>> origins) {
    Impl& s = *impl_;
    const std::lock_guard lock(s.context->impl_->mutex);
    const std::size_t count = std::min<std::size_t>({directions.size(), origins.size(), s.rays});
    auto* out = static_cast<float*>(s.directions.mapped);
    for (std::size_t i = 0; i < count; ++i) {
        std::copy(directions[i].begin(), directions[i].end(), out + 8 * i);
        out[8 * i + 3] = 0.0F;
        std::copy(origins[i].begin(), origins[i].end(), out + 8 * i + 4);
        out[8 * i + 7] = 0.0F;
    }
    flush(s.device(), s.directions);
}

void RayScan::cast(std::span<const CastShape> shapes,
                   const std::array<double, 4>& sensorOrientation, std::span<CastHit> hits) {
    Impl& s = *impl_;
    const Matrix sensor = matrixOf(sensorOrientation);  // world from sensor
    // Planes first (tested by every ray), then the bounded shapes behind the hierarchy.
    s.shapeData.clear();
    s.faceData.clear();
    s.boxes.clear();
    s.boundedShapes.clear();
    std::vector<std::uint32_t> original;  // input index of each uploaded shape
    original.reserve(shapes.size());
    for (const bool planes : {true, false}) {
        for (std::size_t k = 0; k < shapes.size(); ++k) {
            const CastShape& shape = shapes[k];
            if ((shape.type == kPlane) != planes) {
                continue;
            }
            // In double: the sensor in the shape's frame, and the sensor-to-shape rotation.
            const Matrix world = matrixOf(shape.orientation);  // world from shape
            GpuShape g{};
            g.type = shape.type;
            for (std::size_t i = 0; i < 3; ++i) {
                g.size[i] = static_cast<float>(shape.size[i]);
                double origin = 0.0;
                for (std::size_t j = 0; j < 3; ++j) {
                    origin -= world[3 * j + i] * shape.position[j];  // (world^T)(-position)
                    double m = 0.0;
                    for (std::size_t r = 0; r < 3; ++r) {
                        m += world[3 * r + i] * sensor[3 * r + j];  // world^T * sensor
                    }
                    g.rows[i][j] = static_cast<float>(m);
                }
                g.origin[i] = static_cast<float>(origin);
            }
            if (shape.type == kConvexHull) {
                // Face planes are in the shape's frame, so their offsets are small: float.
                g.rows[0][3] = std::bit_cast<float>(
                    static_cast<std::uint32_t>(s.faceData.size() / 4));  // first face
                g.rows[1][3] = std::bit_cast<float>(static_cast<std::uint32_t>(shape.faces.size()));
                for (const auto& face : shape.faces) {
                    for (const double v : face) {
                        s.faceData.push_back(static_cast<float>(v));
                    }
                }
            }
            if (!planes) {
                s.boxes.push_back(boundsOf(shape, world));
                s.boundedShapes.push_back(static_cast<std::uint32_t>(s.shapeData.size()));
            }
            s.shapeData.push_back(g);
            original.push_back(static_cast<std::uint32_t>(k));
        }
    }
    const std::uint32_t planeCount =
        static_cast<std::uint32_t>(s.shapeData.size() - s.boundedShapes.size());
    const std::vector<GpuNode> tree = BvhBuilder(s.boxes, s.boundedShapes).build();

    Params params{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            params.rows[r][c] = static_cast<float>(sensor[3 * r + c]);
        }
    }
    params.rays = s.rays;
    params.planes = planeCount;
    params.nodes = static_cast<std::uint32_t>(tree.size());

    const std::lock_guard lock(s.context->impl_->mutex);
    const Device& device = s.device();
    s.reserve(s.shapes, s.shapeData.size() * sizeof(GpuShape));
    s.reserve(s.nodes, tree.size() * sizeof(GpuNode));
    s.reserve(s.faces, s.faceData.size() * sizeof(float));
    // An empty vector's data() may be null, which memcpy may not be given even for no bytes.
    std::copy(s.shapeData.begin(), s.shapeData.end(), static_cast<GpuShape*>(s.shapes.mapped));
    std::copy(tree.begin(), tree.end(), static_cast<GpuNode*>(s.nodes.mapped));
    std::copy(s.faceData.begin(), s.faceData.end(), static_cast<float*>(s.faces.mapped));
    flush(device, s.shapes);
    flush(device, s.nodes);
    flush(device, s.faces);
    const ComputeKernel& kernel = s.context->impl_->kernel;
    const std::array<const Buffer*, 5> bound{&s.shapes, &s.nodes, &s.directions, &s.hits, &s.faces};
    bindBuffers(device, kernel, bound);
    device.submitAndWait([&](VkCommandBuffer commands) {
        recordDispatch(commands, kernel, std::as_bytes(std::span(&params, 1)),
                       (s.rays + kGroupSize - 1) / kGroupSize);
    });
    invalidate(device, s.hits);
    const auto* out = static_cast<const CastHit*>(s.hits.mapped);
    const std::size_t count = std::min<std::size_t>(hits.size(), s.rays);
    for (std::size_t i = 0; i < count; ++i) {
        hits[i] = out[i];
        if (hits[i].shape != kNoShape) {
            hits[i].shape = original[hits[i].shape];
        }
    }
}

}  // namespace lagrange::gpu
