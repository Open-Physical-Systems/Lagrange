#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace lagrange::gpu {

class Device;

/// A collision shape for the GPU ray caster. `type` and `size` follow lagrange::ShapeType (Plane
/// 0, Sphere 1, Capsule 2, Cylinder 3, Box 4, ConvexHull 5); a plane is unbounded. A convex hull
/// gives its face planes in `faces`, and `size` bounds its points' coordinates. Position and
/// orientation are in double, relative to the sensor (world axes): the caster combines them with
/// the sensor's orientation in double and converts to float last, so the kernel's float only ever
/// holds sensor-relative quantities (docs/design/huge-worlds.md §1).
struct CastShape {
    std::uint32_t type{0};
    std::array<double, 3> size{};
    std::array<double, 3> position{};               // m, shape origin minus sensor position
    std::array<double, 4> orientation{1, 0, 0, 0};  // world from shape, unit (w, x, y, z)
    // ConvexHull only: unit outward normal n and offset c per face, n·x <= c inside, in the
    // shape's frame. Must outlive the cast.
    std::span<const std::array<double, 4>> faces{};
};

struct CastHit {
    float range;          // m; +inf for no hit
    std::uint32_t shape;  // index into the shapes cast against, kNoShape for no hit
};
inline constexpr std::uint32_t kNoShape = 0xFFFF'FFFFU;

/// The GPU side of the Tier 1 lidar (docs/design/domains/sensors.md §6): a Vulkan device and the
/// kernel gpu/kernels/lidar/bvh_traverse.slang. One context serves every lidar; casts are
/// serialised on it.
class RayContext {
public:
    /// Creates a headless device (DeviceDesc::preferredDevice semantics; empty: automatic) and the
    /// kernel. Returns nullptr with a one-line reason in `error`.
    [[nodiscard]] static std::shared_ptr<RayContext> create(const std::string& preferredDevice,
                                                            std::string& error);
    ~RayContext();
    RayContext(const RayContext&) = delete;
    RayContext& operator=(const RayContext&) = delete;

    [[nodiscard]] const std::string& deviceName() const;
    /// The device's capability tier (RFC 0004): 0 for a software device, 1 or 2 for hardware.
    [[nodiscard]] int tier() const;

private:
    RayContext();
    friend class RayScan;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// One lidar's buffers: its beam directions (sensor frame, fixed) and the shapes and hits of a
/// scan.
class RayScan {
public:
    RayScan(std::shared_ptr<RayContext> context, std::span<const std::array<float, 3>> directions);
    ~RayScan();
    RayScan(const RayScan&) = delete;
    RayScan& operator=(const RayScan&) = delete;

    /// Casts every beam of a sensor with world orientation `sensorOrientation` (unit quaternion
    /// w, x, y, z) against `shapes` and waits for the result: one hit per beam in `hits`. Builds
    /// the bounding-volume hierarchy over the bounded shapes on the CPU each call (shapes move
    /// between scans).
    void cast(std::span<const CastShape> shapes, const std::array<double, 4>& sensorOrientation,
              std::span<CastHit> hits);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lagrange::gpu
