#pragma once

#include "lagrange/module/Expected.hpp"
#include "lagrange/signals/SignalBus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::fields {

using Vec3 = std::array<double, 3>;

/// A volumetric field (formats: OpenVDB's NanoVDB, CLAUDE.md §4): a NanoVDB grid of float
/// (scalar: fog extinction, dose) or Vec3f (vector: wind, B, E) values, sampled trilinearly in
/// world coordinates (the grid's own index-to-world map, region-local metres). Outside its
/// active voxels a field is its background value, zero.
///
/// The grid is one block of memory, kept as bytes: the same bytes go onto the signal bus as a
/// Field3D buffer (encoding "nanovdb") and to the GPU later (PNanoVDB), unchanged.
class Field {
public:
    enum class Kind { Scalar, Vector };

    /// The first grid of a .nvdb file.
    [[nodiscard]] static Expected<std::shared_ptr<const Field>, std::string> load(
        const std::string& path);
    /// A grid from its bytes (a Field3D buffer); null when they are not a float or Vec3f grid.
    [[nodiscard]] static std::shared_ptr<const Field> fromBytes(std::vector<std::byte> bytes);

    [[nodiscard]] Kind kind() const { return kind_; }
    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] double voxelSize() const { return voxel_; }
    /// World bounds of the active voxels' centres, min and max.
    [[nodiscard]] const std::array<Vec3, 2>& bounds() const { return bounds_; }
    [[nodiscard]] const std::vector<std::byte>& bytes() const { return bytes_; }

    /// Trilinear samples at a world point; a vector field's magnitude for sampleScalar.
    [[nodiscard]] double sampleScalar(const Vec3& world) const;
    [[nodiscard]] Vec3 sampleVector(const Vec3& world) const;

    /// The integral of a scalar field along a segment (optical depth for an extinction field),
    /// by the midpoint rule with steps of half a voxel.
    [[nodiscard]] double integrate(const Vec3& from, const Vec3& to) const;

    Field(const Field&) = delete;
    Field& operator=(const Field&) = delete;
    Field(Field&&) = delete;
    Field& operator=(Field&&) = delete;
    ~Field();

private:
    Field();
    struct Impl;
    std::unique_ptr<Impl> impl_;
    Kind kind_{Kind::Scalar};
    std::string name_;
    double voxel_{0.0};
    std::array<Vec3, 2> bounds_{};
    std::vector<std::byte> bytes_;
};

/// The Field3D buffer of a field (encoding "nanovdb", the grid's bytes), or an empty buffer for
/// no field.
[[nodiscard]] std::shared_ptr<const Buffer> fieldBuffer(const Field* field);

/// A field read from a Field3D signal, rebuilt only when its buffer changes; null for an empty
/// buffer (no field).
class FieldSignal {
public:
    [[nodiscard]] std::shared_ptr<const Field> read(const SignalBus& bus, SignalId id);

private:
    std::uint64_t handle_{0};
    std::shared_ptr<const Field> field_;
};

/// A dense box of voxels for writing fields: voxel (0, 0, 0)'s centre at `origin`, `size` apart,
/// `count` along x, y and z.
struct DenseBox {
    Vec3 origin{};
    double size{1.0};
    std::array<int, 3> count{1, 1, 1};
};

/// Writes a field sampled from a function at every voxel centre of `box` as a .nvdb file (one
/// grid named `name`). Values equal to zero are left inactive (the background).
[[nodiscard]] Expected<void, std::string> writeScalarField(
    const std::string& path, const std::string& name, const DenseBox& box,
    const std::function<double(const Vec3&)>& value);
[[nodiscard]] Expected<void, std::string> writeVectorField(
    const std::string& path, const std::string& name, const DenseBox& box,
    const std::function<Vec3(const Vec3&)>& value);

}  // namespace lagrange::fields
