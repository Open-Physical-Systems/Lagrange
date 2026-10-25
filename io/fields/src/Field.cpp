// Volumetric fields as NanoVDB grids (Field.hpp).
#include "lagrange/fields/Field.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/GridHandle.h>
#include <nanovdb/util/HostBuffer.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/SampleFromVoxels.h>

namespace lagrange::fields {

struct Field::Impl {
    nanovdb::GridHandle<nanovdb::HostBuffer> handle;
    const nanovdb::FloatGrid* scalar{nullptr};
    const nanovdb::Vec3fGrid* vector{nullptr};
};

Field::Field() : impl_(std::make_unique<Impl>()) {}
Field::~Field() = default;

namespace {

/// Fills in what a field knows about its grid.
template <typename GridT>
void describe(const GridT& grid, std::string& name, double& voxel, std::array<Vec3, 2>& bounds) {
    name = grid.gridName();
    voxel = grid.voxelSize()[0];
    const nanovdb::CoordBBox box = grid.indexBBox();
    const nanovdb::Vec3d lo =
        grid.indexToWorld(nanovdb::Vec3d(box.min()[0], box.min()[1], box.min()[2]));
    const nanovdb::Vec3d hi =
        grid.indexToWorld(nanovdb::Vec3d(box.max()[0], box.max()[1], box.max()[2]));
    bounds = {Vec3{lo[0], lo[1], lo[2]}, Vec3{hi[0], hi[1], hi[2]}};
}

}  // namespace

std::shared_ptr<const Field> Field::fromBytes(std::vector<std::byte> bytes) {
    std::shared_ptr<Field> out(new Field());
    out->bytes_ = std::move(bytes);
    auto buffer = nanovdb::HostBuffer::create(out->bytes_.size());
    std::memcpy(buffer.data(), out->bytes_.data(), out->bytes_.size());
    out->impl_->handle = nanovdb::GridHandle<nanovdb::HostBuffer>(std::move(buffer));
    if (const auto* g = out->impl_->handle.grid<float>()) {
        out->kind_ = Kind::Scalar;
        out->impl_->scalar = g;
        describe(*g, out->name_, out->voxel_, out->bounds_);
        return out;
    }
    if (const auto* g = out->impl_->handle.grid<nanovdb::Vec3f>()) {
        out->kind_ = Kind::Vector;
        out->impl_->vector = g;
        describe(*g, out->name_, out->voxel_, out->bounds_);
        return out;
    }
    return nullptr;
}

Expected<std::shared_ptr<const Field>, std::string> Field::load(const std::string& path) {
    try {
        const auto handle = nanovdb::io::readGrid(path);
        std::vector<std::byte> bytes(handle.size());
        std::memcpy(bytes.data(), handle.data(), handle.size());
        auto field = fromBytes(std::move(bytes));
        if (!field) {
            return Unexpected{path + ": the field's first grid is neither float nor Vec3f"};
        }
        return field;
    } catch (const std::exception& e) {
        return Unexpected{path + ": " + e.what()};
    }
}

double Field::sampleScalar(const Vec3& world) const {
    if (impl_->scalar != nullptr) {
        const nanovdb::FloatGrid& grid = *impl_->scalar;
        const auto sampler = nanovdb::createSampler<1>(grid.getAccessor());
        return sampler(grid.worldToIndexF(nanovdb::Vec3f(static_cast<float>(world[0]),
                                                         static_cast<float>(world[1]),
                                                         static_cast<float>(world[2]))));
    }
    const Vec3 v = sampleVector(world);
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vec3 Field::sampleVector(const Vec3& world) const {
    if (impl_->vector == nullptr) {
        return {};
    }
    const nanovdb::Vec3fGrid& grid = *impl_->vector;
    const auto sampler = nanovdb::createSampler<1>(grid.getAccessor());
    const nanovdb::Vec3f v = sampler(grid.worldToIndexF(nanovdb::Vec3f(
        static_cast<float>(world[0]), static_cast<float>(world[1]), static_cast<float>(world[2]))));
    return {v[0], v[1], v[2]};
}

double Field::integrate(const Vec3& from, const Vec3& to) const {
    const Vec3 d{to[0] - from[0], to[1] - from[1], to[2] - from[2]};
    const double length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (length <= 0.0 || voxel_ <= 0.0) {
        return 0.0;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * voxel_))));
    const double step = length / steps;
    double sum = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = (i + 0.5) / steps;
        sum += sampleScalar({from[0] + t * d[0], from[1] + t * d[1], from[2] + t * d[2]}) * step;
    }
    return sum;
}

namespace {

template <typename ValueT, typename Convert>
Expected<void, std::string> write(const std::string& path, const std::string& name,
                                  const DenseBox& box, const Convert& convert) {
    if (box.size <= 0.0 || box.count[0] < 1 || box.count[1] < 1 || box.count[2] < 1) {
        return Unexpected{std::string("a field's box needs a voxel size > 0 and voxels")};
    }
    try {
        nanovdb::GridBuilder<ValueT> builder(ValueT(0));
        auto accessor = builder.getAccessor();
        for (int i = 0; i < box.count[0]; ++i) {
            for (int j = 0; j < box.count[1]; ++j) {
                for (int k = 0; k < box.count[2]; ++k) {
                    const Vec3 at{box.origin[0] + i * box.size, box.origin[1] + j * box.size,
                                  box.origin[2] + k * box.size};
                    const ValueT v = convert(at);
                    if (v != ValueT(0)) {
                        accessor.setValue(nanovdb::Coord(i, j, k), v);
                    }
                }
            }
        }
        const auto handle = builder.getHandle(
            box.size, nanovdb::Vec3d(box.origin[0], box.origin[1], box.origin[2]), name);
        nanovdb::io::writeGrid(path, handle);
        return {};
    } catch (const std::exception& e) {
        return Unexpected{path + ": " + e.what()};
    }
}

}  // namespace

Expected<void, std::string> writeScalarField(const std::string& path, const std::string& name,
                                             const DenseBox& box,
                                             const std::function<double(const Vec3&)>& value) {
    return write<float>(path, name, box,
                        [&](const Vec3& at) { return static_cast<float>(value(at)); });
}

Expected<void, std::string> writeVectorField(const std::string& path, const std::string& name,
                                             const DenseBox& box,
                                             const std::function<Vec3(const Vec3&)>& value) {
    return write<nanovdb::Vec3f>(path, name, box, [&](const Vec3& at) {
        const Vec3 v = value(at);
        return nanovdb::Vec3f(static_cast<float>(v[0]), static_cast<float>(v[1]),
                              static_cast<float>(v[2]));
    });
}

}  // namespace lagrange::fields
