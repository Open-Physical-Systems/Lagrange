#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/v1/scene.pb.h"

#include <cstring>

namespace lagrange::viewer {

namespace {

constexpr std::size_t kSnapshotTransformBytes = 52;  // 3 f64 + 4 f32 + 3 f32
constexpr std::size_t kDeltaTransformBytes = 40;     // 3 f64 + 4 f32

// Only payloads carried inline are supported so far; blob and shared-memory references
// arrive with the server features that produce them.
bool inlineBytes(const v1::DataRef& ref, const std::string*& out) {
    if (ref.location_case() == v1::DataRef::kInline) {
        out = &ref.inline_();
        return true;
    }
    static const std::string kEmpty;
    out = &kEmpty;
    return ref.location_case() == v1::DataRef::LOCATION_NOT_SET;
}

template <class T>
bool copyArray(const std::string& bytes, std::size_t count, std::vector<T>& out) {
    if (bytes.size() != count * sizeof(T)) {
        return false;
    }
    out.resize(count);
    if (count > 0) {
        std::memcpy(out.data(), bytes.data(), bytes.size());  // wire format is little-endian
    }
    return true;
}

void readPose(const char* bytes, Transform& transform) {
    std::memcpy(transform.position.data(), bytes, 24);
    std::memcpy(transform.rotation.data(), bytes + 24, 16);
}

}  // namespace

bool SceneBuilder::applySnapshot(const v1::SceneSnapshot& snapshot, std::string& error) {
    auto statics = std::make_shared<SceneStatics>();
    const auto primCount = static_cast<std::size_t>(snapshot.prims_size());
    const auto meshCount = static_cast<std::size_t>(snapshot.meshes_size());
    const auto materialCount = static_cast<std::size_t>(snapshot.materials_size());

    statics->meshes.resize(meshCount);
    for (std::size_t i = 0; i < meshCount; ++i) {
        const v1::Mesh& source = snapshot.meshes(static_cast<int>(i));
        MeshData& mesh = statics->meshes[i];
        const std::string* positions = nullptr;
        const std::string* normals = nullptr;
        const std::string* indices = nullptr;
        if (!inlineBytes(source.positions(), positions) ||
            !inlineBytes(source.normals(), normals) || !inlineBytes(source.indices(), indices)) {
            error = "mesh " + std::to_string(i) + ": only inline mesh data is supported";
            return false;
        }
        const std::size_t vertices = source.vertex_count();
        if (!copyArray(*positions, vertices * 3, mesh.positions) ||
            (!normals->empty() && !copyArray(*normals, vertices * 3, mesh.normals)) ||
            !copyArray(*indices, std::size_t{source.index_count()}, mesh.indices)) {
            error = "mesh " + std::to_string(i) + ": data size does not match the counts";
            return false;
        }
        if (mesh.indices.size() % 3 != 0) {
            error = "mesh " + std::to_string(i) + ": index count is not a multiple of 3";
            return false;
        }
        for (const std::uint32_t index : mesh.indices) {
            if (index >= vertices) {
                error = "mesh " + std::to_string(i) + ": index out of range";
                return false;
            }
        }
        if (source.aabb_size() == 6) {
            for (int k = 0; k < 6; ++k) {
                mesh.aabb.at(static_cast<std::size_t>(k)) = source.aabb(k);
            }
        }
    }

    statics->materials.resize(materialCount);
    for (std::size_t i = 0; i < materialCount; ++i) {
        const v1::Material& source = snapshot.materials(static_cast<int>(i));
        MaterialData& material = statics->materials[i];
        material.name = source.name();
        for (int k = 0; k < 4 && k < source.base_color_size(); ++k) {
            material.baseColor.at(static_cast<std::size_t>(k)) = source.base_color(k);
        }
        material.metallic = source.metallic();
        material.roughness = source.roughness();
        for (int k = 0; k < 3 && k < source.emissive_size(); ++k) {
            material.emissive.at(static_cast<std::size_t>(k)) = source.emissive(k);
        }
    }

    statics->prims.resize(primCount);
    for (std::size_t i = 0; i < primCount; ++i) {
        const v1::Prim& source = snapshot.prims(static_cast<int>(i));
        if (source.id() != i) {
            error = "prim ids are not dense: entry " + std::to_string(i) + " has id " +
                    std::to_string(source.id());
            return false;
        }
        if ((source.parent() != kNone && source.parent() >= primCount) ||
            (source.mesh() != kNone && source.mesh() >= meshCount) ||
            (source.material() != kNone && source.material() >= materialCount)) {
            error = "prim " + std::to_string(i) + " refers to a missing parent, mesh or material";
            return false;
        }
        PrimData& prim = statics->prims[i];
        prim.parent = source.parent();
        prim.name = source.name();
        prim.path = source.path();
        prim.typeName = source.type_name();
        prim.apiSchemas.assign(source.api_schemas().begin(), source.api_schemas().end());
        prim.mesh = source.mesh();
        prim.material = source.material();
        prim.visible = source.visible();
    }

    const std::string* transforms = nullptr;
    if (!inlineBytes(snapshot.transforms(), transforms) ||
        transforms->size() != primCount * kSnapshotTransformBytes) {
        error = "transform data does not match the prim count";
        return false;
    }

    auto frame = std::make_shared<SceneFrame>();
    frame->transforms.resize(primCount);
    for (std::size_t i = 0; i < primCount; ++i) {
        const char* bytes = transforms->data() + i * kSnapshotTransformBytes;
        readPose(bytes, frame->transforms[i]);
        std::memcpy(frame->transforms[i].scale.data(), bytes + kDeltaTransformBytes, 12);
    }
    // Electrical topology. Terminals that point at prims the scene does not have are dropped,
    // so panels can index prims with them without checking again.
    statics->nets.reserve(static_cast<std::size_t>(snapshot.nets_size()));
    for (const v1::ElectricalNet& source : snapshot.nets()) {
        ElectricalNet net;
        net.id = source.id();
        net.name = source.name();
        net.busPrim = source.bus_prim() < statics->prims.size() ? source.bus_prim() : kNone;
        net.voltageSignal = source.voltage_signal();
        for (const v1::NetTerminal& terminal : source.terminals()) {
            if (terminal.prim() < statics->prims.size()) {
                net.terminals.push_back(
                    {terminal.prim(), terminal.port(), terminal.current_signal()});
            }
        }
        for (const v1::NetLimit& limit : source.limits()) {
            net.limits.push_back({limit.label(), limit.value(), limit.unit().display(),
                                  limit.upper(), limit.signal()});
        }
        statics->nets.push_back(std::move(net));
    }
    // Colliders with a missing prim or mesh, or a malformed pose, are dropped.
    for (const v1::Collider& source : snapshot.colliders()) {
        if ((source.prim() != kNone && source.prim() >= primCount) || source.mesh() >= meshCount ||
            source.position_size() != 3 || source.rotation_size() != 4) {
            continue;
        }
        ColliderData collider;
        collider.prim = source.prim();
        collider.mesh = source.mesh();
        for (std::size_t k = 0; k < 3; ++k) {
            collider.position.at(k) = source.position(static_cast<int>(k));
        }
        for (std::size_t k = 0; k < 4; ++k) {
            collider.rotation.at(k) = source.rotation(static_cast<int>(k));
        }
        statics->colliders.push_back(collider);
    }
    statics->stagePath = snapshot.stage_path();
    statics->rebaseCount = snapshot.frame().rebase_count();
    if (snapshot.frame().region_enu_size() == 3) {
        for (int i = 0; i < 3; ++i) {
            statics->regionEnu.at(static_cast<std::size_t>(i)) = snapshot.frame().region_enu(i);
        }
    }
    statics->generation = ++generation_;
    frame->statics = std::move(statics);
    frame->sequence = snapshot.sequence();
    frame->simTimeNs = snapshot.sim_time().ns();
    frame_ = std::move(frame);
    return true;
}

SceneBuilder::DeltaResult SceneBuilder::applyDelta(const v1::SceneDelta& delta) {
    if (!frame_ || delta.base_sequence() != frame_->sequence) {
        return DeltaResult::OutOfSequence;
    }
    const auto count = static_cast<std::size_t>(delta.transform_prims_size());
    if (delta.transform_data().size() != count * kDeltaTransformBytes) {
        return DeltaResult::Invalid;
    }
    for (const std::uint32_t prim : delta.transform_prims()) {
        if (prim >= frame_->transforms.size()) {
            return DeltaResult::Invalid;
        }
    }
    // Structural changes (added and removed prims) are not produced by any server yet.
    if (delta.added_size() != 0 || delta.removed_size() != 0) {
        return DeltaResult::Invalid;
    }

    auto frame = std::make_shared<SceneFrame>(*frame_);  // copies transforms, shares statics
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint32_t prim = delta.transform_prims(static_cast<int>(i));
        readPose(delta.transform_data().data() + i * kDeltaTransformBytes, frame->transforms[prim]);
    }
    frame->sequence = delta.sequence();
    frame->simTimeNs = delta.sim_time().ns();
    frame_ = std::move(frame);
    return DeltaResult::Applied;
}

}  // namespace lagrange::viewer
