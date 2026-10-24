#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/v1/scene.pb.h"

#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using lagrange::viewer::kNone;
using lagrange::viewer::SceneBuilder;
namespace v1 = lagrange::viewer::v1;

template <class T>
std::string bytes(const std::vector<T>& values) {
    return std::string(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(T));
}

std::string snapshotTransform(double x, double y, double z) {
    std::string out(52, '\0');
    const double position[3] = {x, y, z};
    const float rotation[4] = {1.0F, 0.0F, 0.0F, 0.0F};
    const float scale[3] = {1.0F, 1.0F, 1.0F};
    std::memcpy(out.data(), position, 24);
    std::memcpy(out.data() + 24, rotation, 16);
    std::memcpy(out.data() + 40, scale, 12);
    return out;
}

// Two prims: a root and a child with a one-triangle mesh.
v1::SceneSnapshot validSnapshot() {
    v1::SceneSnapshot snapshot;
    snapshot.set_sequence(100);
    snapshot.mutable_sim_time()->set_ns(5);
    auto* root = snapshot.add_prims();
    root->set_id(0);
    root->set_parent(kNone);
    root->set_name("robot");
    root->set_mesh(kNone);
    root->set_material(kNone);
    auto* child = snapshot.add_prims();
    child->set_id(1);
    child->set_parent(0);
    child->set_name("左車輪");
    child->set_mesh(0);
    child->set_material(0);
    child->set_visible(true);
    auto* mesh = snapshot.add_meshes();
    mesh->set_vertex_count(3);
    mesh->set_index_count(3);
    mesh->mutable_positions()->set_inline_(bytes<float>({0, 0, 0, 1, 0, 0, 0, 1, 0}));
    mesh->mutable_normals()->set_inline_(bytes<float>({0, 0, 1, 0, 0, 1, 0, 0, 1}));
    mesh->mutable_indices()->set_inline_(bytes<std::uint32_t>({0, 1, 2}));
    auto* material = snapshot.add_materials();
    material->set_name("steel");
    for (const float c : {0.1F, 0.2F, 0.3F, 1.0F}) {
        material->add_base_color(c);
    }
    snapshot.mutable_transforms()->set_inline_(snapshotTransform(1, 2, 3) +
                                               snapshotTransform(4.5e5, 5, 6));
    return snapshot;
}

v1::SceneDelta delta(std::uint64_t base, std::uint32_t prim, double x) {
    v1::SceneDelta d;
    d.set_base_sequence(base);
    d.set_sequence(base + 1);
    d.mutable_sim_time()->set_ns(77);
    d.add_transform_prims(prim);
    std::string data(40, '\0');
    const double position[3] = {x, 0.0, 0.0};
    const float rotation[4] = {0.0F, 0.0F, 0.0F, 1.0F};
    std::memcpy(data.data(), position, 24);
    std::memcpy(data.data() + 24, rotation, 16);
    d.set_transform_data(data);
    return d;
}

TEST(SceneBuilder, AppliesAValidSnapshot) {
    SceneBuilder builder;
    EXPECT_EQ(builder.frame(), nullptr);
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error)) << error;
    const auto frame = builder.frame();
    ASSERT_NE(frame, nullptr);
    EXPECT_EQ(frame->sequence, 100U);
    EXPECT_EQ(frame->simTimeNs, 5);
    ASSERT_EQ(frame->statics->prims.size(), 2U);
    EXPECT_EQ(frame->statics->prims[1].name, "左車輪");
    EXPECT_EQ(frame->statics->prims[1].parent, 0U);
    EXPECT_EQ(frame->statics->meshes[0].indices.size(), 3U);
    EXPECT_FLOAT_EQ(frame->statics->materials[0].baseColor[2], 0.3F);
    EXPECT_DOUBLE_EQ(frame->transforms[1].position[0], 4.5e5);  // double survives intact
    EXPECT_EQ(frame->statics->generation, 1U);
}

TEST(SceneBuilder, CollidersAreKeptAndBadOnesDropped) {
    v1::SceneSnapshot snapshot = validSnapshot();
    v1::Collider* good = snapshot.add_colliders();
    good->set_prim(1);
    good->set_mesh(0);
    for (const double v : {0.1, 0.2, 0.3}) {
        good->add_position(v);
    }
    for (const float v : {1.0F, 0.0F, 0.0F, 0.0F}) {
        good->add_rotation(v);
    }
    v1::Collider* fixed = snapshot.add_colliders();  // fixed in the frame, no prim
    *fixed = *good;
    fixed->set_prim(0xFFFFFFFFU);
    v1::Collider* noMesh = snapshot.add_colliders();
    *noMesh = *good;
    noMesh->set_mesh(9);
    v1::Collider* noPose = snapshot.add_colliders();
    noPose->set_prim(1);
    noPose->set_mesh(0);

    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(snapshot, error)) << error;
    const auto& colliders = builder.frame()->statics->colliders;
    ASSERT_EQ(colliders.size(), 2U);
    EXPECT_EQ(colliders[0].prim, 1U);
    EXPECT_DOUBLE_EQ(colliders[0].position[2], 0.3);
    EXPECT_EQ(colliders[1].prim, kNone);
}

TEST(SceneBuilder, ElectricalNetsAreKeptAndBadTerminalsDropped) {
    v1::SceneSnapshot snapshot = validSnapshot();
    v1::ElectricalNet* net = snapshot.add_nets();
    net->set_id(3);
    net->set_name("dc_48v");
    net->set_bus_prim(0);
    net->set_voltage_signal("bus/dc_48v/voltage");
    v1::NetTerminal* source = net->add_terminals();
    source->set_prim(1);
    source->set_port("dc_out");
    source->set_current_signal("battery/current");
    v1::NetTerminal* load = net->add_terminals();
    load->set_prim(0);
    load->set_port("dc_in");
    v1::NetTerminal* dangling = net->add_terminals();
    dangling->set_prim(99);  // no such prim
    dangling->set_port("dc_in");
    v1::NetLimit* limit = net->add_limits();
    limit->set_label("OV limit");
    limit->set_value(54.6);
    limit->mutable_unit()->set_display("V");
    limit->set_upper(true);
    limit->set_signal("bus/dc_48v/voltage");
    v1::ElectricalNet* orphan = snapshot.add_nets();
    orphan->set_bus_prim(77);  // no such prim either

    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(snapshot, error)) << error;
    const auto& nets = builder.frame()->statics->nets;
    ASSERT_EQ(nets.size(), 2U);
    EXPECT_EQ(nets[0].id, 3U);
    EXPECT_EQ(nets[0].name, "dc_48v");
    EXPECT_EQ(nets[0].busPrim, 0U);
    EXPECT_EQ(nets[0].voltageSignal, "bus/dc_48v/voltage");
    ASSERT_EQ(nets[0].terminals.size(), 2U);
    EXPECT_TRUE(nets[0].terminals[0].isSource());
    EXPECT_EQ(nets[0].terminals[0].currentSignal, "battery/current");
    EXPECT_FALSE(nets[0].terminals[1].isSource());
    ASSERT_EQ(nets[0].limits.size(), 1U);
    EXPECT_EQ(nets[0].limits[0].label, "OV limit");
    EXPECT_DOUBLE_EQ(nets[0].limits[0].value, 54.6);
    EXPECT_EQ(nets[0].limits[0].unit, "V");
    EXPECT_TRUE(nets[0].limits[0].upper);
    EXPECT_EQ(nets[1].busPrim, kNone);
}

TEST(SceneBuilder, DeltaMovesOnlyItsPrimsAndSharesStatics) {
    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    const auto before = builder.frame();
    ASSERT_EQ(builder.applyDelta(delta(100, 1, 9.25)), SceneBuilder::DeltaResult::Applied);
    const auto after = builder.frame();
    EXPECT_EQ(after->sequence, 101U);
    EXPECT_EQ(after->simTimeNs, 77);
    EXPECT_DOUBLE_EQ(after->transforms[1].position[0], 9.25);
    EXPECT_FLOAT_EQ(after->transforms[1].rotation[3], 1.0F);
    EXPECT_FLOAT_EQ(after->transforms[1].scale[0], 1.0F);        // scale is not in a delta
    EXPECT_DOUBLE_EQ(after->transforms[0].position[0], 1.0);     // untouched prim
    EXPECT_EQ(after->statics, before->statics);                  // same immutable object
    EXPECT_DOUBLE_EQ(before->transforms[1].position[0], 4.5e5);  // old frame is unchanged
}

TEST(SceneBuilder, LostDeltaIsDetected) {
    SceneBuilder builder;
    EXPECT_EQ(builder.applyDelta(delta(1, 0, 0.0)), SceneBuilder::DeltaResult::OutOfSequence);
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    ASSERT_EQ(builder.applyDelta(delta(100, 0, 1.0)), SceneBuilder::DeltaResult::Applied);
    // 101 -> 102 never arrives; the next one is based on 102.
    EXPECT_EQ(builder.applyDelta(delta(102, 0, 2.0)), SceneBuilder::DeltaResult::OutOfSequence);
    EXPECT_EQ(builder.frame()->sequence, 101U);
}

TEST(SceneBuilder, MalformedDeltasAreRejectedWithoutChangingTheFrame) {
    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    const auto before = builder.frame();

    v1::SceneDelta unknownPrim = delta(100, 7, 1.0);
    EXPECT_EQ(builder.applyDelta(unknownPrim), SceneBuilder::DeltaResult::Invalid);
    v1::SceneDelta shortData = delta(100, 0, 1.0);
    shortData.set_transform_data(std::string(39, '\0'));
    EXPECT_EQ(builder.applyDelta(shortData), SceneBuilder::DeltaResult::Invalid);
    EXPECT_EQ(builder.frame(), before);
}

struct BadSnapshot {
    const char* name;
    void (*corrupt)(v1::SceneSnapshot&);
    const char* expectedText;
};

class SnapshotValidation : public ::testing::TestWithParam<BadSnapshot> {};

TEST_P(SnapshotValidation, RejectsAndKeepsThePreviousScene) {
    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    const auto before = builder.frame();

    v1::SceneSnapshot bad = validSnapshot();
    GetParam().corrupt(bad);
    EXPECT_FALSE(builder.applySnapshot(bad, error));
    EXPECT_NE(error.find(GetParam().expectedText), std::string::npos) << error;
    EXPECT_EQ(builder.frame(), before);
}

INSTANTIATE_TEST_SUITE_P(
    Cases, SnapshotValidation,
    ::testing::Values(
        BadSnapshot{"IndexOutOfRange",
                    [](v1::SceneSnapshot& s) {
                        s.mutable_meshes(0)->mutable_indices()->set_inline_(
                            bytes<std::uint32_t>({0, 1, 3}));
                    },
                    "index out of range"},
        BadSnapshot{"PositionBytesTooShort",
                    [](v1::SceneSnapshot& s) {
                        s.mutable_meshes(0)->mutable_positions()->set_inline_(
                            std::string(35, '\0'));
                    },
                    "data size"},
        BadSnapshot{"IndexCountNotTriangles",
                    [](v1::SceneSnapshot& s) {
                        s.mutable_meshes(0)->set_index_count(2);
                        s.mutable_meshes(0)->mutable_indices()->set_inline_(
                            bytes<std::uint32_t>({0, 1}));
                    },
                    "multiple of 3"},
        BadSnapshot{
            "HugeVertexCount",
            [](v1::SceneSnapshot& s) { s.mutable_meshes(0)->set_vertex_count(0xFFFFFFFFU); },
            "data size"},
        BadSnapshot{"PrimIdsNotDense", [](v1::SceneSnapshot& s) { s.mutable_prims(1)->set_id(5); },
                    "not dense"},
        BadSnapshot{"ParentMissing",
                    [](v1::SceneSnapshot& s) { s.mutable_prims(1)->set_parent(9); },
                    "missing parent"},
        BadSnapshot{"MeshMissing", [](v1::SceneSnapshot& s) { s.mutable_prims(1)->set_mesh(3); },
                    "missing parent, mesh"},
        BadSnapshot{"TransformBytesWrong",
                    [](v1::SceneSnapshot& s) {
                        s.mutable_transforms()->set_inline_(std::string(52, '\0'));
                    },
                    "transform data"},
        BadSnapshot{"MeshInSharedMemory",
                    [](v1::SceneSnapshot& s) {
                        s.mutable_meshes(0)->mutable_positions()->mutable_shm()->set_offset(0);
                    },
                    "only inline"}),
    [](const auto& param) { return std::string(param.param.name); });

TEST(SceneBuilder, EachSnapshotGetsANewGeneration) {
    SceneBuilder builder;
    std::string error;
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    ASSERT_TRUE(builder.applySnapshot(validSnapshot(), error));
    EXPECT_EQ(builder.frame()->statics->generation, 2U);
}

}  // namespace
