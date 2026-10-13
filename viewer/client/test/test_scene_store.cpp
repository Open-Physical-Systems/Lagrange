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
