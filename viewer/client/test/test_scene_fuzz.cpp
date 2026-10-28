// The scene builder against mutated protocol frames (docs/design/security.md): every golden
// frame of proto/test/golden, and each of them changed 2000 ways (bytes flipped, cut short,
// repeated, lengths enlarged), goes through the same harness as the libFuzzer target. Nothing
// may crash; the builder keeps a valid frame or none. A fixed seed: the same inputs every run.
#include "../fuzz/SceneBuilderHarness.hpp"
#include "lagrange/proto/Framing.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

namespace v1 = lagrange::viewer::v1;

std::vector<std::vector<std::byte>> goldenPayloads() {
    std::ifstream in(std::string(LAGRANGE_GOLDEN_DIR) + "/frames.bin", std::ios::binary);
    const std::vector<char> raw((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
    proto::FrameReader reader(proto::kDefaultMaxFrameSize);
    reader.feed(std::as_bytes(std::span{raw}));
    std::vector<std::vector<std::byte>> out;
    for (;;) {
        const proto::FrameReader::Result frame = reader.next();
        if (frame.status != proto::FrameReader::Status::Frame) {
            break;
        }
        out.emplace_back(frame.payload.begin(), frame.payload.end());
    }
    return out;
}

/// A snapshot the builder accepts (two prims, a one-triangle mesh), so that mutations reach
/// the checks behind the first ones.
std::vector<std::byte> validSnapshot() {
    const auto pack = [](const auto& values) {
        return std::string(reinterpret_cast<const char*>(values.data()),
                           values.size() * sizeof(values[0]));
    };
    v1::Envelope envelope;
    v1::SceneSnapshot& snapshot = *envelope.mutable_snapshot();
    snapshot.set_sequence(1);
    auto* root = snapshot.add_prims();
    root->set_id(0);
    root->set_parent(kNone);
    root->set_name("robot");
    root->set_mesh(kNone);
    root->set_material(kNone);
    auto* child = snapshot.add_prims();
    child->set_id(1);
    child->set_parent(0);
    child->set_name("wheel");
    child->set_mesh(0);
    child->set_material(0);
    auto* mesh = snapshot.add_meshes();
    mesh->set_vertex_count(3);
    mesh->set_index_count(3);
    mesh->mutable_positions()->set_inline_(pack(std::vector<float>{0, 0, 0, 1, 0, 0, 0, 1, 0}));
    mesh->mutable_normals()->set_inline_(pack(std::vector<float>{0, 0, 1, 0, 0, 1, 0, 0, 1}));
    mesh->mutable_indices()->set_inline_(pack(std::vector<std::uint32_t>{0, 1, 2}));
    auto* material = snapshot.add_materials();
    material->set_name("steel");
    for (const float c : {0.1F, 0.2F, 0.3F, 1.0F}) {
        material->add_base_color(c);
    }
    std::string transforms;
    for (int p = 0; p < 2; ++p) {
        const std::array<double, 3> position{1.0 * p, 2.0, 3.0};
        const std::array<float, 7> rest{1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F};
        transforms += pack(position) + pack(rest);
    }
    snapshot.mutable_transforms()->set_inline_(transforms);
    const std::string bytes = envelope.SerializeAsString();
    const auto* data = reinterpret_cast<const std::byte*>(bytes.data());
    return {data, data + bytes.size()};
}

TEST(SceneFuzz, MutatedFramesNeverBreakTheSceneBuilder) {
    std::vector<std::vector<std::byte>> seeds = goldenPayloads();
    ASSERT_GE(seeds.size(), 20U);
    seeds.push_back(validSnapshot());
    std::mt19937_64 random(20261029);
    SceneBuilder builder;
    std::size_t accepted = 0;
    constexpr int kMutationsPerSeed = 2000;
    for (const std::vector<std::byte>& seed : seeds) {
        accepted += fuzz::runSceneBuilder(builder, seed) ? 1 : 0;
        for (int m = 0; m < kMutationsPerSeed; ++m) {
            std::vector<std::byte> input = seed;
            const int edits = 1 + static_cast<int>(random() % 4);
            for (int e = 0; e < edits && !input.empty(); ++e) {
                const std::size_t at = random() % input.size();
                switch (random() % 4) {
                    case 0:  // flip a byte
                        input[at] = static_cast<std::byte>(random() & 0xFF);
                        break;
                    case 1:  // cut short
                        input.resize(at);
                        break;
                    case 2:  // repeat a piece
                        input.insert(input.begin() + static_cast<std::ptrdiff_t>(at), input.begin(),
                                     input.begin() + static_cast<std::ptrdiff_t>(
                                                         std::min<std::size_t>(at, 64)));
                        break;
                    default:  // a varint (a length or a count) made large
                        input[at] = std::byte{0xFF};
                        break;
                }
            }
            accepted += fuzz::runSceneBuilder(builder, input) ? 1 : 0;
            if (const auto frame = builder.frame()) {
                // Whatever was accepted is consistent: one transform per prim.
                ASSERT_EQ(frame->transforms.size(), frame->statics->prims.size());
            }
        }
    }
    EXPECT_GT(accepted, 0U) << "the valid snapshot, at least, is accepted";
}

}  // namespace
}  // namespace lagrange::viewer
