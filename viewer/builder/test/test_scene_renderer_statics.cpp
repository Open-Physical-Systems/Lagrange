// The renderer's mesh cache keys on which statics it draws, not only their generation
// (RFC 0016 WP-F). The Builder's own document and a server's scene both count generations from
// 1; when the Builder's Test switched from one to the other, the server's prims were drawn with
// the document's meshes (rover_mk1's caster on top, its mast below). Renders on the software
// Vulkan device (llvmpipe) unless LAGRANGE_TEST_GPU names another; skipped without Vulkan.
#include "lagrange/gpu/Device.hpp"
#include "lagrange/gpu/Resources.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace builder = lagrange::viewer::builder;
using lagrange::viewer::Camera;
using lagrange::viewer::SceneFrame;
using lagrange::viewer::SceneRenderer;

namespace {

constexpr VkExtent2D kExtent{160, 120};

/// A one-link document whose only shape is `kind`, posed at rest, with statics `generation`.
std::shared_ptr<const SceneFrame> scene(builder::Shape::Kind kind, std::uint64_t generation) {
    builder::Document document = builder::newDocument("probe");
    builder::Shape& shape = document.links.front().shapes.front();
    shape.kind = kind;
    shape.size = kind == builder::Shape::Kind::Box ? std::array<double, 3>{0.6, 0.6, 0.02}
                                                   : std::array<double, 3>{0.15, 0.0, 0.0};
    const builder::BuiltScene built = builder::buildStatics(document, generation);
    return builder::poseScene(built, builder::previewFrames(document, {}));
}

std::vector<std::uint8_t> draw(const lagrange::gpu::Device& device, SceneRenderer& renderer,
                               const SceneFrame& frame) {
    Camera camera;
    camera.target = {0.0, 0.0, 0.0};
    camera.distance = 1.5;
    device.submitAndWait(
        [&](VkCommandBuffer commands) { renderer.render(commands, &frame, camera); });
    return lagrange::gpu::readbackRgba8(device, renderer.color(),
                                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

}  // namespace

TEST(SceneRenderer, AnotherScenesStaticsWithTheSameGenerationAreUploaded) {
    lagrange::gpu::DeviceDesc desc;
    desc.applicationName = "test_scene_renderer_statics";
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    desc.preferredDevice = gpu != nullptr ? gpu : "llvmpipe";
    std::string error;
    const std::unique_ptr<lagrange::gpu::Device> device =
        lagrange::gpu::Device::create(desc, error);
    if (!device) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    const auto plate = scene(builder::Shape::Kind::Box, 1);    // the Builder's document
    const auto ball = scene(builder::Shape::Kind::Sphere, 1);  // the test server's scene
    ASSERT_EQ(plate->statics->generation, ball->statics->generation);

    SceneRenderer fresh(*device);
    fresh.resize(kExtent);
    const std::vector<std::uint8_t> expected = draw(*device, fresh, *ball);

    SceneRenderer switched(*device);
    switched.resize(kExtent);
    const std::vector<std::uint8_t> before = draw(*device, switched, *plate);
    ASSERT_NE(before, expected);  // the two scenes look different
    const std::vector<std::uint8_t> after = draw(*device, switched, *ball);
    EXPECT_EQ(after, expected) << "the ball was drawn with the plate's meshes";
}

// Trees (TreeAPI_1) in the Lit shading (gpu/TreeShapes.hpp): a scene with no meshes of its own
// draws a tree from RenderOptions::trees as a brown trunk and a green crown where it stands.
TEST(SceneRenderer, LitDrawsATreesCrownWhereItStands) {
    lagrange::gpu::DeviceDesc desc;
    desc.applicationName = "test_scene_renderer_statics";
    const char* gpu = std::getenv("LAGRANGE_TEST_GPU");
    desc.preferredDevice = gpu != nullptr ? gpu : "llvmpipe";
    std::string error;
    const std::unique_ptr<lagrange::gpu::Device> device =
        lagrange::gpu::Device::create(desc, error);
    if (!device) {
        GTEST_SKIP() << "no Vulkan device: " << error;
    }
    auto statics = std::make_shared<lagrange::viewer::SceneStatics>();
    statics->generation = 1;
    SceneFrame frame;
    frame.statics = statics;
    lagrange::viewer::RenderOptions options;
    options.shading = lagrange::viewer::Shading::Lit;
    options.grid = false;
    options.axes = false;
    lagrange::viewer::RenderOptions::Tree& tree = options.trees.emplace_back();
    tree.foot = {0.0, 0.0, 0.0};
    tree.height = 6.0;
    tree.crownRadius = 1.8;
    tree.crownBase = 2.4;
    tree.trunkDiameter = 0.4;
    tree.seed = 12345;
    SceneRenderer renderer(*device);
    renderer.resize(kExtent);
    const auto shoot = [&](const lagrange::viewer::RenderOptions& o) {
        Camera camera;
        camera.target = {0.0, 0.0, 3.0};
        camera.distance = 14.0;
        camera.pitch = 0.1;
        device->submitAndWait(
            [&](VkCommandBuffer commands) { renderer.render(commands, &frame, camera, o); });
        return lagrange::gpu::readbackRgba8(*device, renderer.color(),
                                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    };
    const std::vector<std::uint8_t> with = shoot(options);
    // The crown's centre, 4.2 m up, a little over the view's centre (the target, 3 m up); the
    // trunk below it.
    const auto at = [&](std::uint32_t x, std::uint32_t y) {
        return &with[4 * ((std::size_t{y} * kExtent.width) + x)];
    };
    const std::uint8_t* crown = at(kExtent.width / 2, kExtent.height / 2 - 10);
    EXPECT_GT(crown[1], crown[0] + 10) << "the crown is green";
    EXPECT_GT(crown[1], crown[2] + 10);
    int green = 0;
    for (std::uint32_t y = 0; y < kExtent.height; ++y) {
        for (std::uint32_t x = 0; x < kExtent.width; ++x) {
            const std::uint8_t* p = at(x, y);
            green += p[1] > p[0] + 10 && p[1] > p[2] + 10 ? 1 : 0;
        }
    }
    // The crown's outline, about 3.6 m across seen from 14 m in a 0.9 rad view, is some 1/20 of
    // the picture; its clumps cover more than half of it.
    EXPECT_GT(green, static_cast<int>(kExtent.width * kExtent.height / 50));
    // Without the tree, none of it.
    lagrange::viewer::RenderOptions bare = options;
    bare.trees.clear();
    const std::vector<std::uint8_t> without = shoot(bare);
    const std::uint8_t* sky =
        &without[4 * ((std::size_t{kExtent.height / 2 - 10} * kExtent.width) + kExtent.width / 2)];
    EXPECT_FALSE(sky[1] > sky[0] + 10 && sky[1] > sky[2] + 10);
}
