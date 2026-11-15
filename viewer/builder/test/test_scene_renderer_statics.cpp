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
