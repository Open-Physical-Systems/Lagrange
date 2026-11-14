// Stub (RFC 0016 WP0): WP-D replaces this file (plan decisions 1 and 2). Until then the scene is
// empty: no prims, nothing drawn.
#include "lagrange/viewer/builder/SceneAdapter.hpp"

namespace lagrange::viewer::builder {

BuiltScene buildStatics(const Document& document, std::uint64_t generation) {
    auto statics = std::make_shared<SceneStatics>();
    statics->generation = generation;
    statics->stagePath = "builder:" + document.robot;
    BuiltScene scene;
    scene.statics = std::move(statics);
    return scene;
}

std::shared_ptr<const SceneFrame> poseScene(const BuiltScene& scene,
                                            const std::vector<Frame>& /*links*/) {
    auto frame = std::make_shared<SceneFrame>();
    frame->statics = scene.statics;
    frame->transforms.resize(scene.statics ? scene.statics->prims.size() : 0);
    return frame;
}

}  // namespace lagrange::viewer::builder
