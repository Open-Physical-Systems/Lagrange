#include "BuilderHost.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <variant>

namespace lagrange::viewer {

BuilderHost::BuilderHost(BuilderSession& session, ViewerClient& client)
    : session_(session), client_(client) {
    auto loaded = builder::SensorCatalog::load(builder::catalogFiles());
    if (auto* catalog = std::get_if<builder::SensorCatalog>(&loaded)) {
        session_.catalog = std::move(*catalog);
    } else {
        const auto& error = std::get<builder::SensorCatalog::LoadError>(loaded);
        session_.catalogError =
            error.file + ":" + std::to_string(error.line) + ": " + error.message;
    }
}

void BuilderHost::afterUi() {
    // Validation runs on a document change, not every frame (R15).
    if (session_.history.revision() != validatedRevision_) {
        validatedRevision_ = session_.history.revision();
        session_.problems.builder = builder::validate(
            session_.document(), &session_.catalog,
            session_.path.empty() ? std::filesystem::path{} : session_.path.parent_path());
    }
    // WP-F carries these out. Until then they are dropped, so none stays set across frames.
    session_.testRequested = false;
    session_.backRequested = false;
    session_.saveRequested = false;
    session_.openRequested = false;
    session_.exportUsdRequested = false;
    session_.checkRequested = false;
    session_.templateRequested = false;
    (void)client_;
}

std::shared_ptr<const SceneFrame> BuilderHost::frameToDraw(Camera& camera) {
    camera = session_.camera;
    const builder::Document& document = session_.document();
    const bool rebuilt = session_.history.revision() != builtRevision_;
    if (rebuilt) {
        builtRevision_ = session_.history.revision();
        built_ = builder::buildStatics(document, ++generation_);
    }
    if (rebuilt || !frame_ || session_.preview != posedPreview_) {
        posedPreview_ = session_.preview;
        const builder::Tree tree = builder::buildTree(document);
        frame_ = builder::poseScene(built_, builder::linkFrames(document, tree, posedPreview_));
    }
    return frame_;
}

}  // namespace lagrange::viewer
