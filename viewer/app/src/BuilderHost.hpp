#pragma once

// Carries out what the Builder's panels asked for, after drawPanels() (plan decision 3), and
// gives the renderer the Builder view's scene. Owners: WP-D (frameToDraw, overlay lines) and
// WP-F (afterUi: Test, Back, Save, Open, Export USD, Check loads, templates, autosave).

#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"

#include <cstdint>
#include <memory>

namespace lagrange::viewer {

class BuilderHost {
public:
    /// Loads the sensor catalog (builder::catalogFiles()) into the session.
    BuilderHost(BuilderSession& session, ViewerClient& client);

    /// After drawPanels(), on the UI thread, every frame: carries out and clears the session's
    /// out-requests, and validates the document again when it changed. Never blocks (child
    /// processes are polled, CLAUDE.md §8).
    void afterUi();

    /// While the Builder is shown in Build mode: the scene to draw instead of the server's, and
    /// `camera` set to the Builder view's own (R3). The statics are rebuilt only when the
    /// document changed; a preview change makes a new frame with the same statics (plan
    /// decision 2), so the renderer's view cache keys on the returned pointer. nullptr: nothing
    /// to draw.
    [[nodiscard]] std::shared_ptr<const SceneFrame> frameToDraw(Camera& camera);

private:
    BuilderSession& session_;
    ViewerClient& client_;
    std::uint64_t validatedRevision_{~std::uint64_t{0}};
    std::uint64_t builtRevision_{~std::uint64_t{0}};
    std::uint64_t generation_{0};
    builder::BuiltScene built_;
    builder::Preview posedPreview_;
    std::shared_ptr<const SceneFrame> frame_;
};

}  // namespace lagrange::viewer
