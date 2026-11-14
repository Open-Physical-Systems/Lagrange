#pragma once

// Carries out what the Builder's panels asked for, after drawPanels() (plan decision 3), and
// gives the renderer the Builder view's scene. Owners: WP-D (frameToDraw, overlay lines) and
// WP-F (afterUi: Test, Back, Save, Open, Export USD, Check loads, templates, autosave).

#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"
#include "lagrange/viewer/builder/TestServer.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer {

class BuilderHost {
public:
    /// Loads the sensor catalog (builder::catalogFiles()) into the session.
    BuilderHost(BuilderSession& session, ViewerClient& client);
    /// Stops a running test server and any tool, and removes the runtime folder.
    ~BuilderHost();
    BuilderHost(const BuilderHost&) = delete;
    BuilderHost& operator=(const BuilderHost&) = delete;

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

    /// The Builder view's overlay lines for the frame frameToDraw returned last (SPEC "Builder
    /// view": link frames, joint axes, collision shapes, sensor rays and frustums, the
    /// selection in accent). Added by WP-D. Valid until the next frameToDraw.
    [[nodiscard]] std::span<const OverlayLines> overlays() const { return overlays_; }
    /// Grows whenever overlays() changes (selection, toggles, a new frame), for the view key.
    [[nodiscard]] std::uint64_t overlayVersion() const { return overlayVersion_; }

    /// lagrange_server as the Launcher found it (App: findLocalServer); empty: look it up in the
    /// ament prefixes.
    void setServerProgram(std::string program) { serverProgram_ = std::move(program); }
    /// True while a Test owns the client's connection (Starting or Testing): the App then
    /// neither re-aims the Scene camera nor adds the test run to the Recent list (R3).
    [[nodiscard]] bool ownsConnection() const {
        return session_.mode != BuilderSession::Mode::Build;
    }
    /// True while a child process runs or a Test is stopping (headless `wait:idle`).
    [[nodiscard]] bool busy() const {
        return !jobs_.empty() || test_.state() == builder::TestServer::State::Stopping;
    }
    /// Save to `target` now (atomic). False with the reason in Problems. Headless
    /// --builder-save, and Save / Save as.
    bool save(const std::filesystem::path& target);
    /// Open a .urdf now, or start expanding a .xacro (it opens when xacro is done).
    void open(const std::filesystem::path& path);
    /// $XDG_STATE_HOME/lagrange/builder/<robot>.autosave.urdf (RFC 0016 §10).
    [[nodiscard]] static std::filesystem::path autosavePath(const std::string& robot);

private:
    /// A child process that is not the test server: Export USD, Check loads, xacro.
    struct Job {
        enum class Kind : std::uint8_t { Export, Check, Xacro } kind{Kind::Export};
        builder::ChildProcess process;
        std::filesystem::path source;  ///< xacro: the file asked for
        std::filesystem::path output;  ///< export: the .usda; xacro: the expanded URDF
    };
    /// Seconds a test server may take to answer before the Test gives up.
    static constexpr double kStartTimeoutS = 60.0;

    void pollTest(double now);
    void startTest(double now);
    void endTest(const std::string& failure);
    void restoreClient();
    void pollJobs();
    void exportUsd();
    void checkLoads();
    void readCheck(const builder::ChildProcess& process);
    void openTemplate(const std::string& name);
    bool openText(const std::string& text, const std::filesystem::path& from,
                  const std::filesystem::path& path, const std::string& label);
    void replaceDocument(builder::Document document, const std::filesystem::path& path,
                         const std::string& label);
    void autosave(double now);
    void fileProblem(builder::Level level, std::string item, std::string message);
    [[nodiscard]] bool jobRunning(Job::Kind kind) const;
    /// The document written for a tool (absolute meshes) in the runtime folder; empty and
    /// `error` set when it could not be written.
    std::filesystem::path writeCopy(const std::string& name, std::string& error);
    /// $XDG_RUNTIME_DIR/lagrange/builder/<pid>/ (plan decision 4), made on first use.
    std::filesystem::path runtimeDirectory();
    [[nodiscard]] std::filesystem::path program(std::string_view name) const;

    std::string serverProgram_;
    std::filesystem::path runtime_;
    builder::TestServer test_;
    std::vector<std::unique_ptr<Job>> jobs_;
    proto::Endpoint previousEndpoint_;
    bool previousRunning_{false};
    std::uint64_t snapshotsAtStart_{0};
    double startedAt_{0.0};
    double logReadAt_{0.0};
    std::uint64_t autosavedRevision_{0};
    std::uint64_t validatedOpened_{0};
    std::uint64_t builtOpened_{0};

    BuilderSession& session_;
    ViewerClient& client_;
    std::uint64_t validatedRevision_{~std::uint64_t{0}};
    std::uint64_t builtRevision_{~std::uint64_t{0}};
    std::uint64_t generation_{0};
    builder::BuiltScene built_;
    builder::Preview posedPreview_;
    std::shared_ptr<const SceneFrame> frame_;
    std::vector<builder::Frame> links_;  // the link frames frame_ was posed with
    // The overlays and what they were built for.
    void buildOverlays(const std::vector<builder::Frame>& links);
    /// The plane the grid is drawn on: the lowest point of the robot as posed (m).
    [[nodiscard]] double groundHeight() const;
    std::vector<OverlayLines> overlays_;
    std::uint64_t overlayVersion_{0};
    const SceneFrame* overlaysFrame_{nullptr};
    BuilderSession::Selection overlaysSelection_;
    BuilderSession::ViewToggles overlaysToggles_;
};

}  // namespace lagrange::viewer
