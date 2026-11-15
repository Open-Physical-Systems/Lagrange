#pragma once

// Carries out what the Builder's panels asked for, after drawPanels() (plan decision 3), and
// gives the renderer the Builder view's scene. Owners: WP-D (frameToDraw, overlay lines) and
// WP-F (afterUi: Test, Back, Save, Open, Export USD, Check loads, templates, autosave).

#include "lagrange/viewer/BuilderSession.hpp"
#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/SceneRenderer.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/builder/CollisionSplit.hpp"
#include "lagrange/viewer/builder/SceneAdapter.hpp"
#include "lagrange/viewer/builder/TestServer.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
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
        // A split in progress too, so that --input's wait:idle waits for its parts; and a
        // request the UI made that afterUi has not acted on yet (Export USD, Check loads, a
        // xacro to open), so that a wait right after the click waits for its process (QA2 F2).
        return !jobs_.empty() || test_.state() == builder::TestServer::State::Stopping ||
               session_.split.running || session_.exportUsdRequested || session_.checkRequested ||
               session_.openRequested || session_.saveRequested || session_.templateRequested;
    }
    /// Save to `target` now (atomic). False with the reason in Problems. Headless
    /// --builder-save, and Save / Save as.
    bool save(const std::filesystem::path& target);
    /// Open a .urdf now, or start expanding a .xacro (it opens when xacro is done).
    void open(const std::filesystem::path& path);
    /// This viewer's autosave of the document (RFC 0016 §10; code review L6):
    /// builder::autosaveFile in $XDG_STATE_HOME/lagrange/builder, keyed by the document's file
    /// and robot name, with this viewer's pid.
    [[nodiscard]] std::filesystem::path autosavePath() const;
    /// The windowed viewer's Save as… dialog chose `path`: saved there, with ".urdf" added when
    /// it lacks it; when that added name exists, asked first (code review L7).
    void saveAsChosen(const std::filesystem::path& path);
    /// The Save as… dialog was closed without a choice: what waited for that save (an Open
    /// after "Save" in the unsaved-changes prompt) does not happen.
    void saveAsCancelled();
    /// The windowed viewer has file dialogs (the unsaved-changes prompt's Save as…).
    void setCanOpenFiles(bool value) { canOpenFiles_ = value; }
    /// The folder the document's relative meshes resolve against (BuilderSession::meshFolder,
    /// else the working directory).
    [[nodiscard]] std::filesystem::path meshFolder() const;

private:
    /// A child process that is not the test server: Export USD, Check loads, xacro.
    struct Job {
        enum class Kind : std::uint8_t { Export, Check, Xacro } kind{Kind::Export};
        builder::ChildProcess process;
        std::filesystem::path source;  ///< xacro: the file asked for
        std::filesystem::path output;  ///< export: the .usda; xacro: the expanded URDF
        std::uint64_t opened{0};       ///< xacro: session.opened when it started
    };
    /// A new document asked for while the current one had unsaved edits (code review M2):
    /// what to open once the prompt is answered (Save, Discard).
    struct PendingReplace {
        enum class What : std::uint8_t { None, Open, Template } what{What::None};
        std::filesystem::path path;  ///< Open: the file
        std::string name;            ///< Template: its name
        bool waitingForSave{false};  ///< "Save as…" was chosen: replaced once that save is done
    };
    /// Seconds a test server may take to answer before the Test gives up.
    static constexpr double kStartTimeoutS = 60.0;

    void pollTest(double now);
    void startTest(double now);
    /// Test into the connected run (RFC 0016 §7.2): ImportRobot with the document's text, and
    /// RemoveRobot on Back. The client stays on the user's run.
    void startTestInRun(double now);
    void pollTestInRun(double now);
    /// Answers to RemoveRobot (a refusal goes to Problems), and a robot whose import was
    /// answered after Back: removed again.
    void pollRemovals();
    void endTest(const std::string& failure);
    /// While Testing (RFC 0016 §7.3): watches joint/<j>/position of the document's moving
    /// joints into liveJoints, and sends the sliders' jointTargets as CommandJoint.
    void driveJoints(double now);
    /// Stops the joint readouts and forgets the targets (a test ended).
    void stopDriving();
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
    /// Open, a dropped file or a template: now, or after the unsaved-changes prompt.
    void requestReplace(PendingReplace request);
    void askUnsaved();
    /// Carries out the answer to session.prompt, once given.
    void answerPrompt();
    void replacePending();
    /// Offers back an autosave a viewer that did not close left of this document (newer than
    /// its file); removes those the file is newer than (code review L6).
    void offerAutosave();
    void restoreAutosave(const std::filesystem::path& file);
    static void removeAutosave(const std::filesystem::path& file);
    void fileProblem(builder::Level level, std::string item, std::string message);
    [[nodiscard]] bool jobRunning(Job::Kind kind) const;
    /// The document written for a tool (absolute meshes) in the runtime folder; empty and
    /// `error` set when it could not be written.
    std::filesystem::path writeCopy(const std::string& name, std::string& error);
    /// The overlay written for a tool beside such a copy (absolute assets; RFC 0016 phase 3):
    /// `<stem of name>.overlay.yaml`; empty when the document needs none or it failed (`error`).
    std::filesystem::path writeOverlayCopy(const std::string& name, std::string& error);
    /// $XDG_RUNTIME_DIR/lagrange/builder/<pid>/ (plan decision 4), made on first use, 0700 and
    /// checked to be this user's own (code review L2: the /tmp/lagrange-<uid> fallback could
    /// have been made by someone else). Empty, and the reason in `error`, when it cannot be.
    std::filesystem::path runtimeDirectory(std::string& error);
    [[nodiscard]] std::filesystem::path program(std::string_view name) const;

    std::string serverProgram_;
    std::filesystem::path runtime_;
    builder::TestServer test_;
    std::vector<std::unique_ptr<Job>> jobs_;
    proto::Endpoint previousEndpoint_;
    bool previousRunning_{false};
    std::uint64_t snapshotsAtStart_{0};
    bool inRun_{false};  ///< the Test runs in the connected run
    /// Test ▾ → Start at…: the start pose of the test server's run. The Builder view's camera
    /// is carried there when the run is shown (the robot seen as in Build) and back by Back.
    builder::SpawnPose spawn_;
    bool cameraAtSpawn_{false};
    /// Moves the Builder view's camera from the robot's frame into the run's (`into`) or back.
    void carryCamera(bool into);
    std::uint64_t importRequest_{0};      ///< ImportRobot's request id while Starting
    std::uint64_t removeAfterImport_{0};  ///< Back while Starting: this import's robot goes
    std::uint64_t removeRequest_{0};      ///< RemoveRobot's request id until answered
    /// One joint of the tested robot: its name in the run, the readout of its position, and
    /// the command sent last (rate limit: kCommandEveryS).
    struct DrivenJoint {
        std::string runName;
        std::shared_ptr<ScopeBuffer> position;
        double sent{std::numeric_limits<double>::quiet_NaN()};
        double sentAt{-1.0};
        std::uint64_t request{0};  ///< CommandJoint not answered yet; 0: none
    };
    static constexpr double kCommandEveryS = 1.0 / 30.0;  // ≤ 30 commands a second per joint
    std::map<std::string, DrivenJoint, std::less<>> driven_;
    double watchedAt_{-1.0};  ///< when the catalog was last searched for missing readouts
    double startedAt_{0.0};
    double logReadAt_{0.0};
    std::uint64_t autosavedRevision_{0};
    std::filesystem::path autosaveWritten_;  ///< the autosave this viewer wrote last
    bool autosavesChecked_{false};           ///< the first frame looked for left autosaves
    std::filesystem::path restoreFrom_;      ///< with the Restore prompt: the autosave offered
    std::filesystem::path overwriteTarget_;  ///< with the Overwrite prompt: Save as… target
    PendingReplace pending_;
    bool canOpenFiles_{false};
    std::uint64_t jobSerial_{0};    ///< names each xacro's output and log apart
    std::uint64_t splitOpened_{0};  ///< session.opened when the split started
    /// An overlay beside the opened URDF that did not read: Save leaves it as it is, rather
    /// than writing the document's (empty) overlay over it.
    std::filesystem::path unreadOverlay_;
    std::uint64_t validatedOpened_{0};
    std::uint64_t builtOpened_{0};

    BuilderSession& session_;
    ViewerClient& client_;
    std::uint64_t validatedRevision_{~std::uint64_t{0}};
    std::uint64_t builtRevision_{~std::uint64_t{0}};
    std::uint64_t generation_{0};
    bool builtCollision_{false};  // the collision toggle the statics were built with
    // Addendum A: "Split into convex parts" (CoACD) as a polled child process.
    builder::CollisionSplit split_;
    void runSplit();
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
