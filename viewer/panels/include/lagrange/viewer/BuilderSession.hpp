#pragma once

// The Builder workspace's UI-side state (RFC 0016, plan §2.1). Builder panels read and edit it
// during a frame; they never start processes or touch the socket (plan decision 3). What they
// want done is written into the out-requests below, which BuilderHost (viewer/app/src) carries
// out after drawPanels() and then clears.

#include "lagrange/viewer/Camera.hpp"
#include "lagrange/viewer/builder/Document.hpp"
#include "lagrange/viewer/builder/History.hpp"
#include "lagrange/viewer/builder/Kinematics.hpp"
#include "lagrange/viewer/builder/SensorCatalog.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace lagrange::viewer {

/// ImGui window IDs of the Builder's panels. Never "###Problems", "Console" or another
/// workspace's name: a window lives in one dock node (R5). Titles with a count are drawn as
/// "Joints · 4 moving###BuilderJoints"; layouts dock them by these IDs.
namespace builder_windows {
inline constexpr const char* kRobot = "Robot###BuilderRobot";
inline constexpr const char* kFiles = "Files###BuilderFiles";
inline constexpr const char* kAdd = "Add###BuilderAdd";
inline constexpr const char* kTemplates = "Templates###BuilderTemplates";
inline constexpr const char* kView = "Builder view###BuilderView";
inline constexpr const char* kJoints = "Joints###BuilderJoints";
inline constexpr const char* kPoses = "Poses###BuilderPoses";
inline constexpr const char* kCatalog = "Sensor catalog###BuilderCatalog";
inline constexpr const char* kProblems = "Problems###BuilderProblems";
inline constexpr const char* kProperties = "Properties###BuilderProperties";
inline constexpr const char* kHistory = "History###BuilderHistory";
// Test layout (SPEC state 4).
inline constexpr const char* kDrive = "Drive###BuilderDrive";
inline constexpr const char* kTestConsole = "Console###BuilderConsole";
}  // namespace builder_windows

struct BuilderSession {
    enum class Mode : std::uint8_t {
        Build,     ///< editing; no simulation (the default)
        Starting,  ///< the test server was started; waiting for the client to be Ready
        Testing,   ///< connected to the test server; the document is read-only
    };
    /// What the selection points at. Nothing: kind None, name empty.
    struct Selection {
        enum class Kind : std::uint8_t { None, Robot, Link, Joint, Sensor } kind{Kind::None};
        std::string name;  ///< link, joint or sensor name (empty for None and Robot)
    };
    /// Problems by origin (SPEC "Problems"): the builder's own, the server's loader messages
    /// after a Test, and `lagrange check` (R13: its exit status and its table in mono).
    struct Problems {
        std::vector<builder::Problem> builder;  ///< validate(), run on every commit (R15)
        std::vector<builder::Problem> loader;   ///< from the test server's log / Console
        std::vector<builder::Problem> check;    ///< one row per line naming a link or joint
        /// Files and tools (SPEC state 6): a file that could not be read or written, Export
        /// USD's and xacro's outcome. Not counted by errors(): they never refuse a Test.
        std::vector<builder::Problem> files;
        std::string checkTable;  ///< lagrange check's table, verbatim
        int checkExit{-1};       ///< -1: not run
        /// Number of Level::Error across all three.
        [[nodiscard]] int errors() const;
    };

    BuilderSession();

    builder::History history;  ///< the document is history.current()
    builder::SensorCatalog catalog;
    std::string catalogError;  ///< a catalog file that did not load: shown in Problems
    Selection selection;
    builder::Preview preview;  ///< joint preview positions (rad or m); view state, not undoable
    Problems problems;
    /// The Builder view's own camera (R3): never the Scene viewport's.
    Camera camera;
    /// The Builder view's tool column (SPEC "Builder view"; Q W E A M). Added by WP-D.
    enum class Tool : std::uint8_t { Select, Move, Rotate, JointAxis, Measure };
    Tool tool{Tool::Select};
    /// The Builder view's toggles, top right (SPEC "Builder view"); view state, not undoable.
    /// Added by WP-D: BuilderHost draws the overlay lines they ask for.
    struct ViewToggles {
        bool grid{true};
        bool frames{true};  ///< link frames and joint axes
        bool collision{false};
        bool com{false};     ///< centre of mass (needs WP-A's robotMass)
        bool sensors{true};  ///< sensor rays and frustums
        bool names{true};    ///< labels of the selection and of previewed joints
    };
    ViewToggles view;
    Mode mode{Mode::Build};
    /// The file the document was opened from or last saved to; empty for a new document.
    std::filesystem::path path;
    double autosaveTime{0.0};  ///< ImGui::GetTime() of the last autosave, s; 0: none yet
    bool showDegrees{true};    ///< Joints panel's deg | rad (display only; files hold rad)
    std::string testTarget;    ///< "Test · empty world ▾" readout: "empty world", a file name
    /// The Test target is the run this viewer is connected to (RFC 0016 §7.2): the robot is
    /// imported into it as text and removed again by Back, instead of a test server's own run.
    bool testIntoRun{false};
    /// Grows whenever a new document replaces the history (Open, a template): views that key on
    /// history.revision() key on this too, since a new History counts from 0 again.
    std::uint64_t opened{0};
    /// The last file error (SPEC state 6), for the err pill of the Files panel; empty: none.
    std::string fileError;
    /// What a child process is doing now ("Exporting USD…"); empty: nothing runs.
    std::string busy;

    // The test (SPEC state 4), written by BuilderHost for the Drive and Console panels.
    std::string testLog;  ///< the tail of the test server's log, refreshed a few times a second
    std::string testEndpoint;     ///< "unix:/run/user/1000/lagrange/builder-4242.sock"
    std::string testRosDomain;    ///< the test server's ROS_DOMAIN_ID; empty: started with --no-ros
    std::uint64_t testSerial{0};  ///< grows with every Test started (Drive comes to the front)
    /// Into the connected run: the robot's name there, as the server answered ("crate_2" when
    /// "crate" was taken); empty for a test server's own run, or before the answer.
    std::string testRobotName;
    /// Into the connected run: the run already had a robot, so this one is passive (nothing
    /// drives it, no ROS topics of its own).
    bool testPassive{false};

    // Out, read and cleared by BuilderHost::afterUi().
    bool testRequested{false};
    std::string testWorld;      ///< with testRequested: "none", a sample world, or a world file
    bool backRequested{false};  ///< Back to Build (Esc while testing)
    bool saveRequested{false};
    std::filesystem::path savePath;  ///< with saveRequested: empty = `path` (Save), else Save as
    bool openRequested{false};
    std::filesystem::path openPath;  ///< with openRequested: a .urdf or .xacro to open
    bool exportUsdRequested{false};  ///< Export USD… (lagrange_import), next to `path`
    bool checkRequested{false};      ///< Check loads… (lagrange_check)
    bool templateRequested{false};
    std::string templateName;  ///< with templateRequested: assets/robots/templates/<name>.urdf
    /// The windowed viewer's file dialogs (SDL): Open… (.urdf, .xacro) and Save as… (.urdf).
    /// The chosen file comes back as openRequested / saveRequested with its path.
    bool openDialogRequested{false};
    bool saveAsDialogRequested{false};

    // RFC 0016 Addendum A: meshes and how a link collides. Out, read by BuilderHost::afterUi(),
    // which runs the split (CoACD, as `lagrange bake`) as a polled child process.
    bool splitRequested{false};
    std::string splitLink;  ///< with splitRequested: the link whose mesh is split
    bool splitCancelRequested{false};
    double splitThreshold{0.05};  ///< CoACD's concavity threshold (bake.py's default); form state
    int splitSeed{0};             ///< CoACD's seed (bake.py's default); form state
    /// In, from BuilderHost: the split as it runs, for the Collision section.
    struct SplitStatus {
        std::string link;  ///< the link split last (empty: none yet)
        bool running{false};
        double seconds{0.0};  ///< since it started, while running
        bool failed{false};
        std::string message;  ///< "12 convex parts" or why it failed
    };
    SplitStatus split;
    /// Mesh files picked lately (Add → Mesh…), newest first; view state, not undoable.
    std::vector<std::string> recentMeshes;
    /// Add → Mesh… → Browse…: the windowed viewer's SDL file dialog for a mesh (App).
    bool meshDialogRequested{false};
    /// A mesh to add as a link on `addMeshOn` (empty: the Add panel's target link): chosen in
    /// that dialog, or a mesh file dropped on the window while the Builder is shown.
    bool addMeshRequested{false};
    std::string addMeshPath;
    std::string addMeshOn;
    bool addMeshMillimetres{false};

    /// The document as it is now (history.current()).
    [[nodiscard]] const builder::Document& document() const { return history.current(); }
};

}  // namespace lagrange::viewer
