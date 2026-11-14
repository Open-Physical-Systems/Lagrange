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
        std::string checkTable;                 ///< lagrange check's table, verbatim
        int checkExit{-1};                      ///< -1: not run
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

    /// The document as it is now (history.current()).
    [[nodiscard]] const builder::Document& document() const { return history.current(); }
};

}  // namespace lagrange::viewer
