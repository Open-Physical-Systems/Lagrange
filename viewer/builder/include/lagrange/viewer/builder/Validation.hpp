#pragma once

// The builder's own, live validation (RFC 0016 §8.1). Owner: WP-A. The server's loader stays
// the judge (§8.2); this catches what can be caught without it.

#include "lagrange/viewer/builder/Document.hpp"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

class SensorCatalog;
struct CatalogEntry;

enum class Level {
    Error,    ///< Test is refused while one exists (Save still works)
    Warning,  ///< legal but likely wrong
    Hint,     ///< legal, probably unmeant (a sphere caster on a fixed joint slides)
};

/// One finding. `item` names the link, joint or sensor it is about (empty: the robot).
struct Problem {
    Level level;
    std::string item;
    std::string message;  ///< one sentence, what is wrong and what to do
};

/// Every rule of RFC 0016 §8.1: names valid (validName) and unique; one root, no loops, every
/// joint's parent and child exist; revolute and prismatic joints have lower < upper, effort > 0,
/// velocity > 0 (empty is an error); a non-zero axis; positive mass and a valid inertia; a
/// mimic's leader exists; sensor names unique, parameters within the catalog's min and max and
/// of the right type (when `catalog` is not null); meshes found, relative paths against
/// `meshBase` (the saved file's folder; empty: relative meshes are not checked),
/// `lagrange-asset://` through $LAGRANGE_ASSET_DIR, `package://` through the ament index (R12).
/// Errors first, then warnings, then hints; within a level in document order.
[[nodiscard]] std::vector<Problem> validate(const Document& document, const SensorCatalog* catalog,
                                            const std::filesystem::path& meshBase);

/// The catalog rules for one sensor against its entry: every parameter is one the entry lists
/// (a warning otherwise), parses as its type (Number, Int, Bool "true"/"false"/"1"/"0", Enum one
/// of its options; Text and List are free text) and lies within its min and max. validate()
/// applies it to each sensor whose type the catalog finds; the Properties form can use it for
/// one sensor. Added by WP-A (not in the WP0 interface list), additive.
[[nodiscard]] std::vector<Problem> checkSensorParams(const Sensor& sensor,
                                                     const CatalogEntry& entry);

/// What a sensor needs before the server starts it, from its catalog entry (QA F5): each
/// `required` parameter given; a given joint or camera parameter naming one of the robot's; each
/// `requires` item met (an error, or a warning for one the world may give). validate() applies
/// it to each sensor whose type the catalog finds. In SensorNeeds.cpp.
[[nodiscard]] std::vector<Problem> checkSensorNeeds(const Document& document, const Sensor& sensor,
                                                    const CatalogEntry& entry);

/// Sensor steps the scheduler refuses (QA F10): two sensors whose periods (as the server rounds
/// them, the sonar's at least its ping) line up only after more than 10 s, a warning naming both.
/// validate() includes these when it has a catalog. In SensorNeeds.cpp.
[[nodiscard]] std::vector<Problem> checkSensorRates(const Document& document,
                                                    const SensorCatalog& catalog);

/// The overlay's rules (RFC 0016 phase 3; the stage loader's, modules/electrical/network): a
/// form's required values, numbers within the schema's range, an ascending ocvSoc as long as
/// ocvVoltage, a bus's overvoltage above its undervoltage; a motor on a joint that exists, moves
/// (not fixed) and has no other motor, drawing from a bus of the overlay; a battery feeding a
/// bus; a bus fed by a battery or a converter; an entry's prim naming an item of the document; an
/// overlay sensor at its link's origin (the overlay has no mount pose). Items: the joint a motor
/// drives, else `power/<name>` for a battery or bus. validate() includes these.
[[nodiscard]] std::vector<Problem> checkOverlay(const Document& document);

/// The server's own name rule (`validName`, server/src/Server.cpp): ASCII letters, digits and
/// '_', not empty, not starting with a digit. Applied to the robot, links, joints and sensors
/// so that a robot that saves also spawns.
[[nodiscard]] bool validName(std::string_view name);

[[nodiscard]] std::string_view toString(Level level);  // "error", "warning", "hint"

}  // namespace lagrange::viewer::builder
