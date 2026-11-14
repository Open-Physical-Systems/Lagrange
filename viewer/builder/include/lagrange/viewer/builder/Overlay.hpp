#pragma once

// The robot's overlay file (RFC 0016 phase 3; docs/design/formats.md §9): its power system
// (battery, bus, motors by joint) and the sensors URDF has no element for (GNSS, magnetometer,
// barometer, radar), written as `<robot>.overlay.yaml` beside the URDF and applied by
// `lagrange_server --overlay` and `lagrange_import --overlay`.
//
// The document holds the overlay as the file says it (Document::overlay: entries of schema
// instances by prim, values as typed, in the schema's units, SI), so an overlay of anything
// (thermal nodes, rotors, wheels) reads and writes back unchanged; the builder's forms edit the
// electrical schemas through the helpers below. Overlay-only sensors are Document::sensors (the
// tree, the catalog's form and Problems treat them as any sensor) and are written here, never
// into the URDF.
//
// Library rule (invariant 14): no ImGui, no Vulkan, no simulation module. The stage loader
// (io/stage, `applyOverlay`) stays the judge: Test runs the server with the file this writes.

#include "lagrange/viewer/builder/Document.hpp"

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace lagrange::viewer::builder {

// --- names ------------------------------------------------------------------------------------

/// The overlay file of a URDF: `rover.urdf` → `rover.overlay.yaml`, beside it (the samples'
/// rule: assets/robots/amr_diffdrive/amr_diffdrive.overlay.yaml).
[[nodiscard]] std::filesystem::path overlayPathFor(const std::filesystem::path& urdf);

/// The schema of a sensor type URDF cannot hold (RFC 0016 §5, the catalog's `overlay_only`):
/// "gnss" → "GnssAPI_1", "magnetometer", "barometer", "radar"; empty for any other type.
[[nodiscard]] std::string_view overlaySensorSchema(std::string_view type);
/// True for the four types above: written into the overlay, never into the URDF.
[[nodiscard]] bool isOverlaySensor(std::string_view type);

/// Prims the builder creates for the power system: `power/<name>` (the samples' layout).
inline constexpr std::string_view kPowerScope = "power/";

// --- reading and writing ----------------------------------------------------------------------

struct OverlayReadResult {
    bool ok{false};
    std::string error;  ///< why not, one line; empty on success
    /// What was read but changed (one line each): inline comments not kept, ...
    std::vector<std::string> notes;
};

/// Reads overlay YAML into `document`: its entries into document.overlay (replacing it), and
/// each entry that is an overlay-only sensor on a link (`prim: <link>`, `create_child: <name>`,
/// schemas SensorAPI_1 with sensor:rate, latency, jitter and one of the four sensor schemas)
/// into document.sensors, its place kept by a placeholder entry. Relative asset paths (an
/// `asset` attribute of a schema: aero:rotor:propeller, wheel:tyre, ...) are made absolute
/// against `base`, the file's folder (empty: kept as they are). A schema_version other than 1,
/// or text that is not an overlay, is refused and leaves `document` unchanged.
[[nodiscard]] OverlayReadResult readOverlay(std::string_view yaml, Document& document,
                                            const std::filesystem::path& base = {});

struct OverlayWriteOptions {
    /// Where the file will be saved: absolute asset paths under its folder's parent tree are
    /// written relative to its folder (as the samples write them). Empty: as the document has
    /// them.
    std::filesystem::path savedAt;
    /// Every asset path absolute (the Test copy, which lives elsewhere).
    bool absoluteAssets{false};
};

/// The overlay as YAML: the header comment kept, `schema_version: 1`, then `apply:` with the
/// entries in document order, each overlay-only sensor in its placeholder's place (new ones at
/// the end), a unit comment after each value the forms know. Deterministic. Empty text when the
/// document needs no overlay (hasOverlay false).
[[nodiscard]] std::string writeOverlay(const Document& document,
                                       const OverlayWriteOptions& options = {});

/// True when the document has anything for the overlay: an entry, or an overlay-only sensor.
[[nodiscard]] bool hasOverlay(const Document& document);

/// Two overlay texts mean the same (the round-trip tests): the same YAML trees, numbers equal as
/// numbers ("10" and "10.0"), comments and layout ignored. `why` (if given) says where they
/// differ first.
[[nodiscard]] bool sameOverlay(std::string_view a, std::string_view b, std::string* why = nullptr);

// --- the electrical forms (formats.md §9 keys, schemas/electrical/schema.usda) ----------------

/// One attribute a form shows: its name in the schema, its type, its unit (the schema's
/// `lagrangeUnit`), its fallback (the schema's default, "" when it has none), whether the stage
/// loader requires it, and the loader's range rule.
struct OverlayField {
    enum class Type { Real, Int, Token, RealArray, Rel, Asset } type{Type::Real};
    /// PositiveFraction: above 0, at most 1 (a converter's efficiency, the loader's (0, 1]).
    enum class Range {
        Any,
        Positive,
        NonNegative,
        Fraction,
        PositiveFraction,
        AtLeastOne
    } range{Range::Any};
    std::string_view name;      ///< "elec:windingResistance"; for a port, after its prefix
    std::string_view unit;      ///< "ohm", "N.m/A", "1"; empty for tokens and relationships
    std::string_view fallback;  ///< the schema's default as text; empty: none
    bool required{false};
    std::span<const std::string_view> tokens;  ///< Token: the allowed values
    std::string_view meaning;                  ///< the schema's doc, short
};

/// The fields of BatteryAPI_1, ElectricalBusAPI_1, ElectricalMotorAPI_1, of a battery's
/// output port (PortAPI_1:dc_out, names after "elec:port:dc_out:"), and of the parts between
/// buses (RFC 0010): ElectricalLoadAPI_1, ElectricalConverterAPI_1, ElectricalBranchAPI_1, in
/// the schema's order.
[[nodiscard]] std::span<const OverlayField> batteryFields();
[[nodiscard]] std::span<const OverlayField> busFields();
[[nodiscard]] std::span<const OverlayField> motorFields();
[[nodiscard]] std::span<const OverlayField> portFields();
[[nodiscard]] std::span<const OverlayField> loadFields();
[[nodiscard]] std::span<const OverlayField> converterFields();
[[nodiscard]] std::span<const OverlayField> branchFields();
/// The fields of a schema the forms edit (by its name; a port's "PortAPI_1:<x>"); empty for
/// any other.
[[nodiscard]] std::span<const OverlayField> fieldsOf(std::string_view schema);
/// Whether a branch form shows `field` for a branch of `kind` ("fuse": i2tRating, "diode":
/// forwardDrop, "switch": initialState; resistance and the two ends for all).
[[nodiscard]] bool branchFieldApplies(std::string_view kind, std::string_view field);
/// The field of `schema` named `attribute` (port attributes by their full name); nullptr if the
/// forms do not know it.
[[nodiscard]] const OverlayField* overlayField(std::string_view schema, std::string_view attribute);
/// "Ω", "N·m/A", "kg·m²": a schema unit as the fields show it.
[[nodiscard]] std::string displayUnit(std::string_view unit);

/// Checks a value as typed against its field: "" when acceptable (empty is acceptable unless the
/// field is required), else one line ("must be above 0"). Arrays are comma-separated numbers.
[[nodiscard]] std::string checkField(const OverlayField& field, std::string_view text);

// --- the power system ----------------------------------------------------------------------

inline constexpr std::string_view kBatterySchema = "BatteryAPI_1";
inline constexpr std::string_view kBusSchema = "ElectricalBusAPI_1";
inline constexpr std::string_view kMotorSchema = "ElectricalMotorAPI_1";
inline constexpr std::string_view kBatteryPort = "PortAPI_1:dc_out";
inline constexpr std::string_view kBatteryPortPrefix = "elec:port:dc_out:";
inline constexpr std::string_view kLoadSchema = "ElectricalLoadAPI_1";
inline constexpr std::string_view kConverterSchema = "ElectricalConverterAPI_1";
inline constexpr std::string_view kBranchSchema = "ElectricalBranchAPI_1";

/// The schema instance named `schema` of an entry; nullptr when it has none.
[[nodiscard]] const OverlaySchema* findSchema(const OverlayEntry& entry, std::string_view schema);
[[nodiscard]] OverlaySchema* findSchema(OverlayEntry& entry, std::string_view schema);
/// An attribute's text ("" when not given; a list's items joined by ", ").
[[nodiscard]] std::string attributeText(const OverlaySchema& schema, std::string_view name);
/// Sets an attribute from text as typed: empty removes it; for an array type, comma-separated
/// items. Keeps the attribute's place; a new one goes where the form's order puts it.
void setAttribute(OverlaySchema& schema, const OverlayField* field, std::string_view name,
                  std::string_view text);

/// A battery's output port: its first PortAPI_1 instance ("PortAPI_1:dc_out", "PortAPI_1:out");
/// nullptr when it has none. portPrefix: "elec:port:dc_out:", its attributes' prefix.
[[nodiscard]] const OverlaySchema* portOf(const OverlayEntry& entry);
[[nodiscard]] OverlaySchema* portOf(OverlayEntry& entry);
[[nodiscard]] std::string portPrefix(const OverlaySchema& port);

/// The prim an entry makes or names: its `create` path, else `<prim>/<create_child>`, else
/// `prim`. The last part is the name the run gives its signals (bus/<name>/voltage).
[[nodiscard]] std::string entryPrim(const OverlayEntry& entry);
[[nodiscard]] std::string entryName(const OverlayEntry& entry);

/// Indices into document.overlay.entries of the entries with a schema (in order).
[[nodiscard]] std::vector<std::size_t> entriesWith(const Document& document,
                                                   std::string_view schema);
/// The motor entry driving `joint` (its elec:joint), or -1.
[[nodiscard]] int motorOf(const Document& document, std::string_view joint);
/// The bus entry a relationship's text names ("power/dc_48v" or "dc_48v"), or -1.
[[nodiscard]] int busNamed(const Document& document, std::string_view target);

/// Starting values from the samples, for "Add" menus: what a sample robot of the repository has
/// (the file named in `source`), never invented numbers. `values` are (attribute, text) pairs.
struct PowerPreset {
    std::string_view label;   ///< "LFP 16S 20 Ah"
    std::string_view source;  ///< "assets/robots/amr_diffdrive/amr_diffdrive.overlay.yaml"
    std::span<const std::pair<std::string_view, std::string_view>> values;
};
[[nodiscard]] std::span<const PowerPreset> batteryPresets();
[[nodiscard]] std::span<const PowerPreset> busPresets();
[[nodiscard]] std::span<const PowerPreset> motorPresets();
[[nodiscard]] std::span<const PowerPreset> loadPresets();

/// Adds a bus `power/<name>` (name made unique, "dc_bus" when empty) with a preset's values
/// (nullptr: none). Returns its entry's name.
std::string addBus(Document& document, std::string_view name, const PowerPreset* preset);
/// Adds a battery `power/<name>` ("battery") whose output port connects to bus entry `bus`
/// (-1: the first bus, if any), with a preset's values. Returns its name.
std::string addBattery(Document& document, std::string_view name, int bus,
                       const PowerPreset* preset);
/// Adds a motor on `joint` (`prim: <joint>`, `create_child: <joint>_motor`) drawing from bus
/// entry `bus` (-1: the first bus, if any), with a preset's values. Returns its name; empty
/// when the joint does not exist or already has a motor.
std::string addMotor(Document& document, std::string_view joint, int bus,
                     const PowerPreset* preset);
/// Adds a load `power/<name>` ("load") drawing from bus entry `bus` (-1: the first bus), with
/// a preset's values (nullptr: the schema's model, constant_power, and an empty power).
std::string addLoad(Document& document, std::string_view name, int bus, const PowerPreset* preset);
/// Adds a DC-DC converter `power/<name>` ("dcdc") from bus entry `input` to bus entry `output`
/// (-1: none yet; the form's dropdowns pick them). Its voltage and current limit start empty:
/// no sample robot has a converter to take them from.
std::string addConverter(Document& document, std::string_view name, int input, int output);
/// Adds a branch `power/<name>` (named after its kind: "fuse", "switch", "diode", "resistor")
/// of `kind` (resistor, fuse, switch, diode) between bus entries `input` and `output` (-1:
/// none yet). Its resistance (and a fuse's rating) start empty.
std::string addBranch(Document& document, std::string_view name, std::string_view kind, int input,
                      int output);
/// The signals the network writes for an entry with a power schema (electrical.md §3, the
/// network's names): bus/<n>/voltage, brownout; battery/<n>/soc ...; load/<n>/current, powered;
/// converter/<n>/output_current, input_current, loss_power; branch/<n>/current, voltage, state,
/// loss_power. (name, unit) pairs; empty for other entries. Inputs it reads when something
/// writes them (load/<n>/power_demand, a switch's branch/<n>/command) are not listed: the run's
/// catalog has them only then.
[[nodiscard]] std::vector<std::pair<std::string, std::string>> powerSignals(
    const OverlayEntry& entry);

/// Removes entry `index` and every relationship to its prim (a bus's motors and batteries keep
/// their entries, without the relationship: validation then asks for one).
bool removeEntry(Document& document, std::size_t index);
/// The overlay's references after an item was renamed or removed (Edits.cpp calls these):
/// entries naming `from` as their prim or in a relationship follow `to`; removed ones go.
void renameInOverlay(Document& document, std::string_view from, std::string_view to);
void removeFromOverlay(Document& document, std::span<const std::string> names);

}  // namespace lagrange::viewer::builder
