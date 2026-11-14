#pragma once

#include "lagrange/viewer/Icons.hpp"

#include <cstddef>
#include <string>
#include <string_view>

#include <imgui.h>

/// The shared widgets of the design system (docs/design/ui/00-design-system/SPEC.md).
/// Panels use these and lagrange::theme; they do not style ImGui themselves.
namespace lagrange::viewer::widgets {

/// A design length in logical pixels, scaled for the monitor.
[[nodiscard]] float px(float logical);

/// The identity colour of a physics domain ("rigid_body", ...). Grey for other modules.
[[nodiscard]] ImVec4 domainColor(std::string_view domain);
/// The name of a domain as the user reads it ("Rigid body"). Unknown names are returned as is.
[[nodiscard]] std::string_view domainLabel(std::string_view domain);
/// The short name used on the domain strip ("Aero").
[[nodiscard]] std::string_view domainShortLabel(std::string_view domain);

// --- text ---------------------------------------------------------------------------------
/// Numbers, units, paths and names: IBM Plex Mono.
void Mono(const ImVec4& color, const char* format, ...) IM_FMTARGS(2);
/// Small secondary text.
void Caption(const char* format, ...) IM_FMTARGS(1);
/// Emphasised text: row titles, section titles.
void Strong(const char* format, ...) IM_FMTARGS(1);
/// Puts the next item so that it ends at the right edge of the content region.
void AlignRight(float itemWidth);

// --- controls -----------------------------------------------------------------------------
enum class PillKind { Neutral, Ok, Warn, Error, Info, Accent };
void Pill(const char* text, PillKind kind);
[[nodiscard]] float PillWidth(const char* text);

/// A 28x16 on/off switch. Returns true when clicked.
bool Switch(const char* id, bool* value);

enum class ButtonKind { Default, Primary, Ghost, On };
/// A button of the design: 22 px in panels, 26 px (`large`) in toolbars; optional icon. With an
/// icon and a label that is only an id ("##reset") it is a square icon button.
bool Button(const char* label, ButtonKind kind = ButtonKind::Default, Icon icon = Icon::Count,
            bool large = false);
/// Width of a button with this label, to place it from the right edge.
[[nodiscard]] float ButtonWidth(const char* label, Icon icon = Icon::Count, bool large = false);
/// The one primary action of a region: accent fill.
bool PrimaryButton(const char* label);

/// A flat button that opens a menu: label and a small chevron (design: panel header menus).
bool MenuButton(const char* label);
[[nodiscard]] float MenuButtonWidth(const char* label);

/// Starts the tool slot at the right end of the current docked panel's tab bar (design:
/// `.ph .tools`), `width` wide. Returns false when the panel has no tab bar to draw in (it
/// floats, or its tab bar is hidden); otherwise place 22 px controls and call EndPanelTools().
bool BeginPanelTools(const char* id, float width);
void EndPanelTools();

/// A 24 px icon button of a floating tool bar over the viewport: no frame, accent when on.
bool ToolButton(const char* id, Icon icon, bool on, const char* tooltip);
/// Draws the frame of a floating tool bar that holds `count` ToolButtons, laid out in a row
/// or a column from the cursor. Call before the buttons.
void ToolBarFrame(int count, bool vertical);

/// A boxed read-only value in a toolbar: small key, mono value, optional small text after.
void Readout(const char* key, const char* value, const char* after = nullptr,
             const ImVec4& color = ImVec4(0.902F, 0.922F, 0.949F, 1.0F),
             const ImVec4& afterColor = ImVec4(0.506F, 0.565F, 0.651F, 1.0F));
[[nodiscard]] float ReadoutWidth(const char* key, const char* value, const char* after = nullptr);

/// A keyboard shortcut hint ("Ctrl K").
void Kbd(const char* text);

/// A search box with the search icon. Returns true when the text changed.
bool SearchField(const char* id, const char* hint, char* buffer, std::size_t size, float width);

enum class ChipState { On, Substitute, Off, Pending, Error };
/// One domain on the domain strip: colour dot, name, mono detail. Returns true when clicked.
/// A fully transparent `color` draws a neutral chip without a dot (the Builder's Robot strip).
bool DomainChip(const char* id, const ImVec4& color, const char* name, const char* detail,
                ChipState state);

/// A selectable card with a title and one line of description. Returns true when clicked.
bool OptionCard(const char* id, const char* title, const char* description, bool selected,
                const ImVec2& size);

/// A 26 px collapsible section header on the raised surface, with mono text on the right
/// and, when `dot` is given, a domain dot before the title.
bool SectionHeader(const char* title, const char* meta, bool defaultOpen = true,
                   const ImVec4* dot = nullptr);

/// A workspace tab: icon and label, accent underline when active. `available` is false for a
/// workspace that is not built yet; it is drawn dimmer.
bool BarTab(const char* label, Icon icon, bool active, bool available);

/// A row of exclusive choices (time windows, level filters). Returns true when it changed.
bool Segmented(const char* id, const char* const* options, int count, int* index);

/// One row of a key/value grid: label column (112 px), then a read-only value field with the
/// unit right-aligned inside it. `link` draws the value as a reference to a prim or file.
/// Returns true when a link value was clicked.
bool PropertyRow(const char* label, const char* value, const char* unit, bool link = false,
                 const char* tooltip = nullptr);

/// A PropertyRow whose value can be typed: the same 22 px field on bg0 with the unit inside on
/// the right; `edited` (an unapplied change) gives it the accent border (ui-conventions §2).
/// Returns true when the text changed. `numeric` takes only what a number can hold.
bool PropertyEdit(const char* label, char* buffer, std::size_t size, const char* unit, bool edited,
                  const char* tooltip = nullptr, bool numeric = true);

/// One of the two fields of a PropertyEditPair.
struct EditField {
    char* buffer;
    std::size_t size;
    const char* unit;
    bool edited{false};
    const char* tooltip{nullptr};
};

/// A PropertyEdit row with two number fields side by side (the design's two-column `.f3` grid,
/// e.g. the wind's mean speed and direction). Returns which were applied this frame (Enter or
/// leaving the field after typing): bit 0 the first, bit 1 the second; and which is being typed
/// in: bit 2 the first, bit 3 the second.
int PropertyEditPair(const char* label, EditField first, EditField second);

/// Three read-only number fields with coloured X / Y / Z labels.
void Vec3Field(const char* label, const double value[3], const char* format = "%.3f");

/// A small line plot of one signal: one min/max pair per pixel column. NaN columns are gaps.
void Sparkline(const float* low, const float* high, int columns, const ImVec2& size,
               const ImVec4& color);

/// A chip naming an applied schema, in its domain's colour.
void SchemaChip(const char* schema);

/// The physics domain a USD schema belongs to ("ElectricalMotorAPI_1" -> "electrical"), also
/// for attribute namespaces ("elec"). Empty when it is not a physics schema.
[[nodiscard]] std::string_view schemaDomain(std::string_view schemaOrNamespace);

/// A check box: accent fill with a dark tick when on. Returns true when clicked.
bool Checkbox(const char* id, bool* value);

/// A small toggle for a filter ("electrical", "rate >= 100 Hz"), in `color` when on.
bool FilterChip(const char* label, const ImVec4& color, bool selected);
/// The same chip opening a list ("unit: any ▾" in the Signals design), its chevron the designed
/// ChevronDown icon. Returns true when clicked.
bool DropdownChip(const char* label, const ImVec4& color, bool selected);

/// A 6 px bar filled to `fraction` (0..1).
void Meter(float fraction, const ImVec4& color, float width);

/// A small square in a series colour, centred on the line.
void Swatch(const ImVec4& color);

/// A filled dot, vertically centred on the current line. Advances the cursor.
void Dot(const ImVec4& color, bool filled = true);

// --- robot builder (12-builder: "New in this screen") -----------------------------------------
// Fields that apply when committed: there is no Apply in the Builder (SPEC "Properties"), so a
// field changes the document when the user presses Enter or Tab or leaves it, never per key.

/// A typed field, `width` wide at the cursor (the 22 px `.fld` on bg0, the unit inside on the
/// right; `edited` gives it the accent border). `*text` is shown while the field is not being
/// typed in; returns true when the user committed a change, with the new text in `*text`.
bool CommitField(const char* id, std::string* text, const char* unit, bool edited, float width,
                 bool numeric = true);
/// CommitField in a PropertyRow's grid (112 px label column). `monoLabel`: the label is a
/// parameter path in 11 px mono (design: `.k.mono`).
bool CommitEdit(const char* label, std::string* text, const char* unit, bool edited,
                const char* tooltip = nullptr, bool numeric = true, bool monoLabel = false);
/// Three typed number fields with the X / Y / Z labels of Vec3Field (which is read-only).
/// `value` is updated on commit; returns a mask of the axes committed (bit 0 x, 1 y, 2 z).
int Vec3Edit(const char* label, double value[3], const char* format = "%.3f");

/// A field that opens a list (design `.fld.dd`): the value in mono, an optional unit, and the
/// small down triangle. Returns true when clicked; the caller opens its popup.
bool DropdownField(const char* id, const char* value, const char* unit, float width);
/// DropdownField in the 112 px label grid.
bool DropdownRow(const char* label, const char* value, const char* unit = nullptr);

/// The joint preview slider (12-builder `JointSlider`): a 4 px bg4 track from `lower` to
/// `upper`, filled #3A4F6E from zero to the knob, a 1 × 12 px tick at zero and a 14 px knob.
/// `wrap` draws the dashed track of a continuous joint (the value wraps). Without `knob` (a
/// mimic joint) it only shows. Returns true while the value changes.
bool JointSlider(const char* id, double* value, double lower, double upper, bool wrap, bool knob,
                 float width);

/// A 56 × 54 tile of the Add panel (12-builder `ShapeTile`): a 20 px icon over an 11 px label.
bool ShapeTile(const char* id, Icon icon, const char* label);

/// One 22 px row of a tree (00-design-system `TreeRow`): caret, icon, name, a type in t3, and
/// badges on the right; selected rows have the weak accent and a 2 px accent bar.
struct TreeRowSpec {
    int depth{0};
    bool hasChildren{false};
    bool folded{false};
    bool selected{false};
    Icon icon{Icon::Count};
    const char* name{""};
    bool strong{false};  ///< medium weight (the root row, the selection)
    const char* type{""};
    const ImVec4* badges{nullptr};  ///< domain dots on the right
    int badgeCount{0};
    bool error{false};  ///< a red dot badge (an item with an error)
};
enum class TreeHit { None, Row, Caret, Type };
/// Returns where the row was clicked: on the caret, on the type text, or elsewhere on it. The
/// row is the last item, so drag and drop and tooltips can follow.
TreeHit TreeRow(const char* id, const TreeRowSpec& row);

}  // namespace lagrange::viewer::widgets
