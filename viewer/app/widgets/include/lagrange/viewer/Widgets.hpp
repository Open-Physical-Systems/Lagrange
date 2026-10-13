#pragma once

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

/// The one primary action of a region: accent fill.
bool PrimaryButton(const char* label);

enum class ChipState { On, Substitute, Off, Pending, Error };
/// One domain on the domain strip: colour dot, name, mono detail. Returns true when clicked.
bool DomainChip(const char* id, const ImVec4& color, const char* name, const char* detail,
                ChipState state);

/// A selectable card with a title and one line of description. Returns true when clicked.
bool OptionCard(const char* id, const char* title, const char* description, bool selected,
                const ImVec2& size);

/// A 26 px collapsible section header on the raised surface, with mono text on the right.
bool SectionHeader(const char* title, const char* meta, bool defaultOpen = true);

/// A top-level tab of a bar (workspaces): accent underline when active.
bool BarTab(const char* label, bool active);

/// A row of exclusive choices (time windows, level filters). Returns true when it changed.
bool Segmented(const char* id, const char* const* options, int count, int* index);

/// One row of a key/value grid: label column (112 px), then a read-only value field with the
/// unit right-aligned inside it. `link` draws the value as a reference to a prim or file.
/// Returns true when a link value was clicked.
bool PropertyRow(const char* label, const char* value, const char* unit, bool link = false,
                 const char* tooltip = nullptr);

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

/// A 6 px bar filled to `fraction` (0..1).
void Meter(float fraction, const ImVec4& color, float width);

/// A small square in a series colour, centred on the line.
void Swatch(const ImVec4& color);

/// A filled dot, vertically centred on the current line. Advances the cursor.
void Dot(const ImVec4& color, bool filled = true);

}  // namespace lagrange::viewer::widgets
