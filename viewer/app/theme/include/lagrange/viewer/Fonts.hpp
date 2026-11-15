#pragma once

#include <string>

#include <imgui.h>

namespace lagrange::theme {

/// Type scale from docs/design/ui/shared/design-tokens.json, in logical pixels. A size is a CSS
/// font size: the em of the glyphs, as reference.html draws them at 1.0 scale. It is also the
/// font size Dear ImGui lays out with (GetFontSize(), the text line height), so the style's
/// paddings around it give the design's control heights (13 + 2 x 4.5 = 22 px).
inline constexpr float kTitleSize = 14.0F;
inline constexpr float kBodySize = 13.0F;
inline constexpr float kLabelSize = 12.0F;
inline constexpr float kCaptionSize = 11.0F;
inline constexpr float kMonoSize = 12.0F;
inline constexpr float kKpiSize = 20.0F;

/// IBM Plex's line height in ems: (hhea ascender 1025 + descender 275) / 1000 units per em, the
/// same in all five faces in viewer/assets/fonts. Dear ImGui scales a font so that this line,
/// not the em, is the font size, which drew a 13 px token like 10 px CSS text; loadFonts()
/// scales each glyph by this factor so that the em is the font size, as in CSS.
inline constexpr float kPlexLineHeightEm = 1.3F;

/// The faces of the design: IBM Plex Sans for text, IBM Plex Mono for every number, unit,
/// path and name. Sizes are chosen where a font is pushed (Dear ImGui sizes fonts on demand).
struct Fonts {
    ImFont* sans{nullptr};
    ImFont* sansMedium{nullptr};
    ImFont* sansSemiBold{nullptr};
    ImFont* mono{nullptr};
    ImFont* monoMedium{nullptr};
    bool bundled{false};  ///< False when the font files were not found and a fallback is used.
};

/// Loads the fonts from `directory` into the current ImGui context and makes Sans the default.
/// Call once after ImGui::CreateContext(). If a file is missing, every face falls back to
/// ImGui's built-in font, so the viewer still starts.
const Fonts& loadFonts(const std::string& directory);

/// The fonts loaded by loadFonts().
[[nodiscard]] const Fonts& fonts();

}  // namespace lagrange::theme
