#pragma once

#include <string>

#include <imgui.h>

namespace lagrange::theme {

/// Type scale from docs/design/ui/shared/design-tokens.json, in logical pixels.
inline constexpr float kTitleSize = 14.0F;
inline constexpr float kBodySize = 13.0F;
inline constexpr float kLabelSize = 12.0F;
inline constexpr float kCaptionSize = 11.0F;
inline constexpr float kMonoSize = 12.0F;
inline constexpr float kKpiSize = 20.0F;

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
