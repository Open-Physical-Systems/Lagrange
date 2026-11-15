#pragma once

// Profile YAML drawn as the design's code view (.code in docs/design/ui/03-domains/
// reference.html): a mono line with a line-number gutter, syntax colours and a diff tint.
// Shared by the Profile and Diff tabs of the Domains workspace.

#include <string_view>

#include <imgui.h>

namespace lagrange::viewer::yaml_text {

constexpr float kLineHeight = 19.0F;  // design: mono 12 on a 19 px line (logical)
constexpr float kGutter = 44.0F;      // logical

enum class Tint { None, Removed, Added };

/// Draws one line of profile text at `at` with the design's syntax colours: keys, strings,
/// numbers, booleans, comments. A lexer for the canonical layout, not a YAML parser.
void drawYaml(ImDrawList* draw, ImVec2 at, std::string_view line);

/// One code line as an item of the current window: tint, gutter text right-aligned before
/// `gutterWidth` (logical px), then the YAML. `gutterError` draws the gutter in the error colour.
void codeLine(std::string_view text, const char* gutter, Tint tint, float gutterWidth = kGutter);

}  // namespace lagrange::viewer::yaml_text
