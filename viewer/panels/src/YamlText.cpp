// Profile YAML as the design's code view (YamlText.hpp).
#include "YamlText.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <cfloat>
#include <charconv>
#include <system_error>

namespace lagrange::viewer::yaml_text {

namespace {

namespace c = theme::c;
using widgets::px;

// Syntax colours of the design's code view (.code in reference.html).
const ImVec4 kKey = theme::hex(0x8FB8FF);
const ImVec4 kString = theme::hex(0x9BD39B);
const ImVec4 kNumber = theme::hex(0xF5C842);
const ImVec4 kBoolean = theme::hex(0xE8772E);
const ImVec4 kComment = theme::hex(0x6E7F98);
const ImVec4 kLineNumber = theme::hex(0x4A5B74);

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

ImVec4 withAlpha(ImVec4 color, float alpha) {
    color.w = alpha;
    return color;
}

bool isNumber(std::string_view word) {
    double value = 0.0;
    const auto [end, error] = std::from_chars(word.data(), word.data() + word.size(), value);
    return error == std::errc{} && end == word.data() + word.size();
}

}  // namespace

void drawYaml(ImDrawList* draw, ImVec2 at, std::string_view line) {
    ImFont* font = theme::fonts().mono;
    const float size = px(theme::kMonoSize);
    const auto emit = [&](std::string_view text, const ImVec4& color) {
        const char* begin = text.data();
        const char* end = begin + text.size();
        draw->AddText(font, size, at, u32(color), begin, end);
        at.x += font->CalcTextSizeA(size, FLT_MAX, 0.0F, begin, end).x;
    };
    const auto isBreak = [&](std::size_t i) {
        const char ch = line[i];
        if (ch == ':') {  // a colon ends a word only as "key: " (not in "tcp://host")
            return i + 1 == line.size() || line[i + 1] == ' ';
        }
        return ch == ' ' || ch == ',' || ch == '{' || ch == '}' || ch == '[' || ch == ']';
    };
    std::size_t i = 0;
    while (i < line.size()) {
        const char ch = line[i];
        if (ch == '#') {
            emit(line.substr(i), kComment);
            return;
        }
        if (ch == '"' || ch == '\'') {
            std::size_t end = line.find(ch, i + 1);
            end = end == std::string_view::npos ? line.size() : end + 1;
            emit(line.substr(i, end - i), kString);
            i = end;
            continue;
        }
        if (isBreak(i)) {
            std::size_t end = i + 1;
            while (end < line.size() && isBreak(end) && line[end] != '#') {
                ++end;
            }
            emit(line.substr(i, end - i), c::t2);
            i = end;
            continue;
        }
        std::size_t end = i;
        while (end < line.size() && !isBreak(end)) {
            ++end;
        }
        const std::string_view word = line.substr(i, end - i);
        const bool key = end < line.size() && line[end] == ':';
        if (key) {
            emit(word, kKey);
        } else if (word == "true" || word == "false" || word == "null") {
            emit(word, kBoolean);
        } else if (isNumber(word)) {
            emit(word, kNumber);
        } else {
            emit(word, kString);
        }
        i = end;
    }
}

void codeLine(std::string_view text, const char* gutter, Tint tint, float gutterWidth) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, px(kLineHeight));
    ImGui::Dummy(size);
    if (!ImGui::IsItemVisible()) {
        return;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (tint != Tint::None) {
        const ImVec4 color = tint == Tint::Removed ? c::error : c::ok;
        draw->AddRectFilled(min, ImVec2(min.x + size.x, min.y + size.y),
                            u32(withAlpha(color, 0.10F)));
    }
    ImFont* font = theme::fonts().mono;
    const float fontSize = px(theme::kMonoSize);
    const float y = min.y + (size.y - fontSize) * 0.5F;
    const float width = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0F, gutter).x;
    draw->AddText(font, fontSize, ImVec2(min.x + px(gutterWidth - 14.0F) - width, y),
                  u32(tint == Tint::Removed ? c::error
                      : tint == Tint::Added ? c::ok
                                            : kLineNumber),
                  gutter);
    drawYaml(draw, ImVec2(min.x + px(gutterWidth), y), text);
}

}  // namespace lagrange::viewer::yaml_text
