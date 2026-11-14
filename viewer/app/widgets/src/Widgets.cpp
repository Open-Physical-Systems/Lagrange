#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Icons.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <imgui_internal.h>  // ItemSize/ItemAdd for custom-drawn widgets

namespace lagrange::viewer::widgets {

namespace {

namespace c = theme::c;

struct DomainName {
    std::string_view key;
    std::string_view label;
    std::string_view shortLabel;
    theme::Domain domain;
};

constexpr std::array<DomainName, 8> kDomains{{
    {"rigid_body", "Rigid body", "Rigid body", theme::Domain::RigidBody},
    {"electrical", "Electrical", "Electrical", theme::Domain::Electrical},
    {"thermal", "Thermal", "Thermal", theme::Domain::Thermal},
    {"aerodynamics", "Aerodynamics", "Aero", theme::Domain::Aerodynamics},
    {"weather", "Weather", "Weather", theme::Domain::Weather},
    {"emi", "EMI", "EMI", theme::Domain::Emi},
    {"radiation", "Radiation", "Radiation", theme::Domain::Radiation},
    {"sensors", "Sensors", "Sensors", theme::Domain::Sensors},
}};

const DomainName* findDomain(std::string_view domain) {
    for (const DomainName& entry : kDomains) {
        if (entry.key == domain) {
            return &entry;
        }
    }
    return nullptr;
}

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

ImVec4 withAlpha(ImVec4 color, float alpha) {
    color.w = alpha;
    return color;
}

/// A rounded rectangle outline drawn as dashes: the outline is laid out as a path, then
/// walked with a dash pattern.
void dashedRect(ImDrawList* draw, const ImVec2& min, const ImVec2& max, ImU32 color,
                float rounding) {
    const float dash = px(3.0F);
    const float gap = px(2.5F);
    draw->PathRect(min, max, rounding);
    ImVector<ImVec2>& path = draw->_Path;
    const int count = path.Size;
    float carried = 0.0F;  // distance into the current dash + gap period
    for (int i = 0; i < count; ++i) {
        const ImVec2 from = path[i];
        const ImVec2 to = path[(i + 1) % count];
        const float length = std::hypot(to.x - from.x, to.y - from.y);
        if (length <= 0.0F) {
            continue;
        }
        const ImVec2 unit((to.x - from.x) / length, (to.y - from.y) / length);
        float at = 0.0F;
        while (at < length) {
            const float phase = std::fmod(carried + at, dash + gap);
            if (phase < dash) {
                const float end = std::min(at + dash - phase, length);
                draw->AddLine(ImVec2(from.x + unit.x * at, from.y + unit.y * at),
                              ImVec2(from.x + unit.x * end, from.y + unit.y * end), color);
                at = end;
            } else {
                at += dash + gap - phase;
            }
        }
        carried = std::fmod(carried + length, dash + gap);
    }
    path.Size = 0;  // the path was only used to lay out the outline
}

void textV(ImFont* font, float size, const ImVec4& color, const char* format, va_list args) {
    ImGui::PushFont(font, size);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextV(format, args);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

}  // namespace

float px(float logical) {
    return logical * ImGui::GetStyle().FontScaleDpi;
}

ImVec4 domainColor(std::string_view domain) {
    const DomainName* entry = findDomain(domain);
    return entry != nullptr ? theme::kDomainColor[static_cast<int>(entry->domain)] : c::t3;
}

std::string_view domainLabel(std::string_view domain) {
    const DomainName* entry = findDomain(domain);
    return entry != nullptr ? entry->label : domain;
}

std::string_view domainShortLabel(std::string_view domain) {
    const DomainName* entry = findDomain(domain);
    return entry != nullptr ? entry->shortLabel : domain;
}

void Mono(const ImVec4& color, const char* format, ...) {
    va_list args;
    va_start(args, format);
    textV(theme::fonts().mono, theme::kMonoSize, color, format, args);
    va_end(args);
}

void Caption(const char* format, ...) {
    va_list args;
    va_start(args, format);
    textV(theme::fonts().sans, theme::kCaptionSize, c::t3, format, args);
    va_end(args);
}

void Strong(const char* format, ...) {
    va_list args;
    va_start(args, format);
    textV(theme::fonts().sansSemiBold, theme::kBodySize, c::t1, format, args);
    va_end(args);
}

void AlignRight(float itemWidth) {
    const float x = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - itemWidth;
    ImGui::SetCursorPosX(std::max(x, ImGui::GetCursorPosX()));
}

float PillWidth(const char* text) {
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const float width = ImGui::CalcTextSize(text).x + px(14.0F);
    ImGui::PopFont();
    return width;
}

void Pill(const char* text, PillKind kind) {
    // Design (.pill): 18 px high, 9 px radius, 11 px medium, tinted fill, no border.
    ImVec4 color = c::t2;
    ImVec4 fill = c::bg3;
    switch (kind) {
        case PillKind::Ok:
            color = c::ok;
            fill = withAlpha(c::ok, 0.14F);
            break;
        case PillKind::Warn:
            color = c::warn;
            fill = withAlpha(c::warn, 0.14F);
            break;
        case PillKind::Error:
            color = c::error;
            fill = withAlpha(c::error, 0.15F);
            break;
        case PillKind::Info:
            color = c::info;
            fill = withAlpha(c::info, 0.14F);
            break;
        case PillKind::Accent:
            color = c::accent;
            fill = c::accentWeak;
            break;
        case PillKind::Neutral:
            break;
    }
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const ImVec2 textSize = ImGui::CalcTextSize(text);
    const ImVec2 size(textSize.x + px(14.0F), px(18.0F));
    // Centre the pill on the current text line.
    const float lineHeight = ImGui::GetTextLineHeight();
    ImVec2 min = ImGui::GetCursorScreenPos();
    min.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + (lineHeight - size.y) * 0.5F;
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(fill), size.y * 0.5F);
    draw->AddText(ImVec2(min.x + px(7.0F), min.y + (size.y - textSize.y) * 0.5F), u32(color), text);
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(size.x, lineHeight));
}

bool Switch(const char* id, bool* value) {
    // Design (.sw): 28 x 16 track on bg4 with a 1 px border, 10 px knob 2 px from the edge.
    const ImVec2 size(px(28.0F), px(16.0F));
    const float height = ImGui::GetFrameHeight();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(size.x, height));
    if (clicked) {
        *value = !*value;
    }
    const ImVec2 min(origin.x, origin.y + (height - size.y) * 0.5F);
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float radius = size.y * 0.5F;
    draw->AddRectFilled(min, max, u32(*value ? c::accent : c::bg4), radius);
    const bool hot = ImGui::IsItemHovered() || ImGui::IsItemFocused();
    draw->AddRect(min, max, u32(*value ? c::accent : (hot ? c::t3 : c::lineStrong)), radius);
    const float knob = px(5.0F);
    const float x = *value ? max.x - px(3.0F) - knob : min.x + px(3.0F) + knob;
    draw->AddCircleFilled(ImVec2(x, min.y + radius), knob, u32(*value ? c::onAccent : c::t3));
    return clicked;
}

bool Button(const char* label, ButtonKind kind, Icon icon, bool large) {
    // Design (.btn): 26 px in toolbars, 22 px (.sm) in panels; bg3 with a 1 px lineStrong
    // border; 12 / 11 px medium text; an optional 14 / 12 px icon before the text.
    const theme::Fonts& fonts = theme::fonts();
    const float height = px(large ? 26.0F : 22.0F);
    const float fontSize = large ? theme::kLabelSize : theme::kCaptionSize;
    const float iconSize = px(large ? 14.0F : 12.0F);
    const float pad = px(large ? 10.0F : 8.0F);
    ImGui::PushFont(fonts.sansMedium, fontSize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const bool hasIcon = icon != Icon::Count;
    const bool hasText = textSize.x > 0.0F;
    float width = pad * 2.0F + textSize.x;
    if (hasIcon) {
        width = hasText ? width + iconSize + px(6.0F) : height;  // icon only: a square
    }
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    const bool clicked = ImGui::InvisibleButton(label, ImVec2(width, height));
    const bool hovered = ImGui::IsItemHovered();
    const bool held = ImGui::IsItemActive();
    const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;

    ImVec4 fill = hovered ? c::bg4 : c::bg3;
    ImVec4 border = c::lineStrong;
    ImVec4 ink = c::t1;
    switch (kind) {
        case ButtonKind::Primary:
            fill = held ? theme::hex(0xD0661F) : (hovered ? theme::hex(0xF08A45) : c::accent);
            border = fill;
            ink = c::onAccent;
            break;
        case ButtonKind::Ghost:
            fill = hovered ? c::bg3 : ImVec4(0, 0, 0, 0);
            border = ImVec4(0, 0, 0, 0);
            ink = hovered ? c::t1 : c::t2;
            break;
        case ButtonKind::On:
            fill = c::accentWeak;
            border = c::accentBorder;
            ink = c::accent;
            break;
        case ButtonKind::Default:
            if (held) {
                fill = c::lineStrong;
            }
            break;
    }
    if (disabled) {
        const float alpha = ImGui::GetStyle().DisabledAlpha;
        fill.w *= alpha;
        border.w *= alpha;
        ink.w *= alpha;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = px(3.0F);
    draw->AddRectFilled(min, max, u32(fill), rounding);
    draw->AddRect(min, max, u32(border), rounding);
    float x = min.x + (hasIcon && !hasText ? (width - iconSize) * 0.5F : pad);
    if (hasIcon) {
        DrawIcon(draw, icon, ImVec2(x, min.y + (height - iconSize) * 0.5F), iconSize, u32(ink),
                 u32(fill));
        x += iconSize + px(6.0F);
    }
    if (hasText) {
        draw->AddText(ImVec2(x, min.y + (height - textSize.y) * 0.5F), u32(ink), label,
                      ImGui::FindRenderedTextEnd(label));
    }
    ImGui::PopFont();
    return clicked;
}

float MenuButtonWidth(const char* label) {
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const float text = ImGui::CalcTextSize(label, nullptr, true).x;
    ImGui::PopFont();
    return px(8.0F) + text + px(6.0F) + px(10.0F) + px(6.0F);
}

bool MenuButton(const char* label) {
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const float chevron = px(10.0F);
    const ImVec2 size(px(8.0F) + textSize.x + px(6.0F) + chevron + px(6.0F), px(22.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(label, size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (hovered) {
        draw->AddRectFilled(min, max, u32(c::bg3), px(3.0F));
    }
    const ImVec4 ink = hovered ? c::t1 : c::t2;
    draw->AddText(ImVec2(min.x + px(8.0F), min.y + (size.y - textSize.y) * 0.5F), u32(ink), label,
                  ImGui::FindRenderedTextEnd(label));
    DrawIcon(draw, Icon::ChevronDown,
             ImVec2(max.x - px(6.0F) - chevron, min.y + (size.y - chevron) * 0.5F), chevron,
             u32(c::t3), u32(c::bg2));
    ImGui::PopFont();
    return clicked;
}

bool BeginPanelTools(const char* id, float width) {
    const ImGuiWindow* window = ImGui::GetCurrentWindow();
    const ImGuiDockNode* node = window->DockNode;
    if (node == nullptr || node->TabBar == nullptr || node->IsHiddenTabBar() ||
        !window->DockTabIsVisible) {
        return false;
    }
    // The tab bar belongs to the dock node, not to the panel, so the tools are a small
    // window of their own laid over its right end.
    const ImRect bar = node->TabBar->BarRect;
    const float height = px(22.0F);
    ImGui::SetNextWindowPos(ImVec2(bar.Max.x - width - px(4.0F), bar.Min.y));
    ImGui::SetNextWindowSize(ImVec2(width, bar.GetHeight()));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                        ImVec2(0.0F, std::max((bar.GetHeight() - height) * 0.5F, 0.0F)));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(1.0F, 1.0F));
    ImGui::Begin(id, nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar(3);
    return true;
}

void EndPanelTools() {
    ImGui::End();
}

void ToolBarFrame(int count, bool vertical) {
    // Design (.ovbar): 2 px padding around 24 px buttons 2 px apart, on rgba(9, 17, 32, .88)
    // with a lineStrong border and 4 px radius.
    const float cell = px(24.0F);
    const float gap = px(2.0F);
    const float along = cell * static_cast<float>(count) + gap * static_cast<float>(count + 1);
    const float across = cell + gap * 2.0F;
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + (vertical ? across : along), min.y + (vertical ? along : across));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(theme::hex(0x091120, 0.88F)), px(4.0F));
    draw->AddRect(min, max, u32(c::lineStrong), px(4.0F));
    ImGui::SetCursorScreenPos(ImVec2(min.x + gap, min.y + gap));
}

bool ToolButton(const char* id, Icon icon, bool on, const char* tooltip) {
    const float cell = px(24.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + cell, min.y + cell);
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(cell, cell));
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (on) {
        draw->AddRectFilled(min, max, u32(c::accentWeak), px(3.0F));
    } else if (hovered && !disabled) {
        draw->AddRectFilled(min, max, u32(c::bg3), px(3.0F));
    }
    ImVec4 ink = on ? c::accent : c::t2;
    if (disabled) {
        ink = withAlpha(c::t3, 0.55F);
    }
    const float size = px(14.0F);
    DrawIcon(draw, icon, ImVec2(min.x + (cell - size) * 0.5F, min.y + (cell - size) * 0.5F), size,
             u32(ink), u32(theme::hex(0x091120)));
    if (hovered && tooltip != nullptr) {
        ImGui::SetTooltip("%s", tooltip);
    }
    return clicked;
}

float ButtonWidth(const char* label, Icon icon, bool large) {
    ImGui::PushFont(theme::fonts().sansMedium, large ? theme::kLabelSize : theme::kCaptionSize);
    const float text = ImGui::CalcTextSize(label, nullptr, true).x;
    ImGui::PopFont();
    const float height = px(large ? 26.0F : 22.0F);
    if (icon != Icon::Count && text <= 0.0F) {
        return height;
    }
    return px(large ? 20.0F : 16.0F) + text +
           (icon != Icon::Count ? px(large ? 20.0F : 18.0F) : 0.0F);
}

bool PrimaryButton(const char* label) {
    return Button(label, ButtonKind::Primary);
}

void Readout(const char* key, const char* value, const char* after, const ImVec4& color,
             const ImVec4& afterColor) {
    // Design (.ro): 26 px box on bg0 with a 1 px line border; 11 px key, 12 px mono value.
    const theme::Fonts& fonts = theme::fonts();
    const float keySize = px(theme::kCaptionSize);
    const float valueSize = px(theme::kMonoSize);
    const ImVec2 k = fonts.sans->CalcTextSizeA(keySize, FLT_MAX, 0.0F, key);
    const ImVec2 v = fonts.mono->CalcTextSizeA(valueSize, FLT_MAX, 0.0F, value);
    const bool hasAfter = after != nullptr && *after != '\0';
    const ImVec2 a =
        hasAfter ? fonts.sans->CalcTextSizeA(keySize, FLT_MAX, 0.0F, after) : ImVec2(0.0F, 0.0F);
    const float gap = px(6.0F);
    const ImVec2 size(px(16.0F) + k.x + gap + v.x + (hasAfter ? gap + a.x : 0.0F), px(26.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImGui::Dummy(size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(c::bg0), px(3.0F));
    draw->AddRect(min, max, u32(c::line), px(3.0F));
    float x = min.x + px(8.0F);
    const float mid = min.y + size.y * 0.5F;
    draw->AddText(fonts.sans, keySize, ImVec2(x, mid - k.y * 0.5F), u32(c::t3), key);
    x += k.x + gap;
    draw->AddText(fonts.mono, valueSize, ImVec2(x, mid - v.y * 0.5F), u32(color), value);
    x += v.x + gap;
    if (hasAfter) {
        draw->AddText(fonts.sans, keySize, ImVec2(x, mid - a.y * 0.5F), u32(afterColor), after);
    }
}

float ReadoutWidth(const char* key, const char* value, const char* after) {
    const theme::Fonts& fonts = theme::fonts();
    const float keySize = px(theme::kCaptionSize);
    float width = px(16.0F) + fonts.sans->CalcTextSizeA(keySize, FLT_MAX, 0.0F, key).x + px(6.0F) +
                  fonts.mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, value).x;
    if (after != nullptr && *after != '\0') {
        width += px(6.0F) + fonts.sans->CalcTextSizeA(keySize, FLT_MAX, 0.0F, after).x;
    }
    return width;
}

void Kbd(const char* text) {
    // Design (.kbd): 10 px mono in a 1 px lineStrong box.
    const theme::Fonts& fonts = theme::fonts();
    const float size = px(10.0F);
    const ImVec2 extent = fonts.mono->CalcTextSizeA(size, FLT_MAX, 0.0F, text);
    const ImVec2 box(extent.x + px(10.0F), px(16.0F));
    const float line = ImGui::GetTextLineHeight();
    ImVec2 min = ImGui::GetCursorScreenPos();
    min.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + (line - box.y) * 0.5F;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRect(min, ImVec2(min.x + box.x, min.y + box.y), u32(c::lineStrong), px(2.0F));
    draw->AddText(fonts.mono, size, ImVec2(min.x + px(5.0F), min.y + (box.y - extent.y) * 0.5F),
                  u32(c::t3), text);
    ImGui::Dummy(ImVec2(box.x, line));
}

bool SearchField(const char* id, const char* hint, char* buffer, std::size_t size, float width) {
    // Design (.search): 24 px, bg0, 1 px line border, a 12 px search icon, 12 px text.
    const float height = px(24.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + height);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(c::bg0), px(3.0F));
    draw->AddRect(min, max, u32(c::line), px(3.0F));
    const float icon = px(12.0F);
    DrawIcon(draw, Icon::Search, ImVec2(min.x + px(8.0F), min.y + (height - icon) * 0.5F), icon,
             u32(c::t3), u32(c::bg0));
    const float inset = px(8.0F) + icon + px(2.0F);
    ImGui::PushFont(theme::fonts().sans, theme::kLabelSize);
    const float pad = (height - ImGui::GetFontSize()) * 0.5F;
    ImGui::SetCursorScreenPos(ImVec2(min.x + inset, min.y));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(4.0F), pad));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0, 0, 0, 0));
    ImGui::SetNextItemWidth(width - inset - px(2.0F));
    const bool changed = ImGui::InputTextWithHint(id, hint, buffer, size);
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar(2);
    ImGui::PopFont();
    return changed;
}

void Dot(const ImVec4& color, bool filled) {
    const float radius = px(4.0F);
    const float height = ImGui::GetTextLineHeight();
    ImVec2 centre = ImGui::GetCursorScreenPos();
    centre.x += radius;
    centre.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + height * 0.5F;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (filled) {
        draw->AddCircleFilled(centre, radius, u32(color));
    } else {
        draw->AddCircle(centre, radius - px(0.75F), u32(color), 0, px(1.5F));
    }
    ImGui::Dummy(ImVec2(radius * 2.0F, height));
}

bool DomainChip(const char* id, const ImVec4& color, const char* name, const char* detail,
                ChipState state) {
    // Design (.chip): 22 px, 11 px radius, bg2 with a lineStrong border, 8 px dot, 12 px name,
    // 11 px mono detail in t3. Substitute and off: dashed border, no fill, hollow dot.
    const theme::Fonts& fonts = theme::fonts();
    const float nameSize = px(theme::kLabelSize);
    const float detailSize = px(theme::kCaptionSize);
    const ImVec2 nameExtent = fonts.sans->CalcTextSizeA(nameSize, FLT_MAX, 0.0F, name);
    const ImVec2 detailExtent = fonts.mono->CalcTextSizeA(detailSize, FLT_MAX, 0.0F, detail);
    const bool hasDot = color.w > 0.0F;  // a neutral chip has none
    const float dot = hasDot ? px(8.0F) : 0.0F;
    const float gap = px(6.0F);
    const ImVec2 size(px(8.0F) + dot + (hasDot ? gap : 0.0F) + nameExtent.x +
                          (detailExtent.x > 0.0F ? gap + detailExtent.x : 0.0F) + px(9.0F),
                      px(22.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();

    const bool dashed =
        state == ChipState::Substitute || state == ChipState::Off || state == ChipState::Pending;
    ImVec4 border = c::lineStrong;
    ImVec4 text = c::t1;
    ImVec4 detailColor = c::t3;
    switch (state) {
        case ChipState::Substitute:
            text = c::t2;
            break;
        case ChipState::Off:
            text = c::t3;
            break;
        case ChipState::Pending:
            border = c::accent;
            text = c::accent;
            detailColor = c::accent;
            break;
        case ChipState::Error:
            border = c::error;
            text = c::error;
            break;
        case ChipState::On:
            break;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float radius = size.y * 0.5F;
    if (!dashed) {
        draw->AddRectFilled(min, max, u32(hovered ? c::bg3 : c::bg2), radius);
        draw->AddRect(min, max, u32(border), radius);
    } else {
        if (hovered) {
            draw->AddRectFilled(min, max, u32(c::bg2), radius);
        }
        dashedRect(draw, min, max, u32(border), radius);
    }
    const float midY = min.y + size.y * 0.5F;
    const ImVec2 centre(min.x + px(8.0F) + dot * 0.5F, midY);
    if (!hasDot) {
        // Neutral: no dot.
    } else if (dashed) {
        draw->AddCircle(centre, dot * 0.5F - px(0.75F),
                        u32(state == ChipState::Pending ? c::accent : c::t3), 0, px(1.5F));
    } else {
        draw->AddCircleFilled(centre, dot * 0.5F, u32(color));
    }
    float x = min.x + px(8.0F) + dot + (hasDot ? gap : 0.0F);
    draw->AddText(fonts.sans, nameSize, ImVec2(x, midY - nameExtent.y * 0.5F), u32(text), name);
    x += nameExtent.x + gap;
    draw->AddText(fonts.mono, detailSize, ImVec2(x, midY - detailExtent.y * 0.5F), u32(detailColor),
                  detail);
    return clicked;
}

bool OptionCard(const char* id, const char* title, const char* description, bool selected,
                const ImVec2& size) {
    const theme::Fonts& fonts = theme::fonts();
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = ImGui::GetStyle().FrameRounding;
    draw->AddRectFilled(min, max, u32(selected ? c::accentWeak : (hovered ? c::bg3 : c::bg0)),
                        rounding);
    draw->AddRect(min, max, u32(selected ? c::accent : c::lineStrong), rounding);
    const float pad = px(10.0F);
    draw->PushClipRect(min, ImVec2(max.x - px(4.0F), max.y), true);
    draw->AddText(fonts.sansMedium, px(theme::kBodySize), ImVec2(min.x + pad, min.y + px(9.0F)),
                  u32(c::t1), title);
    draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(min.x + pad, min.y + px(31.0F)),
                  u32(c::t3), description);
    draw->PopClipRect();
    return clicked;
}

bool SectionHeader(const char* title, const char* meta, bool defaultOpen, const ImVec4* dot) {
    ImGuiStorage* storage = ImGui::GetStateStorage();
    const ImGuiID id = ImGui::GetID(title);
    bool open = storage->GetBool(id, defaultOpen);

    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 size(ImGui::GetContentRegionAvail().x, px(26.0F));
    const ImVec2 max(min.x + size.x, min.y + size.y);
    if (ImGui::InvisibleButton(title, size)) {
        open = !open;
        storage->SetBool(id, open);
    }
    const theme::Fonts& fonts = theme::fonts();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(ImGui::IsItemHovered() ? c::bg3 : c::bg2));
    draw->AddLine(ImVec2(min.x, max.y), max, u32(c::line));
    const float midY = min.y + size.y * 0.5F;
    const float caret = px(4.0F);
    const ImVec2 at(min.x + px(12.0F), midY);
    if (open) {
        draw->AddTriangleFilled(ImVec2(at.x - caret, at.y - caret * 0.6F),
                                ImVec2(at.x + caret, at.y - caret * 0.6F),
                                ImVec2(at.x, at.y + caret * 0.8F), u32(c::t2));
    } else {
        draw->AddTriangleFilled(ImVec2(at.x - caret * 0.6F, at.y - caret),
                                ImVec2(at.x - caret * 0.6F, at.y + caret),
                                ImVec2(at.x + caret * 0.8F, at.y), u32(c::t2));
    }
    const float titleSize = px(theme::kLabelSize);
    float titleX = min.x + px(24.0F);
    if (dot != nullptr) {
        draw->AddCircleFilled(ImVec2(titleX + px(4.0F), midY), px(4.0F), u32(*dot));
        titleX += px(14.0F);
    }
    draw->AddText(fonts.sansSemiBold, titleSize, ImVec2(titleX, midY - titleSize * 0.6F),
                  u32(c::t1), title);
    if (meta != nullptr && *meta != '\0') {
        const float metaSize = px(theme::kCaptionSize);
        const ImVec2 extent = fonts.mono->CalcTextSizeA(metaSize, FLT_MAX, 0.0F, meta);
        draw->AddText(fonts.mono, metaSize,
                      ImVec2(max.x - extent.x - px(10.0F), midY - extent.y * 0.5F), u32(c::t3),
                      meta);
    }
    return open;
}

std::string_view schemaDomain(std::string_view name) {
    struct Prefix {
        std::string_view prefix;
        std::string_view domain;
    };
    // USD schema names and the attribute namespaces they use.
    static constexpr std::array<Prefix, 22> kPrefixes{{
        {"Physics", "rigid_body"},
        {"physics", "rigid_body"},
        {"Electrical", "electrical"},
        {"Battery", "electrical"},
        {"Port", "electrical"},
        {"elec", "electrical"},
        {"Thermal", "thermal"},
        {"thermal", "thermal"},
        {"Aero", "aerodynamics"},
        {"aero", "aerodynamics"},
        {"Weather", "weather"},
        {"weather", "weather"},
        {"Emi", "emi"},
        {"emi", "emi"},
        {"Radiation", "radiation"},
        {"radiation", "radiation"},
        {"Sensor", "sensors"},
        {"sensor", "sensors"},
        {"Lidar", "sensors"},
        {"Camera", "sensors"},
        {"Imu", "sensors"},
        {"Magnetometer", "sensors"},
    }};
    for (const Prefix& entry : kPrefixes) {
        if (name.starts_with(entry.prefix)) {
            return entry.domain;
        }
    }
    return {};
}

bool Segmented(const char* id, const char* const* options, int count, int* index) {
    // Design (.seg): one 1 px lineStrong frame around 22 px cells, 11 px text in t2; the
    // chosen cell is filled with the weak accent and written in the accent.
    const theme::Fonts& fonts = theme::fonts();
    bool changed = false;
    ImGui::PushID(id);
    ImGui::PushFont(fonts.sans, theme::kCaptionSize);
    const float height = px(22.0F);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = px(3.0F);
    const ImVec2 first = ImGui::GetCursorScreenPos();
    float right = first.x;
    for (int i = 0; i < count; ++i) {
        if (i != 0) {
            ImGui::SameLine(0.0F, 0.0F);
        }
        const ImVec2 textSize = ImGui::CalcTextSize(options[i]);
        const ImVec2 size(textSize.x + px(16.0F), height);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const ImVec2 max(min.x + size.x, min.y + size.y);
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("##option", size) && *index != i) {
            *index = i;
            changed = true;
        }
        ImGui::PopID();
        const bool selected = *index == i;
        const ImDrawFlags corners = i == 0 ? ImDrawFlags_RoundCornersLeft
                                           : (i == count - 1 ? ImDrawFlags_RoundCornersRight
                                                             : ImDrawFlags_RoundCornersNone);
        if (selected || ImGui::IsItemHovered()) {
            draw->AddRectFilled(min, max, u32(selected ? c::accentWeak : c::bg3), rounding,
                                corners);
        }
        if (i != 0) {
            draw->AddLine(min, ImVec2(min.x, max.y), u32(c::lineStrong));
        }
        draw->AddText(ImVec2(min.x + px(8.0F), min.y + (size.y - textSize.y) * 0.5F),
                      u32(selected ? c::accent : c::t2), options[i]);
        right = max.x;
    }
    draw->AddRect(first, ImVec2(right, first.y + height), u32(c::lineStrong), rounding);
    ImGui::PopFont();
    ImGui::PopID();
    return changed;
}

namespace {

constexpr float kLabelColumn = 112.0F;

/// Draws a read-only value field at the cursor, `width` wide. Returns true when clicked.
bool valueField(const char* id, const char* value, const char* unit, float width,
                const ImVec4& color) {
    const theme::Fonts& fonts = theme::fonts();
    const ImVec2 size(width, px(22.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(id, size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = ImGui::GetStyle().FrameRounding;
    draw->AddRectFilled(min, max, u32(c::bg0), rounding);
    draw->AddRect(min, max, u32(c::line), rounding);
    const float fontSize = px(theme::kMonoSize);
    const float y = min.y + (size.y - fontSize) * 0.5F - px(1.0F);
    float right = max.x - px(6.0F);
    if (unit != nullptr && *unit != '\0') {
        const float unitSize = px(theme::kCaptionSize);
        const ImVec2 extent = fonts.mono->CalcTextSizeA(unitSize, FLT_MAX, 0.0F, unit);
        right -= extent.x;
        draw->AddText(fonts.mono, unitSize, ImVec2(right, min.y + (size.y - unitSize) * 0.5F),
                      u32(c::t3), unit);
        right -= px(6.0F);
    }
    draw->PushClipRect(min, ImVec2(std::max(right, min.x), max.y), true);
    draw->AddText(fonts.mono, fontSize, ImVec2(min.x + px(7.0F), y), u32(color), value);
    draw->PopClipRect();
    return clicked;
}

void rowLabel(const char* label) {
    const float start = ImGui::GetCursorPosX();
    ImGui::AlignTextToFramePadding();
    ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::SameLine(start + px(kLabelColumn));
}

}  // namespace

bool PropertyRow(const char* label, const char* value, const char* unit, bool link,
                 const char* tooltip) {
    ImGui::PushID(label);
    rowLabel(label);
    const bool clicked =
        valueField("##value", value, unit, std::max(ImGui::GetContentRegionAvail().x, px(40.0F)),
                   link ? c::info : c::t1);
    if (tooltip != nullptr && *tooltip != '\0') {
        ImGui::SetItemTooltip("%s", tooltip);
    }
    ImGui::PopID();
    return link && clicked;
}

namespace {

/// The editable 22 px value field of PropertyEdit, `width` wide at the cursor.
bool editField(const char* id, char* buffer, std::size_t size, const char* unit, bool edited,
               float width, bool numeric) {
    const theme::Fonts& fonts = theme::fonts();
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + px(22.0F));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = ImGui::GetStyle().FrameRounding;
    draw->AddRectFilled(min, max, u32(c::bg0), rounding);
    draw->AddRect(min, max, u32(edited ? c::accentBorder : c::line), rounding);
    float unitWidth = 0.0F;
    if (unit != nullptr && *unit != '\0') {
        const float unitSize = px(theme::kCaptionSize);
        const ImVec2 extent = fonts.mono->CalcTextSizeA(unitSize, FLT_MAX, 0.0F, unit);
        unitWidth = extent.x + px(12.0F);
        draw->AddText(fonts.mono, unitSize,
                      ImVec2(max.x - px(6.0F) - extent.x, min.y + (px(22.0F) - unitSize) * 0.5F),
                      u32(c::t3), unit);
    }
    ImGui::PushFont(fonts.mono, theme::kMonoSize);
    const float pad = (px(22.0F) - ImGui::GetFontSize()) * 0.5F;
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(px(7.0F), pad));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0F);
    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0, 0, 0, 0));
    ImGui::SetNextItemWidth(width - unitWidth);
    const bool changed = ImGui::InputText(
        id, buffer, size,
        (numeric ? ImGuiInputTextFlags_CharsScientific : ImGuiInputTextFlags_None) |
            ImGuiInputTextFlags_AutoSelectAll);
    ImGui::PopStyleColor(3);
    ImGui::PopStyleVar(2);
    ImGui::PopFont();
    return changed;
}

}  // namespace

bool PropertyEdit(const char* label, char* buffer, std::size_t size, const char* unit, bool edited,
                  const char* tooltip, bool numeric) {
    ImGui::PushID(label);
    rowLabel(label);
    const float width = std::max(ImGui::GetContentRegionAvail().x, px(40.0F));
    const bool changed = editField("##edit", buffer, size, unit, edited, width, numeric);
    if (tooltip != nullptr && *tooltip != '\0') {
        ImGui::SetItemTooltip("%s", tooltip);
    }
    ImGui::PopID();
    return changed;
}

int PropertyEditPair(const char* label, EditField first, EditField second) {
    ImGui::PushID(label);
    rowLabel(label);
    const float gap = px(6.0F);
    const float width = std::max((ImGui::GetContentRegionAvail().x - gap) * 0.5F, px(40.0F));
    const ImVec2 start = ImGui::GetCursorScreenPos();
    int done = 0;
    int index = 0;
    for (const EditField* field : {&first, &second}) {
        if (index == 1) {
            // After the first field's box, not its text (which stops short of the unit).
            ImGui::SameLine();
            ImGui::SetCursorScreenPos(ImVec2(start.x + width + gap, start.y));
        }
        ImGui::PushID(index);
        (void)editField("##edit", field->buffer, field->size, field->unit, field->edited, width,
                        true);
        if (field->tooltip != nullptr && *field->tooltip != '\0') {
            ImGui::SetItemTooltip("%s", field->tooltip);
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            done |= 1 << index;
        }
        if (ImGui::IsItemActive()) {
            done |= 4 << index;
        }
        ImGui::PopID();
        ++index;
    }
    ImGui::PopID();
    return done;
}

void Vec3Field(const char* label, const double value[3], const char* format) {
    static constexpr std::array<const char*, 3> kAxis{"X", "Y", "Z"};
    const std::array<ImVec4, 3> colors{theme::hex(0xF0605D), theme::hex(0x4FC97A),
                                       theme::hex(0x5AA2FF)};  // design tokens: color.axis
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushID(label);
    rowLabel(label);
    const float gap = px(4.0F);
    const float width = (ImGui::GetContentRegionAvail().x - gap * 2.0F) / 3.0F;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (axis != 0) {
            ImGui::SameLine(0.0F, gap);
        }
        char text[32];
        std::snprintf(text, sizeof(text), format, value[axis]);
        const ImVec2 min = ImGui::GetCursorScreenPos();
        // Leave room for the axis letter by drawing the value after it.
        char padded[40];
        std::snprintf(padded, sizeof(padded), "   %s", text);
        valueField(kAxis[axis], padded, nullptr, std::max(width, px(30.0F)), c::t1);
        const float fontSize = px(theme::kMonoSize);
        draw->AddText(fonts.monoMedium, fontSize,
                      ImVec2(min.x + px(7.0F), min.y + (px(22.0F) - fontSize) * 0.5F - px(1.0F)),
                      u32(colors[axis]), kAxis[axis]);
    }
    ImGui::PopID();
}

void Sparkline(const float* low, const float* high, int columns, const ImVec2& size,
               const ImVec4& color) {
    const ImVec2 min = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    if (columns < 2 || !ImGui::IsItemVisible()) {
        return;
    }
    float bottom = FLT_MAX;
    float top = -FLT_MAX;
    for (int i = 0; i < columns; ++i) {
        if (!std::isnan(low[i])) {
            bottom = std::min(bottom, low[i]);
            top = std::max(top, high[i]);
        }
    }
    if (bottom > top) {
        return;  // no samples yet
    }
    const float span = std::max(top - bottom, 1e-9F * std::max(std::abs(top), 1.0F));
    const float pad = px(2.0F);
    const float scaleY = (size.y - pad * 2.0F) / span;
    const float stepX = size.x / static_cast<float>(columns - 1);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 line = u32(color);
    bool previous = false;
    ImVec2 last;
    for (int i = 0; i < columns; ++i) {
        if (std::isnan(low[i])) {
            previous = false;
            continue;
        }
        const float x = min.x + stepX * static_cast<float>(i);
        const float yLow = min.y + size.y - pad - (low[i] - bottom) * scaleY;
        const float yHigh = min.y + size.y - pad - (high[i] - bottom) * scaleY;
        if (yLow - yHigh > 1.0F) {
            draw->AddLine(ImVec2(x, yLow), ImVec2(x, yHigh), line);  // the range in this column
        }
        const ImVec2 mid(x, (yLow + yHigh) * 0.5F);
        if (previous) {
            draw->AddLine(last, mid, line, px(1.2F));
        }
        last = mid;
        previous = true;
    }
}

void SchemaChip(const char* schema) {
    const theme::Fonts& fonts = theme::fonts();
    const std::string_view domain = schemaDomain(schema);
    const ImVec4 color = domain.empty() ? c::t2 : domainColor(domain);
    ImGui::PushFont(fonts.sansMedium, theme::kCaptionSize);
    const ImVec2 textSize = ImGui::CalcTextSize(schema);
    const float dot = px(6.0F);
    const ImVec2 size(textSize.x + dot + px(18.0F), px(20.0F));
    // Wrap to the next line when the chip does not fit.
    if (ImGui::GetCursorPosX() > ImGui::GetStyle().WindowPadding.x + 1.0F &&
        ImGui::GetContentRegionAvail().x < size.x) {
        ImGui::NewLine();
    }
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(withAlpha(color, 0.10F)), size.y * 0.5F);
    draw->AddRect(min, max, u32(withAlpha(color, 0.35F)), size.y * 0.5F);
    draw->AddCircleFilled(ImVec2(min.x + px(8.0F) + dot * 0.5F, min.y + size.y * 0.5F), dot * 0.5F,
                          u32(color));
    draw->AddText(ImVec2(min.x + px(12.0F) + dot, min.y + (size.y - textSize.y) * 0.5F), u32(color),
                  schema);
    ImGui::PopFont();
    ImGui::Dummy(size);
}

bool Checkbox(const char* id, bool* value) {
    const float box = px(14.0F);
    const float height = ImGui::GetFrameHeight();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(box, height));
    if (clicked) {
        *value = !*value;
    }
    const ImVec2 min(origin.x, origin.y + (height - box) * 0.5F);
    const ImVec2 max(min.x + box, min.y + box);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = px(3.0F);
    if (*value) {
        draw->AddRectFilled(min, max, u32(c::accent), rounding);
        const float t = px(1.6F);
        draw->AddLine(ImVec2(min.x + box * 0.24F, min.y + box * 0.52F),
                      ImVec2(min.x + box * 0.43F, min.y + box * 0.72F), u32(c::onAccent), t);
        draw->AddLine(ImVec2(min.x + box * 0.43F, min.y + box * 0.72F),
                      ImVec2(min.x + box * 0.78F, min.y + box * 0.30F), u32(c::onAccent), t);
    } else {
        draw->AddRectFilled(min, max, u32(c::bg0), rounding);
        draw->AddRect(min, max, u32(ImGui::IsItemHovered() ? c::t3 : c::lineStrong), rounding);
    }
    return clicked;
}

bool FilterChip(const char* label, const ImVec4& color, bool selected) {
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const ImVec2 size(textSize.x + px(16.0F), px(20.0F));
    if (ImGui::GetCursorPosX() > ImGui::GetStyle().WindowPadding.x + 1.0F &&
        ImGui::GetContentRegionAvail().x < size.x) {
        ImGui::NewLine();  // wrap
    }
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(label, size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec4 tone = selected ? color : c::t3;
    draw->AddRectFilled(
        min, max, u32(withAlpha(tone, selected ? 0.16F : (ImGui::IsItemHovered() ? 0.12F : 0.06F))),
        size.y * 0.5F);
    draw->AddRect(min, max, u32(withAlpha(tone, selected ? 0.55F : 0.30F)), size.y * 0.5F);
    draw->AddText(ImVec2(min.x + px(8.0F), min.y + (size.y - textSize.y) * 0.5F),
                  u32(selected ? color : c::t2), label, ImGui::FindRenderedTextEnd(label));
    ImGui::PopFont();
    return clicked;
}

bool DropdownChip(const char* label, const ImVec4& color, bool selected) {
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const float chevron = px(10.0F);
    const ImVec2 size(textSize.x + px(16.0F) + chevron + px(4.0F), px(20.0F));
    if (ImGui::GetCursorPosX() > ImGui::GetStyle().WindowPadding.x + 1.0F &&
        ImGui::GetContentRegionAvail().x < size.x) {
        ImGui::NewLine();  // wrap
    }
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(label, size);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec4 tone = selected ? color : c::t3;
    draw->AddRectFilled(
        min, max, u32(withAlpha(tone, selected ? 0.16F : (ImGui::IsItemHovered() ? 0.12F : 0.06F))),
        size.y * 0.5F);
    draw->AddRect(min, max, u32(withAlpha(tone, selected ? 0.55F : 0.30F)), size.y * 0.5F);
    const ImU32 ink = u32(selected ? color : c::t2);
    draw->AddText(ImVec2(min.x + px(8.0F), min.y + (size.y - textSize.y) * 0.5F), ink, label,
                  ImGui::FindRenderedTextEnd(label));
    DrawIcon(draw, Icon::ChevronDown,
             ImVec2(max.x - px(8.0F) - chevron, min.y + (size.y - chevron) * 0.5F), chevron, ink,
             u32(c::bg2));
    ImGui::PopFont();
    return clicked;
}

void Meter(float fraction, const ImVec4& color, float width) {
    const float height = px(6.0F);
    const float line = ImGui::GetTextLineHeight();
    ImVec2 min = ImGui::GetCursorScreenPos();
    min.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + (line - height) * 0.5F;
    const ImVec2 max(min.x + width, min.y + height);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(min, max, u32(c::bg4), height * 0.5F);
    const float filled = width * std::clamp(fraction, 0.0F, 1.0F);
    if (filled > 0.5F) {
        draw->AddRectFilled(min, ImVec2(min.x + std::max(filled, height), max.y), u32(color),
                            height * 0.5F);
    }
    ImGui::Dummy(ImVec2(width, line));
}

void Swatch(const ImVec4& color) {
    const float side = px(10.0F);
    const float line = ImGui::GetTextLineHeight();
    ImVec2 min = ImGui::GetCursorScreenPos();
    min.y += ImGui::GetCurrentWindow()->DC.CurrLineTextBaseOffset + (line - side) * 0.5F;
    ImGui::GetWindowDrawList()->AddRectFilled(min, ImVec2(min.x + side, min.y + side), u32(color),
                                              px(2.0F));
    ImGui::Dummy(ImVec2(side, line));
}

bool BarTab(const char* label, Icon icon, bool active, bool available) {
    // Design (.ws): icon 14 px + 12 px medium label, 10 px side padding; the active tab has a
    // 2 px accent underline and an accent icon.
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushFont(fonts.sansMedium, theme::kLabelSize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const float iconSize = px(14.0F);
    const float pad = px(10.0F);
    const ImVec2 size(pad + iconSize + px(6.0F) + textSize.x + pad,
                      ImGui::GetContentRegionAvail().y);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(label, size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec4 ink = active ? c::t1 : (hovered ? c::t1 : c::t2);
    if (!available && !active) {
        ink = hovered ? c::t2 : c::t3;
    }
    DrawIcon(draw, icon, ImVec2(min.x + pad, min.y + (size.y - iconSize) * 0.5F), iconSize,
             u32(active ? c::accent : ink), u32(c::bg1));
    draw->AddText(ImVec2(min.x + pad + iconSize + px(6.0F), min.y + (size.y - textSize.y) * 0.5F),
                  u32(ink), label, ImGui::FindRenderedTextEnd(label));
    if (active) {
        draw->AddRectFilled(ImVec2(min.x, max.y - px(2.0F)), max, u32(c::accent));
    }
    ImGui::PopFont();
    return clicked;
}

// --- robot builder -------------------------------------------------------------------------

namespace {

/// The one field being typed in (ImGui has one active item at a time): its text lives here
/// while it is active, so that what the document says does not overwrite what is typed.
struct ActiveField {
    ImGuiID id{0};
    std::array<char, 256> text{};
};
ActiveField& activeField() {
    static ActiveField field;
    return field;
}

void monoRowLabel(const char* label) {
    const float start = ImGui::GetCursorPosX();
    ImGui::AlignTextToFramePadding();
    ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
    ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
    ImGui::PopFont();
    ImGui::SameLine(start + px(kLabelColumn));
}

}  // namespace

bool CommitField(const char* id, std::string* text, const char* unit, bool edited, float width,
                 bool numeric) {
    ActiveField& active = activeField();
    const ImGuiID self = ImGui::GetID(id);
    std::array<char, 256> local{};
    char* buffer = local.data();
    if (active.id == self) {
        buffer = active.text.data();
    } else {
        std::snprintf(local.data(), local.size(), "%s", text->c_str());
    }
    (void)editField(id, buffer, local.size(), unit, edited, width, numeric);
    if (ImGui::IsItemActivated() && active.id != self) {
        active.id = self;
        std::memcpy(active.text.data(), buffer, active.text.size());
    }
    bool committed = false;
    if (ImGui::IsItemDeactivated()) {
        if (ImGui::IsItemDeactivatedAfterEdit() && *text != buffer) {
            *text = buffer;
            committed = true;
        }
        if (active.id == self) {
            active.id = 0;
        }
    }
    return committed;
}

bool CommitEdit(const char* label, std::string* text, const char* unit, bool edited,
                const char* tooltip, bool numeric, bool monoLabel) {
    ImGui::PushID(label);
    if (monoLabel) {
        monoRowLabel(label);
    } else {
        rowLabel(label);
    }
    const float width = std::max(ImGui::GetContentRegionAvail().x, px(40.0F));
    const bool committed = CommitField("##edit", text, unit, edited, width, numeric);
    if (tooltip != nullptr && *tooltip != '\0') {
        ImGui::SetItemTooltip("%s", tooltip);
    }
    ImGui::PopID();
    return committed;
}

int Vec3Edit(const char* label, double value[3], const char* format) {
    static constexpr std::array<const char*, 3> kAxis{"X", "Y", "Z"};
    const std::array<ImVec4, 3> colors{theme::hex(0xF0605D), theme::hex(0x4FC97A),
                                       theme::hex(0x5AA2FF)};  // design tokens: color.axis
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushID(label);
    rowLabel(label);
    const float gap = px(4.0F);
    const float width = std::max((ImGui::GetContentRegionAvail().x - gap * 2.0F) / 3.0F, px(30.0F));
    int committed = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (axis != 0) {
            ImGui::SameLine(0.0F, gap);
        }
        const ImVec2 min = ImGui::GetCursorScreenPos();
        // The axis letter is drawn over the field's padding; the value follows it (strtod
        // skips the leading spaces, and a click selects them with the value).
        char shown[40];
        std::snprintf(shown, sizeof(shown), "   ");
        std::snprintf(shown + 3, sizeof(shown) - 3, format, value[axis]);
        std::string text = shown;
        static constexpr std::array<const char*, 3> kIds{"##x", "##y", "##z"};
        if (CommitField(kIds[axis], &text, nullptr, false, width, true)) {
            char* end = nullptr;
            const double parsed = std::strtod(text.c_str(), &end);
            if (end != text.c_str() && std::isfinite(parsed)) {
                value[axis] = parsed;
                committed |= 1 << static_cast<int>(axis);
            }
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const float fontSize = px(theme::kMonoSize);
        draw->AddText(fonts.monoMedium, fontSize,
                      ImVec2(min.x + px(7.0F), min.y + (px(22.0F) - fontSize) * 0.5F - px(1.0F)),
                      u32(colors[axis]), kAxis[axis]);
    }
    ImGui::PopID();
    return committed;
}

bool DropdownField(const char* id, const char* value, const char* unit, float width) {
    // Design (.fld.dd): the value field with a 4 px down triangle at its right end.
    const theme::Fonts& fonts = theme::fonts();
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + width, min.y + px(22.0F));
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(width, px(22.0F)));
    const bool hovered = ImGui::IsItemHovered();
    const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = ImGui::GetStyle().FrameRounding;
    draw->AddRectFilled(min, max, u32(c::bg0), rounding);
    draw->AddRect(min, max, u32(hovered ? c::lineStrong : c::line), rounding);
    const float midY = min.y + px(11.0F);
    const ImVec2 tip(max.x - px(8.0F), midY + px(2.5F));
    draw->AddTriangleFilled(ImVec2(tip.x - px(4.0F), tip.y - px(5.0F)),
                            ImVec2(tip.x + px(4.0F), tip.y - px(5.0F)), tip, u32(c::t3));
    float right = tip.x - px(10.0F);
    if (unit != nullptr && *unit != '\0') {
        const float unitSize = px(theme::kCaptionSize);
        const ImVec2 extent = fonts.mono->CalcTextSizeA(unitSize, FLT_MAX, 0.0F, unit);
        right -= extent.x;
        draw->AddText(fonts.mono, unitSize, ImVec2(right, midY - unitSize * 0.5F), u32(c::t3),
                      unit);
        right -= px(6.0F);
    }
    const float fontSize = px(theme::kMonoSize);
    draw->PushClipRect(min, ImVec2(std::max(right, min.x), max.y), true);
    draw->AddText(fonts.mono, fontSize, ImVec2(min.x + px(7.0F), midY - fontSize * 0.5F - px(1.0F)),
                  u32(disabled ? c::t3 : c::t1), value);
    draw->PopClipRect();
    return clicked;
}

bool DropdownRow(const char* label, const char* value, const char* unit) {
    ImGui::PushID(label);
    rowLabel(label);
    const bool clicked = DropdownField("##dropdown", value, unit,
                                       std::max(ImGui::GetContentRegionAvail().x, px(40.0F)));
    ImGui::PopID();
    return clicked;
}

bool JointSlider(const char* id, double* value, double lower, double upper, bool wrap, bool knob,
                 float width) {
    // Design (12-builder .trk): 4 px bg4 track; fill #3A4F6E from zero to the knob; zero tick
    // 1 × 12 px lineStrong; knob 14 px t1 with a 2 px bg1 ring and a 1 px lineStrong outline;
    // a continuous joint's track is dashed (6 px on, 3 px off).
    const float height = px(30.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const float knobRadius = px(7.0F);
    const float left = min.x + knobRadius;
    const float right = min.x + std::max(width, knobRadius * 4.0F) - knobRadius;
    const float midY = min.y + height * 0.5F;
    bool changed = false;
    ImGui::InvisibleButton(id, ImVec2(std::max(width, knobRadius * 4.0F), height));
    if (upper <= lower) {
        upper = lower + 1.0;  // a malformed range still draws
    }
    const auto toX = [&](double v) {
        const double t = std::clamp((v - lower) / (upper - lower), 0.0, 1.0);
        return left + static_cast<float>(t) * (right - left);
    };
    if (knob && ImGui::IsItemActive()) {
        const double t = std::clamp(
            static_cast<double>((ImGui::GetIO().MousePos.x - left) / (right - left)), 0.0, 1.0);
        const double next = lower + t * (upper - lower);
        if (next != *value) {
            *value = next;
            changed = true;
        }
    }
    double shown = *value;
    if (wrap) {
        const double span = upper - lower;
        shown = lower + std::fmod(std::fmod(shown - lower, span) + span, span);
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float half = px(2.0F);
    if (wrap) {
        for (float x = left; x < right; x += px(9.0F)) {
            draw->AddRectFilled(ImVec2(x, midY - half),
                                ImVec2(std::min(x + px(6.0F), right), midY + half), u32(c::bg4),
                                half);
        }
    } else {
        draw->AddRectFilled(ImVec2(left, midY - half), ImVec2(right, midY + half), u32(c::bg4),
                            half);
    }
    const float zero = toX(0.0);
    const float at = toX(shown);
    if (!wrap && knob) {
        draw->AddRectFilled(ImVec2(std::min(zero, at), midY - half),
                            ImVec2(std::max(zero, at), midY + half), u32(theme::hex(0x3A4F6E)),
                            half);
    }
    if (lower <= 0.0 && upper >= 0.0) {
        draw->AddRectFilled(ImVec2(zero, midY - px(6.0F)), ImVec2(zero + px(1.0F), midY + px(6.0F)),
                            u32(c::lineStrong));
    }
    if (knob) {
        const ImVec2 centre(at, midY);
        draw->AddCircleFilled(centre, knobRadius + px(1.0F), u32(c::lineStrong));
        draw->AddCircleFilled(centre, knobRadius, u32(c::bg1));
        draw->AddCircleFilled(centre, knobRadius - px(2.0F),
                              u32(ImGui::IsItemHovered() || ImGui::IsItemActive() ? c::t1 : c::t2));
    }
    return changed;
}

bool ShapeTile(const char* id, Icon icon, const char* label) {
    // Design (12-builder .tile): 56 × 54, bg2, 1 px lineStrong, 3 px radius; a 20 px icon in t1
    // over an 11 px label in t2, 5 px apart.
    const theme::Fonts& fonts = theme::fonts();
    const ImVec2 size(px(56.0F), px(54.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    ImGui::PushID(id);
    const bool clicked = ImGui::InvisibleButton("##tile", size);
    ImGui::PopID();
    const bool hovered = ImGui::IsItemHovered();
    const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;
    const float alpha = disabled ? ImGui::GetStyle().DisabledAlpha : 1.0F;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float rounding = px(3.0F);
    draw->AddRectFilled(min, max, u32(withAlpha(hovered && !disabled ? c::bg3 : c::bg2, alpha)),
                        rounding);
    draw->AddRect(min, max, u32(withAlpha(c::lineStrong, alpha)), rounding);
    const float iconSize = px(20.0F);
    const float labelSize = px(theme::kCaptionSize);
    const ImVec2 extent = fonts.sans->CalcTextSizeA(labelSize, FLT_MAX, 0.0F, label);
    const float top = min.y + (size.y - iconSize - px(5.0F) - extent.y) * 0.5F;
    DrawIcon(draw, icon, ImVec2(min.x + (size.x - iconSize) * 0.5F, top), iconSize,
             u32(withAlpha(c::t1, alpha)), u32(c::bg2));
    draw->AddText(fonts.sans, labelSize,
                  ImVec2(min.x + (size.x - extent.x) * 0.5F, top + iconSize + px(5.0F)),
                  u32(withAlpha(c::t2, alpha)), label);
    return clicked && !disabled;
}

TreeHit TreeRow(const char* id, const TreeRowSpec& row) {
    // Design (.tr): 22 px, 12 px text; 14 px of indent per level; caret 12 px in t3; icon 12 px
    // (accent when selected); type 11 px in t3; 6 px badges on the right; selected rows bg
    // accentWeak with a 2 px accent bar, hovered rows bg3.
    const theme::Fonts& fonts = theme::fonts();
    const float height = px(22.0F);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const float width = std::max(ImGui::GetContentRegionAvail().x, px(40.0F));
    const ImVec2 max(min.x + width, min.y + height);
    ImGui::PushID(id);
    const bool clicked = ImGui::InvisibleButton("##row", ImVec2(width, height));
    ImGui::PopID();
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (row.selected) {
        draw->AddRectFilled(min, max, u32(c::accentWeak), px(2.0F));
        draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), max.y), u32(c::accent));
    } else if (hovered) {
        draw->AddRectFilled(min, max, u32(c::bg3), px(2.0F));
    }
    const float midY = min.y + height * 0.5F;
    const float caretX = min.x + px(4.0F) + px(14.0F) * static_cast<float>(row.depth);
    if (row.hasChildren) {
        const float r = px(3.5F);
        const ImVec2 at(caretX + px(6.0F), midY);
        if (row.folded) {
            draw->AddTriangleFilled(ImVec2(at.x - r * 0.6F, at.y - r),
                                    ImVec2(at.x - r * 0.6F, at.y + r),
                                    ImVec2(at.x + r * 0.9F, at.y), u32(c::t3));
        } else {
            draw->AddTriangleFilled(ImVec2(at.x - r, at.y - r * 0.6F),
                                    ImVec2(at.x + r, at.y - r * 0.6F),
                                    ImVec2(at.x, at.y + r * 0.9F), u32(c::t3));
        }
    }
    float x = caretX + px(12.0F) + px(6.0F);
    if (row.icon != Icon::Count) {
        const float icon = px(12.0F);
        DrawIcon(draw, row.icon, ImVec2(x, midY - icon * 0.5F), icon,
                 u32(row.selected ? c::accent : c::t3), u32(c::bg1));
        x += icon + px(6.0F);
    }
    // Badges from the right.
    float right = max.x - px(8.0F);
    const float dot = px(3.0F);
    if (row.error) {
        draw->AddCircleFilled(ImVec2(right - dot, midY), dot, u32(c::error));
        right -= dot * 2.0F + px(3.0F);
    }
    for (int i = row.badgeCount; i-- > 0;) {
        draw->AddCircleFilled(ImVec2(right - dot, midY), dot, u32(row.badges[i]));
        right -= dot * 2.0F + px(3.0F);
    }
    const float nameSize = px(theme::kLabelSize);
    ImFont* nameFont = row.strong ? fonts.sansMedium : fonts.sans;
    const float nameWidth = nameFont->CalcTextSizeA(nameSize, FLT_MAX, 0.0F, row.name).x;
    const float typeSize = px(theme::kCaptionSize);
    const float typeX = x + nameWidth + px(6.0F);
    const float typeWidth = fonts.sans->CalcTextSizeA(typeSize, FLT_MAX, 0.0F, row.type).x;
    draw->PushClipRect(min, ImVec2(std::max(right - px(4.0F), min.x), max.y), true);
    draw->AddText(nameFont, nameSize, ImVec2(x, midY - nameSize * 0.5F - px(1.0F)), u32(c::t1),
                  row.name);
    draw->AddText(fonts.sans, typeSize, ImVec2(typeX, midY - typeSize * 0.5F - px(1.0F)),
                  u32(c::t3), row.type);
    draw->PopClipRect();
    if (!clicked) {
        return TreeHit::None;
    }
    const float mouse = ImGui::GetIO().MousePos.x;
    if (row.hasChildren && mouse >= caretX && mouse < caretX + px(14.0F)) {
        return TreeHit::Caret;
    }
    if (*row.type != '\0' && mouse >= typeX && mouse < typeX + typeWidth) {
        return TreeHit::Type;
    }
    return TreeHit::Row;
}

}  // namespace lagrange::viewer::widgets
