#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <array>
#include <cmath>
#include <cstdarg>
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
    ImVec4 color = c::t2;
    switch (kind) {
        case PillKind::Ok:
            color = c::ok;
            break;
        case PillKind::Warn:
            color = c::warn;
            break;
        case PillKind::Error:
            color = c::error;
            break;
        case PillKind::Info:
            color = c::info;
            break;
        case PillKind::Accent:
            color = c::accent;
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
    draw->AddRectFilled(min, max, u32(withAlpha(color, 0.14F)), size.y * 0.5F);
    draw->AddRect(min, max, u32(withAlpha(color, 0.40F)), size.y * 0.5F);
    draw->AddText(ImVec2(min.x + px(7.0F), min.y + (size.y - textSize.y) * 0.5F), u32(color), text);
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(size.x, lineHeight));
}

bool Switch(const char* id, bool* value) {
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
    if (ImGui::IsItemHovered() || ImGui::IsItemFocused()) {
        draw->AddRect(min, max, u32(*value ? c::t1 : c::lineStrong), radius);
    }
    const float knob = radius - px(2.5F);
    const float x = *value ? max.x - radius : min.x + radius;
    draw->AddCircleFilled(ImVec2(x, min.y + radius), knob, u32(*value ? c::onAccent : c::t2));
    return clicked;
}

bool PrimaryButton(const char* label) {
    ImGui::PushStyleColor(ImGuiCol_Button, c::accent);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::hex(0xF08A45));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, theme::hex(0xD0661F));
    ImGui::PushStyleColor(ImGuiCol_Text, c::onAccent);
    ImGui::PushFont(theme::fonts().sansMedium, theme::kLabelSize);
    const bool clicked = ImGui::Button(label);
    ImGui::PopFont();
    ImGui::PopStyleColor(4);
    return clicked;
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
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushFont(fonts.sansMedium, theme::kLabelSize);
    const ImVec2 nameSize = ImGui::CalcTextSize(name);
    ImGui::PopFont();
    ImGui::PushFont(fonts.mono, theme::kCaptionSize);
    const ImVec2 detailSize = ImGui::CalcTextSize(detail);
    ImGui::PopFont();

    const float pad = px(9.0F);
    const float dot = px(8.0F);
    const float gap = px(6.0F);
    const ImVec2 size(
        pad + dot + gap + nameSize.x + (detailSize.x > 0.0F ? gap + detailSize.x : 0) + pad,
        px(22.0F));
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();

    const bool dim = state == ChipState::Off;
    const bool hollow =
        state == ChipState::Substitute || state == ChipState::Off || state == ChipState::Pending;
    ImVec4 border = c::lineStrong;
    ImVec4 text = dim ? c::t3 : c::t1;
    if (state == ChipState::Pending) {
        border = c::accent;
        text = c::accent;
    } else if (state == ChipState::Error) {
        border = c::error;
    }
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float radius = size.y * 0.5F;
    draw->AddRectFilled(min, max, u32(hovered ? c::bg3 : c::bg2), radius);
    if (state == ChipState::Substitute || state == ChipState::Off || state == ChipState::Pending) {
        dashedRect(draw, min, max, u32(border), radius);
    } else {
        draw->AddRect(min, max, u32(border), radius);
    }
    const float midY = min.y + size.y * 0.5F;
    const ImVec4 dotColor =
        state == ChipState::Pending ? c::accent : (dim ? withAlpha(color, 0.5F) : color);
    if (hollow) {
        draw->AddCircle(ImVec2(min.x + pad + dot * 0.5F, midY), dot * 0.5F - px(0.75F),
                        u32(dotColor), 0, px(1.5F));
    } else {
        draw->AddCircleFilled(ImVec2(min.x + pad + dot * 0.5F, midY), dot * 0.5F, u32(dotColor));
    }
    float x = min.x + pad + dot + gap;
    draw->AddText(fonts.sansMedium, px(theme::kLabelSize), ImVec2(x, midY - nameSize.y * 0.5F),
                  u32(text), name);
    x += nameSize.x + gap;
    draw->AddText(fonts.mono, px(theme::kCaptionSize), ImVec2(x, midY - detailSize.y * 0.5F),
                  u32(state == ChipState::Pending ? c::accent : c::t3), detail);
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

bool SectionHeader(const char* title, const char* meta, bool defaultOpen) {
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
    const float titleSize = px(theme::kBodySize);
    draw->AddText(fonts.sansSemiBold, titleSize, ImVec2(min.x + px(24.0F), midY - titleSize * 0.6F),
                  u32(c::t1), title);
    if (meta != nullptr && *meta != '\0') {
        const float metaSize = px(theme::kMonoSize);
        const ImVec2 extent = fonts.mono->CalcTextSizeA(metaSize, FLT_MAX, 0.0F, meta);
        draw->AddText(fonts.mono, metaSize,
                      ImVec2(max.x - extent.x - px(10.0F), midY - extent.y * 0.5F), u32(c::t3),
                      meta);
    }
    return open;
}

bool BarTab(const char* label, bool active) {
    const theme::Fonts& fonts = theme::fonts();
    ImGui::PushFont(fonts.sansMedium, theme::kBodySize);
    const ImVec2 textSize = ImGui::CalcTextSize(label, nullptr, true);
    const ImVec2 size(textSize.x + px(24.0F), ImGui::GetContentRegionAvail().y);
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const ImVec2 max(min.x + size.x, min.y + size.y);
    const bool clicked = ImGui::InvisibleButton(label, size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (hovered && !active) {
        draw->AddRectFilled(min, max, u32(c::bg3), ImGui::GetStyle().FrameRounding);
    }
    draw->AddText(ImVec2(min.x + px(12.0F), min.y + (size.y - textSize.y) * 0.5F),
                  u32(active ? c::t1 : c::t2), label, ImGui::FindRenderedTextEnd(label));
    if (active) {
        draw->AddRectFilled(ImVec2(min.x + px(6.0F), max.y - px(2.0F)),
                            ImVec2(max.x - px(6.0F), max.y), u32(c::accent));
    }
    ImGui::PopFont();
    return clicked;
}

}  // namespace lagrange::viewer::widgets
