#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/TextFit.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>
#include <map>

namespace lagrange::viewer::format {

void clock(std::int64_t simTimeNs, char* buffer, std::size_t size) {
    const std::int64_t ms = simTimeNs / 1'000'000;
    std::snprintf(buffer, size, "%02lld:%02lld.%03lld", static_cast<long long>(ms / 60'000),
                  static_cast<long long>(ms / 1000 % 60), static_cast<long long>(ms % 1000));
}

void step(std::int64_t dtNs, char* buffer, std::size_t size) {
    if (dtNs <= 0) {
        std::snprintf(buffer, size, "-");
    } else if (dtNs < 1'000) {
        std::snprintf(buffer, size, "%lld ns", static_cast<long long>(dtNs));
    } else if (dtNs < 1'000'000) {
        std::snprintf(buffer, size, "%g µs", static_cast<double>(dtNs) * 1e-3);
    } else if (dtNs < 1'000'000'000) {
        std::snprintf(buffer, size, "%g ms", static_cast<double>(dtNs) * 1e-6);
    } else {
        std::snprintf(buffer, size, "%g s", static_cast<double>(dtNs) * 1e-9);
    }
}

void domainDetail(const DomainView& view, char* buffer, std::size_t size) {
    const char* pending = view.pending ? " · pending" : "";
    if (!view.enabled) {
        std::snprintf(buffer, size, "→ %s%s", view.mode.empty() ? "off" : view.mode.c_str(),
                      pending);
        return;
    }
    char dt[24];
    step(view.dtNs, dt, sizeof(dt));
    if (view.mode.empty()) {
        std::snprintf(buffer, size, "%s%s", dt, pending);
    } else if (view.dtNs > 0) {
        std::snprintf(buffer, size, "%s · %s%s", view.mode.c_str(), dt, pending);
    } else {
        std::snprintf(buffer, size, "%s%s", view.mode.c_str(), pending);
    }
}

bool isPhysicsDomain(std::string_view name) {
    constexpr std::array<std::string_view, 8> kDomains{"rigid_body",   "electrical", "thermal",
                                                       "aerodynamics", "weather",    "emi",
                                                       "radiation",    "sensors"};
    for (const std::string_view domain : kDomains) {
        if (domain == name) {
            return true;
        }
    }
    return false;
}

}  // namespace lagrange::viewer::format

namespace lagrange::viewer::fit {

namespace {

float widthOf(ImFont* font, float size, std::string_view text) {
    return font->CalcTextSizeA(size, FLT_MAX, 0.0F, text.data(), text.data() + text.size()).x;
}

}  // namespace

std::string cut(ImFont* font, float size, std::string_view text, float width, bool middle) {
    const TextWidth measure = [font, size](std::string_view s) { return widthOf(font, size, s); };
    return middle ? fitPath(text, width, measure) : fitEnd(text, width, measure);
}

std::string cutDistinct(ImFont* font, float size, std::string_view text,
                        std::span<const std::string> group, float width) {
    const TextWidth measure = [font, size](std::string_view s) { return widthOf(font, size, s); };
    return fitDistinct(text, group, width, measure);
}

std::vector<std::string> cutAll(ImFont* font, float size, std::span<const std::string> texts,
                                float width) {
    std::vector<std::string> shown;
    shown.reserve(texts.size());
    for (const std::string& text : texts) {
        shown.push_back(cut(font, size, text, width, true));
    }
    // group[i]: the texts row i is told apart from (its own group, grown by collisions).
    std::vector<std::size_t> group(texts.size());
    for (std::size_t i = 0; i < group.size(); ++i) {
        group[i] = i;
    }
    const auto root = [&](std::size_t i) {
        while (group[i] != i) {
            i = group[i];
        }
        return i;
    };
    constexpr int kRounds = 4;
    for (int round = 0; round < kRounds; ++round) {
        std::map<std::string_view, std::size_t> first;
        bool merged = false;
        for (std::size_t i = 0; i < texts.size(); ++i) {
            const auto [at, inserted] = first.emplace(shown[i], i);
            if (!inserted && texts[at->second] != texts[i] && root(at->second) != root(i)) {
                group[root(i)] = root(at->second);
                merged = true;
            }
        }
        if (!merged) {
            break;
        }
        std::map<std::size_t, std::vector<std::string>> members;
        for (std::size_t i = 0; i < texts.size(); ++i) {
            members[root(i)].push_back(texts[i]);
        }
        for (std::size_t i = 0; i < texts.size(); ++i) {
            const std::vector<std::string>& alike = members[root(i)];
            if (alike.size() > 1) {
                shown[i] = cutDistinct(font, size, texts[i], alike, width);
            }
        }
    }
    return shown;
}

bool draw(ImDrawList* draw, ImFont* font, float size, const ImVec2& at, ImU32 color,
          std::string_view text, float width, bool middle) {
    if (widthOf(font, size, text) <= width) {
        draw->AddText(font, size, at, color, text.data(), text.data() + text.size());
        return false;
    }
    const std::string shown = cut(font, size, text, width, middle);
    draw->AddText(font, size, at, color, shown.c_str());
    return true;
}

void text(std::string_view text, float width, bool middle) {
    ImFont* font = ImGui::GetFont();
    const float size = ImGui::GetFontSize();
    if (widthOf(font, size, text) <= width) {
        ImGui::TextUnformatted(text.data(), text.data() + text.size());
        return;
    }
    const std::string shown = cut(font, size, text, std::max(width, 0.0F), middle);
    ImGui::TextUnformatted(shown.c_str());
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%.*s", static_cast<int>(text.size()), text.data());
    }
}

void textRight(std::string_view text, bool middle) {
    const float available = ImGui::GetContentRegionAvail().x;
    const float width = widthOf(ImGui::GetFont(), ImGui::GetFontSize(), text);
    if (width < available) {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + available - width);
    }
    fit::text(text, available, middle);
}

void caption(std::string_view text, float width) {
    ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
    ImGui::PushStyleColor(ImGuiCol_Text, theme::c::t3);
    fit::text(text, width);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void pill(const std::string& text, widgets::PillKind kind, float width) {
    const float padding = widgets::PillWidth("");
    if (widgets::PillWidth(text.c_str()) <= width) {
        widgets::Pill(text.c_str(), kind);
        return;
    }
    ImGui::PushFont(theme::fonts().sansMedium, theme::kCaptionSize);  // as PillWidth measures
    const std::string shown =
        cut(ImGui::GetFont(), ImGui::GetFontSize(), text, std::max(width - padding, 0.0F));
    ImGui::PopFont();
    widgets::Pill(shown.c_str(), kind);
    ImGui::SetItemTooltip("%s", text.c_str());
}

}  // namespace lagrange::viewer::fit
