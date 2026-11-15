#include "PanelFormat.hpp"
#include "lagrange/viewer/TextFit.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>

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

}  // namespace lagrange::viewer::fit
