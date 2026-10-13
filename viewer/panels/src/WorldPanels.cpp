// World workspace: the agents, a top-down map of the tiles with their level of detail, and
// the streaming and floating-origin state (docs/design/ui/10-world/SPEC.md).
//
// World positions are double (invariant 6). The map subtracts its centre in double and only
// then converts to float pixels, so a point far from the origin does not jitter.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

// Design tokens, colormap.tileLod: LOD0 physics and sensors .. LOD3 proxy.
const std::array<ImVec4, 4> kLodColors{theme::hex(0xE8772E), theme::hex(0xA0583A),
                                       theme::hex(0x3A4E6E), theme::hex(0x1D2E48)};
constexpr std::array<const char*, 4> kLodNames{
    "LOD0 · physics + sensors", "LOD1 · render + collision", "LOD2 · render only", "LOD3 · proxy"};

ImU32 u32(const ImVec4& color) {
    return ImGui::ColorConvertFloat4ToU32(color);
}

bool containsNoCase(std::string_view text, std::string_view needle) {
    const auto it =
        std::search(text.begin(), text.end(), needle.begin(), needle.end(), [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a)) ==
                   std::tolower(static_cast<unsigned char>(b));
        });
    return it != text.end();
}

std::shared_ptr<const WorldInfo> worldOrHint(PanelContext& context) {
    context.client.setWorldWanted(true);
    std::shared_ptr<const WorldInfo> world = context.client.world();
    if (!world) {
        widgets::Caption("%s", context.connection.state == ConnectionState::Ready
                                   ? "Waiting for the server's world state ..."
                                   : "Not connected. The world appears when a simulation "
                                     "server is reachable.");
    }
    return world;
}

ImVec4 agentColor(const WorldInfo::Agent& agent) {
    if (agent.focus) {
        return c::accent;
    }
    if (agent.kind == "vehicle") {
        return c::info;
    }
    if (agent.kind == "drone") {
        return widgets::domainColor("aerodynamics");
    }
    return c::t1;
}

void bytes(std::uint64_t value, char* out, std::size_t size) {
    const double megabytes = static_cast<double>(value) / (1024.0 * 1024.0);
    if (megabytes >= 1024.0) {
        std::snprintf(out, size, "%.1f GB", megabytes / 1024.0);
    } else {
        std::snprintf(out, size, "%.0f MB", megabytes);
    }
}

// --- agents -------------------------------------------------------------------------------

class AgentsPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Agents"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::World; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 top = ImGui::GetCursorPos();  // below the tab bar
        ImGui::SetCursorPos(ImVec2(top.x + px(8.0F), top.y + px(6.0F)));
        const std::shared_ptr<const WorldInfo> world = worldOrHint(context);
        if (!world) {
            ImGui::End();
            return;
        }
        widgets::SearchField("##filter", "Filter agents", filter_, sizeof(filter_),
                             ImGui::GetContentRegionAvail().x - px(8.0F));
        ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
        if (world->agents.empty()) {
            ImGui::SetCursorPosX(px(8.0F));
            widgets::Caption("This world has no agents.");
        }
        const theme::Fonts& fonts = theme::fonts();
        for (std::size_t i = 0; i < world->agents.size(); ++i) {
            const WorldInfo::Agent& agent = world->agents[i];
            if (filter_[0] != '\0' && !containsNoCase(agent.name, filter_)) {
                continue;
            }
            const float width = ImGui::GetContentRegionAvail().x;
            const float height = px(52.0F);
            const ImVec2 min = ImGui::GetCursorScreenPos();
            const ImVec2 max(min.x + width, min.y + height);
            ImGui::PushID(static_cast<int>(i));
            // Selecting an agent selects its prim everywhere.
            if (ImGui::InvisibleButton("##agent", ImVec2(width, height)) && agent.prim != kNone) {
                context.selectedPrim = agent.prim;
            }
            ImGui::PopID();
            ImDrawList* draw = ImGui::GetWindowDrawList();
            const bool selected = agent.prim != kNone && context.selectedPrim == agent.prim;
            if (selected) {
                draw->AddRectFilled(min, max, u32(c::accentWeak));
                draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), max.y), u32(c::accent));
            } else if (ImGui::IsItemHovered()) {
                draw->AddRectFilled(min, max, u32(c::bg3));
            }
            draw->AddLine(ImVec2(min.x, max.y), max, u32(c::line));
            draw->AddCircleFilled(ImVec2(min.x + px(14.0F), min.y + px(16.0F)), px(4.0F),
                                  u32(agentColor(agent)));
            draw->AddText(fonts.sansMedium, px(theme::kBodySize),
                          ImVec2(min.x + px(28.0F), min.y + px(8.0F)), u32(c::t1),
                          agent.name.c_str());
            char line[96];
            std::snprintf(line, sizeof(line), "%s · %.1f m/s · %.0f m active", agent.kind.c_str(),
                          agent.speed, agent.activeRadiusM);
            draw->AddText(fonts.mono, px(theme::kCaptionSize),
                          ImVec2(min.x + px(28.0F), min.y + px(30.0F)), u32(c::t3), line);
            if (!agent.status.empty()) {
                const float pill = widgets::PillWidth(agent.status.c_str());
                const ImVec2 after = ImGui::GetCursorScreenPos();
                ImGui::SetCursorScreenPos(ImVec2(max.x - pill - px(10.0F), min.y + px(8.0F)));
                widgets::Pill(agent.status.c_str(),
                              agent.focus ? widgets::PillKind::Accent : widgets::PillKind::Neutral);
                ImGui::SetCursorScreenPos(after);
                ImGui::Dummy(ImVec2(0.0F, 0.0F));
            }
        }
        ImGui::End();
    }

private:
    char filter_[96]{};
};

// --- map ----------------------------------------------------------------------------------

class MapPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Map"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::World; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        const ImVec2 top = ImGui::GetCursorPos();  // below the tab bar
        ImGui::SetCursorPos(ImVec2(top.x + px(8.0F), top.y + px(8.0F)));
        const std::shared_ptr<const WorldInfo> world = worldOrHint(context);
        if (!world) {
            ImGui::End();
            return;
        }
        ImGui::SetCursorPos(top);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton(
            "##map", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const ImGuiIO& io = ImGui::GetIO();
        const double tile = world->tileSizeM > 0.0 ? world->tileSizeM : 256.0;

        // First view: centred on the focus agent, wide enough for the loaded tiles.
        if (!framed_) {
            for (const WorldInfo::Agent& agent : world->agents) {
                if (agent.focus) {
                    centreX_ = agent.x;
                    centreY_ = agent.y;
                }
            }
            double extent = tile * 4.0;
            for (const WorldInfo::Tile& loaded : world->tiles) {
                extent = std::max({extent, std::abs((loaded.x + 0.5) * tile - centreX_),
                                   std::abs((loaded.y + 0.5) * tile - centreY_)});
            }
            metresPerPixel_ = (extent * 2.2) / std::max(std::min(size.x, size.y), 1.0F);
            framed_ = !world->tiles.empty() || !world->agents.empty();
        }
        if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Left) ||
                                      ImGui::IsMouseDragging(ImGuiMouseButton_Right))) {
            centreX_ -= static_cast<double>(io.MouseDelta.x) * metresPerPixel_;
            centreY_ += static_cast<double>(io.MouseDelta.y) * metresPerPixel_;
        }
        if (ImGui::IsItemHovered() && io.MouseWheel != 0.0F) {
            metresPerPixel_ =
                std::clamp(metresPerPixel_ * std::pow(0.88, io.MouseWheel), 0.01, 500.0);
        }
        // East to the right, north up. The subtraction is in double; only the difference,
        // which is small on screen, becomes float.
        const ImVec2 middle(origin.x + size.x * 0.5F, origin.y + size.y * 0.5F);
        const auto toScreen = [&](double x, double y) {
            return ImVec2(middle.x + static_cast<float>((x - centreX_) / metresPerPixel_),
                          middle.y - static_cast<float>((y - centreY_) / metresPerPixel_));
        };
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
                            u32(theme::hex(0x091120)));

        // Tiles, coloured by level of detail. Only those on screen are drawn.
        std::array<std::uint32_t, 4> counts{};
        const float tilePixels = static_cast<float>(tile / metresPerPixel_);
        for (const WorldInfo::Tile& loaded : world->tiles) {
            const std::size_t lod = std::min<std::size_t>(loaded.lod, 3);
            ++counts.at(lod);
            const ImVec2 min = toScreen(loaded.x * tile, (loaded.y + 1) * tile);
            if (min.x > origin.x + size.x || min.y > origin.y + size.y ||
                min.x + tilePixels < origin.x || min.y + tilePixels < origin.y) {
                continue;
            }
            ImVec4 fill = kLodColors.at(lod);
            fill.w = lod == 0 ? 0.55F : (lod == 1 ? 0.50F : 0.70F);
            draw->AddRectFilled(ImVec2(min.x + 1.0F, min.y + 1.0F),
                                ImVec2(min.x + tilePixels - 1.0F, min.y + tilePixels - 1.0F),
                                u32(fill));
        }
        // Regions: dashed outline with the name, ready for handoff between servers later.
        const theme::Fonts& fonts = theme::fonts();
        const float small = px(theme::kCaptionSize);
        for (const WorldInfo::Region& region : world->regions) {
            const ImVec2 a = toScreen(region.minX, region.maxY);
            const ImVec2 b = toScreen(region.maxX, region.minY);
            for (float x = a.x; x < b.x; x += px(8.0F)) {
                draw->AddLine(ImVec2(x, a.y), ImVec2(std::min(x + px(4.0F), b.x), a.y), u32(c::t3));
                draw->AddLine(ImVec2(x, b.y), ImVec2(std::min(x + px(4.0F), b.x), b.y), u32(c::t3));
            }
            for (float y = a.y; y < b.y; y += px(8.0F)) {
                draw->AddLine(ImVec2(a.x, y), ImVec2(a.x, std::min(y + px(4.0F), b.y)), u32(c::t3));
                draw->AddLine(ImVec2(b.x, y), ImVec2(b.x, std::min(y + px(4.0F), b.y)), u32(c::t3));
            }
            draw->AddText(fonts.mono, small, ImVec2(a.x + px(6.0F), a.y + px(4.0F)), u32(c::t3),
                          region.name.c_str());
        }
        // Agents: active region, dot, label; a crosshair on the floating origin's anchor.
        for (const WorldInfo::Agent& agent : world->agents) {
            const ImVec2 at = toScreen(agent.x, agent.y);
            const ImVec4 color = agentColor(agent);
            if (agent.activeRadiusM > 0.0) {
                draw->AddCircle(at, static_cast<float>(agent.activeRadiusM / metresPerPixel_),
                                u32(color), 64, px(1.5F));
            }
            draw->AddCircleFilled(at, px(4.5F), u32(color));
            if (agent.name == world->anchor) {
                draw->AddLine(ImVec2(at.x - px(14.0F), at.y), ImVec2(at.x + px(14.0F), at.y),
                              u32(color));
                draw->AddLine(ImVec2(at.x, at.y - px(14.0F)), ImVec2(at.x, at.y + px(14.0F)),
                              u32(color));
            }
            char label[96];
            std::snprintf(label, sizeof(label), "%s%s", agent.name.c_str(),
                          agent.name == world->anchor ? " · origin" : "");
            draw->AddText(fonts.mono, small, ImVec2(at.x + px(10.0F), at.y - small - px(2.0F)),
                          u32(color), label);
        }
        draw->PopClipRect();

        // Legend with a count per tile state (design: top right).
        const ImVec2 legendMin(origin.x + size.x - px(232.0F), origin.y + px(10.0F));
        const ImVec2 legendMax(origin.x + size.x - px(10.0F), origin.y + px(146.0F));
        draw->AddRectFilled(legendMin, legendMax, u32(theme::hex(0x091120, 0.92F)), px(4.0F));
        draw->AddRect(legendMin, legendMax, u32(c::lineStrong), px(4.0F));
        char text[96];
        std::snprintf(text, sizeof(text), "tile state · %.0f m tiles", tile);
        draw->AddText(fonts.sansSemiBold, px(theme::kLabelSize),
                      ImVec2(legendMin.x + px(10.0F), legendMin.y + px(8.0F)), u32(c::t1), text);
        for (std::size_t lod = 0; lod < 4; ++lod) {
            const float y = legendMin.y + px(32.0F) + px(20.0F) * static_cast<float>(lod);
            draw->AddRectFilled(ImVec2(legendMin.x + px(10.0F), y + px(2.0F)),
                                ImVec2(legendMin.x + px(20.0F), y + px(12.0F)),
                                u32(kLodColors.at(lod)), px(2.0F));
            draw->AddText(fonts.sans, px(theme::kLabelSize), ImVec2(legendMin.x + px(28.0F), y),
                          u32(c::t2), kLodNames.at(lod));
            std::snprintf(text, sizeof(text), "%u", counts.at(lod));
            const float width =
                fonts.mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, text).x;
            draw->AddText(fonts.mono, px(theme::kMonoSize),
                          ImVec2(legendMax.x - width - px(10.0F), y), u32(c::t1), text);
        }
        draw->AddCircle(ImVec2(legendMin.x + px(15.0F), legendMin.y + px(120.0F)), px(5.0F),
                        u32(c::accent), 0, px(1.5F));
        draw->AddText(fonts.sans, px(theme::kLabelSize),
                      ImVec2(legendMin.x + px(28.0F), legendMin.y + px(113.0F)), u32(c::t2),
                      "active region");

        // Scale bar: a round length that is about 150 px wide.
        const double target = metresPerPixel_ * static_cast<double>(px(150.0F));
        const double magnitude = std::pow(10.0, std::floor(std::log10(std::max(target, 1e-9))));
        const double lead = target / magnitude;
        const double length = (lead >= 5.0 ? 5.0 : (lead >= 2.0 ? 2.0 : 1.0)) * magnitude;
        const float bar = static_cast<float>(length / metresPerPixel_);
        const ImVec2 barAt(origin.x + px(16.0F), origin.y + size.y - px(64.0F));
        draw->AddLine(barAt, ImVec2(barAt.x + bar, barAt.y), u32(c::t1), px(1.5F));
        draw->AddLine(ImVec2(barAt.x, barAt.y - px(5.0F)), ImVec2(barAt.x, barAt.y + px(5.0F)),
                      u32(c::t1));
        draw->AddLine(ImVec2(barAt.x + bar, barAt.y - px(5.0F)),
                      ImVec2(barAt.x + bar, barAt.y + px(5.0F)), u32(c::t1));
        if (length >= 1000.0) {
            std::snprintf(text, sizeof(text), "%g km", length / 1000.0);
        } else {
            std::snprintf(text, sizeof(text), "%g m", length);
        }
        draw->AddText(fonts.mono, px(theme::kMonoSize),
                      ImVec2(barAt.x + bar + px(8.0F), barAt.y - px(7.0F)), u32(c::t1), text);

        // Cursor position in the world's ENU frame, and the tile under it.
        if (ImGui::IsItemHovered()) {
            const double x =
                centreX_ + static_cast<double>(io.MousePos.x - middle.x) * metresPerPixel_;
            const double y =
                centreY_ - static_cast<double>(io.MousePos.y - middle.y) * metresPerPixel_;
            std::snprintf(text, sizeof(text), "cursor ENU  x %+.1f  y %+.1f m  ·  tile (%d, %d)", x,
                          y, static_cast<int>(std::floor(x / tile)),
                          static_cast<int>(std::floor(y / tile)));
            const float width =
                fonts.mono->CalcTextSizeA(px(theme::kMonoSize), FLT_MAX, 0.0F, text).x;
            const ImVec2 min(origin.x + px(10.0F), origin.y + size.y - px(36.0F));
            draw->AddRectFilled(min, ImVec2(min.x + width + px(16.0F), min.y + px(26.0F)),
                                u32(theme::hex(0x091120, 0.92F)), px(4.0F));
            draw->AddRect(min, ImVec2(min.x + width + px(16.0F), min.y + px(26.0F)),
                          u32(c::lineStrong), px(4.0F));
            draw->AddText(fonts.mono, px(theme::kMonoSize),
                          ImVec2(min.x + px(8.0F), min.y + px(6.0F)), u32(c::t1), text);
        }
        if (world->synthetic) {
            ImGui::SetCursorScreenPos(ImVec2(origin.x + px(16.0F), origin.y + size.y - px(104.0F)));
            widgets::Pill("synthetic world · this server streams no tiles",
                          widgets::PillKind::Warn);
        }
        ImGui::End();
    }

private:
    double centreX_{0.0};  // world ENU of the middle of the view, metres
    double centreY_{0.0};
    double metresPerPixel_{4.0};
    bool framed_{false};
};

// --- streaming and geodesy ----------------------------------------------------------------

class StreamingPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Streaming"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::World; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        const std::shared_ptr<const WorldInfo> world = worldOrHint(context);
        if (!world) {
            ImGui::End();
            return;
        }
        char text[96];
        // Budgets: a meter when the server reports a budget, otherwise the plain value.
        budget("GPU memory", world->gpuMemoryBytes, world->gpuMemoryBudgetBytes, 0.75F);
        budget("host cache", world->hostCacheBytes, world->hostCacheBudgetBytes, 0.9F);
        std::snprintf(text, sizeof(text), "%.0f MB/s", world->diskReadBytesPerS / 1e6);
        widgets::PropertyRow("disk read", text, nullptr);
        std::snprintf(text, sizeof(text), "%u tiles", world->loadQueue);
        widgets::PropertyRow("load queue", text, nullptr);
        std::snprintf(text, sizeof(text), "%zu", world->tiles.size());
        widgets::PropertyRow("tiles loaded", text, nullptr);

        if (widgets::SectionHeader("Floating origin", nullptr)) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("anchored to", world->anchor.empty() ? "-" : world->anchor.c_str(),
                                 nullptr);
            std::snprintf(text, sizeof(text), "%+.1f, %+.1f", world->origin[0], world->origin[1]);
            widgets::PropertyRow("origin ENU", text, "m");
            std::snprintf(text, sizeof(text), "%.0f", world->recenterThresholdM);
            widgets::PropertyRow("re-center at", text, "m drift");
            if (world->recenters == 0) {
                widgets::PropertyRow("last re-center", "never", nullptr);
            } else {
                format::clock(world->lastRecenterNs, text, sizeof(text));
                widgets::PropertyRow("last re-center", text, "sim");
            }
            widgets::PropertyRow("world positions", "f64 world · f32 local", nullptr);
            ImGui::Unindent(px(8.0F));
        }
        if (widgets::SectionHeader("Georeference", nullptr)) {
            ImGui::Indent(px(8.0F));
            if (world->georeferenced) {
                widgets::PropertyRow("datum", "WGS84 → local ENU", nullptr);
                std::snprintf(text, sizeof(text), "%.6f, %.6f", world->latitudeDeg,
                              world->longitudeDeg);
                widgets::PropertyRow("origin lat, lon", text, "°");
                std::snprintf(text, sizeof(text), "%.1f", world->heightM);
                widgets::PropertyRow("origin height", text, "m");
            } else {
                widgets::Caption(
                    "This stage is not georeferenced: positions are in its own frame.");
            }
            ImGui::Unindent(px(8.0F));
        }
        char regions[32];
        std::snprintf(regions, sizeof(regions), "%zu", world->regions.size());
        if (widgets::SectionHeader("Regions", regions)) {
            ImGui::Indent(px(8.0F));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "Region boundaries are where a world could be split between "
                "servers. Distributed simulation is planned after 1.0.");
            ImGui::PopTextWrapPos();
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    static void budget(const char* label, std::uint64_t used, std::uint64_t limit, float warnAt) {
        char value[64];
        char a[24];
        bytes(used, a, sizeof(a));
        if (limit == 0) {
            widgets::PropertyRow(label, used == 0 ? "not used" : a, nullptr);
            return;
        }
        char b[24];
        bytes(limit, b, sizeof(b));
        std::snprintf(value, sizeof(value), "%s / %s", a, b);
        const float fraction =
            static_cast<float>(static_cast<double>(used) / static_cast<double>(limit));
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        ImGui::TextUnformatted(label);
        ImGui::PopStyleColor();
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        const float width = ImGui::CalcTextSize(value).x;
        ImGui::PopFont();
        ImGui::SameLine();
        widgets::AlignRight(px(90.0F) + px(8.0F) + width);
        widgets::Meter(fraction, fraction > warnAt ? c::warn : c::ok, px(90.0F));
        ImGui::SameLine(0.0F, px(8.0F));
        widgets::Mono(c::t1, "%s", value);
    }
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createWorldPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<AgentsPanel>());
    panels.push_back(std::make_unique<MapPanel>());
    panels.push_back(std::make_unique<StreamingPanel>());
    return panels;
}

}  // namespace lagrange::viewer
