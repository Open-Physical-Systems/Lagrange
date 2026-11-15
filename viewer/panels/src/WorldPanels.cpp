// World workspace: the agents, a top-down map of the tiles with their level of detail, and
// the streaming and floating-origin state (docs/design/ui/10-world/SPEC.md).
//
// World positions are double (invariant 6). The map subtracts its centre in double and only
// then converts to float pixels, so a point far from the origin does not jitter.
#include "PanelFormat.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Geodesy.hpp"
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
#include <optional>
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

/// What the World panels share: the region picked in Regions (the map frames and marks it),
/// and the map's cursor (Geodesy reads it out in latitude and longitude).
struct WorldShared {
    std::string selectedRegion;  // a region's name; empty for none
    bool frameRequested{false};  // the map frames the selected region on its next draw
    bool cursorValid{false};     // the map has been hovered
    bool cursorLive{false};      // ... and is now
    double cursorX{0.0};         // world ENU, metres
    double cursorY{0.0};
};

/// A region as the panels show it: one the server reports, or, when it reports none, the one
/// region its run is (`local`), around everything loaded.
struct RegionView {
    std::string name;
    double minX{0.0};
    double minY{0.0};
    double maxX{0.0};
    double maxY{0.0};
    bool local{false};

    [[nodiscard]] bool contains(double x, double y) const {
        return x >= minX && x < maxX && y >= minY && y < maxY;
    }
};

std::vector<RegionView> regionsOf(const WorldInfo& world) {
    std::vector<RegionView> regions;
    for (const WorldInfo::Region& region : world.regions) {
        regions.push_back({region.name, region.minX, region.minY, region.maxX, region.maxY});
    }
    if (!regions.empty()) {
        return regions;
    }
    // One server, one region: its extent is what it has loaded and where its agents are.
    RegionView local{"this server", DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, true};
    const double tile = world.tileSizeM > 0.0 ? world.tileSizeM : 256.0;
    for (const WorldInfo::Tile& loaded : world.tiles) {
        local.minX = std::min(local.minX, loaded.x * tile);
        local.minY = std::min(local.minY, loaded.y * tile);
        local.maxX = std::max(local.maxX, (loaded.x + 1) * tile);
        local.maxY = std::max(local.maxY, (loaded.y + 1) * tile);
    }
    for (const WorldInfo::Agent& agent : world.agents) {
        const double r = std::max(agent.activeRadiusM, 1.0);
        local.minX = std::min(local.minX, agent.x - r);
        local.minY = std::min(local.minY, agent.y - r);
        local.maxX = std::max(local.maxX, agent.x + r);
        local.maxY = std::max(local.maxY, agent.y + r);
    }
    if (local.minX > local.maxX) {
        local.minX = local.minY = -1.0;
        local.maxX = local.maxY = 1.0;
    }
    regions.push_back(local);
    return regions;
}

/// The name of the region an agent is in, or nullptr when the server reports no regions.
const char* regionOfAgent(const WorldInfo& world, const WorldInfo::Agent& agent) {
    for (const WorldInfo::Region& region : world.regions) {
        if (agent.x >= region.minX && agent.x < region.maxX && agent.y >= region.minY &&
            agent.y < region.maxY) {
            return region.name.c_str();
        }
    }
    return nullptr;
}

/// An agent's height in world ENU from its prim in the scene, when the scene has it.
std::optional<double> agentUp(const PanelContext& context, const WorldInfo::Agent& agent) {
    if (!context.scene || !context.scene->statics || agent.prim == kNone ||
        agent.prim >= context.scene->transforms.size()) {
        return std::nullopt;
    }
    return context.scene->transforms[agent.prim].position[2] + context.scene->statics->regionEnu[2];
}

std::optional<geodesy::Origin> geoOriginOf(const WorldInfo& world) {
    if (!world.georeferenced) {
        return std::nullopt;
    }
    return geodesy::Origin({world.latitudeDeg, world.longitudeDeg, world.heightM});
}

/// "1.2 km" or "850 m".
void length(double metres, char* out, std::size_t size) {
    if (std::abs(metres) >= 1000.0) {
        std::snprintf(out, size, "%.2f km", metres / 1000.0);
    } else {
        std::snprintf(out, size, "%.0f m", metres);
    }
}

void bytes(std::uint64_t value, char* out, std::size_t size) {
    const double megabytes = static_cast<double>(value) / (1024.0 * 1024.0);
    if (megabytes >= 1024.0) {
        std::snprintf(out, size, "%.1f GB", megabytes / 1024.0);
    } else {
        std::snprintf(out, size, "%.0f MB", megabytes);
    }
}

// --- scenery --------------------------------------------------------------------------------

std::array<double, 3> rotated(const std::array<float, 4>& q, const std::array<double, 3>& v) {
    // q = (w, x, y, z): v + 2w(u × v) + 2u × (u × v), u the vector part.
    const double w = q[0];
    const std::array<double, 3> u{q[1], q[2], q[3]};
    const std::array<double, 3> t{2.0 * (u[1] * v[2] - u[2] * v[1]),
                                  2.0 * (u[2] * v[0] - u[0] * v[2]),
                                  2.0 * (u[0] * v[1] - u[1] * v[0])};
    return {v[0] + w * t[0] + (u[1] * t[2] - u[2] * t[1]),
            v[1] + w * t[1] + (u[2] * t[0] - u[0] * t[2]),
            v[2] + w * t[2] + (u[0] * t[1] - u[1] * t[0])};
}

/// The outlines of the scene's collision shapes seen from above, for the map of a world that
/// streams no tiles (it would be empty otherwise): each shape's box at its prim's pose now.
/// Rebuilt when the scene changes; only the moving prims' poses are read each frame.
class Scenery {
public:
    struct Outline {
        std::uint32_t prim{kNone};                      // kNone: fixed in the region-local frame
        std::array<std::array<double, 3>, 4> corner{};  // in the prim's frame, m
    };

    /// Shapes larger than this across are the ground or terrain: left out.
    static constexpr double kMaxExtentM = 200.0;

    void update(const SceneFrame& frame) {
        if (!frame.statics || frame.statics->generation == generation_) {
            return;
        }
        generation_ = frame.statics->generation;
        outlines_.clear();
        const SceneStatics& statics = *frame.statics;
        for (const ColliderData& collider : statics.colliders) {
            if (collider.mesh >= statics.meshes.size()) {
                continue;
            }
            const std::array<float, 6>& box = statics.meshes[collider.mesh].aabb;
            if (box[3] - box[0] > kMaxExtentM || box[4] - box[1] > kMaxExtentM) {
                continue;
            }
            Outline outline;
            outline.prim = collider.prim;
            const double z = 0.5 * (box[2] + box[5]);
            const std::array<std::array<double, 2>, 4> xy{
                {{box[0], box[1]}, {box[3], box[1]}, {box[3], box[4]}, {box[0], box[4]}}};
            for (std::size_t k = 0; k < 4; ++k) {
                const std::array<double, 3> local =
                    rotated(collider.rotation, {xy[k][0], xy[k][1], z});
                outline.corner[k] = {local[0] + collider.position[0],
                                     local[1] + collider.position[1],
                                     local[2] + collider.position[2]};
            }
            outlines_.push_back(outline);
        }
    }

    [[nodiscard]] const std::vector<Outline>& outlines() const { return outlines_; }

    /// A corner in world ENU (the map's frame): through its prim's pose now, then the region.
    static std::array<double, 2> world(const SceneFrame& frame, const Outline& outline,
                                       std::size_t k) {
        std::array<double, 3> p = outline.corner[k];
        if (outline.prim != kNone && outline.prim < frame.transforms.size()) {
            const Transform& t = frame.transforms[outline.prim];
            const std::array<double, 3> r = rotated(t.rotation, p);
            p = {r[0] + t.position[0], r[1] + t.position[1], r[2] + t.position[2]};
        }
        const std::array<double, 3>& region = frame.statics->regionEnu;
        return {p[0] + region[0], p[1] + region[1]};
    }

private:
    std::uint64_t generation_{0};
    std::vector<Outline> outlines_;
};

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
            if (const char* region = regionOfAgent(*world, agent)) {
                std::snprintf(line, sizeof(line), "region %s · %s · %.1f m/s", region,
                              agent.kind.c_str(), agent.speed);
            } else {
                std::snprintf(line, sizeof(line), "%s · %.1f m/s · %.0f m active",
                              agent.kind.c_str(), agent.speed, agent.activeRadiusM);
            }
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

// --- regions ------------------------------------------------------------------------------

class RegionsPanel final : public Panel {
public:
    explicit RegionsPanel(std::shared_ptr<WorldShared> shared) : shared_(std::move(shared)) {}

    [[nodiscard]] const char* title() const override { return "Regions"; }
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
        const std::vector<RegionView> regions = regionsOf(*world);
        const std::optional<geodesy::Origin> geo = geoOriginOf(*world);
        const double tile = world->tileSizeM > 0.0 ? world->tileSizeM : 256.0;
        const theme::Fonts& fonts = theme::fonts();
        ImGui::SetCursorPosY(top.y);
        char text[128];
        const RegionView* chosen = nullptr;
        for (std::size_t i = 0; i < regions.size(); ++i) {
            const RegionView& region = regions[i];
            std::size_t agents = 0;
            bool focus = false;
            for (const WorldInfo::Agent& agent : world->agents) {
                if (region.local || region.contains(agent.x, agent.y)) {
                    ++agents;
                    focus = focus || agent.focus;
                }
            }
            const float width = ImGui::GetContentRegionAvail().x;
            const float height = px(52.0F);
            const ImVec2 min = ImGui::GetCursorScreenPos();
            const ImVec2 max(min.x + width, min.y + height);
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::InvisibleButton("##region", ImVec2(width, height))) {
                shared_->selectedRegion = region.name;
                shared_->frameRequested = true;
            }
            ImGui::PopID();
            const bool selected = region.name == shared_->selectedRegion;
            if (selected) {
                chosen = &region;
            }
            ImDrawList* draw = ImGui::GetWindowDrawList();
            if (selected) {
                draw->AddRectFilled(min, max, u32(c::accentWeak));
                draw->AddRectFilled(min, ImVec2(min.x + px(2.0F), max.y), u32(c::accent));
            } else if (ImGui::IsItemHovered()) {
                draw->AddRectFilled(min, max, u32(c::bg3));
            }
            draw->AddLine(ImVec2(min.x, max.y), max, u32(c::line));
            // A small dashed square: a region's boundary as the map draws it.
            const ImVec2 box(min.x + px(9.0F), min.y + px(11.0F));
            const float side = px(10.0F);
            for (float t = 0.0F; t < side; t += px(4.0F)) {
                const float e = std::min(t + px(2.0F), side);
                const ImU32 dash = u32(selected ? c::accent : c::t3);
                draw->AddLine(ImVec2(box.x + t, box.y), ImVec2(box.x + e, box.y), dash);
                draw->AddLine(ImVec2(box.x + t, box.y + side), ImVec2(box.x + e, box.y + side),
                              dash);
                draw->AddLine(ImVec2(box.x, box.y + t), ImVec2(box.x, box.y + e), dash);
                draw->AddLine(ImVec2(box.x + side, box.y + t), ImVec2(box.x + side, box.y + e),
                              dash);
            }
            draw->AddText(fonts.sansMedium, px(theme::kBodySize),
                          ImVec2(min.x + px(28.0F), min.y + px(8.0F)), u32(c::t1),
                          region.name.c_str());
            char across[24];
            char along[24];
            length(region.maxX - region.minX, across, sizeof(across));
            length(region.maxY - region.minY, along, sizeof(along));
            std::size_t tiles = 0;
            for (const WorldInfo::Tile& loaded : world->tiles) {
                if (region.local ||
                    region.contains((loaded.x + 0.5) * tile, (loaded.y + 0.5) * tile)) {
                    ++tiles;
                }
            }
            std::snprintf(text, sizeof(text), "%s × %s · %zu tiles%s", across, along, tiles,
                          region.local ? " · one server" : "");
            draw->AddText(fonts.mono, px(theme::kCaptionSize),
                          ImVec2(min.x + px(28.0F), min.y + px(30.0F)), u32(c::t3), text);
            std::snprintf(text, sizeof(text), "%zu agent%s", agents, agents == 1 ? "" : "s");
            const float pill = widgets::PillWidth(text);
            const ImVec2 after = ImGui::GetCursorScreenPos();
            ImGui::SetCursorScreenPos(ImVec2(max.x - pill - px(10.0F), min.y + px(8.0F)));
            widgets::Pill(text, focus ? widgets::PillKind::Accent : widgets::PillKind::Neutral);
            ImGui::SetCursorScreenPos(after);
            ImGui::Dummy(ImVec2(0.0F, 0.0F));
        }

        // The picked region in detail (the first one until one is picked).
        if (chosen == nullptr && !regions.empty()) {
            chosen = &regions.front();
        }
        ImGui::Indent(px(8.0F));
        if (chosen != nullptr &&
            widgets::SectionHeader(chosen->name.c_str(), chosen->local ? "local" : "region")) {
            const RegionView& region = *chosen;
            std::snprintf(text, sizeof(text), "%+.1f … %+.1f", region.minX, region.maxX);
            widgets::PropertyRow("east (x)", text, "m");
            std::snprintf(text, sizeof(text), "%+.1f … %+.1f", region.minY, region.maxY);
            widgets::PropertyRow("north (y)", text, "m");
            const double cx = 0.5 * (region.minX + region.maxX);
            const double cy = 0.5 * (region.minY + region.maxY);
            std::snprintf(text, sizeof(text), "%+.1f, %+.1f", cx, cy);
            widgets::PropertyRow("centre ENU", text, "m");
            if (geo) {
                const geodesy::Geodetic g = geo->toGeodetic({cx, cy, 0.0});
                std::snprintf(text, sizeof(text), "%.6f, %.6f", g.latitudeDeg, g.longitudeDeg);
                widgets::PropertyRow("centre lat, lon", text, "°");
            }
            std::array<std::uint32_t, 4> lods{};
            for (const WorldInfo::Tile& loaded : world->tiles) {
                if (region.local ||
                    region.contains((loaded.x + 0.5) * tile, (loaded.y + 0.5) * tile)) {
                    ++lods.at(std::min<std::size_t>(loaded.lod, 3));
                }
            }
            std::snprintf(text, sizeof(text), "%u · %u · %u · %u", lods[0], lods[1], lods[2],
                          lods[3]);
            widgets::PropertyRow("tiles LOD0…3", text, nullptr);
            if (region.local) {
                widgets::PropertyRow("origin anchor",
                                     world->anchor.empty() ? "-" : world->anchor.c_str(), nullptr);
                std::snprintf(text, sizeof(text), "%+.1f, %+.1f", world->origin[0],
                              world->origin[1]);
                widgets::PropertyRow("region origin", text, "m");
                std::snprintf(text, sizeof(text), "%llu",
                              static_cast<unsigned long long>(world->recenters));
                widgets::PropertyRow("re-centers", text, nullptr);
            }
            // Its agents: where each is, in world ENU.
            std::size_t shown = 0;
            for (const WorldInfo::Agent& agent : world->agents) {
                if (!region.local && !region.contains(agent.x, agent.y)) {
                    continue;
                }
                if (shown++ == kAgentRows) {
                    break;
                }
                std::snprintf(text, sizeof(text), "%+.1f, %+.1f", agent.x, agent.y);
                widgets::PropertyRow(agent.name.c_str(), text, "m");
            }
            if (shown > kAgentRows) {
                widgets::Caption("and more: the Agents tab lists every one.");
            }
        }
        if (widgets::SectionHeader("Handoff", "1 server today")) {
            ImGui::PushTextWrapPos(0.0F);
            if (world->regions.empty()) {
                widgets::Caption(
                    "This server runs its world as one region. Robots far apart run in "
                    "regions of their own, each with its own origin, in headless runs; a "
                    "viewer sees its server's region.");
            } else {
                std::snprintf(text, sizeof(text),
                              "%zu region%s · boundaries ready for multi-server handoff",
                              world->regions.size(), world->regions.size() == 1 ? "" : "s");
                widgets::Caption("%s", text);
            }
            widgets::Caption("Distributed simulation is post-1.0.");
            ImGui::PopTextWrapPos();
        }
        ImGui::Unindent(px(8.0F));
        ImGui::End();
    }

private:
    static constexpr std::size_t kAgentRows = 12;  // agents listed under a region
    std::shared_ptr<WorldShared> shared_;
};

// --- map ----------------------------------------------------------------------------------

class MapPanel final : public Panel {
public:
    explicit MapPanel(std::shared_ptr<WorldShared> shared) : shared_(std::move(shared)) {}

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
        // The scenery's outlines, where no tiles are streamed (a world in one piece).
        const bool drawScenery = world->tiles.empty() && context.scene != nullptr;
        if (drawScenery) {
            scenery_.update(*context.scene);
        }

        // First view: centred on the focus agent, wide enough for the loaded tiles (or, for a
        // world in one piece, for its scenery and agents).
        if (!framed_) {
            for (const WorldInfo::Agent& agent : world->agents) {
                if (agent.focus) {
                    centreX_ = agent.x;
                    centreY_ = agent.y;
                }
            }
            double extent = world->tiles.empty() ? kMinExtentM : tile * 4.0;
            for (const WorldInfo::Tile& loaded : world->tiles) {
                extent = std::max({extent, std::abs((loaded.x + 0.5) * tile - centreX_),
                                   std::abs((loaded.y + 0.5) * tile - centreY_)});
            }
            if (drawScenery) {
                for (const Scenery::Outline& outline : scenery_.outlines()) {
                    for (std::size_t k = 0; k < 4; ++k) {
                        const auto p = Scenery::world(*context.scene, outline, k);
                        extent = std::max(
                            {extent, std::abs(p[0] - centreX_), std::abs(p[1] - centreY_)});
                    }
                }
                for (const WorldInfo::Agent& agent : world->agents) {
                    extent = std::max(
                        {extent, std::abs(agent.x - centreX_), std::abs(agent.y - centreY_)});
                }
                extent = std::min(extent, kMaxFrameM);
            }
            metresPerPixel_ = (extent * 2.2) / std::max(std::min(size.x, size.y), 1.0F);
            framed_ = !world->tiles.empty() || !world->agents.empty();
        }
        // The region picked in the Regions tab: framed once, with a margin.
        if (shared_->frameRequested) {
            shared_->frameRequested = false;
            for (const RegionView& region : regionsOf(*world)) {
                if (region.name == shared_->selectedRegion) {
                    centreX_ = 0.5 * (region.minX + region.maxX);
                    centreY_ = 0.5 * (region.minY + region.maxY);
                    const double across =
                        std::max((region.maxX - region.minX) / std::max(size.x, 1.0F),
                                 (region.maxY - region.minY) / std::max(size.y, 1.0F));
                    metresPerPixel_ = std::clamp(across * 1.15, 0.01, 500.0);
                    framed_ = true;
                }
            }
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
        // Scenery: each collision shape's outline from above, under the agents.
        if (drawScenery) {
            for (const Scenery::Outline& outline : scenery_.outlines()) {
                std::array<ImVec2, 4> corners{};
                bool visible = false;
                for (std::size_t k = 0; k < 4; ++k) {
                    const auto p = Scenery::world(*context.scene, outline, k);
                    corners.at(k) = toScreen(p[0], p[1]);
                    visible =
                        visible ||
                        (corners.at(k).x >= origin.x && corners.at(k).x <= origin.x + size.x &&
                         corners.at(k).y >= origin.y && corners.at(k).y <= origin.y + size.y);
                }
                if (!visible) {
                    continue;
                }
                draw->AddConvexPolyFilled(corners.data(), 4, u32(theme::hex(0x3A4E6E, 0.35F)));
                draw->AddPolyline(corners.data(), 4, u32(theme::hex(0x6E7F98, 0.8F)),
                                  ImDrawFlags_Closed, px(1.0F));
            }
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
        // The region picked in the Regions tab, marked in the accent.
        if (!shared_->selectedRegion.empty()) {
            for (const RegionView& region : regionsOf(*world)) {
                if (region.name != shared_->selectedRegion) {
                    continue;
                }
                const ImVec2 a = toScreen(region.minX, region.maxY);
                const ImVec2 b = toScreen(region.maxX, region.minY);
                draw->AddRectFilled(a, b, u32(theme::hex(0xE8772E, 0.06F)));
                draw->AddRect(a, b, u32(c::accent), 0.0F, 0, px(1.5F));
                if (region.local) {
                    draw->AddText(fonts.mono, small, ImVec2(a.x + px(6.0F), a.y + px(4.0F)),
                                  u32(c::accent), region.name.c_str());
                }
            }
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

        // Cursor position in the world's ENU frame, and the tile under it; Geodesy reads it.
        shared_->cursorLive = ImGui::IsItemHovered();
        if (shared_->cursorLive) {
            const double x =
                centreX_ + static_cast<double>(io.MousePos.x - middle.x) * metresPerPixel_;
            const double y =
                centreY_ - static_cast<double>(io.MousePos.y - middle.y) * metresPerPixel_;
            shared_->cursorValid = true;
            shared_->cursorX = x;
            shared_->cursorY = y;
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
    static constexpr double kMinExtentM = 10.0;   // a small world still fills a sensible view
    static constexpr double kMaxFrameM = 2000.0;  // the first view of a world in one piece
    std::shared_ptr<WorldShared> shared_;
    Scenery scenery_;
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

class GeodesyPanel final : public Panel {
public:
    explicit GeodesyPanel(std::shared_ptr<WorldShared> shared) : shared_(std::move(shared)) {}

    [[nodiscard]] const char* title() const override { return "Geodesy"; }
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
        const std::optional<geodesy::Origin> geo = geoOriginOf(*world);
        const double tile = world->tileSizeM > 0.0 ? world->tileSizeM : 256.0;
        char text[128];
        char unit[32];

        if (widgets::SectionHeader("Datum", nullptr)) {
            ImGui::Indent(px(8.0F));
            widgets::PropertyRow("datum", "WGS84 → local ENU", nullptr);
            std::snprintf(text, sizeof(text), "%.3f", geodesy::kSemiMajorAxisM);
            widgets::PropertyRow("semi-major a", text, "m");
            std::snprintf(text, sizeof(text), "%.9f", geodesy::kInverseFlattening);
            widgets::PropertyRow("1 / f", text, nullptr);
            widgets::PropertyRow("axes", "x east · y north · z up", nullptr);
            ImGui::Unindent(px(8.0F));
        }

        if (widgets::SectionHeader("Geo origin", geo ? "stage" : "none")) {
            ImGui::Indent(px(8.0F));
            if (geo) {
                const geodesy::Geodetic& g = geo->geodetic();
                std::snprintf(text, sizeof(text), "%.7f", g.latitudeDeg);
                widgets::PropertyRow("latitude", text, "°");
                std::snprintf(text, sizeof(text), "%.7f", g.longitudeDeg);
                widgets::PropertyRow("longitude", text, "°");
                widgets::PropertyRow("lat · DMS", geodesy::dms(g.latitudeDeg, 'N', 'S').c_str(),
                                     nullptr);
                widgets::PropertyRow("lon · DMS", geodesy::dms(g.longitudeDeg, 'E', 'W').c_str(),
                                     nullptr);
                std::snprintf(text, sizeof(text), "%.2f", g.heightM);
                widgets::PropertyRow("height", text, "m ellipsoid");
                const std::string zone = geodesy::utmZone(g.latitudeDeg, g.longitudeDeg);
                widgets::PropertyRow("UTM zone", zone.empty() ? "UPS (polar)" : zone.c_str(),
                                     nullptr);
                const geodesy::Ecef& e = geo->ecef();
                std::snprintf(text, sizeof(text), "%.3f, %.3f, %.3f", e.x / 1000.0, e.y / 1000.0,
                              e.z / 1000.0);
                widgets::PropertyRow("ECEF", text, "km");
                const std::array<double, 2> perDegree = geo->metresPerDegree();
                std::snprintf(text, sizeof(text), "%.1f", perDegree[0]);
                widgets::PropertyRow("1° latitude", text, "m");
                std::snprintf(text, sizeof(text), "%.1f", perDegree[1]);
                widgets::PropertyRow("1° longitude", text, "m");
            } else {
                ImGui::PushTextWrapPos(0.0F);
                widgets::Caption(
                    "This stage is not georeferenced: positions are metres in its own frame. "
                    "Give it a place with --geo-origin LAT,LON,HEIGHT or the profile's "
                    "world.geo_origin.");
                ImGui::PopTextWrapPos();
            }
            ImGui::Unindent(px(8.0F));
        }

        // Where the run's local frame is, in the world.
        if (widgets::SectionHeader("Floating origin", nullptr)) {
            ImGui::Indent(px(8.0F));
            std::snprintf(text, sizeof(text), "%+.1f, %+.1f, %+.1f", world->origin[0],
                          world->origin[1], world->origin[2]);
            widgets::PropertyRow("origin ENU", text, "m");
            if (geo) {
                const geodesy::Geodetic g =
                    geo->toGeodetic({world->origin[0], world->origin[1], world->origin[2]});
                std::snprintf(text, sizeof(text), "%.7f, %.7f", g.latitudeDeg, g.longitudeDeg);
                widgets::PropertyRow("origin lat, lon", text, "°");
            }
            ImGui::Unindent(px(8.0F));
        }

        // The map's cursor: in the world's ENU, and on the Earth.
        if (widgets::SectionHeader("Cursor",
                                   shared_->cursorLive
                                       ? "on the map"
                                       : (shared_->cursorValid ? "last on the map" : nullptr))) {
            ImGui::Indent(px(8.0F));
            if (shared_->cursorValid) {
                const double x = shared_->cursorX;
                const double y = shared_->cursorY;
                std::snprintf(text, sizeof(text), "%+.1f, %+.1f", x, y);
                widgets::PropertyRow("ENU", text, "m");
                std::snprintf(text, sizeof(text), "(%d, %d)",
                              static_cast<int>(std::floor(x / tile)),
                              static_cast<int>(std::floor(y / tile)));
                std::snprintf(unit, sizeof(unit), "%.0f m tiles", tile);
                widgets::PropertyRow("tile", text, unit);
                if (geo) {
                    const geodesy::Geodetic g = geo->toGeodetic({x, y, 0.0});
                    std::snprintf(text, sizeof(text), "%.7f, %.7f", g.latitudeDeg, g.longitudeDeg);
                    widgets::PropertyRow("lat, lon", text, "°");
                    std::snprintf(text, sizeof(text), "%.3f", geo->planeRiseM(x, y));
                    widgets::PropertyRow("flat-world error", text, "m");
                }
            } else {
                widgets::Caption("Hover the map to read a position here.");
            }
            ImGui::Unindent(px(8.0F));
        }

        std::snprintf(text, sizeof(text), "%zu", world->agents.size());
        if (widgets::SectionHeader("Agents", text)) {
            ImGui::Indent(px(8.0F));
            std::size_t shown = 0;
            double farthest = 0.0;
            for (const WorldInfo::Agent& agent : world->agents) {
                farthest = std::max(farthest, std::hypot(agent.x, agent.y));
                if (shown++ >= kAgentRows) {
                    continue;
                }
                const std::optional<double> up = agentUp(context, agent);
                char tip[160];
                if (geo) {
                    const geodesy::Geodetic g =
                        geo->toGeodetic({agent.x, agent.y, up.value_or(0.0)});
                    std::snprintf(text, sizeof(text), "%.6f, %.6f", g.latitudeDeg, g.longitudeDeg);
                    if (up) {
                        std::snprintf(
                            tip, sizeof(tip),
                            "ENU %+.2f, %+.2f, %+.2f m\nheight %.2f m above the ellipsoid", agent.x,
                            agent.y, *up, g.heightM);
                    } else {
                        std::snprintf(tip, sizeof(tip), "ENU %+.2f, %+.2f m", agent.x, agent.y);
                    }
                    widgets::PropertyRow(agent.name.c_str(), text, "°", false, tip);
                } else {
                    std::snprintf(text, sizeof(text), "%+.1f, %+.1f, %+.1f", agent.x, agent.y,
                                  up.value_or(0.0));
                    widgets::PropertyRow(agent.name.c_str(), text, "m");
                }
            }
            if (shown > kAgentRows) {
                widgets::Caption("and %zu more", shown - kAgentRows);
            }
            // How far the flat world is from the Earth's curve where its farthest agent is.
            if (geo && farthest >= 1.0) {
                char far[24];
                length(farthest, far, sizeof(far));
                std::snprintf(text, sizeof(text), "%.3f", geo->planeRiseM(farthest, 0.0));
                std::snprintf(unit, sizeof(unit), "m at %s", far);
                widgets::PropertyRow("flat-world error", text, unit, false,
                                     "How far the flat world (the tangent plane at the geo "
                                     "origin) is above the ellipsoid as far out as the "
                                     "farthest agent: about d² / 2R.");
            }
            ImGui::Unindent(px(8.0F));
        }
        ImGui::End();
    }

private:
    static constexpr std::size_t kAgentRows = 12;
    std::shared_ptr<WorldShared> shared_;
};

}  // namespace

std::vector<std::unique_ptr<Panel>> createWorldPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    auto shared = std::make_shared<WorldShared>();
    panels.push_back(std::make_unique<AgentsPanel>());
    panels.push_back(std::make_unique<RegionsPanel>(shared));
    panels.push_back(std::make_unique<MapPanel>(shared));
    panels.push_back(std::make_unique<StreamingPanel>());
    panels.push_back(std::make_unique<GeodesyPanel>(shared));
    return panels;
}

}  // namespace lagrange::viewer
