#include "lagrange/viewer/Panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <imgui_internal.h>  // DockBuilder: the supported way to script a default layout

#include <imgui.h>

namespace lagrange::viewer {

namespace {

constexpr double kOrbitRadiansPerPixel = 0.006;
constexpr double kZoomPerWheelStep = 0.88;
constexpr double kSeekIntervalSeconds = 1.0 / 30.0;

class ViewportPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Viewport"; }

    void draw(PanelContext& context) override {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
        const bool open = ImGui::Begin(
            title(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();
        if (open) {
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
            context.requestedViewportWidth =
                static_cast<std::uint32_t>(std::max(available.x * scale.x, 1.0F));
            context.requestedViewportHeight =
                static_cast<std::uint32_t>(std::max(available.y * scale.y, 1.0F));

            const ImVec2 origin = ImGui::GetCursorScreenPos();
            if (context.viewportTexture != 0) {
                ImGui::Image(static_cast<ImTextureID>(context.viewportTexture), available);
            } else {
                ImGui::Dummy(available);
            }
            if (ImGui::IsItemHovered()) {
                const ImGuiIO& io = ImGui::GetIO();
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    context.camera.orbit(-io.MouseDelta.x * kOrbitRadiansPerPixel,
                                         io.MouseDelta.y * kOrbitRadiansPerPixel);
                }
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                    ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
                    context.camera.pan(-io.MouseDelta.x / std::max(available.y, 1.0F),
                                       io.MouseDelta.y / std::max(available.y, 1.0F));
                }
                if (io.MouseWheel != 0.0F) {
                    context.camera.zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
                }
            }

            // Overlay text in the corner of the image.
            ImDrawList* draw = ImGui::GetWindowDrawList();
            char text[160];
            std::snprintf(text, sizeof(text), "%.1f ms   %u draws   %s", context.frameMs,
                          context.drawCalls, context.tierName.c_str());
            draw->AddText(ImVec2(origin.x + 8.0F, origin.y + 6.0F), IM_COL32(220, 220, 220, 200),
                          text);
            if (!context.scene) {
                const char* message = context.connection.state == ConnectionState::Ready
                                          ? "Waiting for the scene..."
                                          : "Not connected to a server";
                const ImVec2 size = ImGui::CalcTextSize(message);
                draw->AddText(ImVec2(origin.x + (available.x - size.x) * 0.5F,
                                     origin.y + (available.y - size.y) * 0.5F),
                              IM_COL32(230, 230, 230, 255), message);
            }
        }
        ImGui::End();
    }
};

class ScenePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Scene"; }

    void draw(PanelContext& context) override {
        if (ImGui::Begin(title())) {
            if (!context.scene) {
                ImGui::TextDisabled("No scene");
            } else {
                const SceneStatics& statics = *context.scene->statics;
                if (statics.generation != generation_) {
                    rebuild(statics);
                }
                for (const std::uint32_t root : roots_) {
                    drawNode(statics, root, context.selectedPrim);
                }
            }
        }
        ImGui::End();
    }

private:
    // Child lists are rebuilt only when a new snapshot arrives, not every frame.
    void rebuild(const SceneStatics& statics) {
        children_.assign(statics.prims.size(), {});
        roots_.clear();
        for (std::uint32_t i = 0; i < statics.prims.size(); ++i) {
            const std::uint32_t parent = statics.prims[i].parent;
            if (parent == kNone) {
                roots_.push_back(i);
            } else {
                children_[parent].push_back(i);
            }
        }
        generation_ = statics.generation;
    }

    void drawNode(const SceneStatics& statics, std::uint32_t id, std::uint32_t& selected) {
        const PrimData& prim = statics.prims[id];
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                   ImGuiTreeNodeFlags_SpanAvailWidth |
                                   ImGuiTreeNodeFlags_DefaultOpen;
        if (children_[id].empty()) {
            flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        }
        if (selected == id) {
            flags |= ImGuiTreeNodeFlags_Selected;
        }
        ImGui::PushID(static_cast<int>(id));
        const bool open = ImGui::TreeNodeEx("##prim", flags, "%s", prim.name.c_str());
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
            selected = id;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", prim.typeName.c_str());
        if (open && !children_[id].empty()) {
            for (const std::uint32_t child : children_[id]) {
                drawNode(statics, child, selected);
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }

    std::vector<std::vector<std::uint32_t>> children_;
    std::vector<std::uint32_t> roots_;
    std::uint64_t generation_{0};
};

class InspectorPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Inspector"; }

    void draw(PanelContext& context) override {
        if (ImGui::Begin(title())) {
            const bool valid = context.scene && context.selectedPrim != kNone &&
                               context.selectedPrim < context.scene->statics->prims.size();
            if (!valid) {
                ImGui::TextDisabled("Select a prim in the Scene panel");
            } else {
                const SceneStatics& statics = *context.scene->statics;
                const PrimData& prim = statics.prims[context.selectedPrim];
                const Transform& transform = context.scene->transforms[context.selectedPrim];
                ImGui::TextUnformatted(prim.path.c_str());
                ImGui::TextDisabled("%s", prim.typeName.c_str());
                ImGui::SeparatorText("Transform");
                ImGui::Text("position  %.3f  %.3f  %.3f  m", transform.position[0],
                            transform.position[1], transform.position[2]);
                ImGui::Text("rotation  %.3f  %.3f  %.3f  %.3f  (w x y z)",
                            static_cast<double>(transform.rotation[0]),
                            static_cast<double>(transform.rotation[1]),
                            static_cast<double>(transform.rotation[2]),
                            static_cast<double>(transform.rotation[3]));
                if (!prim.apiSchemas.empty()) {
                    ImGui::SeparatorText("Schemas");
                    for (const std::string& schema : prim.apiSchemas) {
                        ImGui::BulletText("%s", schema.c_str());
                    }
                }
                if (prim.mesh != kNone) {
                    const MeshData& mesh = statics.meshes[prim.mesh];
                    ImGui::SeparatorText("Geometry");
                    ImGui::Text("%zu vertices, %zu triangles", mesh.positions.size() / 3,
                                mesh.indices.size() / 3);
                    if (prim.material != kNone) {
                        ImGui::Text("material  %s", statics.materials[prim.material].name.c_str());
                    }
                }
                if (ImGui::Button("Look at")) {
                    context.camera.target = transform.position;
                }
            }
        }
        ImGui::End();
    }
};

class TimelinePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Timeline"; }

    void draw(PanelContext& context) override {
        if (ImGui::Begin(title())) {
            const bool ready = context.connection.state == ConnectionState::Ready;
            const bool running = context.status.runState == RunState::Running;
            ImGui::BeginDisabled(!ready);
            if (ImGui::Button(running ? "Pause" : "Play",
                              ImVec2(64.0F * ImGui::GetStyle().FontScaleDpi, 0.0F))) {
                running ? context.client.pause() : context.client.play();
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(!context.connection.canStep || running);
            if (ImGui::Button("Step")) {
                context.client.step(10'000'000);  // 10 ms
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.0F * ImGui::GetStyle().FontScaleDpi);
            static const double kRates[] = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0};
            char label[32];
            std::snprintf(label, sizeof(label), "%.2gx", context.status.realTimeFactorTarget);
            if (ImGui::BeginCombo("##rate", label)) {
                for (const double rate : kRates) {
                    char item[32];
                    std::snprintf(item, sizeof(item), "%.2gx", rate);
                    if (ImGui::Selectable(item, rate == context.status.realTimeFactorTarget)) {
                        context.client.setRealTimeFactor(rate);
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();
            const double begin = static_cast<double>(context.status.replayBeginNs) * 1e-9;
            const double end = static_cast<double>(context.status.replayEndNs) * 1e-9;
            const double now = static_cast<double>(context.status.simTimeNs) * 1e-9;
            ImGui::Text("t = %.3f s", now);

            // While the user drags the slider it shows their value, not the server's.
            auto shown = static_cast<float>(dragging_ ? dragValue_ : now);
            ImGui::BeginDisabled(!context.connection.canSeek || end <= begin);
            ImGui::SetNextItemWidth(-1.0F);
            if (ImGui::SliderFloat("##time", &shown, static_cast<float>(begin),
                                   static_cast<float>(end), "%.2f s")) {
                dragging_ = true;
                dragValue_ = shown;
                const double clock = ImGui::GetTime();
                if (clock - lastSeek_ >= kSeekIntervalSeconds) {
                    context.client.seek(
                        static_cast<std::int64_t>(static_cast<double>(shown) * 1e9));
                    lastSeek_ = clock;
                }
            }
            if (dragging_ && ImGui::IsItemDeactivated()) {
                context.client.seek(
                    static_cast<std::int64_t>(static_cast<double>(dragValue_) * 1e9));
                dragging_ = false;
            }
            ImGui::EndDisabled();
            ImGui::EndDisabled();

            ImGui::Separator();
            ImGui::Text("%s  %s", toString(context.connection.state),
                        context.connection.serverName.c_str());
            if (!context.connection.message.empty()) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0F, 0.6F, 0.4F, 1.0F), "%s",
                                   context.connection.message.c_str());
            }
            if (!context.notice.text.empty()) {
                ImGui::TextColored(context.notice.error ? ImVec4(1.0F, 0.45F, 0.35F, 1.0F)
                                                        : ImVec4(0.65F, 0.85F, 1.0F, 1.0F),
                                   "%s", context.notice.text.c_str());
            }
            ImGui::TextDisabled(
                "%u signals   %llu samples   %llu snapshots   %llu deltas   %llu resyncs   "
                "%llu dropped by server   %s",
                context.status.signalCount, static_cast<unsigned long long>(context.stats.samples),
                static_cast<unsigned long long>(context.stats.snapshots),
                static_cast<unsigned long long>(context.stats.deltas),
                static_cast<unsigned long long>(context.stats.resyncs),
                static_cast<unsigned long long>(context.status.droppedDeltas),
                context.deviceName.c_str());
        }
        ImGui::End();
    }

private:
    bool dragging_{false};
    float dragValue_{0.0F};
    double lastSeek_{0.0};
};

void buildDefaultLayout(ImGuiID dockspace) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->Size);
    ImGuiID centre = dockspace;
    const ImGuiID bottom =
        ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.15F, nullptr, &centre);
    const ImGuiID scope =
        ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.42F, nullptr, &centre);
    const ImGuiID left =
        ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, 0.20F, nullptr, &centre);
    const ImGuiID right =
        ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.26F, nullptr, &centre);
    ImGui::DockBuilderDockWindow("Viewport", centre);
    ImGui::DockBuilderDockWindow("Scene", left);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Timeline", bottom);
    ImGui::DockBuilderDockWindow("Scope", scope);
    ImGui::DockBuilderFinish(dockspace);
}

}  // namespace

std::vector<std::unique_ptr<Panel>> createDefaultPanels() {
    std::vector<std::unique_ptr<Panel>> panels;
    panels.push_back(std::make_unique<ViewportPanel>());
    panels.push_back(std::make_unique<ScenePanel>());
    panels.push_back(std::make_unique<InspectorPanel>());
    panels.push_back(std::make_unique<TimelinePanel>());
    panels.push_back(createScopePanel());
    return panels;
}

void drawPanels(std::vector<std::unique_ptr<Panel>>& panels, PanelContext& context) {
    const ImGuiID dockspace = ImGui::GetID("lagrange-dockspace");
    // Lay out only when there is no saved layout for this dock space.
    if (ImGui::DockBuilderGetNode(dockspace) == nullptr) {
        buildDefaultLayout(dockspace);
    }
    if (context.canOpenFiles && ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open robot or recording...", "Ctrl+O")) {
                context.openFileRequested = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
    if (context.canOpenFiles && ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) {
        context.openFileRequested = true;
    }
    ImGui::DockSpaceOverViewport(dockspace, ImGui::GetMainViewport());
    for (const std::unique_ptr<Panel>& panel : panels) {
        panel->draw(context);
    }
}

}  // namespace lagrange::viewer
