// Signals workspace, centre: stacked plots with a linked time axis, cursors, a single-shot
// trigger and CSV export (docs/design/ui/05-signals/SPEC.md).
#include "ScopePlots.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <imgui.h>
#include <implot.h>

namespace lagrange::viewer {

namespace {

namespace c = theme::c;
using widgets::px;

constexpr std::array<const char*, 2> kModes{"Live", "Paused"};
constexpr std::array<const char*, 4> kWindows{"1 s", "10 s", "60 s", "run"};
constexpr std::array<const char*, 2> kEdges{"falling", "rising"};

/// The last two parts of a signal name: enough to tell traces apart in a legend.
const char* shortName(const std::string& label) {
    const std::size_t last = label.rfind('/');
    if (last == std::string::npos || last == 0) {
        return label.c_str();
    }
    const std::size_t before = label.rfind('/', last - 1);
    return label.c_str() + (before == std::string::npos ? 0 : before + 1);
}

class ScopePanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return "Scope"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Signals; }

    void draw(PanelContext& context) override {
        ScopeSession& session = *context.scopeSession;
        // Signals asked for on the command line or by another panel need the catalog first.
        if (context.initialScopeSignals != nullptr && !context.initialScopeSignals->empty() &&
            context.catalog) {
            for (const std::string& name : *context.initialScopeSignals) {
                // "body/b/pose.x": one component of a signal, by its name.
                const std::size_t dot = name.rfind('.');
                const SignalInfo* whole =
                    dot == std::string::npos ? nullptr : context.catalog->find(name.substr(0, dot));
                std::uint32_t component = 0;
                if (context.catalog->find(name) == nullptr && whole != nullptr) {
                    const auto it = std::find(whole->componentNames.begin(),
                                              whole->componentNames.end(), name.substr(dot + 1));
                    if (it != whole->componentNames.end()) {
                        component = static_cast<std::uint32_t>(it - whole->componentNames.begin());
                        session.add(context.client, *context.catalog, whole->name, component);
                        continue;
                    }
                }
                session.add(context.client, *context.catalog, name, 0);
            }
            context.initialScopeSignals->clear();
        }
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        toolbar(context, session);
        if (session.traces().empty()) {
            widgets::Caption("%s", context.catalog
                                       ? "No signals plotted. Tick one in the Signals list."
                                       : (context.connection.state == ConnectionState::Ready
                                              ? "Waiting for the signal list ..."
                                              : "Not connected. Signals appear when a "
                                                "simulation server is reachable."));
        } else {
            plotter_.draw(session);
        }
        ImGui::End();
    }

private:
    void toolbar(PanelContext& context, ScopeSession& session) {
        int mode = session.paused() ? 1 : 0;
        if (widgets::Segmented("##mode", kModes.data(), 2, &mode)) {
            session.setPaused(mode == 1);
        }
        ImGui::SameLine(0.0F, px(12.0F));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("window");
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Segmented("##window", kWindows.data(), 4, &session.window);

        ImGui::SameLine(0.0F, px(12.0F));
        ImGui::BeginDisabled(session.traces().empty());
        if (widgets::Button("Trigger ...")) {
            ImGui::OpenPopup("##trigger");
        }
        triggerPopup(session);
        const ScopeSession::Trigger& trigger = session.trigger();
        if (trigger.armed || trigger.firedNs >= 0) {
            char text[96];
            if (trigger.armed) {
                std::snprintf(
                    text, sizeof(text), "armed · %s %s %g",
                    shortName(session.traces()[static_cast<std::size_t>(trigger.trace)].label),
                    trigger.rising ? "↑" : "↓", trigger.level);
            } else {
                std::snprintf(text, sizeof(text), "fired at %.4f s",
                              static_cast<double>(trigger.firedNs) * 1e-9);
            }
            ImGui::SameLine(0.0F, px(8.0F));
            widgets::Pill(text,
                          trigger.armed ? widgets::PillKind::Accent : widgets::PillKind::Warn);
        }

        ImGui::SameLine(0.0F, px(12.0F));
        if (widgets::Button("Cursors A·B", session.cursors ? widgets::ButtonKind::On
                                                           : widgets::ButtonKind::Default)) {
            session.showCursors(!session.cursors);
        }
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::Button("Export CSV")) {
            exportCsv(session);
        }
        ImGui::SetItemTooltip("Raw samples of the visible window, into the current directory");
        ImGui::EndDisabled();
        if (!exportMessage_.empty()) {
            ImGui::SameLine(0.0F, px(10.0F));
            widgets::Mono(exportFailed_ ? c::error : c::t2, "%s", exportMessage_.c_str());
        }
        (void)context;
    }

    void triggerPopup(ScopeSession& session) {
        if (!ImGui::BeginPopup("##trigger")) {
            return;
        }
        const std::vector<ScopeTrace>& traces = session.traces();
        triggerTrace_ = std::clamp(triggerTrace_, 0, static_cast<int>(traces.size()) - 1);
        widgets::Caption("Pause once when a signal crosses a level");
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(px(280.0F));
        if (ImGui::BeginCombo("##signal",
                              traces[static_cast<std::size_t>(triggerTrace_)].label.c_str())) {
            for (int i = 0; i < static_cast<int>(traces.size()); ++i) {
                if (ImGui::Selectable(traces[static_cast<std::size_t>(i)].label.c_str(),
                                      i == triggerTrace_)) {
                    triggerTrace_ = i;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopFont();
        widgets::Segmented("##edge", kEdges.data(), 2, &triggerEdge_);
        ImGui::SameLine(0.0F, px(10.0F));
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("level");
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::PushFont(theme::fonts().mono, theme::kMonoSize);
        ImGui::SetNextItemWidth(px(90.0F));
        ImGui::InputDouble("##level", &triggerLevel_, 0.0, 0.0, "%g");
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::TextDisabled("%s", traces[static_cast<std::size_t>(triggerTrace_)].unit.c_str());
        ImGui::PopFont();
        if (widgets::PrimaryButton("Arm")) {
            session.arm(triggerTrace_, triggerEdge_ == 1, triggerLevel_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!session.trigger().armed);
        if (widgets::Button("Disarm")) {
            session.disarm();
        }
        ImGui::EndDisabled();
        ImGui::EndPopup();
    }

    void exportCsv(const ScopeSession& session) {
        const ScopeSession::View view = session.view();
        char name[64];
        std::snprintf(name, sizeof(name), "scope_%.3fs_%.3fs.csv",
                      static_cast<double>(view.beginNs) * 1e-9,
                      static_cast<double>(view.endNs) * 1e-9);
        std::error_code ec;
        const std::filesystem::path path = std::filesystem::current_path(ec) / name;
        std::ofstream file(path);
        const std::size_t rows = file ? writeCsv(file, session, view) : 0;
        file.flush();
        exportFailed_ = !file;
        char text[256];
        if (exportFailed_) {
            std::snprintf(text, sizeof(text), "cannot write %s", path.c_str());
        } else {
            std::snprintf(text, sizeof(text), "%zu samples -> %s", rows, path.c_str());
        }
        exportMessage_ = text;
    }

    ScopePlotter plotter_;
    std::size_t drawn_{0};
    int triggerTrace_{0};
    int triggerEdge_{0};
    double triggerLevel_{0.0};
    std::string exportMessage_;
    bool exportFailed_{false};
};

}  // namespace

std::unique_ptr<Panel> createScopePanel() {
    return std::make_unique<ScopePanel>();
}

}  // namespace lagrange::viewer
