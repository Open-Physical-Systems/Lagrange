// The Netlist tab (docs/design/ui/04-electrical/SPEC.md, Netlist): a circuit edited as parts on
// buses, drawn as the Circuit tab draws it, and written as a netlist file. Only what the importer
// can simulate can be drawn here, so there are no free wires to get wrong. It writes a file on this
// machine and changes nothing in the running simulation.
#include "CircuitDraw.hpp"
#include "lagrange/viewer/Circuit.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer {

namespace {

using widgets::px;

constexpr float kFormWidth = 300.0F;
constexpr float kTextHeight = 130.0F;
constexpr std::size_t kMaxRcPairs = 2;  // what a battery takes

class NetlistPanel final : public Panel {
public:
    explicit NetlistPanel(std::function<const Circuit*(PanelContext&)> running)
        : running_(std::move(running)) {
        const char* home = std::getenv("HOME");
        const std::string path =
            (home != nullptr ? std::string(home) + "/" : std::string{}) + "lagrange_circuit.cir";
        std::snprintf(path_.data(), path_.size(), "%s", path.c_str());
    }
    [[nodiscard]] const char* title() const override { return "Netlist"; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Electrical; }

    void draw(PanelContext& context) override {
        exportOnRequest(context);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(px(8.0F), px(8.0F)));
        const bool open = ImGui::Begin(title());
        ImGui::PopStyleVar();
        if (!open) {
            ImGui::End();
            return;
        }
        // The first time, the robot's circuit as soon as it is whole: a blank is one click away.
        if (const Circuit* running = running_(context);
            !touched_ && running != nullptr && circuitProblems(*running).empty()) {
            draft_ = *running;
            touched_ = true;
            select(CircuitPick{0, std::nullopt});
        }
        tools(context);
        if (draft_.buses.empty()) {
            widgets::Caption(
                "Start from this robot's power system, or add a bus. A netlist saved here runs in "
                "ngspice and with --overlay FILE.cir.");
            ImGui::End();
            return;
        }
        const float bottom = px(kTextHeight) + px(64.0F);
        const float height = std::max(ImGui::GetContentRegionAvail().y - bottom, px(120.0F));
        ImGui::BeginChild("##form", ImVec2(px(kFormWidth), height));
        form();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("##drawing", ImVec2(0.0F, height), ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        const auto picked = drawCircuit(
            draft_, [](std::size_t, std::size_t) { return NAN; }, [](std::size_t) { return NAN; },
            selected_);
        if (picked) {
            select(*picked);
        }
        ImGui::EndChild();
        file();
        ImGui::End();
    }

private:
    void tools(PanelContext& context) {
        const Circuit* running = running_(context);
        ImGui::BeginDisabled(running == nullptr || running->buses.empty());
        if (widgets::Button("From this robot")) {
            draft_ = *running;
            touched_ = true;
            select(CircuitPick{0, std::nullopt});
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (widgets::Button("New")) {
            draft_ = Circuit{};
            touched_ = true;
            selected_.reset();
            fields_.clear();
        }
        ImGui::SameLine();
        if (widgets::Button("+ Bus")) {
            touched_ = true;
            CircuitBus bus;
            bus.name = uniqueName("dc");
            draft_.buses.push_back(std::move(bus));
            select(CircuitPick{draft_.buses.size() - 1, std::nullopt});
        }
        ImGui::BeginDisabled(!selected_);
        ImGui::SameLine();
        if (widgets::Button("+ Battery")) {
            addPart(CircuitPart::Kind::Battery, "pack");
        }
        ImGui::SameLine();
        if (widgets::Button("+ Resistor")) {
            addPart(CircuitPart::Kind::Resistor, "load");
        }
        ImGui::SameLine();
        if (widgets::Button("+ Power load")) {
            addPart(CircuitPart::Kind::PowerLoad, "power");
        }
        ImGui::SameLine();
        if (widgets::Button("+ Motor")) {
            addPart(CircuitPart::Kind::Motor, "motor");
        }
        ImGui::SameLine();
        if (widgets::Button("+ DC-DC")) {
            addPart(CircuitPart::Kind::Converter, "dcdc");
            // It feeds the selected bus, from the first other one until told.
            CircuitBus& bus = draft_.buses[selected_->bus];
            for (const CircuitBus& other : draft_.buses) {
                if (other.name != bus.name) {
                    bus.parts.back().inputBus = other.name;
                    break;
                }
            }
            bus.parts.back().efficiency = 0.9;
        }
        ImGui::SameLine();
        if (widgets::Button("+ Fuse")) {
            addBranch("fuse");
        }
        ImGui::SameLine();
        if (widgets::Button("+ Switch")) {
            addBranch("switch");
        }
        ImGui::SameLine();
        if (widgets::Button("+ Diode")) {
            addBranch("diode");
        }
        ImGui::SameLine();
        if (widgets::Button("Remove", widgets::ButtonKind::Ghost)) {
            remove();
        }
        ImGui::EndDisabled();
    }

    /// The selected part or bus's values.
    void form() {
        if (!selected_ || selected_->bus >= draft_.buses.size()) {
            widgets::Caption("Click a part in the drawing, or a bus's name.");
            return;
        }
        CircuitBus& bus = draft_.buses[selected_->bus];
        if (!selected_->part) {
            text("name", bus.name);
            number("undervoltage", bus.undervoltage, "V");
            number("overvoltage", bus.overvoltage, "V");
            number("capacitance", bus.capacitance, "F");
            return;
        }
        CircuitPart& part = bus.parts[*selected_->part];
        text("name", part.name);
        switch (part.kind) {
            case CircuitPart::Kind::Battery:
                number("voltage", part.voltage, "V");
                number("r0", part.r0, "ohm");
                number("capacity", part.capacity, "C");
                number("cutoff", part.cutoff, "V");
                number("max charge", part.maxCharge, "A");
                number("state of charge", part.soc, "");
                for (std::size_t i = 0; i < part.pairs.size(); ++i) {
                    const std::string n = std::to_string(i + 1);
                    number("R" + n, part.pairs[i].first, "ohm");
                    number("C" + n, part.pairs[i].second, "F");
                }
                if (part.pairs.size() < kMaxRcPairs && widgets::Button("+ RC pair")) {
                    part.pairs.emplace_back(0.0, 0.0);
                }
                if (!part.pairs.empty()) {
                    ImGui::SameLine();
                    if (widgets::Button("- RC pair", widgets::ButtonKind::Ghost)) {
                        part.pairs.pop_back();
                        fields_.clear();
                    }
                }
                break;
            case CircuitPart::Kind::Resistor:
                number("resistance", part.resistance, "ohm");
                break;
            case CircuitPart::Kind::PowerLoad:
                number("power", part.power, "W");
                number("off below", part.minVoltage, "V");
                break;
            case CircuitPart::Kind::Branch:
                if (part.inputBus.empty()) {
                    widgets::Caption("It leaves here. Its values are on the bus it feeds.");
                    break;
                }
                text("from bus", part.inputBus);
                number("resistance", part.resistance, "ohm");
                if (part.branchKind == "fuse") {
                    number("rating", part.i2t, "A2 s");
                } else if (part.branchKind == "diode") {
                    number("forward drop", part.forwardDrop, "V");
                }
                break;
            case CircuitPart::Kind::Converter:
                if (part.inputBus.empty() && part.outputVolts > 0.0) {
                    widgets::Caption("It draws here. Its values are on the bus it feeds.");
                    break;
                }
                text("draws from", part.inputBus);
                number("output", part.outputVolts, "V");
                number("max current", part.maxOutput, "A");
                number("efficiency", part.efficiency, "");
                break;
            case CircuitPart::Kind::Motor:
                text("joint", part.joint);
                number("R", part.windingResistance, "ohm");
                number("L", part.inductance, "H");
                number("Kt", part.torqueConstant, "N m/A");
                number("max current", part.maxCurrent, "A");
                number("gear", part.gear, "");
                number("efficiency", part.efficiency, "");
                number("rotor inertia", part.rotorInertia, "kg m2");
                break;
            default:
                widgets::Caption("A netlist cannot hold this part.");
                break;
        }
    }

    void file() {
        const std::vector<std::string> problems = circuitProblems(draft_);
        netlist_ = netlistOf(draft_, "Lagrange circuit");
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        ImGui::InputTextMultiline("##netlist", netlist_.data(), netlist_.size() + 1,
                                  ImVec2(-1.0F, px(kTextHeight)), ImGuiInputTextFlags_ReadOnly);
        ImGui::PopFont();
        widgets::PropertyEdit("file", path_.data(), path_.size(), "", false, nullptr, false);
        std::error_code ec;
        const bool exists = std::filesystem::exists(path_.data(), ec);
        ImGui::BeginDisabled(!problems.empty());
        if (widgets::PrimaryButton(exists ? "Overwrite" : "Save")) {
            save();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        if (!problems.empty()) {
            widgets::Pill("cannot simulate", widgets::PillKind::Warn);
            ImGui::SameLine();
            widgets::Caption("%s", problems.front().c_str());
        } else if (!message_.empty()) {
            widgets::Caption("%s", message_.c_str());
        }
    }

    void save() {
        std::ofstream out(path_.data());
        out << netlist_;
        out.close();
        message_ = out ? std::string("Saved. Run it with --overlay ") + path_.data()
                       : std::string("Could not write ") + path_.data();
    }

    /// --export-netlist: writes the running stage's circuit once it is whole.
    void exportOnRequest(PanelContext& context) {
        if (context.exportNetlist == nullptr || context.exportNetlist->empty()) {
            return;
        }
        const Circuit* running = running_(context);
        if (running == nullptr || !circuitProblems(*running).empty()) {
            return;
        }
        std::ofstream(*context.exportNetlist) << netlistOf(*running, "Lagrange circuit");
        context.exportNetlist->clear();
    }

    void select(const CircuitPick& pick) {
        selected_ = pick;
        fields_.clear();
        message_.clear();
    }

    void addPart(CircuitPart::Kind kind, const char* base) {
        CircuitBus& bus = draft_.buses[selected_->bus];
        CircuitPart part;
        part.kind = kind;
        part.name = uniqueName(base);
        bus.parts.push_back(std::move(part));
        select(CircuitPick{selected_->bus, bus.parts.size() - 1});
    }

    /// A part between buses, on the selected bus, from the first other one until told.
    void addBranch(const char* kind) {
        addPart(CircuitPart::Kind::Branch, kind);
        CircuitBus& bus = draft_.buses[selected_->bus];
        CircuitPart& part = bus.parts.back();
        part.branchKind = kind;
        part.detail = kind;
        for (const CircuitBus& other : draft_.buses) {
            if (other.name != bus.name) {
                part.inputBus = other.name;
                break;
            }
        }
    }

    void remove() {
        if (selected_->part) {
            std::vector<CircuitPart>& parts = draft_.buses[selected_->bus].parts;
            parts.erase(parts.begin() + static_cast<std::ptrdiff_t>(*selected_->part));
            select(CircuitPick{selected_->bus, std::nullopt});
        } else {
            draft_.buses.erase(draft_.buses.begin() + static_cast<std::ptrdiff_t>(selected_->bus));
            selected_.reset();
            fields_.clear();
        }
    }

    [[nodiscard]] std::string uniqueName(const std::string& base) const {
        const auto taken = [this](const std::string& name) {
            for (const CircuitBus& bus : draft_.buses) {
                if (bus.name == name ||
                    std::any_of(bus.parts.begin(), bus.parts.end(),
                                [&name](const CircuitPart& p) { return p.name == name; })) {
                    return true;
                }
            }
            return false;
        };
        std::string name = base;
        for (int i = 2; taken(name); ++i) {
            name = base + std::to_string(i);
        }
        return name;
    }

    /// A value's text while it is typed; dropped when the selection changes.
    std::array<char, 64>& field(const std::string& label, const std::string& initial) {
        const auto [entry, fresh] = fields_.try_emplace(label);
        if (fresh) {
            std::snprintf(entry->second.data(), entry->second.size(), "%s", initial.c_str());
        }
        return entry->second;
    }

    void number(const std::string& label, double& value, const char* unit) {
        std::array<char, 32> start{};
        std::snprintf(start.data(), start.size(), "%.10g", value);
        std::array<char, 64>& buffer = field(label, start.data());
        if (widgets::PropertyEdit(label.c_str(), buffer.data(), buffer.size(), unit, false)) {
            char* end = nullptr;
            const double typed = std::strtod(buffer.data(), &end);
            value = end != buffer.data() ? typed : 0.0;
        }
    }

    void text(const std::string& label, std::string& value) {
        std::array<char, 64>& buffer = field(label, value);
        if (widgets::PropertyEdit(label.c_str(), buffer.data(), buffer.size(), "", false, nullptr,
                                  false)) {
            value = buffer.data();
        }
    }

    std::function<const Circuit*(PanelContext&)> running_;
    Circuit draft_;
    std::optional<CircuitPick> selected_;
    bool touched_{false};  // the user has chosen what to edit
    std::map<std::string, std::array<char, 64>> fields_;
    std::array<char, 512> path_{};
    std::string netlist_;
    std::string message_;
};

}  // namespace

std::unique_ptr<Panel> createNetlistPanel(std::function<const Circuit*(PanelContext&)> running) {
    return std::make_unique<NetlistPanel>(std::move(running));
}

}  // namespace lagrange::viewer
