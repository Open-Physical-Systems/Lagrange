// The Builder's power-system forms (RFC 0016 phase 3; docs/design/ui/12-builder/SPEC.md
// "Properties", proposed there as "Power"): a robot's batteries, buses, loads, converters and
// branches (resistors, fuses, switches, diodes; RFC 0010) in its Properties, a joint's motor in
// the joint's. The fields are formats.md §9's overlay keys, as
// schemas/electrical/schema.usda declares them (builder::batteryFields() ...): the schema's
// units, its defaults in the unit slot, required ones marked. Every change is one undo step;
// values out of range are kept and listed in Problems, as the loader would refuse them.

#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/Overlay.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <functional>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using builder::OverlayEntry;
using builder::OverlayField;
using builder::OverlaySchema;
using widgets::px;

[[nodiscard]] ImVec4 electricalColor() {
    return widgets::domainColor("electrical");
}

/// The label of a field in the 112 px column: the attribute without its schema's namespace
/// ("elec:battery:capacity" → "capacity").
std::string shortLabel(std::string_view name) {
    const std::size_t colon = name.rfind(':');
    return std::string(colon == std::string_view::npos ? name : name.substr(colon + 1));
}

/// Edits the entry that makes or names `prim` (entries move when one is removed: never by
/// index across frames) and commits one step.
void editEntry(BuilderSession& session, const std::string& prim, const std::string& label,
               const std::string& key, const std::function<void(Document&, OverlayEntry&)>& f) {
    Document next = session.document();
    for (OverlayEntry& entry : next.overlay.entries) {
        if (entry.sensor.empty() && builder::entryPrim(entry) == prim) {
            f(next, entry);
            commit(session, std::move(next), label, key);
            return;
        }
    }
}

void markRefused(float right, const std::string& why) {
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max(right, ImGui::GetItemRectMax().y);
    ImGui::GetWindowDrawList()->AddRect(min, max, ImGui::GetColorU32(c::error), px(3.0F), 0,
                                        px(1.0F));
    ImGui::SetItemTooltip("%s", why.c_str());
}

/// The rows of one schema instance: each field of `fields` but the relationships, named with
/// `prefix` (a port's "elec:port:dc_out:"). Returns how many differ from their defaults.
/// `shown` (if given) leaves out the fields that do not apply (a branch's kind), unless set.
int fieldRows(BuilderSession& session, const OverlayEntry& entry, const OverlaySchema& schema,
              std::span<const OverlayField> fields, const std::string& prefix,
              const std::function<bool(const OverlayField&)>& applies = {}) {
    const std::string prim = builder::entryPrim(entry);
    const std::string who = builder::entryName(entry);
    int differing = 0;
    for (const OverlayField& field : fields) {
        if (field.type == OverlayField::Type::Rel || field.type == OverlayField::Type::Asset) {
            continue;
        }
        const std::string name = prefix + std::string(field.name);
        const std::string value = builder::attributeText(schema, name);
        if (applies && !applies(field) && value.empty()) {
            continue;
        }
        ImGui::PushID(name.c_str());
        const std::string shown = value.empty() ? std::string(field.fallback) : value;
        const auto number = builder::parseNumber(value);
        const bool sameNumber = number && number == builder::parseNumber(field.fallback);
        const bool differs =
            !value.empty() && !field.fallback.empty() && value != field.fallback && !sameNumber;
        differing += differs ? 1 : 0;
        std::string unit = field.unit == "1" ? std::string{} : builder::displayUnit(field.unit);
        if (differs) {
            unit +=
                std::string(unit.empty() ? "" : " · ") + "default " + std::string(field.fallback);
        } else if (field.required && value.empty()) {
            unit += std::string(unit.empty() ? "" : " · ") + "required";
        }
        const std::string refused = builder::checkField(field, value);
        const std::string tooltip = name + "\n" + std::string(field.meaning);
        const std::string label = shortLabel(field.name);
        const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
        const auto apply = [&](const std::string& typed) {
            const std::string from = value.empty() ? "unset" : value;
            const std::string unitText = field.unit.empty() || field.unit == "1"
                                             ? ""
                                             : " " + builder::displayUnit(field.unit);
            editEntry(session, prim,
                      "Set " + who + " " + label + " " + from + " → " +
                          (typed.empty() ? std::string("unset") : typed) + unitText,
                      "overlay/" + prim + "/" + name, [&](Document&, OverlayEntry& target) {
                          OverlaySchema* s = builder::findSchema(target, schema.name);
                          if (s != nullptr) {
                              builder::setAttribute(*s, &field, name, typed);
                          }
                      });
        };
        if (field.type == OverlayField::Type::Token) {
            if (widgets::DropdownRow(label.c_str(), shown.c_str(), nullptr)) {
                ImGui::OpenPopup("##tokens");
            }
            ImGui::SetItemTooltip("%s", tooltip.c_str());
            if (ImGui::BeginPopup("##tokens")) {
                for (const std::string_view token : field.tokens) {
                    const std::string option(token);
                    if (ImGui::Selectable(option.c_str(), option == shown)) {
                        apply(option == field.fallback ? std::string{} : option);
                    }
                }
                ImGui::EndPopup();
            }
        } else {
            std::string typed = shown;
            const bool numeric = field.type != OverlayField::Type::RealArray;
            const bool committed =
                widgets::CommitEdit(label.c_str(), &typed, unit.c_str(), differs && refused.empty(),
                                    tooltip.c_str(), numeric, true);
            if (!refused.empty()) {
                markRefused(right, refused);
            }
            if (committed) {
                // A number keeps the server's form ("−2" is "-2"); words are refused here, a
                // value out of range is kept and listed in Problems (the loader's judgement).
                std::string entered = typed;
                const bool cleared = entered.find_first_not_of(' ') == std::string::npos;
                if (cleared) {
                    entered.clear();
                } else if (numeric) {
                    const auto number = builder::parseNumber(entered);
                    entered = number ? builder::formatNumber(*number, 15) : std::string{};
                }
                if (cleared || !entered.empty() || !numeric) {
                    // Typing the default back leaves the file without it.
                    if (!field.fallback.empty() && builder::parseNumber(entered) &&
                        builder::parseNumber(entered) == builder::parseNumber(field.fallback)) {
                        entered.clear();
                    }
                    if (entered != value) {
                        apply(entered);
                    }
                }
            }
        }
        ImGui::PopID();
    }
    return differing;
}

/// The buses of the overlay, by prim ("power/dc_48v"), for the dropdowns.
std::vector<std::string> busPrims(const Document& document) {
    std::vector<std::string> out;
    for (const std::size_t i : builder::entriesWith(document, builder::kBusSchema)) {
        out.push_back(builder::entryPrim(document.overlay.entries[i]));
    }
    return out;
}

/// A dropdown row choosing a bus for a relationship; returns the chosen prim when changed.
std::optional<std::string> busRow(const char* label, const Document& document,
                                  const std::string& current, const char* tooltip) {
    const int bus = builder::busNamed(document, current);
    const std::string shown =
        bus >= 0 ? builder::entryName(document.overlay.entries[static_cast<std::size_t>(bus)])
        : current.empty() ? std::string("none")
                          : current + " (missing)";
    std::optional<std::string> chosen;
    if (widgets::DropdownRow(label, shown.c_str(), bus >= 0 ? "bus" : nullptr)) {
        ImGui::OpenPopup("##bus");
    }
    ImGui::SetItemTooltip("%s", tooltip);
    if (bus < 0) {
        const float right = ImGui::GetItemRectMax().x;
        markRefused(right, "Pick a bus: Add bus in the robot's Power section");
    }
    if (ImGui::BeginPopup("##bus")) {
        const std::vector<std::string> buses = busPrims(document);
        for (const std::string& prim : buses) {
            if (ImGui::Selectable(
                    prim.c_str(),
                    bus >= 0 &&
                        builder::entryPrim(
                            document.overlay.entries[static_cast<std::size_t>(bus)]) == prim)) {
                chosen = prim;
            }
        }
        if (buses.empty()) {
            ImGui::BeginDisabled();
            ImGui::Selectable("no bus yet: add one on the robot", false);
            ImGui::EndDisabled();
        }
        ImGui::EndPopup();
    }
    return chosen;
}

/// "Add …" with a menu: the samples' values, or empty fields (the required ones then show).
const builder::PowerPreset* presetMenu(const char* id,
                                       std::span<const builder::PowerPreset> presets,
                                       bool& picked) {
    picked = false;
    const builder::PowerPreset* chosen = nullptr;
    if (ImGui::BeginPopup(id)) {
        widgets::Caption("Start from");
        for (const builder::PowerPreset& preset : presets) {
            const std::string label(preset.label);
            if (ImGui::Selectable(label.c_str())) {
                chosen = &preset;
                picked = true;
            }
            ImGui::SetItemTooltip("Values of %s (illustrative, not a real product)",
                                  std::string(preset.source).c_str());
        }
        if (ImGui::Selectable("empty fields")) {
            picked = true;
        }
        ImGui::SetItemTooltip("Every required field empty: Problems lists them until typed");
        ImGui::EndPopup();
    }
    return chosen;
}

void removeButton(BuilderSession& session, const std::string& prim, const std::string& what) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
    const std::string label = "Remove " + what + "##" + prim;
    if (widgets::Button(label.c_str(), widgets::ButtonKind::Ghost, widgets::Icon::Delete)) {
        Document next = session.document();
        for (std::size_t i = 0; i < next.overlay.entries.size(); ++i) {
            if (next.overlay.entries[i].sensor.empty() &&
                builder::entryPrim(next.overlay.entries[i]) == prim) {
                (void)builder::removeEntry(next, i);
                commit(session, std::move(next), "Remove " + what + " " + prim);
                break;
            }
        }
    }
}

void busSection(BuilderSession& session, const OverlayEntry& entry) {
    const std::string prim = builder::entryPrim(entry);
    // Who feeds it and who draws from it, read-only (the forms of each say it). Worked out
    // before any field commits: the document is replaced then.
    std::string feeds;
    std::string draws;
    {
        const Document& document = session.document();
        const auto busPrimOf = [&](const std::string& target) {
            const int bus = builder::busNamed(document, target);
            return bus < 0 ? std::string{}
                           : builder::entryPrim(
                                 document.overlay.entries[static_cast<std::size_t>(bus)]);
        };
        const auto list = [](std::string& text, const std::string& name) {
            text += (text.empty() ? "" : ", ") + name;
        };
        for (const OverlayEntry& other : document.overlay.entries) {
            const std::string who = builder::entryName(other);
            for (const std::string_view schema : {builder::kMotorSchema, builder::kLoadSchema}) {
                if (const OverlaySchema* part = builder::findSchema(other, schema);
                    part != nullptr &&
                    busPrimOf(builder::attributeText(*part, "elec:powerPort")) == prim) {
                    list(draws, who);
                }
            }
            if (const OverlaySchema* converter =
                    builder::findSchema(other, builder::kConverterSchema)) {
                if (busPrimOf(builder::attributeText(*converter, "elec:conv:input")) == prim) {
                    list(draws, who);
                }
                if (busPrimOf(builder::attributeText(*converter, "elec:conv:output")) == prim) {
                    list(feeds, who);
                }
            }
            if (const OverlaySchema* branch = builder::findSchema(other, builder::kBranchSchema)) {
                if (busPrimOf(builder::attributeText(*branch, "elec:branch:input")) == prim ||
                    busPrimOf(builder::attributeText(*branch, "elec:branch:output")) == prim) {
                    list(feeds, who + " (branch)");
                }
            }
            if (const OverlaySchema* port = builder::portOf(other);
                port != nullptr && builder::findSchema(other, builder::kBatterySchema) != nullptr &&
                busPrimOf(builder::attributeText(
                    *port, builder::portPrefix(*port) + "connectedTo")) == prim) {
                list(feeds, who);
            }
        }
    }
    const std::string title = "Bus · " + builder::entryName(entry);
    const ImVec4 dot = electricalColor();
    ImGui::PushID(prim.c_str());
    if (widgets::SectionHeader(title.c_str(), "ElectricalBusAPI_1", true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                             "Signals bus/<name>/voltage and brownout when tested");
        widgets::PropertyRow("fed by", feeds.empty() ? "nothing" : feeds.c_str(), nullptr);
        widgets::PropertyRow("drawn by", draws.empty() ? "nothing" : draws.c_str(), nullptr, false,
                             "Motors, loads and converter inputs on this bus");
        (void)fieldRows(session, entry, *builder::findSchema(entry, builder::kBusSchema),
                        builder::busFields(), "");
        removeButton(session, prim, "bus");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    ImGui::PopID();
}

void batterySection(BuilderSession& session, const OverlayEntry& entry) {
    const std::string prim = builder::entryPrim(entry);
    const std::string title = "Battery · " + builder::entryName(entry);
    const ImVec4 dot = electricalColor();
    ImGui::PushID(prim.c_str());
    if (widgets::SectionHeader(title.c_str(), "BatteryAPI_1", true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                             "Signals battery/<name>/soc, terminal_voltage, current when tested");
        const OverlaySchema* port = builder::portOf(entry);
        const std::string portName =
            port != nullptr ? port->name : std::string(builder::kBatteryPort);
        const std::string prefix =
            port != nullptr ? builder::portPrefix(*port) : std::string(builder::kBatteryPortPrefix);
        const std::string connected =
            port != nullptr ? builder::attributeText(*port, prefix + "connectedTo") : std::string{};
        if (const auto chosen = busRow("output to", session.document(), connected,
                                       (portName + " connectedTo: the bus it feeds").c_str())) {
            editEntry(session, prim, "Connect " + builder::entryName(entry) + " to " + *chosen, {},
                      [&](Document&, OverlayEntry& target) {
                          OverlaySchema* p = builder::portOf(target);
                          if (p == nullptr) {
                              target.schemas.push_back({portName, {}});
                              p = &target.schemas.back();
                          }
                          builder::setAttribute(*p, nullptr, prefix + "connectedTo", *chosen);
                      });
        }
        (void)fieldRows(session, entry, *builder::findSchema(entry, builder::kBatterySchema),
                        builder::batteryFields(), "");
        if (port != nullptr) {
            ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
            ImGui::TextColored(c::t3, "%s", portName.c_str());
            ImGui::PopFont();
            (void)fieldRows(session, entry, *port, builder::portFields(), prefix);
        }
        removeButton(session, prim, "battery");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    ImGui::PopID();
}

/// A relationship row of a part: a bus dropdown that sets `attribute` of `schema`.
void relationRow(BuilderSession& session, const OverlayEntry& entry, std::string_view schema,
                 std::string_view attribute, const char* label) {
    const OverlaySchema* instance = builder::findSchema(entry, schema);
    const OverlayField* field = builder::overlayField(schema, attribute);
    if (instance == nullptr || field == nullptr) {
        return;
    }
    const std::string prim = builder::entryPrim(entry);
    const std::string name(attribute);
    const std::string tooltip = name + ": " + std::string(field->meaning);
    ImGui::PushID(name.c_str());
    if (const auto chosen = busRow(label, session.document(),
                                   builder::attributeText(*instance, name), tooltip.c_str())) {
        editEntry(session, prim,
                  "Connect " + builder::entryName(entry) + " " + label + " to " + *chosen, {},
                  [&](Document&, OverlayEntry& target) {
                      if (OverlaySchema* s = builder::findSchema(target, schema)) {
                          builder::setAttribute(*s, field, name, *chosen);
                      }
                  });
    }
    ImGui::PopID();
}

/// The run's signals of a part, read-only (electrical.md §3; the network's names).
void signalRows(const OverlayEntry& entry) {
    bool first = true;
    for (const auto& [signal, unit] : builder::powerSignals(entry)) {
        ImGui::PushID(signal.c_str());
        const std::string shown = unit == "1" ? std::string{} : unit;
        widgets::PropertyRow(first ? "signals" : "", signal.c_str(),
                             shown.empty() ? nullptr : shown.c_str(), false,
                             "Written by the electrical network while tested");
        ImGui::PopID();
        first = false;
    }
}

void loadSection(BuilderSession& session, const OverlayEntry& entry) {
    const std::string prim = builder::entryPrim(entry);
    const std::string title = "Load · " + builder::entryName(entry);
    const ImVec4 dot = electricalColor();
    ImGui::PushID(prim.c_str());
    if (widgets::SectionHeader(title.c_str(), "ElectricalLoadAPI_1", true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                             "A computer, a sensor, a heater: what it draws from its bus");
        relationRow(session, entry, builder::kLoadSchema, "elec:powerPort", "bus");
        const OverlaySchema& load = *builder::findSchema(entry, builder::kLoadSchema);
        const bool resistive = builder::attributeText(load, "elec:load:model") == "resistive";
        (void)fieldRows(session, entry, load, builder::loadFields(), "",
                        [resistive](const OverlayField& field) {
                            return resistive || field.name != "elec:load:resistance";
                        });
        signalRows(entry);
        removeButton(session, prim, "load");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    ImGui::PopID();
}

void converterSection(BuilderSession& session, const OverlayEntry& entry) {
    const std::string prim = builder::entryPrim(entry);
    const std::string title = "Converter · " + builder::entryName(entry);
    const ImVec4 dot = electricalColor();
    ImGui::PushID(prim.c_str());
    if (widgets::SectionHeader(title.c_str(), "ElectricalConverterAPI_1", true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                             "A DC-DC converter: an averaged regulator from one bus to another");
        relationRow(session, entry, builder::kConverterSchema, "elec:conv:input", "input");
        relationRow(session, entry, builder::kConverterSchema, "elec:conv:output", "output");
        (void)fieldRows(session, entry, *builder::findSchema(entry, builder::kConverterSchema),
                        builder::converterFields(), "");
        signalRows(entry);
        removeButton(session, prim, "converter");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    ImGui::PopID();
}

void branchSection(BuilderSession& session, const OverlayEntry& entry) {
    const std::string prim = builder::entryPrim(entry);
    const OverlaySchema& branch = *builder::findSchema(entry, builder::kBranchSchema);
    std::string kind = builder::attributeText(branch, "elec:branch:kind");
    kind = kind.empty() ? "resistor" : kind;
    std::string title = kind + " · " + builder::entryName(entry);
    title[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(title[0])));
    const ImVec4 dot = electricalColor();
    ImGui::PushID(prim.c_str());
    if (widgets::SectionHeader(title.c_str(), "ElectricalBranchAPI_1", true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                             "A two-terminal part between two buses (RFC 0010)");
        // Its kind first (it decides the rest), then its two ends, then the values.
        const std::span<const OverlayField> fields = builder::branchFields();
        (void)fieldRows(session, entry, branch, fields.first(1), "");
        relationRow(session, entry, builder::kBranchSchema, "elec:branch:input",
                    kind == "diode" ? "anode" : "input");
        relationRow(session, entry, builder::kBranchSchema, "elec:branch:output",
                    kind == "diode" ? "cathode" : "output");
        (void)fieldRows(session, entry, branch, fields.subspan(1), "",
                        [&kind](const OverlayField& field) {
                            return builder::branchFieldApplies(kind, field.name);
                        });
        signalRows(entry);
        removeButton(session, prim, kind);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    ImGui::PopID();
}

/// The bus a new part starts on: the first a battery feeds (else the first bus), and for its
/// other end the first other bus that no battery and no converter feeds (else another bus).
std::pair<int, int> defaultEnds(const Document& document) {
    const auto buses = builder::entriesWith(document, builder::kBusSchema);
    std::set<int> fedByBattery;
    std::set<int> fedAtAll;
    for (const OverlayEntry& entry : document.overlay.entries) {
        if (const OverlaySchema* port = builder::portOf(entry);
            port != nullptr && builder::findSchema(entry, builder::kBatterySchema) != nullptr) {
            const int bus = builder::busNamed(
                document,
                builder::attributeText(*port, builder::portPrefix(*port) + "connectedTo"));
            fedByBattery.insert(bus);
            fedAtAll.insert(bus);
        }
        if (const OverlaySchema* c = builder::findSchema(entry, builder::kConverterSchema)) {
            fedAtAll.insert(
                builder::busNamed(document, builder::attributeText(*c, "elec:conv:output")));
        }
    }
    int input = buses.empty() ? -1 : static_cast<int>(buses.front());
    for (const std::size_t b : buses) {
        if (fedByBattery.contains(static_cast<int>(b))) {
            input = static_cast<int>(b);
            break;
        }
    }
    int output = -1;
    for (const std::size_t b : buses) {
        if (static_cast<int>(b) != input && !fedAtAll.contains(static_cast<int>(b))) {
            output = static_cast<int>(b);
            break;
        }
    }
    for (const std::size_t b : buses) {
        if (output < 0 && static_cast<int>(b) != input) {
            output = static_cast<int>(b);
        }
    }
    return {input, output};
}

/// Other entries of the overlay (thermal nodes, rotors, wheels): kept as written.
void keptEntries(const Document& document) {
    std::vector<const OverlayEntry*> kept;
    for (const OverlayEntry& entry : document.overlay.entries) {
        if (entry.sensor.empty() &&
            builder::findSchema(entry, builder::kBatterySchema) == nullptr &&
            builder::findSchema(entry, builder::kBusSchema) == nullptr &&
            builder::findSchema(entry, builder::kMotorSchema) == nullptr &&
            builder::findSchema(entry, builder::kLoadSchema) == nullptr &&
            builder::findSchema(entry, builder::kConverterSchema) == nullptr &&
            builder::findSchema(entry, builder::kBranchSchema) == nullptr) {
            kept.push_back(&entry);
        }
    }
    if (kept.empty()) {
        return;
    }
    const std::string meta = std::to_string(kept.size()) + " entries · read-only";
    if (!widgets::SectionHeader("Overlay, kept as written", meta.c_str(), false)) {
        return;
    }
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    for (const OverlayEntry* entry : kept) {
        std::string schemas;
        for (const OverlaySchema& schema : entry->schemas) {
            schemas += (schemas.empty() ? "" : " · ") + schema.name;
        }
        const std::string prim = builder::entryPrim(*entry);
        ImGui::PushID(prim.c_str());
        widgets::PropertyRow(builder::entryName(*entry).c_str(), schemas.c_str(), nullptr, false,
                             prim.c_str());
        ImGui::PopID();
    }
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
}

}  // namespace

void robotPowerSections(BuilderSession& session) {
    // A copy: a form's edit replaces the document (History keeps its states in a vector).
    const Document document = session.document();
    const auto batteries = builder::entriesWith(document, builder::kBatterySchema);
    const auto buses = builder::entriesWith(document, builder::kBusSchema);
    const auto motors = builder::entriesWith(document, builder::kMotorSchema);
    const auto loads = builder::entriesWith(document, builder::kLoadSchema);
    const auto converters = builder::entriesWith(document, builder::kConverterSchema);
    const auto branches = builder::entriesWith(document, builder::kBranchSchema);
    char counts[96];
    std::snprintf(counts, sizeof(counts), "overlay · %zu battery · %zu bus · %zu motors",
                  batteries.size(), buses.size(), motors.size());
    std::string meta = counts;
    const std::size_t parts = loads.size() + converters.size() + branches.size();
    if (parts > 0) {
        meta += " · " + std::to_string(parts) + " parts";
    }
    const ImVec4 dot = electricalColor();
    if (widgets::SectionHeader("Power", meta.c_str(), true, &dot)) {
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        if (batteries.empty() && buses.empty() && motors.empty()) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption(
                "No power system: a test runs the joints on ideal torque (the electrical "
                "domain's substitute). Add a bus, then a battery to feed it, then a motor on "
                "each driven joint (its Properties).");
            ImGui::PopTextWrapPos();
        }
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        if (widgets::Button("Add bus", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-bus");
        }
        ImGui::SetItemTooltip("A DC bus, power/<name> in the overlay (ElectricalBusAPI_1)");
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::Button("Add battery", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-battery");
        }
        ImGui::SetItemTooltip("A battery feeding the first bus (BatteryAPI_1, PortAPI_1:dc_out)");
        bool picked = false;
        if (const builder::PowerPreset* preset =
                presetMenu("##add-bus", builder::busPresets(), picked);
            picked) {
            Document next = document;
            const std::string name = builder::addBus(next, "dc_bus", preset);
            commit(session, std::move(next), "Add bus " + name);
        }
        if (const builder::PowerPreset* preset =
                presetMenu("##add-battery", builder::batteryPresets(), picked);
            picked) {
            Document next = document;
            const std::string name = builder::addBattery(next, "battery", -1, preset);
            commit(session, std::move(next), "Add battery " + name);
        }
        // The parts between buses (RFC 0010): a load on a bus, a converter or a branch from one
        // bus to another.
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        if (widgets::Button("Add load", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-load");
        }
        ImGui::SetItemTooltip(
            "A load on the first bus: a computer, a sensor, a heater "
            "(ElectricalLoadAPI_1)");
        ImGui::SameLine(0.0F, px(6.0F));
        if (widgets::Button("Add converter", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            const auto [input, output] = defaultEnds(document);
            Document next = document;
            const std::string name = builder::addConverter(next, "dcdc", input, output);
            commit(session, std::move(next), "Add converter " + name);
        }
        ImGui::SetItemTooltip(
            "A DC-DC converter from a battery's bus to another bus "
            "(ElectricalConverterAPI_1): type its voltage and current limit");
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        if (widgets::Button("Add fuse, switch…", widgets::ButtonKind::Default,
                            widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-branch");
        }
        ImGui::SetItemTooltip(
            "A part between two buses: a resistor, a fuse, a switch or a "
            "diode (ElectricalBranchAPI_1)");
        if (const builder::PowerPreset* preset =
                presetMenu("##add-load", builder::loadPresets(), picked);
            picked) {
            Document next = document;
            const std::string name = builder::addLoad(next, "load", -1, preset);
            commit(session, std::move(next), "Add load " + name);
        }
        if (ImGui::BeginPopup("##add-branch")) {
            widgets::Caption("Between two buses");
            for (const char* kind : {"resistor", "fuse", "switch", "diode"}) {
                if (ImGui::Selectable(kind)) {
                    const auto [input, output] = defaultEnds(document);
                    Document next = document;
                    const std::string name = builder::addBranch(next, "", kind, input, output);
                    commit(session, std::move(next), "Add " + std::string(kind) + " " + name);
                }
            }
            ImGui::EndPopup();
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }
    const std::vector<OverlayEntry>& entries = document.overlay.entries;
    for (const std::size_t i : buses) {
        busSection(session, entries[i]);
    }
    for (const std::size_t i : batteries) {
        batterySection(session, entries[i]);
    }
    for (const std::size_t i : converters) {
        converterSection(session, entries[i]);
    }
    for (const std::size_t i : branches) {
        branchSection(session, entries[i]);
    }
    for (const std::size_t i : loads) {
        loadSection(session, entries[i]);
    }
    keptEntries(document);
}

void jointMotorSection(BuilderSession& session, const std::string& jointName) {
    const builder::Joint* found = builder::findJoint(session.document(), jointName);
    if (found == nullptr) {
        return;
    }
    const builder::Joint joint = *found;
    const Document& document = session.document();
    const int index = builder::motorOf(document, joint.name);
    const ImVec4 dot = electricalColor();
    if (!widgets::SectionHeader("Motor", index >= 0 ? "ElectricalMotorAPI_1 · overlay" : "none",
                                true, &dot)) {
        return;
    }
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    if (index < 0) {
        const bool fixed = joint.type == builder::Joint::Type::Fixed;
        ImGui::PushTextWrapPos(0.0F);
        widgets::Caption(fixed ? "A fixed joint does not turn: no motor."
                               : "No motor: tested, this joint takes torque as it is commanded "
                                 "(ideal). A motor draws its current from a bus.");
        ImGui::PopTextWrapPos();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        ImGui::BeginDisabled(fixed);
        if (widgets::Button("Add a motor", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            ImGui::OpenPopup("##add-motor");
        }
        ImGui::EndDisabled();
        ImGui::SetItemTooltip(fixed ? "Make the joint revolute, continuous or prismatic first"
                                    : "ElectricalMotorAPI_1 on %s, drawing from the first bus",
                              joint.name.c_str());
        bool picked = false;
        if (const builder::PowerPreset* preset =
                presetMenu("##add-motor", builder::motorPresets(), picked);
            picked) {
            Document next = document;
            const std::string name = builder::addMotor(next, joint.name, -1, preset);
            if (!name.empty()) {
                commit(session, std::move(next), "Add motor " + name + " on " + joint.name);
            }
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        return;
    }
    const OverlayEntry entry = document.overlay.entries[static_cast<std::size_t>(index)];
    const std::string prim = builder::entryPrim(entry);
    const std::string name = builder::entryName(entry);
    widgets::PropertyRow("prim", prim.c_str(), nullptr, false,
                         "create_child of the joint's entry in the overlay");
    const OverlaySchema& motor = *builder::findSchema(entry, builder::kMotorSchema);
    if (const auto chosen =
            busRow("bus", session.document(), builder::attributeText(motor, "elec:powerPort"),
                   "elec:powerPort: the bus the drive draws from")) {
        editEntry(session, prim, "Connect " + name + " to " + *chosen, {},
                  [&](Document&, OverlayEntry& target) {
                      builder::setAttribute(
                          *builder::findSchema(target, builder::kMotorSchema),
                          builder::overlayField(builder::kMotorSchema, "elec:powerPort"),
                          "elec:powerPort", *chosen);
                  });
    }
    (void)fieldRows(session, entry, motor, builder::motorFields(), "");
    // What the run will have (electrical.md §3), read-only.
    bool first = true;
    for (const std::string& signal :
         {"motor/" + name + "/current_q", "motor/" + name + "/bus_current",
          "joint/" + joint.name + "/torque"}) {
        ImGui::PushID(signal.c_str());
        widgets::PropertyRow(first ? "signals" : "", signal.c_str(),
                             signal.ends_with("torque") ? "N·m" : "A");
        ImGui::PopID();
        first = false;
    }
    removeButton(session, prim, "motor");
    ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
}

}  // namespace lagrange::viewer::builder_ui
