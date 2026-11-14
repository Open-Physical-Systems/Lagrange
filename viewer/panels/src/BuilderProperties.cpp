// The Builder's Properties and History tabs (docs/design/ui/12-builder/SPEC.md "Properties").
// Edits apply to the document at once (there is no Apply: nothing runs); every committed field
// is one undo step through History.
#include "BuilderCommon.hpp"
#include "lagrange/viewer/Fonts.hpp"
#include "lagrange/viewer/Panel.hpp"
#include "lagrange/viewer/Widgets.hpp"
#include "lagrange/viewer/builder/MassProperties.hpp"
#include "lagrange/viewer/builder/Materials.hpp"
#include "lagrange/viewer/lagrange_theme.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>

namespace lagrange::viewer::builder_ui {

namespace {

namespace c = theme::c;
using builder::Joint;
using builder::Link;
using builder::Sensor;
using builder::Shape;
using widgets::px;

/// The URDF reader's default update rate, Hz (server/src/Robot.cpp; Document.hpp's Sensor).
constexpr double kDefaultUpdateRate = 10.0;

/// Copies the document, lets `change` edit it and commits it as one step. `change` returns
/// false when nothing changed.
void edit(BuilderSession& session, const std::string& label, const std::string& key,
          const std::function<bool(Document&)>& change) {
    Document next = session.document();
    if (change(next)) {
        commit(session, std::move(next), label, key);
    }
}

std::string text(const std::optional<double>& value, double scale = 1.0) {
    return value ? builder::formatNumber(*value * scale) : std::string{};
}

/// A number field of the 112 px grid. Returns the committed value: nullopt when nothing was
/// committed; an empty optional inside when the field was cleared (and `optional` allows it).
std::optional<std::optional<double>> numberRow(const char* label,
                                               const std::optional<double>& value, const char* unit,
                                               double scale = 1.0,
                                               const char* defaultText = nullptr,
                                               bool optional = true, bool mono = false,
                                               const char* tooltip = nullptr) {
    std::string shown = text(value, scale);
    const bool differs =
        defaultText != nullptr && !shown.empty() && shown != std::string(defaultText);
    std::string unitText = unit != nullptr ? unit : "";
    if (differs) {
        unitText += std::string(unitText.empty() ? "" : " · ") + "default " + defaultText;
    }
    if (!widgets::CommitEdit(label, &shown, unitText.c_str(), differs, tooltip, true, mono)) {
        return std::nullopt;
    }
    if (shown.find_first_not_of(' ') == std::string::npos) {
        return optional ? std::optional<std::optional<double>>(std::optional<double>{})
                        : std::nullopt;
    }
    const auto parsed = builder::parseNumber(shown);
    if (!parsed) {
        return std::nullopt;  // not a number: the field shows the document's value again
    }
    return std::optional<double>(*parsed / scale);
}

/// A pose as two Vec3Edit rows, rotate in degrees or radians. Returns true when it changed.
bool poseRows(builder::Pose& pose, bool degrees) {
    bool changed = false;
    double xyz[3] = {pose.xyz[0], pose.xyz[1], pose.xyz[2]};
    if (widgets::Vec3Edit("translate", xyz, "%.3f") != 0) {
        pose.xyz = {xyz[0], xyz[1], xyz[2]};
        changed = true;
    }
    const double scale = degrees ? builder::toDegrees(1.0) : 1.0;
    double rpy[3] = {pose.rpy[0] * scale, pose.rpy[1] * scale, pose.rpy[2] * scale};
    if (widgets::Vec3Edit("rotate", rpy, degrees ? "%.1f" : "%.4f") != 0) {
        pose.rpy = {rpy[0] / scale, rpy[1] / scale, rpy[2] / scale};
        changed = true;
    }
    return changed;
}

/// The path through the tree: robot / base_link / ... / item (design: mono t3 11 px).
std::string treePath(const Document& document, std::string link) {
    std::vector<std::string> parts;
    for (int guard = 0; !link.empty() && guard < 256; ++guard) {
        parts.push_back(link);
        const auto joint =
            std::ranges::find_if(document.joints, [&](const Joint& j) { return j.child == link; });
        link = joint != document.joints.end() ? joint->parent : std::string{};
    }
    std::string out = document.robot;
    for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
        out += " / " + *it;
    }
    return out;
}

class PropertiesPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return builder_windows::kProperties; }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        if (!ImGui::Begin(title())) {
            ImGui::End();
            return;
        }
        BuilderSession& session = *context.builder;
        const Document& document = session.document();
        auto& selection = session.selection;
        // A selection whose item is gone (undo, delete) falls back to the robot.
        if ((selection.kind == Kind::Link &&
             builder::findLink(document, selection.name) == nullptr) ||
            (selection.kind == Kind::Joint &&
             builder::findJoint(document, selection.name) == nullptr) ||
            (selection.kind == Kind::Sensor &&
             builder::findSensor(document, selection.name) == nullptr)) {
            select(session, Kind::None, {});
        }
        tools(session);
        differing_ = 0;
        resetDefaults_ = nullptr;
        ImGui::BeginDisabled(!editable(session));
        const float footer = px(38.0F);
        ImGui::BeginChild("##properties", ImVec2(0.0F, -footer));
        switch (selection.kind) {
            case Kind::None:
            case Kind::Robot:
                robot(session);
                break;
            case Kind::Link:
                link(session, *builder::findLink(document, selection.name));
                break;
            case Kind::Joint:
                joint(session, *builder::findJoint(document, selection.name));
                break;
            case Kind::Sensor:
                sensor(session, *builder::findSensor(document, selection.name));
                break;
        }
        ImGui::EndChild();
        footerBar(session);
        ImGui::EndDisabled();
        ImGui::End();
    }

private:
    // --- header and tools -----------------------------------------------------------------

    static void tools(BuilderSession& session) {
        const Kind kind = session.selection.kind;
        const bool item = kind == Kind::Link || kind == Kind::Joint || kind == Kind::Sensor;
        if (!widgets::BeginPanelTools("##properties-tools", px(48.0F))) {
            return;
        }
        ImGui::BeginDisabled(!item || !editable(session) || kind == Kind::Joint);
        if (widgets::Button("##duplicate", widgets::ButtonKind::Ghost, widgets::Icon::Duplicate)) {
            duplicate(session);
        }
        ImGui::SetItemTooltip("Duplicate");
        ImGui::EndDisabled();
        ImGui::SameLine(0.0F, px(2.0F));
        ImGui::BeginDisabled(!item || !editable(session));
        if (widgets::Button("##delete", widgets::ButtonKind::Ghost, widgets::Icon::Delete)) {
            deleteSelection(session);
        }
        ImGui::SetItemTooltip("Delete (and everything under it)");
        ImGui::EndDisabled();
        widgets::EndPanelTools();
    }

    static void duplicate(BuilderSession& session) {
        Document next = session.document();
        const std::string name = session.selection.name;
        std::string copy;
        if (session.selection.kind == Kind::Sensor) {
            Sensor sensor = *builder::findSensor(next, name);
            sensor.name = copy = builder::uniqueName(next, name);
            next.sensors.push_back(std::move(sensor));
        } else if (session.selection.kind == Kind::Link) {
            Link link = *builder::findLink(next, name);
            link.name = copy = builder::uniqueName(next, name);
            const auto joint =
                std::ranges::find_if(next.joints, [&](const Joint& j) { return j.child == name; });
            if (joint == next.joints.end()) {
                return;  // the root is not duplicated: a robot has one
            }
            Joint twin = *joint;
            twin.child = copy;
            twin.name = builder::uniqueName(next, copy + "_joint");
            next.links.push_back(std::move(link));
            next.joints.push_back(std::move(twin));
        } else {
            return;
        }
        commit(session, std::move(next), "Duplicate " + name + " as " + copy);
        select(session, session.selection.kind, copy);
    }

    /// Icon, name (14 px semibold), what it is; the path; pills. Then the name field (F2).
    void header(BuilderSession& session, widgets::Icon icon, const std::string& name,
                const char* what, const std::string& path, builder::ItemKind kind) {
        const theme::Fonts& fonts = theme::fonts();
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        widgets::IconItem(icon, 16.0F, c::accent);
        ImGui::SameLine(0.0F, px(6.0F));
        ImGui::PushFont(fonts.sansSemiBold, 14.0F);
        ImGui::TextColored(c::t1, "%s", name.c_str());
        ImGui::PopFont();
        ImGui::SameLine(0.0F, px(6.0F));
        widgets::Caption("%s", what);
        ImGui::PushFont(fonts.mono, theme::kCaptionSize);
        ImGui::TextColored(c::t3, "%s", path.c_str());
        ImGui::PopFont();
        std::string renamed = name;
        if (widgets::CommitEdit("name", &renamed, nullptr, false,
                                "Letters, digits and _; references follow the new name", false)) {
            Document next = session.document();
            const std::string error = builder::rename(next, kind, name, renamed);
            if (error.empty()) {
                commit(session, std::move(next), "Rename " + name + " to " + renamed);
                if (kind != builder::ItemKind::Robot) {
                    session.selection.name = renamed;
                }
            } else {
                renameError_ = error;
            }
        }
        if (!renameError_.empty()) {
            widgets::Pill(renameError_.c_str(), widgets::PillKind::Error);
            if (ImGui::IsItemClicked()) {
                renameError_.clear();
            }
        }
    }

    // --- robot ----------------------------------------------------------------------------

    void robot(BuilderSession& session) {
        const Document& document = session.document();
        header(session, widgets::Icon::Scene, document.robot, "robot", document.robot,
               builder::ItemKind::Robot);
        const builder::PartCounts counts = builder::countParts(document, &session.catalog);
        if (widgets::SectionHeader("Parts", "")) {
            char value[48];
            std::snprintf(value, sizeof(value), "%d", counts.links);
            widgets::PropertyRow("links", value, counts.roots == 1 ? "1 root" : "roots ≠ 1");
            std::snprintf(value, sizeof(value), "%d", counts.joints);
            widgets::PropertyRow("joints", value, nullptr);
            std::snprintf(value, sizeof(value), "%d", counts.sensors);
            widgets::PropertyRow("sensors", value, nullptr);
            const builder::MassProperties mass = builder::robotMass(document);
            std::snprintf(value, sizeof(value), "%.3f", mass.mass);
            widgets::PropertyRow("mass", value, "kg", false,
                                 "From the links' shapes and densities (RFC 0016 §4)");
        }
        keptXml(document.keptXml);
        ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
        widgets::Caption("Select a link, joint or sensor in the tree to edit it.");
    }

    // --- link -----------------------------------------------------------------------------

    void link(BuilderSession& session, const Link& link) {
        const Document& document = session.document();
        header(session, widgets::Icon::Box, link.name, "link", treePath(document, link.name),
               builder::ItemKind::Link);
        char shapes[32];
        std::snprintf(shapes, sizeof(shapes), "%zu shape%s", link.shapes.size(),
                      link.shapes.size() == 1 ? "" : "s");
        widgets::Pill(shapes, widgets::PillKind::Neutral);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string& name = link.name;
        for (std::size_t i = 0; i < link.shapes.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            shape(session, link, i);
            ImGui::PopID();
        }
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(8.0F));
        if (widgets::Button("Add shape", widgets::ButtonKind::Default, widgets::Icon::Plus)) {
            edit(session, "Add a shape to " + name, {}, [&](Document& next) {
                builder::findLink(next, name)
                    ->shapes.push_back(builder::defaultShape(Shape::Kind::Box));
                return true;
            });
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        mass(session, link);
        keptXml(link.keptXml);
    }

    void shape(BuilderSession& session, const Link& link, std::size_t index) {
        const Shape& shape = link.shapes[index];
        const std::string name = link.name;
        const std::string title = "Shape " + std::to_string(index + 1) + " · " +
                                  std::string(builder::toString(shape.kind));
        if (!widgets::SectionHeader(title.c_str(), "m · deg")) {
            return;
        }
        const std::string key = name + "/shape" + std::to_string(index);
        const auto change = [&](const std::string& what, const std::function<void(Shape&)>& f) {
            edit(session, "Set " + name + " " + what, key + "/" + what, [&](Document& next) {
                f(builder::findLink(next, name)->shapes[index]);
                return true;
            });
        };
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        switch (shape.kind) {
            case Shape::Kind::Box: {
                double size[3] = {shape.size[0], shape.size[1], shape.size[2]};
                if (widgets::Vec3Edit("size", size, "%.3f") != 0) {
                    change("size", [&](Shape& s) { s.size = {size[0], size[1], size[2]}; });
                }
                break;
            }
            case Shape::Kind::Cylinder:
                if (const auto v = numberRow("radius", shape.size[0], "m", 1.0, nullptr, false)) {
                    change("radius", [&](Shape& s) { s.size[0] = v->value_or(s.size[0]); });
                }
                if (const auto v = numberRow("length", shape.size[1], "m", 1.0, nullptr, false)) {
                    change("length", [&](Shape& s) { s.size[1] = v->value_or(s.size[1]); });
                }
                break;
            case Shape::Kind::Sphere:
                if (const auto v = numberRow("radius", shape.size[0], "m", 1.0, nullptr, false)) {
                    change("radius", [&](Shape& s) { s.size[0] = v->value_or(s.size[0]); });
                }
                break;
            case Shape::Kind::Mesh:
                widgets::PropertyRow("file", shape.mesh.c_str(), nullptr);
                break;
        }
        builder::Pose origin = shape.origin;
        if (poseRows(origin, true)) {
            change("origin", [&](Shape& s) { s.origin = origin; });
        }
        switchRow("visual", shape.visual,
                  [&](bool on) { change("visual", [&](Shape& s) { s.visual = on; }); });
        switchRow("collision", shape.collision,
                  [&](bool on) { change("collision", [&](Shape& s) { s.collision = on; }); });
        switchRow(
            "solid", shape.solid,
            [&](bool on) { change("solid", [&](Shape& s) { s.solid = on; }); },
            "Counts for mass (RFC 0016 §4)");
        // Colour: linear 0..1 RGBA; all zero is "no colour given".
        const bool none = shape.rgba == std::array<float, 4>{};
        char rgba[64] = "";
        if (!none) {
            std::snprintf(rgba, sizeof(rgba), "%.2f %.2f %.2f %.2f", shape.rgba[0], shape.rgba[1],
                          shape.rgba[2], shape.rgba[3]);
        }
        std::string colour = rgba;
        if (widgets::CommitEdit("colour", &colour, "r g b a", false,
                                "Four numbers 0 to 1; empty for none", false)) {
            std::array<float, 4> parsed{};
            if (colour.find_first_not_of(' ') == std::string::npos ||
                std::sscanf(colour.c_str(), "%f %f %f %f", &parsed[0], &parsed[1], &parsed[2],
                            &parsed[3]) == 4) {
                change("colour", [&](Shape& s) { s.rgba = parsed; });
            }
        }
        if (link.shapes.size() > 1) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + px(112.0F));
            if (widgets::Button("Remove shape", widgets::ButtonKind::Ghost)) {
                edit(session, "Remove a shape of " + name, {}, [&](Document& next) {
                    auto& shapes = builder::findLink(next, name)->shapes;
                    shapes.erase(shapes.begin() + static_cast<std::ptrdiff_t>(index));
                    return true;
                });
            }
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }

    static void switchRow(const char* label, bool value, const std::function<void(bool)>& apply,
                          const char* tooltip = nullptr) {
        ImGui::PushID(label);
        const float start = ImGui::GetCursorPosX();
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, c::t2);
        ImGui::TextUnformatted(label);
        ImGui::PopStyleColor();
        ImGui::SameLine(start + px(112.0F));
        bool on = value;
        if (widgets::Switch("##switch", &on)) {
            apply(on);
        }
        if (tooltip != nullptr) {
            ImGui::SetItemTooltip("%s", tooltip);
        }
        ImGui::PopID();
    }

    void mass(BuilderSession& session, const Link& link) {
        if (!widgets::SectionHeader("Mass", "kg · kg/m³")) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string name = link.name;
        const std::string key = name + "/mass";
        // Material: a preset sets the density; "custom" is the density field below.
        const auto presets = builder::densityPresets();
        std::string material = "custom";
        for (const builder::DensityPreset& preset : presets) {
            if (preset.density == link.inertial.density) {
                material = std::string(preset.name);
            }
        }
        if (widgets::DropdownRow("material", material.c_str())) {
            ImGui::OpenPopup("##link-material");
        }
        if (ImGui::BeginPopup("##link-material")) {
            if (presets.empty()) {
                widgets::Caption("No density presets yet (RFC 0016 WP-A): type a density");
            }
            for (const builder::DensityPreset& preset : presets) {
                if (ImGui::Selectable(std::string(preset.name).c_str(), material == preset.name)) {
                    const double density = preset.density;
                    edit(session, "Set " + name + " material " + std::string(preset.name),
                         key + "/density", [&](Document& next) {
                             builder::findLink(next, name)->inertial.density = density;
                             return true;
                         });
                }
            }
            ImGui::EndPopup();
        }
        const std::optional<double> density = link.inertial.density > 0.0
                                                  ? std::optional<double>(link.inertial.density)
                                                  : std::nullopt;
        if (const auto v = numberRow("density", density, "kg/m³")) {
            edit(session, "Set " + name + " density", key + "/density", [&](Document& next) {
                builder::findLink(next, name)->inertial.density = v->value_or(0.0);
                return true;
            });
        }
        const std::optional<builder::MassProperties> computed = builder::linkMass(link);
        char value[96];
        std::snprintf(value, sizeof(value), "%s",
                      computed ? builder::formatNumber(computed->mass, 4).c_str() : "—");
        widgets::PropertyRow("computed", value, "kg", false,
                             computed ? nullptr
                                      : "No mass: give a density or a mass (or the mass model "
                                        "is not built yet, RFC 0016 WP-A)");
        if (const auto v = numberRow("mass override", link.inertial.mass, "kg")) {
            edit(session, "Set " + name + " mass", key + "/override", [&](Document& next) {
                builder::findLink(next, name)->inertial.mass = *v;
                return true;
            });
        }
        // Inertia: computed and read-only unless overridden.
        const bool inertiaOverride = link.inertial.inertia.has_value();
        switchRow("inertia override", inertiaOverride, [&](bool on) {
            edit(session, std::string(on ? "Override " : "Compute ") + name + " inertia",
                 key + "/inertia", [&](Document& next) {
                     auto& inertia = builder::findLink(next, name)->inertial.inertia;
                     if (on) {
                         inertia = computed ? computed->inertia
                                            : std::array<double, 6>{1e-3, 0, 0, 1e-3, 0, 1e-3};
                     } else {
                         inertia.reset();
                     }
                     return true;
                 });
        });
        const std::array<double, 6> tensor =
            inertiaOverride ? *link.inertial.inertia
                            : (computed ? computed->inertia : std::array<double, 6>{});
        double diagonal[3] = {tensor[0], tensor[3], tensor[5]};  // ixx iyy izz
        double products[3] = {tensor[1], tensor[2], tensor[4]};  // ixy ixz iyz
        if (inertiaOverride) {
            const int a = widgets::Vec3Edit("ixx iyy izz", diagonal, "%.3g");
            const int b = widgets::Vec3Edit("ixy ixz iyz", products, "%.3g");
            if ((a | b) != 0) {
                edit(session, "Set " + name + " inertia", key + "/inertia", [&](Document& next) {
                    builder::findLink(next, name)->inertial.inertia =
                        std::array<double, 6>{diagonal[0], products[0], products[1],
                                              diagonal[1], products[2], diagonal[2]};
                    return true;
                });
            }
        } else {
            widgets::Vec3Field("ixx iyy izz", diagonal, "%.3g");
            widgets::Vec3Field("ixy ixz iyz", products, "%.3g");
        }
        const bool comOverride = link.inertial.com.has_value();
        switchRow("COM override", comOverride, [&](bool on) {
            edit(session, std::string(on ? "Override " : "Compute ") + name + " COM", key + "/com",
                 [&](Document& next) {
                     auto& com = builder::findLink(next, name)->inertial.com;
                     if (on) {
                         com = computed ? computed->com : std::array<double, 3>{};
                     } else {
                         com.reset();
                     }
                     return true;
                 });
        });
        const std::array<double, 3> com =
            comOverride ? *link.inertial.com : (computed ? computed->com : std::array<double, 3>{});
        double point[3] = {com[0], com[1], com[2]};
        if (comOverride) {
            if (widgets::Vec3Edit("COM", point, "%.3f") != 0) {
                edit(session, "Set " + name + " COM", key + "/com", [&](Document& next) {
                    builder::findLink(next, name)->inertial.com =
                        std::array<double, 3>{point[0], point[1], point[2]};
                    return true;
                });
            }
        } else {
            widgets::Vec3Field("COM", point, "%.3f");
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }

    static void keptXml(const std::string& xml) {
        if (xml.empty()) {
            return;
        }
        if (widgets::SectionHeader("Kept as written", "read-only", false)) {
            ImGui::PushTextWrapPos(0.0F);
            widgets::Mono(c::t2, "%s", xml.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    // --- joint ----------------------------------------------------------------------------

    void joint(BuilderSession& session, const Joint& joint) {
        const Document& document = session.document();
        header(session, widgets::Icon::JointAxis, joint.name, "joint",
               treePath(document, joint.child), builder::ItemKind::Joint);
        widgets::Pill(std::string(builder::toString(joint.type)).c_str(),
                      widgets::PillKind::Neutral);
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string name = joint.name;
        const std::string key = "joint/" + name;
        const bool degrees = session.showDegrees;
        const auto change = [&](const std::string& what, const std::function<void(Joint&)>& f) {
            edit(session, "Set " + name + " " + what, key + "/" + what, [&](Document& next) {
                f(*builder::findJoint(next, name));
                return true;
            });
        };

        if (widgets::SectionHeader("Joint", "parent → child")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            const char* types[] = {"fixed", "revolute", "continuous", "prismatic"};
            int index = static_cast<int>(joint.type);
            const float start = ImGui::GetCursorPosX();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(c::t2, "type");
            ImGui::SameLine(start + px(112.0F));
            if (widgets::Segmented("##type", types, 4, &index)) {
                change("type", [&](Joint& j) { j.type = static_cast<Joint::Type>(index); });
            }
            if (widgets::DropdownRow("parent", joint.parent.c_str())) {
                ImGui::OpenPopup("##parent");
            }
            if (ImGui::BeginPopup("##parent")) {
                const std::vector<std::string> below = builder::subtreeOf(document, joint.child);
                for (const Link& link : document.links) {
                    if (link.name == joint.child ||
                        std::ranges::find(below, link.name) != below.end()) {
                        continue;  // never under itself
                    }
                    if (ImGui::Selectable(link.name.c_str(), link.name == joint.parent)) {
                        Document next = document;
                        const builder::Tree tree = builder::buildTree(next);
                        if (builder::reparent(next, joint.child, link.name,
                                              builder::linkFrames(next, tree, builder::Preview{}))
                                .empty()) {
                            commit(session, std::move(next),
                                   "Move " + joint.child + " to " + link.name);
                        }
                    }
                }
                ImGui::EndPopup();
            }
            if (widgets::PropertyRow("child", joint.child.c_str(), nullptr, true,
                                     "Select the child link")) {
                select(session, Kind::Link, joint.child);
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (widgets::SectionHeader("Origin and axis", degrees ? "m · deg" : "m · rad")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            builder::Pose origin = joint.origin;
            if (poseRows(origin, degrees)) {
                change("origin", [&](Joint& j) { j.origin = origin; });
            }
            double axis[3] = {joint.axis[0], joint.axis[1], joint.axis[2]};
            if (widgets::Vec3Edit("axis", axis, "%.3f") != 0) {
                change("axis", [&](Joint& j) { j.axis = {axis[0], axis[1], axis[2]}; });
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        const bool angularJoint = joint.type != Joint::Type::Prismatic;
        const double scale = angularJoint && degrees ? builder::toDegrees(1.0) : 1.0;
        const char* position = angularJoint ? (degrees ? "°" : "rad") : "m";
        if (widgets::SectionHeader("Limits",
                                   joint.type == Joint::Type::Fixed ? "none" : "required")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            if (joint.type == Joint::Type::Fixed) {
                widgets::Caption("A fixed joint has no limits.");
            }
            if (joint.type == Joint::Type::Revolute || joint.type == Joint::Type::Prismatic) {
                if (const auto v = numberRow("lower", joint.lower, position, scale)) {
                    change("lower", [&](Joint& j) { j.lower = *v; });
                }
                if (const auto v = numberRow("upper", joint.upper, position, scale)) {
                    change("upper", [&](Joint& j) { j.upper = *v; });
                }
            }
            if (joint.type != Joint::Type::Fixed) {
                if (const auto v = numberRow("effort", joint.effort, angularJoint ? "N·m" : "N")) {
                    change("effort", [&](Joint& j) { j.effort = *v; });
                }
                if (const auto v =
                        numberRow("velocity", joint.velocity, angularJoint ? "rad/s" : "m/s")) {
                    change("velocity", [&](Joint& j) { j.velocity = *v; });
                }
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (widgets::SectionHeader("Dynamics", "damping · friction")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            if (const auto v = numberRow("damping", joint.damping,
                                         angularJoint ? "N·m·s/rad" : "N·s/m", 1.0, "0", false)) {
                change("damping", [&](Joint& j) { j.damping = v->value_or(0.0); });
            }
            if (const auto v = numberRow("friction", joint.friction, angularJoint ? "N·m" : "N",
                                         1.0, "0", false)) {
                change("friction", [&](Joint& j) { j.friction = v->value_or(0.0); });
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (widgets::SectionHeader("Mimic", joint.mimic ? "follows a joint" : "none", false)) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            const std::string leader = joint.mimic ? std::get<0>(*joint.mimic) : "none";
            if (widgets::DropdownRow("joint", leader.c_str())) {
                ImGui::OpenPopup("##leader");
            }
            if (ImGui::BeginPopup("##leader")) {
                if (ImGui::Selectable("none", !joint.mimic)) {
                    change("mimic", [&](Joint& j) { j.mimic.reset(); });
                }
                for (const Joint& other : document.joints) {
                    if (other.name != name && builder::isMoving(other.type) &&
                        ImGui::Selectable(other.name.c_str(), other.name == leader)) {
                        const std::string chosen = other.name;
                        change("mimic",
                               [&](Joint& j) { j.mimic = std::make_tuple(chosen, 1.0, 0.0); });
                    }
                }
                ImGui::EndPopup();
            }
            if (joint.mimic) {
                if (const auto v =
                        numberRow("multiplier", std::get<1>(*joint.mimic), "1", 1.0, "1", false)) {
                    change("mimic", [&](Joint& j) { std::get<1>(*j.mimic) = v->value_or(1.0); });
                }
                if (const auto v = numberRow("offset", std::get<2>(*joint.mimic), position, scale,
                                             "0", false)) {
                    change("mimic", [&](Joint& j) { std::get<2>(*j.mimic) = v->value_or(0.0); });
                }
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        keptXml(joint.keptXml);
    }

    // --- sensor ---------------------------------------------------------------------------

    void sensor(BuilderSession& session, const Sensor& sensor) {
        const Document& document = session.document();
        const builder::CatalogEntry* entry = session.catalog.find(sensor.type);
        header(session, widgets::Icon::Sensors, sensor.name, "sensor",
               treePath(document, sensor.link) + " / " + sensor.name, builder::ItemKind::Sensor);
        const std::string kindPill =
            sensor.type + (entry == nullptr
                               ? " · not in the catalog"
                               : (entry->builtinType ? " · built-in type" : " · built-in kind"));
        widgets::Pill(kindPill.c_str(),
                      entry != nullptr ? widgets::PillKind::Ok : widgets::PillKind::Warn);
        if (entry != nullptr && !entry->doc.empty()) {
            ImGui::SameLine(0.0F, px(4.0F));
            widgets::Pill(entry->doc.c_str(), widgets::PillKind::Neutral);
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string name = sensor.name;
        const std::string key = "sensor/" + name;
        const auto change = [&](const std::string& what, const std::function<void(Sensor&)>& f) {
            edit(session, "Set " + name + " " + what, key + "/" + what, [&](Document& next) {
                f(*builder::findSensor(next, name));
                return true;
            });
        };

        if (widgets::SectionHeader("Mount", "on its link · m · deg")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            if (widgets::DropdownRow("link", sensor.link.c_str())) {
                ImGui::OpenPopup("##mount-link");
            }
            if (ImGui::BeginPopup("##mount-link")) {
                for (const Link& link : document.links) {
                    if (ImGui::Selectable(link.name.c_str(), link.name == sensor.link)) {
                        const std::string chosen = link.name;
                        change("link", [&](Sensor& s) { s.link = chosen; });
                    }
                }
                ImGui::EndPopup();
            }
            builder::Pose origin = sensor.origin;
            if (poseRows(origin, true)) {
                change("mount", [&](Sensor& s) { s.origin = origin; });
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (widgets::SectionHeader("Timing", "sensor:latency · jitter")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            const std::string rateDefault = builder::formatNumber(kDefaultUpdateRate);
            if (sensor.updateRate != kDefaultUpdateRate) {
                ++differing_;
            }
            if (const auto v = numberRow("update_rate", sensor.updateRate, "Hz", 1.0,
                                         rateDefault.c_str(), false, true)) {
                change("update_rate", [&](Sensor& s) { s.updateRate = v->value_or(s.updateRate); });
            }
            if (const auto v =
                    numberRow("latency", sensor.latency, "s", 1.0, nullptr, true, true)) {
                change("latency", [&](Sensor& s) { s.latency = *v; });
            }
            if (const auto v = numberRow("jitter", sensor.jitter, "s", 1.0, nullptr, true, true)) {
                change("jitter", [&](Sensor& s) { s.jitter = *v; });
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (entry != nullptr) {
            catalogParams(session, sensor, *entry);
        } else {
            freeParams(session, sensor);
        }
        if (entry != nullptr && widgets::SectionHeader("Output", "when tested")) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            bool first = true;
            for (const builder::CatalogOutput& signal : entry->signals) {
                const std::string path = "sensor/" + name + "/" + signal.name;
                ImGui::PushID(path.c_str());
                widgets::PropertyRow(
                    first ? "signals" : "", path.c_str(),
                    signal.unit.empty() ? signal.type.c_str() : signal.unit.c_str());
                ImGui::PopID();
                first = false;
            }
            first = true;
            for (const builder::CatalogOutput& topic : entry->rosTopics) {
                // "/imu" is the robot's own topic; "range" is under the sensor's name.
                const std::string path =
                    topic.name.starts_with('/') ? topic.name : "/" + name + "/" + topic.name;
                ImGui::PushID(path.c_str());
                widgets::PropertyRow(first ? "ROS 2" : "", path.c_str(), topic.type.c_str());
                ImGui::PopID();
                first = false;
            }
            if (entry->signals.empty() && entry->rosTopics.empty()) {
                widgets::Caption("The catalog lists no outputs for %s.", entry->name.c_str());
            }
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        }
        if (widgets::SectionHeader("Faults", "none", false)) {
            ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
            widgets::Caption("The catalog has no fault parameters for this sensor yet.");
        }
        // Reset to defaults: the update rate and every catalog parameter.
        resetDefaults_ = [name](Document& next) {
            Sensor* s = builder::findSensor(next, name);
            s->updateRate = kDefaultUpdateRate;
            s->params.clear();
        };
    }

    /// The text is the catalog's default: "4.0" is the default 4 of a number.
    static bool atDefault(const builder::CatalogParam& param, const std::string& text) {
        if (text == param.defaultText) {
            return true;
        }
        if (param.type != builder::CatalogParam::Type::Number &&
            param.type != builder::CatalogParam::Type::Int) {
            return false;
        }
        const auto value = builder::parseNumber(text);
        const auto fallback = builder::parseNumber(param.defaultText);
        return value && fallback && *value == *fallback;
    }

    void catalogParams(BuilderSession& session, const Sensor& sensor,
                       const builder::CatalogEntry& entry) {
        const std::string meta = std::to_string(entry.params.size()) + " parameters · catalog";
        const ImVec4 dot = sensorsColor();
        if (!widgets::SectionHeader(entry.name.c_str(), meta.c_str(), true, &dot)) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string name = sensor.name;
        // Paths too long for the label column ("imu/angular_velocity/{axis}/noise/stddev") are
        // shown under their parent path, a mono caption row, with the last part as the label.
        ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
        const bool nested = std::ranges::any_of(entry.params, [](const builder::CatalogParam& p) {
            return ImGui::CalcTextSize(p.path.c_str()).x > px(106.0F);
        });
        ImGui::PopFont();
        std::string parent;
        for (const builder::CatalogParam& param : entry.params) {
            const std::size_t slash = nested ? param.path.rfind('/') : std::string::npos;
            const std::string label =
                slash == std::string::npos ? param.path : param.path.substr(slash + 1);
            const std::string under =
                slash == std::string::npos ? std::string{} : param.path.substr(0, slash);
            if (nested && under != parent) {
                parent = under;
                if (!under.empty()) {
                    ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
                    ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
                    ImGui::TextColored(c::t3, "%s/", under.c_str());
                    ImGui::PopFont();
                }
            }
            ImGui::PushID(param.path.c_str());
            const auto found = sensor.params.find(param.path);
            const std::string value = found != sensor.params.end() ? found->second : "";
            const bool differs = !value.empty() && !atDefault(param, value);
            differing_ += differs ? 1 : 0;
            std::string unit = param.unit == "1" ? "1" : param.unit;
            if (differs && !param.defaultText.empty()) {
                unit += std::string(unit.empty() ? "" : " · ") + "default " + param.defaultText;
            }
            const auto apply = [&](const std::string& entered) {
                const std::string error = builder::checkParam(param, entered);
                if (!error.empty()) {
                    paramError_ = param.path + ": " + error;
                    return;
                }
                paramError_.clear();
                // A number is kept as the server reads it: "−2" (the design's minus) is "-2".
                std::string typed = entered;
                if (param.type == builder::CatalogParam::Type::Number ||
                    param.type == builder::CatalogParam::Type::Int) {
                    if (const auto number = builder::parseNumber(entered)) {
                        typed = builder::formatNumber(*number, 15);
                    }
                }
                const std::string from = value.empty() ? param.defaultText : value;
                const std::string label =
                    "Set " + param.path + " " + from + " → " + typed +
                    (param.unit.empty() || param.unit == "1" ? "" : " " + param.unit);
                edit(session, label, "sensor/" + name + "/" + param.path, [&](Document& next) {
                    auto& params = builder::findSensor(next, name)->params;
                    if (typed.empty() || atDefault(param, typed)) {
                        params.erase(param.path);
                    } else {
                        params[param.path] = typed;
                    }
                    return true;
                });
            };
            const std::string shown = value.empty() ? param.defaultText : value;
            if (param.type == builder::CatalogParam::Type::Enum ||
                param.type == builder::CatalogParam::Type::Bool) {
                // Enum parameters are dropdowns (SPEC); a bool is the dropdown true | false.
                const float start = ImGui::GetCursorPosX();
                ImGui::AlignTextToFramePadding();
                ImGui::PushFont(theme::fonts().mono, theme::kCaptionSize);
                ImGui::TextColored(c::t2, "%s", label.c_str());
                ImGui::PopFont();
                ImGui::SetItemTooltip("%s", param.path.c_str());
                ImGui::SameLine(start + px(112.0F));
                if (widgets::DropdownField("##enum", shown.c_str(), unit.c_str(),
                                           ImGui::GetContentRegionAvail().x)) {
                    ImGui::OpenPopup("##options");
                }
                if (ImGui::BeginPopup("##options")) {
                    const std::vector<std::string> bools{"true", "false"};
                    const std::vector<std::string>& options =
                        param.type == builder::CatalogParam::Type::Bool ? bools : param.options;
                    for (const std::string& option : options) {
                        if (ImGui::Selectable(option.c_str(), option == shown)) {
                            apply(option);
                        }
                    }
                    ImGui::EndPopup();
                }
            } else {
                std::string typed = shown;
                const bool numeric = param.type == builder::CatalogParam::Type::Number ||
                                     param.type == builder::CatalogParam::Type::Int;
                const std::string tooltip =
                    param.path + (param.meaning.empty() ? "" : "\n") + param.meaning;
                if (widgets::CommitEdit(label.c_str(), &typed, unit.c_str(), differs,
                                        tooltip.c_str(), numeric, true)) {
                    apply(typed);
                }
            }
            ImGui::PopID();
        }
        if (!paramError_.empty()) {
            widgets::Pill(paramError_.c_str(), widgets::PillKind::Error);
        }
        if (!entry.summary.empty()) {
            ImGui::Dummy(ImVec2(0.0F, px(2.0F)));
            ImGui::PushTextWrapPos(0.0F);
            widgets::Caption("%s", entry.summary.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }

    /// A kind the catalog does not know: a free key and value form (SPEC "Other kind…").
    void freeParams(BuilderSession& session, const Sensor& sensor) {
        const std::string meta = std::to_string(sensor.params.size()) + " parameters · free form";
        const ImVec4 dot = sensorsColor();
        if (!widgets::SectionHeader(sensor.type.c_str(), meta.c_str(), true, &dot)) {
            return;
        }
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
        const std::string name = sensor.name;
        for (const auto& [path, value] : sensor.params) {
            std::string typed = value;
            if (widgets::CommitEdit(path.c_str(), &typed, nullptr, false,
                                    "Empty removes the parameter", false, true)) {
                const std::string key = path;
                edit(session, "Set " + key + " " + value + " → " + typed,
                     "sensor/" + name + "/" + key, [&](Document& next) {
                         auto& params = builder::findSensor(next, name)->params;
                         if (typed.empty()) {
                             params.erase(key);
                         } else {
                             params[key] = typed;
                         }
                         return true;
                     });
                break;  // the map changed under the loop
            }
        }
        const float half = (ImGui::GetContentRegionAvail().x - px(6.0F)) * 0.5F;
        (void)widgets::CommitField("##new-key", &newKey_, nullptr, false, half, false);
        ImGui::SetItemTooltip("A parameter path, e.g. range/max");
        ImGui::SameLine(0.0F, px(6.0F));
        std::string value;
        if (widgets::CommitField("##new-value", &value, nullptr, false, half, false) &&
            !newKey_.empty() && !value.empty()) {
            const std::string key = newKey_;
            edit(session, "Set " + key + " → " + value, "sensor/" + name + "/" + key,
                 [&](Document& next) {
                     builder::findSensor(next, name)->params[key] = value;
                     return true;
                 });
            newKey_.clear();
        }
        ImGui::SetItemTooltip("Its value; Enter adds it");
        ImGui::Dummy(ImVec2(0.0F, px(4.0F)));
    }

    // --- footer ---------------------------------------------------------------------------

    void footerBar(BuilderSession& session) {
        // Design: a bar on bg2 with a rule on top: "N values differ from the catalog" in the
        // accent, and Reset to defaults.
        const ImVec2 min = ImGui::GetCursorScreenPos();
        const ImVec2 pad = ImGui::GetStyle().WindowPadding;
        const ImVec2 bar(min.x - pad.x, min.y);
        const ImVec2 end(min.x + ImGui::GetContentRegionAvail().x + pad.x, min.y + px(38.0F));
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(bar, end, ImGui::ColorConvertFloat4ToU32(c::bg2));
        draw->AddLine(bar, ImVec2(end.x, bar.y), ImGui::ColorConvertFloat4ToU32(c::line));
        ImGui::SetCursorScreenPos(ImVec2(min.x, min.y + px(8.0F)));
        if (differing_ == 0 || !resetDefaults_) {
            widgets::Caption("%s", session.selection.kind == Kind::Sensor
                                       ? "Every value at the catalog's default"
                                       : "Every change is one undo step (Ctrl Z)");
            return;
        }
        ImGui::AlignTextToFramePadding();
        ImGui::PushFont(theme::fonts().sans, theme::kCaptionSize);
        ImGui::TextColored(c::accent, "%d value%s differ%s from the catalog", differing_,
                           differing_ == 1 ? "" : "s", differing_ == 1 ? "s" : "");
        ImGui::PopFont();
        ImGui::SameLine();
        widgets::AlignRight(widgets::ButtonWidth("Reset to defaults"));
        if (widgets::Button("Reset to defaults")) {
            const auto reset = resetDefaults_;
            edit(session, "Reset " + session.selection.name + " to defaults", {},
                 [&](Document& next) {
                     reset(next);
                     return true;
                 });
        }
    }

    int differing_{0};
    std::function<void(Document&)> resetDefaults_;
    std::string renameError_;
    std::string paramError_;
    std::string newKey_;
};

class HistoryPanel final : public Panel {
public:
    [[nodiscard]] const char* title() const override { return title_.c_str(); }
    [[nodiscard]] Workspace workspace() const override { return Workspace::Builder; }

    void draw(PanelContext& context) override {
        BuilderSession& session = *context.builder;
        const auto labels = session.history.labels();
        title_ = "History · " + std::to_string(labels.size() - 1) + "###BuilderHistory";
        if (!ImGui::Begin(title_.c_str())) {
            ImGui::End();
            return;
        }
        const std::size_t position = session.history.position();
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0F, 0.0F));
        // Newest at the top; a click goes back (or forward) to that step.
        for (std::size_t i = labels.size(); i-- > 0;) {
            widgets::TreeRowSpec row;
            row.name = labels[i].c_str();
            row.selected = i == position;
            row.strong = row.selected;
            const std::string step = i == 0 ? "start" : "#" + std::to_string(i);
            row.type = step.c_str();
            ImGui::PushID(static_cast<int>(i));
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, i > position ? ImGui::GetStyle().Alpha * 0.55F
                                                                  : ImGui::GetStyle().Alpha);
            if (widgets::TreeRow("##step", row) != widgets::TreeHit::None && editable(session)) {
                (void)session.history.goTo(i);
            }
            ImGui::PopStyleVar();
            ImGui::PopID();
        }
        ImGui::PopStyleVar();
        if (labels.size() == 1) {
            ImGui::Dummy(ImVec2(0.0F, px(6.0F)));
            widgets::Caption("%s", session.history.revision() == 0
                                       ? "No edits yet."
                                       : "Only the current state is kept: nothing to go back to.");
        }
        ImGui::End();
    }

private:
    std::string title_ = "History###BuilderHistory";
};

}  // namespace

std::unique_ptr<Panel> createPropertiesPanel() {
    return std::make_unique<PropertiesPanel>();
}

std::unique_ptr<Panel> createHistoryPanel() {
    return std::make_unique<HistoryPanel>();
}

}  // namespace lagrange::viewer::builder_ui
