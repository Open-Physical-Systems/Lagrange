#include "lagrange/viewer/builder/Edits.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <set>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

/// Indices of the joints whose parent is `link`, in document order.
std::vector<std::size_t> childJoints(const Document& document, std::string_view link) {
    std::vector<std::size_t> out;
    for (std::size_t j = 0; j < document.joints.size(); ++j) {
        if (document.joints[j].parent == link) {
            out.push_back(j);
        }
    }
    return out;
}

bool isChild(const Document& document, std::string_view link) {
    return std::ranges::any_of(document.joints,
                               [&](const Joint& joint) { return joint.child == link; });
}

bool hasSensorOn(const Document& document, std::string_view link) {
    return std::ranges::any_of(document.sensors,
                               [&](const Sensor& sensor) { return sensor.link == link; });
}

bool usedName(const Document& document, std::string_view name) {
    return findLink(document, name) != nullptr || findJoint(document, name) != nullptr ||
           findSensor(document, name) != nullptr;
}

/// The URDF sensor types the reader knows without a catalog (server/src/Robot.cpp).
bool knownType(std::string_view type) {
    static constexpr std::array<std::string_view, 8> kTypes{
        "camera", "lidar", "gpu_lidar", "ray", "gpu_ray", "imu", "depth", "thermal"};
    return std::ranges::find(kTypes, type) != kTypes.end();
}

using Quat = std::array<double, 4>;  // w x y z

Quat conjugate(const Quat& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

/// URDF rpy (fixed x, y, z: R = Rz Ry Rx) of a unit quaternion.
std::array<double, 3> toRpy(const Quat& q) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sinPitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(sinPitch);
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return {roll, pitch, yaw};
}

Frame inverse(const Frame& frame) {
    Frame out;
    out.rotation = conjugate(frame.rotation);
    const Frame rotated =
        compose(Frame{{0, 0, 0}, out.rotation}, Frame{frame.position, {1, 0, 0, 0}});
    out.position = {-rotated.position[0], -rotated.position[1], -rotated.position[2]};
    return out;
}

}  // namespace

std::vector<TreeItem> treeItems(const Document& document) {
    std::vector<TreeItem> items;
    items.push_back(TreeItem{TreeItem::Kind::Robot,
                             document.robot,
                             0,
                             {},
                             !document.links.empty() || !document.sensors.empty()});
    std::set<std::string, std::less<>> shown;
    const auto visit = [&](auto&& self, const std::string& link, const std::string& joint,
                           int depth) -> void {
        if (!shown.insert(link).second) {
            return;  // a loop: each link once
        }
        const std::vector<std::size_t> children = childJoints(document, link);
        items.push_back(TreeItem{TreeItem::Kind::Link, link, depth, joint,
                                 !children.empty() || hasSensorOn(document, link)});
        for (const std::size_t j : children) {
            if (findLink(document, document.joints[j].child) != nullptr) {
                self(self, document.joints[j].child, document.joints[j].name, depth + 1);
            }
        }
        for (const Sensor& sensor : document.sensors) {
            if (sensor.link == link) {
                items.push_back(
                    TreeItem{TreeItem::Kind::Sensor, sensor.name, depth + 1, {}, false});
            }
        }
    };
    // Roots first, in document order, then whatever they did not reach.
    for (const Link& link : document.links) {
        if (!isChild(document, link.name)) {
            visit(visit, link.name, {}, 1);
        }
    }
    for (const Link& link : document.links) {
        if (!shown.contains(link.name)) {
            const auto joint = std::ranges::find_if(
                document.joints, [&](const Joint& j) { return j.child == link.name; });
            visit(visit, link.name, joint != document.joints.end() ? joint->name : std::string{},
                  1);
        }
    }
    for (const Sensor& sensor : document.sensors) {
        if (findLink(document, sensor.link) == nullptr) {
            items.push_back(TreeItem{TreeItem::Kind::Sensor, sensor.name, 1, {}, false});
        }
    }
    return items;
}

PartCounts countParts(const Document& document, const SensorCatalog* catalog) {
    PartCounts counts;
    counts.links = static_cast<int>(document.links.size());
    for (const Link& link : document.links) {
        counts.roots += isChild(document, link.name) ? 0 : 1;
    }
    counts.joints = static_cast<int>(document.joints.size());
    for (const Joint& joint : document.joints) {
        switch (joint.type) {
            case Joint::Type::Revolute:
                ++counts.revolute;
                break;
            case Joint::Type::Continuous:
                ++counts.continuous;
                break;
            case Joint::Type::Prismatic:
                ++counts.prismatic;
                break;
            case Joint::Type::Fixed:
                ++counts.fixed;
                break;
        }
    }
    counts.sensors = static_cast<int>(document.sensors.size());
    std::set<std::string, std::less<>> types;
    std::set<std::string, std::less<>> kinds;
    for (const Sensor& sensor : document.sensors) {
        const CatalogEntry* entry = catalog != nullptr ? catalog->find(sensor.type) : nullptr;
        const bool type = entry != nullptr ? entry->builtinType : knownType(sensor.type);
        (type ? types : kinds).insert(entry != nullptr ? entry->name : sensor.type);
    }
    counts.types = static_cast<int>(types.size());
    counts.kinds = static_cast<int>(kinds.size());
    return counts;
}

std::string uniqueName(const Document& document, std::string_view base) {
    std::string name(base);
    for (int n = 2; usedName(document, name); ++n) {
        name = std::string(base) + "_" + std::to_string(n);
    }
    return name;
}

Shape defaultShape(Shape::Kind kind) {
    Shape shape;
    shape.kind = kind;
    switch (kind) {
        case Shape::Kind::Box:
            shape.size = {0.1, 0.1, 0.1};
            break;
        case Shape::Kind::Cylinder:
            shape.size = {0.05, 0.1, 0.0};
            break;
        case Shape::Kind::Sphere:
            shape.size = {0.05, 0.0, 0.0};
            break;
        case Shape::Kind::Mesh:
            break;
    }
    return shape;
}

namespace {

double halfHeight(const Shape& shape) {
    switch (shape.kind) {
        case Shape::Kind::Box:
            return shape.size[2] * 0.5;
        case Shape::Kind::Cylinder:
            return shape.size[1] * 0.5;
        case Shape::Kind::Sphere:
            return shape.size[0];
        case Shape::Kind::Mesh:
            return 0.0;
    }
    return 0.0;
}

}  // namespace

double topOf(const Link& link) {
    double top = 0.0;
    for (const Shape& shape : link.shapes) {
        top = std::max(top, shape.origin.xyz[2] + halfHeight(shape));
    }
    return top;
}

AddedLink addLink(Document& document, std::string_view parent, Shape::Kind kind, Joint::Type type,
                  double density) {
    const Link* parentLink = findLink(document, parent);
    if (parentLink == nullptr) {
        return {};
    }
    const Shape shape = defaultShape(kind);
    Link link;
    link.name = uniqueName(document, std::string(toString(kind)) + "_link");
    link.shapes.push_back(shape);
    link.inertial.density = density > 0.0 ? density : 0.0;
    Joint joint;
    joint.type = type;
    joint.parent = std::string(parent);
    joint.child = link.name;
    // On top of the parent, so that the new shape does not overlap it.
    joint.origin.xyz = {0.0, 0.0, topOf(*parentLink) + halfHeight(shape)};
    joint.axis = {0.0, 0.0, 1.0};
    AddedLink added{link.name, {}};
    document.links.push_back(std::move(link));
    joint.name = uniqueName(document, added.link + "_joint");
    added.joint = joint.name;
    document.joints.push_back(std::move(joint));
    return added;
}

std::string addSensor(Document& document, std::string_view link, std::string_view type) {
    if (findLink(document, link) == nullptr || type.empty()) {
        return {};
    }
    Sensor sensor;
    sensor.name = uniqueName(document, type);
    sensor.link = std::string(link);
    sensor.type = std::string(type);
    std::string name = sensor.name;
    document.sensors.push_back(std::move(sensor));
    return name;
}

std::vector<std::string> subtreeOf(const Document& document, std::string_view link) {
    std::vector<std::string> out;
    std::set<std::string, std::less<>> seen{std::string(link)};
    std::vector<std::string> stack{std::string(link)};
    while (!stack.empty()) {
        const std::string current = stack.back();
        stack.pop_back();
        const std::vector<std::size_t> children = childJoints(document, current);
        for (auto it = children.rbegin(); it != children.rend(); ++it) {
            const std::string& child = document.joints[*it].child;
            if (seen.insert(child).second) {
                out.push_back(child);
                stack.push_back(child);
            }
        }
    }
    return out;
}

bool removeLink(Document& document, std::string_view link) {
    if (findLink(document, link) == nullptr) {
        return false;
    }
    std::set<std::string, std::less<>> gone{std::string(link)};
    for (std::string& name : subtreeOf(document, link)) {
        gone.insert(std::move(name));
    }
    std::erase_if(document.links, [&](const Link& l) { return gone.contains(l.name); });
    std::erase_if(document.joints, [&](const Joint& j) {
        return gone.contains(j.child) || gone.contains(j.parent);
    });
    std::erase_if(document.sensors, [&](const Sensor& s) { return gone.contains(s.link); });
    return true;
}

bool removeJoint(Document& document, std::string_view joint) {
    const Joint* found = findJoint(document, joint);
    if (found == nullptr) {
        return false;
    }
    const std::string child = found->child;
    if (!removeLink(document, child)) {
        std::erase_if(document.joints, [&](const Joint& j) { return j.name == joint; });
    }
    return true;
}

bool removeSensor(Document& document, std::string_view sensor) {
    return std::erase_if(document.sensors, [&](const Sensor& s) { return s.name == sensor; }) != 0;
}

std::string rename(Document& document, ItemKind kind, std::string_view from, std::string_view to) {
    if (from == to) {
        return {};
    }
    if (!validName(to)) {
        return "a name is letters, digits and _, and does not start with a digit";
    }
    const std::string old(from);
    const std::string name(to);
    switch (kind) {
        case ItemKind::Robot:
            document.robot = name;
            return {};
        case ItemKind::Link: {
            Link* link = findLink(document, old);
            if (link == nullptr) {
                return "there is no link " + old;
            }
            if (findLink(document, name) != nullptr) {
                return "a link is already named " + name;
            }
            link->name = name;
            for (Joint& joint : document.joints) {
                if (joint.parent == old) {
                    joint.parent = name;
                }
                if (joint.child == old) {
                    joint.child = name;
                }
            }
            for (Sensor& sensor : document.sensors) {
                if (sensor.link == old) {
                    sensor.link = name;
                }
            }
            return {};
        }
        case ItemKind::Joint: {
            Joint* joint = findJoint(document, old);
            if (joint == nullptr) {
                return "there is no joint " + old;
            }
            if (findJoint(document, name) != nullptr) {
                return "a joint is already named " + name;
            }
            joint->name = name;
            for (Joint& other : document.joints) {
                if (other.mimic && std::get<0>(*other.mimic) == old) {
                    std::get<0>(*other.mimic) = name;
                }
            }
            return {};
        }
        case ItemKind::Sensor: {
            Sensor* sensor = findSensor(document, old);
            if (sensor == nullptr) {
                return "there is no sensor " + old;
            }
            if (findSensor(document, name) != nullptr) {
                return "a sensor is already named " + name;
            }
            sensor->name = name;
            return {};
        }
    }
    return {};
}

std::string reparent(Document& document, std::string_view link, std::string_view parent,
                     const std::vector<Frame>& frames) {
    if (findLink(document, link) == nullptr) {
        return "there is no link " + std::string(link);
    }
    if (findLink(document, parent) == nullptr) {
        return "there is no link " + std::string(parent);
    }
    if (link == parent) {
        return "a link cannot be its own parent";
    }
    const std::vector<std::string> below = subtreeOf(document, link);
    if (std::ranges::find(below, parent) != below.end()) {
        return std::string(parent) + " is under " + std::string(link);
    }
    const auto joint =
        std::ranges::find_if(document.joints, [&](const Joint& j) { return j.child == link; });
    if (joint == document.joints.end()) {
        return std::string(link) + " is the root: it has no joint to move";
    }
    if (joint->parent == parent) {
        return {};
    }
    joint->parent = std::string(parent);
    // Keep the link where it is relative to the root.
    const auto index = [&](std::string_view name) {
        return static_cast<std::size_t>(
            std::ranges::find_if(document.links, [&](const Link& l) { return l.name == name; }) -
            document.links.begin());
    };
    if (frames.size() == document.links.size()) {
        const Frame relative = compose(inverse(frames[index(parent)]), frames[index(link)]);
        joint->origin.xyz = relative.position;
        joint->origin.rpy = toRpy(relative.rotation);
    }
    return {};
}

std::string formatNumber(double value, int digits) {
    if (value == 0.0) {
        return "0";  // also minus zero
    }
    char text[48];
    std::snprintf(text, sizeof(text), "%.*g", std::clamp(digits, 1, 17), value);
    return text;
}

std::optional<double> parseNumber(std::string_view text) {
    std::string clean;
    clean.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        // U+2212 MINUS SIGN is E2 88 92 in UTF-8.
        if (text.substr(i, 3) == "\xE2\x88\x92") {
            clean.push_back('-');
            i += 2;
        } else {
            clean.push_back(text[i]);
        }
    }
    const auto first = clean.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return std::nullopt;
    }
    const auto last = clean.find_last_not_of(" \t");
    clean = clean.substr(first, last - first + 1);
    char* end = nullptr;
    const double value = std::strtod(clean.c_str(), &end);
    if (end != clean.c_str() + clean.size() || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::string checkParam(const CatalogParam& param, std::string_view text) {
    if (text.find_first_not_of(" \t") == std::string_view::npos) {
        return {};  // empty: the default
    }
    const std::string unit = param.unit.empty() || param.unit == "1" ? "" : " " + param.unit;
    switch (param.type) {
        case CatalogParam::Type::Number:
        case CatalogParam::Type::Int: {
            const std::optional<double> value = parseNumber(text);
            if (!value) {
                return "not a number";
            }
            if (param.type == CatalogParam::Type::Int && std::floor(*value) != *value) {
                return "a whole number";
            }
            if (param.min && *value < *param.min) {
                return "at least " + formatNumber(*param.min) + unit;
            }
            if (param.max && *value > *param.max) {
                return "at most " + formatNumber(*param.max) + unit;
            }
            return {};
        }
        case CatalogParam::Type::Bool:
            if (text == "true" || text == "false" || text == "1" || text == "0") {
                return {};
            }
            return "true or false";
        case CatalogParam::Type::Enum:
            if (std::ranges::find(param.options, text) != param.options.end()) {
                return {};
            }
            return "one of the listed values";
        case CatalogParam::Type::Text:
        case CatalogParam::Type::List:
            return {};
    }
    return {};
}

double toDegrees(double radians) {
    return radians * 180.0 / std::numbers::pi;
}

double toRadians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

}  // namespace lagrange::viewer::builder
