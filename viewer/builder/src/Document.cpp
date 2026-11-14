#include "lagrange/viewer/builder/Document.hpp"

#include <algorithm>
#include <utility>

namespace lagrange::viewer::builder {

namespace {

template <typename Items>
auto* findByName(Items& items, std::string_view name) {
    const auto it =
        std::ranges::find_if(items, [name](const auto& item) { return item.name == name; });
    return it == items.end() ? nullptr : &*it;
}

}  // namespace

Document newDocument(std::string robot) {
    Document document;
    document.robot = std::move(robot);
    Link root;
    root.name = std::string(kNewRootLink);
    Shape box;
    box.kind = Shape::Kind::Box;
    box.size = kNewRootBox;
    root.shapes.push_back(box);
    document.links.push_back(std::move(root));
    return document;
}

const Link* findLink(const Document& document, std::string_view name) {
    return findByName(document.links, name);
}
const Joint* findJoint(const Document& document, std::string_view name) {
    return findByName(document.joints, name);
}
const Sensor* findSensor(const Document& document, std::string_view name) {
    return findByName(document.sensors, name);
}
Link* findLink(Document& document, std::string_view name) {
    return findByName(document.links, name);
}
Joint* findJoint(Document& document, std::string_view name) {
    return findByName(document.joints, name);
}
Sensor* findSensor(Document& document, std::string_view name) {
    return findByName(document.sensors, name);
}

std::string_view toString(Joint::Type type) {
    switch (type) {
        case Joint::Type::Fixed:
            return "fixed";
        case Joint::Type::Revolute:
            return "revolute";
        case Joint::Type::Continuous:
            return "continuous";
        case Joint::Type::Prismatic:
            return "prismatic";
    }
    return "fixed";
}

std::optional<Joint::Type> parseJointType(std::string_view text) {
    for (const Joint::Type type : {Joint::Type::Fixed, Joint::Type::Revolute,
                                   Joint::Type::Continuous, Joint::Type::Prismatic}) {
        if (toString(type) == text) {
            return type;
        }
    }
    return std::nullopt;
}

std::string_view toString(Shape::Kind kind) {
    switch (kind) {
        case Shape::Kind::Box:
            return "box";
        case Shape::Kind::Cylinder:
            return "cylinder";
        case Shape::Kind::Sphere:
            return "sphere";
        case Shape::Kind::Mesh:
            return "mesh";
    }
    return "box";
}

bool isMoving(Joint::Type type) {
    return type != Joint::Type::Fixed;
}

}  // namespace lagrange::viewer::builder
