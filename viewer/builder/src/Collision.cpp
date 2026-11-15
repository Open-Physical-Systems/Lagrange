// How a link collides (RFC 0016 Addendum A). See Collision.hpp.
#include "lagrange/viewer/builder/Collision.hpp"
#include "lagrange/viewer/builder/Validation.hpp"

#include <algorithm>
#include <filesystem>
#include <utility>

namespace lagrange::viewer::builder {

std::optional<std::size_t> collisionMeshOf(const Link& link) {
    for (std::size_t i = 0; i < link.shapes.size(); ++i) {
        if (link.shapes[i].kind == Shape::Kind::Mesh && link.shapes[i].visual) {
            return i;
        }
    }
    return std::nullopt;
}

bool collisionOnly(const Shape& shape) {
    return shape.collision && !shape.visual;
}

void setCollisionChoice(Link& link, CollisionChoice choice) {
    link.collision = choice;
    const std::optional<std::size_t> mesh = collisionMeshOf(link);
    if (choice == CollisionChoice::Unset ||
        (!mesh && (choice == CollisionChoice::Mesh || choice == CollisionChoice::Split))) {
        return;
    }
    if (mesh) {
        Shape& shape = link.shapes[*mesh];
        shape.collision = choice == CollisionChoice::Mesh;
        shape.solid = true;
    }
    const auto removed = [choice](const Shape& shape) {
        if (!collisionOnly(shape)) {
            return false;
        }
        switch (choice) {
            case CollisionChoice::Mesh:
            case CollisionChoice::Split:
                return true;
            case CollisionChoice::Shapes:
            case CollisionChoice::Drawn:
                return shape.kind == Shape::Kind::Mesh;
            case CollisionChoice::Unset:
                return false;
        }
        return false;
    };
    std::erase_if(link.shapes, removed);
}

void setSplitParts(Link& link, const std::vector<std::string>& files) {
    setCollisionChoice(link, CollisionChoice::Split);
    std::erase_if(link.shapes, collisionOnly);
    const std::optional<std::size_t> mesh = collisionMeshOf(link);
    const Pose origin = mesh ? link.shapes[*mesh].origin : Pose{};
    for (const std::string& file : files) {
        Shape part;
        part.kind = Shape::Kind::Mesh;
        part.mesh = file;
        part.origin = origin;
        part.visual = false;
        part.collision = true;
        part.solid = false;
        link.shapes.push_back(std::move(part));
    }
}

std::size_t addCollisionShape(Link& link, Shape shape) {
    shape.visual = false;
    shape.collision = true;
    shape.solid = false;
    shape.rgba = {};
    link.shapes.push_back(std::move(shape));
    return link.shapes.size() - 1;
}

AddedLink addMeshLink(Document& document, std::string_view parent, const std::string& file,
                      Joint::Type type, double density, const std::array<double, 3>& scale) {
    AddedLink added = addLink(document, parent, Shape::Kind::Mesh, type, density);
    if (added.link.empty()) {
        return added;
    }
    Link* link = findLink(document, added.link);
    Shape& shape = link->shapes.front();
    shape.mesh = file;
    shape.scale = scale;
    link->collision = CollisionChoice::Mesh;
    // The file's stem as the name, when it makes a valid one.
    std::string stem = std::filesystem::path(file).stem().string();
    for (char& c : stem) {
        const bool ok =
            (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
        c = ok ? c : '_';
    }
    if (!stem.empty() && stem.front() >= '0' && stem.front() <= '9') {
        stem = "mesh_" + stem;
    }
    if (validName(stem)) {
        const std::string name = uniqueName(document, stem);
        if (rename(document, ItemKind::Link, added.link, name).empty()) {
            added.link = name;
        }
        const std::string joint = uniqueName(document, name + "_joint");
        if (rename(document, ItemKind::Joint, added.joint, joint).empty()) {
            added.joint = joint;
        }
    }
    return added;
}

}  // namespace lagrange::viewer::builder
