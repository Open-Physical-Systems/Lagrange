#include "lagrange/world/World.hpp"

// Header-only xxHash, as in proto/transport: no library to link.
#define XXH_INLINE_ALL
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <xxhash.h>

namespace lagrange {

namespace {

template <class Map>
auto lookup(const Map& map, std::string_view key) -> std::optional<typename Map::mapped_type> {
    const auto it = map.find(key);
    if (it == map.end()) {
        return std::nullopt;
    }
    return it->second;
}

}  // namespace

std::optional<double> SchemaInstance::real(std::string_view attr) const {
    return lookup(reals, attr);
}

std::optional<std::int64_t> SchemaInstance::integer(std::string_view attr) const {
    return lookup(integers, attr);
}

std::optional<std::string_view> SchemaInstance::token(std::string_view attr) const {
    const auto it = tokens.find(attr);
    return it == tokens.end() ? std::nullopt : std::optional<std::string_view>(it->second);
}

std::optional<std::string_view> SchemaInstance::assetPath(std::string_view attr) const {
    const auto it = assetPaths.find(attr);
    return it == assetPaths.end() ? std::nullopt : std::optional<std::string_view>(it->second);
}

std::span<const PrimId> SchemaInstance::rel(std::string_view name) const {
    const auto it = relationships.find(name);
    return it == relationships.end() ? std::span<const PrimId>() : std::span(it->second);
}

std::span<const double> SchemaInstance::realArray(std::string_view attr) const {
    const auto it = realArrays.find(attr);
    return it == realArrays.end() ? std::span<const double>() : std::span(it->second);
}

PrimId World::addPrim(std::string path, std::string instanceName) {
    prims.paths.push_back(std::move(path));
    prims.instanceNames.push_back(std::move(instanceName));
    prims.body.push_back(kWorldBody);
    prims.poseInBody.emplace_back();
    return PrimId{static_cast<std::uint32_t>(prims.paths.size() - 1)};
}

BodyId World::addBody(std::string name, BodyId parentBody, const Pose& poseInParentBody,
                      double massKg, const std::array<double, 3>& centreOfMass,
                      const std::array<double, 6>& inertia) {
    const PrimId body = addPrim("/" + name, name);
    bodies.names.push_back(std::move(name));
    bodies.mass.push_back(massKg);
    bodies.comLocal.push_back(centreOfMass);
    bodies.inertiaLocal.push_back(inertia);
    bodies.prim.push_back(body);
    bodies.robot.push_back(0);
    bodies.kinematic.push_back(false);
    bodies.parent.push_back(parentBody);
    bodies.poseInParent.push_back(poseInParentBody);
    return BodyId{static_cast<std::uint32_t>(bodies.names.size() - 1)};
}

JointId World::addJoint(std::string name, JointType jointType, BodyId childBody,
                        const std::array<double, 3>& jointAxis, bool isActuated) {
    const PrimId joint = addPrim("/" + name, name);
    joints.names.push_back(std::move(name));
    joints.type.push_back(jointType);
    joints.parent.push_back(bodies.parent.at(childBody.index));
    joints.child.push_back(childBody);
    joints.axis.push_back(jointAxis);
    joints.lowerLimit.push_back(0.0);
    joints.upperLimit.push_back(0.0);  // lower == upper: unlimited
    joints.effortLimit.push_back(0.0);
    joints.velocityLimit.push_back(0.0);
    joints.armature.push_back(0.0);
    joints.initialPosition.push_back(0.0);
    joints.prim.push_back(joint);
    joints.actuated.push_back(isActuated);
    return JointId{static_cast<std::uint32_t>(joints.names.size() - 1)};
}

void World::addSensor(std::string name, SensorType sensorType, BodyId onBody, const Pose& pose,
                      double updateRateHz,
                      std::map<std::string, double, std::less<>> sensorSettings) {
    const PrimId sensor = addPrim("/" + name, name);
    sensors.names.push_back(std::move(name));
    sensors.type.push_back(sensorType);
    sensors.body.push_back(onBody);
    sensors.poseInBody.push_back(pose);
    sensors.rateHz.push_back(updateRateHz);
    sensors.prim.push_back(sensor);
    sensors.settings.push_back(std::move(sensorSettings));
    sensors.kind.emplace_back();
    sensors.text.emplace_back();
}

void World::addCustomSensor(std::string name, std::string sensorKind, BodyId onBody,
                            const Pose& pose, double updateRateHz,
                            std::map<std::string, double, std::less<>> sensorSettings,
                            std::map<std::string, std::string, std::less<>> textSettings) {
    addSensor(std::move(name), SensorType::Custom, onBody, pose, updateRateHz,
              std::move(sensorSettings));
    sensors.kind.back() = std::move(sensorKind);
    sensors.text.back() = std::move(textSettings);
}

void World::addCollider(BodyId onBody, ShapeType shapeType, const std::array<double, 3>& shapeSize,
                        const Pose& poseInOwner, double slidingFriction) {
    colliders.body.push_back(onBody);
    colliders.shape.push_back(shapeType);
    colliders.size.push_back(shapeSize);
    colliders.poseInBody.push_back(poseInOwner);
    colliders.friction.push_back(slidingFriction);
    colliders.hull.push_back(kNoHull);
    colliders.material.push_back(kNoMaterial);
}

bool surfaceMaterialPreset(std::string_view name, SurfaceMaterial& out) {
    // reflectivity 905, 1550, specular, transmission, retroreflective; illustrative.
    struct Row {
        std::string_view name;
        SurfaceMaterial material;
    };
    static constexpr std::array<Row, 9> kPresets{{
        {"default", {0.5, 0.5, 0.0, 0.0, false}},
        // A smooth water surface mirrors: about 2 % comes straight back at normal incidence
        // (Fresnel, n = 1.33) and nothing at a slant, and water absorbs what enters.
        {"water", {0.02, 0.02, 1.0, 0.0, false}},
        {"glass", {0.08, 0.08, 0.9, 0.9, false}},
        {"mirror", {0.9, 0.9, 1.0, 0.0, false}},
        {"asphalt", {0.1, 0.1, 0.0, 0.0, false}},
        {"concrete", {0.35, 0.3, 0.0, 0.0, false}},
        {"white_paint", {0.8, 0.7, 0.05, 0.0, false}},
        {"black_rubber", {0.05, 0.05, 0.0, 0.0, false}},
        {"retroreflector", {0.9, 0.9, 0.0, 0.0, true}},
    }};
    for (const Row& row : kPresets) {
        if (row.name == name) {
            out = row.material;
            return true;
        }
    }
    return false;
}

void World::addHullCollider(BodyId onBody, ConvexHull hull, const Pose& poseInOwner,
                            double slidingFriction) {
    std::array<double, 3> half{};
    for (const auto& p : hull.points) {
        for (std::size_t i = 0; i < 3; ++i) {
            half.at(i) = std::max(half.at(i), std::abs(p.at(i)));
        }
    }
    addCollider(onBody, ShapeType::ConvexHull, half, poseInOwner, slidingFriction);
    colliders.hull.back() = static_cast<std::uint32_t>(hulls.size());
    hulls.push_back(std::move(hull));
}

void World::addHeightFieldCollider(HeightField field, const Pose& poseInWorld,
                                   double slidingFriction) {
    double top = 0.0;
    for (const double h : field.heights) {
        top = std::max(top, std::abs(h));
    }
    const std::array<double, 3> half{(field.columns - 1) * field.cellX / 2.0,
                                     (field.rows - 1) * field.cellY / 2.0, top};
    addCollider(kWorldBody, ShapeType::HeightField, half, poseInWorld, slidingFriction);
    colliders.hull.back() = static_cast<std::uint32_t>(heightfields.size());
    heightfields.push_back(std::move(field));
}

std::optional<double> HeightField::heightAt(double x, double y) const {
    if (columns < 2 || rows < 2 || !(cellX > 0.0) || !(cellY > 0.0)) {
        return std::nullopt;
    }
    const double u = x / cellX + (columns - 1) / 2.0;
    const double v = y / cellY + (rows - 1) / 2.0;
    if (u < 0.0 || v < 0.0 || u > columns - 1 || v > rows - 1) {
        return std::nullopt;
    }
    const auto c = std::min(static_cast<std::uint32_t>(u), columns - 2);
    const auto r = std::min(static_cast<std::uint32_t>(v), rows - 2);
    const double fu = u - c;
    const double fv = v - r;
    const auto at = [this](std::uint32_t cc, std::uint32_t rr) {
        return heights[static_cast<std::size_t>(rr) * columns + cc];
    };
    // Two triangles per cell, split along (c, r)-(c + 1, r + 1).
    if (fu >= fv) {
        return at(c, r) + fu * (at(c + 1, r) - at(c, r)) + fv * (at(c + 1, r + 1) - at(c + 1, r));
    }
    return at(c, r) + fv * (at(c, r + 1) - at(c, r)) + fu * (at(c + 1, r + 1) - at(c, r + 1));
}

ConvexHull HeightField::triangles() const {
    ConvexHull out;
    for (std::uint32_t r = 0; r < rows; ++r) {
        for (std::uint32_t c = 0; c < columns; ++c) {
            out.points.push_back({(c - (columns - 1) / 2.0) * cellX, (r - (rows - 1) / 2.0) * cellY,
                                  heights[static_cast<std::size_t>(r) * columns + c]});
        }
    }
    for (std::uint32_t r = 0; r + 1 < rows; ++r) {
        for (std::uint32_t c = 0; c + 1 < columns; ++c) {
            const std::uint32_t a = r * columns + c;
            out.triangles.push_back({a, a + 1, a + columns + 1});
            out.triangles.push_back({a, a + columns + 1, a + columns});
        }
    }
    return out;
}

std::string_view WorldView::primPath(PrimId prim) const {
    return prim.index < world_->prims.paths.size()
               ? std::string_view(world_->prims.paths[prim.index])
               : std::string_view();
}

std::string_view WorldView::signalInstanceName(PrimId prim) const {
    return prim.index < world_->prims.instanceNames.size()
               ? std::string_view(world_->prims.instanceNames[prim.index])
               : std::string_view();
}

std::vector<const SchemaInstance*> WorldView::instancesOf(std::string_view schema) const {
    std::vector<const SchemaInstance*> out;
    for (const SchemaInstance& instance : world_->schemaInstances) {
        if (instance.schema == schema) {
            out.push_back(&instance);
        }
    }
    return out;
}

std::uint64_t WorldView::topologyHash() const {
    XXH3_state_t* state = XXH3_createState();
    XXH3_64bits_reset(state);
    const auto addText = [state](std::string_view text) {
        const std::uint64_t size = text.size();
        XXH3_64bits_update(state, &size, sizeof(size));
        XXH3_64bits_update(state, text.data(), text.size());
    };
    for (const std::string& path : world_->prims.paths) {
        addText(path);
    }
    for (const std::string& name : world_->bodies.names) {
        addText(name);
    }
    for (std::size_t i = 0; i < world_->joints.names.size(); ++i) {
        addText(world_->joints.names[i]);
        const auto type = static_cast<std::uint8_t>(world_->joints.type[i]);
        XXH3_64bits_update(state, &type, sizeof(type));
        XXH3_64bits_update(state, &world_->joints.parent[i].index, sizeof(std::uint32_t));
        XXH3_64bits_update(state, &world_->joints.child[i].index, sizeof(std::uint32_t));
    }
    for (const SchemaInstance& instance : world_->schemaInstances) {
        addText(instance.schema);
        addText(instance.instance);
        XXH3_64bits_update(state, &instance.prim.index, sizeof(std::uint32_t));
    }
    const std::uint64_t hash = XXH3_64bits_digest(state);
    XXH3_freeState(state);
    return hash;
}

}  // namespace lagrange
