#include "lagrange/world/World.hpp"

// Header-only xxHash, as in proto/transport: no library to link.
#define XXH_INLINE_ALL
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
}

void World::addCollider(BodyId onBody, ShapeType shapeType, const std::array<double, 3>& shapeSize,
                        const Pose& poseInOwner, double slidingFriction) {
    colliders.body.push_back(onBody);
    colliders.shape.push_back(shapeType);
    colliders.size.push_back(shapeSize);
    colliders.poseInBody.push_back(poseInOwner);
    colliders.friction.push_back(slidingFriction);
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
        XXH3_64bits_update(state, &instance.prim.index, sizeof(std::uint32_t));
    }
    const std::uint64_t hash = XXH3_64bits_digest(state);
    XXH3_freeState(state);
    return hash;
}

}  // namespace lagrange
