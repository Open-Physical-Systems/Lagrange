// Overlay files (docs/design/formats.md §9): the project's API schemas applied by name to a
// model that came from a format without them (URDF, SDFormat, MJCF), or added to a USD one.
#include "Netlist.hpp"
#include "Schemas.hpp"
#include "lagrange/stage/Stage.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/stage.h>
#include <yaml-cpp/yaml.h>

namespace lagrange::stage {

namespace {

using namespace pxr;  // NOLINT(google-build-using-namespace): OpenUSD names are long enough

constexpr int kOverlayVersion = 1;

/// A prim an overlay names: where it is in the scene and the world, and its USD path.
struct Target {
    std::string usdPath;                // "/amr/joints/left_wheel_joint", "/amr/power/dc_48v"
    std::uint32_t scenePrim{kNoIndex};  // for create_child: the prim new ones go under
    std::optional<PrimId> worldPrim;    // set for links, joints and sensors
    bool joint{false};
};

class Overlay {
public:
    Overlay(Model& model, std::filesystem::path file) : m_(model), file_(std::move(file)) {}

    Expected<void, std::string> apply(const YAML::Node& root) {
        if (m_.scene.prims.empty()) {
            return Unexpected{std::string("the model has no prims to apply an overlay to")};
        }
        if (!root.IsMap() || !root["schema_version"] ||
            root["schema_version"].as<int>(0) != kOverlayVersion) {
            return Unexpected{std::string("schema_version must be 1")};
        }
        const YAML::Node entries = root["apply"];
        if (!entries || !entries.IsSequence()) {
            return Unexpected{std::string("apply must be a list")};
        }
        rootPath_ = m_.scene.prims[0].path;
        stage_ = UsdStage::CreateInMemory();
        // First every prim the entries name or create, so relationships may point at prims
        // created further down the file.
        std::vector<std::optional<Target>> owners;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            owners.push_back(ownerOf(entries[i], i));
        }
        for (std::size_t i = 0; i < entries.size(); ++i) {
            if (owners[i]) {
                applySchemas(entries[i], *owners[i]);
            }
        }
        if (errors_.empty()) {
            collect();
        }
        if (errors_.empty()) {
            addSensors();
        }
        if (!errors_.empty()) {
            std::string message = "the overlay cannot be applied:";
            for (const std::string& e : errors_) {
                message += "\n  " + e;
            }
            return Unexpected{message};
        }
        return {};
    }

private:
    /// The prim an entry applies its schemas to: `prim` (a link, joint or sensor by name, or a
    /// path under the robot's top prim), its new child `create_child`, or the new prim at
    /// `create`.
    std::optional<Target> ownerOf(const YAML::Node& entry, std::size_t index) {
        const std::string where = "apply[" + std::to_string(index) + "]";
        if (!entry.IsMap()) {
            errors_.push_back(where + ": an entry is a map with prim or create, and schemas");
            return std::nullopt;
        }
        if (entry["create"]) {
            return created(entry["create"].as<std::string>());
        }
        if (!entry["prim"]) {
            errors_.push_back(where + ": names no prim (prim: or create:)");
            return std::nullopt;
        }
        std::optional<Target> target = resolve(entry["prim"].as<std::string>(), where);
        if (target && entry["create_child"]) {
            return child(*target, entry["create_child"].as<std::string>());
        }
        return target;
    }

    /// A link, joint or sensor by name, or an existing prim by its path under the top prim.
    std::optional<Target> resolve(const std::string& name, const std::string& where) {
        const World& w = m_.world;
        for (std::size_t j = 0; j < w.joints.names.size(); ++j) {
            if (w.joints.names[j] == name) {
                Target t;
                t.usdPath = rootPath_ + "/joints/" + name;
                // A joint's motor is mounted on the joint's parent link.
                const BodyId parent = w.joints.parent[j];
                t.scenePrim = parent == kWorldBody ? 0 : m_.bodyPrim[parent.index];
                t.worldPrim = w.joints.prim[j];
                t.joint = true;
                return define(t);
            }
        }
        for (std::size_t b = 0; b < w.bodies.names.size(); ++b) {
            if (w.bodies.names[b] == name && m_.bodyPrim[b] != kNoIndex) {
                return define(sceneTarget(m_.bodyPrim[b], w.bodies.prim[b]));
            }
        }
        for (std::size_t s = 0; s < w.sensors.names.size(); ++s) {
            if (w.sensors.names[s] == name && s < m_.sensorPrim.size()) {
                return define(sceneTarget(m_.sensorPrim[s], w.sensors.prim[s]));
            }
        }
        const std::string path = rootPath_ + "/" + name;
        for (std::size_t p = 0; p < m_.scene.prims.size(); ++p) {
            if (m_.scene.prims[p].path == path) {
                return define(sceneTarget(static_cast<std::uint32_t>(p), std::nullopt));
            }
        }
        errors_.push_back(where + ": " + name + " is not a link, joint, sensor or prim of " +
                          rootPath_);
        return std::nullopt;
    }

    Target sceneTarget(std::uint32_t scenePrim, std::optional<PrimId> worldPrim) const {
        Target t;
        t.usdPath = m_.scene.prims[scenePrim].path;
        t.scenePrim = scenePrim;
        t.worldPrim = worldPrim;
        return t;
    }

    /// The prim at `relative` under the top prim, created with any missing parents.
    Target created(const std::string& relative) {
        std::uint32_t parent = 0;
        for (const auto& part : std::filesystem::path(relative)) {
            parent = child(sceneTarget(parent, std::nullopt), part.string()).scenePrim;
        }
        return define(sceneTarget(parent, std::nullopt));
    }

    /// The scene prim `name` under `parent`'s scene prim, added if it is not there: an Xform
    /// that moves with the parent.
    Target child(const Target& parent, const std::string& name) {
        const ScenePrim& up = m_.scene.prims[parent.scenePrim];
        const std::string path = up.path + "/" + name;
        for (std::size_t p = 0; p < m_.scene.prims.size(); ++p) {
            if (m_.scene.prims[p].path == path) {
                return define(sceneTarget(static_cast<std::uint32_t>(p), std::nullopt));
            }
        }
        ScenePrim prim;
        prim.name = name;
        prim.path = path;
        prim.typeName = "Xform";
        prim.parent = parent.scenePrim;
        prim.body = up.body;
        prim.offset = up.offset;
        m_.scene.prims.push_back(std::move(prim));
        return define(
            sceneTarget(static_cast<std::uint32_t>(m_.scene.prims.size() - 1), std::nullopt));
    }

    /// Puts the target in the in-memory stage the schemas are applied on, and records its world
    /// prim.
    Target define(const Target& t) {
        const SdfPath path(t.usdPath);
        if (!stage_->GetPrimAtPath(path)) {
            stage_->DefinePrim(path, TfToken(t.joint ? "PhysicsJoint" : "Xform"));
        }
        if (!worldPrim_.contains(t.usdPath)) {
            const World& w = m_.world;
            PrimId id{kNoIndex};
            if (t.worldPrim) {
                id = *t.worldPrim;
            } else {
                for (std::size_t p = 0; p < w.prims.paths.size(); ++p) {
                    if (w.prims.paths[p] == t.usdPath) {
                        id = PrimId{static_cast<std::uint32_t>(p)};
                    }
                }
                if (id.index == kNoIndex) {
                    id = m_.world.addPrim(t.usdPath, path.GetName());
                }
            }
            worldPrim_.emplace(t.usdPath, id);
            if (t.scenePrim != kNoIndex && !t.joint) {  // a joint's is its parent link
                scenePrimOf_.emplace(t.usdPath, t.scenePrim);
            }
        }
        return t;
    }

    void applySchemas(const YAML::Node& entry, const Target& owner) {
        const YAML::Node schemas = entry["schemas"];
        const std::string where = owner.usdPath;
        if (!schemas || !schemas.IsMap()) {
            errors_.push_back(where + ": schemas must be a map of schema names to values");
            return;
        }
        UsdPrim prim = stage_->GetPrimAtPath(SdfPath(owner.usdPath));
        for (const auto& schema : schemas) {
            const std::string name = schema.first.as<std::string>();
            const auto [family, instance] =
                UsdSchemaRegistry::GetTypeNameAndInstance(TfToken(name));
            if (UsdSchemaRegistry::GetInstance().FindAppliedAPIPrimDefinition(family) == nullptr) {
                errors_.push_back(where + ": " + name + " is not a registered schema");
                continue;
            }
            prim.AddAppliedSchema(TfToken(name));
            if (const auto scene = scenePrimOf_.find(owner.usdPath); scene != scenePrimOf_.end()) {
                m_.scene.prims[scene->second].apiSchemas.push_back(name);
            }
            if (!schema.second.IsMap()) {
                continue;
            }
            for (const auto& value : schema.second) {
                setProperty(prim, name, value.first.as<std::string>(), value.second);
            }
        }
    }

    void setProperty(UsdPrim& prim, const std::string& schema, const std::string& property,
                     const YAML::Node& value) {
        const std::string where = prim.GetPath().GetString() + ": " + property;
        const TfToken token(property);
        try {
            if (UsdRelationship rel = prim.GetRelationship(token)) {
                SdfPathVector targets;
                const auto add = [&](const std::string& target) {
                    if (const std::optional<Target> t = resolve(target, where)) {
                        targets.emplace_back(t->usdPath);
                    }
                };
                if (value.IsSequence()) {
                    for (const auto& item : value) {
                        add(item.as<std::string>());
                    }
                } else {
                    add(value.as<std::string>());
                }
                rel.SetTargets(targets);
                return;
            }
            UsdAttribute attribute = prim.GetAttribute(token);
            if (!attribute) {
                errors_.push_back(where + " is not a property of " + schema);
                return;
            }
            const SdfValueTypeName type = attribute.GetTypeName();
            if (type == SdfValueTypeNames->Double) {
                attribute.Set(value.as<double>());
            } else if (type == SdfValueTypeNames->Int) {
                attribute.Set(value.as<int>());
            } else if (type == SdfValueTypeNames->Bool) {
                attribute.Set(value.as<bool>());
            } else if (type == SdfValueTypeNames->Token) {
                attribute.Set(TfToken(value.as<std::string>()));
            } else if (type == SdfValueTypeNames->String) {
                attribute.Set(value.as<std::string>());
            } else if (type == SdfValueTypeNames->Asset) {
                // Relative to the overlay file, as USD resolves asset paths relative to a layer.
                std::filesystem::path asset(value.as<std::string>());
                if (asset.is_relative()) {
                    asset = std::filesystem::absolute(file_.parent_path() / asset);
                }
                attribute.Set(SdfAssetPath(asset.lexically_normal().string()));
            } else if (type == SdfValueTypeNames->DoubleArray) {
                const auto values = value.as<std::vector<double>>();
                attribute.Set(VtDoubleArray(values.begin(), values.end()));
            } else if (type == SdfValueTypeNames->Double2) {
                const auto v = value.as<std::vector<double>>();
                if (v.size() != 2) {
                    errors_.push_back(where + " takes 2 values");
                    return;
                }
                attribute.Set(GfVec2d(v[0], v[1]));
            } else if (type == SdfValueTypeNames->Double3) {
                const auto v = value.as<std::vector<double>>();
                if (v.size() != 3) {
                    errors_.push_back(where + " takes 3 values");
                    return;
                }
                attribute.Set(GfVec3d(v[0], v[1], v[2]));
            } else {
                errors_.push_back(where + ": values of type " + type.GetAsToken().GetString() +
                                  " cannot be set from an overlay yet");
            }
        } catch (const YAML::Exception& e) {
            errors_.push_back(where + ": " + e.what());
        }
    }

    /// The schemas from the in-memory stage into the model, checked as the stage reader checks
    /// them.
    void collect() {
        std::vector<SchemaInstance> found;
        SchemaCollector collector(
            stage_,
            [this](const UsdPrim& prim) { return worldPrim_.at(prim.GetPath().GetString()); },
            errors_, [this](std::string text) { m_.notes.push_back(std::move(text)); });
        collector.collect(found);
        for (SchemaInstance& s : found) {
            for (const SchemaInstance& existing : m_.world.schemaInstances) {
                if (existing.schema == s.schema && existing.instance == s.instance &&
                    existing.prim == s.prim) {
                    errors_.push_back(std::string(m_.world.prims.paths[s.prim.index]) +
                                      " already has " + s.schema +
                                      (s.instance.empty() ? "" : ":" + s.instance));
                }
            }
        }
        for (SchemaInstance& s : found) {
            m_.world.schemaInstances.push_back(std::move(s));
        }
    }

    /// Prims the overlay made sensors of (SensorAPI_1 and a sensor type) become the world's
    /// sensors, as the stage reader makes them from a USD stage, so that a robot file with an
    /// overlay runs the same as the USD lagrange_import writes from them. IMU, GNSS,
    /// magnetometer and barometer take only attributes; a camera, lidar or radar needs more of
    /// the stage than an overlay holds and is refused with a note to import the robot first.
    void addSensors() {
        for (const UsdPrim& prim : stage_->Traverse()) {
            if (!prim.HasAPI(TfToken("SensorAPI_1"))) {
                continue;
            }
            const std::string path = prim.GetPath().GetString();
            const auto scene = scenePrimOf_.find(path);
            const auto world = worldPrim_.find(path);
            if (scene == scenePrimOf_.end() || world == worldPrim_.end()) {
                continue;
            }
            if (std::ranges::find(m_.world.sensors.prim, world->second) !=
                m_.world.sensors.prim.end()) {
                continue;  // already a sensor (the overlay added schemas to it)
            }
            const auto real = [&prim](const char* name) -> std::optional<double> {
                double v = 0.0;
                const UsdAttribute a = prim.GetAttribute(TfToken(name));
                if (a && a.HasAuthoredValue() && a.Get(&v)) {
                    return v;
                }
                return std::nullopt;
            };
            SensorType type = SensorType::Imu;
            std::map<std::string, double, std::less<>> settings;
            if (prim.HasAPI(TfToken("ImuAPI_1"))) {
                settings = {{"accel_noise", real("sensor:imu:accelNoiseStd").value_or(0.0)},
                            {"gyro_noise", real("sensor:imu:gyroNoiseStd").value_or(0.0)}};
            } else if (prim.HasAPI(TfToken("GnssAPI_1"))) {
                type = SensorType::Gnss;
                settings = {
                    {"horizontal_noise", real("sensor:gnss:horizontalNoiseStd").value_or(0.0)},
                    {"vertical_noise", real("sensor:gnss:verticalNoiseStd").value_or(0.0)},
                    {"velocity_noise", real("sensor:gnss:velocityNoiseStd").value_or(0.0)}};
            } else if (prim.HasAPI(TfToken("MagnetometerAPI_1"))) {
                type = SensorType::Magnetometer;
                settings = {{"noise", real("sensor:magnetometer:noiseStd").value_or(0.0)}};
            } else if (prim.HasAPI(TfToken("BarometerAPI_1"))) {
                type = SensorType::Barometer;
                settings = {{"noise", real("sensor:barometer:noiseStd").value_or(0.0)}};
            } else {
                errors_.push_back(path +
                                  ": an overlay on a robot file makes IMU, GNSS, magnetometer "
                                  "and barometer sensors only; import the robot to USD "
                                  "(lagrange_import ROBOT OUT.usda --overlay FILE) for others");
                continue;
            }
            const std::optional<double> rate = real("sensor:rate");
            if (!rate || *rate <= 0.0) {
                errors_.push_back(path + ": sensor:rate is required and positive");
                continue;
            }
            for (const auto& [attribute, key] :
                 {std::pair{"sensor:latency", "latency"}, std::pair{"sensor:jitter", "jitter"}}) {
                if (const std::optional<double> v = real(attribute); v && *v > 0.0) {
                    settings[key] = *v;
                }
            }
            ScenePrim& at = m_.scene.prims[scene->second];
            if (at.body == kNoIndex) {
                errors_.push_back(path + ": a sensor must be on a link");
                continue;
            }
            // The world's prim for it, by the sensor's own name.
            m_.world.addSensor(prim.GetName().GetString(), type, BodyId{at.body}, at.offset, *rate,
                               settings);
            m_.world.sensors.prim.back() = world->second;
            at.typeName = type == SensorType::Imu            ? "Imu"
                          : type == SensorType::Gnss         ? "Gnss"
                          : type == SensorType::Magnetometer ? "Magnetometer"
                                                             : "Barometer";
            m_.sensorPrim.push_back(scene->second);
        }
    }

    Model& m_;
    std::filesystem::path file_;
    std::string rootPath_;
    UsdStageRefPtr stage_;
    std::map<std::string, PrimId> worldPrim_;           // USD path -> world prim
    std::map<std::string, std::uint32_t> scenePrimOf_;  // USD path -> scene prim
    std::vector<std::string> errors_;
};

}  // namespace

namespace {

Expected<void, std::string> applyRoot(Model& model, const std::string& path,
                                      const YAML::Node& root) {
    Model changed = model;  // the model stays as it was when the overlay is refused
    Overlay overlay(changed, path);
    if (auto status = overlay.apply(root); !status) {
        return Unexpected{path + ": " + status.error()};
    }
    model = std::move(changed);
    return {};
}

}  // namespace

Expected<void, std::string> applyNetlist(Model& model, const std::string& path,
                                         const std::vector<std::string>& directives) {
    registerSchemaLibraries();
    std::ifstream in(path);
    if (!in) {
        return Unexpected{path + ": cannot read it"};
    }
    std::ostringstream text;
    text << in.rdbuf();
    auto overlay = overlayOfNetlist(text.str(), directives);
    if (!overlay) {
        return Unexpected{path + ": " + overlay.error()};
    }
    return applyRoot(model, path, *overlay);
}

Expected<void, std::string> applyOverlay(Model& model, const std::string& path) {
    if (isNetlistPath(path)) {
        return applyNetlist(model, path);
    }
    registerSchemaLibraries();
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        return Unexpected{path + ": " + e.what()};
    }
    return applyRoot(model, path, root);
}

}  // namespace lagrange::stage
