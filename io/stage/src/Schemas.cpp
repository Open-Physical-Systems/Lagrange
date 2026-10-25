// Applied schemas of the project's libraries -> World::schemaInstances (RFC 0002, "Loading").
#include "Schemas.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <string_view>
#include <utility>

#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/primDefinition.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/tokens.h>
#include <pxr/usd/usdPhysics/joint.h>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace lagrange::stage {

using namespace pxr;  // NOLINT(google-build-using-namespace): OpenUSD names are long enough

namespace {

// Property namespaces of the project's schema libraries (RFC 0002 convention 2): an applied
// schema whose properties live in one of them is collected into World::schemaInstances.
constexpr std::array<std::string_view, 13> kSchemaNamespaces{
    "elec:",   "thermal:", "aero:",  "weather:", "emi:",     "rad:",  "sensmat:",
    "sensor:", "world:",   "comms:", "can:",     "vehicle:", "wheel:"};

// Loader checks of RFC 0002 ("Loading") that the schema definitions cannot express.
struct PairedArrays {
    std::string_view schema, first, second;
};
constexpr std::array<PairedArrays, 1> kPairedArrays{
    {{"BatteryAPI_1", "elec:battery:ocvSoc", "elec:battery:ocvVoltage"}}};
struct Fraction {
    std::string_view schema, attribute;
};
constexpr std::array<Fraction, 3> kFractions{
    {{"BatteryAPI_1", "elec:battery:initialSoc"},
     {"ElectricalMotorAPI_1", "elec:gearEfficiency"},
     {"ElectricalConverterAPI_1", "elec:conv:efficiency"}}};
struct JointTarget {
    std::string_view schema, relationship;
};
constexpr std::array<JointTarget, 1> kJointTargets{{{"ElectricalMotorAPI_1", "elec:joint"}}};

}  // namespace

void registerSchemaLibraries() {
    static std::once_flag once;
    std::call_once(once, [] {
        PlugRegistry& plugins = PlugRegistry::GetInstance();
        if (plugins.GetPluginWithName("lagrangeMetadata")) {
            return;  // registered through PXR_PLUGINPATH_NAME
        }
        std::filesystem::path share;
        try {
            share = ament_index_cpp::get_package_share_path("lagrange_schemas");
        } catch (const std::exception&) {
            return;  // not installed: the schemas stay unknown and the loader says so
        }
        std::vector<std::string> paths;
        std::error_code error;
        for (const auto& entry : std::filesystem::directory_iterator(share / "usd", error)) {
            if (std::filesystem::exists(entry.path() / "plugInfo.json")) {
                paths.push_back(entry.path().string());
            }
        }
        plugins.RegisterPlugins(paths);
    });
}

std::optional<SchemaAttribute> schemaAttribute(const std::string& schema,
                                               const std::string& instance,
                                               const std::string& attribute) {
    registerSchemaLibraries();
    const UsdPrimDefinition* definition =
        UsdSchemaRegistry::GetInstance().FindAppliedAPIPrimDefinition(TfToken(schema));
    if (definition == nullptr) {
        return std::nullopt;
    }
    // A multiple-apply schema's properties are templates: "thermal:link:__INSTANCE_NAME__:...".
    std::string name = attribute;
    if (!instance.empty()) {
        const std::string named = ":" + instance + ":";
        if (const std::size_t at = name.find(named); at != std::string::npos) {
            name = UsdSchemaRegistry::MakeMultipleApplyNameTemplate(name.substr(0, at),
                                                                    name.substr(at + named.size()))
                       .GetString();
        }
    }
    UsdPrimDefinition::Attribute property = definition->GetAttributeDefinition(TfToken(name));
    if (!property) {
        return std::nullopt;
    }
    SchemaAttribute out;
    out.type = property.GetTypeName().GetAsToken().GetString();
    out.doc = property.GetDocumentation();
    VtValue unit;
    if (property.GetMetadata(TfToken("lagrangeUnit"), &unit) && unit.IsHolding<std::string>()) {
        out.unit = unit.UncheckedGet<std::string>();
        if (out.unit == "1") {
            out.unit.clear();
        }
    }
    return out;
}

std::optional<std::string> checkSchemaReal(const std::string& schema, const std::string& attribute,
                                           double value) {
    if (!std::isfinite(value)) {
        return attribute + " must be a finite number";
    }
    for (const Fraction& rule : kFractions) {
        if (schema == rule.schema && attribute == rule.attribute &&
            !(value >= 0.0 && value <= 1.0)) {
            return attribute + " is " + std::to_string(value) + "; it must be within 0 to 1";
        }
    }
    return std::nullopt;
}

SchemaCollector::SchemaCollector(UsdStageRefPtr stage, WorldPrimOf worldPrimOf,
                                 std::vector<std::string>& errors, Note note)
    : stage_(std::move(stage))
    , worldPrimOf_(std::move(worldPrimOf))
    , errors_(errors)
    , note_(std::move(note)) {}

void SchemaCollector::collect(std::vector<SchemaInstance>& out) {
    const UsdSchemaRegistry& registry = UsdSchemaRegistry::GetInstance();
    for (const UsdPrim& prim : stage_->Traverse()) {
        // GetAppliedSchemas() leaves out schemas USD does not know; the authored list has
        // them.
        SdfTokenListOp authored;
        TfTokenVector names;
        if (prim.GetMetadata(UsdTokens->apiSchemas, &authored)) {
            authored.ApplyOperations(&names);
        }
        for (const TfToken& applied : names) {
            const auto [schema, instance] = UsdSchemaRegistry::GetTypeNameAndInstance(applied);
            const UsdPrimDefinition* definition = registry.FindAppliedAPIPrimDefinition(schema);
            if (definition == nullptr) {
                if (ours(schema.GetString())) {
                    note_(prim.GetPath().GetString() + ": " + applied.GetString() +
                          " is not a registered schema here; its values are not read");
                }
                continue;
            }
            if (!ours(*definition)) {
                continue;
            }
            SchemaInstance s;
            s.schema = schema.GetString();
            s.instance = instance.GetString();
            s.prim = worldPrimOf_(prim);
            for (const TfToken& property : definition->GetPropertyNames()) {
                const TfToken name = instance.IsEmpty()
                                         ? property
                                         : UsdSchemaRegistry::MakeMultipleApplyNameInstance(
                                               property.GetString(), instance.GetString());
                readProperty(prim, *definition, property, name, s);
            }
            check(prim, s);
            out.push_back(std::move(s));
        }
    }
}

bool SchemaCollector::ours(const std::string& name) {
    return name.find("API_") != std::string::npos && name.rfind("Physics", 0) != 0;
}

bool SchemaCollector::ours(const UsdPrimDefinition& definition) {
    for (const TfToken& property : definition.GetPropertyNames()) {
        const std::string_view text = property.GetString();
        for (const std::string_view space : kSchemaNamespaces) {
            if (text.starts_with(space)) {
                return true;
            }
        }
    }
    return false;
}

void SchemaCollector::readProperty(const UsdPrim& prim, const UsdPrimDefinition& definition,
                                   const TfToken& property, const TfToken& name,
                                   SchemaInstance& s) {
    const std::string where = prim.GetPath().GetString() + ": " + name.GetString();
    if (const UsdRelationship rel = prim.GetRelationship(name)) {
        SdfPathVector targets;
        rel.GetForwardedTargets(&targets);
        std::vector<PrimId> ids;
        for (const SdfPath& target : targets) {
            const UsdPrim targetPrim = stage_->GetPrimAtPath(target.GetPrimPath());
            if (!targetPrim) {
                errors_.get().push_back(where + " targets " + target.GetString() +
                                        ", which is not in the stage");
                continue;
            }
            ids.push_back(worldPrimOf_(targetPrim));
        }
        if (!ids.empty()) {
            s.relationships.emplace(name.GetString(), std::move(ids));
        }
        return;
    }
    const UsdAttribute attribute = prim.GetAttribute(name);
    VtValue value;
    if (!attribute || !attribute.Get(&value) || value.IsEmpty()) {
        return;  // no fallback and not authored: the module says whether it is required
    }
    const std::string key = name.GetString();
    if (value.IsHolding<double>()) {
        s.reals.emplace(key, value.UncheckedGet<double>());
    } else if (value.IsHolding<int>()) {
        s.integers.emplace(key, value.UncheckedGet<int>());
    } else if (value.IsHolding<bool>()) {
        s.integers.emplace(key, value.UncheckedGet<bool>() ? 1 : 0);
    } else if (value.IsHolding<TfToken>()) {
        const TfToken token = value.UncheckedGet<TfToken>();
        VtTokenArray allowed;
        if (definition.GetPropertyMetadata(property, SdfFieldKeys->AllowedTokens, &allowed) &&
            !allowed.empty() && std::find(allowed.begin(), allowed.end(), token) == allowed.end()) {
            std::string list;
            for (const TfToken& a : allowed) {
                list += (list.empty() ? "" : ", ") + a.GetString();
            }
            errors_.get().push_back(where + " is \"" + token.GetString() + "\"; allowed: " + list);
        }
        s.tokens.emplace(key, token.GetString());
    } else if (value.IsHolding<std::string>()) {
        s.tokens.emplace(key, value.UncheckedGet<std::string>());
    } else if (value.IsHolding<SdfAssetPath>()) {
        const SdfAssetPath& asset = value.UncheckedGet<SdfAssetPath>();
        if (asset.GetAssetPath().empty()) {
            return;
        }
        if (asset.GetResolvedPath().empty()) {
            errors_.get().push_back(where + " names " + asset.GetAssetPath() +
                                    ", which cannot be found");
            return;
        }
        s.assetPaths.emplace(key, asset.GetResolvedPath());
    } else if (value.IsHolding<VtDoubleArray>()) {
        const VtDoubleArray& array = value.UncheckedGet<VtDoubleArray>();
        s.realArrays.emplace(key, std::vector<double>(array.begin(), array.end()));
    } else if (value.IsHolding<GfVec2d>()) {
        const GfVec2d& v = value.UncheckedGet<GfVec2d>();
        s.realArrays.emplace(key, std::vector<double>{v[0], v[1]});
    } else if (value.IsHolding<GfVec3d>()) {
        const GfVec3d& v = value.UncheckedGet<GfVec3d>();
        s.realArrays.emplace(key, std::vector<double>{v[0], v[1], v[2]});
    } else {
        note_(where + ": values of type " + attribute.GetTypeName().GetAsToken().GetString() +
              " are not read yet");
    }
}

void SchemaCollector::check(const UsdPrim& prim, const SchemaInstance& s) {
    const std::string path = prim.GetPath().GetString();
    for (const PairedArrays& rule : kPairedArrays) {
        if (s.schema == rule.schema &&
            s.realArray(rule.first).size() != s.realArray(rule.second).size()) {
            errors_.get().push_back(path + ": " + std::string(rule.first) + " and " +
                                    std::string(rule.second) + " differ in length (" +
                                    std::to_string(s.realArray(rule.first).size()) + " and " +
                                    std::to_string(s.realArray(rule.second).size()) + ")");
        }
    }
    for (const Fraction& rule : kFractions) {
        const std::optional<double> value = s.real(rule.attribute);
        if (s.schema == rule.schema && value && !(*value >= 0.0 && *value <= 1.0)) {
            errors_.get().push_back(path + ": " + std::string(rule.attribute) + " is " +
                                    std::to_string(*value) + "; it must be within 0 to 1");
        }
    }
    for (const JointTarget& rule : kJointTargets) {
        if (s.schema != rule.schema) {
            continue;
        }
        const UsdRelationship rel = prim.GetRelationship(TfToken(std::string(rule.relationship)));
        SdfPathVector targets;
        if (rel) {
            rel.GetForwardedTargets(&targets);
        }
        for (const SdfPath& target : targets) {
            const UsdPrim joint = stage_->GetPrimAtPath(target);
            if (joint && !joint.IsA<UsdPhysicsJoint>()) {
                errors_.get().push_back(path + ": " + std::string(rule.relationship) + " targets " +
                                        target.GetString() + ", which is not a joint");
            }
        }
    }
}

}  // namespace lagrange::stage
