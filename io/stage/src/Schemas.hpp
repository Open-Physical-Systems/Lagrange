#pragma once

// Reading the project's applied schemas from a stage, shared by the stage reader and the overlay
// files.
#include "lagrange/world/World.hpp"

#include <functional>
#include <string>
#include <vector>

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primDefinition.h>
#include <pxr/usd/usd/stage.h>

namespace lagrange::stage {

/// Makes the project's schema libraries known to USD when the environment does not
/// (PXR_PLUGINPATH_NAME is set by lagrange_schemas' environment hook only in a sourced
/// workspace): every library installed under lagrange_schemas' share/usd, found through the
/// ament index. It must run before USD's schema registry is first built, which does not see
/// plugins registered later, so every entry point of lagrange_stage calls it first.
void registerSchemaLibraries();

/// Collects every applied schema of the project's libraries (properties in the namespaces of
/// RFC 0002 convention 2) into SchemaInstance rows, values and fallbacks read once (invariant 5),
/// and applies RFC 0002's loader checks. Problems are appended to `errors` with their prims.
class SchemaCollector {
public:
    /// The world prim standing for a USD prim (an instance's own prim, a relationship target).
    using WorldPrimOf = std::function<PrimId(const pxr::UsdPrim&)>;
    using Note = std::function<void(std::string)>;

    SchemaCollector(pxr::UsdStageRefPtr stage, WorldPrimOf worldPrimOf,
                    std::vector<std::string>& errors, Note note);

    /// Every applied schema of ours on every prim of the stage, appended to `out`.
    void collect(std::vector<SchemaInstance>& out);

private:
    /// Whether `name` looks like one of ours though its library is not loaded: the families
    /// of RFC 0002's schema set end in "API_<N>" and are not USD's own.
    static bool ours(const std::string& name);
    static bool ours(const pxr::UsdPrimDefinition& definition);
    void readProperty(const pxr::UsdPrim& prim, const pxr::UsdPrimDefinition& definition,
                      const pxr::TfToken& property, const pxr::TfToken& name, SchemaInstance& s);
    void check(const pxr::UsdPrim& prim, const SchemaInstance& s);

    pxr::UsdStageRefPtr stage_;
    WorldPrimOf worldPrimOf_;
    std::reference_wrapper<std::vector<std::string>> errors_;
    Note note_;
};

}  // namespace lagrange::stage
