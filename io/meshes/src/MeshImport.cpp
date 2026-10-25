#include "lagrange/meshes/MeshImport.hpp"

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace lagrange::meshes {

namespace {

constexpr float kSmoothingAngleDeg = 30.0F;  // sharper edges keep flat normals (CAD parts)

void note(ImportedMesh& out, std::string text) {
    for (const std::string& existing : out.notes) {
        if (existing == text) {
            return;
        }
    }
    out.notes.push_back(std::move(text));
}

}  // namespace

Expected<ImportedMesh, std::string> importMesh(const std::string& path) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        return Unexpected{"cannot read mesh " + path};
    }
    Assimp::Importer importer;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
    importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, kSmoothingAngleDeg);
    // Points and lines are dropped; only triangles are drawn.
    importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                                aiPrimitiveType_POINT | aiPrimitiveType_LINE);
    const aiScene* scene = importer.ReadFile(
        path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals |
                  aiProcess_PreTransformVertices | aiProcess_SortByPType |
                  aiProcess_FindDegenerates | aiProcess_FindInvalidData);
    if (scene == nullptr || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) != 0U) {
        return Unexpected{"cannot import mesh " + path + ": " + importer.GetErrorString()};
    }

    ImportedMesh out;
    // One part per material, in material order; PreTransformVertices already merged meshes of
    // the same material and baked the node transforms.
    std::vector<int> partOf(scene->mNumMaterials, -1);
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh& mesh = *scene->mMeshes[m];
        if ((mesh.mPrimitiveTypes & aiPrimitiveType_TRIANGLE) == 0U || mesh.mNumFaces == 0) {
            continue;
        }
        if (mesh.mMaterialIndex >= partOf.size()) {
            partOf.resize(mesh.mMaterialIndex + 1, -1);
        }
        int& index = partOf[mesh.mMaterialIndex];
        if (index < 0) {
            index = static_cast<int>(out.parts.size());
            MeshPart part;
            if (mesh.mMaterialIndex < scene->mNumMaterials) {
                const aiMaterial& material = *scene->mMaterials[mesh.mMaterialIndex];
                aiString name;
                if (material.Get(AI_MATKEY_NAME, name) == aiReturn_SUCCESS) {
                    part.material = name.C_Str();
                }
                aiColor4D colour;
                // glTF's base colour first, then the classic diffuse colour (OBJ, Collada).
                if (material.Get(AI_MATKEY_BASE_COLOR, colour) == aiReturn_SUCCESS ||
                    material.Get(AI_MATKEY_COLOR_DIFFUSE, colour) == aiReturn_SUCCESS) {
                    part.rgba = {colour.r, colour.g, colour.b, colour.a};
                    part.hasColor = true;
                }
                float opacity = 1.0F;
                if (material.Get(AI_MATKEY_OPACITY, opacity) == aiReturn_SUCCESS &&
                    opacity < 1.0F) {
                    part.rgba[3] = opacity;
                }
                if (material.GetTextureCount(aiTextureType_DIFFUSE) > 0 ||
                    material.GetTextureCount(aiTextureType_BASE_COLOR) > 0) {
                    note(out, "textures are not imported; textured parts show their colour");
                }
            }
            out.parts.push_back(std::move(part));
        }
        MeshPart& part = out.parts[static_cast<std::size_t>(index)];
        const auto base = static_cast<std::uint32_t>(part.positions.size() / 3);
        for (unsigned v = 0; v < mesh.mNumVertices; ++v) {
            const aiVector3D& p = mesh.mVertices[v];
            part.positions.insert(part.positions.end(), {p.x, p.y, p.z});
            const aiVector3D n = mesh.HasNormals() ? mesh.mNormals[v] : aiVector3D(0, 0, 1);
            part.normals.insert(part.normals.end(), {n.x, n.y, n.z});
        }
        for (unsigned f = 0; f < mesh.mNumFaces; ++f) {
            const aiFace& face = mesh.mFaces[f];
            if (face.mNumIndices != 3) {
                continue;
            }
            part.indices.insert(
                part.indices.end(),
                {base + face.mIndices[0], base + face.mIndices[1], base + face.mIndices[2]});
        }
    }
    if (scene->mNumAnimations > 0) {
        note(out, "animations are not imported");
    }
    if (out.parts.empty()) {
        return Unexpected{"mesh " + path + " has no triangles"};
    }
    return out;
}

Expected<std::string, std::string> resolveMeshUri(const std::string& uri,
                                                  const std::string& baseDirectory) {
    constexpr std::string_view kPackage = "package://";
    constexpr std::string_view kFile = "file://";
    std::filesystem::path path;
    if (uri.starts_with(kPackage)) {
        const std::string rest = uri.substr(kPackage.size());
        const std::size_t slash = rest.find('/');
        const std::string package = rest.substr(0, slash);
        try {
            path = ament_index_cpp::get_package_share_path(package);
        } catch (const std::exception&) {
            return Unexpected{"package '" + package + "' of " + uri + " is not installed"};
        }
        if (slash != std::string::npos) {
            path /= rest.substr(slash + 1);
        }
    } else if (uri.starts_with(kFile)) {
        path = uri.substr(kFile.size());
    } else {
        path = std::filesystem::path(uri);
        if (path.is_relative()) {
            path = std::filesystem::path(baseDirectory) / path;
        }
    }
    return path.lexically_normal().string();
}

}  // namespace lagrange::meshes
