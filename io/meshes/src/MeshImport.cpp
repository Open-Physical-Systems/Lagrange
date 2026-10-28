#include "Image.hpp"
#include "lagrange/meshes/MeshImport.hpp"

#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <string_view>

#include <ament_index_cpp/get_package_share_path.hpp>

namespace lagrange::meshes {

namespace {

constexpr float kSmoothingAngleDeg = 30.0F;  // sharper edges keep flat normals (CAD parts)
// A textured part becomes this many parts of one colour each: textures are not drawn, so each
// triangle takes its texture's colour and the triangles are grouped by k-means on colour.
constexpr int kTextureColours = 16;
constexpr int kKMeansIterations = 10;

void note(ImportedMesh& out, std::string text) {
    for (const std::string& existing : out.notes) {
        if (existing == text) {
            return;
        }
    }
    out.notes.push_back(std::move(text));
}

/// The base colour (glTF) or diffuse (OBJ, Collada) texture of a material, decoded: embedded in
/// the scene ("*N"), or a file next to the mesh (by its path, then by its file name alone).
std::optional<detail::Image> textureOf(const aiScene& scene, const aiMaterial& material,
                                       const std::filesystem::path& meshDirectory) {
    aiString name;
    if (material.GetTexture(aiTextureType_BASE_COLOR, 0, &name) != aiReturn_SUCCESS &&
        material.GetTexture(aiTextureType_DIFFUSE, 0, &name) != aiReturn_SUCCESS) {
        return std::nullopt;
    }
    if (const aiTexture* embedded = scene.GetEmbeddedTexture(name.C_Str())) {
        if (embedded->mHeight == 0) {  // compressed: mWidth bytes of PNG or JPEG
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(embedded->pcData);
            return detail::decodeImage({bytes, embedded->mWidth});
        }
        detail::Image image;
        image.width = static_cast<int>(embedded->mWidth);
        image.height = static_cast<int>(embedded->mHeight);
        image.rgb.reserve(static_cast<std::size_t>(image.width) * image.height * 3);
        for (unsigned i = 0; i < embedded->mWidth * embedded->mHeight; ++i) {
            const aiTexel& t = embedded->pcData[i];
            image.rgb.insert(image.rgb.end(), {t.r, t.g, t.b});
        }
        return image;
    }
    std::string file = name.C_Str();
    if (file.starts_with("file://")) {
        file = file.substr(7);
    }
    const std::filesystem::path given(file);
    for (const std::filesystem::path& candidate :
         {given.is_absolute() ? given : meshDirectory / given, meshDirectory / given.filename()}) {
        if (auto image = detail::readImage(candidate.string())) {
            return image;
        }
    }
    return std::nullopt;
}

using Rgb = std::array<float, 3>;

/// The texture's colour under a point of the mesh (nearest texel, repeating, v up).
Rgb sample(const detail::Image& image, const aiVector3D& uv) {
    const auto wrap = [](float x) { return x - std::floor(x); };
    const int x =
        std::min(image.width - 1, static_cast<int>(wrap(uv.x) * static_cast<float>(image.width)));
    const int y = std::min(
        image.height - 1, static_cast<int>((1.0F - wrap(uv.y)) * static_cast<float>(image.height)));
    const std::size_t at = (static_cast<std::size_t>(y) * image.width + x) * 3;
    constexpr float kByte = 255.0F;
    return {image.rgb[at] / kByte, image.rgb[at + 1] / kByte, image.rgb[at + 2] / kByte};
}

float distance2(const Rgb& a, const Rgb& b) {
    const float dr = a[0] - b[0];
    const float dg = a[1] - b[1];
    const float db = a[2] - b[2];
    return dr * dr + dg * dg + db * db;
}

/// Groups colours by k-means (deterministic: the centres start at luminance quantiles) and
/// returns each colour's group; `centres` receives the groups' mean colours.
std::vector<int> cluster(const std::vector<Rgb>& colours, int k, std::vector<Rgb>& centres) {
    std::vector<std::size_t> order(colours.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    const auto luminance = [&](std::size_t i) {
        return 0.2126F * colours[i][0] + 0.7152F * colours[i][1] + 0.0722F * colours[i][2];
    };
    std::stable_sort(order.begin(), order.end(),
                     [&](std::size_t a, std::size_t b) { return luminance(a) < luminance(b); });
    centres.clear();
    for (int c = 0; c < k; ++c) {
        centres.push_back(colours[order[(order.size() - 1) * (2 * c + 1) / (2 * k)]]);
    }
    std::vector<int> group(colours.size(), 0);
    for (int iteration = 0; iteration < kKMeansIterations; ++iteration) {
        std::vector<std::array<double, 4>> sums(centres.size(), {0, 0, 0, 0});
        for (std::size_t i = 0; i < colours.size(); ++i) {
            int best = 0;
            for (int c = 1; c < static_cast<int>(centres.size()); ++c) {
                if (distance2(colours[i], centres[c]) < distance2(colours[i], centres[best])) {
                    best = c;
                }
            }
            group[i] = best;
            auto& sum = sums[static_cast<std::size_t>(best)];
            sum = {sum[0] + colours[i][0], sum[1] + colours[i][1], sum[2] + colours[i][2],
                   sum[3] + 1};
        }
        for (std::size_t c = 0; c < centres.size(); ++c) {
            if (sums[c][3] > 0) {
                centres[c] = {static_cast<float>(sums[c][0] / sums[c][3]),
                              static_cast<float>(sums[c][1] / sums[c][3]),
                              static_cast<float>(sums[c][2] / sums[c][3])};
            }
        }
    }
    return group;
}

/// A triangle mesh with a texture as parts of one colour each (kTextureColours at most).
void addTextured(const aiMesh& mesh, const detail::Image& image, const MeshPart& like,
                 ImportedMesh& out) {
    std::vector<std::array<unsigned, 3>> faces;
    std::vector<Rgb> colours;
    for (unsigned f = 0; f < mesh.mNumFaces; ++f) {
        const aiFace& face = mesh.mFaces[f];
        if (face.mNumIndices != 3) {
            continue;
        }
        const std::array<unsigned, 3> v{face.mIndices[0], face.mIndices[1], face.mIndices[2]};
        const aiVector3D* uv = mesh.mTextureCoords[0];
        const aiVector3D centre = (uv[v[0]] + uv[v[1]] + uv[v[2]]) / 3.0F;
        Rgb colour{0, 0, 0};
        for (const aiVector3D& at : {uv[v[0]], uv[v[1]], uv[v[2]], centre}) {
            const Rgb texel = sample(image, at);
            for (std::size_t i = 0; i < 3; ++i) {
                colour.at(i) += texel.at(i) * like.rgba.at(i) / 4.0F;
            }
        }
        faces.push_back(v);
        colours.push_back(colour);
    }
    if (faces.empty()) {
        return;
    }
    std::vector<Rgb> centres;
    const std::vector<int> group =
        cluster(colours, std::min<int>(kTextureColours, static_cast<int>(faces.size())), centres);
    for (std::size_t c = 0; c < centres.size(); ++c) {
        MeshPart part;
        part.material = like.material + "#" + std::to_string(c);
        part.rgba = {centres[c][0], centres[c][1], centres[c][2], like.rgba[3]};
        part.hasColor = true;
        std::vector<std::uint32_t> remap(mesh.mNumVertices, UINT32_MAX);
        for (std::size_t f = 0; f < faces.size(); ++f) {
            if (group[f] != static_cast<int>(c)) {
                continue;
            }
            for (const unsigned v : faces[f]) {
                if (remap[v] == UINT32_MAX) {
                    remap[v] = static_cast<std::uint32_t>(part.positions.size() / 3);
                    const aiVector3D& p = mesh.mVertices[v];
                    part.positions.insert(part.positions.end(), {p.x, p.y, p.z});
                    const aiVector3D n = mesh.HasNormals() ? mesh.mNormals[v] : aiVector3D(0, 0, 1);
                    part.normals.insert(part.normals.end(), {n.x, n.y, n.z});
                }
                part.indices.push_back(remap[v]);
            }
        }
        if (!part.indices.empty()) {
            out.parts.push_back(std::move(part));
        }
    }
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
            }
            out.parts.push_back(std::move(part));
        }
        // A texture: its colours, baked into parts of one colour each (textures are not drawn).
        if (mesh.mMaterialIndex < scene->mNumMaterials) {
            const aiMaterial& material = *scene->mMaterials[mesh.mMaterialIndex];
            if (material.GetTextureCount(aiTextureType_DIFFUSE) > 0 ||
                material.GetTextureCount(aiTextureType_BASE_COLOR) > 0) {
                std::optional<detail::Image> image;
                if (mesh.HasTextureCoords(0)) {
                    image = textureOf(*scene, material, std::filesystem::path(path).parent_path());
                }
                if (image) {
                    const MeshPart like = out.parts[static_cast<std::size_t>(index)];
                    addTextured(mesh, *image, like, out);
                    note(out, "textures are baked into parts of " +
                                  std::to_string(kTextureColours) + " colours at most");
                    continue;
                }
                note(out, "textures are not imported; textured parts show their colour");
            }
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
    // Parts a texture replaced keep no triangles.
    std::erase_if(out.parts, [](const MeshPart& part) { return part.indices.empty(); });
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
    constexpr std::string_view kAsset = "lagrange-asset://";
    std::filesystem::path path;
    if (uri.starts_with(kAsset)) {
        // A fetched asset (assets/external_assets.yaml): $LAGRANGE_ASSET_DIR, by default
        // $XDG_DATA_HOME/lagrange/assets (~/.local/share/lagrange/assets).
        if (const char* dir = std::getenv("LAGRANGE_ASSET_DIR"); dir != nullptr && *dir != 0) {
            path = dir;
        } else if (const char* data = std::getenv("XDG_DATA_HOME"); data != nullptr && *data != 0) {
            path = std::filesystem::path(data) / "lagrange" / "assets";
        } else if (const char* home = std::getenv("HOME"); home != nullptr) {
            path = std::filesystem::path(home) / ".local" / "share" / "lagrange" / "assets";
        }
        path /= uri.substr(kAsset.size());
        if (!std::filesystem::is_regular_file(path)) {
            return Unexpected{uri + " is not fetched (" + path.string() +
                              "): run `ros2 run lagrange_assets fetch_assets`"};
        }
    } else if (uri.starts_with(kPackage)) {
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
