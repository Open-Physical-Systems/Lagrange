// OGC 3D Tiles (Tiles3d.hpp).
#include "lagrange/geospatial/Tiles3d.hpp"
#include "lagrange/meshes/MeshImport.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <jsoncons/json.hpp>

namespace lagrange::geospatial {

namespace {

using Mat4 = std::array<double, 16>;  // column-major, as 3D Tiles writes it

constexpr Mat4 kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
constexpr std::size_t kB3dmHeaderBytes = 28;

Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 out{};
    for (std::size_t c = 0; c < 4; ++c) {
        for (std::size_t r = 0; r < 4; ++r) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 4; ++k) {
                sum += a.at(k * 4 + r) * b.at(c * 4 + k);
            }
            out.at(c * 4 + r) = sum;
        }
    }
    return out;
}

std::array<double, 3> transformPoint(const Mat4& m, const std::array<double, 3>& p) {
    return {m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
            m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
            m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]};
}

std::uint32_t readU32(const std::string& bytes, std::size_t at) {
    std::uint32_t v = 0;
    std::memcpy(&v, bytes.data() + at, sizeof v);
    return v;  // 3D Tiles is little-endian, as this platform (amd64, arm64)
}

/// A content file as a mesh, with the RTC_CENTER of a b3dm (an offset added to its positions).
Expected<std::pair<meshes::ImportedMesh, std::array<double, 3>>, std::string> readContent(
    const std::filesystem::path& file) {
    std::array<double, 3> rtc{};
    std::string ext = file.extension().string();
    std::filesystem::path glb = file;
    std::filesystem::path temporary;
    if (ext == ".b3dm") {
        std::ifstream in(file, std::ios::binary);
        const std::string bytes((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
        if (bytes.size() < kB3dmHeaderBytes || bytes.compare(0, 4, "b3dm") != 0) {
            return Unexpected{file.string() + ": not a b3dm file"};
        }
        const std::uint32_t featureJson = readU32(bytes, 12);
        const std::uint32_t featureBin = readU32(bytes, 16);
        const std::uint32_t batchJson = readU32(bytes, 20);
        const std::uint32_t batchBin = readU32(bytes, 24);
        const std::size_t start =
            kB3dmHeaderBytes + featureJson + featureBin + batchJson + batchBin;
        if (start >= bytes.size()) {
            return Unexpected{file.string() + ": the b3dm holds no glTF"};
        }
        if (featureJson > 0) {
            const auto table = jsoncons::json::parse(bytes.substr(kB3dmHeaderBytes, featureJson));
            if (table.contains("RTC_CENTER") && table["RTC_CENTER"].size() == 3) {
                for (std::size_t i = 0; i < 3; ++i) {
                    rtc.at(i) = table["RTC_CENTER"][i].as<double>();
                }
            }
        }
        temporary =
            std::filesystem::temp_directory_path() /
            ("lagrange_b3dm_" + std::to_string(std::hash<std::string>{}(file.string())) + ".glb");
        std::ofstream(temporary, std::ios::binary) << bytes.substr(start);
        glb = temporary;
    }
    auto mesh = meshes::importMesh(glb.string());
    if (!temporary.empty()) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
    }
    if (!mesh) {
        return Unexpected{mesh.error()};
    }
    return std::pair{std::move(*mesh), rtc};
}

struct Walker {
    const coords::GeoOrigin& origin;
    std::filesystem::path base;
    Tiles3dContent out;

    Expected<void, std::string> tile(const jsoncons::json& t, const Mat4& parent) {
        Mat4 m = parent;
        if (t.contains("transform") && t["transform"].size() == 16) {
            Mat4 local{};
            for (std::size_t i = 0; i < 16; ++i) {
                local.at(i) = t["transform"][i].as<double>();
            }
            m = multiply(parent, local);
        }
        const bool leaf = !t.contains("children") || t["children"].empty();
        if (leaf) {
            std::vector<std::string> uris;
            if (t.contains("content")) {
                const auto& c = t["content"];
                uris.push_back(c.contains("uri") ? c["uri"].as<std::string>()
                                                 : c.get_value_or<std::string>("url", ""));
            }
            if (t.contains("contents")) {
                for (const auto& c : t["contents"].array_range()) {
                    uris.push_back(c.get_value_or<std::string>("uri", ""));
                }
            }
            for (const std::string& uri : uris) {
                if (uri.empty()) {
                    continue;
                }
                const std::filesystem::path file = base / uri;
                if (file.extension() == ".json") {
                    out.notes.push_back(uri + ": external tilesets are not followed");
                    continue;
                }
                auto content = readContent(file);
                if (!content) {
                    return Unexpected{content.error()};
                }
                const auto& [mesh, rtc] = *content;
                out.notes.insert(out.notes.end(), mesh.notes.begin(), mesh.notes.end());
                for (const meshes::MeshPart& part : mesh.parts) {
                    Tiles3dMesh enu;
                    std::array<double, 3> sum{};
                    for (std::size_t i = 0; i + 2 < part.positions.size(); i += 3) {
                        // glTF y-up to 3D Tiles z-up: (x, y, z) -> (x, -z, y).
                        const std::array<double, 3> local{part.positions[i] + rtc[0],
                                                          -part.positions[i + 2] + rtc[1],
                                                          part.positions[i + 1] + rtc[2]};
                        const std::array<double, 3> ecef = transformPoint(m, local);
                        const coords::Enu p = origin.fromEcef({ecef[0], ecef[1], ecef[2]});
                        enu.points.push_back({p.e, p.n, p.u});
                        sum = {sum[0] + p.e, sum[1] + p.n, sum[2] + p.u};
                    }
                    if (enu.points.empty()) {
                        continue;
                    }
                    const auto count = static_cast<double>(enu.points.size());
                    enu.centre = {sum[0] / count, sum[1] / count, sum[2] / count};
                    enu.indices = part.indices;
                    out.meshes.push_back(std::move(enu));
                }
            }
            return {};
        }
        for (const auto& child : t["children"].array_range()) {
            if (auto ok = tile(child, m); !ok) {
                return ok;
            }
        }
        return {};
    }
};

}  // namespace

Expected<Tiles3dContent, std::string> readTiles3d(const std::string& tileset,
                                                  const coords::GeoOrigin& origin) {
    std::ifstream in(tileset);
    if (!in) {
        return Unexpected{"cannot read " + tileset};
    }
    jsoncons::json json;
    try {
        json = jsoncons::json::parse(in);
    } catch (const std::exception& e) {
        return Unexpected{tileset + ": " + e.what()};
    }
    if (!json.contains("root")) {
        return Unexpected{tileset + ": not a 3D Tiles tileset (no root tile)"};
    }
    Walker walker{origin, std::filesystem::path(tileset).parent_path(), {}};
    if (auto ok = walker.tile(json["root"], kIdentity); !ok) {
        return Unexpected{ok.error()};
    }
    return std::move(walker.out);
}

}  // namespace lagrange::geospatial
