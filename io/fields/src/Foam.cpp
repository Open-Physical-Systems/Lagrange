// OpenFOAM ASCII vector fields to a NanoVDB wind grid (Foam.hpp).
#include "lagrange/fields/Foam.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>

namespace lagrange::fields {

namespace {

/// The file's text without // and /* */ comments.
std::string withoutComments(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text.compare(i, 2, "//") == 0) {
            i = text.find('\n', i);
            if (i == std::string::npos) {
                break;
            }
            out += '\n';
        } else if (text.compare(i, 2, "/*") == 0) {
            i = text.find("*/", i + 2);
            if (i == std::string::npos) {
                break;
            }
            ++i;
            out += ' ';
        } else {
            out += text[i];
        }
    }
    return out;
}

/// Splits into tokens, with ( ) { } ; each a token of its own.
std::vector<std::string> tokens(const std::string& text) {
    std::vector<std::string> out;
    std::string current;
    const auto flush = [&] {
        if (!current.empty()) {
            out.push_back(current);
            current.clear();
        }
    };
    for (const char c : text) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            flush();
        } else if (c == '(' || c == ')' || c == '{' || c == '}' || c == ';') {
            flush();
            out.emplace_back(1, c);
        } else {
            current += c;
        }
    }
    flush();
    return out;
}

/// The numbers starting at tokens[i] in "( x y z )"; advances i past it.
Expected<Vec3, std::string> vectorAt(const std::vector<std::string>& t, std::size_t& i) {
    if (i + 4 >= t.size() || t[i] != "(" || t[i + 4] != ")") {
        return Unexpected{std::string("expected a vector (x y z)")};
    }
    Vec3 v{};
    for (std::size_t k = 0; k < 3; ++k) {
        try {
            std::size_t used = 0;
            v[k] = std::stod(t[i + 1 + k], &used);
            if (used != t[i + 1 + k].size()) {
                return Unexpected{"not a number: " + t[i + 1 + k]};
            }
        } catch (const std::exception&) {
            return Unexpected{"not a number: " + t[i + 1 + k]};
        }
    }
    i += 5;
    return v;
}

}  // namespace

Expected<std::vector<Vec3>, std::string> readFoamVectorField(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return Unexpected{path + ": cannot be read"};
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::vector<std::string> t = tokens(withoutComments(buffer.str()));
    const auto fail = [&path](const std::string& why) { return Unexpected{path + ": " + why}; };
    // The header: FoamFile { ... format ascii; class volVectorField; ... }
    const auto format = std::find(t.begin(), t.end(), "format");
    if (format != t.end() && std::next(format) != t.end() && *std::next(format) != "ascii") {
        return fail("binary OpenFOAM files are not read; write the case with writeFormat ascii");
    }
    const auto cls = std::find(t.begin(), t.end(), "class");
    if (cls == t.end() || std::next(cls) == t.end() || *std::next(cls) != "volVectorField") {
        return fail("not a volVectorField");
    }
    const auto field = std::find(t.begin(), t.end(), "internalField");
    if (field == t.end()) {
        return fail("no internalField");
    }
    std::size_t i = static_cast<std::size_t>(field - t.begin()) + 1;
    if (i < t.size() && t[i] == "uniform") {
        ++i;
        auto v = vectorAt(t, i);
        if (!v) {
            return fail(v.error());
        }
        return std::vector<Vec3>{*v};
    }
    if (i + 1 >= t.size() || t[i] != "nonuniform" || t[i + 1] != "List<vector>") {
        return fail("internalField must be uniform or nonuniform List<vector>");
    }
    i += 2;
    std::size_t count = 0;
    try {
        count = std::stoul(t.at(i));
    } catch (const std::exception&) {
        return fail("expected the number of cells after List<vector>");
    }
    ++i;
    std::vector<Vec3> out;
    if (i < t.size() && t[i] == "{") {
        // N{(x y z)}: every cell the same.
        ++i;
        auto v = vectorAt(t, i);
        if (!v || i >= t.size() || t[i] != "}") {
            return fail("expected N{(x y z)}");
        }
        out.assign(count, *v);
        return out;
    }
    if (i >= t.size() || t[i] != "(") {
        return fail("expected ( after the number of cells");
    }
    ++i;
    out.reserve(count);
    for (std::size_t c = 0; c < count; ++c) {
        auto v = vectorAt(t, i);
        if (!v) {
            return fail("cell " + std::to_string(c) + ": " + v.error());
        }
        out.push_back(*v);
    }
    if (i >= t.size() || t[i] != ")") {
        return fail("more values than the " + std::to_string(count) + " cells it announces");
    }
    return out;
}

Expected<FoamImport, std::string> foamToWindField(const std::vector<Vec3>& centres,
                                                  const std::vector<Vec3>& velocity,
                                                  const std::string& path, const std::string& name,
                                                  double voxelSize, const Vec3& offset) {
    if (voxelSize <= 0.0) {
        return Unexpected{std::string("the voxel size must be positive")};
    }
    if (centres.empty()) {
        return Unexpected{std::string("no cells")};
    }
    if (velocity.size() != centres.size() && velocity.size() != 1) {
        return Unexpected{"U has " + std::to_string(velocity.size()) + " cells and C " +
                          std::to_string(centres.size())};
    }
    struct Sum {
        Vec3 v{};
        int n{0};
    };
    std::map<std::array<std::int64_t, 3>, Sum> voxels;
    for (std::size_t c = 0; c < centres.size(); ++c) {
        std::array<std::int64_t, 3> key{};
        for (std::size_t k = 0; k < 3; ++k) {
            key[k] = static_cast<std::int64_t>(std::floor((centres[c][k] + offset[k]) / voxelSize));
        }
        Sum& s = voxels[key];
        const Vec3& u = velocity.size() == 1 ? velocity[0] : velocity[c];
        for (std::size_t k = 0; k < 3; ++k) {
            s.v[k] += u[k];
        }
        ++s.n;
    }
    std::array<std::int64_t, 3> lo = voxels.begin()->first;
    std::array<std::int64_t, 3> hi = lo;
    for (const auto& [key, sum] : voxels) {
        for (std::size_t k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], key[k]);
            hi[k] = std::max(hi[k], key[k]);
        }
    }
    DenseBox box;
    box.size = voxelSize;
    for (std::size_t k = 0; k < 3; ++k) {
        box.origin[k] = (static_cast<double>(lo[k]) + 0.5) * voxelSize;
        box.count[k] = static_cast<int>(hi[k] - lo[k] + 1);
    }
    auto written = writeVectorField(path, name, box, [&](const Vec3& at) -> Vec3 {
        std::array<std::int64_t, 3> key{};
        for (std::size_t k = 0; k < 3; ++k) {
            key[k] = static_cast<std::int64_t>(std::floor(at[k] / voxelSize));
        }
        const auto it = voxels.find(key);
        if (it == voxels.end()) {
            return {};
        }
        const double n = it->second.n;
        return {it->second.v[0] / n, it->second.v[1] / n, it->second.v[2] / n};
    });
    if (!written) {
        return Unexpected{written.error()};
    }
    FoamImport report;
    report.cells = centres.size();
    report.voxels = voxels.size();
    for (std::size_t k = 0; k < 3; ++k) {
        report.min[k] = box.origin[k];
        report.max[k] = (static_cast<double>(hi[k]) + 0.5) * voxelSize;
    }
    return report;
}

}  // namespace lagrange::fields
