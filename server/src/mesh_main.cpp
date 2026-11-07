// lagrange_mesh_to_obj URI BASE_DIR SCALE_X,SCALE_Y,SCALE_Z OUT.obj: one mesh file, found as the
// loader finds it (package://, file:// or relative to BASE_DIR), as one OBJ for collision_bake.
#include "lagrange/meshes/MeshImport.hpp"

#include <cstdio>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    double scale[3] = {1.0, 1.0, 1.0};
    if (argc != 5 || std::sscanf(argv[3], "%lf,%lf,%lf", &scale[0], &scale[1], &scale[2]) != 3) {
        std::fprintf(stderr, "usage: lagrange_mesh_to_obj URI BASE_DIR SX,SY,SZ OUT.obj\n");
        return 2;
    }
    const auto path = lagrange::meshes::resolveMeshUri(argv[1], argv[2]);
    if (!path) {
        std::fprintf(stderr, "%s\n", path.error().c_str());
        return 1;
    }
    const auto mesh = lagrange::meshes::importMesh(*path);
    if (!mesh) {
        std::fprintf(stderr, "%s\n", mesh.error().c_str());
        return 1;
    }
    std::ofstream out(argv[4]);
    std::size_t first = 1;
    for (const auto& part : mesh->parts) {
        for (std::size_t i = 0; i + 2 < part.positions.size(); i += 3) {
            out << "v " << part.positions[i] * scale[0] << ' ' << part.positions[i + 1] * scale[1]
                << ' ' << part.positions[i + 2] * scale[2] << '\n';
        }
        for (std::size_t i = 0; i + 2 < part.indices.size(); i += 3) {
            out << "f " << first + part.indices[i] << ' ' << first + part.indices[i + 1] << ' '
                << first + part.indices[i + 2] << '\n';
        }
        first += part.positions.size() / 3;
    }
    return out ? 0 : 1;
}
