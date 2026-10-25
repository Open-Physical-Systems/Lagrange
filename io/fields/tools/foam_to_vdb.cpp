// lagrange_foam_to_vdb CASE/TIME/C CASE/TIME/U OUT.nvdb [--voxel M] [--offset X,Y,Z] [--name N]
//
// An OpenFOAM case's velocities as a NanoVDB wind field (Foam.hpp): C from
// `postProcess -func writeCellCentres -time TIME`, U from the same time directory, both ASCII.
// The result goes to the weather domain as --weather wind_field=OUT.nvdb or the profile's
// weather.wind_field.
#include "lagrange/fields/Foam.hpp"

#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char** argv) {
    using lagrange::fields::Vec3;
    std::vector<std::string> positional;
    double voxel = 1.0;
    Vec3 offset{};
    std::string name = "wind";
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--voxel" && i + 1 < argc) {
            voxel = std::stod(argv[++i]);
        } else if (arg == "--offset" && i + 1 < argc) {
            if (std::sscanf(argv[++i], "%lf,%lf,%lf", &offset[0], &offset[1], &offset[2]) != 3) {
                std::fprintf(stderr, "--offset takes X,Y,Z (m)\n");
                return 2;
            }
        } else if (arg == "--name" && i + 1 < argc) {
            name = argv[++i];
        } else if (arg.starts_with("--")) {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        } else {
            positional.emplace_back(arg);
        }
    }
    if (positional.size() != 3) {
        std::fprintf(stderr,
                     "usage: lagrange_foam_to_vdb CASE/TIME/C CASE/TIME/U OUT.nvdb [--voxel M]\n"
                     "       [--offset X,Y,Z] [--name NAME]\n"
                     "  C: cell centres (postProcess -func writeCellCentres), U: velocity; ASCII\n"
                     "  --voxel   voxel size, m (default 1)\n"
                     "  --offset  added to the case's coordinates to give region-local ones, m\n");
        return 2;
    }
    const auto centres = lagrange::fields::readFoamVectorField(positional[0]);
    if (!centres) {
        std::fprintf(stderr, "%s\n", centres.error().c_str());
        return 1;
    }
    const auto velocity = lagrange::fields::readFoamVectorField(positional[1]);
    if (!velocity) {
        std::fprintf(stderr, "%s\n", velocity.error().c_str());
        return 1;
    }
    const auto report =
        lagrange::fields::foamToWindField(*centres, *velocity, positional[2], name, voxel, offset);
    if (!report) {
        std::fprintf(stderr, "%s\n", report.error().c_str());
        return 1;
    }
    std::printf("%s: %zu cells in %zu voxels of %g m, from (%g, %g, %g) to (%g, %g, %g)\n",
                positional[2].c_str(), report->cells, report->voxels, voxel, report->min[0],
                report->min[1], report->min[2], report->max[0], report->max[1], report->max[2]);
    return 0;
}
