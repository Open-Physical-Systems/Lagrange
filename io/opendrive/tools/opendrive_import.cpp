// lagrange_opendrive_import IN.xodr OUT.usda [--root NAME] [--step M]
//
// An OpenDRIVE road network as a USD layer (OpenDrive.hpp): road surfaces and marks to draw
// (the roads lie on the ground plane, which the loader adds) and RoadLaneAPI_1 lanes for
// traffic. Compose it with a world's other layers by sublayering.
#include "lagrange/opendrive/OpenDrive.hpp"
#include "lagrange/stage/Stage.hpp"

#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char** argv) {
    std::vector<std::string> positional;
    lagrange::opendrive::ConvertOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--root" && i + 1 < argc) {
            options.root = argv[++i];
        } else if (arg == "--step" && i + 1 < argc) {
            options.step = std::stod(argv[++i]);
        } else if (arg.starts_with("--")) {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        } else {
            positional.emplace_back(arg);
        }
    }
    if (positional.size() != 2) {
        std::fprintf(stderr,
                     "usage: lagrange_opendrive_import IN.xodr OUT.usda [--root NAME] [--step M]\n"
                     "  --root  the layer's top prim (default roads)\n"
                     "  --step  sample spacing along the roads, m (default 0.5)\n");
        return 2;
    }
    const auto network = lagrange::opendrive::read(positional[0]);
    if (!network) {
        std::fprintf(stderr, "%s\n", network.error().c_str());
        return 1;
    }
    const auto model = lagrange::opendrive::toModel(*network, options);
    if (!model) {
        std::fprintf(stderr, "%s: %s\n", positional[0].c_str(), model.error().c_str());
        return 1;
    }
    lagrange::stage::WriteOptions write;
    write.doc = "Converted from " + positional[0] + " by lagrange_opendrive_import.";
    if (const auto status = lagrange::stage::writeStage(*model, positional[1], write); !status) {
        std::fprintf(stderr, "%s\n", status.error().c_str());
        return 1;
    }
    std::printf("wrote %s: %zu roads, %zu lanes\n", positional[1].c_str(), network->roads.size(),
                model->world.schemaInstances.size());
    return 0;
}
