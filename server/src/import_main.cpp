// lagrange_import: convert a robot or world description to USD (roadmap 0.2-G1, G3, G4).
//
//   lagrange_import robot.urdf|robot.sdf|robot.xml robot.usda [--doc "text"]
//                   [--overlay robot.overlay.yaml]
//
// The input is URDF, SDFormat or MJCF, told apart by its root element. The result is what
// lagrange_server reads back to the same simulation: bodies with PhysicsRigidBodyAPI and
// PhysicsMassAPI, UsdPhysics joints (PhysicsDriveAPI on the actuated joints: a URDF's
// <ros2_control> block, the joints an SDFormat model plugin names, MJCF actuators), collision
// shapes with PhysicsCollisionAPI, visual meshes, and the sensors as SensorAPI_1 with LidarAPI_1
// or ImuAPI_1. An overlay file adds the schemas these formats cannot carry (motors, battery,
// bus: docs/design/formats.md §9). What could not be converted is listed on stderr.
#include "lagrange/server/Robot.hpp"
#include "lagrange/stage/Stage.hpp"

#include <cstdio>
#include <string>
#include <string_view>

namespace {

int usage(const char* error) {
    if (error != nullptr) {
        std::fprintf(stderr, "lagrange_import: %s\n", error);
    }
    std::fprintf(stderr,
                 "usage: lagrange_import INPUT OUTPUT.usda [--doc TEXT] [--overlay FILE.yaml]\n"
                 "  Converts a URDF, SDFormat or MJCF file to USD, applying the overlay's\n"
                 "  schemas (motors, battery, bus) when one is given.\n");
    return error != nullptr ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::string input;
    std::string output;
    lagrange::stage::WriteOptions write;
    std::string overlay;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            return usage(nullptr);
        }
        if (arg == "--doc") {
            if (i + 1 >= argc) {
                return usage("--doc needs a text");
            }
            write.doc = argv[++i];
        } else if (arg == "--overlay") {
            if (i + 1 >= argc) {
                return usage("--overlay needs a file");
            }
            overlay = argv[++i];
        } else if (input.empty()) {
            input = arg;
        } else if (output.empty()) {
            output = arg;
        } else {
            return usage(("unexpected argument " + std::string(arg)).c_str());
        }
    }
    if (input.empty() || output.empty()) {
        return usage("needs an input and an output file");
    }
    if (lagrange::server::isUsd(input)) {
        return usage("the input is already USD");
    }
    auto model = lagrange::server::loadModel(input);
    if (!model) {
        std::fprintf(stderr, "lagrange_import: %s\n", model.error().c_str());
        return 1;
    }
    if (!overlay.empty()) {
        if (auto applied = lagrange::stage::applyOverlay(*model, overlay); !applied) {
            std::fprintf(stderr, "lagrange_import: %s\n", applied.error().c_str());
            return 1;
        }
    }
    for (const std::string& note : model->notes) {
        std::fprintf(stderr, "note: %s\n", note.c_str());
    }
    if (write.doc.empty()) {
        write.doc = "Converted from " + input + " by lagrange_import.";
    }
    if (auto written = lagrange::stage::writeStage(*model, output, write); !written) {
        std::fprintf(stderr, "lagrange_import: %s\n", written.error().c_str());
        return 1;
    }
    std::printf("wrote %s\n", output.c_str());
    return 0;
}
