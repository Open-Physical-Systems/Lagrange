// lagrange check ROBOT [--overlay FILE]: what each joint must hold against gravity. Exit 1 when one
// cannot hold what it carries at the pose the robot starts in, 0 otherwise.
#include "lagrange/server/Robot.hpp"
#include "lagrange/server/RobotCheck.hpp"
#include "lagrange/stage/Stage.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    std::string robot;
    std::string overlay;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--overlay" && i + 1 < argc) {
            overlay = argv[++i];
        } else if (robot.empty() && !arg.starts_with("--")) {
            robot = arg;
        } else {
            robot.clear();
            break;
        }
    }
    if (robot.empty()) {
        std::fprintf(stderr, "usage: lagrange check ROBOT [--overlay FILE]\n");
        return 2;
    }
    auto model = lagrange::server::loadModel(robot);
    if (!model) {
        std::fprintf(stderr, "lagrange check: %s\n", model.error().c_str());
        return 2;
    }
    if (!overlay.empty()) {
        if (auto applied = lagrange::stage::applyOverlay(*model, overlay); !applied) {
            std::fprintf(stderr, "lagrange check: %s\n", applied.error().c_str());
            return 2;
        }
    }
    const auto loads = lagrange::server::jointLoads(model->world);
    if (loads.empty()) {
        std::printf("%s moves about freely or has no joints that carry weight: nothing to check\n",
                    robot.c_str());
        return 0;
    }
    std::printf("%-24s %10s %12s %14s\n", "link", "mass kg", "shapes litres", "at least kg/m3");
    int impossible = 0;
    for (const auto& link : lagrange::server::linkDensities(model->world)) {
        const bool wrong = link.density() > lagrange::server::kDensestSolid;
        impossible += wrong ? 1 : 0;
        if (link.volume > 0.0) {
            std::printf("%-24s %10.2f %12.2f %14.0f%s\n", link.name.c_str(), link.mass,
                        link.volume * 1000.0, link.density(),
                        wrong ? "  denser than any solid: check its mass or scale" : "");
        } else {
            std::printf("%-24s %10.2f %12s %14s\n", link.name.c_str(), link.mass, "-", "-");
        }
    }
    std::printf(
        "Steel is about 7,850 kg/m3, aluminium 2,700, plastic 1,000. Hulls fill in what is "
        "hollow, so a real density can only be higher than this.\n\n");
    if (impossible > 0) {
        std::printf("%d link(s) are heavier than their shape can be.\n\n", impossible);
    }
    int short_at_rest = 0;
    int short_worst = 0;
    std::printf("%-24s %10s %10s %10s %10s %10s\n", "joint", "carries kg", "at rest", "worst",
                "limit", "motor");
    for (const auto& load : loads) {
        char motor[32] = "-";
        if (load.motorMost) {
            std::snprintf(motor, sizeof(motor), "%.1f", *load.motorMost);
        }
        const double most =
            load.motorMost ? std::min(*load.motorMost,
                                      load.effortLimit > 0.0 ? load.effortLimit : *load.motorMost)
                           : load.effortLimit;
        const bool limited = most > 0.0;
        const char* verdict = "";
        if (limited && load.atRest > most) {
            verdict = "  CANNOT HOLD where it starts";
            ++short_at_rest;
        } else if (limited && load.worst > most) {
            verdict = "  cannot hold stretched out";
            ++short_worst;
        }
        std::printf("%-24s %10.2f %10.1f %10.1f %10.1f %10s%s\n", load.name.c_str(),
                    load.massCarried, load.atRest, load.worst, load.effortLimit, motor, verdict);
    }
    std::printf(
        "Torque in N m, force in N. 'worst' takes every link straight out sideways, an "
        "upper bound.\n");
    if (short_at_rest > 0) {
        std::printf(
            "%d joint(s) cannot hold the robot where it starts: raise their effort, or "
            "check the masses, which CAD exports often have wrong.\n",
            short_at_rest);
        return 1;
    }
    if (short_worst > 0) {
        std::printf("%d joint(s) can hold it here but not in every pose.\n", short_worst);
    }
    return 0;
}
