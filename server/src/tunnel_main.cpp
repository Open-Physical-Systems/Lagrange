// lagrange tunnel ROBOT [--speed M_S] [--yaw DEG] [--cells N] [--out DIR]: air past a robot.
#include "lagrange/aerodynamics/WindTunnel.hpp"
#include "lagrange/gpu/Device.hpp"
#include "lagrange/server/Robot.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

/// The whole of `text` as a finite number, or nothing (atof took "nan", "5abc" and "" alike).
std::optional<double> number(const char* text) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

/// A count: a whole number from 0 up to `most`.
std::optional<std::size_t> count(const char* text, double most) {
    const auto value = number(text);
    if (!value || *value < 0.0 || *value > most || std::floor(*value) != *value) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(*value);
}

}  // namespace

int main(int argc, char** argv) {
    lagrange::aerodynamics::WindTunnelOptions options;
    std::string robot;
    std::string out = "tunnel";
    bool useGpu = true;
    bool cellsGiven = false;
    bool study = false;
    std::string exportDir;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool more = i + 1 < argc;
        const auto bad = [&arg](const char* what) {
            std::fprintf(stderr, "lagrange tunnel: %s needs %s\n", arg.c_str(), what);
            return 2;
        };
        if (arg == "--speed" && more) {
            const auto v = number(argv[++i]);
            if (!v || *v <= 0.0) {
                return bad("a wind speed above zero, in m/s");
            }
            options.speed = *v;
        } else if (arg == "--yaw" && more) {
            const auto v = number(argv[++i]);
            if (!v) {
                return bad("an angle in degrees");
            }
            options.yawDegrees = *v;
        } else if (arg == "--cells" && more) {
            constexpr double kMostCells = 1e10;
            const auto v = count(argv[++i], kMostCells);
            if (!v || *v == 0) {  // the tunnel says how many it needs at least
                return bad("a whole number of cells");
            }
            options.maxCells = *v;
            cellsGiven = true;
        } else if (arg == "--passes" && more) {
            const auto v = number(argv[++i]);
            if (!v || *v <= 0.0) {
                return bad("a number of passes above zero");
            }
            options.passes = *v;
        } else if (arg == "--cpu") {
            useGpu = false;
        } else if (arg == "--ground") {
            options.ground = true;
        } else if (arg == "--mean-passes" && more) {
            const auto v = number(argv[++i]);
            if (!v || *v <= 0.0) {
                return bad("a number of passes above zero");
            }
            options.meanPasses = *v;
        } else if (arg == "--frames" && more) {
            constexpr double kMostFrames = 100000;
            const auto v = count(argv[++i], kMostFrames);
            if (!v) {
                return bad("a whole number of frames from 0 to 100000");
            }
            options.frames = *v;
        } else if (arg == "--frame-spin") {
            options.frameSpin = true;
        } else if (arg == "--clearance" && more) {
            const auto v = number(argv[++i]);
            if (!v) {
                return bad("a distance in m");
            }
            options.clearance = *v;
        } else if (arg == "--export-surface" && more) {
            exportDir = argv[++i];
        } else if (arg == "--study") {
            study = true;
        } else if (arg == "--out" && more) {
            out = argv[++i];
        } else if (robot.empty() && !arg.starts_with("--")) {
            robot = arg;
        } else {
            robot.clear();
            break;
        }
    }
    if (robot.empty()) {
        std::fprintf(
            stderr,
            "usage: lagrange tunnel ROBOT [--speed M_S] [--yaw DEG] [--cells N] "
            "[--passes N] [--mean-passes N] [--ground] [--clearance M] [--frames N] [--frame-spin] "
            "[--study] [--cpu] "
            "[--out DIR]\n");
        return 2;
    }
    auto model = lagrange::server::loadModel(robot);
    if (!model) {
        std::fprintf(stderr, "lagrange tunnel: %s\n", model.error().c_str());
        return 2;
    }
    if (!exportDir.empty()) {
        const std::string error = lagrange::aerodynamics::exportTunnelSurface(
            model->world, options.yawDegrees, exportDir);
        if (!error.empty()) {
            std::fprintf(stderr, "lagrange tunnel: %s\n", error.c_str());
            return 1;
        }
        return 0;
    }
    // On the GPU when there is a real one; a software device is slower than the CPU reference.
    std::unique_ptr<lagrange::gpu::Device> device;
    if (useGpu) {
        std::string error;
        lagrange::gpu::DeviceDesc desc;
        device = lagrange::gpu::Device::create(desc, error);
        if (device && device->info().isSoftware) {
            device.reset();
        }
    }
    options.device = device.get();
    if (!cellsGiven && device) {
        options.maxCells = 4000000;  // about 500 million cell updates a second on an RTX 4060
    }
    std::fprintf(stderr, "on %s\n", device ? device->info().name.c_str() : "the CPU");
    // A grid independence study: the same air on a quarter, a half and all the cells, finest last,
    // so what follows reports the finest and its files are the ones left.
    std::vector<lagrange::aerodynamics::GridStudyRow> rows;
    if (study) {
        for (const std::size_t divide : {std::size_t{4}, std::size_t{2}}) {
            auto coarse = options;
            coarse.maxCells = options.maxCells / divide;
            coarse.frames = 0;
            const std::string dir = out + "/grid_" + std::to_string(coarse.maxCells);
            std::fprintf(stderr, "grid of %zu cells\n", coarse.maxCells);
            auto run = lagrange::aerodynamics::runWindTunnel(model->world, coarse, dir);
            if (!run) {
                std::fprintf(stderr, "lagrange tunnel: %s\n", run.error().c_str());
                return 1;
            }
            rows.push_back({run->grid[0] * run->grid[1] * run->grid[2], run->dragCoefficient,
                            run->dragSpread, run->liftCoefficient});
            std::error_code ignored;
            std::filesystem::remove_all(dir, ignored);
        }
    }
    int last = -1;
    auto result =
        lagrange::aerodynamics::runWindTunnel(model->world, options, out, [&last](double share) {
            const int percent = static_cast<int>(share * 100.0);
            if (percent / 10 != last / 10) {
                std::fprintf(stderr, "%d %%\n", percent);
                last = percent;
            }
        });
    if (!result) {
        std::fprintf(stderr, "lagrange tunnel: %s\n", result.error().c_str());
        return 1;
    }
    std::printf("wind %.1f m/s, Reynolds %.3g, grid %zu x %zu x %zu of %.1f mm, %zu steps\n",
                options.speed, result->reynolds, result->grid[0], result->grid[1], result->grid[2],
                result->cellSize * 1000.0, result->steps);
    std::printf("frontal area %.4f m2, blockage %.1f %%\n", result->frontalArea,
                result->blockage * 100.0);
    if (options.ground) {
        std::printf("road %.1f mm below its lowest point, %.1f cells\n", result->clearance * 1000.0,
                    result->clearance / result->cellSize);
    }
    std::printf("Cd %.3f (moves by %.3f as the wake sheds)   Cl %.3f   Cy %.3f\n",
                result->dragCoefficient, result->dragSpread, result->liftCoefficient,
                result->sideCoefficient);
    std::printf("%-24s %10s %10s %10s\n", "link", "drag N", "side N", "lift N");
    for (const auto& link : result->links) {
        std::printf("%-24s %10.2f %10.2f %10.2f\n", link.name.c_str(), link.force[0], link.force[1],
                    link.force[2]);
    }
    std::printf("%-24s %10.2f %10.2f %10.2f\n", "total", result->force[0], result->force[1],
                result->force[2]);
    if (study) {
        rows.push_back({result->grid[0] * result->grid[1] * result->grid[2],
                        result->dragCoefficient, result->dragSpread, result->liftCoefficient});
        const auto verdict = lagrange::aerodynamics::gridStudyVerdict(rows);
        std::printf("\n%12s %8s %8s %8s\n", "cells", "Cd", "moves", "Cl");
        std::ofstream json(out + "/study.json");
        json << "{\"grids\": [";
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const auto& row = rows[i];
            std::printf("%12zu %8.3f %8.3f %8.3f\n", row.cells, row.dragCoefficient, row.dragSpread,
                        row.liftCoefficient);
            json << (i > 0 ? ", " : "") << "{\"cells\": " << row.cells
                 << ", \"Cd\": " << row.dragCoefficient << ", \"moves\": " << row.dragSpread
                 << ", \"Cl\": " << row.liftCoefficient << "}";
        }
        json << "], \"drag_change\": " << verdict.dragChange
             << ", \"drag_moves\": " << verdict.dragMoves
             << ", \"lift_change\": " << verdict.liftChange
             << ", \"settled\": " << (verdict.settled ? "true" : "false") << "}\n";
        std::printf(
            "%s: on the last refinement Cd changed %.0f %% and Cl by %.0f %% of Cd; Cd moves %.0f "
            "%% "
            "in time (5 %% is a rule of thumb). Settled means the grid stopped mattering, not that "
            "the number is right.\n",
            verdict.settled ? "SETTLED" : "NOT SETTLED", verdict.dragChange * 100.0,
            verdict.liftChange * 100.0, verdict.dragMoves * 100.0);
    }
    std::printf(
        "\nA grid this coarse with no wall model gives a bluff body's size of force, not an "
        "engineering number: compare shapes and angles rather than trust the third digit.\n"
        "See the flow:     lagrange --robot none --world %s/flow_view.usda\n"
        "In ParaView:      %s/flow.vti\n"
        "As wind in a run: --weather ...,wind_field=%s/flow.nvdb\n"
        "Wake pictures:    --frames 200 writes frames/*.png, one per moment\n",
        out.c_str(), out.c_str(), out.c_str());
    return 0;
}
