// lagrange_lbm_hover: a multicopter's rotors as actuator discs in a lattice Boltzmann box
// (LbmBox), for measuring how fast the box runs and what flow it makes. Reads the rotors from a
// file of lines "x y z ax ay az radius thrust" (m, unit axis, m, N; the box's frame, its corner at
// the origin), runs, and writes the mean velocity over the last part of the run as raw float32
// (3 per cell, x fastest, components interleaved, m/s) with a JSON description beside it.
//
//   lagrange_lbm_hover ROTORS.txt OUT_PREFIX --box X,Y,Z --cell M [--floor] [--seconds S]
//                      [--average S] [--max-speed M_S] [--cpu]
#include "lagrange/aerodynamics/LbmBox.hpp"
#include "lagrange/gpu/Device.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using lagrange::aerodynamics::LbmBox;
using lagrange::aerodynamics::LbmBoxConfig;

std::vector<double> numbers(const std::string& text) {
    std::vector<double> out;
    std::stringstream in(text);
    std::string part;
    while (std::getline(in, part, ',')) {
        out.push_back(std::stod(part));
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: lagrange_lbm_hover ROTORS.txt OUT_PREFIX --box X,Y,Z --cell M "
                     "[--floor] [--seconds S] [--average S] [--max-speed M_S] [--cpu]\n";
        return 2;
    }
    const std::string rotorFile = argv[1];
    const std::string prefix = argv[2];
    std::vector<double> box{4.0, 4.0, 3.0};
    double cell = 0.05;
    double seconds = 3.0;
    double average = 1.0;
    double maxSpeed = 15.0;
    bool floor = false;
    bool cpu = false;
    bool trace = false;
    for (int a = 3; a < argc; ++a) {
        const std::string arg = argv[a];
        const auto value = [&]() { return std::string(a + 1 < argc ? argv[++a] : ""); };
        if (arg == "--box") {
            box = numbers(value());
        } else if (arg == "--cell") {
            cell = std::stod(value());
        } else if (arg == "--seconds") {
            seconds = std::stod(value());
        } else if (arg == "--average") {
            average = std::stod(value());
        } else if (arg == "--max-speed") {
            maxSpeed = std::stod(value());
        } else if (arg == "--floor") {
            floor = true;
        } else if (arg == "--trace") {
            trace = true;
        } else if (arg == "--cpu") {
            cpu = true;
        } else {
            std::cerr << "lagrange_lbm_hover: unknown " << arg << "\n";
            return 2;
        }
    }
    std::vector<LbmBox::Rotor> rotors;
    std::ifstream in(rotorFile);
    for (std::string line; std::getline(in, line);) {
        std::stringstream words(line);
        LbmBox::Rotor r;
        if (words >> r.centre[0] >> r.centre[1] >> r.centre[2] >> r.axis[0] >> r.axis[1] >>
            r.axis[2] >> r.radius >> r.thrust) {
            rotors.push_back(r);
        }
    }
    LbmBoxConfig config;
    // An even number across x, as the GPU step needs.
    config.nx = 2 * static_cast<std::size_t>(std::lround(box.at(0) / (2.0 * cell)));
    config.ny = static_cast<std::size_t>(std::lround(box.at(1) / cell));
    config.nz = static_cast<std::size_t>(std::lround(box.at(2) / cell));
    config.cellSize = cell;
    config.maxSpeed = maxSpeed;
    config.floor = floor;
    std::unique_ptr<lagrange::gpu::Device> device;  // outlives the box
    if (!cpu) {
        std::string error;
        device = lagrange::gpu::Device::create(lagrange::gpu::DeviceDesc{}, error);
    }
    LbmBox air(config);
    std::string gpuName = "cpu";
    {
        if (device) {
            air.useGpu(*device);
            gpuName = air.onGpu() ? device->info().name : "cpu";
        }
    }
    air.setRotors(rotors);
    const double dt = air.timeStep();
    const auto steps = static_cast<std::size_t>(std::lround(seconds / dt));
    const auto averageSteps = static_cast<std::size_t>(std::lround(average / dt));
    constexpr std::size_t kChunk = 50;  // steps between samples of the mean
    const std::size_t count = air.cells();
    std::vector<double> sum(count * 3, 0.0);
    std::size_t samples = 0;
    const auto start = std::chrono::steady_clock::now();
    double steppingSeconds = 0.0;
    std::size_t done = 0;
    while (done < steps) {
        const std::size_t now = std::min(kChunk, steps - done);
        const auto t0 = std::chrono::steady_clock::now();
        air.step(now);
        steppingSeconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        done += now;
        if (trace && done % 500 == 0) {
            air.syncVelocity();
            double top = 0.0;
            std::size_t where = 0;
            for (std::size_t c = 0; c < count; ++c) {
                const auto v = air.velocity(c);
                const double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                if (!(s <= top)) {
                    top = s;
                    where = c;
                }
            }
            std::printf("t %.3f s: fastest %.2f m/s at cell %zu,%zu,%zu\n",
                        static_cast<double>(done) * dt, top, where % config.nx,
                        (where / config.nx) % config.ny, where / (config.nx * config.ny));
        }
        if (done + averageSteps >= steps) {
            air.syncVelocity();
            for (std::size_t c = 0; c < count; ++c) {
                const auto v = air.velocity(c);
                for (int k = 0; k < 3; ++k) {
                    sum[c * 3 + static_cast<std::size_t>(k)] += v.at(k);
                }
            }
            ++samples;
        }
    }
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    air.syncVelocity();
    const bool diverged = air.diverged();
    std::vector<float> mean(count * 3);
    for (std::size_t i = 0; i < mean.size(); ++i) {
        mean[i] =
            static_cast<float>(sum[i] / static_cast<double>(std::max<std::size_t>(samples, 1)));
    }
    std::ofstream raw(prefix + ".f32", std::ios::binary);
    raw.write(reinterpret_cast<const char*>(mean.data()),
              static_cast<std::streamsize>(mean.size() * sizeof(float)));
    const double stepsPerSecond = static_cast<double>(steps) / steppingSeconds;
    std::ofstream json(prefix + ".json");
    json << "{\"nx\": " << config.nx << ", \"ny\": " << config.ny << ", \"nz\": " << config.nz
         << ", \"cell\": " << cell << ", \"dt\": " << dt << ", \"steps\": " << steps
         << ", \"seconds\": " << seconds << ", \"samples\": " << samples
         << ", \"steps_per_second\": " << stepsPerSecond
         << ", \"real_time_factor\": " << stepsPerSecond * dt
         << ", \"cells_per_second\": " << stepsPerSecond * static_cast<double>(count)
         << ", \"wall_seconds\": " << wall << ", \"floor\": " << (floor ? "true" : "false")
         << ", \"device\": \"" << gpuName << "\", \"diverged\": " << (diverged ? "true" : "false")
         << "}\n";
    std::printf("%s: %zu cells of %.3f m, dt %.3g ms, %.0f steps/s, %.2fx real time, %s%s\n",
                gpuName.c_str(), count, cell, dt * 1e3, stepsPerSecond, stepsPerSecond * dt,
                diverged ? "DIVERGED " : "", prefix.c_str());
    return diverged ? 1 : 0;
}
