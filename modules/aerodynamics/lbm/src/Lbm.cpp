// See Lbm.hpp. The lattice and the collision follow Krüger et al., The Lattice Boltzmann Method
// (Springer, 2017): D3Q19 velocities and weights (ch. 3), half-way bounce-back and momentum
// exchange (ch. 5, 11), and the Smagorinsky relaxation time of Hou et al. (1996).
#include "lagrange/aerodynamics/Lbm.hpp"

#include <algorithm>
#include <cmath>
#include <tbb/parallel_for.h>

namespace lagrange::aerodynamics {

namespace {

constexpr int kQ = 19;
constexpr std::size_t kStepsPerMeanSample = 10;
constexpr std::array<int, kQ> kCx{0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0};
constexpr std::array<int, kQ> kCy{0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1};
constexpr std::array<int, kQ> kCz{0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1};
constexpr std::array<int, kQ> kOpposite{0, 2,  1,  4,  3,  6,  5,  8,  7, 10,
                                        9, 12, 11, 14, 13, 16, 15, 18, 17};
constexpr double kW0 = 1.0 / 3.0;
constexpr double kW1 = 1.0 / 18.0;
constexpr double kW2 = 1.0 / 36.0;
constexpr std::array<double, kQ> kWeight{kW0, kW1, kW1, kW1, kW1, kW1, kW1, kW2, kW2, kW2,
                                         kW2, kW2, kW2, kW2, kW2, kW2, kW2, kW2, kW2};

double equilibrium(int i, double rho, double ux, double uy, double uz) {
    const double cu = kCx[i] * ux + kCy[i] * uy + kCz[i] * uz;
    const double uu = ux * ux + uy * uy + uz * uz;
    return kWeight[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * uu);
}

}  // namespace

LbmTunnel::LbmTunnel(const LbmConfig& config) : config_(config) {
    dt_ = config.latticeSpeed * config.cellSize / config.windSpeed;
    const double nuLattice = config.kinematicViscosity * dt_ / (config.cellSize * config.cellSize);
    tau_ = 3.0 * nuLattice + 0.5;
    // Lattice momentum per step, ρ dx³ · (dx/dt) / dt, is a force.
    const double dx = config.cellSize;
    forceScale_ = config.airDensity * dx * dx * dx * dx / (dt_ * dt_);
    const std::size_t count = cells();
    f_.resize(count * kQ);
    next_.resize(count * kQ);
    solid_.assign(count, kFluid);
    const double u = config.periodic ? 0.0 : config.latticeSpeed;
    for (std::size_t c = 0; c < count; ++c) {
        for (int i = 0; i < kQ; ++i) {
            f_[c * kQ + i] = static_cast<float>(equilibrium(i, 1.0, u, 0.0, 0.0));
        }
    }
}

void LbmTunnel::setSolid(std::size_t x, std::size_t y, std::size_t z, std::uint16_t solid) {
    solid_[index(x, y, z)] = solid;
    if (loads_.size() <= solid) {
        loads_.resize(std::size_t{solid} + 1);
    }
}

void LbmTunnel::setLatticeFlow(std::size_t cell, double density,
                               const std::array<double, 3>& velocity) {
    for (int i = 0; i < kQ; ++i) {
        f_[cell * kQ + i] =
            static_cast<float>(equilibrium(i, density, velocity[0], velocity[1], velocity[2]));
    }
}

void LbmTunnel::step(std::size_t steps) {
    std::vector<Load> sum(loads_.size());
    for (std::size_t s = 0; s < steps; ++s) {
        stepOnce(sum);
        if (averaging_ && ++sinceSample_ >= kStepsPerMeanSample) {
            sinceSample_ = 0;
            ++samples_;
            tbb::parallel_for(std::size_t{0}, cells(), [this](std::size_t c) {
                const std::array<double, 3> u = velocity(c);
                for (std::size_t k = 0; k < 3; ++k) {
                    meanVelocity_[3 * c + k] += static_cast<float>(u.at(k));
                }
                meanPressure_[c] += static_cast<float>(pressure(c));
            });
        }
    }
    for (std::size_t b = 0; b < loads_.size(); ++b) {
        for (std::size_t k = 0; k < 3; ++k) {
            loads_[b].force.at(k) = sum[b].force.at(k) / static_cast<double>(steps);
            loads_[b].torque.at(k) = sum[b].torque.at(k) / static_cast<double>(steps);
        }
    }
}

void LbmTunnel::stepOnce(std::vector<Load>& sum) {
    const auto nx = static_cast<std::ptrdiff_t>(config_.nx);
    const auto ny = static_cast<std::ptrdiff_t>(config_.ny);
    const auto nz = static_cast<std::ptrdiff_t>(config_.nz);
    const bool periodic = config_.periodic;
    const double inletU = config_.latticeSpeed;
    const double cs2 = config_.smagorinsky * config_.smagorinsky;
    const double tau = tau_;
    const double dx = config_.cellSize;
    const std::size_t solids = loads_.size();
    // Each slab keeps its own sums: added below in z order, so threads cannot change the result.
    std::vector<std::vector<Load>> slabs(static_cast<std::size_t>(nz), std::vector<Load>(solids));
    tbb::parallel_for(std::ptrdiff_t{0}, nz, [&](std::ptrdiff_t z) {
        std::vector<Load>& slab = slabs[static_cast<std::size_t>(z)];
        for (std::ptrdiff_t y = 0; y < ny; ++y) {
            for (std::ptrdiff_t x = 0; x < nx; ++x) {
                const auto cell = static_cast<std::size_t>(x + nx * (y + ny * z));
                if (solid_[cell] != kFluid) {
                    continue;
                }
                std::array<double, kQ> in{};
                // Streaming: pull each direction from where it came.
                for (int i = 0; i < kQ; ++i) {
                    std::ptrdiff_t sx = x - kCx[i];
                    const std::ptrdiff_t sy = (y - kCy[i] + ny) % ny;
                    const std::ptrdiff_t sz = (z - kCz[i] + nz) % nz;
                    if (periodic) {
                        sx = (sx + nx) % nx;
                    } else if (sx < 0) {
                        in[i] = equilibrium(i, 1.0, inletU, 0.0, 0.0);  // the inlet
                        continue;
                    } else if (sx >= nx) {
                        sx = nx - 1;  // the outlet: what the last column has, zero gradient
                    }
                    const auto source = static_cast<std::size_t>(sx + nx * (sy + ny * sz));
                    const std::uint16_t solid = solid_[source];
                    if (solid != kFluid) {
                        // Half-way bounce-back: what left here towards the solid comes back.
                        const double back = f_[cell * kQ + kOpposite[i]];
                        in[i] = back;
                        // The solid takes twice that momentum along the way it was going.
                        const double px = -kCx[i] * 2.0 * back;
                        const double py = -kCy[i] * 2.0 * back;
                        const double pz = -kCz[i] * 2.0 * back;
                        Load& load = slab[solid];
                        load.force[0] += px;
                        load.force[1] += py;
                        load.force[2] += pz;
                        // At the wall, half a link from this cell's centre.
                        const double rx = (static_cast<double>(x) + 0.5 - 0.5 * kCx[i]) * dx;
                        const double ry = (static_cast<double>(y) + 0.5 - 0.5 * kCy[i]) * dx;
                        const double rz = (static_cast<double>(z) + 0.5 - 0.5 * kCz[i]) * dx;
                        load.torque[0] += ry * pz - rz * py;
                        load.torque[1] += rz * px - rx * pz;
                        load.torque[2] += rx * py - ry * px;
                    } else {
                        in[i] = f_[source * kQ + i];
                    }
                }
                double rho = 0.0;
                double ux = 0.0;
                double uy = 0.0;
                double uz = 0.0;
                for (int i = 0; i < kQ; ++i) {
                    rho += in[i];
                    ux += kCx[i] * in[i];
                    uy += kCy[i] * in[i];
                    uz += kCz[i] * in[i];
                }
                ux /= rho;
                uy /= rho;
                uz /= rho;
                std::array<double, kQ> eq{};
                for (int i = 0; i < kQ; ++i) {
                    eq[i] = equilibrium(i, rho, ux, uy, uz);
                }
                double relax = tau;
                if (cs2 > 0.0) {
                    // Hou et al. 1996: the eddy viscosity from the non-equilibrium stress.
                    double xx = 0.0, yy = 0.0, zz = 0.0, xy = 0.0, xz = 0.0, yz = 0.0;
                    for (int i = 0; i < kQ; ++i) {
                        const double off = in[i] - eq[i];
                        xx += kCx[i] * kCx[i] * off;
                        yy += kCy[i] * kCy[i] * off;
                        zz += kCz[i] * kCz[i] * off;
                        xy += kCx[i] * kCy[i] * off;
                        xz += kCx[i] * kCz[i] * off;
                        yz += kCy[i] * kCz[i] * off;
                    }
                    const double q = std::sqrt(xx * xx + yy * yy + zz * zz +
                                               2.0 * (xy * xy + xz * xz + yz * yz));
                    relax =
                        0.5 * (tau + std::sqrt(tau * tau + 18.0 * std::sqrt(2.0) * cs2 * q / rho));
                }
                const double omega = 1.0 / relax;
                for (int i = 0; i < kQ; ++i) {
                    next_[cell * kQ + i] = static_cast<float>(in[i] + omega * (eq[i] - in[i]));
                }
            }
        }
    });
    f_.swap(next_);
    for (const std::vector<Load>& slab : slabs) {
        for (std::size_t b = 0; b < solids; ++b) {
            for (std::size_t k = 0; k < 3; ++k) {
                sum[b].force.at(k) += slab[b].force.at(k) * forceScale_;
                sum[b].torque.at(k) += slab[b].torque.at(k) * forceScale_;
            }
        }
    }
}

void LbmTunnel::startAveraging() {
    averaging_ = true;
    sinceSample_ = 0;
    samples_ = 0;
    meanVelocity_.assign(3 * cells(), 0.0F);
    meanPressure_.assign(cells(), 0.0F);
}

std::array<double, 3> LbmTunnel::meanVelocity(std::size_t cell) const {
    if (samples_ == 0) {
        return velocity(cell);
    }
    const auto n = static_cast<double>(samples_);
    return {meanVelocity_[3 * cell] / n, meanVelocity_[3 * cell + 1] / n,
            meanVelocity_[3 * cell + 2] / n};
}

double LbmTunnel::meanPressure(std::size_t cell) const {
    return samples_ == 0 ? pressure(cell) : meanPressure_[cell] / static_cast<double>(samples_);
}

std::array<double, 3> LbmTunnel::latticeVelocity(std::size_t cell) const {
    if (solid_[cell] != kFluid) {
        return {};
    }
    double rho = 0.0;
    std::array<double, 3> u{};
    for (int i = 0; i < kQ; ++i) {
        const double value = f_[cell * kQ + i];
        rho += value;
        u[0] += kCx[i] * value;
        u[1] += kCy[i] * value;
        u[2] += kCz[i] * value;
    }
    return {u[0] / rho, u[1] / rho, u[2] / rho};
}

std::array<double, 3> LbmTunnel::velocity(std::size_t cell) const {
    const std::array<double, 3> u = latticeVelocity(cell);
    const double scale = config_.cellSize / dt_;
    return {u[0] * scale, u[1] * scale, u[2] * scale};
}

double LbmTunnel::pressure(std::size_t cell) const {
    if (solid_[cell] != kFluid) {
        return 0.0;
    }
    double rho = 0.0;
    for (int i = 0; i < kQ; ++i) {
        rho += f_[cell * kQ + i];
    }
    // p = c_s² ρ with c_s² = 1/3 in lattice units, then to Pa.
    const double scale = config_.cellSize / dt_;
    return (rho - 1.0) / 3.0 * config_.airDensity * scale * scale;
}

bool LbmTunnel::diverged() const {
    return std::any_of(f_.begin(), f_.end(), [](float v) { return !std::isfinite(v); });
}

}  // namespace lagrange::aerodynamics
