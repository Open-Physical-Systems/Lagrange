// See LbmBox.hpp. The lattice, collision and bounce-back follow Krüger et al., The Lattice
// Boltzmann Method (Springer, 2017): D3Q19 (ch. 3), half-way bounce-back (ch. 5), and Guo's
// forcing (ch. 6.3; Guo, Zheng and Shi, Phys. Rev. E 65, 046308, 2002): the velocity is
// (Σ c f + F/2)/ρ and each population gets (1 − ω/2) w_i [3(c − u) + 9 (c·u) c]·F after collision.
// The Smagorinsky relaxation time is Hou et al. (1996), as in the tunnel (Lbm.cpp). The open sides
// take in populations at equilibrium at ambient density and the edge cell's own velocity with any
// inward part across that face dropped, a pressure condition with the flow carried outwards
// (Krüger ch. 5.3).
#include "lagrange/aerodynamics/LbmBox.hpp"
#include "lagrange/gpu/LbmBoxStepper.hpp"

#include <algorithm>
#include <cmath>
#include <tbb/parallel_for.h>

namespace lagrange::aerodynamics {

namespace {

constexpr int kQ = 19;
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

using Vec = std::array<double, 3>;

double equilibrium(int i, double rho, const Vec& u) {
    const double cu = kCx[i] * u[0] + kCy[i] * u[1] + kCz[i] * u[2];
    const double uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    return kWeight[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * uu);
}

double dot(const Vec& a, const Vec& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// A disc's weight at a cell centre (cells): a hat one cell thick along the axis times the share of
// the cell inside the radius across it. lbm_box.slang has the same.
double weightAt(const Vec& centre, const Vec& axis, double radius, const Vec& at) {
    const Vec d{at[0] - centre[0], at[1] - centre[1], at[2] - centre[2]};
    const double along = dot(d, axis);
    const Vec off{d[0] - along * axis[0], d[1] - along * axis[1], d[2] - along * axis[2]};
    const double across = std::sqrt(dot(off, off));
    const double wa = std::max(0.0, 1.0 - std::abs(along));
    const double wr = std::clamp(radius - across + 0.5, 0.0, 1.0);
    return wa * wr;
}

}  // namespace

LbmBox::LbmBox(const LbmBoxConfig& config) : config_(config) {
    const double dx = config.cellSize;
    dt_ = config.latticeSpeed * dx / config.maxSpeed;
    speedScale_ = dx / dt_;
    const double nuLattice = config.kinematicViscosity * dt_ / (dx * dx);
    tau_ = 3.0 * nuLattice + 0.5;
    // Lattice momentum per step, ρ dx³ · (dx/dt) / dt, is a force.
    forceScale_ = config.airDensity * dx * dx * dx * dx / (dt_ * dt_);
    const std::size_t count = cells();
    f_.resize(count * kQ);
    next_.resize(count * kQ);
    velocity_.assign(count * 3, 0.0F);
    solid_.assign(count, 0);
    for (int i = 0; i < kQ; ++i) {
        const auto rest = static_cast<float>(kWeight[i]);
        std::fill_n(f_.begin() + static_cast<std::ptrdiff_t>(i * count), count, rest);
    }
    next_ = f_;
}

LbmBox::~LbmBox() = default;

void LbmBox::useGpu(const gpu::Device& device) {
    gpu::LbmBoxStepper::Settings settings;
    settings.nx = config_.nx;
    settings.ny = config_.ny;
    settings.nz = config_.nz;
    settings.floor = config_.floor;
    settings.tau = static_cast<float>(tau_);
    settings.smagorinsky = static_cast<float>(config_.smagorinsky);
    std::vector<std::uint32_t> solid(cells());
    for (std::size_t c = 0; c < solid.size(); ++c) {
        solid[c] = solid_[c] != 0 ? 0U : 0xFFFFU;
    }
    auto stepper = std::make_unique<gpu::LbmBoxStepper>(device, settings, f_, solid);
    if (stepper->valid()) {
        stepper_ = std::move(stepper);
        pushDiscs();
        setAmbient(ambient_);
    }
}

void LbmBox::setSolid(std::size_t x, std::size_t y, std::size_t z) {
    solid_[index(x, y, z)] = 1;
}

double LbmBox::discWeightSum(const Rotor& rotor) const {
    const double dx = config_.cellSize;
    const Vec centre{rotor.centre[0] / dx, rotor.centre[1] / dx, rotor.centre[2] / dx};
    const double radius = rotor.radius / dx;
    const double reach = radius + 2.0;
    double sum = 0.0;
    const auto range = [&](int axis, std::size_t n) {
        const auto lo = static_cast<long>(std::floor(centre.at(axis) - reach));
        const auto hi = static_cast<long>(std::ceil(centre.at(axis) + reach));
        return std::pair<std::size_t, std::size_t>{
            static_cast<std::size_t>(std::max(0L, lo)),
            static_cast<std::size_t>(std::clamp(hi, 0L, static_cast<long>(n)))};
    };
    const auto [x0, x1] = range(0, config_.nx);
    const auto [y0, y1] = range(1, config_.ny);
    const auto [z0, z1] = range(2, config_.nz);
    for (std::size_t z = z0; z < z1; ++z) {
        for (std::size_t y = y0; y < y1; ++y) {
            for (std::size_t x = x0; x < x1; ++x) {
                if (solid_[index(x, y, z)] != 0) {
                    continue;
                }
                const Vec at{static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5,
                             static_cast<double>(z) + 0.5};
                sum += weightAt(centre, rotor.axis, radius, at);
            }
        }
    }
    return sum;
}

void LbmBox::setRotors(std::span<const Rotor> rotors) {
    {
        discs_.clear();
        const double dx = config_.cellSize;
        for (const Rotor& rotor : rotors) {
            const double sum = discWeightSum(rotor);
            if (sum <= 0.0) {
                continue;  // outside the box
            }
            const double pushPerWeight = rotor.thrust / forceScale_ / sum;
            Disc disc;
            disc.centre = {rotor.centre[0] / dx, rotor.centre[1] / dx, rotor.centre[2] / dx};
            disc.axis = rotor.axis;
            disc.radius = rotor.radius / dx;
            // The air is pushed against the thrust.
            disc.push = {-pushPerWeight * rotor.axis[0], -pushPerWeight * rotor.axis[1],
                         -pushPerWeight * rotor.axis[2]};
            discs_.push_back(disc);
        }
    }
    pushDiscs();
}

void LbmBox::pushDiscs() {
    if (stepper_) {
        std::vector<gpu::LbmBoxStepper::Disc> out;
        for (const Disc& d : discs_) {
            gpu::LbmBoxStepper::Disc g;
            g.centreRadius = {static_cast<float>(d.centre[0]), static_cast<float>(d.centre[1]),
                              static_cast<float>(d.centre[2]), static_cast<float>(d.radius)};
            g.axis = {static_cast<float>(d.axis[0]), static_cast<float>(d.axis[1]),
                      static_cast<float>(d.axis[2]), 0.0F};
            g.push = {static_cast<float>(d.push[0]), static_cast<float>(d.push[1]),
                      static_cast<float>(d.push[2]), 0.0F};
            out.push_back(g);
        }
        stepper_->setDiscs(out);
    }
}

void LbmBox::step(std::size_t steps) {
    if (stepper_) {
        stepper_->step(steps);
        return;
    }
    for (std::size_t s = 0; s < steps; ++s) {
        stepOnce();
    }
}

void LbmBox::syncVelocity() {
    if (stepper_) {
        stepper_->readVelocity(velocity_);
    }
}

void LbmBox::setAmbient(const std::array<double, 3>& wind) {
    ambient_ = wind;
    if (stepper_) {
        stepper_->setAmbient({static_cast<float>(wind[0] / speedScale_),
                              static_cast<float>(wind[1] / speedScale_),
                              static_cast<float>(wind[2] / speedScale_)});
    }
}

void LbmBox::fillWithAmbient() {
    const std::size_t count = cells();
    const Vec u{ambient_[0] / speedScale_, ambient_[1] / speedScale_, ambient_[2] / speedScale_};
    for (std::size_t c = 0; c < count; ++c) {
        if (solid_[c] != 0) {
            continue;
        }
        for (int i = 0; i < kQ; ++i) {
            f_[static_cast<std::size_t>(i) * count + c] =
                static_cast<float>(equilibrium(i, 1.0, u));
        }
        for (std::size_t k = 0; k < 3; ++k) {
            velocity_[k * count + c] = static_cast<float>(u.at(k));
        }
    }
    next_ = f_;
}

void LbmBox::syncPopulations() {
    if (stepper_) {
        stepper_->readPopulations(f_);
    }
}

void LbmBox::carryOver(const LbmBox& old, const std::array<long, 3>& offset) {
    const std::size_t count = cells();
    const std::size_t oldCount = old.cells();
    const std::array<long, 3> n{static_cast<long>(config_.nx), static_cast<long>(config_.ny),
                                static_cast<long>(config_.nz)};
    const std::array<long, 3> m{static_cast<long>(old.config_.nx),
                                static_cast<long>(old.config_.ny),
                                static_cast<long>(old.config_.nz)};
    for (long z = 0; z < n[2]; ++z) {
        for (long y = 0; y < n[1]; ++y) {
            for (long x = 0; x < n[0]; ++x) {
                const long ox = x + offset[0];
                const long oy = y + offset[1];
                const long oz = z + offset[2];
                if (ox < 0 || oy < 0 || oz < 0 || ox >= m[0] || oy >= m[1] || oz >= m[2]) {
                    continue;
                }
                const auto to = static_cast<std::size_t>(x + n[0] * (y + n[1] * z));
                const auto from = static_cast<std::size_t>(ox + m[0] * (oy + m[1] * oz));
                if (solid_[to] != 0 || old.solid_[from] != 0) {
                    continue;
                }
                for (std::size_t i = 0; i < kQ; ++i) {
                    f_[i * count + to] = old.f_[i * oldCount + from];
                }
                for (std::size_t k = 0; k < 3; ++k) {
                    velocity_[k * count + to] = old.velocity_[k * oldCount + from];
                }
            }
        }
    }
    next_ = f_;
}

void LbmBox::stepOnce() {
    const auto nx = static_cast<long>(config_.nx);
    const auto ny = static_cast<long>(config_.ny);
    const auto nz = static_cast<long>(config_.nz);
    const std::size_t count = cells();
    const double tau = tau_;
    const double c2 = config_.smagorinsky * config_.smagorinsky;
    const bool floor = config_.floor;
    // The wind in lattice units, as the kernel's params.ambient (in float there).
    const Vec wind{static_cast<float>(ambient_[0] / speedScale_),
                   static_cast<float>(ambient_[1] / speedScale_),
                   static_cast<float>(ambient_[2] / speedScale_)};
    tbb::parallel_for(0L, nz, [&](long z) {
        for (long y = 0; y < ny; ++y) {
            for (long x = 0; x < nx; ++x) {
                const auto cell = static_cast<std::size_t>(x + nx * (y + ny * z));
                if (solid_[cell] != 0) {
                    continue;
                }
                const Vec own{velocity_[cell], velocity_[count + cell],
                              velocity_[2 * count + cell]};
                std::array<double, kQ> in{};
                for (int i = 0; i < kQ; ++i) {
                    const long sx = x - kCx[i];
                    const long sy = y - kCy[i];
                    const long sz = z - kCz[i];
                    const auto opposite = static_cast<std::size_t>(kOpposite[i]);
                    if (sz < 0 && floor) {
                        in[i] = f_[opposite * count + cell];
                    } else if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) {
                        // Open: ambient pressure. Where the wind blows in, the wind comes in;
                        // elsewhere the flow is carried on outwards and air drawn in comes by
                        // the pressure alone (an inward speed carried in feeds on itself),
                        // relative to the wind.
                        const std::array<long, 3> s{sx, sy, sz};
                        const std::array<long, 3> n{nx, ny, nz};
                        bool windIn = false;
                        for (int k = 0; k < 3; ++k) {
                            windIn = windIn || (s.at(k) < 0 && wind.at(k) > 0.0) ||
                                     (s.at(k) >= n.at(k) && wind.at(k) < 0.0);
                        }
                        Vec out = windIn ? wind : own;
                        for (int k = 0; k < 3 && !windIn; ++k) {
                            if ((s.at(k) < 0 && out.at(k) > wind.at(k)) ||
                                (s.at(k) >= n.at(k) && out.at(k) < wind.at(k))) {
                                out.at(k) = wind.at(k);
                            }
                        }
                        in[i] = equilibrium(i, 1.0, out);
                    } else {
                        const auto source = static_cast<std::size_t>(sx + nx * (sy + ny * sz));
                        in[i] = solid_[source] != 0
                                    ? f_[opposite * count + cell]
                                    : f_[static_cast<std::size_t>(i) * count + source];
                    }
                }
                Vec force{0.0, 0.0, 0.0};
                const Vec centre{static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5,
                                 static_cast<double>(z) + 0.5};
                for (const Disc& d : discs_) {
                    const double w = weightAt(d.centre, d.axis, d.radius, centre);
                    for (int k = 0; k < 3; ++k) {
                        force.at(k) += d.push.at(k) * w;
                    }
                }
                double rho = 0.0;
                Vec u{0.0, 0.0, 0.0};
                for (int i = 0; i < kQ; ++i) {
                    rho += in[i];
                    u[0] += kCx[i] * in[i];
                    u[1] += kCy[i] * in[i];
                    u[2] += kCz[i] * in[i];
                }
                for (int k = 0; k < 3; ++k) {
                    u.at(k) = (u.at(k) + 0.5 * force.at(k)) / rho;
                }
                std::array<double, kQ> eq{};
                double xx = 0.0, yy = 0.0, zz = 0.0, xy = 0.0, xz = 0.0, yz = 0.0;
                for (int i = 0; i < kQ; ++i) {
                    eq[i] = equilibrium(i, rho, u);
                    const double off = in[i] - eq[i];
                    xx += kCx[i] * kCx[i] * off;
                    yy += kCy[i] * kCy[i] * off;
                    zz += kCz[i] * kCz[i] * off;
                    xy += kCx[i] * kCy[i] * off;
                    xz += kCx[i] * kCz[i] * off;
                    yz += kCy[i] * kCz[i] * off;
                }
                double relax = tau;
                if (c2 > 0.0) {
                    const double q = std::sqrt(xx * xx + yy * yy + zz * zz +
                                               2.0 * (xy * xy + xz * xz + yz * yz));
                    relax =
                        0.5 * (tau + std::sqrt(tau * tau + 18.0 * std::sqrt(2.0) * c2 * q / rho));
                }
                const double omega = 1.0 / relax;
                for (int i = 0; i < kQ; ++i) {
                    const Vec c{static_cast<double>(kCx[i]), static_cast<double>(kCy[i]),
                                static_cast<double>(kCz[i])};
                    const double cu = dot(c, u);
                    const Vec g{3.0 * (c[0] - u[0]) + 9.0 * cu * c[0],
                                3.0 * (c[1] - u[1]) + 9.0 * cu * c[1],
                                3.0 * (c[2] - u[2]) + 9.0 * cu * c[2]};
                    const double source = (1.0 - 0.5 * omega) * kWeight[i] * dot(g, force);
                    next_[static_cast<std::size_t>(i) * count + cell] =
                        static_cast<float>(in[i] + omega * (eq[i] - in[i]) + source);
                }
                velocity_[cell] = static_cast<float>(u[0]);
                velocity_[count + cell] = static_cast<float>(u[1]);
                velocity_[2 * count + cell] = static_cast<float>(u[2]);
            }
        }
    });
    std::swap(f_, next_);
}

std::array<double, 3> LbmBox::velocity(std::size_t cell) const {
    const std::size_t count = cells();
    return {velocity_[cell] * speedScale_, velocity_[count + cell] * speedScale_,
            velocity_[2 * count + cell] * speedScale_};
}

std::array<double, 3> LbmBox::velocityAt(const std::array<double, 3>& point) const {
    const double dx = config_.cellSize;
    const std::array<std::size_t, 3> n{config_.nx, config_.ny, config_.nz};
    std::array<std::size_t, 3> lo{};
    std::array<double, 3> t{};
    for (int k = 0; k < 3; ++k) {
        const double g =
            std::clamp(point.at(k) / dx - 0.5, 0.0, static_cast<double>(n.at(k)) - 1.000001);
        lo.at(k) = static_cast<std::size_t>(g);
        t.at(k) = g - static_cast<double>(lo.at(k));
    }
    std::array<double, 3> out{};
    for (int corner = 0; corner < 8; ++corner) {
        double w = 1.0;
        std::array<std::size_t, 3> at{};
        for (int k = 0; k < 3; ++k) {
            const bool up = ((corner >> k) & 1) != 0;
            at.at(k) = std::min(lo.at(k) + (up ? 1 : 0), n.at(k) - 1);
            w *= up ? t.at(k) : 1.0 - t.at(k);
        }
        const std::array<double, 3> v = velocity(index(at[0], at[1], at[2]));
        for (int k = 0; k < 3; ++k) {
            out.at(k) += w * v.at(k);
        }
    }
    return out;
}

bool LbmBox::diverged() const {
    return std::any_of(velocity_.begin(), velocity_.end(),
                       [](float v) { return !std::isfinite(v) || std::abs(v) > 0.5F; });
}

}  // namespace lagrange::aerodynamics
