// The downwash's load on water surfaces (WaterAir.hpp).
#include "lagrange/aerodynamics/WaterAir.hpp"
#include "lagrange/random/Philox.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::aerodynamics {

namespace {

SignalSpec arraySpec(std::string name, Unit unit, std::size_t length) {
    SignalSpec s;
    s.name = std::move(name);
    s.type = SignalType::ScalarArray;
    s.unit = unit;
    s.arrayLength = static_cast<std::uint32_t>(length);
    s.frame = "world_local";
    return s;
}

SignalSpec scalarSpec(std::string name, Unit unit) {
    SignalSpec s;
    s.name = std::move(name);
    s.unit = unit;
    return s;
}

// kg/s: no named unit in core, the dimension is what wiring checks (as the fire domain's).
constexpr Unit kKgPerS{{.mass = 1, .time = -1}, 1.0, QuantityKind::Unspecified};

/// Surface cells per coarse spray cell, a side: drops are launched from 4 × 4 cells together.
constexpr int kSprayCoarse = 4;
/// Philox entities of the spray's draws, apart from the links' turbulence.
constexpr std::uint32_t kSprayEntity = 0x5350'0000U;

}  // namespace

double kelvinHelmholtzOnset(double rhoAir, double rhoWater, double tension, double gravity) {
    if (rhoAir <= 0.0 || rhoWater <= rhoAir || tension <= 0.0 || gravity <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(2.0 * (rhoAir + rhoWater) / (rhoAir * rhoWater) *
                     std::sqrt(gravity * tension * (rhoWater - rhoAir)));
}

double sprayEmission(double peakOutwash, double friction, double rhoAir, double onset) {
    if (peakOutwash <= onset || friction <= 0.0 || rhoAir <= 0.0) {
        return 0.0;
    }
    const double ratio = onset / peakOutwash;
    return kSprayEntrainment * rhoAir * friction * (1.0 - ratio * ratio);
}

double dropRelaxation(double diameter, double slip, double rhoAir, double rhoWater,
                      double viscosity) {
    const double stokes = rhoWater * diameter * diameter / (18.0 * viscosity);
    const double re = rhoAir * std::abs(slip) * diameter / viscosity;
    return stokes / (1.0 + 0.15 * std::pow(re, 0.687));
}

double dropTerminalSpeed(double diameter, double rhoAir, double gravity, double rhoWater,
                         double viscosity) {
    double v = gravity * dropRelaxation(diameter, 0.0, rhoAir, rhoWater, viscosity);
    for (int i = 0; i < 60; ++i) {  // v = g·τ(v): a contraction, from Stokes's speed down
        v = 0.5 * (v + gravity * dropRelaxation(diameter, v, rhoAir, rhoWater, viscosity));
    }
    return v;
}

double sprayExtinction(std::span<const double> water, double rhoWater) {
    double beta = 0.0;
    for (std::size_t i = 0; i < water.size() && i < kSprayDiameters.size(); ++i) {
        beta += 3.0 * water[i] / (rhoWater * kSprayDiameters.at(i));
    }
    return beta;
}

std::array<double, 2> patchCorner(const WaterGrid& grid, const std::array<double, 2>& centre) {
    const double half = grid.cells * grid.cell / 2.0;
    return {std::floor((centre[0] - half) / grid.cell) * grid.cell,
            std::floor((centre[1] - half) / grid.cell) * grid.cell};
}

Status WaterAir::declare(const std::vector<Downwash::WaterSurface>& water, const WaterGrid& grid,
                         bool spray, const std::vector<std::string>& rotors, double gravity,
                         SignalDeclarer& d) {
    pools_.clear();
    rotorSpray_.clear();
    drops_.clear();
    spray_ = spray && !water.empty();
    gravity_ = gravity > 0.0 ? gravity : 9.80665;
    if (grid.cells < 2 || grid.cell <= 0.0 || grid.step <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the water's surface grid needs cells >= 2, a cell and a step > 0");
    }
    grid_ = grid;
    const auto n = static_cast<std::size_t>(grid.cells) * static_cast<std::size_t>(grid.cells);
    for (const Downwash::WaterSurface& w : water) {
        Pool pool;
        pool.water = w;
        const std::string base = "water/" + w.name + "/";
        pool.patchOut = d.output(arraySpec(base + "patch", units::m, 2));
        pool.pressureOut = d.output(arraySpec(base + "air_pressure", units::Pa, n));
        pool.stressOut = d.output(arraySpec(base + "air_stress", units::Pa, 2 * n));
        pool.pressure.assign(n, 0.0);
        pool.stress.assign(2 * n, 0.0);
        if (spray_) {
            pool.peakOut = d.output(scalarSpec(base + "outwash_peak", units::m_per_s));
            pool.rateOut = d.output(scalarSpec(base + "spray_rate", kKgPerS));
            const int coarse = (grid.cells + kSprayCoarse - 1) / kSprayCoarse;
            pool.torn.assign(static_cast<std::size_t>(coarse * coarse), 0.0);
        }
        pools_.push_back(std::move(pool));
    }
    if (spray_) {
        for (const std::string& j : rotors) {
            rotorSpray_.push_back(d.output(scalarSpec("rotor/" + j + "/spray", units::kg_per_m3)));
        }
        sprayOut_ = d.output(scalarSpec("env/spray/downwash", units::kg_per_m3));
        extinctionOut_ = d.output(scalarSpec("env/spray/extinction", units::per_m));
        rateOut_ = d.output(scalarSpec("env/spray/rate", units::m_per_s));
        dropsOut_ = d.output(arraySpec("aero/spray_drops", units::m, 3 * kSprayMaxDrops));
        SignalSpec mass = arraySpec("aero/spray_mass", units::kg, kSprayMaxDrops);
        mass.frame.clear();
        massOut_ = d.output(mass);
        dropValues_.assign(3 * kSprayMaxDrops, std::numeric_limits<double>::quiet_NaN());
        massValues_.assign(kSprayMaxDrops, 0.0);
    }
    return {};
}

void WaterAir::init(SignalBus& bus) {
    next_ = 0.0;
    last_ = 0.0;
    drops_.clear();
    if (spray_) {
        for (const SignalId id : rotorSpray_) {
            bus.write(id, 0.0);
        }
        for (const SignalId id : {sprayOut_, extinctionOut_, rateOut_}) {
            bus.write(id, 0.0);
        }
        std::ranges::fill(dropValues_, std::numeric_limits<double>::quiet_NaN());
        bus.writeN(dropsOut_, dropValues_);
        std::ranges::fill(massValues_, 0.0);
        bus.writeN(massOut_, massValues_);
    }
    for (Pool& pool : pools_) {
        if (spray_) {
            std::ranges::fill(pool.torn, 0.0);
            bus.write(pool.peakOut, 0.0);
            bus.write(pool.rateOut, 0.0);
        }
        pool.placed = false;
        pool.loaded = false;
        std::ranges::fill(pool.pressure, 0.0);
        std::ranges::fill(pool.stress, 0.0);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bus.writeN(pool.patchOut, std::array<double, 2>{nan, nan});
        bus.writeN(pool.pressureOut, pool.pressure);
        bus.writeN(pool.stressOut, pool.stress);
    }
}

void WaterAir::step(const StepContext& context, const Downwash& downwash, double rhoAir,
                    std::span<const Downwash::Vec3> hubs, const AirAt& air, SignalBus& bus) {
    const double t = context.t.seconds();
    if (t + 1e-12 < next_) {
        return;
    }
    next_ = t + grid_.step;
    const double dt = std::max(t - last_, 0.0);
    last_ = t;
    // The onset over each body of water: its density against the air's, at 20 °C's tension.
    constexpr double kTension = 0.07275;  // N/m (Vargaftik et al. 1983), as the water's surface
    constexpr double kWaterDensity = 1000.0;
    const double onset = kelvinHelmholtzOnset(rhoAir, kWaterDensity, kTension, gravity_);
    const int n = grid_.cells;
    const double side = n * grid_.cell;
    for (std::size_t w = 0; w < pools_.size(); ++w) {
        Pool& pool = pools_[w];
        // The first wake on this water: the patch follows it.
        const Downwash::Group* over = nullptr;
        for (const Downwash::Group& g : downwash.groups()) {
            if (g.hits && g.hitWater == static_cast<int>(w)) {
                over = &g;
                break;
            }
        }
        if (over == nullptr) {
            if (spray_) {
                bus.write(pool.peakOut, 0.0);
                bus.write(pool.rateOut, 0.0);
            }
            if (pool.loaded) {
                std::ranges::fill(pool.pressure, 0.0);
                std::ranges::fill(pool.stress, 0.0);
                bus.writeN(pool.pressureOut, pool.pressure);
                bus.writeN(pool.stressOut, pool.stress);
                pool.loaded = false;
            }
            continue;
        }
        const std::array<double, 2> centre{over->hitPoint[0], over->hitPoint[1]};
        const bool inMiddle = pool.placed && centre[0] >= pool.corner[0] + side / 4.0 &&
                              centre[0] <= pool.corner[0] + 3.0 * side / 4.0 &&
                              centre[1] >= pool.corner[1] + side / 4.0 &&
                              centre[1] <= pool.corner[1] + 3.0 * side / 4.0;
        if (!inMiddle) {
            pool.corner = patchCorner(grid_, centre);
            pool.placed = true;
            bus.writeN(pool.patchOut, pool.corner);
        }
        const auto size = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
        const double level = pool.water.centre[2];
        const int coarse = (n + kSprayCoarse - 1) / kSprayCoarse;
        double peak = 0.0;
        double torn = 0.0;  // kg/s
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const auto i = static_cast<std::size_t>(y * n + x);
                const Downwash::Vec3 at{pool.corner[0] + (x + 0.5) * grid_.cell,
                                        pool.corner[1] + (y + 0.5) * grid_.cell, level};
                const bool wet = std::abs(at[0] - pool.water.centre[0]) <= pool.water.halfX &&
                                 std::abs(at[1] - pool.water.centre[1]) <= pool.water.halfY;
                if (!wet) {
                    pool.pressure[i] = pool.stress[i] = pool.stress[size + i] = 0.0;
                    continue;
                }
                pool.pressure[i] = downwash.surfacePressure(at, rhoAir, static_cast<int>(w));
                const Downwash::Vec3 f = downwash.surfaceFriction(at, static_cast<int>(w));
                const double speed = std::hypot(f[0], f[1]);
                pool.stress[i] = rhoAir * speed * f[0];
                pool.stress[size + i] = rhoAir * speed * f[1];
                if (spray_) {
                    const double um = speed / kWallJetFriction;
                    peak = std::max(peak, um);
                    const double rate =
                        sprayEmission(um, speed, rhoAir, onset) * grid_.cell * grid_.cell;
                    torn += rate;
                    pool.torn[static_cast<std::size_t>((y / kSprayCoarse) * coarse +
                                                       x / kSprayCoarse)] += rate * dt;
                }
            }
        }
        if (spray_) {
            bus.write(pool.peakOut, peak);
            bus.write(pool.rateOut, torn);
            // Launch: parcels of equal mass, at most kSprayDropsPerStep, from the coarse cells
            // that tore most, each a size in turn; what is left waits for the next step.
            const double total = torn * dt;
            const double quantum = total / static_cast<double>(kSprayDropsPerStep);
            random::Stream rng(context.seed, context.stepIndex,
                               kSprayEntity + static_cast<std::uint32_t>(w));
            const double coarseSide = kSprayCoarse * grid_.cell;
            for (int cy = 0; cy < coarse && quantum > 0.0; ++cy) {
                for (int cx = 0; cx < coarse; ++cx) {
                    double& left = pool.torn[static_cast<std::size_t>(cy * coarse + cx)];
                    while (left >= quantum && drops_.size() < kSprayMaxDrops) {
                        left -= quantum;
                        Drop drop;
                        drop.at = {pool.corner[0] + (cx + rng.uniform()) * coarseSide,
                                   pool.corner[1] + (cy + rng.uniform()) * coarseSide,
                                   level + 0.005};  // m: within the wall jet's lowest layer
                        const Downwash::Vec3 u = air(drop.at);
                        const double up = kSprayTurbulence * std::hypot(u[0], u[1], u[2]);
                        drop.velocity = {u[0], u[1], u[2] + up};
                        drop.mass = quantum;
                        drop.size = static_cast<int>(drops_.size() % kSprayDiameters.size());
                        drop.water = static_cast<int>(w);
                        drops_.push_back(drop);
                    }
                    if (drops_.size() >= kSprayMaxDrops) {
                        left = 0.0;  // the cloud is full: this water is not carried
                    }
                }
            }
        }
        bus.writeN(pool.pressureOut, pool.pressure);
        bus.writeN(pool.stressOut, pool.stress);
        pool.loaded = true;
    }
    if (!spray_) {
        return;
    }
    moveDrops(context, dt, rhoAir, air);
    for (std::size_t j = 0; j < rotorSpray_.size(); ++j) {
        bus.write(rotorSpray_[j], j < hubs.size() ? sprayAt(hubs[j]) : 0.0);
    }
    double water = 0.0;
    double extinction = 0.0;
    double rate = 0.0;
    if (!downwash.groups().empty()) {
        const Downwash::Vec3& c = downwash.groups().front().centre;
        Downwash::Vec3 v{};
        water = sprayAt(c, &v);
        const std::array<double, 3> sizes = sprayBySize(c, kSprayViewReach);
        extinction = sprayExtinction(sizes, kWaterDensity);
        rate = water * std::hypot(v[0], v[1], v[2]) / kWaterDensity;
    }
    bus.write(sprayOut_, water);
    bus.write(extinctionOut_, extinction);
    bus.write(rateOut_, rate);
    std::ranges::fill(dropValues_, std::numeric_limits<double>::quiet_NaN());
    std::ranges::fill(massValues_, 0.0);
    for (std::size_t i = 0; i < drops_.size(); ++i) {
        std::copy(drops_[i].at.begin(), drops_[i].at.end(),
                  dropValues_.begin() + static_cast<std::ptrdiff_t>(3 * i));
        massValues_[i] = drops_[i].mass;
    }
    bus.writeN(dropsOut_, dropValues_);
    bus.writeN(massOut_, massValues_);
}

void WaterAir::moveDrops(const StepContext& context, double dt, double rhoAir, const AirAt& air) {
    if (dt <= 0.0) {
        return;
    }
    const double keep = std::exp(-dt / kSprayLagrangianTime);
    const double fresh = std::sqrt(1.0 - keep * keep);
    for (std::size_t i = 0; i < drops_.size(); ++i) {
        Drop& d = drops_[i];
        random::Stream rng(context.seed, context.stepIndex,
                           kSprayEntity + 0x100U + static_cast<std::uint32_t>(i));
        const Downwash::Vec3 u = air(d.at);
        const double sigma = kSprayTurbulence * std::hypot(u[0], u[1], u[2]);
        for (double& g : d.gust) {
            g = keep * g + sigma * fresh * rng.normal();
        }
        const Downwash::Vec3 seen{u[0] + d.gust[0], u[1] + d.gust[1], u[2] + d.gust[2]};
        const double slip =
            std::hypot(d.velocity[0] - seen[0], d.velocity[1] - seen[1], d.velocity[2] - seen[2]);
        const double tau =
            dropRelaxation(kSprayDiameters.at(static_cast<std::size_t>(d.size)), slip, rhoAir);
        // Exactly over the step for the air held: v → u + g·τ, with x its integral.
        const double e = std::exp(-dt / tau);
        for (std::size_t k = 0; k < 3; ++k) {
            const double settled = seen[k] - (k == 2 ? gravity_ * tau : 0.0);
            const double v0 = d.velocity[k];
            d.at[k] += settled * dt + (v0 - settled) * tau * (1.0 - e);
            d.velocity[k] = settled + (v0 - settled) * e;
        }
    }
    // Back under the surface it fell from (or far below it): gone.
    std::erase_if(drops_, [&](const Drop& d) {
        return d.at[2] < pools_.at(static_cast<std::size_t>(d.water)).water.centre[2];
    });
}

double WaterAir::sprayAt(const Downwash::Vec3& x, Downwash::Vec3* velocity, double reach) const {
    const double volume = 4.0 / 3.0 * std::numbers::pi * reach * reach * reach;
    double mass = 0.0;
    Downwash::Vec3 momentum{};
    for (const Drop& d : drops_) {
        const double dx = d.at[0] - x[0];
        const double dy = d.at[1] - x[1];
        const double dz = d.at[2] - x[2];
        if (dx * dx + dy * dy + dz * dz <= reach * reach) {
            mass += d.mass;
            for (std::size_t k = 0; k < 3; ++k) {
                momentum[k] += d.mass * d.velocity[k];
            }
        }
    }
    if (velocity != nullptr) {
        *velocity = mass > 0.0
                        ? Downwash::Vec3{momentum[0] / mass, momentum[1] / mass, momentum[2] / mass}
                        : Downwash::Vec3{};
    }
    return mass / volume;
}

std::array<double, 3> WaterAir::sprayBySize(const Downwash::Vec3& x, double reach) const {
    const double volume = 4.0 / 3.0 * std::numbers::pi * reach * reach * reach;
    std::array<double, 3> out{};
    for (const Drop& d : drops_) {
        const double dx = d.at[0] - x[0];
        const double dy = d.at[1] - x[1];
        const double dz = d.at[2] - x[2];
        if (dx * dx + dy * dy + dz * dz <= reach * reach) {
            out.at(static_cast<std::size_t>(d.size)) += d.mass / volume;
        }
    }
    return out;
}

void WaterAir::saveState(StateWriter& writer) const {
    writer.put(next_);
    writer.put(last_);
    writer.put<std::uint64_t>(drops_.size());
    for (const Drop& d : drops_) {
        writer.put(d);
    }
    for (const Pool& pool : pools_) {
        writer.put(pool.corner);
        writer.put(pool.placed);
        writer.putDoubles(pool.torn);
    }
}

bool WaterAir::restoreState(StateReader& reader) {
    std::uint64_t count = 0;
    if (!reader.get(next_) || !reader.get(last_) || !reader.get(count) || count > kSprayMaxDrops) {
        return false;
    }
    drops_.resize(count);
    for (Drop& d : drops_) {
        if (!reader.get(d)) {
            return false;
        }
    }
    for (Pool& pool : pools_) {
        if (!reader.get(pool.corner) || !reader.get(pool.placed) || !reader.getDoubles(pool.torn)) {
            return false;
        }
        pool.loaded = true;
    }
    return true;
}

}  // namespace lagrange::aerodynamics
