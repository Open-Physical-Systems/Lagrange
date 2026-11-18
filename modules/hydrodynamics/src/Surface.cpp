// A moving patch of water surface: linear capillary-gravity waves (Surface.hpp; water.md).
#include "lagrange/hydrodynamics/Surface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace lagrange::hydrodynamics {

namespace {

using Complex = std::complex<double>;

/// Twiddles and the bit-reversal permutation of one size, kept per thread.
struct Plan {
    int n{0};
    std::vector<Complex> twiddle;  // e^(−2πi·j/n), j < n/2
    std::vector<int> reversed;
};

const Plan& planFor(int n) {
    thread_local Plan plan;
    if (plan.n != n) {
        plan.n = n;
        plan.twiddle.resize(static_cast<std::size_t>(n / 2));
        for (int j = 0; j < n / 2; ++j) {
            const double a = -2.0 * std::numbers::pi * j / n;
            plan.twiddle[static_cast<std::size_t>(j)] = {std::cos(a), std::sin(a)};
        }
        plan.reversed.resize(static_cast<std::size_t>(n));
        int bits = 0;
        while ((1 << bits) < n) {
            ++bits;
        }
        for (int i = 0; i < n; ++i) {
            int r = 0;
            for (int b = 0; b < bits; ++b) {
                r |= ((i >> b) & 1) << (bits - 1 - b);
            }
            plan.reversed[static_cast<std::size_t>(i)] = r;
        }
    }
    return plan;
}

/// One radix-2 transform of `n` values `stride` apart from `first`.
void fft1(Complex* first, int n, int stride, const Plan& plan, bool inverse) {
    for (int i = 0; i < n; ++i) {
        const int r = plan.reversed[static_cast<std::size_t>(i)];
        if (r > i) {
            std::swap(first[i * stride], first[r * stride]);
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len / 2;
        const int step = n / len;
        for (int start = 0; start < n; start += len) {
            for (int j = 0; j < half; ++j) {
                Complex w = plan.twiddle[static_cast<std::size_t>(j * step)];
                if (inverse) {
                    w = std::conj(w);
                }
                Complex& a = first[(start + j) * stride];
                Complex& b = first[(start + j + half) * stride];
                const Complex t = w * b;
                b = a - t;
                a += t;
            }
        }
    }
}

/// The two real grids a and b packed as a + i·b, transformed, split: â_k, b̂_k.
void split(const std::vector<Complex>& z, int n, std::vector<Complex>& a, std::vector<Complex>& b) {
    for (int y = 0; y < n; ++y) {
        const int my = (n - y) % n;
        for (int x = 0; x < n; ++x) {
            const int mx = (n - x) % n;
            const Complex zk = z[static_cast<std::size_t>(y * n + x)];
            const Complex zm = std::conj(z[static_cast<std::size_t>(my * n + mx)]);
            a[static_cast<std::size_t>(y * n + x)] = 0.5 * (zk + zm);
            b[static_cast<std::size_t>(y * n + x)] = Complex(0.0, -0.5) * (zk - zm);
        }
    }
}

}  // namespace

void fft2(std::vector<Complex>& data, int n, bool inverse) {
    const Plan& plan = planFor(n);
    thread_local std::vector<Complex> turned;
    turned.resize(data.size());
    const auto un = static_cast<std::size_t>(n);
    // Rows, then the columns as rows of the transposed grid (contiguous, cache-friendly).
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t y = 0; y < un; ++y) {
            fft1(data.data() + y * un, n, 1, plan, inverse);
        }
        constexpr std::size_t kBlock = 16;
        for (std::size_t by = 0; by < un; by += kBlock) {
            for (std::size_t bx = 0; bx < un; bx += kBlock) {
                for (std::size_t y = by; y < std::min(by + kBlock, un); ++y) {
                    for (std::size_t x = bx; x < std::min(bx + kBlock, un); ++x) {
                        turned[x * un + y] = data[y * un + x];
                    }
                }
            }
        }
        data.swap(turned);
    }
    if (inverse) {
        const double scale = 1.0 / (static_cast<double>(n) * n);
        for (Complex& c : data) {
            c *= scale;
        }
    }
}

double SurfacePatch::omega(double k, double gravity, double tension, double density, double depth) {
    if (k <= 0.0) {
        return 0.0;
    }
    return std::sqrt((gravity * k + tension * k * k * k / density) * std::tanh(k * depth));
}

SurfacePatch::SurfacePatch(const SurfaceParams& params) : p_(params) {
    if (p_.cells < 2 || (p_.cells & (p_.cells - 1)) != 0 || p_.cell <= 0.0 || p_.depth <= 0.0 ||
        p_.density <= 0.0) {
        throw std::invalid_argument("SurfacePatch: cells a power of two, cell, depth, density > 0");
    }
    const int n = p_.cells;
    const auto size = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    eta_.assign(size, 0.0);
    rate_.assign(size, 0.0);
    wet_.assign(size, true);
    for (auto* v : {&k_, &kx_, &ky_, &omega_, &gamma_, &pressureGain_, &stressGain_}) {
        v->assign(size, 0.0);
    }
    for (auto* v : {&a_, &b_, &f_, &g_}) {
        v->assign(size, Complex{});
    }
    const double dk = 2.0 * std::numbers::pi / (n * p_.cell);
    for (int y = 0; y < n; ++y) {
        const double ky = dk * (y < n / 2 ? y : y - n);
        for (int x = 0; x < n; ++x) {
            const double kx = dk * (x < n / 2 ? x : x - n);
            const auto i = static_cast<std::size_t>(y * n + x);
            const double k = std::hypot(kx, ky);
            kx_[i] = kx;
            ky_[i] = ky;
            k_[i] = k;
            omega_[i] = omega(k, p_.gravity, p_.surfaceTension, p_.density, p_.depth);
            // Viscous decay, held under the oscillation (it is far under it below k ~ 10⁵ /m).
            gamma_[i] = std::min(2.0 * p_.viscosity * k * k, 0.5 * omega_[i]);
            if (k > 0.0) {
                const double t = std::tanh(k * p_.depth);
                pressureGain_[i] = -k * t / p_.density;
                stressGain_[i] = -t / (k * p_.depth) / p_.density;
            }
        }
    }
    buildDamping();
}

void SurfacePatch::buildDamping() {
    const int n = p_.cells;
    damping_.assign(eta_.size(), 0.0);
    const double band = std::max(1.0, p_.spongeShare * n);
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const auto i = static_cast<std::size_t>(y * n + x);
            // Distance in cells from the nearest edge, the sponge growing as its square.
            const double edge = std::min({x + 0.5, n - x - 0.5, y + 0.5, n - y - 0.5});
            const double s = std::clamp((band - edge) / band, 0.0, 1.0);
            const double cx = origin_[0] + (x + 0.5) * p_.cell;
            const double cy = origin_[1] + (y + 0.5) * p_.cell;
            wet_[i] =
                cx >= waterLo_[0] && cx <= waterHi_[0] && cy >= waterLo_[1] && cy <= waterHi_[1];
            damping_[i] = wet_[i] ? p_.spongeRate * s * s : 4.0 * p_.spongeRate;
        }
    }
}

void SurfacePatch::setWater(const std::array<double, 2>& lo, const std::array<double, 2>& hi) {
    waterLo_ = lo;
    waterHi_ = hi;
    buildDamping();
}

void SurfacePatch::moveTo(const std::array<double, 2>& origin) {
    const auto sx = static_cast<long>(std::lround((origin[0] - origin_[0]) / p_.cell));
    const auto sy = static_cast<long>(std::lround((origin[1] - origin_[1]) / p_.cell));
    if (sx == 0 && sy == 0) {
        return;
    }
    const int n = p_.cells;
    std::vector<double> eta(eta_.size(), 0.0);
    std::vector<double> rate(rate_.size(), 0.0);
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const long ox = x + sx;  // the old cell over the same water
            const long oy = y + sy;
            if (ox < 0 || oy < 0 || ox >= n || oy >= n) {
                continue;
            }
            const auto from = static_cast<std::size_t>(oy * n + ox);
            eta[static_cast<std::size_t>(y * n + x)] = eta_[from];
            rate[static_cast<std::size_t>(y * n + x)] = rate_[from];
        }
    }
    eta_ = std::move(eta);
    rate_ = std::move(rate);
    origin_[0] += static_cast<double>(sx) * p_.cell;
    origin_[1] += static_cast<double>(sy) * p_.cell;
    buildDamping();
}

void SurfacePatch::step(double dt, std::span<const double> pressure,
                        std::span<const double> stressX, std::span<const double> stressY) {
    if (dt <= 0.0) {
        return;
    }
    const int n = p_.cells;
    const std::size_t size = eta_.size();
    const bool loaded = pressure.size() == size;
    const bool sheared = stressX.size() == size && stressY.size() == size;
    // η + i·η̇ and, for the load, p + i·τx and τy.
    for (std::size_t i = 0; i < size; ++i) {
        a_[i] = {eta_[i], rate_[i]};
    }
    fft2(a_, n, false);
    thread_local std::vector<Complex> eta, rate, ty;
    eta.resize(size);
    rate.resize(size);
    split(a_, n, eta, rate);
    std::ranges::fill(f_, Complex{});  // the forcing per mode
    if (loaded || sheared) {
        for (std::size_t i = 0; i < size; ++i) {
            b_[i] = {loaded ? pressure[i] : 0.0, sheared ? stressX[i] : 0.0};
        }
        fft2(b_, n, false);
        split(b_, n, f_, g_);  // f_ = p̂, g_ = τ̂x
        ty.assign(size, Complex{});
        if (sheared) {
            for (std::size_t i = 0; i < size; ++i) {
                ty[i] = {stressY[i], 0.0};
            }
            fft2(ty, n, false);
        }
        for (std::size_t i = 0; i < size; ++i) {
            const Complex divergence = Complex(0.0, 1.0) * (kx_[i] * g_[i] + ky_[i] * ty[i]);
            f_[i] = pressureGain_[i] * f_[i] + stressGain_[i] * (sheared ? divergence : 0.0);
        }
    }
    // Each mode exactly over dt: the damped oscillator about its steady answer F/ω².
    for (std::size_t i = 0; i < size; ++i) {
        const double w = omega_[i];
        if (w <= 0.0) {
            eta[i] += rate[i] * dt;  // the mean: water neither comes nor goes
            continue;
        }
        const Complex steady = f_[i] / (w * w);
        const Complex e0 = eta[i] - steady;
        const Complex v0 = rate[i];
        const double g = gamma_[i];
        const double wd = std::sqrt(w * w - g * g);
        const double decay = std::exp(-g * dt);
        const double c = std::cos(wd * dt);
        const double s = std::sin(wd * dt);
        eta[i] = steady + decay * (e0 * c + (v0 + g * e0) / wd * s);
        rate[i] = decay * (v0 * c - (w * w * e0 + g * v0) / wd * s);
    }
    for (std::size_t i = 0; i < size; ++i) {
        a_[i] = eta[i] + Complex(0.0, 1.0) * rate[i];
    }
    fft2(a_, n, true);
    for (std::size_t i = 0; i < size; ++i) {
        eta_[i] = a_[i].real();
        rate_[i] = a_[i].imag() * std::exp(-damping_[i] * dt);
        if (!wet_[i]) {
            eta_[i] *= std::exp(-damping_[i] * dt);  // the shore keeps no water above it
        }
    }
}

double SurfacePatch::heightAt(double x, double y) const {
    const int n = p_.cells;
    const double fx = (x - origin_[0]) / p_.cell - 0.5;
    const double fy = (y - origin_[1]) / p_.cell - 0.5;
    if (fx < 0.0 || fy < 0.0 || fx > n - 1 || fy > n - 1) {
        return 0.0;
    }
    const int col = std::min(static_cast<int>(fx), n - 2);
    const int row = std::min(static_cast<int>(fy), n - 2);
    const double tx = fx - col;
    const double ty = fy - row;
    const auto at = [&](int cx, int cy) { return eta_[static_cast<std::size_t>(cy * n + cx)]; };
    return (1.0 - ty) * ((1.0 - tx) * at(col, row) + tx * at(col + 1, row)) +
           ty * ((1.0 - tx) * at(col, row + 1) + tx * at(col + 1, row + 1));
}

void SurfacePatch::reset() {
    std::ranges::fill(eta_, 0.0);
    std::ranges::fill(rate_, 0.0);
}

void SurfacePatch::setState(const std::array<double, 2>& origin, std::span<const double> heights,
                            std::span<const double> rates) {
    origin_ = origin;
    if (heights.size() == eta_.size() && rates.size() == rate_.size()) {
        std::ranges::copy(heights, eta_.begin());
        std::ranges::copy(rates, rate_.begin());
    }
    buildDamping();
}

double gridHeight(std::span<const double> heights, int cells, double cell,
                  const std::array<double, 2>& origin, double x, double y) {
    const auto n = static_cast<std::size_t>(cells);
    if (cells < 2 || cell <= 0.0 || heights.size() != n * n || !std::isfinite(origin[0]) ||
        !std::isfinite(origin[1])) {
        return 0.0;
    }
    const double fx = (x - origin[0]) / cell - 0.5;
    const double fy = (y - origin[1]) / cell - 0.5;
    if (!(fx >= 0.0 && fy >= 0.0 && fx <= cells - 1 && fy <= cells - 1)) {
        return 0.0;
    }
    const int col = std::min(static_cast<int>(fx), cells - 2);
    const int row = std::min(static_cast<int>(fy), cells - 2);
    const double tx = fx - col;
    const double ty = fy - row;
    const auto at = [&](int cx, int cy) {
        return heights[static_cast<std::size_t>(cy) * n + static_cast<std::size_t>(cx)];
    };
    return (1.0 - ty) * ((1.0 - tx) * at(col, row) + tx * at(col + 1, row)) +
           ty * ((1.0 - tx) * at(col, row + 1) + tx * at(col + 1, row + 1));
}

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

/// Below this (m, m/s, Pa) a surface counts as flat, still and unloaded.
constexpr double kQuiet = 1e-9;

}  // namespace

ModuleInfo SurfaceWaves::info() const {
    return {"hydrodynamics", "surface", 1, 0};
}

Duration SurfaceWaves::preferredStep() const {
    return step_;
}

Status SurfaceWaves::declare(const WorldView& world, const ModuleConfig& config,
                             SignalDeclarer& d) {
    pools_.clear();
    const double stepS = config.get<double>("surface_step").value_or(0.04);
    const auto cells = config.get<std::int64_t>("surface_cells").value_or(128);
    const double cell = config.get<double>("surface_cell").value_or(0.03);
    if (stepS <= 0.0 || cells < 2 || cells > 1024 || (cells & (cells - 1)) != 0 || cell <= 0.0) {
        return Status::error(StatusCode::InvalidConfig,
                             "the water's surface needs surface_step > 0 s, surface_cells a "
                             "power of two up to 1024 and surface_cell > 0 m");
    }
    step_ = Duration{static_cast<std::int64_t>(std::llround(stepS * 1e9))};
    const double gravity = std::hypot(world.raw().gravityLocal[0], world.raw().gravityLocal[1],
                                      world.raw().gravityLocal[2]);
    for (const WaterBody& w : waterOf(world)) {
        SurfaceParams params;
        params.cells = static_cast<int>(cells);
        params.cell = cell;
        params.depth = std::max(w.depth, 1e-3);
        params.density = w.density;
        params.gravity = gravity > 0.0 ? gravity : 9.80665;
        Pool pool{w, SurfacePatch(params), {}, {}, {}, {}, {}, {}, {}, true};
        const auto n = static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells);
        const std::string base = "water/" + w.name + "/";
        SignalSpec patch = arraySpec(base + "patch", units::m, 2);
        patch.optional = patch.feedback = true;
        patch.defaultValue = std::numeric_limits<double>::quiet_NaN();
        SignalSpec pressure = arraySpec(base + "air_pressure", units::Pa, n);
        pressure.optional = pressure.feedback = true;
        SignalSpec stress = arraySpec(base + "air_stress", units::Pa, 2 * n);
        stress.optional = stress.feedback = true;
        pool.patchIn = d.input(patch);
        pool.pressureIn = d.input(pressure);
        pool.stressIn = d.input(stress);
        pool.surfaceOut = d.output(arraySpec(base + "surface", units::m, n));
        pool.originOut = d.output(arraySpec(base + "surface_origin", units::m, 2));
        SignalSpec low;
        low.name = base + "surface_min";
        low.unit = units::m;
        SignalSpec high = low;
        high.name = base + "surface_max";
        pool.minOut = d.output(low);
        pool.maxOut = d.output(high);
        pools_.push_back(std::move(pool));
    }
    return {};
}

void SurfaceWaves::write(Pool& pool, SignalBus& bus) const {
    const std::span<const double> eta = pool.patch.heights();
    bus.writeN(pool.surfaceOut, eta);
    const std::array<double, 2> origin = pool.patch.origin();
    bus.writeN(pool.originOut, origin);
    const auto [lo, hi] = std::ranges::minmax(eta);
    bus.write(pool.minOut, lo);
    bus.write(pool.maxOut, hi);
}

Status SurfaceWaves::init(const WorldView&, const ModuleConfig&, SignalBus& bus) {
    for (Pool& pool : pools_) {
        const WaterBody& w = pool.water;
        const double half = pool.patch.side() / 2.0;
        pool.patch.reset();
        // At whole multiples of the cell, as the aerodynamics domain puts its load grid.
        const double cell = pool.patch.params().cell;
        pool.patch.setState({std::floor((w.surface[0] - half) / cell) * cell,
                             std::floor((w.surface[1] - half) / cell) * cell},
                            {}, {});
        pool.patch.setWater({w.surface[0] - w.sizeX / 2.0, w.surface[1] - w.sizeY / 2.0},
                            {w.surface[0] + w.sizeX / 2.0, w.surface[1] + w.sizeY / 2.0});
        pool.quiet = true;
        write(pool, bus);
    }
    return {};
}

Status SurfaceWaves::step(const StepContext& context, SignalBus& bus) {
    const double dt = context.dt.seconds();
    for (Pool& pool : pools_) {
        const std::span<const double> corner = bus.readN(pool.patchIn);
        if (corner.size() == 2 && std::isfinite(corner[0]) && std::isfinite(corner[1])) {
            pool.patch.moveTo({corner[0], corner[1]});
        }
        const std::span<const double> pressure = bus.readN(pool.pressureIn);
        const std::span<const double> stress = bus.readN(pool.stressIn);
        const std::size_t n = pool.patch.heights().size();
        const auto loaded = [](std::span<const double> v) {
            return std::ranges::any_of(v, [](double x) { return std::abs(x) > kQuiet; });
        };
        const bool load = loaded(pressure) || loaded(stress);
        if (!load && pool.quiet) {
            continue;  // flat, still and nothing on it: no work
        }
        pool.patch.step(dt, pressure.size() == n ? pressure : std::span<const double>{},
                        stress.size() == 2 * n ? stress.first(n) : std::span<const double>{},
                        stress.size() == 2 * n ? stress.subspan(n) : std::span<const double>{});
        pool.quiet = !load && !loaded(pool.patch.heights()) && !loaded(pool.patch.rates());
        write(pool, bus);
    }
    return {};
}

Status SurfaceWaves::saveState(StateWriter& writer) const {
    for (const Pool& pool : pools_) {
        const std::array<double, 2> origin = pool.patch.origin();
        writer.putDoubles(origin);
        writer.putDoubles(pool.patch.heights());
        writer.putDoubles(pool.patch.rates());
    }
    return {};
}

Status SurfaceWaves::restoreState(StateReader& reader) {
    for (Pool& pool : pools_) {
        std::array<double, 2> origin{};
        std::vector<double> heights(pool.patch.heights().size());
        std::vector<double> rates(heights.size());
        if (!reader.getDoubles(origin) || !reader.getDoubles(heights) ||
            !reader.getDoubles(rates)) {
            return Status::error(StatusCode::InvalidConfig, "the water's surface state is short");
        }
        pool.patch.setState({origin[0], origin[1]}, heights, rates);
        pool.quiet = false;
    }
    return {};
}

}  // namespace lagrange::hydrodynamics
