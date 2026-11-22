// Smoke puffs and their extinction field (RFC 0022 §4); the model is described in Smoke.hpp.
#include "lagrange/fire/Correlations.hpp"
#include "lagrange/fire/Smoke.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace lagrange::fire {

namespace {

// Heskestad's plume radius to where the excess temperature is half the centreline's,
// b = 0.12·(T0/T∞)^½·(z − z0), with the temperature factor taken as 1. TODO(verify) the
// coefficient and the factor: SFPE Handbook 5th ed. ch. 13 (Heskestad), not openly readable
// (RFC 0022 §4).
constexpr double kPlumeHalfWidth = 0.12;
// A Gaussian's half-value radius over its σ: √(2 ln 2).
const double kHalfOverSigma = std::sqrt(2.0 * std::numbers::ln2);
// c_p of dry air near room temperature (J/(kg·K)), for Heskestad's plume velocity.
// TODO(verify) against a property table (the plume tests use the same value).
constexpr double kAirCp = 1005.0;
// A puff spreads over the voxels within this many σ of its centre.
constexpr double kSplatSigmas = 3.0;
// m: how far below the lowest fire's base the box reaches (smoke that sinks is not modelled).
constexpr double kBoxBelow = 1.0;
constexpr std::size_t kPuffValues = 9;

double clampSigma(double sigma, double voxel) {
    return std::max(sigma, 0.5 * voxel);
}

/// The plume's σ at `height` above the fire's base.
double plumeSigma(double height, double virtualOriginHeight, double voxel) {
    return clampSigma(
        kPlumeHalfWidth * std::max(height - virtualOriginHeight, 0.0) / kHalfOverSigma, voxel);
}

/// The rise velocity of a puff at `z` over its fire (m/s).
double riseVelocity(const PlumeSource& f, double z, const SmokeAir& air) {
    if (!(f.hrr > 0.0) || !(air.temperature > 0.0) || !(air.density > 0.0)) {
        return 0.0;
    }
    return plumeVelocity(f.hrr, f.radiativeFraction, z - f.base[2], f.virtualOriginHeight,
                         air.gravity, kAirCp, air.temperature, air.density);
}

/// 1D weights of a Gaussian at `centre` (σ) over voxel centres `origin + i·voxel`, i in [lo, hi],
/// normalised to 1 over the 3σ span (before any clipping to the grid).
/// Nothing of it in voxels whose centre is above `top`.
void weights(double centre, double sigma, double origin, double voxel, int& lo, int& hi,
             std::vector<double>& w, double top = std::numeric_limits<double>::infinity()) {
    const double reach = kSplatSigmas * sigma;
    lo = static_cast<int>(std::ceil((centre - reach - origin) / voxel - 1e-9));
    hi = static_cast<int>(std::floor((std::min(centre + reach, top) - origin) / voxel + 1e-9));
    if (hi < lo) {  // narrower than a voxel: all in the nearest one
        lo = hi = static_cast<int>(std::lround((centre - origin) / voxel));
    }
    w.assign(static_cast<std::size_t>(hi - lo + 1), 0.0);
    double sum = 0.0;
    for (int i = lo; i <= hi; ++i) {
        const double x = (origin + i * voxel - centre) / sigma;
        const double v = std::exp(-0.5 * x * x);
        w[static_cast<std::size_t>(i - lo)] = v;
        sum += v;
    }
    if (sum > 0.0) {
        for (double& v : w) {
            v /= sum;
        }
    }
}

}  // namespace

double smokeExtinction(double sootDensity) {
    return kSpecificExtinction * std::max(sootDensity, 0.0);
}

void Smoke::configure(const SmokeSettings& settings, std::span<const Vec3> fireBases) {
    settings_ = settings;
    puffs_.clear();
    left_ = 0.0;
    if (fireBases.empty()) {
        box_ = {};
        return;
    }
    Vec3 lo = fireBases[0];
    Vec3 hi = fireBases[0];
    for (const Vec3& b : fireBases) {
        for (std::size_t a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], b[a]);
            hi[a] = std::max(hi[a], b[a]);
        }
    }
    box_ = {Vec3{lo[0] - settings.reach, lo[1] - settings.reach, lo[2] - kBoxBelow},
            Vec3{hi[0] + settings.reach, hi[1] + settings.reach, lo[2] + settings.height}};
}

void Smoke::step(double dt, std::span<const PlumeSource> fires, const SmokeAir& air,
                 const services::SceneQuery* ceilings) {
    const double voxel = settings_.voxel;
    // Move the puffs there are: wind, and the plume's rise up to a ceiling.
    std::vector<Puff> kept;
    kept.reserve(puffs_.size() + fires.size());
    for (Puff p : puffs_) {
        const PlumeSource* f = p.fire < fires.size() ? &fires[p.fire] : nullptr;
        const double rise = f != nullptr ? riseVelocity(*f, p.centre[2], air) : 0.0;
        for (std::size_t a = 0; a < 3; ++a) {
            p.centre[a] += air.wind[a] * dt;
        }
        // The ceiling above it, within its next rise and the reach of its spread: nothing of it
        // is above that (when it rises close under a roof, too).
        double ceiling = std::numeric_limits<double>::infinity();
        const double look = (rise * dt) + (kSplatSigmas * p.sigmaV) + voxel;
        if (ceilings != nullptr) {
            const services::RayHit hit = ceilings->ray(p.centre, {0.0, 0.0, 1.0}, look);
            if (hit.hit()) {
                ceiling = p.centre[2] + hit.distance;
            }
        }
        if (p.centre[2] + (rise * dt) + p.sigmaV >= ceiling) {
            // Under it: no higher than its depth below it, spreading at its arrival velocity.
            p.centre[2] = std::min(p.centre[2] + (rise * dt), ceiling - p.sigmaV);
            p.sigmaH += kPlumeHalfWidth / kHalfOverSigma * rise * dt;
            p.underCeiling = true;
        } else {
            p.centre[2] += rise * dt;
            p.underCeiling = false;
            if (f != nullptr) {
                const double sigma =
                    plumeSigma(p.centre[2] - f->base[2], f->virtualOriginHeight, voxel);
                p.sigmaH = std::max(p.sigmaH, sigma);
                p.sigmaV = std::max(p.sigmaV, sigma);
            }
        }
        p.top = ceiling;
        bool inside = true;
        for (std::size_t a = 0; a < 3; ++a) {
            inside = inside && p.centre[a] >= box_[0][a] && p.centre[a] <= box_[1][a];
        }
        if (inside) {
            kept.push_back(p);
        } else {
            left_ += p.mass;
        }
    }
    puffs_ = std::move(kept);
    // This step's soot, at each flame's tip.
    for (std::size_t k = 0; k < fires.size(); ++k) {
        const PlumeSource& f = fires[k];
        const double mass = f.sootRate * dt;
        if (!(mass > 0.0)) {
            continue;
        }
        Puff p;
        p.centre = f.tip;
        p.mass = mass;
        p.sigmaH = p.sigmaV = plumeSigma(f.tip[2] - f.base[2], f.virtualOriginHeight, voxel);
        p.fire = static_cast<std::uint32_t>(k);
        if (ceilings != nullptr) {
            const services::RayHit hit =
                ceilings->ray(p.centre, {0.0, 0.0, 1.0}, (kSplatSigmas * p.sigmaV) + voxel);
            if (hit.hit()) {
                p.top = p.centre[2] + hit.distance;
            }
        }
        puffs_.push_back(p);
    }
    while (puffs_.size() > std::max<std::size_t>(settings_.maxPuffs, 2)) {
        mergeOldest();
    }
}

void Smoke::mergeOldest() {
    // One puff with the two's mass, centre of mass and second moments.
    const Puff a = puffs_[0];
    const Puff b = puffs_[1];
    const double m = a.mass + b.mass;
    Puff p = a;
    if (m > 0.0) {
        for (std::size_t k = 0; k < 3; ++k) {
            p.centre[k] = (a.mass * a.centre[k] + b.mass * b.centre[k]) / m;
        }
        const auto moment = [&](const Puff& q, bool across) {
            const double dx = q.centre[0] - p.centre[0];
            const double dy = q.centre[1] - p.centre[1];
            const double dz = q.centre[2] - p.centre[2];
            return across ? q.mass * ((q.sigmaH * q.sigmaH) + (0.5 * (dx * dx + dy * dy)))
                          : q.mass * ((q.sigmaV * q.sigmaV) + (dz * dz));
        };
        p.sigmaH = std::sqrt((moment(a, true) + moment(b, true)) / m);
        p.sigmaV = std::sqrt((moment(a, false) + moment(b, false)) / m);
    }
    p.mass = m;
    p.underCeiling = a.underCeiling || b.underCeiling;
    p.top = std::max(a.top, b.top);
    puffs_.erase(puffs_.begin(), puffs_.begin() + 2);
    puffs_.insert(puffs_.begin(), p);
}

double Smoke::mass() const {
    double m = 0.0;
    for (const Puff& p : puffs_) {
        m += p.mass;
    }
    return m;
}

std::shared_ptr<const fields::Field> Smoke::raster() const {
    if (puffs_.empty()) {
        return nullptr;
    }
    const double voxel = settings_.voxel;
    const Vec3& origin = box_[0];
    // The lattice from the box's corner; the grid spans the box.
    std::array<int, 3> count{};
    for (std::size_t a = 0; a < 3; ++a) {
        count[a] = static_cast<int>(std::floor((box_[1][a] - box_[0][a]) / voxel)) + 1;
    }
    // Only the part of the box the puffs reach.
    std::array<int, 3> lo{count[0], count[1], count[2]};
    std::array<int, 3> hi{-1, -1, -1};
    for (const Puff& p : puffs_) {
        for (std::size_t a = 0; a < 3; ++a) {
            const double s = kSplatSigmas * (a == 2 ? p.sigmaV : p.sigmaH);
            lo[a] = std::min(
                lo[a],
                std::max(0, static_cast<int>(std::floor((p.centre[a] - s - origin[a]) / voxel))));
            hi[a] = std::max(
                hi[a],
                std::min(count[a] - 1,
                         static_cast<int>(std::ceil((p.centre[a] + s - origin[a]) / voxel))));
        }
    }
    if (hi[0] < lo[0] || hi[1] < lo[1] || hi[2] < lo[2]) {
        return nullptr;
    }
    fields::DenseBox box;
    box.size = voxel;
    for (std::size_t a = 0; a < 3; ++a) {
        box.origin[a] = origin[a] + lo[a] * voxel;
        box.count[a] = hi[a] - lo[a] + 1;
    }
    const auto nx = static_cast<std::size_t>(box.count[0]);
    const auto ny = static_cast<std::size_t>(box.count[1]);
    const auto nz = static_cast<std::size_t>(box.count[2]);
    std::vector<double> sigma(nx * ny * nz, 0.0);
    const double perMass = kSpecificExtinction / (voxel * voxel * voxel);
    std::vector<double> wx;
    std::vector<double> wy;
    std::vector<double> wz;
    for (const Puff& p : puffs_) {
        int x0 = 0;
        int x1 = 0;
        int y0 = 0;
        int y1 = 0;
        int z0 = 0;
        int z1 = 0;
        weights(p.centre[0], p.sigmaH, box.origin[0], voxel, x0, x1, wx);
        weights(p.centre[1], p.sigmaH, box.origin[1], voxel, y0, y1, wy);
        weights(p.centre[2], p.sigmaV, box.origin[2], voxel, z0, z1, wz, p.top);
        const double scale = p.mass * perMass;
        for (int i = std::max(x0, 0); i <= std::min(x1, box.count[0] - 1); ++i) {
            const double a = scale * wx[static_cast<std::size_t>(i - x0)];
            for (int j = std::max(y0, 0); j <= std::min(y1, box.count[1] - 1); ++j) {
                const double ab = a * wy[static_cast<std::size_t>(j - y0)];
                double* row =
                    &sigma[((static_cast<std::size_t>(i) * ny) + static_cast<std::size_t>(j)) * nz];
                for (int k = std::max(z0, 0); k <= std::min(z1, box.count[2] - 1); ++k) {
                    row[k] += ab * wz[static_cast<std::size_t>(k - z0)];
                }
            }
        }
    }
    std::vector<float> values(sigma.size());
    for (std::size_t n = 0; n < sigma.size(); ++n) {
        values[n] = static_cast<float>(sigma[n]);
    }
    return fields::scalarField("smoke_extinction", box, values);
}

void Smoke::save(std::vector<double>& out) const {
    out.push_back(left_);
    out.push_back(static_cast<double>(puffs_.size()));
    for (const Puff& p : puffs_) {
        out.insert(out.end(), {p.centre[0], p.centre[1], p.centre[2], p.mass, p.sigmaH, p.sigmaV,
                               static_cast<double>(p.fire), p.underCeiling ? 1.0 : 0.0, p.top});
    }
}

bool Smoke::restore(std::span<const double> in) {
    if (in.size() < 2) {
        return false;
    }
    const auto n = static_cast<std::size_t>(in[1]);
    if (in.size() != 2 + (n * kPuffValues)) {
        return false;
    }
    left_ = in[0];
    puffs_.assign(n, Puff{});
    for (std::size_t i = 0; i < n; ++i) {
        const std::span<const double> v = in.subspan(2 + (i * kPuffValues), kPuffValues);
        Puff& p = puffs_[i];
        p.centre = {v[0], v[1], v[2]};
        p.mass = v[3];
        p.sigmaH = v[4];
        p.sigmaV = v[5];
        p.fire = static_cast<std::uint32_t>(v[6]);
        p.underCeiling = v[7] != 0.0;
        p.top = v[8];
    }
    return true;
}

}  // namespace lagrange::fire
