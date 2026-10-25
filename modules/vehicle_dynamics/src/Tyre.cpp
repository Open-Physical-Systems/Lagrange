// Tyre force models (Tyre.hpp).
#include "lagrange/vehicle_dynamics/Tyre.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>

namespace lagrange::vehicle_dynamics {

namespace {

double sign(double x) {
    return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0);
}

/// The weighting function cos(C·atan(B·x − E·(B·x − atan(B·x)))), normalised to 1 at x = 0.
double weight(double x, double b, double c, double e) {
    const double bx = b * x;
    return std::cos(c * std::atan(bx - e * (bx - std::atan(bx))));
}

}  // namespace

double magicFormula(double x, double b, double c, double d, double e) {
    const double bx = b * x;
    return d * std::sin(c * std::atan(bx - e * (bx - std::atan(bx))));
}

Expected<MagicFormula, std::string> MagicFormula::fromValues(
    const std::map<std::string, double>& values) {
    MagicFormula mf;
    std::string missing;
    const auto take = [&](const char* name, double& out, bool required) {
        const auto it = values.find(name);
        if (it != values.end()) {
            out = it->second;
        } else if (required && missing.empty()) {
            missing = name;
        }
    };
    take("FNOMIN", mf.fz0, true);
    take("UNLOADED_RADIUS", mf.unloadedRadius, false);
    take("PCX1", mf.pcx1, true);
    take("PDX1", mf.pdx1, true);
    take("PDX2", mf.pdx2, false);
    take("PEX1", mf.pex1, false);
    take("PEX2", mf.pex2, false);
    take("PEX3", mf.pex3, false);
    take("PEX4", mf.pex4, false);
    take("PKX1", mf.pkx1, true);
    take("PKX2", mf.pkx2, false);
    take("PKX3", mf.pkx3, false);
    take("PCY1", mf.pcy1, true);
    take("PDY1", mf.pdy1, true);
    take("PDY2", mf.pdy2, false);
    take("PEY1", mf.pey1, false);
    take("PEY2", mf.pey2, false);
    take("PEY3", mf.pey3, false);
    take("PKY1", mf.pky1, true);
    take("PKY2", mf.pky2, true);
    take("RBX1", mf.rbx1, false);
    take("RBX2", mf.rbx2, false);
    take("RCX1", mf.rcx1, false);
    take("REX1", mf.rex1, false);
    take("REX2", mf.rex2, false);
    take("RBY1", mf.rby1, false);
    take("RBY2", mf.rby2, false);
    take("RBY3", mf.rby3, false);
    take("RCY1", mf.rcy1, false);
    take("REY1", mf.rey1, false);
    take("REY2", mf.rey2, false);
    if (!missing.empty()) {
        return Unexpected{"the Magic Formula needs " + missing};
    }
    if (mf.fz0 <= 0.0 || mf.pcx1 <= 0.0 || mf.pcy1 <= 0.0 || mf.pkx1 == 0.0 || mf.pky1 == 0.0 ||
        mf.pky2 == 0.0) {
        return Unexpected{
            std::string("FNOMIN, PCX1 and PCY1 must be positive, PKX1, PKY1 and "
                        "PKY2 not zero")};
    }
    return mf;
}

Expected<MagicFormula, std::string> MagicFormula::fromTir(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        return Unexpected{path + ": cannot be read"};
    }
    std::map<std::string, double> values;
    std::string line;
    while (std::getline(file, line)) {
        // Comments start with $ or !; section headers are [NAME].
        for (const char c : {'$', '!'}) {
            if (const std::size_t at = line.find(c); at != std::string::npos) {
                line.erase(at);
            }
        }
        const std::size_t equals = line.find('=');
        if (equals == std::string::npos || line.find('[') != std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, equals);
        std::string value = line.substr(equals + 1);
        const auto trim = [](std::string& s) {
            s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                            [](unsigned char c) { return std::isspace(c) == 0; }));
            s.erase(std::find_if(s.rbegin(), s.rend(),
                                 [](unsigned char c) { return std::isspace(c) == 0; })
                        .base(),
                    s.end());
        };
        trim(key);
        trim(value);
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        try {
            std::size_t used = 0;
            const double number = std::stod(value, &used);
            if (used == value.size()) {
                values[key] = number;
            }
        } catch (const std::exception&) {
            // Text values ('FILE_TYPE', units): not coefficients.
        }
    }
    auto mf = fromValues(values);
    if (!mf) {
        return Unexpected{path + ": " + mf.error()};
    }
    return mf;
}

TyreForce pacejka(const MagicFormula& mf, double kappa, double alpha, double fz, double muScale) {
    TyreForce out;
    if (fz <= 0.0) {
        return out;
    }
    const double dfz = (fz - mf.fz0) / mf.fz0;
    const double k = std::abs(kappa);
    const double a = std::abs(alpha);
    // Pure longitudinal slip.
    const double cx = mf.pcx1;
    const double dx = (mf.pdx1 + mf.pdx2 * dfz) * muScale * fz;
    const double ex =
        std::min(1.0, (mf.pex1 + mf.pex2 * dfz + mf.pex3 * dfz * dfz) * (1.0 - mf.pex4));
    const double kx = fz * (mf.pkx1 + mf.pkx2 * dfz) * std::exp(mf.pkx3 * dfz);
    const double bx = dx != 0.0 ? kx / (cx * dx) : 0.0;
    const double fx0 = std::abs(magicFormula(k, bx, cx, dx, ex));
    // Pure lateral slip.
    const double cy = mf.pcy1;
    const double dy = (mf.pdy1 + mf.pdy2 * dfz) * muScale * fz;
    const double ey = std::min(1.0, (mf.pey1 + mf.pey2 * dfz) * (1.0 - mf.pey3));
    const double ky =
        std::abs(mf.pky1 * mf.fz0 * std::sin(2.0 * std::atan(fz / (mf.pky2 * mf.fz0))));
    const double by = dy != 0.0 ? ky / (cy * dy) : 0.0;
    const double fy0 = std::abs(magicFormula(a, by, cy, dy, ey));
    // Combined slip: each force weighted down by the other slip.
    const double bxa = mf.rbx1 * std::cos(std::atan(mf.rbx2 * k));
    const double exa = std::min(1.0, mf.rex1 + mf.rex2 * dfz);
    const double gxa = mf.rbx1 != 0.0 ? std::max(0.0, weight(a, bxa, mf.rcx1, exa)) : 1.0;
    const double byk = mf.rby1 * std::cos(std::atan(mf.rby2 * (a - mf.rby3)));
    const double eyk = std::min(1.0, mf.rey1 + mf.rey2 * dfz);
    const double gyk = mf.rby1 != 0.0 ? std::max(0.0, weight(k, byk, mf.rcy1, eyk)) : 1.0;
    out.fx = sign(kappa) * gxa * fx0;
    out.fy = -sign(alpha) * gyk * fy0;
    return out;
}

TyreForce linearTyre(double longitudinalStiffness, double corneringStiffness, double mu,
                     double kappa, double alpha, double fz) {
    TyreForce out;
    if (fz <= 0.0) {
        return out;
    }
    out.fx = longitudinalStiffness * kappa;
    out.fy = -corneringStiffness * alpha;
    const double limit = mu * fz;
    const double total = std::hypot(out.fx, out.fy);
    if (total > limit) {
        out.fx *= limit / total;
        out.fy *= limit / total;
    }
    return out;
}

}  // namespace lagrange::vehicle_dynamics
