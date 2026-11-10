// Heat conduction in solids (RFC 0012); rules on the class in Conduction.hpp.
#include "lagrange/thermal/Conduction.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace lagrange::thermal {

namespace {

using Vec3 = std::array<double, 3>;

// Conjugate gradients stop at this relative residual, or after kMaxIterations.
constexpr double kTolerance = 1e-12;
constexpr int kMaxIterations = 4000;

// The six face directions of a cell: −x, +x, −y, +y, −z, +z.
constexpr std::array<std::array<int, 3>, 6> kDirections{
    {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}}};

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// A point of the body frame in a shape's frame.
Vec3 inShape(const SolidShape& s, const Vec3& p) {
    return rotate(conjugate(s.pose.orientation), sub(p, s.pose.position));
}

bool inside(const SolidShape& s, const Vec3& p) {
    const Vec3 q = inShape(s, p);
    switch (s.type) {
        case ShapeType::Box:
            return std::abs(q[0]) <= s.size[0] && std::abs(q[1]) <= s.size[1] &&
                   std::abs(q[2]) <= s.size[2];
        case ShapeType::Sphere:
            return dot(q, q) <= s.size[0] * s.size[0];
        case ShapeType::Cylinder:
            return q[0] * q[0] + q[1] * q[1] <= s.size[0] * s.size[0] &&
                   std::abs(q[2]) <= s.size[1];
        case ShapeType::Capsule: {
            const double z = std::clamp(q[2], -s.size[1], s.size[1]);
            const Vec3 d{q[0], q[1], q[2] - z};
            return dot(d, d) <= s.size[0] * s.size[0];
        }
        case ShapeType::ConvexHull:
            if (s.faces.empty()) {
                return false;
            }
            for (const auto& f : s.faces) {
                if (f[0] * q[0] + f[1] * q[1] + f[2] * q[2] > f[3]) {
                    return false;
                }
            }
            return true;
        default:
            return false;
    }
}

/// Half extents of a shape's bounding box in its own frame.
Vec3 halfExtents(const SolidShape& s) {
    switch (s.type) {
        case ShapeType::Box:
            return s.size;
        case ShapeType::Sphere:
            return {s.size[0], s.size[0], s.size[0]};
        case ShapeType::Cylinder:
            return {s.size[0], s.size[0], s.size[1]};
        case ShapeType::Capsule:
            return {s.size[0], s.size[0], s.size[1] + s.size[0]};
        case ShapeType::ConvexHull: {
            Vec3 h{0.0, 0.0, 0.0};
            for (const Vec3& p : s.points) {
                for (int i = 0; i < 3; ++i) {
                    h[i] = std::max(h[i], std::abs(p[i]));
                }
            }
            return h;
        }
        default:
            return {0.0, 0.0, 0.0};
    }
}

/// The thinnest full dimension of a shape.
double thinnest(const SolidShape& s) {
    switch (s.type) {
        case ShapeType::Box:
            return 2.0 * std::min({s.size[0], s.size[1], s.size[2]});
        case ShapeType::Sphere:
        case ShapeType::Capsule:
            return 2.0 * s.size[0];
        case ShapeType::Cylinder:
            return 2.0 * std::min(s.size[0], s.size[1]);
        case ShapeType::ConvexHull: {
            Vec3 lo{1e300, 1e300, 1e300};
            Vec3 hi{-1e300, -1e300, -1e300};
            for (const Vec3& p : s.points) {
                for (int i = 0; i < 3; ++i) {
                    lo[i] = std::min(lo[i], p[i]);
                    hi[i] = std::max(hi[i], p[i]);
                }
            }
            return s.points.empty() ? 0.0 : std::min({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
        }
        default:
            return 0.0;
    }
}

/// Conductivity of a material along a body-frame axis direction (0, 1, 2).
double conductivityAlongAxis(const SolidPart& part, int direction) {
    const ThermalMaterial& m = part.material;
    if (!m.conductivityAlong) {
        return m.conductivity;
    }
    const double a = part.axis[static_cast<std::size_t>(direction)];
    return m.conductivity + (*m.conductivityAlong - m.conductivity) * a * a;
}

/// Natural convection Nusselt numbers (Incropera Ch. 9).
double churchillChu(double ra) {
    const double f = std::pow(1.0 + std::pow(0.492 / kAirPrandtl, 9.0 / 16.0), 8.0 / 27.0);
    const double root = 0.825 + 0.387 * std::pow(ra, 1.0 / 6.0) / f;
    return root * root;
}

double rayleigh(double surface, double air, double length) {
    const double film = 0.5 * (surface + air);
    const double beta = 1.0 / film;  // ideal gas
    return kStandardGravity * beta * std::abs(surface - air) * length * length * length /
           (kAirKinematicViscosity * kAirDiffusivity);
}

}  // namespace

double naturalConvection(double surface, double air, double normalZ, double height,
                         double horizontalLength) {
    double vertical = 0.0;
    if (height > 0.0) {
        vertical = churchillChu(rayleigh(surface, air, height)) * kAirConductivity / height;
    }
    double horizontal = 0.0;
    const double w = std::min(std::abs(normalZ), 1.0);
    if (horizontalLength > 0.0 && w > 0.0) {
        const double ra = rayleigh(surface, air, horizontalLength);
        // Upward face of a hot plate, or downward face of a cold one: the buoyant plume leaves.
        const bool rising = (normalZ > 0.0) == (surface > air);
        const double nu =
            rising ? (ra <= 1e7 ? 0.54 * std::pow(ra, 0.25) : 0.15 * std::pow(ra, 1.0 / 3.0))
                   : 0.52 * std::pow(ra, 0.2);
        horizontal = nu * kAirConductivity / horizontalLength;
    }
    return (1.0 - w) * vertical + w * horizontal;
}

double forcedConvection(double speed, double length) {
    if (speed <= 0.0 || length <= 0.0) {
        return 0.0;
    }
    const double re = speed * length / kAirKinematicViscosity;
    const double pr = std::cbrt(kAirPrandtl);
    const double nu =
        re < 5e5 ? 0.664 * std::sqrt(re) * pr : (0.037 * std::pow(re, 0.8) - 871.0) * pr;
    return nu * kAirConductivity / length;
}

std::optional<ConductionModel> ConductionModel::build(std::vector<SolidPart> parts,
                                                      std::size_t maxCellsPerPart,
                                                      std::vector<std::string>& notes,
                                                      std::string& error) {
    ConductionModel m;
    m.parts_ = std::move(parts);
    m.grids_.resize(m.parts_.size());
    std::vector<Vec3> centres;
    for (std::size_t p = 0; p < m.parts_.size(); ++p) {
        const SolidPart& part = m.parts_[p];
        Vec3 lo{1e300, 1e300, 1e300};
        Vec3 hi{-1e300, -1e300, -1e300};
        double thin = 1e300;
        for (const SolidShape& s : part.shapes) {
            const Vec3 h = halfExtents(s);
            for (int corner = 0; corner < 8; ++corner) {
                const Vec3 local{(corner & 1) != 0 ? h[0] : -h[0], (corner & 2) != 0 ? h[1] : -h[1],
                                 (corner & 4) != 0 ? h[2] : -h[2]};
                const Vec3 b = add(s.pose.position, rotate(s.pose.orientation, local));
                for (int i = 0; i < 3; ++i) {
                    lo[i] = std::min(lo[i], b[i]);
                    hi[i] = std::max(hi[i], b[i]);
                }
            }
            if (const double t = thinnest(s); t > 0.0) {
                thin = std::min(thin, t);
            }
        }
        if (part.shapes.empty() || thin >= 1e300) {
            error = part.name + ": a thermal material on a link with no collision shape";
            return std::nullopt;
        }
        double cell = std::min(part.cell, thin / 2.0);
        const Vec3 extent = sub(hi, lo);
        const auto dimsFor = [&](double c) {
            return std::array<int, 3>{std::max(1, static_cast<int>(std::ceil(extent[0] / c))),
                                      std::max(1, static_cast<int>(std::ceil(extent[1] / c))),
                                      std::max(1, static_cast<int>(std::ceil(extent[2] / c)))};
        };
        std::array<int, 3> dims = dimsFor(cell);
        const auto total = [&] { return static_cast<double>(dims[0]) * dims[1] * dims[2]; };
        if (total() > static_cast<double>(maxCellsPerPart)) {
            cell *= std::cbrt(total() / static_cast<double>(maxCellsPerPart)) * 1.0001;
            dims = dimsFor(cell);
            notes.push_back(part.name + ": cells of " + std::to_string(cell * 1000.0) +
                            " mm, to stay under " + std::to_string(maxCellsPerPart) + " cells");
        }
        Grid& g = m.grids_[p];
        g.cell = cell;
        g.dims = dims;
        const Vec3 middle{0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])};
        for (int i = 0; i < 3; ++i) {
            g.origin[static_cast<std::size_t>(i)] = middle[static_cast<std::size_t>(i)] -
                                                    0.5 * cell * dims[static_cast<std::size_t>(i)];
        }
        g.first = centres.size();
        g.cellAt.assign(static_cast<std::size_t>(dims[0]) * dims[1] * dims[2], -1);
        const double capacity =
            part.material.density * part.material.specificHeat * cell * cell * cell;
        for (int z = 0; z < dims[2]; ++z) {
            for (int y = 0; y < dims[1]; ++y) {
                for (int x = 0; x < dims[0]; ++x) {
                    const Vec3 c{g.origin[0] + (x + 0.5) * cell, g.origin[1] + (y + 0.5) * cell,
                                 g.origin[2] + (z + 0.5) * cell};
                    bool solid = false;
                    for (const SolidShape& s : part.shapes) {
                        solid = solid || inside(s, c);
                    }
                    if (solid) {
                        g.cellAt[(static_cast<std::size_t>(z) * dims[1] + y) * dims[0] + x] =
                            static_cast<std::int32_t>(centres.size() - g.first);
                        centres.push_back(c);
                        m.partOf_.push_back(static_cast<std::uint32_t>(p));
                        m.capacity_.push_back(capacity);
                    }
                }
            }
        }
        g.count = centres.size() - g.first;
        if (g.count == 0) {
            error = part.name + ": its shapes hold no cell of " + std::to_string(cell) + " m";
            return std::nullopt;
        }
    }
    // The cell of part q holding a body-frame point of part p (rest poses), or −1.
    const auto cellOf = [&](std::size_t q, const Vec3& worldPoint) -> std::int64_t {
        const Grid& g = m.grids_[q];
        const Pose& rest = m.parts_[q].rest;
        const Vec3 local = rotate(conjugate(rest.orientation), sub(worldPoint, rest.position));
        std::array<int, 3> i{};
        for (int a = 0; a < 3; ++a) {
            const double f =
                (local[static_cast<std::size_t>(a)] - g.origin[static_cast<std::size_t>(a)]) /
                g.cell;
            if (f < 0.0 || f >= g.dims[static_cast<std::size_t>(a)]) {
                return -1;
            }
            i[static_cast<std::size_t>(a)] = static_cast<int>(f);
        }
        const std::int32_t c =
            g.cellAt[(static_cast<std::size_t>(i[2]) * g.dims[1] + i[1]) * g.dims[0] + i[0]];
        return c < 0 ? -1 : static_cast<std::int64_t>(g.first) + c;
    };
    std::set<std::pair<std::size_t, std::size_t>> untold;
    m.facesFirst_.resize(m.parts_.size() + 1);
    for (std::size_t p = 0; p < m.parts_.size(); ++p) {
        m.facesFirst_[p] = m.faces_.size();
        const SolidPart& part = m.parts_[p];
        const Grid& g = m.grids_[p];
        for (std::size_t local = 0; local < g.count; ++local) {
            const std::size_t cell = g.first + local;
            const Vec3& c = centres[cell];
            const auto cellX = static_cast<int>((c[0] - g.origin[0]) / g.cell);
            const auto cellY = static_cast<int>((c[1] - g.origin[1]) / g.cell);
            const auto cellZ = static_cast<int>((c[2] - g.origin[2]) / g.cell);
            for (std::uint8_t d = 0; d < 6; ++d) {
                const auto& step = kDirections[d];
                const int nx = cellX + step[0];
                const int ny = cellY + step[1];
                const int nz = cellZ + step[2];
                const int axis = d / 2;
                const double k = conductivityAlongAxis(part, axis);
                const bool inGrid = nx >= 0 && ny >= 0 && nz >= 0 && nx < g.dims[0] &&
                                    ny < g.dims[1] && nz < g.dims[2];
                const std::int32_t neighbour =
                    inGrid
                        ? g.cellAt[(static_cast<std::size_t>(nz) * g.dims[1] + ny) * g.dims[0] + nx]
                        : -1;
                if (neighbour >= 0) {
                    if ((d & 1) != 0) {  // each pair once, from its lower cell
                        m.links_.push_back({static_cast<std::uint32_t>(cell),
                                            static_cast<std::uint32_t>(g.first + neighbour),
                                            k * g.cell});
                    }
                    continue;
                }
                // An outside face: does another part's solid touch it?
                const Vec3 out{c[0] + step[0] * g.cell, c[1] + step[1] * g.cell,
                               c[2] + step[2] * g.cell};
                const Vec3 world = add(part.rest.position, rotate(part.rest.orientation, out));
                bool covered = false;
                for (std::size_t q = 0; q < m.parts_.size() && !covered; ++q) {
                    if (q == p) {
                        continue;
                    }
                    const std::int64_t other = cellOf(q, world);
                    if (other < 0) {
                        continue;
                    }
                    covered = true;
                    const SolidPart& qp = m.parts_[q];
                    const bool childOfQ = part.parent == q && part.contactConductance;
                    const bool parentOfQ = qp.parent == p && qp.contactConductance;
                    if (childOfQ) {  // made once, from the child's side
                        const double area = g.cell * g.cell;
                        const double kq = conductivityAlongAxis(qp, axis);
                        const double r = 0.5 * g.cell / (k * area) +
                                         1.0 / (*part.contactConductance * area) +
                                         0.5 * m.grids_[q].cell / (kq * area);
                        m.links_.push_back({static_cast<std::uint32_t>(cell),
                                            static_cast<std::uint32_t>(other), 1.0 / r});
                    } else if (!parentOfQ) {
                        untold.insert({std::min(p, q), std::max(p, q)});
                    }
                }
                if (!covered) {
                    m.faces_.push_back({static_cast<std::uint32_t>(cell), d, g.cell * g.cell});
                }
            }
        }
    }
    m.facesFirst_[m.parts_.size()] = m.faces_.size();
    for (const auto& [a, b] : untold) {
        notes.push_back(m.parts_[a].name + " and " + m.parts_[b].name +
                        " touch but exchange no heat: give the child link "
                        "thermal:material:contactConductance");
    }
    // Heat sources: the cells whose centres are inside each box, else the nearest cell.
    m.sourceCells_.resize(m.parts_.size());
    for (std::size_t p = 0; p < m.parts_.size(); ++p) {
        const Grid& g = m.grids_[p];
        for (const HeatBox& box : m.parts_[p].sources) {
            std::vector<std::size_t> cells;
            std::size_t nearest = g.first;
            double best = std::numeric_limits<double>::infinity();
            for (std::size_t cell = g.first; cell < g.first + g.count; ++cell) {
                const Vec3 d = sub(centres[cell], box.centre);
                if (std::abs(d[0]) <= 0.5 * box.size[0] && std::abs(d[1]) <= 0.5 * box.size[1] &&
                    std::abs(d[2]) <= 0.5 * box.size[2]) {
                    cells.push_back(cell);
                }
                if (const double r = dot(d, d); r < best) {
                    best = r;
                    nearest = cell;
                }
            }
            if (cells.empty()) {
                cells.push_back(nearest);
            }
            m.sourceCells_[p].push_back(std::move(cells));
        }
    }
    const std::size_t n = m.capacity_.size();
    m.diagonal_.assign(n, 0.0);
    m.rhs_.assign(n, 0.0);
    m.residual_.assign(n, 0.0);
    m.direction_.assign(n, 0.0);
    m.product_.assign(n, 0.0);
    m.preconditioned_.assign(n, 0.0);
    m.centres_ = std::move(centres);
    m.faceCoefficient_.assign(m.faces_.size(), 0.0);
    m.faceSky_.assign(m.faces_.size(), 0.0);
    m.heatOut_.assign(m.parts_.size(), 0.0);
    return m;
}

std::array<double, 3> ConductionModel::centre(std::size_t cell) const {
    return centres_.at(cell);
}

std::span<const std::size_t> ConductionModel::sourceCells(std::size_t part,
                                                          std::size_t source) const {
    return sourceCells_.at(part).at(source);
}

std::vector<double> ConductionModel::initialTemperatures() const {
    std::vector<double> t(capacity_.size());
    for (std::size_t i = 0; i < t.size(); ++i) {
        t[i] = parts_[partOf_[i]].initialTemperature;
    }
    return t;
}

void ConductionModel::step(std::span<double> t, std::span<const PartInput> inputs,
                           std::span<const double> sourceHeat, const Surroundings& around,
                           double dt, std::optional<double> fixedCoefficient) {
    const std::size_t n = capacity_.size();
    // Capacity and the cells' own heat.
    for (std::size_t i = 0; i < n; ++i) {
        diagonal_[i] = capacity_[i] / dt;
        rhs_[i] = capacity_[i] / dt * t[i];
    }
    std::size_t source = 0;
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        const Grid& g = grids_[p];
        const double even = inputs[p].heat / static_cast<double>(g.count);
        for (std::size_t c = g.first; c < g.first + g.count; ++c) {
            rhs_[c] += even;
        }
        for (const std::vector<std::size_t>& cells : sourceCells_[p]) {
            const double share = sourceHeat[source++] / static_cast<double>(cells.size());
            for (const std::size_t c : cells) {
                rhs_[c] += share;
            }
        }
    }
    for (const Link& l : links_) {
        diagonal_[l.a] += l.conductance;
        diagonal_[l.b] += l.conductance;
    }
    // Faces: convection from the correlations on the part's mean surface temperature and its
    // size as it sits now, radiation linearised around each face's cell temperature.
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        const SolidPart& part = parts_[p];
        const Grid& g = grids_[p];
        const Pose& pose = inputs[p].pose;
        // The part's extent along the world axes, from its grid's box.
        Vec3 worldExtent{0.0, 0.0, 0.0};
        for (int a = 0; a < 3; ++a) {
            Vec3 axis{0.0, 0.0, 0.0};
            axis[static_cast<std::size_t>(a)] = g.cell * g.dims[static_cast<std::size_t>(a)];
            const Vec3 w = rotate(pose.orientation, axis);
            for (int i = 0; i < 3; ++i) {
                worldExtent[static_cast<std::size_t>(i)] +=
                    std::abs(w[static_cast<std::size_t>(i)]);
            }
        }
        const double height = worldExtent[2];
        const double footprint = worldExtent[0] * worldExtent[1];
        const double horizontal =
            footprint > 0.0 ? footprint / (2.0 * (worldExtent[0] + worldExtent[1])) : 0.0;
        const double longest = std::max({worldExtent[0], worldExtent[1], worldExtent[2]});
        double area = 0.0;
        double weighted = 0.0;
        for (std::size_t f = facesFirst_[p]; f < facesFirst_[p + 1]; ++f) {
            area += faces_[f].area;
            weighted += faces_[f].area * t[faces_[f].cell];
        }
        const double surface = area > 0.0 ? weighted / area : around.air;
        const double forced = forcedConvection(inputs[p].airspeed, longest);
        const double eps = part.material.emissivity;
        for (std::size_t f = facesFirst_[p]; f < facesFirst_[p + 1]; ++f) {
            const Face& face = faces_[f];
            const auto& step = kDirections[face.direction];
            const Vec3 normal = rotate(
                pose.orientation, Vec3{static_cast<double>(step[0]), static_cast<double>(step[1]),
                                       static_cast<double>(step[2])});
            double h = 0.0;
            if (fixedCoefficient) {
                h = *fixedCoefficient;
            } else {
                const double natural =
                    naturalConvection(surface, around.air, normal[2], height, horizontal);
                // Mixed convection: Churchill and Usagi's blend with n = 3 (Incropera Eq. 9.65).
                h = std::cbrt(natural * natural * natural + forced * forced * forced);
            }
            const double tc = t[face.cell];
            const double hr =
                eps * kStefanBoltzmann * (tc * tc + around.sky * around.sky) * (tc + around.sky);
            faceCoefficient_[f] = h * face.area;
            faceSky_[f] = hr * face.area;
            diagonal_[face.cell] += faceCoefficient_[f] + faceSky_[f];
            rhs_[face.cell] += faceCoefficient_[f] * around.air + faceSky_[f] * around.sky;
            // The sun, overhead, on the faces looking up: a grey body absorbs as it emits.
            if (around.irradiance > 0.0 && normal[2] > 0.0) {
                rhs_[face.cell] += eps * around.irradiance * normal[2] * face.area;
            }
        }
    }
    // Jacobi-preconditioned conjugate gradients from the last temperatures.
    const auto multiply = [&](std::span<const double> x, std::span<double> y) {
        for (std::size_t i = 0; i < n; ++i) {
            y[i] = diagonal_[i] * x[i];
        }
        for (const Link& l : links_) {
            y[l.a] -= l.conductance * x[l.b];
            y[l.b] -= l.conductance * x[l.a];
        }
    };
    multiply(t, product_);
    double bNorm = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        residual_[i] = rhs_[i] - product_[i];
        bNorm += rhs_[i] * rhs_[i];
    }
    const double stop = kTolerance * kTolerance * bNorm;
    double rz = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        preconditioned_[i] = residual_[i] / diagonal_[i];
        direction_[i] = preconditioned_[i];
        rz += residual_[i] * preconditioned_[i];
    }
    iterations_ = 0;
    for (; iterations_ < kMaxIterations; ++iterations_) {
        double rr = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            rr += residual_[i] * residual_[i];
        }
        if (rr <= stop) {
            break;
        }
        multiply(direction_, product_);
        double pAp = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            pAp += direction_[i] * product_[i];
        }
        if (pAp <= 0.0) {
            break;
        }
        const double alpha = rz / pAp;
        double next = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            t[i] += alpha * direction_[i];
            residual_[i] -= alpha * product_[i];
            preconditioned_[i] = residual_[i] / diagonal_[i];
            next += residual_[i] * preconditioned_[i];
        }
        const double beta = next / rz;
        rz = next;
        for (std::size_t i = 0; i < n; ++i) {
            direction_[i] = preconditioned_[i] + beta * direction_[i];
        }
    }
    for (std::size_t p = 0; p < parts_.size(); ++p) {
        double out = 0.0;
        for (std::size_t f = facesFirst_[p]; f < facesFirst_[p + 1]; ++f) {
            const double tc = t[faces_[f].cell];
            out += faceCoefficient_[f] * (tc - around.air) + faceSky_[f] * (tc - around.sky);
        }
        heatOut_[p] = out;
    }
}

double ConductionModel::mean(std::span<const double> t, std::size_t part) const {
    const Grid& g = grids_.at(part);
    double sum = 0.0;
    for (std::size_t c = g.first; c < g.first + g.count; ++c) {
        sum += t[c];
    }
    return sum / static_cast<double>(g.count);
}

double ConductionModel::maximum(std::span<const double> t, std::size_t part) const {
    const Grid& g = grids_.at(part);
    return *std::max_element(t.begin() + static_cast<std::ptrdiff_t>(g.first),
                             t.begin() + static_cast<std::ptrdiff_t>(g.first + g.count));
}

double ConductionModel::minimum(std::span<const double> t, std::size_t part) const {
    const Grid& g = grids_.at(part);
    return *std::min_element(t.begin() + static_cast<std::ptrdiff_t>(g.first),
                             t.begin() + static_cast<std::ptrdiff_t>(g.first + g.count));
}

double ConductionModel::sourceMean(std::span<const double> t, std::size_t part,
                                   std::size_t source) const {
    const auto cells = sourceCells(part, source);
    double sum = 0.0;
    for (const std::size_t c : cells) {
        sum += t[c];
    }
    return sum / static_cast<double>(cells.size());
}

}  // namespace lagrange::thermal
