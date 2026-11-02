// See WindTunnel.hpp.
#include "lagrange/aerodynamics/WindTunnel.hpp"
#include "lagrange/fields/Field.hpp"
#include "lagrange/world/Model.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <sstream>

namespace lagrange::aerodynamics {

namespace {

using Vec3 = std::array<double, 3>;

// The tunnel around the body, in lengths of its largest size (a choice, from common practice for
// external flow: more behind than in front, to let the wake leave).
constexpr double kAhead = 1.0;
constexpr double kBehind = 3.0;
constexpr double kSideways = 1.2;
constexpr int kColours = 32;     // bands of each colour map: one mesh per band
constexpr int kSeedsAcross = 9;  // streamlines per side of the seeding grid
constexpr int kTubeSides = 6;

Vec3 minus(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 plus(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 times(const Vec3& a, double k) {
    return {a[0] * k, a[1] * k, a[2] * k};
}

double dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double length(const Vec3& a) {
    return std::sqrt(dot(a, a));
}

/// One collider where it stands, with its test for a point.
struct Solid {
    std::size_t body{0};
    ShapeType shape{ShapeType::Box};
    std::array<double, 3> size{};
    Pose pose;
    const ConvexHull* hull{nullptr};
    Vec3 low{}, high{};
};

bool insideLocal(const Solid& s, const Vec3& p) {
    switch (s.shape) {
        case ShapeType::Box:
            return std::abs(p[0]) <= s.size[0] && std::abs(p[1]) <= s.size[1] &&
                   std::abs(p[2]) <= s.size[2];
        case ShapeType::Sphere:
            return dot(p, p) <= s.size[0] * s.size[0];
        case ShapeType::Cylinder:
            return p[0] * p[0] + p[1] * p[1] <= s.size[0] * s.size[0] &&
                   std::abs(p[2]) <= s.size[1];
        case ShapeType::Capsule: {
            const double along = std::clamp(p[2], -s.size[1], s.size[1]);
            const Vec3 q{p[0], p[1], p[2] - along};
            return dot(q, q) <= s.size[0] * s.size[0];
        }
        case ShapeType::ConvexHull:
            return std::ranges::all_of(s.hull->faces, [&p](const std::array<double, 4>& f) {
                return f[0] * p[0] + f[1] * p[1] + f[2] * p[2] <= f[3];
            });
        default:
            return false;
    }
}

/// The surface of a shape, in its own frame, as triangles (each three points).
std::vector<std::array<Vec3, 3>> surfaceOf(const Solid& s) {
    std::vector<std::array<Vec3, 3>> out;
    const auto fromMesh = [&out](const TriMesh& mesh) {
        const auto at = [&mesh](std::uint32_t i) {
            return Vec3{mesh.positions[3 * i], mesh.positions[3 * i + 1],
                        mesh.positions[3 * i + 2]};
        };
        for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            out.push_back({at(mesh.indices[t]), at(mesh.indices[t + 1]), at(mesh.indices[t + 2])});
        }
    };
    switch (s.shape) {
        case ShapeType::Box:
            fromMesh(boxMesh(2.0 * s.size[0], 2.0 * s.size[1], 2.0 * s.size[2]));
            break;
        case ShapeType::Sphere:
            fromMesh(sphereMesh(s.size[0], 24, 12));
            break;
        case ShapeType::Cylinder:
            fromMesh(cylinderMesh(s.size[0], 2.0 * s.size[1], 24));
            break;
        case ShapeType::Capsule:
            fromMesh(cylinderMesh(s.size[0], 2.0 * s.size[1], 24));
            break;
        case ShapeType::ConvexHull:
            for (const auto& t : s.hull->triangles) {
                out.push_back({s.hull->points[t[0]], s.hull->points[t[1]], s.hull->points[t[2]]});
            }
            break;
        default:
            break;
    }
    return out;
}

/// Bounds of a shape in the tunnel's frame, from sampling its surface.
void boundsOf(Solid& s) {
    s.low = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    s.high = {-s.low[0], -s.low[1], -s.low[2]};
    const double pad = s.shape == ShapeType::Capsule ? s.size[0] : 0.0;
    for (const auto& triangle : surfaceOf(s)) {
        for (const Vec3& corner : triangle) {
            const Vec3 at = plus(rotate(s.pose.orientation, corner), s.pose.position);
            for (std::size_t k = 0; k < 3; ++k) {
                s.low.at(k) = std::min(s.low.at(k), at.at(k) - pad);
                s.high.at(k) = std::max(s.high.at(k), at.at(k) + pad);
            }
        }
    }
}

/// viridis (the design tokens' field colour map) at t in 0..1, as sRGB.
Vec3 viridis(double t) {
    static constexpr std::array<Vec3, 5> kStops{{{0.267, 0.005, 0.329},
                                                 {0.229, 0.322, 0.546},
                                                 {0.128, 0.567, 0.551},
                                                 {0.369, 0.789, 0.383},
                                                 {0.993, 0.906, 0.144}}};
    const double at = std::clamp(t, 0.0, 1.0) * (kStops.size() - 1);
    const auto i = std::min(static_cast<std::size_t>(at), kStops.size() - 2);
    const double f = at - static_cast<double>(i);
    return plus(times(kStops.at(i), 1.0 - f), times(kStops.at(i + 1), f));
}

/// Blue through white to red, for a coefficient of pressure from -1 to +1.
Vec3 diverging(double t) {
    const double x = std::clamp(t, 0.0, 1.0);
    return x < 0.5 ? Vec3{2.0 * x, 2.0 * x * 0.9 + 0.1, 1.0}
                   : Vec3{1.0, 2.0 * (1.0 - x) * 0.9 + 0.1, 2.0 * (1.0 - x)};
}

/// Triangles in bands of one colour: a mesh prim each, which is what a viewer draws plainly.
class Bands {
public:
    void add(int band, const Vec3& a, const Vec3& b, const Vec3& c) {
        std::vector<Vec3>& out =
            bands_.at(static_cast<std::size_t>(std::clamp(band, 0, kColours - 1)));
        out.push_back(a);
        out.push_back(b);
        out.push_back(c);
    }
    void quad(int band, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        add(band, a, b, c);
        add(band, a, c, d);
    }
    /// As USD: an Xform of meshes named after what they span.
    void write(std::ostream& out, const std::string& group,
               const std::function<Vec3(double)>& colour, double low, double high,
               const char* unit) const {
        out << "    def Xform \"" << group << "\"\n    {\n";
        for (int b = 0; b < kColours; ++b) {
            const std::vector<Vec3>& points = bands_.at(static_cast<std::size_t>(b));
            if (points.empty()) {
                continue;
            }
            const double from = low + (high - low) * b / kColours;
            const double to = low + (high - low) * (b + 1) / kColours;
            const Vec3 c = colour((b + 0.5) / kColours);
            char name[96];
            std::snprintf(name, sizeof(name), "b%02d_%+.2f_to_%+.2f_%s", b, from, to, unit);
            std::string clean = name;
            std::ranges::replace(clean, '.', 'p');
            std::ranges::replace(clean, '+', 'P');
            std::ranges::replace(clean, '-', 'M');
            out << "        def Mesh \"" << clean << "\"\n        {\n"
                << "            uniform bool doubleSided = 1\n"
                << "            color3f[] primvars:displayColor = [(" << c[0] << ", " << c[1]
                << ", " << c[2] << ")]\n            int[] faceVertexCounts = [";
            const std::size_t triangles = points.size() / 3;
            for (std::size_t t = 0; t < triangles; ++t) {
                out << (t == 0 ? "3" : ",3");
            }
            out << "]\n            int[] faceVertexIndices = [";
            for (std::size_t i = 0; i < points.size(); ++i) {
                out << (i == 0 ? "" : ",") << i;
            }
            out << "]\n            point3f[] points = [";
            for (std::size_t i = 0; i < points.size(); ++i) {
                out << (i == 0 ? "(" : ",(") << static_cast<float>(points[i][0]) << ","
                    << static_cast<float>(points[i][1]) << "," << static_cast<float>(points[i][2])
                    << ")";
            }
            out << "]\n        }\n";
        }
        out << "    }\n";
    }

private:
    std::array<std::vector<Vec3>, kColours> bands_;
};

void appendBigEndian(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

/// An RGB image as a PNG, its data stored rather than compressed: no library, and frames are read
/// once by an encoder.
void writePng(const std::filesystem::path& file, std::size_t width, std::size_t height,
              const std::vector<std::uint8_t>& rgb) {
    std::vector<std::uint8_t> raw;
    raw.reserve((width * 3 + 1) * height);
    for (std::size_t y = 0; y < height; ++y) {
        raw.push_back(0);  // no filter
        raw.insert(raw.end(), rgb.begin() + static_cast<std::ptrdiff_t>(y * width * 3),
                   rgb.begin() + static_cast<std::ptrdiff_t>((y + 1) * width * 3));
    }
    std::vector<std::uint8_t> z{0x78, 0x01};
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    for (const std::uint8_t byte : raw) {
        a = (a + byte) % 65521U;
        b = (b + a) % 65521U;
    }
    constexpr std::size_t kBlock = 65535;
    for (std::size_t at = 0; at < raw.size(); at += kBlock) {
        const std::size_t n = std::min(kBlock, raw.size() - at);
        z.push_back(at + n == raw.size() ? 1 : 0);
        z.push_back(static_cast<std::uint8_t>(n));
        z.push_back(static_cast<std::uint8_t>(n >> 8));
        z.push_back(static_cast<std::uint8_t>(~n));
        z.push_back(static_cast<std::uint8_t>((~n) >> 8));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(at),
                 raw.begin() + static_cast<std::ptrdiff_t>(at + n));
    }
    appendBigEndian(z, (b << 16) | a);
    std::ofstream out(file, std::ios::binary);
    out.write("\x89PNG\r\n\x1a\n", 8);
    const auto chunk = [&out](const char* type, const std::vector<std::uint8_t>& body) {
        std::vector<std::uint8_t> head;
        appendBigEndian(head, static_cast<std::uint32_t>(body.size()));
        out.write(reinterpret_cast<const char*>(head.data()), 4);
        std::vector<std::uint8_t> checked(type, type + 4);
        checked.insert(checked.end(), body.begin(), body.end());
        out.write(reinterpret_cast<const char*>(checked.data()),
                  static_cast<std::streamsize>(checked.size()));
        std::vector<std::uint8_t> tail;
        appendBigEndian(tail, crc32(checked.data(), checked.size()));
        out.write(reinterpret_cast<const char*>(tail.data()), 4);
    };
    std::vector<std::uint8_t> header;
    appendBigEndian(header, static_cast<std::uint32_t>(width));
    appendBigEndian(header, static_cast<std::uint32_t>(height));
    header.insert(header.end(), {8, 2, 0, 0, 0});  // 8 bits, RGB
    chunk("IHDR", header);
    chunk("IDAT", z);
    chunk("IEND", {});
}

/// The air's spin about y in the slice at `y`: blue one way, red the other, the solid dark.
void writeSpinFrame(const std::filesystem::path& file, const LbmTunnel& tunnel, std::size_t y,
                    double speed, double size) {
    const LbmConfig& c = tunnel.config();
    constexpr std::size_t kScale = 3;
    const std::size_t width = c.nx * kScale;
    const std::size_t height = c.nz * kScale;
    std::vector<std::uint8_t> rgb(width * height * 3);
    const double scale = 0.25 * size / speed;  // a spin of 4 U / L fills the colour map
    for (std::size_t z = 0; z < c.nz; ++z) {
        for (std::size_t x = 0; x < c.nx; ++x) {
            Vec3 colour{0.08, 0.08, 0.1};
            const std::size_t cell = tunnel.index(x, y, z);
            if (!tunnel.isSolid(cell) && x > 0 && x + 1 < c.nx && z > 0 && z + 1 < c.nz) {
                const double dudz = (tunnel.velocity(tunnel.index(x, y, z + 1))[0] -
                                     tunnel.velocity(tunnel.index(x, y, z - 1))[0]);
                const double dwdx = (tunnel.velocity(tunnel.index(x + 1, y, z))[2] -
                                     tunnel.velocity(tunnel.index(x - 1, y, z))[2]);
                const double spin = (dudz - dwdx) / (2.0 * c.cellSize);
                colour = diverging(0.5 + 0.5 * spin * scale);
            }
            for (std::size_t py = 0; py < kScale; ++py) {
                for (std::size_t px = 0; px < kScale; ++px) {
                    const std::size_t at =
                        3 * ((height - 1 - (z * kScale + py)) * width + x * kScale + px);
                    for (std::size_t k = 0; k < 3; ++k) {
                        rgb[at + k] = static_cast<std::uint8_t>(colour.at(k) * 255.0);
                    }
                }
            }
        }
    }
    writePng(file, width, height, rgb);
}

/// ParaView's ImageData, the mean flow, with raw binary arrays appended.
void writeVti(const std::filesystem::path& file, const LbmTunnel& tunnel, const Vec3& origin) {
    const LbmConfig& c = tunnel.config();
    const std::size_t n = tunnel.cells();
    std::ofstream out(file, std::ios::binary);
    const std::uint64_t velocityBytes = 3 * n * sizeof(float);
    const std::uint64_t scalarBytes = n * sizeof(float);
    out << "<?xml version=\"1.0\"?>\n<VTKFile type=\"ImageData\" version=\"1.0\" "
           "byte_order=\"LittleEndian\" header_type=\"UInt64\">\n"
        << "<ImageData WholeExtent=\"0 " << c.nx - 1 << " 0 " << c.ny - 1 << " 0 " << c.nz - 1
        << "\" Origin=\"" << origin[0] + c.cellSize / 2 << " " << origin[1] + c.cellSize / 2 << " "
        << origin[2] + c.cellSize / 2 << "\" Spacing=\"" << c.cellSize << " " << c.cellSize << " "
        << c.cellSize << "\">\n<Piece Extent=\"0 " << c.nx - 1 << " 0 " << c.ny - 1 << " 0 "
        << c.nz - 1 << "\">\n<PointData Vectors=\"velocity\" Scalars=\"pressure\">\n"
        << "<DataArray type=\"Float32\" Name=\"velocity\" NumberOfComponents=\"3\" "
           "format=\"appended\" offset=\"0\"/>\n"
        << "<DataArray type=\"Float32\" Name=\"pressure\" format=\"appended\" offset=\""
        << velocityBytes + 8 << "\"/>\n"
        << "<DataArray type=\"Float32\" Name=\"solid\" format=\"appended\" offset=\""
        << velocityBytes + 8 + scalarBytes + 8
        << "\"/>\n</PointData>\n</Piece>\n</ImageData>\n<AppendedData encoding=\"raw\">\n_";
    out.write(reinterpret_cast<const char*>(&velocityBytes), 8);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 u = tunnel.meanVelocity(i);
        const std::array<float, 3> f{static_cast<float>(u[0]), static_cast<float>(u[1]),
                                     static_cast<float>(u[2])};
        out.write(reinterpret_cast<const char*>(f.data()), sizeof(f));
    }
    out.write(reinterpret_cast<const char*>(&scalarBytes), 8);
    for (std::size_t i = 0; i < n; ++i) {
        const auto p = static_cast<float>(tunnel.meanPressure(i));
        out.write(reinterpret_cast<const char*>(&p), sizeof(p));
    }
    out.write(reinterpret_cast<const char*>(&scalarBytes), 8);
    for (std::size_t i = 0; i < n; ++i) {
        const float solid = tunnel.isSolid(i) ? 1.0F : 0.0F;
        out.write(reinterpret_cast<const char*>(&solid), sizeof(solid));
    }
    out << "\n</AppendedData>\n</VTKFile>\n";
}

}  // namespace

Expected<WindTunnelResult, std::string> runWindTunnel(const World& world,
                                                      const WindTunnelOptions& options,
                                                      const std::string& outputDirectory,
                                                      const std::function<void(double)>& progress) {
    if (options.speed <= 0.0) {
        return Unexpected{std::string("the wind speed must be above zero")};
    }
    // Every collider where it stands at rest, turned by the yaw about z.
    const double half = options.yawDegrees * std::numbers::pi / 360.0;
    const Pose yaw{{0.0, 0.0, 0.0}, {std::cos(half), 0.0, 0.0, std::sin(half)}};
    const std::vector<Pose> poses = restPoses(world);
    std::vector<Solid> solids;
    const World::Colliders& colliders = world.colliders;
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        if (colliders.body[c] == kWorldBody || colliders.shape[c] == ShapeType::Plane ||
            colliders.shape[c] == ShapeType::HeightField) {
            continue;
        }
        Solid s;
        s.body = colliders.body[c].index;
        s.shape = colliders.shape[c];
        s.size = colliders.size[c];
        s.pose = compose(yaw, compose(poses[s.body], colliders.poseInBody[c]));
        s.hull = s.shape == ShapeType::ConvexHull ? &world.hulls.at(colliders.hull[c]) : nullptr;
        boundsOf(s);
        solids.push_back(s);
    }
    if (solids.empty()) {
        return Unexpected{std::string("the robot has no collision shapes for the air to meet")};
    }
    Vec3 low = solids[0].low;
    Vec3 high = solids[0].high;
    for (const Solid& s : solids) {
        for (std::size_t k = 0; k < 3; ++k) {
            low.at(k) = std::min(low.at(k), s.low.at(k));
            high.at(k) = std::max(high.at(k), s.high.at(k));
        }
    }
    const Vec3 span = minus(high, low);
    const double size = std::max({span[0], span[1], span[2]});
    // With a road, the bottom layer of cells is the road, its surface a cell above the box's floor
    // (bounce-back is half-way between cell centres). The cell comes from the box, so from a box
    // with no gap: the gap is small against the box and changes it little.
    double clearance = options.clearance;
    double firstCell = 0.0;
    if (options.ground) {
        firstCell =
            std::cbrt((span[0] + (kAhead + kBehind) * size) * (span[1] + 2.0 * kSideways * size) *
                      (span[2] + kSideways * size) / static_cast<double>(options.maxCells));
        clearance = std::max(clearance, 2.0 * firstCell);
    }
    const Vec3 boxLow{low[0] - kAhead * size, low[1] - kSideways * size,
                      options.ground ? low[2] - clearance - firstCell : low[2] - kSideways * size};
    const Vec3 boxHigh{high[0] + kBehind * size, high[1] + kSideways * size,
                       high[2] + kSideways * size};
    const Vec3 box = minus(boxHigh, boxLow);
    const double cell = std::cbrt(box[0] * box[1] * box[2] / static_cast<double>(options.maxCells));

    LbmConfig config;
    config.nx = static_cast<std::size_t>(std::ceil(box[0] / cell));
    config.ny = static_cast<std::size_t>(std::ceil(box[1] / cell));
    config.nz = static_cast<std::size_t>(std::ceil(box[2] / cell));
    config.cellSize = cell;
    config.windSpeed = options.speed;
    config.airDensity = options.airDensity;
    config.kinematicViscosity = options.viscosity;
    LbmTunnel tunnel(config);

    // Solids are numbered by link, in the order links first appear.
    std::vector<std::size_t> linkOfSolid;
    const auto solidOf = [&linkOfSolid](std::size_t body) {
        const auto found = std::ranges::find(linkOfSolid, body);
        if (found != linkOfSolid.end()) {
            return static_cast<std::uint16_t>(found - linkOfSolid.begin());
        }
        linkOfSolid.push_back(body);
        return static_cast<std::uint16_t>(linkOfSolid.size() - 1);
    };
    const auto cellCentre = [&boxLow, cell](std::size_t x, std::size_t y, std::size_t z) {
        return Vec3{boxLow[0] + (static_cast<double>(x) + 0.5) * cell,
                    boxLow[1] + (static_cast<double>(y) + 0.5) * cell,
                    boxLow[2] + (static_cast<double>(z) + 0.5) * cell};
    };
    const auto cellRange = [&boxLow, cell](double from, double to, std::size_t axis,
                                           std::size_t n) {
        const double start = (from - boxLow.at(axis)) / cell;
        const double end = (to - boxLow.at(axis)) / cell;
        return std::pair{static_cast<std::size_t>(std::max(std::floor(start), 0.0)),
                         std::min(static_cast<std::size_t>(std::max(std::ceil(end), 0.0)), n)};
    };
    for (const Solid& s : solids) {
        const std::uint16_t id = solidOf(s.body);
        const std::array<double, 4> back = conjugate(s.pose.orientation);
        const auto [x0, x1] = cellRange(s.low[0], s.high[0], 0, config.nx);
        const auto [y0, y1] = cellRange(s.low[1], s.high[1], 1, config.ny);
        const auto [z0, z1] = cellRange(s.low[2], s.high[2], 2, config.nz);
        for (std::size_t z = z0; z < z1; ++z) {
            for (std::size_t y = y0; y < y1; ++y) {
                for (std::size_t x = x0; x < x1; ++x) {
                    const Vec3 local = rotate(back, minus(cellCentre(x, y, z), s.pose.position));
                    if (insideLocal(s, local)) {
                        tunnel.setSolid(x, y, z, id);
                    }
                }
            }
        }
    }
    if (options.ground) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            for (std::size_t x = 0; x < config.nx; ++x) {
                tunnel.setRollingFloor(x, y, 0);
            }
        }
    }
    // Frontal area: the cells seen along the wind (the road is not part of it).
    std::size_t shadow = 0;
    for (std::size_t z = 0; z < config.nz; ++z) {
        for (std::size_t y = 0; y < config.ny; ++y) {
            for (std::size_t x = 0; x < config.nx; ++x) {
                if (z > 0 && tunnel.isSolid(tunnel.index(x, y, z))) {
                    ++shadow;
                    break;
                }
                if (z == 0 && !options.ground && tunnel.isSolid(tunnel.index(x, y, z))) {
                    ++shadow;
                    break;
                }
            }
        }
    }
    if (shadow == 0) {
        return Unexpected{std::string("the robot is thinner than a cell: give more --cells")};
    }

    if (options.device != nullptr) {
        tunnel.useGpu(*options.device);
    }
    // Until the wake has formed, then the mean over one more pass.
    const auto pass =
        static_cast<std::size_t>(static_cast<double>(config.nx) / config.latticeSpeed);
    const auto settle = static_cast<std::size_t>(options.passes * static_cast<double>(pass));
    constexpr std::size_t kChunks = 40;
    std::size_t done = 0;
    for (std::size_t i = 0; i < kChunks; ++i) {
        tunnel.step(settle / kChunks);
        done += settle / kChunks;
        if (tunnel.diverged()) {
            return Unexpected{
                std::string("the flow went unstable: give more --cells or less --speed")};
        }
        if (progress) {
            progress(static_cast<double>(i + 1) / (kChunks + 10));
        }
    }
    tunnel.startAveraging();
    std::vector<LbmTunnel::Load> mean(linkOfSolid.size());
    constexpr int kWindows = 10;
    const auto window =
        static_cast<std::size_t>(options.meanPasses * static_cast<double>(pass) / kWindows);
    std::array<double, kWindows> windowDrag{};
    const std::size_t framesPerWindow = options.frames / kWindows;
    std::size_t frame = 0;
    const auto frameY = static_cast<std::size_t>(((low[1] + high[1]) / 2.0 - boxLow[1]) / cell);
    if (framesPerWindow > 0 && !outputDirectory.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(outputDirectory) / "frames", ec);
    }
    for (std::size_t i = 0; i < kWindows; ++i) {
        // A window's loads are the mean of its pieces, however many frames cut it up.
        const std::size_t pieces =
            framesPerWindow > 0 && !outputDirectory.empty() ? framesPerWindow : 1;
        for (std::size_t f = 0; f < pieces; ++f) {
            const std::size_t steps =
                f + 1 == pieces ? window - (pieces - 1) * (window / pieces) : window / pieces;
            tunnel.step(steps);
            const double share = static_cast<double>(steps) / static_cast<double>(window);
            for (std::size_t b = 0; b < mean.size(); ++b) {
                windowDrag.at(i) += tunnel.loads()[b].force[0] * share;
                for (std::size_t k = 0; k < 3; ++k) {
                    mean[b].force.at(k) += tunnel.loads()[b].force.at(k) * share / kWindows;
                    mean[b].torque.at(k) += tunnel.loads()[b].torque.at(k) * share / kWindows;
                }
            }
            if (framesPerWindow > 0 && !outputDirectory.empty()) {
                char name[32];
                std::snprintf(name, sizeof(name), "%05zu.png", frame++);
                writeSpinFrame(std::filesystem::path(outputDirectory) / "frames" / name, tunnel,
                               frameY, options.speed, size);
            }
        }
        done += window;
        if (progress) {
            progress(static_cast<double>(kChunks + i + 1) / (kChunks + 10));
        }
    }

    WindTunnelResult result;
    result.clearance = options.ground ? clearance : 0.0;
    result.frontalArea = static_cast<double>(shadow) * cell * cell;
    result.dynamicPressure = 0.5 * options.airDensity * options.speed * options.speed;
    result.reynolds = options.speed * size / options.viscosity;
    result.cellSize = cell;
    result.grid = {config.nx, config.ny, config.nz};
    result.steps = done;
    result.seconds = static_cast<double>(done) * tunnel.timeStep();
    result.blockage = result.frontalArea / (box[1] * box[2]);
    for (std::size_t b = 0; b < mean.size(); ++b) {
        LinkLoad link;
        link.name = world.bodies.names[linkOfSolid[b]];
        link.force = mean[b].force;
        link.torque = mean[b].torque;
        result.links.push_back(std::move(link));
        result.force = plus(result.force, mean[b].force);
    }
    const double reference = result.dynamicPressure * result.frontalArea;
    result.dragCoefficient = result.force[0] / reference;
    result.sideCoefficient = result.force[1] / reference;
    double variance = 0.0;
    for (const double drag : windowDrag) {
        const double off = drag / reference - result.dragCoefficient;
        variance += off * off / kWindows;
    }
    result.dragSpread = std::sqrt(variance);
    result.liftCoefficient = result.force[2] / reference;

    if (outputDirectory.empty()) {
        return result;
    }
    std::error_code ec;
    const std::filesystem::path dir(outputDirectory);
    std::filesystem::create_directories(dir, ec);

    // The mean flow, for ParaView and as a wind field for a run (--weather wind_field=).
    writeVti(dir / "flow.vti", tunnel, boxLow);
    const auto cellAt = [&](const Vec3& p) -> std::optional<std::size_t> {
        const double fx = (p[0] - boxLow[0]) / cell;
        const double fy = (p[1] - boxLow[1]) / cell;
        const double fz = (p[2] - boxLow[2]) / cell;
        if (fx < 0.0 || fy < 0.0 || fz < 0.0 || fx >= static_cast<double>(config.nx) ||
            fy >= static_cast<double>(config.ny) || fz >= static_cast<double>(config.nz)) {
            return std::nullopt;
        }
        return tunnel.index(static_cast<std::size_t>(fx), static_cast<std::size_t>(fy),
                            static_cast<std::size_t>(fz));
    };
    fields::DenseBox fieldBox;
    fieldBox.size = cell;
    fieldBox.origin = cellCentre(0, 0, 0);
    fieldBox.count = {static_cast<int>(config.nx), static_cast<int>(config.ny),
                      static_cast<int>(config.nz)};
    auto written = fields::writeVectorField((dir / "flow.nvdb").string(), "wind", fieldBox,
                                            [&](const Vec3& at) -> Vec3 {
                                                const auto c = cellAt(at);
                                                return c ? tunnel.meanVelocity(*c) : Vec3{};
                                            });
    if (!written) {
        return Unexpected{written.error()};
    }

    // Streamlines: tubes from a grid of seeds ahead of the body, coloured by speed.
    Bands streamlines;
    const double fastest = 1.6 * options.speed;
    const int speedBands = kColours;
    const auto bandOf = [speedBands](double value, double low2, double high2) {
        return static_cast<int>((value - low2) / (high2 - low2) * speedBands);
    };
    const double radius = size * 0.004;
    for (int j = 0; j < kSeedsAcross; ++j) {
        for (int k = 0; k < kSeedsAcross; ++k) {
            const auto along = [&](double l, double h, int i) {
                const double reach = (h - l) * 0.15;
                return l - reach + (h - l + 2.0 * reach) * (i + 0.5) / kSeedsAcross;
            };
            Vec3 p{low[0] - 0.5 * size, along(low[1], high[1], j), along(low[2], high[2], k)};
            std::array<Vec3, kTubeSides> ring{};
            bool started = false;
            const std::size_t most = 4 * (config.nx + config.ny);
            for (std::size_t s = 0; s < most; ++s) {
                const auto here = cellAt(p);
                if (!here || tunnel.isSolid(*here)) {
                    break;
                }
                const Vec3 u0 = tunnel.meanVelocity(*here);
                const double speed0 = length(u0);
                if (speed0 < 0.02 * options.speed) {
                    break;
                }
                const double h = 0.8 * cell / speed0;
                const Vec3 middle = plus(p, times(u0, h / 2.0));
                const auto mid = cellAt(middle);
                if (!mid || tunnel.isSolid(*mid)) {
                    break;
                }
                const Vec3 u = tunnel.meanVelocity(*mid);
                const double speed = length(u);
                if (speed < 0.02 * options.speed) {
                    break;
                }
                const Vec3 step = times(u, 0.8 * cell / speed);
                const Vec3 axis = times(step, 1.0 / length(step));
                const Vec3 pick =
                    std::abs(axis[2]) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
                const Vec3 e1 = times(cross(axis, pick), 1.0 / length(cross(axis, pick)));
                const Vec3 e2 = cross(axis, e1);
                std::array<Vec3, kTubeSides> next{};
                const Vec3 ahead = plus(p, step);
                for (int m = 0; m < kTubeSides; ++m) {
                    const double angle = 2.0 * std::numbers::pi * m / kTubeSides;
                    next.at(static_cast<std::size_t>(m)) =
                        plus(ahead, plus(times(e1, radius * std::cos(angle)),
                                         times(e2, radius * std::sin(angle))));
                }
                if (started) {
                    const int band = bandOf(speed, 0.0, fastest);
                    for (std::size_t m = 0; m < kTubeSides; ++m) {
                        const std::size_t n = (m + 1) % kTubeSides;
                        streamlines.quad(band, ring.at(m), ring.at(n), next.at(n), next.at(m));
                    }
                }
                ring = next;
                started = true;
                p = ahead;
            }
        }
    }

    // A slice down the middle, along the wind: speed.
    Bands slice;
    const auto midY = static_cast<std::size_t>(((low[1] + high[1]) / 2.0 - boxLow[1]) / cell);
    const std::size_t stride = std::max<std::size_t>(1, config.nx / 300);
    for (std::size_t z = 0; z + stride < config.nz; z += stride) {
        for (std::size_t x = 0; x + stride < config.nx; x += stride) {
            const std::size_t c = tunnel.index(x, midY, z);
            const Vec3 at = cellCentre(x, midY, z);
            // Near the body, where there is something to see: half a size around it, more behind.
            if (tunnel.isSolid(c) || at[0] < low[0] - 0.5 * size || at[0] > high[0] + 1.5 * size ||
                at[2] < low[2] - 0.5 * size || at[2] > high[2] + 0.5 * size) {
                continue;
            }
            const double w = cell * static_cast<double>(stride);
            slice.quad(bandOf(length(tunnel.meanVelocity(c)), 0.0, fastest), at,
                       plus(at, {w, 0.0, 0.0}), plus(at, {w, 0.0, w}), plus(at, {0.0, 0.0, w}));
        }
    }

    // The body's skin: the pressure coefficient just outside it, on triangles no larger than a
    // cell or two.
    Bands skin;
    const double reference2 = result.dynamicPressure;
    const auto cpBand = [&](const Vec3& p, const Vec3& normal) {
        for (const double reach : {1.5, 2.5, 3.5}) {
            const auto c = cellAt(plus(p, times(normal, reach * cell)));
            if (c && !tunnel.isSolid(*c)) {
                return bandOf(tunnel.meanPressure(*c) / reference2, -1.0, 1.0);
            }
        }
        return kColours / 2;
    };
    for (const Solid& s : solids) {
        const Vec3 centre = s.pose.position;
        std::vector<std::array<Vec3, 3>> triangles = surfaceOf(s);
        // Split what is large until each piece is about two cells across.
        for (std::size_t i = 0; i < triangles.size() && triangles.size() < 200000; ++i) {
            while (true) {
                const std::array<Vec3, 3> t = triangles[i];
                const double a = length(minus(t[0], t[1]));
                const double b = length(minus(t[1], t[2]));
                const double d = length(minus(t[2], t[0]));
                const double longest = std::max({a, b, d});
                if (longest <= 2.0 * cell || triangles.size() >= 200000) {
                    break;
                }
                const int edge = longest == a ? 0 : longest == b ? 1 : 2;
                const Vec3& p0 = t.at(static_cast<std::size_t>(edge));
                const Vec3& p1 = t.at(static_cast<std::size_t>((edge + 1) % 3));
                const Vec3& p2 = t.at(static_cast<std::size_t>((edge + 2) % 3));
                const Vec3 mid = times(plus(p0, p1), 0.5);
                triangles[i] = {p0, mid, p2};
                triangles.push_back({mid, p1, p2});
            }
        }
        for (const auto& t : triangles) {
            const auto world3 = [&s](const Vec3& q) {
                return plus(rotate(s.pose.orientation, q), s.pose.position);
            };
            const Vec3 a = world3(t[0]);
            const Vec3 b = world3(t[1]);
            const Vec3 c = world3(t[2]);
            Vec3 normal = cross(minus(b, a), minus(c, a));
            const double area = length(normal);
            if (area <= 0.0) {
                continue;
            }
            normal = times(normal, 1.0 / area);
            const Vec3 middle = times(plus(plus(a, b), c), 1.0 / 3.0);
            if (dot(normal, minus(middle, centre)) < 0.0) {
                normal = times(normal, -1.0);
            }
            skin.add(cpBand(middle, normal), a, b, c);
        }
    }

    std::ofstream usd(dir / "flow_view.usda");
    usd << "#usda 1.0\n(\n    defaultPrim = \"flow\"\n    metersPerUnit = 1\n    upAxis = "
           "\"Z\"\n)\n"
        << "def Xform \"flow\"\n{\n";
    streamlines.write(usd, "streamlines_speed", viridis, 0.0, fastest, "mps");
    slice.write(usd, "slice_speed", viridis, 0.0, fastest, "mps");
    skin.write(usd, "surface_pressure_coefficient", diverging, -1.0, 1.0, "cp");
    usd << "}\n";

    std::ofstream report(dir / "report.json");
    report << "{\n  \"speed_m_s\": " << options.speed << ",\n  \"yaw_deg\": " << options.yawDegrees
           << ",\n  \"reynolds\": " << result.reynolds
           << ",\n  \"frontal_area_m2\": " << result.frontalArea
           << ",\n  \"drag_coefficient\": " << result.dragCoefficient
           << ",\n  \"drag_spread\": " << result.dragSpread
           << ",\n  \"lift_coefficient\": " << result.liftCoefficient
           << ",\n  \"side_coefficient\": " << result.sideCoefficient << ",\n  \"force_n\": ["
           << result.force[0] << ", " << result.force[1] << ", " << result.force[2]
           << "],\n  \"cell_m\": " << cell << ",\n  \"grid\": [" << config.nx << ", " << config.ny
           << ", " << config.nz << "],\n  \"blockage\": " << result.blockage << ",\n  \"links\": {";
    for (std::size_t b = 0; b < result.links.size(); ++b) {
        const LinkLoad& link = result.links[b];
        report << (b == 0 ? "\n" : ",\n") << "    \"" << link.name << "\": [" << link.force[0]
               << ", " << link.force[1] << ", " << link.force[2] << "]";
    }
    report << "\n  }\n}\n";
    return result;
}

GridStudyVerdict gridStudyVerdict(const std::vector<GridStudyRow>& rows) {
    GridStudyVerdict verdict;
    if (rows.size() < 2 || rows.back().dragCoefficient == 0.0) {
        return verdict;
    }
    const GridStudyRow& last = rows.back();
    const double size = std::abs(last.dragCoefficient);
    verdict.dragChange =
        std::abs(last.dragCoefficient - rows[rows.size() - 2].dragCoefficient) / size;
    verdict.dragMoves = last.dragSpread / size;
    verdict.liftChange =
        std::abs(last.liftCoefficient - rows[rows.size() - 2].liftCoefficient) / size;
    verdict.settled = verdict.dragChange <= kGridSettledShare &&
                      verdict.dragMoves <= kGridSettledShare &&
                      verdict.liftChange <= kGridSettledShare;
    return verdict;
}

std::string exportTunnelSurface(const World& world, double yawDegrees,
                                const std::string& directory) {
    const double half = yawDegrees * std::numbers::pi / 360.0;
    const Pose yaw{{0.0, 0.0, 0.0}, {std::cos(half), 0.0, 0.0, std::sin(half)}};
    const std::vector<Pose> poses = restPoses(world);
    const World::Colliders& colliders = world.colliders;
    const std::filesystem::path dir(directory);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::vector<std::string> names;
    std::vector<std::ostringstream> links;
    Vec3 low{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    Vec3 high{-low[0], -low[1], -low[2]};
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        if (colliders.body[c] == kWorldBody || colliders.shape[c] == ShapeType::Plane ||
            colliders.shape[c] == ShapeType::HeightField) {
            continue;
        }
        Solid s;
        s.body = colliders.body[c].index;
        s.shape = colliders.shape[c];
        s.size = colliders.size[c];
        s.pose = compose(yaw, compose(poses[s.body], colliders.poseInBody[c]));
        s.hull = s.shape == ShapeType::ConvexHull ? &world.hulls.at(colliders.hull[c]) : nullptr;
        std::string name = world.bodies.names[s.body];
        std::ranges::replace_if(
            name, [](char ch) { return std::isalnum(static_cast<unsigned char>(ch)) == 0; }, '_');
        const auto found = std::ranges::find(names, name);
        const auto at = static_cast<std::size_t>(found - names.begin());
        if (found == names.end()) {
            names.push_back(name);
            links.emplace_back();
        }
        for (const auto& triangle : surfaceOf(s)) {
            std::array<Vec3, 3> w{};
            for (std::size_t k = 0; k < 3; ++k) {
                w.at(k) = plus(rotate(s.pose.orientation, triangle.at(k)), s.pose.position);
                for (std::size_t a = 0; a < 3; ++a) {
                    low.at(a) = std::min(low.at(a), w.at(k).at(a));
                    high.at(a) = std::max(high.at(a), w.at(k).at(a));
                }
            }
            const Vec3 e1 = minus(w[1], w[0]);
            const Vec3 e2 = minus(w[2], w[0]);
            Vec3 n{e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                   e1[0] * e2[1] - e1[1] * e2[0]};
            const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (length == 0.0) {
                continue;
            }
            std::ostringstream& out = links[at];
            out << std::setprecision(12);
            out << "facet normal " << n[0] / length << ' ' << n[1] / length << ' ' << n[2] / length
                << "\nouter loop\n";
            for (const Vec3& v : w) {
                out << "vertex " << v[0] << ' ' << v[1] << ' ' << v[2] << '\n';
            }
            out << "endloop\nendfacet\n";
        }
    }
    if (names.empty()) {
        return "the robot has no collision shapes for the air to meet";
    }
    std::ofstream json(dir / "surface.json");
    json << std::setprecision(12) << "{\"links\": [";
    for (std::size_t i = 0; i < names.size(); ++i) {
        std::ofstream stl(dir / (names[i] + ".stl"));
        stl << "solid " << names[i] << '\n' << links[i].str() << "endsolid " << names[i] << '\n';
        json << (i > 0 ? ", " : "") << '"' << names[i] << '"';
    }
    json << "], \"low\": [" << low[0] << ", " << low[1] << ", " << low[2] << "], \"high\": ["
         << high[0] << ", " << high[1] << ", " << high[2] << "]}\n";
    return json ? std::string{} : "could not write " + directory;
}

}  // namespace lagrange::aerodynamics
