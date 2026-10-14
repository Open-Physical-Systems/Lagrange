#include "lagrange/server/MapExport.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

namespace lagrange::server {

namespace {

/// Whether a point given in a shape's own frame lies inside it.
bool inside(ShapeType shape, const std::array<double, 3>& size, const std::array<double, 3>& p) {
    switch (shape) {
        case ShapeType::Box:
            return std::abs(p[0]) <= size[0] && std::abs(p[1]) <= size[1] &&
                   std::abs(p[2]) <= size[2];
        case ShapeType::Sphere:
            return p[0] * p[0] + p[1] * p[1] + p[2] * p[2] <= size[0] * size[0];
        case ShapeType::Cylinder:
            return p[0] * p[0] + p[1] * p[1] <= size[0] * size[0] && std::abs(p[2]) <= size[1];
        case ShapeType::Capsule: {
            const double z = std::clamp(p[2], -size[1], size[1]);
            return p[0] * p[0] + p[1] * p[1] + (p[2] - z) * (p[2] - z) <= size[0] * size[0];
        }
        case ShapeType::Plane:
            return false;
    }
    return false;
}

/// Half-extent of the shape at `pose` along world x and y (exact for every shape).
std::array<double, 2> halfExtent(ShapeType shape, const std::array<double, 3>& size,
                                 const Pose& pose) {
    std::array<double, 2> out{0.0, 0.0};
    if (shape == ShapeType::Box) {
        for (int axis = 0; axis < 3; ++axis) {
            std::array<double, 3> edge{0.0, 0.0, 0.0};
            edge[axis] = size[axis];
            const std::array<double, 3> world = rotate(pose.orientation, edge);
            out[0] += std::abs(world[0]);
            out[1] += std::abs(world[1]);
        }
    } else if (shape == ShapeType::Sphere) {
        out = {size[0], size[0]};
    } else if (shape == ShapeType::Cylinder || shape == ShapeType::Capsule) {
        // Along world axis i the end caps reach |a_i| h; a disc of radius r across the axis
        // reaches r sqrt(1 - a_i²) and a capsule's hemispheres reach r.
        const std::array<double, 3> axis = rotate(pose.orientation, {0.0, 0.0, 1.0});
        for (int i = 0; i < 2; ++i) {
            const double across = shape == ShapeType::Cylinder
                                      ? size[0] * std::sqrt(std::max(0.0, 1.0 - axis[i] * axis[i]))
                                      : size[0];
            out[i] = std::abs(axis[i]) * size[1] + across;
        }
    }
    return out;
}

}  // namespace

Expected<void, std::string> exportOccupancyMap(const World& world, const std::string& yamlPath,
                                               const MapSettings& settings) {
    struct Shape {
        ShapeType type;
        std::array<double, 3> size;
        Pose pose;
    };
    const std::vector<Pose> rest = restPoses(world);
    const std::vector<bool> moving = movingBodies(world);
    const World::Colliders& colliders = world.colliders;
    std::vector<Shape> shapes;
    double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;
    for (std::size_t c = 0; c < colliders.body.size(); ++c) {
        const BodyId owner = colliders.body[c];
        if (colliders.shape[c] == ShapeType::Plane ||
            (owner != kWorldBody && moving[owner.index])) {
            continue;
        }
        const Pose pose = owner == kWorldBody ? colliders.poseInBody[c]
                                              : compose(rest[owner.index], colliders.poseInBody[c]);
        const std::array<double, 2> half = halfExtent(colliders.shape[c], colliders.size[c], pose);
        if (shapes.empty()) {
            minX = maxX = pose.position[0];
            minY = maxY = pose.position[1];
        }
        minX = std::min(minX, pose.position[0] - half[0]);
        maxX = std::max(maxX, pose.position[0] + half[0]);
        minY = std::min(minY, pose.position[1] - half[1]);
        maxY = std::max(maxY, pose.position[1] + half[1]);
        shapes.push_back({colliders.shape[c], colliders.size[c], pose});
    }
    if (shapes.empty()) {
        return Unexpected{std::string("the world has no fixed geometry")};
    }
    const double resolution = settings.resolutionM;
    minX -= settings.marginM;
    minY -= settings.marginM;
    const auto width =
        static_cast<std::size_t>(std::ceil((maxX + settings.marginM - minX) / resolution));
    const auto height =
        static_cast<std::size_t>(std::ceil((maxY + settings.marginM - minY) / resolution));

    // Row 0 of a PGM is the top of the image, i.e. the largest y.
    std::vector<unsigned char> pixels(width * height, 254);
    for (std::size_t row = 0; row < height; ++row) {
        const double y = minY + (static_cast<double>(height - 1 - row) + 0.5) * resolution;
        for (std::size_t col = 0; col < width; ++col) {
            const double x = minX + (static_cast<double>(col) + 0.5) * resolution;
            for (const Shape& shape : shapes) {
                const std::array<double, 3> local =
                    rotate(conjugate(shape.pose.orientation),
                           {x - shape.pose.position[0], y - shape.pose.position[1],
                            settings.heightM - shape.pose.position[2]});
                if (inside(shape.type, shape.size, local)) {
                    pixels[row * width + col] = 0;
                    break;
                }
            }
        }
    }

    const std::filesystem::path yaml(yamlPath);
    const std::filesystem::path image = std::filesystem::path(yaml).replace_extension(".pgm");
    std::ofstream pgm(image, std::ios::binary);
    pgm << "P5\n# lagrange_server --export-map\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char*>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
    std::ofstream text(yaml);
    text << "# Occupancy map of the fixed geometry at z = " << settings.heightM
         << " m, written by lagrange_server --export-map.\n"
         << "image: " << image.filename().string() << "\n"
         << "mode: trinary\n"
         << "resolution: " << resolution << "\n"
         << "origin: [" << minX << ", " << minY << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n";
    if (!pgm || !text) {
        return Unexpected{"cannot write " + yaml.string() + " or " + image.string()};
    }
    return {};
}

}  // namespace lagrange::server
