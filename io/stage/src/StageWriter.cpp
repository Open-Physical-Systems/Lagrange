// Model -> USD: the back end of the importers. Writes what loadStage reads back (StageReader.cpp
// documents the rules in the other direction).
#include "Math.hpp"
#include "Schemas.hpp"
#include "lagrange/stage/Stage.hpp"
#include "lagrange/world/PoseMath.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <numbers>
#include <set>
#include <string>
#include <vector>

#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec2d.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/capsule.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/materialAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/metrics.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

namespace lagrange::stage {

namespace {

using namespace pxr;  // NOLINT(google-build-using-namespace): OpenUSD names are long enough

// UsdGeomCamera's fallback horizontal aperture (tenths of a scene unit, by its convention). Only
// the ratio to the focal length matters here.
constexpr double kHorizontalApertureMm = 20.955;

GfVec3d vec3d(const std::array<double, 3>& v) {
    return {v[0], v[1], v[2]};
}
GfVec3f vec3f(const std::array<double, 3>& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}
GfQuatd quatd(const std::array<double, 4>& q) {
    return {q[0], q[1], q[2], q[3]};
}
GfQuatf quatf(const std::array<double, 4>& q) {
    return {static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]),
            static_cast<float>(q[3])};
}

/// Translate, orient and (when not 1) scale, in that order.
void setTransform(const UsdGeomXformable& xformable, const Pose& pose,
                  const std::array<float, 3>& scale = {1.0F, 1.0F, 1.0F}) {
    xformable.AddTranslateOp(UsdGeomXformOp::PrecisionDouble).Set(vec3d(pose.position));
    xformable.AddOrientOp(UsdGeomXformOp::PrecisionDouble).Set(quatd(pose.orientation));
    if (scale != std::array<float, 3>{1.0F, 1.0F, 1.0F}) {
        xformable.AddScaleOp(UsdGeomXformOp::PrecisionFloat)
            .Set(GfVec3f(scale[0], scale[1], scale[2]));
    }
}

/// A valid prim name: letters, digits and underscores, not starting with a digit.
std::string primName(const std::string& name) {
    std::string out;
    for (const char c : name) {
        out += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
    }
    if (out.empty() || std::isdigit(static_cast<unsigned char>(out.front())) != 0) {
        out.insert(out.begin(), '_');
    }
    return out;
}

class Writer {
public:
    Writer(const Model& model, UsdStageRefPtr stage, std::filesystem::path directory,
           std::string staticColliders)
        : m_(model)
        , stage_(std::move(stage))
        , directory_(std::move(directory))
        , staticColliders_(std::move(staticColliders)) {}

    Expected<void, std::string> write() {
        const std::vector<bool> moving = movingBodies(m_.world);
        rigid_.assign(m_.world.bodies.names.size(), false);
        for (std::size_t b = 0; b < rigid_.size(); ++b) {
            // Static scenery (welded to the world, no mass) needs no body: its colliders become
            // world colliders, which is what the physics makes of it anyway.
            rigid_[b] = moving[b] || m_.world.bodies.mass[b] > 0.0 ||
                        m_.world.bodies.parent[b] != kWorldBody;
        }
        primOfBody_.assign(rigid_.size(), kNoIndex);
        for (std::size_t b = 0; b < m_.bodyPrim.size() && b < rigid_.size(); ++b) {
            primOfBody_[b] = m_.bodyPrim[b];
        }
        for (std::size_t s = 0; s < m_.sensorPrim.size(); ++s) {
            sensorOfPrim_[m_.sensorPrim[s]] = s;
        }

        const std::vector<ScenePrim>& prims = m_.scene.prims;
        paths_.resize(prims.size());
        for (std::size_t p = 0; p < prims.size(); ++p) {
            if (skipped(p)) {
                continue;
            }
            const ScenePrim& prim = prims[p];
            const SdfPath parent =
                prim.parent == kNoIndex ? SdfPath::AbsoluteRootPath() : paths_[prim.parent];
            paths_[p] = parent.AppendChild(TfToken(primName(prim.name)));
            if (auto status = writePrim(p); !status) {
                return status;
            }
        }
        if (auto status = writeBodies(); !status) {
            return status;
        }
        writeColliders();
        if (auto status = writeJoints(); !status) {
            return status;
        }
        return writeSchemas();
    }

private:
    [[nodiscard]] bool skipped(std::size_t p) const {
        const ScenePrim& prim = m_.scene.prims[p];
        if (prim.parent == kNoIndex && prim.name == "ground" && prim.body == kNoIndex) {
            return true;  // loadStage adds the ground again
        }
        return prim.parent != kNoIndex && skipped(prim.parent);
    }

    [[nodiscard]] Pose worldPose(std::uint32_t p) const {
        const ScenePrim& prim = m_.scene.prims[p];
        return prim.body == kNoIndex ? prim.offset
                                     : compose(m_.bodyRestPose[prim.body], prim.offset);
    }

    [[nodiscard]] Pose localPose(std::uint32_t p) const {
        const ScenePrim& prim = m_.scene.prims[p];
        const Pose world = worldPose(p);
        if (prim.parent == kNoIndex) {
            return world;
        }
        return relative(worldPose(prim.parent), world);
    }

    Expected<void, std::string> writePrim(std::size_t p) {
        const ScenePrim& prim = m_.scene.prims[p];
        const auto index = static_cast<std::uint32_t>(p);
        if (prim.typeName == "Mesh" && prim.mesh != kNoIndex) {
            const TriMesh& mesh = m_.scene.meshes[prim.mesh];
            UsdGeomMesh usd = UsdGeomMesh::Define(stage_, paths_[p]);
            setTransform(usd, localPose(index), prim.scale);
            VtArray<GfVec3f> points;
            VtArray<GfVec3f> normals;
            for (std::size_t v = 0; v + 2 < mesh.positions.size(); v += 3) {
                points.push_back({mesh.positions[v], mesh.positions[v + 1], mesh.positions[v + 2]});
                normals.push_back({mesh.normals[v], mesh.normals[v + 1], mesh.normals[v + 2]});
            }
            VtArray<int> counts(mesh.indices.size() / 3, 3);
            VtArray<int> indices(mesh.indices.begin(), mesh.indices.end());
            usd.CreatePointsAttr().Set(points);
            usd.CreateNormalsAttr().Set(normals);
            usd.SetNormalsInterpolation(UsdGeomTokens->vertex);
            usd.CreateFaceVertexCountsAttr().Set(counts);
            usd.CreateFaceVertexIndicesAttr().Set(indices);
            usd.CreateSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
            if (prim.material != kNoIndex) {
                const std::array<float, 4>& rgba = m_.scene.materials[prim.material].rgba;
                usd.CreateDisplayColorAttr().Set(VtArray<GfVec3f>{{rgba[0], rgba[1], rgba[2]}});
                if (rgba[3] < 1.0F) {
                    usd.CreateDisplayOpacityAttr().Set(VtArray<float>{rgba[3]});
                }
                bindLook(usd.GetPrim(), m_.scene.materials[prim.material]);
            }
            return {};
        }
        const auto sensor = sensorOfPrim_.find(index);
        if (sensor != sensorOfPrim_.end() &&
            m_.world.sensors.type[sensor->second] == SensorType::Camera) {
            // A UsdGeomCamera looks along its -z: turn the sensor frame (x forward) into it.
            UsdGeomCamera camera = UsdGeomCamera::Define(stage_, paths_[p]);
            const Pose mount = localPose(index);
            setTransform(camera,
                         {mount.position, multiply(mount.orientation, kSensorFromUsdCamera)});
            return writeSensor(camera.GetPrim(), sensor->second);
        }
        UsdGeomXform xform = UsdGeomXform::Define(stage_, paths_[p]);
        setTransform(xform, localPose(index));
        if (sensor != sensorOfPrim_.end()) {
            return writeSensor(xform.GetPrim(), sensor->second);
        }
        return {};
    }

    Expected<void, std::string> writeSensor(const UsdPrim& prim, std::size_t s) {
        const World::Sensors& sensors = m_.world.sensors;
        const auto& settings = sensors.settings[s];
        const auto get = [&settings](const char* key, double fallback) {
            const auto it = settings.find(key);
            return it == settings.end() ? fallback : it->second;
        };
        const auto real = [&prim](const char* name, double value) {
            prim.CreateAttribute(TfToken(name), SdfValueTypeNames->Double).Set(value);
        };
        if (!prim.ApplyAPI(TfToken("SensorAPI_1"))) {
            return Unexpected{std::string(
                "cannot apply SensorAPI_1: is lagrange_schemas on PXR_PLUGINPATH_NAME?")};
        }
        real("sensor:rate", sensors.rateHz[s]);
        if (get("latency", 0.0) > 0.0) {
            real("sensor:latency", get("latency", 0.0));
        }
        if (get("jitter", 0.0) > 0.0) {
            real("sensor:jitter", get("jitter", 0.0));
        }
        if (sensors.type[s] == SensorType::Lidar) {
            prim.ApplyAPI(TfToken("LidarAPI_1"));
            prim.CreateAttribute(TfToken("sensor:lidar:channels"), SdfValueTypeNames->Int)
                .Set(static_cast<int>(get("vertical_samples", 1.0)));
            prim.CreateAttribute(TfToken("sensor:lidar:horizontalSamples"), SdfValueTypeNames->Int)
                .Set(static_cast<int>(get("samples", 360.0)));
            prim.CreateAttribute(TfToken("sensor:lidar:horizontalFov"), SdfValueTypeNames->Double2)
                .Set(GfVec2d(get("min_angle", -std::numbers::pi),
                             get("max_angle", std::numbers::pi)));
            prim.CreateAttribute(TfToken("sensor:lidar:verticalFov"), SdfValueTypeNames->Double2)
                .Set(GfVec2d(get("vertical_min_angle", 0.0), get("vertical_max_angle", 0.0)));
            real("sensor:lidar:rangeMin", get("min_range", 0.1));
            real("sensor:lidar:rangeMax", get("max_range", 30.0));
            real("sensor:lidar:rangeNoiseStd", get("range_noise", 0.0));
            if (get("motion_distortion", 1.0) == 0.0) {
                prim.CreateAttribute(TfToken("sensor:lidar:motionDistortion"),
                                     SdfValueTypeNames->Bool)
                    .Set(false);
            }
        } else if (sensors.type[s] == SensorType::Camera) {
            prim.ApplyAPI(TfToken("CameraSensorAPI_1"));
            const double width = get("width", 640.0);
            const double height = get("height", 480.0);
            prim.CreateAttribute(TfToken("sensor:camera:width"), SdfValueTypeNames->Int)
                .Set(static_cast<int>(width));
            prim.CreateAttribute(TfToken("sensor:camera:height"), SdfValueTypeNames->Int)
                .Set(static_cast<int>(height));
            // The lens: square pixels, so the apertures have the image's aspect ratio.
            const UsdGeomCamera camera(prim);
            const double aperture = kHorizontalApertureMm;
            const double focal = aperture / 2.0 / std::tan(get("horizontal_fov", 1.047) / 2.0);
            camera.CreateFocalLengthAttr().Set(static_cast<float>(focal));
            camera.CreateHorizontalApertureAttr().Set(static_cast<float>(aperture));
            camera.CreateVerticalApertureAttr().Set(static_cast<float>(aperture * height / width));
            camera.CreateClippingRangeAttr().Set(
                GfVec2f(static_cast<float>(get("near_clip", 0.05)),
                        static_cast<float>(get("far_clip", 100.0))));
            const auto token = [&prim](const char* name, const char* value) {
                prim.CreateAttribute(TfToken(name), SdfValueTypeNames->Token, false,
                                     SdfVariabilityUniform)
                    .Set(TfToken(value));
            };
            if (const auto it = settings.find("rolling_shutter_time"); it != settings.end()) {
                token("sensor:camera:shutter", "rolling");
                real("sensor:camera:rollingShutterTime", it->second);
            }
            if (const double model = get("distortion_model", 0.0); model > 0.0) {
                token("sensor:camera:distortionModel", model == 1.0 ? "plumb_bob" : "equidistant");
                VtArray<double> coeffs;
                for (int k = 1; k <= (model == 1.0 ? 5 : 4); ++k) {
                    coeffs.push_back(get(("distortion_k" + std::to_string(k)).c_str(), 0.0));
                }
                prim.CreateAttribute(TfToken("sensor:camera:distortionCoeffs"),
                                     SdfValueTypeNames->DoubleArray)
                    .Set(coeffs);
            }
            if (const auto it = settings.find("vignetting"); it != settings.end()) {
                real("sensor:camera:vignetting", it->second);
            }
            if (const auto it = settings.find("exposure_time"); it != settings.end()) {
                real("sensor:camera:exposureTime", it->second);
                prim.CreateAttribute(TfToken("sensor:camera:autoExposure"), SdfValueTypeNames->Bool)
                    .Set(get("auto_exposure", 0.0) != 0.0);
                real("sensor:camera:electronRate", get("electron_rate", 0.0));
                real("sensor:camera:gain", get("gain", 1.0));
                real("sensor:camera:readNoiseStd", get("read_noise", 2.0));
                real("sensor:camera:fullWellCapacity", get("full_well", 10000.0));
                prim.CreateAttribute(TfToken("sensor:camera:bitDepth"), SdfValueTypeNames->Int)
                    .Set(static_cast<int>(get("bit_depth", 8.0)));
            }
        } else if (sensors.type[s] == SensorType::Gnss) {
            prim.ApplyAPI(TfToken("GnssAPI_1"));
            real("sensor:gnss:horizontalNoiseStd", get("horizontal_noise", 0.0));
            real("sensor:gnss:verticalNoiseStd", get("vertical_noise", 0.0));
            real("sensor:gnss:velocityNoiseStd", get("velocity_noise", 0.0));
        } else if (sensors.type[s] == SensorType::Magnetometer) {
            prim.ApplyAPI(TfToken("MagnetometerAPI_1"));
            real("sensor:magnetometer:noiseStd", get("noise", 0.0));
        } else if (sensors.type[s] == SensorType::Barometer) {
            prim.ApplyAPI(TfToken("BarometerAPI_1"));
            real("sensor:barometer:noiseStd", get("noise", 0.0));
        } else if (sensors.type[s] == SensorType::Radar) {
            prim.ApplyAPI(TfToken("RadarAPI_1"));
            for (const auto& [key, attribute] :
                 {std::pair{"carrier_frequency", "sensor:radar:carrierFrequency"},
                  std::pair{"max_range", "sensor:radar:rangeMax"},
                  std::pair{"azimuth_fov", "sensor:radar:azimuthFov"},
                  std::pair{"elevation_fov", "sensor:radar:elevationFov"},
                  std::pair{"azimuth_resolution", "sensor:radar:azimuthResolution"},
                  std::pair{"range_resolution", "sensor:radar:rangeResolution"},
                  std::pair{"velocity_resolution", "sensor:radar:velocityResolution"},
                  std::pair{"tx_power", "sensor:radar:txPower"},
                  std::pair{"antenna_gain", "sensor:radar:antennaGain"},
                  std::pair{"noise_floor", "sensor:radar:noiseFloor"},
                  std::pair{"detection_threshold", "sensor:radar:detectionThreshold"}}) {
                if (const auto it = sensors.settings[s].find(key);
                    it != sensors.settings[s].end()) {
                    real(attribute, it->second);
                }
            }
        } else if (sensors.type[s] == SensorType::Custom) {
            prim.ApplyAPI(TfToken("CustomSensorAPI_1"));
            prim.CreateAttribute(TfToken("sensor:custom:kind"), SdfValueTypeNames->String)
                .Set(sensors.kind[s]);
            const auto param = [](std::string key) {
                std::replace(key.begin(), key.end(), '/', ':');
                return TfToken("sensor:custom:param:" + key);
            };
            for (const auto& [key, value] : sensors.settings[s]) {
                prim.CreateAttribute(param(key), SdfValueTypeNames->Double).Set(value);
            }
            for (const auto& [key, value] : sensors.text[s]) {
                prim.CreateAttribute(param(key), SdfValueTypeNames->String).Set(value);
            }
        } else {
            prim.ApplyAPI(TfToken("ImuAPI_1"));
            real("sensor:imu:accelNoiseStd", get("accel_noise", 0.0));
            real("sensor:imu:gyroNoiseStd", get("gyro_noise", 0.0));
        }
        return {};
    }

    Expected<void, std::string> writeBodies() {
        const World::Bodies& bodies = m_.world.bodies;
        for (std::size_t b = 0; b < rigid_.size(); ++b) {
            if (!rigid_[b]) {
                continue;
            }
            if (primOfBody_[b] == kNoIndex || paths_[primOfBody_[b]].IsEmpty()) {
                return Unexpected{"body '" + bodies.names[b] + "' has no prim to write it on"};
            }
            const UsdPrim prim = stage_->GetPrimAtPath(paths_[primOfBody_[b]]);
            UsdPhysicsRigidBodyAPI::Apply(prim);
            UsdPhysicsMassAPI mass = UsdPhysicsMassAPI::Apply(prim);
            mass.CreateMassAttr().Set(static_cast<float>(bodies.mass[b]));
            mass.CreateCenterOfMassAttr().Set(vec3f(bodies.comLocal[b]));
            const Principal principal = principalAxes(bodies.inertiaLocal[b]);
            mass.CreateDiagonalInertiaAttr().Set(vec3f(principal.moments));
            mass.CreatePrincipalAxesAttr().Set(quatf(principal.axes));
        }
        // A top prim that moves with a robot's root body without being it marks the articulation.
        const std::vector<ScenePrim>& prims = m_.scene.prims;
        for (std::size_t p = 0; p < prims.size(); ++p) {
            if (paths_[p].IsEmpty() || prims[p].parent != kNoIndex || prims[p].body == kNoIndex) {
                continue;
            }
            if (primOfBody_[prims[p].body] != p) {
                UsdPhysicsArticulationRootAPI::Apply(stage_->GetPrimAtPath(paths_[p]));
            }
        }
        return {};
    }

    /// The prim a collider is written under: its body's prim, or a scope for static geometry
    /// (WriteOptions::staticColliders).
    SdfPath colliderParent(BodyId body) {
        if (body != kWorldBody && rigid_[body.index]) {
            return paths_[primOfBody_[body.index]];
        }
        if (body != kWorldBody && primOfBody_[body.index] != kNoIndex) {
            return paths_[primOfBody_[body.index]];  // static scenery: under its own prim
        }
        const SdfPath scope = SdfPath(staticColliders_);
        UsdGeomScope::Define(stage_, scope);
        return scope;
    }

    void writeColliders() {
        const World::Colliders& colliders = m_.world.colliders;
        std::map<SdfPath, int> used;
        for (std::size_t c = 0; c < colliders.body.size(); ++c) {
            const BodyId body = colliders.body[c];
            if (body == kWorldBody && colliders.shape[c] == ShapeType::Plane) {
                continue;  // the ground: loadStage adds it
            }
            const SdfPath parent = colliderParent(body);
            const int n = used[parent]++;
            const SdfPath path = parent.AppendChild(
                TfToken(n == 0 ? "collision" : "collision_" + std::to_string(n)));
            // Posed in the prim it is written under: its body's prim, or the world.
            Pose pose = colliders.poseInBody[c];
            if (body != kWorldBody) {
                pose = relative(worldPose(primOfBody_[body.index]),
                                compose(m_.bodyRestPose[body.index], pose));
            }
            const std::array<double, 3>& size = colliders.size[c];
            UsdGeomGprim gprim;
            switch (colliders.shape[c]) {
                case ShapeType::Box: {
                    UsdGeomCube cube = UsdGeomCube::Define(stage_, path);
                    cube.CreateSizeAttr().Set(1.0);
                    setTransform(cube, pose,
                                 {static_cast<float>(2 * size[0]), static_cast<float>(2 * size[1]),
                                  static_cast<float>(2 * size[2])});
                    gprim = cube;
                    break;
                }
                case ShapeType::Sphere: {
                    UsdGeomSphere sphere = UsdGeomSphere::Define(stage_, path);
                    sphere.CreateRadiusAttr().Set(size[0]);
                    setTransform(sphere, pose);
                    gprim = sphere;
                    break;
                }
                case ShapeType::Cylinder: {
                    UsdGeomCylinder cylinder = UsdGeomCylinder::Define(stage_, path);
                    cylinder.CreateRadiusAttr().Set(size[0]);
                    cylinder.CreateHeightAttr().Set(2 * size[1]);
                    cylinder.CreateAxisAttr().Set(UsdGeomTokens->z);
                    setTransform(cylinder, pose);
                    gprim = cylinder;
                    break;
                }
                case ShapeType::Capsule: {
                    UsdGeomCapsule capsule = UsdGeomCapsule::Define(stage_, path);
                    capsule.CreateRadiusAttr().Set(size[0]);
                    capsule.CreateHeightAttr().Set(2 * size[1]);
                    capsule.CreateAxisAttr().Set(UsdGeomTokens->z);
                    setTransform(capsule, pose);
                    gprim = capsule;
                    break;
                }
                case ShapeType::ConvexHull: {
                    const ConvexHull& hull = m_.world.hulls.at(colliders.hull[c]);
                    UsdGeomMesh mesh = UsdGeomMesh::Define(stage_, path);
                    VtVec3fArray points;
                    for (const auto& p : hull.points) {
                        points.push_back(GfVec3f(static_cast<float>(p[0]), static_cast<float>(p[1]),
                                                 static_cast<float>(p[2])));
                    }
                    VtIntArray counts(hull.triangles.size(), 3);
                    VtIntArray indices;
                    for (const auto& t : hull.triangles) {
                        for (const std::uint32_t i : t) {
                            indices.push_back(static_cast<int>(i));
                        }
                    }
                    mesh.CreatePointsAttr().Set(points);
                    mesh.CreateFaceVertexCountsAttr().Set(counts);
                    mesh.CreateFaceVertexIndicesAttr().Set(indices);
                    setTransform(mesh, pose);
                    UsdPhysicsMeshCollisionAPI::Apply(mesh.GetPrim())
                        .CreateApproximationAttr()
                        .Set(UsdPhysicsTokens->convexHull);
                    gprim = mesh;
                    break;
                }
                case ShapeType::HeightField: {
                    // A grid mesh the reader turns back into a height field: its sample
                    // counts in lagrange:heightField:columns and rows, its points in row order.
                    const HeightField& field = m_.world.heightfields.at(colliders.hull[c]);
                    const ConvexHull surface = field.triangles();
                    UsdGeomMesh mesh = UsdGeomMesh::Define(stage_, path);
                    VtVec3fArray points;
                    for (const auto& p : surface.points) {
                        points.push_back(GfVec3f(static_cast<float>(p[0]), static_cast<float>(p[1]),
                                                 static_cast<float>(p[2])));
                    }
                    VtIntArray counts(surface.triangles.size(), 3);
                    VtIntArray indices;
                    for (const auto& t : surface.triangles) {
                        for (const std::uint32_t i : t) {
                            indices.push_back(static_cast<int>(i));
                        }
                    }
                    mesh.CreatePointsAttr().Set(points);
                    mesh.CreateFaceVertexCountsAttr().Set(counts);
                    mesh.CreateFaceVertexIndicesAttr().Set(indices);
                    setTransform(mesh, pose);
                    UsdPhysicsMeshCollisionAPI::Apply(mesh.GetPrim())
                        .CreateApproximationAttr()
                        .Set(UsdPhysicsTokens->none);
                    mesh.GetPrim()
                        .CreateAttribute(TfToken("lagrange:heightField:columns"),
                                         SdfValueTypeNames->Int, false, SdfVariabilityUniform)
                        .Set(static_cast<int>(field.columns));
                    mesh.GetPrim()
                        .CreateAttribute(TfToken("lagrange:heightField:rows"),
                                         SdfValueTypeNames->Int, false, SdfVariabilityUniform)
                        .Set(static_cast<int>(field.rows));
                    gprim = mesh;
                    break;
                }
                case ShapeType::Plane:
                    continue;  // only the ground is a plane
            }
            gprim.CreatePurposeAttr().Set(UsdGeomTokens->guide);
            UsdPhysicsCollisionAPI::Apply(gprim.GetPrim());
            bindFriction(gprim.GetPrim(), colliders.friction[c]);
        }
    }

    /// A UsdPreviewSurface for a look the file gave, shared by the meshes that have it. A material
    /// without one is left to its name, which is not written; nor is it needed by old samples.
    void bindLook(const UsdPrim& prim, const SceneMaterial& material) {
        if (!material.look) {
            return;
        }
        const std::string token = TfMakeValidIdentifier(material.name);
        const SdfPath path = SdfPath("/looks").AppendChild(TfToken(token));
        if (!stage_->GetPrimAtPath(path)) {
            UsdGeomScope::Define(stage_, SdfPath("/looks"));
            UsdShadeMaterial looks = UsdShadeMaterial::Define(stage_, path);
            UsdShadeShader shader =
                UsdShadeShader::Define(stage_, path.AppendChild(TfToken("surface")));
            shader.CreateIdAttr().Set(TfToken("UsdPreviewSurface"));
            shader.CreateInput(TfToken("diffuseColor"), SdfValueTypeNames->Color3f)
                .Set(GfVec3f(material.rgba[0], material.rgba[1], material.rgba[2]));
            shader.CreateInput(TfToken("metallic"), SdfValueTypeNames->Float)
                .Set(material.look->metallic);
            shader.CreateInput(TfToken("roughness"), SdfValueTypeNames->Float)
                .Set(material.look->roughness);
            shader.CreateInput(TfToken("emissiveColor"), SdfValueTypeNames->Color3f)
                .Set(GfVec3f(material.look->emissive[0], material.look->emissive[1],
                             material.look->emissive[2]));
            looks.CreateSurfaceOutput().ConnectToSource(
                shader.CreateOutput(TfToken("surface"), SdfValueTypeNames->Token));
        }
        UsdShadeMaterialBindingAPI::Apply(prim).Bind(UsdShadeMaterial::Get(stage_, path));
    }

    /// A physics material with the friction, shared by every collider that has it; none for the
    /// default.
    void bindFriction(const UsdPrim& prim, double friction) {
        if (std::abs(friction - ReadOptions{}.defaultFriction) < 1e-12) {
            return;
        }
        char name[48];
        std::snprintf(name, sizeof(name), "friction_%g", friction);
        std::string token = primName(name);
        const SdfPath path = SdfPath("/physics_materials").AppendChild(TfToken(token));
        if (!stage_->GetPrimAtPath(path)) {
            UsdGeomScope::Define(stage_, SdfPath("/physics_materials"));
            UsdShadeMaterial material = UsdShadeMaterial::Define(stage_, path);
            UsdPhysicsMaterialAPI api = UsdPhysicsMaterialAPI::Apply(material.GetPrim());
            api.CreateDynamicFrictionAttr().Set(static_cast<float>(friction));
            api.CreateStaticFrictionAttr().Set(static_cast<float>(friction));
        }
        UsdShadeMaterialBindingAPI::Apply(prim).Bind(UsdShadeMaterial::Get(stage_, path),
                                                     UsdShadeTokens->fallbackStrength,
                                                     TfToken("physics"));
    }

    Expected<void, std::string> writeJoints() {
        const World& world = m_.world;
        std::map<std::uint32_t, std::size_t> jointOfChild;
        for (std::size_t j = 0; j < world.joints.names.size(); ++j) {
            jointOfChild[world.joints.child[j].index] = j;
        }
        for (std::size_t b = 0; b < rigid_.size(); ++b) {
            if (!rigid_[b]) {
                continue;
            }
            const BodyId parent = world.bodies.parent[b];
            const auto found = jointOfChild.find(static_cast<std::uint32_t>(b));
            const bool hasJoint = found != jointOfChild.end();
            if (hasJoint && world.joints.type[found->second] == JointType::Free) {
                continue;  // a body joined to nothing is free when read back
            }
            const std::string name =
                hasJoint ? world.joints.names[found->second] : world.bodies.names[b] + "_weld";
            const SdfPath scope = jointScope(static_cast<std::uint32_t>(b));
            const SdfPath path = scope.AppendChild(TfToken(primName(name)));
            if (hasJoint) {
                jointPath_[found->second] = path;
            }
            const JointType type = hasJoint ? world.joints.type[found->second] : JointType::Fixed;
            const std::array<double, 3> axis =
                hasJoint ? world.joints.axis[found->second] : std::array<double, 3>{1, 0, 0};
            // UsdPhysics names the axis X, Y or Z of the joint frame: turn the frame so that
            // its X is the axis (the joint is at body1's origin).
            const std::array<double, 4> turn = rotationFromXTo(axis);
            UsdPhysicsJoint joint;
            switch (type) {
                case JointType::Revolute:
                case JointType::Continuous: {
                    UsdPhysicsRevoluteJoint revolute =
                        UsdPhysicsRevoluteJoint::Define(stage_, path);
                    revolute.CreateAxisAttr().Set(UsdPhysicsTokens->x);
                    const std::size_t j = found->second;
                    if (type == JointType::Revolute &&
                        world.joints.lowerLimit[j] < world.joints.upperLimit[j]) {
                        constexpr double kDegrees = 180.0 / std::numbers::pi;
                        revolute.CreateLowerLimitAttr().Set(
                            static_cast<float>(world.joints.lowerLimit[j] * kDegrees));
                        revolute.CreateUpperLimitAttr().Set(
                            static_cast<float>(world.joints.upperLimit[j] * kDegrees));
                    }
                    joint = revolute;
                    break;
                }
                case JointType::Prismatic: {
                    UsdPhysicsPrismaticJoint prismatic =
                        UsdPhysicsPrismaticJoint::Define(stage_, path);
                    prismatic.CreateAxisAttr().Set(UsdPhysicsTokens->x);
                    const std::size_t j = found->second;
                    if (world.joints.lowerLimit[j] < world.joints.upperLimit[j]) {
                        prismatic.CreateLowerLimitAttr().Set(
                            static_cast<float>(world.joints.lowerLimit[j]));
                        prismatic.CreateUpperLimitAttr().Set(
                            static_cast<float>(world.joints.upperLimit[j]));
                    }
                    joint = prismatic;
                    break;
                }
                case JointType::Spherical:
                    joint = UsdPhysicsSphericalJoint::Define(stage_, path);
                    break;
                case JointType::Fixed:
                    joint = UsdPhysicsFixedJoint::Define(stage_, path);
                    break;
                case JointType::Free:
                    continue;
            }
            if (parent != kWorldBody) {
                joint.CreateBody0Rel().SetTargets({paths_[primOfBody_[parent.index]]});
            }
            joint.CreateBody1Rel().SetTargets({paths_[primOfBody_[b]]});
            const Pose& inParent = world.bodies.poseInParent[b];
            joint.CreateLocalPos0Attr().Set(vec3f(inParent.position));
            joint.CreateLocalRot0Attr().Set(quatf(multiply(inParent.orientation, turn)));
            joint.CreateLocalPos1Attr().Set(GfVec3f(0.0F));
            joint.CreateLocalRot1Attr().Set(quatf(turn));
            if (hasJoint) {
                const std::size_t j = found->second;
                const World::Joints& table = world.joints;
                if (table.damping[j] > 0.0 || table.friction[j] > 0.0 ||
                    table.mimic[j] != kNoMimic) {
                    UsdPrim prim = joint.GetPrim();
                    prim.ApplyAPI(TfToken("JointDynamicsAPI_1"));
                    const auto real = [&prim](const char* name, double value) {
                        prim.CreateAttribute(TfToken(name), SdfValueTypeNames->Double).Set(value);
                    };
                    real("joint:damping", table.damping[j]);
                    real("joint:friction", table.friction[j]);
                    if (table.mimic[j] != kNoMimic) {
                        prim.CreateAttribute(TfToken("joint:mimic"), SdfValueTypeNames->String)
                            .Set(table.names[table.mimic[j]]);
                        real("joint:mimicMultiplier", table.mimicMultiplier[j]);
                        real("joint:mimicOffset", table.mimicOffset[j]);
                    }
                }
            }
            if (hasJoint && world.joints.actuated[found->second]) {
                const TfToken drive = type == JointType::Prismatic ? UsdPhysicsTokens->linear
                                                                   : UsdPhysicsTokens->angular;
                UsdPhysicsDriveAPI api = UsdPhysicsDriveAPI::Apply(joint.GetPrim(), drive);
                api.CreateTypeAttr().Set(UsdPhysicsTokens->force);
                const double effort = world.joints.effortLimit[found->second];
                if (effort > 0.0) {
                    api.CreateMaxForceAttr().Set(static_cast<float>(effort));
                }
            }
        }
        return {};
    }

    /// "<top prim>/joints" for the model a body belongs to.
    SdfPath jointScope(std::uint32_t body) {
        SdfPath top = paths_[primOfBody_[body]];
        while (top.GetParentPath() != SdfPath::AbsoluteRootPath()) {
            top = top.GetParentPath();
        }
        const SdfPath scope = top.AppendChild(TfToken("joints"));
        UsdGeomScope::Define(stage_, scope);
        return scope;
    }

    /// Where a world prim was written: its body's, joint's or sensor's prim, or the scene prim
    /// with its path (an overlay's motor or bus); otherwise a new Xform at its path.
    SdfPath pathOf(PrimId prim) {
        const World& w = m_.world;
        for (std::size_t b = 0; b < w.bodies.prim.size(); ++b) {
            if (w.bodies.prim[b] == prim && primOfBody_[b] != kNoIndex) {
                return paths_[primOfBody_[b]];
            }
        }
        for (const auto& [joint, path] : jointPath_) {
            if (w.joints.prim[joint] == prim) {
                return path;
            }
        }
        for (std::size_t s = 0; s < w.sensors.prim.size() && s < m_.sensorPrim.size(); ++s) {
            if (w.sensors.prim[s] == prim) {
                return paths_[m_.sensorPrim[s]];
            }
        }
        const std::string& text = w.prims.paths[prim.index];
        for (std::size_t p = 0; p < m_.scene.prims.size(); ++p) {
            if (m_.scene.prims[p].path == text && !paths_[p].IsEmpty()) {
                return paths_[p];
            }
        }
        const SdfPath path(text);
        if (!stage_->GetPrimAtPath(path)) {
            UsdGeomXform::Define(stage_, path);
        }
        return path;
    }

    /// Schema instances, apart from those the writer has applied itself (the sensors'). Only
    /// values that differ from the schema's fallback are authored.
    Expected<void, std::string> writeSchemas() {
        for (const SchemaInstance& s : m_.world.schemaInstances) {
            const SdfPath path = pathOf(s.prim);
            const UsdPrim prim = stage_->GetPrimAtPath(path);
            const TfToken name(s.instance.empty() ? s.schema : s.schema + ":" + s.instance);
            const TfTokenVector applied = prim.GetAppliedSchemas();
            if (std::find(applied.begin(), applied.end(), name) != applied.end()) {
                continue;
            }
            if (!prim.AddAppliedSchema(name) || !prim.HasAPI(TfToken(s.schema))) {
                return Unexpected{"cannot apply " + name.GetString() + " to " + path.GetString() +
                                  ": is lagrange_schemas on PXR_PLUGINPATH_NAME?"};
            }
            const auto set = [&](const std::string& key, const VtValue& value) {
                UsdAttribute attribute = prim.GetAttribute(TfToken(key));
                VtValue fallback;
                if (!attribute || (attribute.Get(&fallback) && fallback == value)) {
                    return;
                }
                attribute.Set(value);
            };
            const auto typeOf = [&](const std::string& key) {
                const UsdAttribute attribute = prim.GetAttribute(TfToken(key));
                return attribute ? attribute.GetTypeName() : SdfValueTypeName();
            };
            for (const auto& [key, value] : s.reals) {
                set(key, VtValue(value));
            }
            for (const auto& [key, value] : s.integers) {
                set(key, typeOf(key) == SdfValueTypeNames->Bool ? VtValue(value != 0)
                                                                : VtValue(static_cast<int>(value)));
            }
            for (const auto& [key, value] : s.tokens) {
                set(key, typeOf(key) == SdfValueTypeNames->String ? VtValue(value)
                                                                  : VtValue(TfToken(value)));
            }
            for (const auto& [key, value] : s.realArrays) {
                const SdfValueTypeName type = typeOf(key);
                if (type == SdfValueTypeNames->Double2 && value.size() == 2) {
                    set(key, VtValue(GfVec2d(value[0], value[1])));
                } else if (type == SdfValueTypeNames->Double3 && value.size() == 3) {
                    set(key, VtValue(GfVec3d(value[0], value[1], value[2])));
                } else {
                    set(key, VtValue(VtDoubleArray(value.begin(), value.end())));
                }
            }
            for (const auto& [key, value] : s.assetPaths) {
                // Relative to the written layer when both are in the same tree (the same first
                // directory: a source tree or an install space), so the files move together;
                // "./" for one below the layer.
                const std::filesystem::path asset(value);
                const std::filesystem::path relative = asset.lexically_relative(directory_);
                const auto top = [](const std::filesystem::path& p) {
                    auto it = p.begin();
                    return it != p.end() && ++it != p.end() ? *it : std::filesystem::path();
                };
                const bool sameTree = asset.is_absolute() && !relative.empty() &&
                                      !top(asset).empty() && top(asset) == top(directory_);
                const bool below = sameTree && *relative.begin() != "..";
                set(key, VtValue(SdfAssetPath(below      ? "./" + relative.string()
                                              : sameTree ? relative.string()
                                                         : value)));
            }
            for (const auto& [key, targets] : s.relationships) {
                SdfPathVector paths;
                for (const PrimId target : targets) {
                    paths.push_back(pathOf(target));
                }
                prim.CreateRelationship(TfToken(key)).SetTargets(paths);
            }
        }
        return {};
    }

    const Model& m_;
    UsdStageRefPtr stage_;
    std::filesystem::path directory_;
    std::string staticColliders_;               // WriteOptions::staticColliders
    std::map<std::size_t, SdfPath> jointPath_;  // world joint -> its prim
    std::vector<bool> rigid_;
    std::vector<std::uint32_t> primOfBody_;
    std::map<std::uint32_t, std::size_t> sensorOfPrim_;
    std::vector<SdfPath> paths_;
};

}  // namespace

Expected<void, std::string> writeStage(const Model& model, const std::string& path,
                                       const WriteOptions& options) {
    registerSchemaLibraries();
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    if (!stage) {
        return Unexpected{std::string("cannot create a USD stage")};
    }
    UsdGeomSetStageUpAxis(stage, UsdGeomTokens->z);
    UsdGeomSetStageMetersPerUnit(stage, 1.0);
    UsdPhysicsSetStageKilogramsPerUnit(stage, 1.0);
    if (!options.doc.empty()) {
        stage->GetRootLayer()->SetDocumentation(options.doc);
    }
    Writer writer(model, stage, std::filesystem::absolute(path).parent_path(),
                  options.staticColliders);
    if (auto written = writer.write(); !written) {
        return written;
    }
    if (!model.scene.prims.empty()) {
        stage->SetDefaultPrim(stage->GetPrimAtPath(
            SdfPath::AbsoluteRootPath().AppendChild(TfToken(primName(model.scene.prims[0].name)))));
    }
    if (!stage->GetRootLayer()->Export(path)) {
        return Unexpected{"cannot write " + path};
    }
    return {};
}

}  // namespace lagrange::stage
