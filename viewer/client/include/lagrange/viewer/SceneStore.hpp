#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace lagrange::viewer::v1 {
class SceneSnapshot;
class SceneDelta;
}  // namespace lagrange::viewer::v1

namespace lagrange::viewer {

inline constexpr std::uint32_t kNone = 0xFFFFFFFFU;

/// Pose of one prim, absolute in the region-local frame. Positions are double (invariant 6);
/// they become float only relative to the camera, when sent to the GPU.
struct Transform {
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<float, 4> rotation{1.0F, 0.0F, 0.0F, 0.0F};  // w, x, y, z
    std::array<float, 3> scale{1.0F, 1.0F, 1.0F};
};

struct MeshData {
    std::vector<float> positions;  // xyz per vertex, mesh-local
    std::vector<float> normals;    // xyz per vertex; empty if the mesh has none
    std::vector<std::uint32_t> indices;
    std::array<float, 6> aabb{};
};

struct MaterialData {
    std::string name;
    std::array<float, 4> baseColor{0.8F, 0.8F, 0.8F, 1.0F};
    float metallic{0.0F};
    float roughness{0.5F};
};

struct PrimData {
    std::uint32_t parent{kNone};
    std::string name;
    std::string path;
    std::string typeName;
    std::vector<std::string> apiSchemas;
    std::uint32_t mesh{kNone};
    std::uint32_t material{kNone};
    bool visible{true};
};

/// A collision shape: a mesh that moves with a prim (kNone: fixed in the region-local frame),
/// at a pose in that prim's frame. Drawn by the Collision shading only.
struct ColliderData {
    std::uint32_t prim{kNone};
    std::uint32_t mesh{kNone};
    std::array<double, 3> position{0.0, 0.0, 0.0};
    std::array<float, 4> rotation{1.0F, 0.0F, 0.0F, 0.0F};  // w, x, y, z
};

/// One connection of a prim to an electrical net.
struct NetTerminal {
    std::uint32_t prim{kNone};
    std::string port;           ///< "dc_out" for a source, "dc_in" for a load.
    std::string currentSignal;  ///< Signal with the current through this terminal; may be empty.
    /// A terminal that feeds the net (a battery, a supply), by the port naming convention.
    [[nodiscard]] bool isSource() const { return port.ends_with("_out"); }
};

/// A bound a net's protection acts on: over- and undervoltage, a current rating.
struct NetLimit {
    std::string label;  ///< "OV limit"
    double value{0.0};
    std::string unit;    ///< Display form.
    bool upper{true};    ///< The signal must stay below the value (true) or above it.
    std::string signal;  ///< The signal compared with it; may be empty.
};

/// An electrical net: the prims connected to one bus. This is the topology the schematic is
/// laid out from; it comes from the stage, never from the viewer.
struct ElectricalNet {
    std::uint32_t id{0};
    std::string name;  ///< "dc_48v"
    std::uint32_t busPrim{kNone};
    std::string voltageSignal;  ///< Signal with the bus voltage; may be empty.
    std::vector<NetTerminal> terminals;
    std::vector<NetLimit> limits;
};

/// Everything a snapshot defines that deltas do not change. Immutable once published, so the
/// UI thread and the renderer can keep a pointer to it.
struct SceneStatics {
    std::vector<PrimData> prims;  // index == prim id
    std::vector<MeshData> meshes;
    std::vector<MaterialData> materials;
    std::vector<ElectricalNet> nets;
    std::vector<ColliderData> colliders;
    std::string stagePath;
    std::uint64_t generation{0};  // increases with every snapshot; GPU caches key on it
    /// The floating origin: world ENU of the region-local frame (m), and how often it moved.
    std::array<double, 3> regionEnu{};
    std::uint64_t rebaseCount{0};
};

/// One consistent state of the scene. Immutable.
struct SceneFrame {
    std::shared_ptr<const SceneStatics> statics;
    std::vector<Transform> transforms;  // index == prim id
    std::uint64_t sequence{0};
    std::int64_t simTimeNs{0};
};

/// Builds SceneFrames from protocol messages. Used by the client's worker thread only.
///
/// Messages come from the network: every size, index and reference is validated, and an
/// invalid message is rejected as a whole without changing the current frame.
class SceneBuilder {
public:
    enum class DeltaResult : std::uint8_t {
        Applied,
        OutOfSequence,  ///< A delta was lost. The caller must request a snapshot.
        Invalid,        ///< Malformed. The caller should request a snapshot.
    };

    /// Replaces the scene. Returns false and sets `error` if the snapshot is malformed.
    [[nodiscard]] bool applySnapshot(const v1::SceneSnapshot& snapshot, std::string& error);

    [[nodiscard]] DeltaResult applyDelta(const v1::SceneDelta& delta);

    /// The latest frame, or nullptr before the first snapshot.
    [[nodiscard]] std::shared_ptr<const SceneFrame> frame() const { return frame_; }

private:
    std::shared_ptr<const SceneFrame> frame_;
    std::uint64_t generation_{0};
};

}  // namespace lagrange::viewer
