#pragma once

#include "lagrange/proto/Connection.hpp"
#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/proto/UniqueFd.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ScopeBuffer.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace lagrange::viewer::v1 {
class Envelope;
class SensorFrame;
}  // namespace lagrange::viewer::v1

namespace lagrange::viewer {

enum class ConnectionState : std::uint8_t {
    Disconnected,  ///< Not connected; will retry.
    Connecting,
    Handshaking,
    Ready,
    Refused,  ///< The server refused the handshake; see ConnectionInfo::message.
};

[[nodiscard]] const char* toString(ConnectionState state);

struct ConnectionInfo {
    ConnectionState state{ConnectionState::Disconnected};
    std::string serverName;
    std::string message;  ///< Refusal reason or last connection error, for the status bar.
    bool canSeek{false};
    bool canStep{false};
};

enum class RunState : std::uint8_t { Unknown = 0, Stopped = 1, Paused = 2, Running = 3, Error = 4 };

struct ServerStatus {
    std::int64_t simTimeNs{0};
    RunState runState{RunState::Unknown};
    double realTimeFactor{0.0};
    double realTimeFactorTarget{1.0};
    std::int64_t replayBeginNs{0};
    std::int64_t replayEndNs{0};
    std::uint64_t droppedDeltas{0};
    std::uint32_t signalCount{0};
    std::string recordingPath;  ///< The file the server records the run to; empty: not recording.
};

/// One signal the server offers.
struct SignalInfo {
    std::uint32_t id{0};
    std::string name;    ///< "motor/left_drive_motor/current_q"
    std::string unit;    ///< Display form of the unit, e.g. "A", "N·m".
    std::string writer;  ///< Module that produces it.
    std::uint32_t components{1};
    std::vector<std::string> componentNames;  ///< Empty for scalars.
    std::int64_t dtNs{0};                     ///< Native sample period.
    std::uint32_t prim{kNone};
};

/// The server's list of signals. Immutable once published.
struct SignalCatalog {
    std::uint64_t generation{0};
    std::vector<SignalInfo> signals;

    [[nodiscard]] const SignalInfo* find(const std::string& name) const {
        for (const SignalInfo& info : signals) {
            if (info.name == name) {
                return &info;
            }
        }
        return nullptr;
    }
};

/// The latest thing worth telling the user: a command that failed, or a file that loaded.
struct Notice {
    std::uint64_t serial{0};  ///< Increases with every notice, so the UI can spot a new one.
    bool error{false};
    std::string text;
};

enum class LogLevel : std::uint8_t { Debug = 1, Info = 2, Warn = 3, Error = 4 };

/// One line of the server's log, for the console.
struct LogEntry {
    std::int64_t simTimeNs{0};
    LogLevel level{LogLevel::Info};
    std::string source;  ///< "stage", "electrical/averaged_network"
    std::string text;
};

/// The most recent log lines, oldest first. Immutable once published.
struct LogBook {
    static constexpr std::size_t kCapacity = 500;
    std::uint64_t total{0};  ///< Lines received since the connection was made.
    std::uint32_t warnings{0};
    std::uint32_t errors{0};  ///< Counts over the lines still held.
    std::vector<LogEntry> entries;
};

/// One attribute of a prim, as the inspector shows it.
struct AttributeInfo {
    enum class Kind : std::uint8_t { Real, Integer, Boolean, Token, Asset, Reals, Targets };
    std::string name;   ///< "elec:windingResistance"
    std::string value;  ///< Display text of the value, without the unit.
    std::string unit;   ///< Display form of the unit; empty when it has none.
    std::string doc;
    Kind kind{Kind::Token};
    bool editable{false};
    std::vector<std::uint32_t> targets;  ///< Prims a relationship points at.
    std::vector<double> numbers;         ///< The value of a real, integer or array, in full.

    /// "elec" for "elec:windingResistance"; empty when the name has no namespace.
    [[nodiscard]] std::string_view group() const {
        const std::size_t colon = name.find(':');
        return colon == std::string::npos ? std::string_view{}
                                          : std::string_view(name).substr(0, colon);
    }
};

/// The server's answer to requestAttributes(). Immutable once published.
struct PrimAttributes {
    std::uint64_t requestId{0};  ///< What requestAttributes() returned.
    std::uint32_t prim{kNone};
    std::vector<AttributeInfo> attributes;
};

/// One revolution of a lidar, or one frame of any sensor that returns points. Immutable.
struct PointCloud {
    std::int64_t simTimeNs{0};
    std::uint32_t count{0};
    std::vector<float> points;  ///< count x (x, y, z, intensity), in the sensor's frame.
};

/// One image of a camera, as RGBA8 rows from the top. Immutable.
struct SensorImage {
    std::int64_t simTimeNs{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<std::uint8_t> rgba;  ///< width x height x 4
};

/// The frames of one sensor the viewer watches. Written by the client's worker, read by the UI.
class SensorStream {
public:
    explicit SensorStream(std::string name) : name_(std::move(name)) {}
    [[nodiscard]] const std::string& name() const { return name_; }  ///< The sensor's prim path.
    /// The newest frame, or nullptr before the first one.
    [[nodiscard]] std::shared_ptr<const PointCloud> latest() const { return latest_.load(); }
    /// The newest image (cameras), or nullptr before the first one.
    [[nodiscard]] std::shared_ptr<const SensorImage> latestImage() const {
        return latestImage_.load();
    }
    /// Frames received since the stream was opened.
    [[nodiscard]] std::uint64_t frames() const { return frames_.load(); }
    /// Frames rejected because they were malformed.
    [[nodiscard]] std::uint64_t rejected() const { return rejected_.load(); }

private:
    friend class ViewerClient;
    std::string name_;
    std::atomic<std::shared_ptr<const PointCloud>> latest_;
    std::atomic<std::shared_ptr<const SensorImage>> latestImage_;
    std::atomic<std::uint64_t> frames_{0};
    std::atomic<std::uint64_t> rejected_{0};
};

/// Something that happened in a run at a time or over a span.
struct EventInfo {
    enum class Severity : std::uint8_t { Info = 1, Warning = 2, Error = 3 };
    std::uint64_t id{0};
    std::int64_t beginNs{0};
    std::int64_t endNs{0};  ///< Equal to beginNs for an instant.
    std::string domain;     ///< The lane it is drawn in.
    std::string kind;       ///< "regen", "slip": stable, for filtering.
    Severity severity{Severity::Info};
    std::string text;
    std::uint32_t prim{kNone};
    std::string signal;  ///< The signal that shows it; may be empty.
};

/// The events of the run so far, ordered by begin time. Immutable once published.
struct EventLog {
    static constexpr std::size_t kCapacity = 20'000;
    std::vector<EventInfo> events;
    bool complete{false};  ///< A recording: no further events will come.
};

/// What the server costs to run, as it last reported. Immutable once published.
struct ServerStatistics {
    struct Module {
        std::string module;  ///< "rigid_body/mujoco"
        std::string domain;
        std::int64_t dtNs{0};
        double stepsPerSecond{0.0};
        double cpuMsPerStep{0.0};
        double cpuMsPerStepMax{0.0};
        bool onGpu{false};
    };
    std::uint64_t serial{0};  ///< Increases with every report.
    std::int64_t simTimeNs{0};
    std::vector<Module> modules;
    double realTimeFactor{0.0};
    double tickMs{0.0};
    double tickMsMax{0.0};
    std::uint64_t memoryBytes{0};
    std::uint64_t gpuMemoryBytes{0};
    double bytesPerSecondOut{0.0};
    std::uint32_t viewers{0};
    bool estimated{false};  ///< Module costs are estimates, not measurements.
};

/// The recording a replay reads. Immutable once published.
struct RecordingFacts {
    struct Channel {
        std::string topic;
        double rateHz{0.0};
        std::uint64_t messages{0};
        std::string unit;
    };
    std::string path;  ///< Empty for a generated run.
    std::int64_t durationNs{0};
    std::uint64_t sizeBytes{0};
    std::string compression;
    std::vector<Channel> channels;
    std::uint64_t messages{0};
    std::int64_t recordedUnixNs{0};
    bool synthetic{false};
    std::string generator;
    std::string description;
};

/// The modules and substitutes a profile would connect, and the signals between them.
struct WiringInfo {
    struct Node {
        std::string module;
        std::string domain;
        bool substitute{false};
        std::string detail;  ///< Backend, fidelity, or the substitute's name.
        std::int64_t dtNs{0};
        std::vector<std::string> inputs;
        std::vector<std::string> outputs;
        bool onGpu{false};
    };
    struct Wire {
        std::string from;
        std::string to;
        std::string signal;
        std::string symbol;
        std::string sourceUnit;
        std::string sinkUnit;
        bool unitOk{true};
        bool stiff{false};
    };
    std::vector<Node> nodes;
    std::vector<Wire> wires;
    std::int64_t baseTickNs{0};
    std::int64_t syncNs{0};
    [[nodiscard]] bool empty() const { return nodes.empty(); }
};

/// The environment the simulation uses, and the fields it can slice. Immutable once published.
struct EnvironmentInfo {
    double gravity{0.0};      ///< m/s², downward
    double windFromDeg{0.0};  ///< compass: where the wind comes from
    struct EmiSource {
        std::string name;
        std::uint32_t prim{kNone};
        double frequencyHz{0.0};
        double currentA{0.0};
        double powerW{0.0};
        std::string detail;
    };
    struct Field {
        std::string name;   ///< "emi/flux_density"
        std::string label;  ///< "|B|"
        std::string domain;
        std::string unit;
        std::uint32_t components{1};
        bool available{false};
        std::string unavailableReason;
    };
    std::string layer;
    double localTimeHours{0.0};
    double sunElevationDeg{0.0};
    double ambientTemperatureK{0.0};
    double rainMmPerH{0.0};
    double fogVisibilityM{0.0};
    double dustMgPerM3{0.0};
    double surfaceWetness{0.0};
    double windSpeed{0.0};
    double windDirectionDeg{0.0};
    double gustIntensity{0.0};
    std::string windField;
    std::vector<EmiSource> emiSources;
    std::vector<Field> fields;
    bool synthetic{false};
};

/// Which plane a slice lies in (protocol SliceOrientation).
enum class SliceOrientation : int { Horizontal = 0, VerticalXZ = 1, VerticalYZ = 2 };

/// One slice of a field. Immutable. Horizontal: cells along +X and +Y at z = heightM. Vertical:
/// columns along +X (VerticalYZ: +Y), rows along +Z from originY, the plane at heightM on the
/// other horizontal axis.
struct FieldSliceData {
    std::int64_t simTimeNs{0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t components{1};
    double originX{0.0};  ///< Region frame, centre of cell (0, 0).
    double originY{0.0};
    double cellM{0.0};
    double heightM{0.0};
    float valueMin{0.0F};
    float valueMax{0.0F};
    std::vector<float> values;  ///< Row-major from cell (0, 0), `components` per cell.
    SliceOrientation orientation{SliceOrientation::Horizontal};

    /// The magnitude at a point of the slice's plane (its two axes), by the nearest cell. NaN
    /// outside.
    [[nodiscard]] float sample(double x, double y) const;
};

/// A box of a vector field around the robot (protocol FieldVolume), for moving particles.
/// Immutable.
struct FieldVolumeData {
    std::int64_t simTimeNs{0};
    std::uint32_t nx{0}, ny{0}, nz{0};
    std::array<double, 3> origin{};  ///< Region frame, centre of cell (0, 0, 0).
    double cellM{0.0};
    float valueMax{0.0F};
    std::vector<float> values;  ///< x fastest, then y, then z; three per cell.

    /// The vector at a point of the region frame, trilinear; false outside the box.
    bool sample(const std::array<double, 3>& at, std::array<float, 3>& out) const;
};

/// The slices of one field the viewer watches.
class FieldStream {
public:
    explicit FieldStream(std::string name,
                         SliceOrientation orientation = SliceOrientation::Horizontal)
        : name_(std::move(name)), orientation_(orientation) {}
    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] std::shared_ptr<const FieldSliceData> latest() const { return latest_.load(); }
    /// The latest box, when the stream was asked for volumes.
    [[nodiscard]] std::shared_ptr<const FieldVolumeData> latestVolume() const {
        return volume_.load();
    }
    [[nodiscard]] SliceOrientation orientation() const { return orientation_; }
    [[nodiscard]] std::uint64_t slices() const { return slices_.load(); }
    [[nodiscard]] std::uint64_t rejected() const { return rejected_.load(); }

private:
    friend class ViewerClient;
    std::string name_;
    SliceOrientation orientation_{SliceOrientation::Horizontal};
    std::atomic<std::shared_ptr<const FieldSliceData>> latest_;
    std::atomic<std::shared_ptr<const FieldVolumeData>> volume_;
    std::atomic<std::uint64_t> slices_{0};
    std::atomic<std::uint64_t> rejected_{0};
};

/// The ROS 2 graph as the simulator's bridge reports it. Immutable once published.
struct RosGraphInfo {
    struct Topic {
        std::string name;
        std::string type;
        std::uint32_t publishers{0};
        std::uint32_t subscribers{0};
        double rateHz{0.0};
        double bandwidthBps{0.0};
        bool fromBridge{false};
        std::string status;
    };
    struct Node {
        std::string name;
        std::vector<std::string> publishes;
        std::vector<std::string> subscribes;
        bool external{false};
        std::string detail;
    };
    struct Joint {
        std::string name;
        std::string interface;
        double command{0.0};
        double state{0.0};
        std::string unit;
    };
    struct Controller {
        std::string name;
        double rateHz{0.0};
        std::string state;
    };
    std::vector<Topic> topics;
    std::vector<Node> nodes;
    std::string rmw;
    std::string distro;
    std::uint32_t domainId{0};
    bool useSimTime{false};
    std::string note;  ///< Not empty when no real bridge is behind this graph.
    bool hasControl{false};
    std::string hardware;
    std::string plugin;
    double controlRateHz{0.0};
    bool controlActive{false};
    std::vector<Joint> joints;
    std::vector<Controller> controllers;
};

/// A large world: tiles, agents, floating origin, streaming. Immutable once published.
struct WorldInfo {
    struct Tile {
        std::int32_t x{0};
        std::int32_t y{0};
        std::uint8_t lod{0};  ///< 0 physics and sensors .. 3 proxy.
    };
    struct Agent {
        std::string name;
        std::uint32_t prim{kNone};
        std::string kind;
        double x{0.0};  ///< World ENU, metres.
        double y{0.0};
        double speed{0.0};
        std::string status;
        double activeRadiusM{0.0};
        bool focus{false};
    };
    struct Region {
        std::string name;
        double minX{0.0};
        double minY{0.0};
        double maxX{0.0};
        double maxY{0.0};
    };
    static constexpr std::size_t kMaxTiles = 1'000'000;
    std::int64_t simTimeNs{0};
    bool georeferenced{false};
    double latitudeDeg{0.0};
    double longitudeDeg{0.0};
    double heightM{0.0};
    double tileSizeM{0.0};
    std::uint64_t tilesGeneration{0};
    std::vector<Tile> tiles;
    std::vector<Agent> agents;
    std::string anchor;
    std::array<double, 3> origin{};
    double recenterThresholdM{0.0};
    std::int64_t lastRecenterNs{0};
    std::uint64_t recenters{0};
    std::uint64_t gpuMemoryBytes{0};
    std::uint64_t gpuMemoryBudgetBytes{0};
    std::uint64_t hostCacheBytes{0};
    std::uint64_t hostCacheBudgetBytes{0};
    double diskReadBytesPerS{0.0};
    std::uint32_t loadQueue{0};
    std::vector<Region> regions;
    bool synthetic{false};
};

/// One thing that can stand in for a disabled domain.
struct SubstituteOption {
    std::string token;  ///< What the profile says: "zero", "replay", ...
    std::string kind;   ///< ideal | constant | replay
    std::string description;
    bool needsSource{false};  ///< A replay needs a recording to read from.
};

/// One domain as the server sees it under the applied profile.
struct DomainInfo {
    std::string name;  ///< "rigid_body"
    bool enabled{false};
    bool available{true};
    std::string unavailableReason;
    std::string activeModule;  ///< Backend, fidelity, or the substitute in use.
    bool substituted{false};
    std::vector<SubstituteOption> substitutes;
    std::vector<std::string> backends;
    std::vector<std::string> fidelities;
    std::int64_t dtNs{0};
    std::int64_t minDtNs{0};
    std::int64_t maxDtNs{0};
    double cpuMsPerSimS{0.0};
    std::vector<std::string> dependsOn;
};

/// The profile the server is running. Immutable once published.
struct ProfileInfo {
    std::uint64_t serial{0};  ///< Increases every time the server sends its profile.
    std::string yaml;         ///< Canonical text: one domain per line.
    std::uint32_t schemaVersion{0};
    bool restartRequired{false};
    std::vector<DomainInfo> domains;

    [[nodiscard]] const DomainInfo* find(std::string_view name) const {
        for (const DomainInfo& domain : domains) {
            if (domain.name == name) {
                return &domain;
            }
        }
        return nullptr;
    }
};

struct ProfileIssue {
    bool error{true};
    std::string pointer;  ///< JSON pointer into the profile: "/domains/thermal/dt"
    std::string message;
};

/// The server's answer to validateProfile().
struct ProfileCheck {
    std::uint64_t requestId{0};  ///< What validateProfile() returned for this text.
    bool valid{false};
    std::vector<ProfileIssue> issues;
    WiringInfo wiring;  ///< Empty when the text could not be read as a profile.
};

struct ClientStats {
    std::uint64_t snapshots{0};
    std::uint64_t deltas{0};
    std::uint64_t resyncs{0};  ///< Snapshots requested because a delta was lost or invalid.
    std::uint64_t connects{0};
    std::uint64_t signalBlocks{0};
    std::uint64_t samples{0};         ///< Samples stored into scope buffers.
    std::uint64_t invalidRuns{0};     ///< Signal runs rejected as malformed.
    std::uint64_t droppedSamples{0};  ///< Samples missing between runs, over all buffers.
};

/// The viewer's only link to a simulation (invariant 14).
///
/// A worker thread owns the socket: it connects, reconnects with a backoff, decodes messages
/// and builds scene frames. The UI thread only reads the latest published state and enqueues
/// commands; it never waits on the network.
class ViewerClient {
public:
    explicit ViewerClient(proto::Endpoint endpoint);
    ~ViewerClient();
    /// Before start(): the CA file that verifies a tls:// server (empty: the system's trust
    /// store) and the token a TCP server may require. Both default to $LAGRANGE_TLS_CA and
    /// $LAGRANGE_AUTH_TOKEN.
    void setSecurity(std::string caFile, std::string authToken);
    ViewerClient(const ViewerClient&) = delete;
    ViewerClient& operator=(const ViewerClient&) = delete;

    void start();
    void stop();
    /// True between start() and stop().
    [[nodiscard]] bool running() const { return thread_.joinable(); }
    /// Chooses the server to talk to. Only while the client is not running; ignored otherwise.
    void setEndpoint(proto::Endpoint endpoint);

    // --- latest state; safe from any thread, never blocks on I/O ---------------------------
    [[nodiscard]] std::shared_ptr<const SceneFrame> scene() const { return scene_.load(); }
    [[nodiscard]] ServerStatus status() const { return *status_.load(); }
    [[nodiscard]] ConnectionInfo connection() const { return *connection_.load(); }
    [[nodiscard]] ClientStats stats() const;
    /// The server's signal list, or nullptr before it arrived.
    [[nodiscard]] std::shared_ptr<const SignalCatalog> catalog() const { return catalog_.load(); }

    // --- signals ---------------------------------------------------------------------------
    /// Starts receiving one component of a signal into a new buffer. Subscriptions are by
    /// name, so they survive reconnects and catalog changes. Returns nullptr if the catalog
    /// has no such signal or component. Call from the UI thread.
    [[nodiscard]] std::shared_ptr<ScopeBuffer> subscribe(const std::string& name,
                                                         std::uint32_t component = 0);
    /// Stops feeding `buffer`. The buffer keeps its history.
    void unsubscribe(const std::shared_ptr<ScopeBuffer>& buffer);
    [[nodiscard]] const proto::Endpoint& endpoint() const { return endpoint_; }

    // --- commands; enqueue and return ------------------------------------------------------
    void play();
    void pause();
    void seek(std::int64_t simTimeNs);
    /// Restarts a live run at t = 0 (time, poses and velocities), as /reset_simulation does.
    void reset();
    /// Starts recording the run to an MCAP file on the server (`path` empty: the server picks
    /// one), or stops it.
    void setRecording(bool on, const std::string& path = {});
    void step(std::int64_t durationNs);
    void setRealTimeFactor(double target);
    /// Asks the server to load a file that it can read: a robot description (.urdf, .xacro)
    /// or a recording (.mcap). The result arrives as a Notice and, on success, a new scene.
    void openFile(const std::string& path);

    [[nodiscard]] Notice notice() const { return *notice_.load(); }

    // --- console and inspector --------------------------------------------------------------
    /// The server's recent log lines. Never null.
    [[nodiscard]] std::shared_ptr<const LogBook> logs() const { return logs_.load(); }
    /// Asks for the attributes of one prim. Returns the request id.
    std::uint64_t requestAttributes(std::uint32_t prim);
    /// Asks for the attributes of many prims at once. The answer goes to attributesOf() only, so
    /// it does not disturb what the inspector waits for.
    void requestAttributesOf(const std::vector<std::uint32_t>& prims);
    /// Every prim's latest attributes from any request; never null. Replaced as a whole when an
    /// answer arrives, so a pointer that compares equal has the same contents.
    [[nodiscard]] std::shared_ptr<const std::map<std::uint32_t, PrimAttributes>> attributesOf()
        const {
        return attributesByPrim_.load();
    }
    /// The latest attributes answer, or nullptr.
    [[nodiscard]] std::shared_ptr<const PrimAttributes> attributes() const {
        return attributes_.load();
    }

    // --- the run -------------------------------------------------------------------------
    /// Events of the run. Never null; empty until the server sends some.
    [[nodiscard]] std::shared_ptr<const EventLog> events() const { return events_.load(); }
    /// The server's latest statistics, or nullptr before the first report.
    [[nodiscard]] std::shared_ptr<const ServerStatistics> statistics() const {
        return statistics_.load();
    }
    /// The recording being replayed, or nullptr when the server did not describe one.
    [[nodiscard]] std::shared_ptr<const RecordingFacts> recording() const {
        return recording_.load();
    }

    // --- environment, ROS 2, world ----------------------------------------------------------
    /// The environment, or nullptr before the server described it.
    [[nodiscard]] std::shared_ptr<const EnvironmentInfo> environment() const {
        return environment_.load();
    }
    /// Starts receiving slices of a field at a height (horizontal), or through the robot
    /// (vertical), and with `volume` boxes of it around the robot too. Asking again for the same
    /// field and orientation returns the same stream (and moves the slice to the new height).
    [[nodiscard]] std::shared_ptr<FieldStream> watchField(
        const std::string& name, double heightM,
        SliceOrientation orientation = SliceOrientation::Horizontal, bool volume = false);
    /// Asks the server to send (or stop sending) its ROS graph and its world state. Both are
    /// off until a panel that shows them asks. Remembered across reconnects.
    void setRosGraphWanted(bool wanted);
    void setWorldWanted(bool wanted);
    /// Where the camera is (region-local, m), for a streamed world's tile selection. Cheap to
    /// call every frame: sent only when it moved 10 m or more and at most twice a second.
    void setCameraInterest(const std::array<double, 3>& eye);
    /// The latest of each, or nullptr.
    [[nodiscard]] std::shared_ptr<const RosGraphInfo> rosGraph() const { return rosGraph_.load(); }
    [[nodiscard]] std::shared_ptr<const WorldInfo> world() const { return world_.load(); }

    // --- sensors ---------------------------------------------------------------------------
    /// Starts receiving the frames of a sensor, named by its prim path. Asking again for the
    /// same sensor returns the same stream. The subscription survives reconnects.
    [[nodiscard]] std::shared_ptr<SensorStream> watchSensor(const std::string& name);

    // --- profile ---------------------------------------------------------------------------
    /// The profile the server runs, or nullptr before it arrived.
    [[nodiscard]] std::shared_ptr<const ProfileInfo> profile() const { return profile_.load(); }
    /// The latest validation answer, or nullptr. Compare its requestId with the id returned
    /// by validateProfile() to know which text it is about.
    [[nodiscard]] std::shared_ptr<const ProfileCheck> profileCheck() const {
        return profileCheck_.load();
    }
    /// Asks the server whether `yaml` is a valid profile. Returns the request id.
    std::uint64_t validateProfile(const std::string& yaml);
    /// Asks the server to adopt `yaml` and restart the run at t = 0. A refusal arrives as a
    /// Notice; success arrives as a new profile().
    void applyProfile(const std::string& yaml);
    /// Sets a number of a prim's schema in the running simulation (SetAttribute). A refusal
    /// arrives as an error notice; `save` also writes the server's session layer.
    void setAttribute(std::uint32_t prim, const std::string& name, double value, bool save);

private:
    void run();
    void handle(const v1::Envelope& envelope);
    void enqueue(std::unique_ptr<v1::Envelope> envelope);
    void handleSignalBlock(const v1::Envelope& envelope);
    void sendSubscriptions();
    void requestSnapshot();
    void publish(ConnectionState state, const std::string& message = {});
    void wake() const;

    struct Worker;  // socket and builder; lives on the worker thread

    proto::Endpoint endpoint_;
    std::string tlsCaFile_;
    std::string authToken_;
    std::shared_ptr<proto::TlsContext> tls_;  // tls:// only; made on the first connect
    std::thread thread_;
    std::atomic<bool> stopRequested_{false};
    proto::UniqueFd wakeFd_;  // eventfd: commands were queued or stop was requested
    std::unique_ptr<Worker> worker_;

    std::atomic<std::shared_ptr<const SceneFrame>> scene_;
    std::atomic<std::shared_ptr<const ServerStatus>> status_;
    std::atomic<std::shared_ptr<const ConnectionInfo>> connection_;
    std::atomic<std::uint64_t> snapshots_{0};
    std::atomic<std::uint64_t> deltas_{0};
    std::atomic<std::uint64_t> resyncs_{0};
    std::atomic<std::uint64_t> connects_{0};

    std::atomic<std::shared_ptr<const SignalCatalog>> catalog_;
    std::atomic<std::shared_ptr<const Notice>> notice_;
    std::atomic<std::shared_ptr<const ProfileInfo>> profile_;
    std::atomic<std::shared_ptr<const LogBook>> logs_;
    std::atomic<std::shared_ptr<const EventLog>> events_;
    std::atomic<std::shared_ptr<const ServerStatistics>> statistics_;
    std::atomic<std::shared_ptr<const RecordingFacts>> recording_;
    std::atomic<std::uint64_t> statisticsSerial_{0};
    std::atomic<std::shared_ptr<const PrimAttributes>> attributes_;
    std::atomic<std::shared_ptr<const std::map<std::uint32_t, PrimAttributes>>> attributesByPrim_{
        std::make_shared<const std::map<std::uint32_t, PrimAttributes>>()};
    std::atomic<std::uint64_t> manyAttributesRequest_{0};
    std::atomic<std::shared_ptr<const ProfileCheck>> profileCheck_;
    std::atomic<std::uint64_t> profileSerial_{0};
    void post(bool error, std::string text);
    std::atomic<std::uint64_t> signalBlocks_{0};
    std::atomic<std::uint64_t> samples_{0};
    std::atomic<std::uint64_t> invalidRuns_{0};
    std::atomic<std::uint64_t> droppedSamples_{0};

    /// A change to the set of subscriptions, handed from the UI thread to the worker.
    struct SubscriptionChange {
        bool add{true};
        std::string name;
        std::uint32_t component{0};
        std::shared_ptr<ScopeBuffer> buffer;
    };

    void subscribeSensors();  // (re)sends the sensor subscriptions; worker thread
    void handleFieldSlice(const v1::Envelope& envelope);
    void handleFieldVolume(const v1::Envelope& envelope);
    void sendFieldSubscription(const std::string& name, double heightM,
                               SliceOrientation orientation, bool volume);
    void sendViewRequests();  // ROS graph and world, as wanted
    struct WatchedField {
        std::shared_ptr<FieldStream> stream;
        double heightM{0.0};
        bool volume{false};
    };
    std::vector<WatchedField> fields_;  // guarded by sensorMutex_
    std::atomic<bool> rosGraphWanted_{false};
    std::atomic<bool> worldWanted_{false};
    std::array<double, 3> sentEye_{1e300, 1e300, 1e300};  // UI thread only
    std::chrono::steady_clock::time_point sentEyeAt_{};
    std::atomic<std::shared_ptr<const EnvironmentInfo>> environment_;
    std::atomic<std::shared_ptr<const RosGraphInfo>> rosGraph_;
    std::atomic<std::shared_ptr<const WorldInfo>> world_;
    void handleSensorFrame(const v1::Envelope& envelope);
    void handleImage(const v1::SensorFrame& frame, SensorStream& stream);
    std::mutex sensorMutex_;  // guards the list below; held only to look a stream up
    std::vector<std::shared_ptr<SensorStream>> sensors_;

    std::mutex commandMutex_;  // held only to push or swap a small queue
    std::deque<std::unique_ptr<v1::Envelope>> commands_;
    std::deque<SubscriptionChange> subscriptionChanges_;
    std::atomic<std::uint64_t> nextRequestId_{1};
};

}  // namespace lagrange::viewer
