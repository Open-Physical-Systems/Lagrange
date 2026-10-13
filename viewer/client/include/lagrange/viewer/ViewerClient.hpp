#pragma once

#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/proto/UniqueFd.hpp"
#include "lagrange/viewer/SceneStore.hpp"
#include "lagrange/viewer/ScopeBuffer.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace lagrange::viewer::v1 {
class Envelope;
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
    /// The latest attributes answer, or nullptr.
    [[nodiscard]] std::shared_ptr<const PrimAttributes> attributes() const {
        return attributes_.load();
    }

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
    std::atomic<std::shared_ptr<const PrimAttributes>> attributes_;
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

    std::mutex commandMutex_;  // held only to push or swap a small queue
    std::deque<std::unique_ptr<v1::Envelope>> commands_;
    std::deque<SubscriptionChange> subscriptionChanges_;
    std::atomic<std::uint64_t> nextRequestId_{1};
};

}  // namespace lagrange::viewer
