#pragma once

#include "lagrange/proto/Endpoint.hpp"
#include "lagrange/proto/UniqueFd.hpp"
#include "lagrange/viewer/SceneStore.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

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

struct ClientStats {
    std::uint64_t snapshots{0};
    std::uint64_t deltas{0};
    std::uint64_t resyncs{0};  ///< Snapshots requested because a delta was lost or invalid.
    std::uint64_t connects{0};
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

    // --- latest state; safe from any thread, never blocks on I/O ---------------------------
    [[nodiscard]] std::shared_ptr<const SceneFrame> scene() const { return scene_.load(); }
    [[nodiscard]] ServerStatus status() const { return *status_.load(); }
    [[nodiscard]] ConnectionInfo connection() const { return *connection_.load(); }
    [[nodiscard]] ClientStats stats() const;
    [[nodiscard]] const proto::Endpoint& endpoint() const { return endpoint_; }

    // --- commands; enqueue and return ------------------------------------------------------
    void play();
    void pause();
    void seek(std::int64_t simTimeNs);
    void step(std::int64_t durationNs);
    void setRealTimeFactor(double target);

private:
    void run();
    void handle(const v1::Envelope& envelope);
    void enqueue(std::unique_ptr<v1::Envelope> envelope);
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

    std::mutex commandMutex_;  // held only to push or swap a small queue
    std::deque<std::unique_ptr<v1::Envelope>> commands_;
    std::atomic<std::uint64_t> nextRequestId_{1};
};

}  // namespace lagrange::viewer
