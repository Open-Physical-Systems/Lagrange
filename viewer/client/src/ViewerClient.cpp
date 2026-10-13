#include "lagrange/proto/Connection.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <array>
#include <chrono>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <utility>

namespace lagrange::viewer {

namespace {

constexpr std::uint32_t kProtocolMajor = 0;
constexpr std::uint32_t kProtocolMinor = 1;
constexpr int kPollTimeoutMs = 100;

}  // namespace

const char* toString(ConnectionState state) {
    switch (state) {
        case ConnectionState::Disconnected:
            return "disconnected";
        case ConnectionState::Connecting:
            return "connecting";
        case ConnectionState::Handshaking:
            return "handshaking";
        case ConnectionState::Ready:
            return "connected";
        case ConnectionState::Refused:
            return "refused";
    }
    return "unknown";
}

struct ViewerClient::Worker {
    proto::Connection connection;
    proto::Backoff backoff;
    SceneBuilder builder;
    ConnectionInfo info;
    ServerStatus status;
    bool snapshotRequested{false};
    bool helloSent{false};
    std::chrono::steady_clock::time_point nextAttempt{};
};

ViewerClient::ViewerClient(proto::Endpoint endpoint)
    : endpoint_(std::move(endpoint))
    , wakeFd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC))
    , worker_(std::make_unique<Worker>()) {
    status_.store(std::make_shared<const ServerStatus>());
    connection_.store(std::make_shared<const ConnectionInfo>());
}

ViewerClient::~ViewerClient() {
    stop();
}

void ViewerClient::start() {
    if (!thread_.joinable()) {
        stopRequested_ = false;
        thread_ = std::thread([this] { run(); });
    }
}

void ViewerClient::stop() {
    stopRequested_ = true;
    wake();
    if (thread_.joinable()) {
        thread_.join();
    }
}

ClientStats ViewerClient::stats() const {
    return {snapshots_.load(), deltas_.load(), resyncs_.load(), connects_.load()};
}

void ViewerClient::wake() const {
    const std::uint64_t one = 1;
    // The counter cannot overflow in practice; a failed write only delays the wake-up.
    (void)!::write(wakeFd_.get(), &one, sizeof(one));
}

void ViewerClient::enqueue(std::unique_ptr<v1::Envelope> envelope) {
    {
        const std::lock_guard lock(commandMutex_);
        commands_.push_back(std::move(envelope));
    }
    wake();
}

void ViewerClient::play() {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    envelope->mutable_command()->mutable_play();
    enqueue(std::move(envelope));
}

void ViewerClient::pause() {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    envelope->mutable_command()->mutable_pause();
    enqueue(std::move(envelope));
}

void ViewerClient::seek(std::int64_t simTimeNs) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    envelope->mutable_command()->mutable_seek()->mutable_to()->set_ns(simTimeNs);
    enqueue(std::move(envelope));
}

void ViewerClient::step(std::int64_t durationNs) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    envelope->mutable_command()->mutable_step()->set_duration_ns(durationNs);
    enqueue(std::move(envelope));
}

void ViewerClient::setRealTimeFactor(double target) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    envelope->mutable_command()->mutable_set_rtf()->set_target(target);
    enqueue(std::move(envelope));
}

void ViewerClient::publish(ConnectionState state, const std::string& message) {
    // A refusal stays visible while the client keeps retrying in the background; only a
    // successful handshake (or another refusal) replaces it.
    if (worker_->info.state == ConnectionState::Refused && state != ConnectionState::Ready &&
        state != ConnectionState::Refused) {
        return;
    }
    worker_->info.state = state;
    if (!message.empty() || state == ConnectionState::Ready) {
        worker_->info.message = message;
    }
    connection_.store(std::make_shared<const ConnectionInfo>(worker_->info));
}

void ViewerClient::requestSnapshot() {
    if (worker_->snapshotRequested) {
        return;  // one request at a time; deltas until it arrives are dropped anyway
    }
    v1::Envelope envelope;
    envelope.mutable_command()->set_request_id(nextRequestId_++);
    envelope.mutable_command()->mutable_request_snapshot();
    worker_->snapshotRequested = worker_->connection.queue(envelope);
}

void ViewerClient::handle(const v1::Envelope& envelope) {
    Worker& w = *worker_;
    switch (envelope.body_case()) {
        case v1::Envelope::kServerHello: {
            const v1::ServerHello& hello = envelope.server_hello();
            if (!hello.refusal().empty()) {
                w.connection.close();
                publish(ConnectionState::Refused, hello.refusal());
                return;
            }
            if (hello.version().major() != kProtocolMajor) {
                w.connection.close();
                publish(ConnectionState::Refused,
                        "the server speaks protocol " + std::to_string(hello.version().major()) +
                            "." + std::to_string(hello.version().minor()) +
                            ", this viewer speaks " + std::to_string(kProtocolMajor) + "." +
                            std::to_string(kProtocolMinor));
                return;
            }
            w.info.serverName = hello.server_name();
            w.info.canSeek = hello.capabilities().can_seek();
            w.info.canStep = hello.capabilities().can_step();
            w.backoff.reset();
            publish(ConnectionState::Ready);
            requestSnapshot();
            break;
        }
        case v1::Envelope::kSnapshot: {
            w.snapshotRequested = false;
            std::string error;
            if (w.builder.applySnapshot(envelope.snapshot(), error)) {
                scene_.store(w.builder.frame());
                ++snapshots_;
            } else {
                w.info.message = "invalid scene from the server: " + error;
                connection_.store(std::make_shared<const ConnectionInfo>(w.info));
            }
            break;
        }
        case v1::Envelope::kDelta:
            if (w.builder.applyDelta(envelope.delta()) == SceneBuilder::DeltaResult::Applied) {
                scene_.store(w.builder.frame());
                ++deltas_;
            } else if (!w.snapshotRequested) {
                ++resyncs_;
                requestSnapshot();
            }
            break;
        case v1::Envelope::kStatus: {
            const v1::ServerStatus& s = envelope.status();
            w.status.simTimeNs = s.sim_time().ns();
            w.status.runState = static_cast<RunState>(s.state());
            w.status.realTimeFactor = s.real_time_factor();
            w.status.realTimeFactorTarget = s.real_time_factor_target();
            w.status.replayBeginNs = s.replay_begin().ns();
            w.status.replayEndNs = s.replay_end().ns();
            w.status.droppedDeltas = s.dropped_deltas();
            status_.store(std::make_shared<const ServerStatus>(w.status));
            break;
        }
        case v1::Envelope::kCatalog:
            w.status.signalCount = static_cast<std::uint32_t>(envelope.catalog().signals_size());
            status_.store(std::make_shared<const ServerStatus>(w.status));
            break;
        default:
            break;  // messages this viewer does not use yet, or newer than it knows
    }
}

void ViewerClient::run() {
    Worker& w = *worker_;
    while (!stopRequested_) {
        const auto now = std::chrono::steady_clock::now();

        if (w.connection.state() == proto::Connection::State::Closed && now >= w.nextAttempt) {
            w.snapshotRequested = false;
            w.helloSent = false;
            std::error_code ec;
            w.connection = proto::Connection::startConnect(endpoint_, ec);
            w.nextAttempt = now + w.backoff.next();
            if (w.connection.state() == proto::Connection::State::Closed) {
                if (w.info.state != ConnectionState::Refused) {
                    publish(ConnectionState::Disconnected, ec.message());
                }
            } else {
                publish(ConnectionState::Connecting);
            }
        }

        std::array<pollfd, 2> fds{};
        fds[0] = {wakeFd_.get(), POLLIN, 0};
        nfds_t count = 1;
        if (w.connection.state() != proto::Connection::State::Closed) {
            fds[1] = {w.connection.fd(), w.connection.pollEvents(), 0};
            count = 2;
        }
        ::poll(fds.data(), count, kPollTimeoutMs);
        if ((fds[0].revents & POLLIN) != 0) {
            std::uint64_t drained = 0;
            (void)!::read(wakeFd_.get(), &drained, sizeof(drained));
        }

        std::error_code ec;
        if (w.connection.state() == proto::Connection::State::Connecting &&
            w.connection.finishConnect(ec) == proto::IoStatus::Error) {
            publish(ConnectionState::Disconnected, ec.message());
        }
        // A Unix socket connects immediately, TCP after finishConnect: greet in both cases.
        if (w.connection.state() == proto::Connection::State::Connected && !w.helloSent) {
            ++connects_;
            v1::Envelope hello;
            hello.mutable_client_hello()->mutable_version()->set_major(kProtocolMajor);
            hello.mutable_client_hello()->mutable_version()->set_minor(kProtocolMinor);
            hello.mutable_client_hello()->set_client_name("lagrange_viewer");
            (void)w.connection.queue(hello);
            w.helloSent = true;
            publish(ConnectionState::Handshaking);
        }

        if (w.connection.state() == proto::Connection::State::Connected) {
            const proto::IoStatus received = w.connection.receive(ec);
            proto::FrameReader::Status last = proto::FrameReader::Status::NeedMore;
            for (;;) {
                const proto::FrameReader::Result frame = w.connection.next();
                last = frame.status;
                if (last != proto::FrameReader::Status::Frame) {
                    break;
                }
                v1::Envelope envelope;
                if (proto::parsePayload(frame.payload, envelope)) {
                    handle(envelope);
                }
                if (w.connection.state() != proto::Connection::State::Connected) {
                    break;  // handle() closed it (refusal)
                }
            }
            if (last == proto::FrameReader::Status::Error) {
                // The byte stream is not the protocol (oversize frame): drop the connection.
                w.connection.close();
                publish(ConnectionState::Disconnected, "invalid data from the server");
            }
            if (received == proto::IoStatus::Closed || received == proto::IoStatus::Error) {
                if (w.info.state != ConnectionState::Refused) {
                    publish(ConnectionState::Disconnected,
                            received == proto::IoStatus::Error
                                ? ec.message()
                                : "the server closed the connection");
                }
            }
        }

        if (w.connection.state() == proto::Connection::State::Connected) {
            if (w.info.state == ConnectionState::Ready) {
                std::deque<std::unique_ptr<v1::Envelope>> pending;
                {
                    const std::lock_guard lock(commandMutex_);
                    pending.swap(commands_);
                }
                for (const auto& command : pending) {
                    (void)w.connection.queue(*command);
                }
            }
            (void)w.connection.flush(ec);
        }
    }
    w.connection.close();
}

}  // namespace lagrange::viewer
