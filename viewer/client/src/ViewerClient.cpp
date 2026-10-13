#include "lagrange/proto/Connection.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <unordered_map>
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

    // Subscriptions by signal name; `byId` is rebuilt whenever the catalog changes.
    struct Wanted {
        std::uint32_t component;
        std::shared_ptr<ScopeBuffer> buffer;
    };
    std::unordered_map<std::string, std::vector<Wanted>> wanted;
    std::vector<const std::vector<Wanted>*> byId;  // index: signal id; null if not subscribed
    std::shared_ptr<const SignalCatalog> catalog;
    bool subscriptionsDirty{false};

    void resetBuffers() {
        for (auto& [name, list] : wanted) {
            const SignalInfo* signal = catalog ? catalog->find(name) : nullptr;
            for (Wanted& item : list) {
                item.buffer->reset(signal != nullptr ? signal->dtNs : item.buffer->dtNs());
            }
        }
    }
    std::chrono::steady_clock::time_point nextAttempt{};
};

ViewerClient::ViewerClient(proto::Endpoint endpoint)
    : endpoint_(std::move(endpoint))
    , wakeFd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC))
    , worker_(std::make_unique<Worker>()) {
    status_.store(std::make_shared<const ServerStatus>());
    notice_.store(std::make_shared<const Notice>());
    logs_.store(std::make_shared<const LogBook>());
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

void ViewerClient::setEndpoint(proto::Endpoint endpoint) {
    if (!thread_.joinable()) {
        endpoint_ = std::move(endpoint);
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
    return {snapshots_.load(),    deltas_.load(),  resyncs_.load(),     connects_.load(),
            signalBlocks_.load(), samples_.load(), invalidRuns_.load(), droppedSamples_.load()};
}

std::shared_ptr<ScopeBuffer> ViewerClient::subscribe(const std::string& name,
                                                     std::uint32_t component) {
    const std::shared_ptr<const SignalCatalog> catalog = catalog_.load();
    const SignalInfo* info = catalog ? catalog->find(name) : nullptr;
    if (info == nullptr || component >= info->components) {
        return nullptr;
    }
    auto buffer = std::make_shared<ScopeBuffer>(info->dtNs);
    {
        const std::lock_guard lock(commandMutex_);
        subscriptionChanges_.push_back({true, name, component, buffer});
    }
    wake();
    return buffer;
}

void ViewerClient::unsubscribe(const std::shared_ptr<ScopeBuffer>& buffer) {
    {
        const std::lock_guard lock(commandMutex_);
        subscriptionChanges_.push_back({false, {}, 0, buffer});
    }
    wake();
}

void ViewerClient::sendSubscriptions() {
    Worker& w = *worker_;
    w.subscriptionsDirty = false;
    w.byId.clear();
    if (!w.catalog) {
        return;
    }
    w.byId.assign(w.catalog->signals.size(), nullptr);
    // The server keeps what was subscribed before, so name what is no longer wanted too.
    v1::Envelope subscribe;
    v1::Envelope unsubscribe;
    subscribe.mutable_subscribe()->set_catalog_generation(w.catalog->generation);
    for (const SignalInfo& info : w.catalog->signals) {
        const auto found = w.wanted.find(info.name);
        if (found != w.wanted.end() && !found->second.empty()) {
            w.byId[info.id] = &found->second;
            auto* item = subscribe.mutable_subscribe()->add_signals();
            item->set_signal(info.id);
            item->set_mode(v1::SignalSubscription::MODE_FULL);
            item->set_decimation(1);
        } else {
            unsubscribe.mutable_unsubscribe()->add_signals(info.id);
        }
    }
    (void)w.connection.queue(unsubscribe);
    (void)w.connection.queue(subscribe);
}

void ViewerClient::handleSignalBlock(const v1::Envelope& envelope) {
    Worker& w = *worker_;
    const v1::SignalBlock& block = envelope.signal_block();
    if (!w.catalog || block.catalog_generation() != w.catalog->generation) {
        return;  // for a catalog we no longer use
    }
    ++signalBlocks_;
    for (const v1::SignalRun& run : block.runs()) {
        if (run.signal() >= w.byId.size() || w.byId[run.signal()] == nullptr) {
            continue;  // not subscribed (any more)
        }
        const SignalInfo& info = w.catalog->signals[run.signal()];
        // Everything in a run comes from the network: check it before reading any sample.
        const std::size_t components = run.components();
        if (run.mode() != v1::SignalSubscription::MODE_FULL ||
            run.encoding() != v1::SAMPLE_ENCODING_F64_LE || components != info.components ||
            run.dt_ns() != info.dtNs ||
            run.samples().size() != std::size_t{run.sample_count()} * components * sizeof(double)) {
            ++invalidRuns_;
            continue;
        }
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): bytes of a protobuf string.
        const auto* bytes = reinterpret_cast<const std::byte*>(run.samples().data());
        for (const Worker::Wanted& wanted : *w.byId[run.signal()]) {
            const std::uint64_t droppedBefore = wanted.buffer->droppedSamples();
            wanted.buffer->push(run.first_index(), bytes + wanted.component * sizeof(double),
                                run.sample_count(), components * sizeof(double));
            samples_ += run.sample_count();
            droppedSamples_ += wanted.buffer->droppedSamples() - droppedBefore;
        }
    }
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

void ViewerClient::openFile(const std::string& path) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    const bool recording = path.size() > 5 && path.compare(path.size() - 5, 5, ".mcap") == 0;
    if (recording) {
        envelope->mutable_command()->mutable_open_recording()->set_path(path);
    } else {
        envelope->mutable_command()->mutable_import_robot()->set_path(path);
    }
    post(false, "Loading " + path + " ...");
    enqueue(std::move(envelope));
}

std::uint64_t ViewerClient::requestAttributes(std::uint32_t prim) {
    auto envelope = std::make_unique<v1::Envelope>();
    const std::uint64_t id = nextRequestId_++;
    envelope->mutable_command()->set_request_id(id);
    envelope->mutable_command()->mutable_get_attributes()->add_prims(prim);
    enqueue(std::move(envelope));
    return id;
}

std::uint64_t ViewerClient::validateProfile(const std::string& yaml) {
    auto envelope = std::make_unique<v1::Envelope>();
    const std::uint64_t id = nextRequestId_++;
    envelope->mutable_command()->set_request_id(id);
    envelope->mutable_command()->mutable_validate_profile()->set_yaml(yaml);
    enqueue(std::move(envelope));
    return id;
}

void ViewerClient::applyProfile(const std::string& yaml) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    v1::ApplyProfile* apply = envelope->mutable_command()->mutable_apply_profile();
    apply->set_yaml(yaml);
    apply->set_restart(true);
    enqueue(std::move(envelope));
}

// Called from the UI thread (openFile) and from the worker (results); the serial only has to
// change, so a rare race between the two costs nothing.
void ViewerClient::post(bool error, std::string text) {
    auto notice = std::make_shared<Notice>();
    notice->serial = notice_.load()->serial + 1;
    notice->error = error;
    notice->text = std::move(text);
    notice_.store(std::move(notice));
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

namespace {

AttributeInfo toAttribute(const v1::Attribute& source) {
    AttributeInfo info;
    info.name = source.name();
    info.unit = source.unit().display();
    if (info.unit == "1") {
        info.unit.clear();  // a pure number
    }
    info.doc = source.doc();
    info.editable = source.editable();
    char text[64];
    switch (source.value_case()) {
        case v1::Attribute::kReal:
            info.kind = AttributeInfo::Kind::Real;
            std::snprintf(text, sizeof(text), "%.6g", source.real());
            info.value = text;
            break;
        case v1::Attribute::kInteger:
            info.kind = AttributeInfo::Kind::Integer;
            info.value = std::to_string(source.integer());
            break;
        case v1::Attribute::kBoolean:
            info.kind = AttributeInfo::Kind::Boolean;
            info.value = source.boolean() ? "true" : "false";
            break;
        case v1::Attribute::kAsset:
            info.kind = AttributeInfo::Kind::Asset;
            info.value = source.asset();
            break;
        case v1::Attribute::kReals:
            info.kind = AttributeInfo::Kind::Reals;
            for (int i = 0; i < source.reals().v_size() && i < 16; ++i) {
                std::snprintf(text, sizeof(text), "%s%.6g", i == 0 ? "" : ", ",
                              source.reals().v(i));
                info.value += text;
            }
            break;
        case v1::Attribute::kTargets:
            info.kind = AttributeInfo::Kind::Targets;
            info.targets.assign(source.targets().prims().begin(), source.targets().prims().end());
            break;
        default:
            info.kind = AttributeInfo::Kind::Token;
            info.value = source.token();
            break;
    }
    return info;
}

}  // namespace

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
            // The server sends its console history again on every connection.
            logs_.store(std::make_shared<const LogBook>());
            publish(ConnectionState::Ready);
            requestSnapshot();
            break;
        }
        case v1::Envelope::kSnapshot: {
            w.snapshotRequested = false;
            // A snapshot marks a discontinuity in time (seek, restart): sample indices start
            // again after it, so the scope histories start again too.
            w.resetBuffers();
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
        case v1::Envelope::kCatalog: {
            auto catalog = std::make_shared<SignalCatalog>();
            catalog->generation = envelope.catalog().generation();
            for (const v1::SignalInfo& source : envelope.catalog().signals()) {
                // Ids must be dense so that they can index tables directly.
                if (source.id() != catalog->signals.size() || source.native_dt_ns() <= 0 ||
                    source.components() == 0 || source.components() > 64) {
                    break;
                }
                SignalInfo info;
                info.id = source.id();
                info.name = source.name();
                info.unit = source.unit().display();
                info.writer = source.writer();
                info.components = source.components();
                info.componentNames.assign(source.component_names().begin(),
                                           source.component_names().end());
                info.dtNs = source.native_dt_ns();
                info.prim = source.prim();
                catalog->signals.push_back(std::move(info));
            }
            w.catalog = catalog;
            catalog_.store(catalog);
            w.resetBuffers();
            sendSubscriptions();
            w.status.signalCount = static_cast<std::uint32_t>(catalog->signals.size());
            status_.store(std::make_shared<const ServerStatus>(w.status));
            break;
        }
        case v1::Envelope::kSignalBlock:
            handleSignalBlock(envelope);
            break;
        case v1::Envelope::kCommandResult: {
            const v1::CommandResult& result = envelope.command_result();
            if (result.has_attributes()) {
                // The inspector asks for one prim at a time; take the first item.
                auto answer = std::make_shared<PrimAttributes>();
                answer->requestId = result.request_id();
                if (result.attributes().items_size() > 0) {
                    const v1::PrimAttributes& item = result.attributes().items(0);
                    answer->prim = item.prim();
                    answer->attributes.reserve(static_cast<std::size_t>(item.attributes_size()));
                    for (const v1::Attribute& source : item.attributes()) {
                        answer->attributes.push_back(toAttribute(source));
                    }
                }
                attributes_.store(std::move(answer));
            } else if (result.has_validation()) {
                auto check = std::make_shared<ProfileCheck>();
                check->requestId = result.request_id();
                check->valid = result.validation().valid();
                for (const v1::ProfileIssue& source : result.validation().issues()) {
                    check->issues.push_back(
                        {source.severity() != v1::ProfileIssue::SEVERITY_WARNING,
                         source.json_pointer(), source.message()});
                }
                profileCheck_.store(std::move(check));
            } else if (!result.ok()) {
                post(true,
                     result.error().empty() ? "the server refused a command" : result.error());
            } else if (!result.imported_stage_path().empty()) {
                post(false, "Loaded " + result.imported_stage_path());
            }
            break;
        }
        case v1::Envelope::kProfileState: {
            const v1::ProfileState& source = envelope.profile_state();
            auto profile = std::make_shared<ProfileInfo>();
            profile->serial = ++profileSerial_;
            profile->yaml = source.yaml();
            profile->schemaVersion = source.schema_version();
            profile->restartRequired = source.restart_required();
            profile->domains.reserve(static_cast<std::size_t>(source.domains_size()));
            for (const v1::DomainState& domain : source.domains()) {
                DomainInfo info;
                info.name = domain.domain();
                info.enabled = domain.enabled();
                info.available = domain.available();
                info.unavailableReason = domain.unavailable_reason();
                info.activeModule = domain.active_module();
                info.substituted = domain.substituted();
                for (const v1::SubstituteOption& option : domain.substitutes()) {
                    info.substitutes.push_back({option.token(), option.kind(), option.description(),
                                                option.needs_source()});
                }
                info.backends.assign(domain.backends().begin(), domain.backends().end());
                info.fidelities.assign(domain.fidelities().begin(), domain.fidelities().end());
                info.dtNs = domain.dt_ns();
                info.minDtNs = domain.min_dt_ns();
                info.maxDtNs = domain.max_dt_ns();
                info.cpuMsPerSimS = domain.cpu_ms_per_sim_s();
                info.dependsOn.assign(domain.depends_on().begin(), domain.depends_on().end());
                profile->domains.push_back(std::move(info));
            }
            profile_.store(std::move(profile));
            break;
        }
        case v1::Envelope::kLog: {
            // Copy-on-write: the book is small and log lines are rare next to frames.
            auto book = std::make_shared<LogBook>(*logs_.load());
            if (book->entries.size() >= LogBook::kCapacity) {
                const LogEntry& oldest = book->entries.front();
                book->warnings -= oldest.level == LogLevel::Warn ? 1U : 0U;
                book->errors -= oldest.level == LogLevel::Error ? 1U : 0U;
                book->entries.erase(book->entries.begin());
            }
            const v1::LogRecord& record = envelope.log();
            const int level =
                std::clamp(static_cast<int>(record.level()), static_cast<int>(LogLevel::Debug),
                           static_cast<int>(LogLevel::Error));
            book->entries.push_back({record.sim_time().ns(), static_cast<LogLevel>(level),
                                     record.source(), record.text()});
            book->warnings += level == static_cast<int>(LogLevel::Warn) ? 1U : 0U;
            book->errors += level == static_cast<int>(LogLevel::Error) ? 1U : 0U;
            ++book->total;
            logs_.store(std::move(book));
            break;
        }
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
            w.catalog.reset();  // the next server may number its signals differently
            w.byId.clear();
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
                std::deque<SubscriptionChange> changes;
                {
                    const std::lock_guard lock(commandMutex_);
                    pending.swap(commands_);
                    changes.swap(subscriptionChanges_);
                }
                for (const auto& command : pending) {
                    (void)w.connection.queue(*command);
                }
                for (SubscriptionChange& change : changes) {
                    if (change.add) {
                        w.wanted[change.name].push_back(
                            {change.component, std::move(change.buffer)});
                    } else {
                        for (auto& [name, list] : w.wanted) {
                            std::erase_if(list, [&](const Worker::Wanted& item) {
                                return item.buffer == change.buffer;
                            });
                        }
                    }
                    w.subscriptionsDirty = true;
                }
                if (w.subscriptionsDirty) {
                    sendSubscriptions();
                }
            }
            (void)w.connection.flush(ec);
        }
    }
    w.connection.close();
}

}  // namespace lagrange::viewer
