#include "lagrange/proto/Connection.hpp"
#include "lagrange/viewer/ViewerClient.hpp"
#include "lagrange/viewer/v1/envelope.pb.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
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

void ViewerClient::setSecurity(std::string caFile, std::string authToken) {
    if (!caFile.empty()) {
        tlsCaFile_ = std::move(caFile);
    }
    if (!authToken.empty()) {
        authToken_ = std::move(authToken);
    }
    tls_.reset();
}

ViewerClient::ViewerClient(proto::Endpoint endpoint)
    : endpoint_(std::move(endpoint))
    , wakeFd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC))
    , worker_(std::make_unique<Worker>()) {
    // NOLINTBEGIN(concurrency-mt-unsafe): read at construction, before the worker starts.
    if (const char* ca = std::getenv("LAGRANGE_TLS_CA")) {
        tlsCaFile_ = ca;
    }
    if (const char* token = std::getenv("LAGRANGE_AUTH_TOKEN")) {
        authToken_ = token;
    }
    // NOLINTEND(concurrency-mt-unsafe)
    status_.store(std::make_shared<const ServerStatus>());
    notice_.store(std::make_shared<const Notice>());
    logs_.store(std::make_shared<const LogBook>());
    events_.store(std::make_shared<const EventLog>());
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

std::shared_ptr<SensorStream> ViewerClient::watchSensor(const std::string& name) {
    {
        const std::lock_guard lock(sensorMutex_);
        for (const std::shared_ptr<SensorStream>& stream : sensors_) {
            if (stream->name() == name) {
                return stream;
            }
        }
        sensors_.push_back(std::make_shared<SensorStream>(name));
    }
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    v1::SubscribeSensor* subscribe = envelope->mutable_command()->mutable_subscribe_sensor();
    subscribe->set_sensor(name);
    subscribe->set_enabled(true);
    enqueue(std::move(envelope));
    const std::lock_guard lock(sensorMutex_);
    return sensors_.back();
}

void ViewerClient::subscribeSensors() {
    std::vector<std::string> names;
    {
        const std::lock_guard lock(sensorMutex_);
        for (const std::shared_ptr<SensorStream>& stream : sensors_) {
            names.push_back(stream->name());
        }
    }
    for (const std::string& name : names) {
        auto envelope = std::make_unique<v1::Envelope>();
        envelope->mutable_command()->set_request_id(nextRequestId_++);
        v1::SubscribeSensor* subscribe = envelope->mutable_command()->mutable_subscribe_sensor();
        subscribe->set_sensor(name);
        subscribe->set_enabled(true);
        enqueue(std::move(envelope));
    }
}

void ViewerClient::handleSensorFrame(const v1::Envelope& envelope) {
    constexpr std::uint32_t kMaxPoints = 4'000'000;
    constexpr std::uint32_t kFloats = 4;  // x, y, z, intensity
    const v1::SensorFrame& frame = envelope.sensor_frame();
    std::shared_ptr<SensorStream> stream;
    {
        const std::lock_guard lock(sensorMutex_);
        for (const std::shared_ptr<SensorStream>& candidate : sensors_) {
            if (candidate->name() == frame.sensor()) {
                stream = candidate;
            }
        }
    }
    if (stream && frame.has_image()) {
        handleImage(frame, *stream);
        return;
    }
    if (!stream || !frame.has_points()) {
        return;  // not watched, or a kind of frame this viewer does not draw yet
    }
    // The message comes from the network: check every size before touching the bytes.
    const v1::PointCloudData& points = frame.points();
    const std::string& bytes = points.data().inline_();
    const bool layout = points.fields_size() >= 4 && points.fields(0) == "x" &&
                        points.fields(1) == "y" && points.fields(2) == "z" &&
                        points.fields(3) == "intensity";
    if (!layout || points.count() > kMaxPoints || points.stride() < kFloats * sizeof(float) ||
        bytes.size() != static_cast<std::size_t>(points.count()) * points.stride()) {
        ++stream->rejected_;
        return;
    }
    auto cloud = std::make_shared<PointCloud>();
    cloud->simTimeNs = frame.sim_time().ns();
    cloud->count = points.count();
    cloud->points.resize(static_cast<std::size_t>(points.count()) * kFloats);
    for (std::uint32_t i = 0; i < points.count(); ++i) {
        std::memcpy(&cloud->points[static_cast<std::size_t>(i) * kFloats],
                    bytes.data() + static_cast<std::size_t>(i) * points.stride(),
                    kFloats * sizeof(float));
    }
    stream->latest_.store(std::move(cloud));
    ++stream->frames_;
}

void ViewerClient::handleImage(const v1::SensorFrame& frame, SensorStream& stream) {
    constexpr std::uint32_t kMaxSide = 16'384;  // as the camera module allows
    // The message comes from the network: check every size before touching the bytes.
    const v1::ImageData& image = frame.image();
    const std::string& bytes = image.data().inline_();
    if (image.encoding() != "rgba8" || image.width() == 0 || image.height() == 0 ||
        image.width() > kMaxSide || image.height() > kMaxSide ||
        bytes.size() != std::size_t{image.width()} * image.height() * 4) {
        ++stream.rejected_;
        return;
    }
    auto out = std::make_shared<SensorImage>();
    out->simTimeNs = frame.sim_time().ns();
    out->width = image.width();
    out->height = image.height();
    out->rgba.assign(bytes.begin(), bytes.end());
    stream.latestImage_.store(std::move(out));
    ++stream.frames_;
}

float FieldSliceData::sample(double x, double y) const {
    if (cellM <= 0.0 || width == 0 || height == 0) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const double column = std::round((x - originX) / cellM);
    const double row = std::round((y - originY) / cellM);
    if (column < 0.0 || row < 0.0 || column >= static_cast<double>(width) ||
        row >= static_cast<double>(height)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const std::size_t at =
        (static_cast<std::size_t>(row) * width + static_cast<std::size_t>(column)) * components;
    if (components == 1) {
        return values[at];
    }
    float squares = 0.0F;
    for (std::uint32_t c = 0; c < components; ++c) {
        squares += values[at + c] * values[at + c];
    }
    return std::sqrt(squares);
}

void ViewerClient::sendFieldSubscription(const std::string& name, double heightM) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    v1::SubscribeField* subscribe = envelope->mutable_command()->mutable_subscribe_field();
    subscribe->set_field(name);
    subscribe->set_enabled(true);
    subscribe->set_height_m(heightM);
    enqueue(std::move(envelope));
}

std::shared_ptr<FieldStream> ViewerClient::watchField(const std::string& name, double heightM) {
    std::shared_ptr<FieldStream> stream;
    bool changed = false;
    {
        const std::lock_guard lock(sensorMutex_);
        for (WatchedField& field : fields_) {
            if (field.stream->name() == name) {
                stream = field.stream;
                changed = field.heightM != heightM;
                field.heightM = heightM;
            }
        }
        if (!stream) {
            stream = std::make_shared<FieldStream>(name);
            fields_.push_back({stream, heightM});
            changed = true;
        }
    }
    if (changed) {
        sendFieldSubscription(name, heightM);
    }
    return stream;
}

void ViewerClient::sendViewRequests() {
    auto graph = std::make_unique<v1::Envelope>();
    graph->mutable_command()->set_request_id(nextRequestId_++);
    graph->mutable_command()->mutable_set_ros_graph()->set_enabled(rosGraphWanted_);
    enqueue(std::move(graph));
    auto world = std::make_unique<v1::Envelope>();
    world->mutable_command()->set_request_id(nextRequestId_++);
    world->mutable_command()->mutable_set_world()->set_enabled(worldWanted_);
    enqueue(std::move(world));
}

void ViewerClient::setRosGraphWanted(bool wanted) {
    if (rosGraphWanted_.exchange(wanted) != wanted) {
        sendViewRequests();
    }
}

void ViewerClient::setCameraInterest(const std::array<double, 3>& eye) {
    constexpr double kMovedM = 10.0;
    constexpr auto kEvery = std::chrono::milliseconds(500);
    const auto now = std::chrono::steady_clock::now();
    const double moved =
        std::hypot(eye[0] - sentEye_[0], eye[1] - sentEye_[1], eye[2] - sentEye_[2]);
    if (moved < kMovedM || now - sentEyeAt_ < kEvery) {
        return;
    }
    sentEye_ = eye;
    sentEyeAt_ = now;
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    v1::SetCameraInterest& hint = *envelope->mutable_command()->mutable_set_camera_interest();
    hint.set_enabled(true);
    hint.set_x(eye[0]);
    hint.set_y(eye[1]);
    hint.set_z(eye[2]);
    enqueue(std::move(envelope));
}

void ViewerClient::setWorldWanted(bool wanted) {
    if (worldWanted_.exchange(wanted) != wanted) {
        sendViewRequests();
    }
}

void ViewerClient::handleFieldSlice(const v1::Envelope& envelope) {
    constexpr std::uint64_t kMaxCells = 4096ULL * 4096ULL;
    const v1::FieldSlice& slice = envelope.field_slice();
    std::shared_ptr<FieldStream> stream;
    {
        const std::lock_guard lock(sensorMutex_);
        for (const WatchedField& field : fields_) {
            if (field.stream->name() == slice.field()) {
                stream = field.stream;
            }
        }
    }
    if (!stream) {
        return;
    }
    // From the network: every size is checked before the bytes are read.
    const std::string& bytes = slice.values().inline_();
    const std::uint64_t cells = static_cast<std::uint64_t>(slice.width()) * slice.height();
    const std::uint32_t components = slice.components();
    if (cells == 0 || cells > kMaxCells || components == 0 || components > 4 ||
        slice.cell_m() <= 0.0 || bytes.size() != cells * components * sizeof(float)) {
        ++stream->rejected_;
        return;
    }
    auto data = std::make_shared<FieldSliceData>();
    data->simTimeNs = slice.sim_time().ns();
    data->width = slice.width();
    data->height = slice.height();
    data->components = components;
    data->originX = slice.origin_x();
    data->originY = slice.origin_y();
    data->cellM = slice.cell_m();
    data->heightM = slice.height_m();
    data->valueMin = slice.value_min();
    data->valueMax = slice.value_max();
    data->values.resize(static_cast<std::size_t>(cells) * components);
    std::memcpy(data->values.data(), bytes.data(), bytes.size());
    stream->latest_.store(std::move(data));
    ++stream->slices_;
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

void ViewerClient::setAttribute(std::uint32_t prim, const std::string& name, double value,
                                bool save) {
    auto envelope = std::make_unique<v1::Envelope>();
    envelope->mutable_command()->set_request_id(nextRequestId_++);
    v1::SetAttribute* set = envelope->mutable_command()->mutable_set_attribute();
    set->set_prim(prim);
    set->mutable_attribute()->set_name(name);
    set->mutable_attribute()->set_real(value);
    set->set_save_to_session_layer(save);
    enqueue(std::move(envelope));
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

EventInfo toEvent(const v1::Event& source) {
    EventInfo event;
    event.id = source.id();
    event.beginNs = source.begin().ns();
    event.endNs = std::max(source.end().ns(), event.beginNs);  // never a negative span
    event.domain = source.domain();
    event.kind = source.kind();
    const int severity =
        std::clamp(static_cast<int>(source.severity()), static_cast<int>(EventInfo::Severity::Info),
                   static_cast<int>(EventInfo::Severity::Error));
    event.severity = static_cast<EventInfo::Severity>(severity);
    event.text = source.text();
    event.prim = source.prim();
    event.signal = source.signal();
    return event;
}

WiringInfo toWiring(const v1::Wiring& source) {
    WiringInfo wiring;
    wiring.baseTickNs = source.base_tick_ns();
    wiring.syncNs = source.sync_ns();
    for (const v1::Wiring::Node& node : source.nodes()) {
        WiringInfo::Node out;
        out.module = node.module();
        out.domain = node.domain();
        out.substitute = node.substitute();
        out.detail = node.detail();
        out.dtNs = node.dt_ns();
        out.inputs.assign(node.inputs().begin(), node.inputs().end());
        out.outputs.assign(node.outputs().begin(), node.outputs().end());
        out.onGpu = node.on_gpu();
        wiring.nodes.push_back(std::move(out));
    }
    // A wire between modules the message does not list would have nothing to attach to.
    const auto known = [&](const std::string& module) {
        return std::ranges::any_of(
            wiring.nodes, [&](const WiringInfo::Node& node) { return node.module == module; });
    };
    for (const v1::Wiring::Wire& wire : source.wires()) {
        if (known(wire.from()) && known(wire.to())) {
            wiring.wires.push_back({wire.from(), wire.to(), wire.signal(), wire.symbol(),
                                    wire.source_unit(), wire.sink_unit(), wire.unit_ok(),
                                    wire.stiff()});
        }
    }
    return wiring;
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
            subscribeSensors();  // a new connection knows nothing of earlier requests
            {
                std::vector<std::pair<std::string, double>> wanted;
                {
                    const std::lock_guard lock(sensorMutex_);
                    for (const WatchedField& field : fields_) {
                        wanted.emplace_back(field.stream->name(), field.heightM);
                    }
                }
                for (const auto& [name, heightM] : wanted) {
                    sendFieldSubscription(name, heightM);
                }
            }
            if (rosGraphWanted_ || worldWanted_) {
                sendViewRequests();
            }
            environment_.store(nullptr);
            rosGraph_.store(nullptr);
            world_.store(nullptr);
            events_.store(std::make_shared<const EventLog>());
            statistics_.store(nullptr);
            recording_.store(nullptr);
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
                // Ids must be dense so that they can index tables directly, so a bad entry ends
                // the list. Arrays may be wide (a 3D lidar has a range per beam: 128 × 4096 is a
                // big one) but are bounded; the scope keeps a buffer per component it plots.
                constexpr std::uint32_t kMaxComponents = 1U << 20U;
                if (source.id() != catalog->signals.size() || source.native_dt_ns() <= 0 ||
                    source.components() == 0 || source.components() > kMaxComponents) {
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
        case v1::Envelope::kSensorFrame:
            handleSensorFrame(envelope);
            break;
        case v1::Envelope::kFieldSlice:
            handleFieldSlice(envelope);
            break;
        case v1::Envelope::kEnvironment: {
            const v1::EnvironmentState& source = envelope.environment();
            auto info = std::make_shared<EnvironmentInfo>();
            info->layer = source.layer();
            info->localTimeHours = source.local_time_hours();
            info->sunElevationDeg = source.sun_elevation_deg();
            info->ambientTemperatureK = source.ambient_temperature_k();
            info->gravity = source.gravity_m_per_s2();
            info->rainMmPerH = source.weather().rain_mm_per_h();
            info->fogVisibilityM = source.weather().fog_visibility_m();
            info->dustMgPerM3 = source.weather().dust_mg_per_m3();
            info->surfaceWetness = source.weather().surface_wetness();
            info->windSpeed = source.wind().speed_m_per_s();
            info->windFromDeg =
                std::fmod(90.0 - source.wind().direction_deg() + 180.0 + 720.0, 360.0);
            info->windDirectionDeg = source.wind().direction_deg();
            info->gustIntensity = source.wind().gust_intensity();
            info->windField = source.wind().field();
            info->synthetic = source.synthetic();
            for (const auto& emi : source.emi_sources()) {
                info->emiSources.push_back({emi.name(), emi.prim(), emi.frequency_hz(),
                                            emi.current_a(), emi.power_w(), emi.detail()});
            }
            for (const auto& field : source.fields()) {
                info->fields.push_back({field.name(), field.label(), field.domain(),
                                        field.unit().display(),
                                        std::max<std::uint32_t>(field.components(), 1),
                                        field.available(), field.unavailable_reason()});
            }
            environment_.store(std::move(info));
            break;
        }
        case v1::Envelope::kRosGraph: {
            const v1::RosGraph& source = envelope.ros_graph();
            auto graph = std::make_shared<RosGraphInfo>();
            graph->rmw = source.rmw();
            graph->distro = source.distro();
            graph->domainId = source.domain_id();
            graph->useSimTime = source.use_sim_time();
            graph->note = source.note();
            for (const auto& topic : source.topics()) {
                graph->topics.push_back(
                    {topic.name(), topic.type(), topic.publishers(), topic.subscribers(),
                     topic.rate_hz(), topic.bandwidth_bps(), topic.from_bridge(), topic.status()});
            }
            for (const auto& node : source.node_info()) {
                RosGraphInfo::Node out;
                out.name = node.name();
                out.publishes.assign(node.publishes().begin(), node.publishes().end());
                out.subscribes.assign(node.subscribes().begin(), node.subscribes().end());
                out.external = node.external();
                out.detail = node.detail();
                graph->nodes.push_back(std::move(out));
            }
            // An older server lists node names only.
            for (int i = 0; source.node_info_size() == 0 && i < source.nodes_size(); ++i) {
                RosGraphInfo::Node out;
                out.name = source.nodes(i);
                graph->nodes.push_back(std::move(out));
            }
            graph->hasControl = source.has_control();
            graph->hardware = source.control().hardware();
            graph->plugin = source.control().plugin();
            graph->controlRateHz = source.control().rate_hz();
            graph->controlActive = source.control().active();
            for (const auto& joint : source.control().joints()) {
                graph->joints.push_back({joint.name(), joint.interface(), joint.command(),
                                         joint.state(), joint.unit()});
            }
            for (const auto& controller : source.control().controllers()) {
                graph->controllers.push_back(
                    {controller.name(), controller.rate_hz(), controller.state()});
            }
            rosGraph_.store(std::move(graph));
            break;
        }
        case v1::Envelope::kWorld: {
            const v1::WorldState& source = envelope.world();
            auto world = std::make_shared<WorldInfo>();
            const std::shared_ptr<const WorldInfo> before = world_.load();
            world->simTimeNs = source.sim_time().ns();
            world->georeferenced = source.geo_origin().valid();
            world->latitudeDeg = source.geo_origin().latitude_deg();
            world->longitudeDeg = source.geo_origin().longitude_deg();
            world->heightM = source.geo_origin().height_m();
            world->tileSizeM = source.tile_size_m();
            world->tilesGeneration = source.tiles_generation();
            // Tiles come only when they changed; otherwise keep the ones already held. The
            // three arrays must be the same length, or the message is not used for tiles.
            const auto count = static_cast<std::size_t>(source.tile_x_size());
            if (count != 0 && count <= WorldInfo::kMaxTiles &&
                count == static_cast<std::size_t>(source.tile_y_size()) &&
                count == source.tile_lod().size()) {
                world->tiles.reserve(count);
                for (std::size_t i = 0; i < count; ++i) {
                    world->tiles.push_back({source.tile_x(static_cast<int>(i)),
                                            source.tile_y(static_cast<int>(i)),
                                            static_cast<std::uint8_t>(source.tile_lod()[i])});
                }
            } else if (before && before->tilesGeneration == world->tilesGeneration) {
                world->tiles = before->tiles;
            }
            for (const auto& agent : source.agents()) {
                world->agents.push_back({agent.name(), agent.prim(), agent.kind(), agent.x(),
                                         agent.y(), agent.speed_m_per_s(), agent.status(),
                                         agent.active_radius_m(), agent.focus()});
            }
            world->anchor = source.floating_origin().anchor();
            world->origin = {source.floating_origin().x(), source.floating_origin().y(),
                             source.floating_origin().z()};
            world->recenterThresholdM = source.floating_origin().recenter_threshold_m();
            world->lastRecenterNs = source.floating_origin().last_recenter().ns();
            world->recenters = source.floating_origin().recenters();
            world->gpuMemoryBytes = source.streaming().gpu_memory_bytes();
            world->gpuMemoryBudgetBytes = source.streaming().gpu_memory_budget_bytes();
            world->hostCacheBytes = source.streaming().host_cache_bytes();
            world->hostCacheBudgetBytes = source.streaming().host_cache_budget_bytes();
            world->diskReadBytesPerS = source.streaming().disk_read_bytes_per_s();
            world->loadQueue = source.streaming().load_queue();
            for (const auto& region : source.regions()) {
                world->regions.push_back({region.name(), region.min_x(), region.min_y(),
                                          region.max_x(), region.max_y()});
            }
            world->synthetic = source.synthetic();
            world_.store(std::move(world));
            break;
        }
        case v1::Envelope::kEventList: {
            auto log = std::make_shared<EventLog>();
            log->complete = envelope.event_list().complete();
            const int count = std::min(envelope.event_list().events_size(),
                                       static_cast<int>(EventLog::kCapacity));
            log->events.reserve(static_cast<std::size_t>(count));
            for (int i = 0; i < count; ++i) {
                log->events.push_back(toEvent(envelope.event_list().events(i)));
            }
            std::ranges::stable_sort(log->events, {}, &EventInfo::beginNs);
            events_.store(std::move(log));
            break;
        }
        case v1::Envelope::kEvent: {
            // Copy-on-write, like the log: events are rare next to frames.
            auto log = std::make_shared<EventLog>(*events_.load());
            if (log->events.size() < EventLog::kCapacity) {
                EventInfo event = toEvent(envelope.event());
                const auto at =
                    std::ranges::upper_bound(log->events, event.beginNs, {}, &EventInfo::beginNs);
                log->events.insert(at, std::move(event));
                events_.store(std::move(log));
            }
            break;
        }
        case v1::Envelope::kServerStats: {
            const v1::ServerStats& source = envelope.server_stats();
            auto stats = std::make_shared<ServerStatistics>();
            stats->serial = ++statisticsSerial_;
            stats->simTimeNs = source.sim_time().ns();
            stats->realTimeFactor = source.real_time_factor();
            stats->tickMs = source.tick_ms();
            stats->tickMsMax = source.tick_ms_max();
            stats->memoryBytes = source.memory_bytes();
            stats->gpuMemoryBytes = source.gpu_memory_bytes();
            stats->bytesPerSecondOut = source.bytes_per_second_out();
            stats->viewers = source.viewers();
            stats->estimated = source.estimated();
            for (const v1::ServerStats::Module& module : source.modules()) {
                stats->modules.push_back({module.module(), module.domain(), module.dt_ns(),
                                          module.steps_per_second(), module.cpu_ms_per_step(),
                                          module.cpu_ms_per_step_max(), module.on_gpu()});
            }
            statistics_.store(std::move(stats));
            break;
        }
        case v1::Envelope::kRecordingInfo: {
            const v1::RecordingInfo& source = envelope.recording_info();
            auto facts = std::make_shared<RecordingFacts>();
            facts->path = source.path();
            facts->durationNs = source.duration().ns();
            facts->sizeBytes = source.size_bytes();
            facts->compression = source.compression();
            facts->messages = source.messages();
            facts->recordedUnixNs = source.recorded_unix_ns();
            facts->synthetic = source.synthetic();
            facts->generator = source.generator();
            facts->description = source.description();
            for (const v1::RecordingInfo::Channel& channel : source.channels()) {
                facts->channels.push_back(
                    {channel.topic(), channel.rate_hz(), channel.messages(), channel.unit()});
            }
            recording_.store(std::move(facts));
            break;
        }
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
                check->wiring = toWiring(result.validation().wiring());
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
            std::string tlsProblem;
            if (endpoint_.tls && !tls_) {
                tls_ = proto::TlsContext::client(tlsCaFile_, tlsProblem);
            }
            w.connection =
                proto::Connection::startConnect(endpoint_, ec, proto::kDefaultMaxFrameSize, tls_);
            w.nextAttempt = now + w.backoff.next();
            if (w.connection.state() == proto::Connection::State::Closed) {
                if (w.info.state != ConnectionState::Refused) {
                    publish(ConnectionState::Disconnected,
                            tlsProblem.empty() ? ec.message() : "TLS: " + tlsProblem);
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
            if (endpoint_.kind == proto::Endpoint::Kind::Tcp && !authToken_.empty()) {
                hello.mutable_client_hello()->set_auth_token(authToken_);
            }
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
                    publish(
                        ConnectionState::Disconnected,
                        received == proto::IoStatus::Error
                            ? (w.connection.tlsError().empty() ? ec.message()
                                                               : "TLS: " + w.connection.tlsError())
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
