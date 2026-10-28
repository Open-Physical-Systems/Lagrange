#include "lagrange/proto/Connection.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <openssl/err.h>
#include <openssl/ssl.h>
#include <openssl/x509v3.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <google/protobuf/message_lite.h>

namespace lagrange::proto {

namespace {

constexpr std::size_t kReceiveChunk = 64U * 1024U;
constexpr std::size_t kReceiveBudgetPerCall = 1024U * 1024U;  // keeps one peer from starving others
constexpr std::size_t kCompactThreshold = 64U * 1024U;

std::error_code lastError() {
    return {errno, std::generic_category()};
}

bool fillUnixAddress(const std::string& path, sockaddr_un& address) {
    if (path.empty() || path.size() >= sizeof(address.sun_path)) {
        return false;
    }
    address = {};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.data(), path.size());
    return true;
}

void setNoDelay(int fd) {
    const int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

// Owns the result of getaddrinfo.
struct AddressList {
    addrinfo* head{nullptr};
    AddressList() = default;
    AddressList(const AddressList&) = delete;
    AddressList& operator=(const AddressList&) = delete;
    ~AddressList() {
        if (head != nullptr) {
            ::freeaddrinfo(head);
        }
    }
};

bool resolve(const Endpoint& endpoint, bool passive, AddressList& out, std::error_code& ec) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_NUMERICSERV | (passive ? AI_PASSIVE : 0);
    const std::string port = std::to_string(endpoint.port);
    const int rc = ::getaddrinfo(endpoint.address.c_str(), port.c_str(), &hints, &out.head);
    if (rc != 0 || out.head == nullptr) {
        ec = std::make_error_code(std::errc::address_not_available);
        return false;
    }
    return true;
}

}  // namespace

// --- Connection -------------------------------------------------------------------------

// --- TLS ----------------------------------------------------------------------------------

namespace {

/// The newest OpenSSL error as text, and the queue cleared.
std::string sslErrorText() {
    std::string text;
    while (const unsigned long code = ERR_get_error()) {
        std::array<char, 256> buffer{};
        ERR_error_string_n(code, buffer.data(), buffer.size());
        text = buffer.data();
    }
    return text.empty() ? std::string("TLS failed") : text;
}

/// A context for TLS 1.2 or newer.
SSL_CTX* newContext(const SSL_METHOD* method) {
    SSL_CTX* context = SSL_CTX_new(method);
    if (context != nullptr) {
        SSL_CTX_set_min_proto_version(context, TLS1_2_VERSION);
        SSL_CTX_set_mode(context,
                         SSL_MODE_ENABLE_PARTIAL_WRITE | SSL_MODE_ACCEPT_MOVING_WRITE_BUFFER);
    }
    return context;
}

}  // namespace

std::shared_ptr<TlsContext> TlsContext::server(const std::string& certificateFile,
                                               const std::string& keyFile, std::string& error) {
    SSL_CTX* context = newContext(TLS_server_method());
    if (context == nullptr) {
        error = sslErrorText();
        return nullptr;
    }
    if (SSL_CTX_use_certificate_chain_file(context, certificateFile.c_str()) != 1 ||
        SSL_CTX_use_PrivateKey_file(context, keyFile.c_str(), SSL_FILETYPE_PEM) != 1 ||
        SSL_CTX_check_private_key(context) != 1) {
        error = certificateFile + ", " + keyFile + ": " + sslErrorText();
        SSL_CTX_free(context);
        return nullptr;
    }
    return std::shared_ptr<TlsContext>(new TlsContext(context, true));
}

std::shared_ptr<TlsContext> TlsContext::client(const std::string& caFile, std::string& error) {
    SSL_CTX* context = newContext(TLS_client_method());
    if (context == nullptr) {
        error = sslErrorText();
        return nullptr;
    }
    const int loaded = caFile.empty()
                           ? SSL_CTX_set_default_verify_paths(context)
                           : SSL_CTX_load_verify_locations(context, caFile.c_str(), nullptr);
    if (loaded != 1) {
        error = (caFile.empty() ? std::string("the system's trust store") : caFile) + ": " +
                sslErrorText();
        SSL_CTX_free(context);
        return nullptr;
    }
    SSL_CTX_set_verify(context, SSL_VERIFY_PEER, nullptr);
    return std::shared_ptr<TlsContext>(new TlsContext(context, false));
}

TlsContext::~TlsContext() {
    SSL_CTX_free(context_);
}

void Connection::SslDeleter::operator()(ssl_st* ssl) const {
    SSL_free(ssl);
}

IoStatus Connection::tlsFailure(std::error_code& ec, int result) {
    const int reason = SSL_get_error(ssl_.get(), result);
    if (reason == SSL_ERROR_WANT_READ) {
        tlsWantsWrite_ = false;
        return IoStatus::WouldBlock;
    }
    if (reason == SSL_ERROR_WANT_WRITE) {
        tlsWantsWrite_ = true;
        return IoStatus::WouldBlock;
    }
    if (reason == SSL_ERROR_ZERO_RETURN) {
        close();
        return IoStatus::Closed;
    }
    if (reason == SSL_ERROR_SYSCALL && ERR_peek_error() == 0) {
        // The peer went away without a TLS close.
        close();
        return IoStatus::Closed;
    }
    tlsError_ = sslErrorText();
    const long verify = SSL_get_verify_result(ssl_.get());
    if (verify != X509_V_OK) {
        tlsError_ += std::string(" (certificate: ") + X509_verify_cert_error_string(verify) + ")";
    }
    ec = std::make_error_code(std::errc::protocol_error);
    close();
    return IoStatus::Error;
}

IoStatus Connection::tlsHandshake(std::error_code& ec) {
    if (!ssl_ || tlsReady_) {
        return IoStatus::Ok;
    }
    const int result = SSL_do_handshake(ssl_.get());
    if (result == 1) {
        tlsReady_ = true;
        tlsWantsWrite_ = false;
        return IoStatus::Ok;
    }
    return tlsFailure(ec, result);
}

IoStatus Connection::flushTls(std::error_code& ec) {
    while (outPos_ < out_.size()) {
        const auto length =
            static_cast<int>(std::min<std::size_t>(out_.size() - outPos_, 1U << 30U));
        const int sent = SSL_write(ssl_.get(), out_.data() + outPos_, length);
        if (sent <= 0) {
            return tlsFailure(ec, sent);
        }
        outPos_ += static_cast<std::size_t>(sent);
        sentTotal_ += static_cast<std::uint64_t>(sent);
    }
    out_.clear();
    outPos_ = 0;
    return IoStatus::Ok;
}

IoStatus Connection::receiveTls(std::error_code& ec) {
    std::size_t total = 0;
    while (total < kReceiveBudgetPerCall) {
        const int received =
            SSL_read(ssl_.get(), scratch_.data(), static_cast<int>(scratch_.size()));
        if (received <= 0) {
            const IoStatus status = tlsFailure(ec, received);
            if (status == IoStatus::WouldBlock) {
                return total > 0 ? IoStatus::Ok : IoStatus::WouldBlock;
            }
            return total > 0 && status == IoStatus::Closed ? IoStatus::Ok : status;
        }
        reader_.feed(std::span{scratch_}.first(static_cast<std::size_t>(received)));
        total += static_cast<std::size_t>(received);
    }
    return IoStatus::Ok;
}

Connection Connection::adopt(UniqueFd socket, Endpoint::Kind kind, std::uint32_t maxFrameSize,
                             std::shared_ptr<TlsContext> tls) {
    Connection connection;
    connection.socket_ = std::move(socket);
    connection.kind_ = kind;
    connection.maxFrameSize_ = maxFrameSize;
    connection.reader_ = FrameReader(maxFrameSize);
    connection.state_ = connection.socket_.valid() ? State::Connected : State::Closed;
    if (connection.socket_.valid() && kind == Endpoint::Kind::Tcp) {
        setNoDelay(connection.socket_.get());
    }
    if (tls && connection.socket_.valid()) {
        connection.ssl_.reset(SSL_new(tls->get()));
        if (!connection.ssl_ || SSL_set_fd(connection.ssl_.get(), connection.socket_.get()) != 1) {
            connection.tlsError_ = sslErrorText();
            connection.close();
            return connection;
        }
        SSL_set_accept_state(connection.ssl_.get());
        connection.tlsContext_ = std::move(tls);
    }
    return connection;
}

Connection Connection::startConnect(const Endpoint& endpoint, std::error_code& ec,
                                    std::uint32_t maxFrameSize, std::shared_ptr<TlsContext> tls) {
    ec.clear();
    Connection connection;
    if (endpoint.tls && (!tls || tls->isServer())) {
        ec = std::make_error_code(std::errc::invalid_argument);  // tls:// needs a client context
        return connection;
    }
    connection.kind_ = endpoint.kind;
    connection.maxFrameSize_ = maxFrameSize;
    connection.reader_ = FrameReader(maxFrameSize);

    int rc = -1;
    if (endpoint.kind == Endpoint::Kind::Unix) {
        sockaddr_un address{};
        if (!fillUnixAddress(endpoint.address, address)) {
            ec = std::make_error_code(std::errc::filename_too_long);
            return connection;
        }
        connection.socket_.reset(::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
        if (!connection.socket_.valid()) {
            ec = lastError();
            return connection;
        }
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): the sockets API needs it.
        rc = ::connect(connection.socket_.get(), reinterpret_cast<const sockaddr*>(&address),
                       sizeof(address));
    } else {
        AddressList addresses;
        if (!resolve(endpoint, false, addresses, ec)) {
            return connection;
        }
        const addrinfo* info = addresses.head;
        connection.socket_.reset(
            ::socket(info->ai_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
        if (!connection.socket_.valid()) {
            ec = lastError();
            return connection;
        }
        rc = ::connect(connection.socket_.get(), info->ai_addr, info->ai_addrlen);
    }

    if (endpoint.tls && connection.socket_.valid() && (rc == 0 || errno == EINPROGRESS)) {
        const int saved = errno;
        connection.ssl_.reset(SSL_new(tls->get()));
        SSL* ssl = connection.ssl_.get();
        // The certificate must name the endpoint: its host name, or its IP address.
        in6_addr scratch{};
        const bool literal = ::inet_pton(AF_INET, endpoint.address.c_str(), &scratch) == 1 ||
                             ::inet_pton(AF_INET6, endpoint.address.c_str(), &scratch) == 1;
        if (ssl == nullptr || SSL_set_fd(ssl, connection.socket_.get()) != 1 ||
            (literal
                 ? X509_VERIFY_PARAM_set1_ip_asc(SSL_get0_param(ssl), endpoint.address.c_str()) != 1
                 : (SSL_set_tlsext_host_name(ssl, endpoint.address.c_str()) != 1 ||
                    SSL_set1_host(ssl, endpoint.address.c_str()) != 1))) {
            connection.tlsError_ = sslErrorText();
            ec = std::make_error_code(std::errc::protocol_error);
            connection.socket_.reset();
            connection.ssl_.reset();
            return connection;
        }
        SSL_set_connect_state(ssl);
        connection.tlsContext_ = std::move(tls);
        errno = saved;
    }
    if (rc == 0) {
        connection.state_ = State::Connected;
        if (connection.kind_ == Endpoint::Kind::Tcp) {
            setNoDelay(connection.socket_.get());
        }
    } else if (errno == EINPROGRESS) {
        connection.state_ = State::Connecting;
    } else {
        ec = lastError();
        connection.socket_.reset();
    }
    return connection;
}

short Connection::pollEvents() const {
    switch (state_) {
        case State::Connecting:
            return POLLOUT;
        case State::Connected:
            if (ssl_ && !tlsReady_) {
                return tlsWantsWrite_ ? POLLOUT : POLLIN;
            }
            return static_cast<short>(POLLIN | (pendingOutputBytes() > 0 ? POLLOUT : 0));
        case State::Closed:
            break;
    }
    return 0;
}

IoStatus Connection::finishConnect(std::error_code& ec) {
    ec.clear();
    if (state_ == State::Connected) {
        return IoStatus::Ok;
    }
    if (state_ != State::Connecting) {
        ec = std::make_error_code(std::errc::not_connected);
        return IoStatus::Error;
    }
    pollfd probe{socket_.get(), POLLOUT, 0};
    if (::poll(&probe, 1, 0) == 0) {
        return IoStatus::WouldBlock;
    }
    int error = 0;
    socklen_t length = sizeof(error);
    if (::getsockopt(socket_.get(), SOL_SOCKET, SO_ERROR, &error, &length) != 0) {
        error = errno;
    }
    if (error != 0) {
        ec = {error, std::generic_category()};
        close();
        return IoStatus::Error;
    }
    state_ = State::Connected;
    if (kind_ == Endpoint::Kind::Tcp) {
        setNoDelay(socket_.get());
    }
    return IoStatus::Ok;
}

bool Connection::queue(const google::protobuf::MessageLite& message, std::span<const int> fds) {
    if (state_ == State::Closed) {
        return false;
    }
    if (fds.size() > kMaxFdsPerFrame || (!fds.empty() && kind_ != Endpoint::Kind::Unix)) {
        return false;
    }
    // Duplicate first: the caller keeps ownership of its descriptors.
    std::vector<UniqueFd> duplicates;
    duplicates.reserve(fds.size());
    for (const int fd : fds) {
        UniqueFd duplicate(::fcntl(fd, F_DUPFD_CLOEXEC, 0));
        if (!duplicate.valid()) {
            return false;
        }
        duplicates.push_back(std::move(duplicate));
    }
    const std::uint64_t frameOffset = sentTotal_ + pendingOutputBytes();
    if (!appendFrame(message, out_, maxFrameSize_)) {
        return false;
    }
    if (!duplicates.empty()) {
        outFds_.push_back({frameOffset, std::move(duplicates)});
    }
    return true;
}

IoStatus Connection::flush(std::error_code& ec) {
    ec.clear();
    if (state_ != State::Connected) {
        return state_ == State::Connecting ? IoStatus::WouldBlock : IoStatus::Closed;
    }
    if (ssl_) {
        if (const IoStatus status = tlsHandshake(ec); status != IoStatus::Ok) {
            return status;
        }
        return flushTls(ec);
    }
    while (outPos_ < out_.size()) {
        std::size_t length = out_.size() - outPos_;

        // Ancillary data is delivered with the first byte of the sendmsg call that carries
        // it, so a call may contain at most one batch, and the batch's frame must start it.
        alignas(cmsghdr) std::array<char, CMSG_SPACE(kMaxFdsPerFrame * sizeof(int))> control{};
        msghdr message{};
        bool attached = false;
        if (!outFds_.empty()) {
            const FdBatch& batch = outFds_.front();
            if (batch.streamOffset == sentTotal_) {
                const std::size_t bytes = batch.fds.size() * sizeof(int);
                message.msg_control = control.data();
                message.msg_controllen = CMSG_SPACE(bytes);
                cmsghdr* header = CMSG_FIRSTHDR(&message);
                header->cmsg_level = SOL_SOCKET;
                header->cmsg_type = SCM_RIGHTS;
                header->cmsg_len = CMSG_LEN(bytes);
                std::array<int, kMaxFdsPerFrame> raw{};
                for (std::size_t i = 0; i < batch.fds.size(); ++i) {
                    raw.at(i) = batch.fds[i].get();
                }
                std::memcpy(CMSG_DATA(header), raw.data(), bytes);
                attached = true;
                if (outFds_.size() > 1) {
                    length = std::min<std::size_t>(length, outFds_[1].streamOffset - sentTotal_);
                }
            } else {
                length = std::min<std::size_t>(length, batch.streamOffset - sentTotal_);
            }
        }

        iovec io{out_.data() + outPos_, length};
        message.msg_iov = &io;
        message.msg_iovlen = 1;
        const ssize_t sent = ::sendmsg(socket_.get(), &message, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return IoStatus::WouldBlock;
            }
            if (errno == EPIPE || errno == ECONNRESET) {
                close();
                return IoStatus::Closed;
            }
            ec = lastError();
            close();
            return IoStatus::Error;
        }
        if (attached && sent > 0) {
            outFds_.pop_front();  // delivered; our duplicates are closed here
        }
        outPos_ += static_cast<std::size_t>(sent);
        sentTotal_ += static_cast<std::uint64_t>(sent);
    }
    out_.clear();
    outPos_ = 0;
    return IoStatus::Ok;
}

IoStatus Connection::receive(std::error_code& ec) {
    ec.clear();
    if (state_ != State::Connected) {
        return state_ == State::Connecting ? IoStatus::WouldBlock : IoStatus::Closed;
    }
    // Drop sent bytes from the front of the output buffer while we are here and idle.
    if (outPos_ >= kCompactThreshold && outPos_ >= out_.size() / 2) {
        out_.erase(out_.begin(), out_.begin() + static_cast<std::ptrdiff_t>(outPos_));
        outPos_ = 0;
    }

    if (scratch_.size() != kReceiveChunk) {
        scratch_.resize(kReceiveChunk);
    }
    if (ssl_) {
        if (const IoStatus status = tlsHandshake(ec); status != IoStatus::Ok) {
            return status;
        }
        return receiveTls(ec);
    }
    std::size_t total = 0;
    while (total < kReceiveBudgetPerCall) {
        alignas(cmsghdr) std::array<char, CMSG_SPACE(kMaxFdsPerFrame * sizeof(int))> control{};
        iovec io{scratch_.data(), scratch_.size()};
        msghdr message{};
        message.msg_iov = &io;
        message.msg_iovlen = 1;
        message.msg_control = control.data();
        message.msg_controllen = control.size();

        const ssize_t received = ::recvmsg(socket_.get(), &message, MSG_CMSG_CLOEXEC);
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return total > 0 ? IoStatus::Ok : IoStatus::WouldBlock;
            }
            if (errno == ECONNRESET) {
                if (total > 0) {
                    return IoStatus::Ok;  // deliver what arrived; the next call reports Closed
                }
                close();
                return IoStatus::Closed;
            }
            ec = lastError();
            close();
            return IoStatus::Error;
        }

        // Take ownership of any descriptors before anything else can fail.
        bool tooManyFds = false;
        for (cmsghdr* header = CMSG_FIRSTHDR(&message); header != nullptr;
             header = CMSG_NXTHDR(&message, header)) {
            if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS) {
                continue;
            }
            const std::size_t count = (header->cmsg_len - CMSG_LEN(0)) / sizeof(int);
            for (std::size_t i = 0; i < count; ++i) {
                int fd = -1;
                std::memcpy(&fd, CMSG_DATA(header) + i * sizeof(int), sizeof(int));
                UniqueFd owned(fd);
                if (receivedFds_.size() >= kMaxPendingReceivedFds) {
                    tooManyFds = true;  // `owned` closes it
                } else {
                    receivedFds_.push_back(std::move(owned));
                }
            }
        }
        if (tooManyFds || (message.msg_flags & MSG_CTRUNC) != 0) {
            ec = std::make_error_code(std::errc::too_many_files_open);
            close();
            return IoStatus::Error;
        }

        if (received == 0) {
            if (total > 0) {
                return IoStatus::Ok;
            }
            close();
            return IoStatus::Closed;
        }
        reader_.feed(std::span{scratch_}.first(static_cast<std::size_t>(received)));
        total += static_cast<std::size_t>(received);
    }
    return IoStatus::Ok;
}

std::vector<UniqueFd> Connection::takeReceivedFds() {
    return std::exchange(receivedFds_, {});
}

void Connection::close() {
    if (ssl_ && tlsReady_ && socket_.valid()) {
        (void)SSL_shutdown(ssl_.get());  // best effort: tell the peer, do not wait for it
    }
    ssl_.reset();
    tlsReady_ = false;
    socket_.reset();
    state_ = State::Closed;
    out_.clear();
    outPos_ = 0;
    outFds_.clear();
    // Frames already read stay available through next(); received fds stay until taken.
}

// --- Listener ---------------------------------------------------------------------------

Listener::~Listener() {
    close();
}

Listener::Listener(Listener&& other) noexcept
    : socket_(std::move(other.socket_)), endpoint_(std::move(other.endpoint_)) {}

Listener& Listener::operator=(Listener&& other) noexcept {
    if (this != &other) {
        close();
        socket_ = std::move(other.socket_);
        endpoint_ = std::move(other.endpoint_);
    }
    return *this;
}

void Listener::close() {
    if (socket_.valid() && endpoint_.kind == Endpoint::Kind::Unix) {
        (void)::unlink(endpoint_.address.c_str());
    }
    socket_.reset();
}

Listener Listener::listen(const Endpoint& endpoint, std::error_code& ec, int backlog) {
    ec.clear();
    Listener listener;

    if (endpoint.kind == Endpoint::Kind::Unix) {
        sockaddr_un address{};
        if (!fillUnixAddress(endpoint.address, address)) {
            ec = std::make_error_code(std::errc::filename_too_long);
            return listener;
        }
        // Create the parent directory (one level), private to the user.
        if (const std::size_t slash = endpoint.address.rfind('/');
            slash != std::string::npos && slash > 0) {
            const std::string parent = endpoint.address.substr(0, slash);
            if (::mkdir(parent.c_str(), 0700) != 0 && errno != EEXIST) {
                ec = lastError();
                return listener;
            }
        }
        // A socket file may be left over from a crashed server. If nobody answers, replace it.
        struct stat info{};
        if (::lstat(endpoint.address.c_str(), &info) == 0) {
            if (!S_ISSOCK(info.st_mode)) {
                ec = std::make_error_code(std::errc::file_exists);
                return listener;
            }
            const UniqueFd probe(::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0));
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
            const auto* raw = reinterpret_cast<const sockaddr*>(&address);
            if (probe.valid() && ::connect(probe.get(), raw, sizeof(address)) == 0) {
                ec = std::make_error_code(std::errc::address_in_use);
                return listener;
            }
            (void)::unlink(endpoint.address.c_str());
        }

        UniqueFd socket(::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
        if (!socket.valid()) {
            ec = lastError();
            return listener;
        }
        // Mode 0600 before bind, so the file never exists with wider permissions.
        (void)::fchmod(socket.get(), S_IRUSR | S_IWUSR);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        if (::bind(socket.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) !=
                0 ||
            ::chmod(endpoint.address.c_str(), S_IRUSR | S_IWUSR) != 0 ||
            ::listen(socket.get(), backlog) != 0) {
            ec = lastError();
            return listener;
        }
        listener.socket_ = std::move(socket);
        listener.endpoint_ = endpoint;
        return listener;
    }

    AddressList addresses;
    if (!resolve(endpoint, true, addresses, ec)) {
        return listener;
    }
    const addrinfo* info = addresses.head;
    UniqueFd socket(::socket(info->ai_family, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
    if (!socket.valid()) {
        ec = lastError();
        return listener;
    }
    const int one = 1;
    (void)::setsockopt(socket.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (::bind(socket.get(), info->ai_addr, info->ai_addrlen) != 0 ||
        ::listen(socket.get(), backlog) != 0) {
        ec = lastError();
        return listener;
    }
    // Report the port the system chose when 0 was requested.
    sockaddr_storage bound{};
    socklen_t length = sizeof(bound);
    std::uint16_t port = endpoint.port;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    if (::getsockname(socket.get(), reinterpret_cast<sockaddr*>(&bound), &length) == 0) {
        if (bound.ss_family == AF_INET) {
            sockaddr_in v4{};
            std::memcpy(&v4, &bound, sizeof(v4));
            port = ntohs(v4.sin_port);
        } else if (bound.ss_family == AF_INET6) {
            sockaddr_in6 v6{};
            std::memcpy(&v6, &bound, sizeof(v6));
            port = ntohs(v6.sin6_port);
        }
    }
    listener.socket_ = std::move(socket);
    listener.endpoint_ = Endpoint::tcp(endpoint.address, port, endpoint.tls);
    return listener;
}

std::optional<Connection> Listener::accept(std::error_code& ec, std::uint32_t maxFrameSize,
                                           std::shared_ptr<TlsContext> tls) {
    ec.clear();
    if (!socket_.valid()) {
        ec = std::make_error_code(std::errc::bad_file_descriptor);
        return std::nullopt;
    }
    for (;;) {
        UniqueFd peer(::accept4(socket_.get(), nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC));
        if (peer.valid()) {
            return Connection::adopt(std::move(peer), endpoint_.kind, maxFrameSize, std::move(tls));
        }
        if (errno == EINTR || errno == ECONNABORTED) {
            continue;
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ec = lastError();
        }
        return std::nullopt;
    }
}

}  // namespace lagrange::proto
