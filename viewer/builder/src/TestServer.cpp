// Stub (RFC 0016 WP0): WP-F replaces this file with posix_spawn, waitpid(WNOHANG) and the
// SIGTERM-then-SIGKILL stop of RFC 0016 §7.1.
#include "lagrange/viewer/builder/TestServer.hpp"

namespace lagrange::viewer::builder {

struct TestServer::Process {
    State state{State::Idle};
};

TestServer::TestServer() : process_(std::make_unique<Process>()) {}

TestServer::~TestServer() = default;

std::string TestServer::start(const TestRequest& /*request*/) {
    return "Test is not implemented yet (RFC 0016 WP-F)";
}

TestServer::State TestServer::poll() {
    return process_->state;
}

void TestServer::requestStop() {}

TestServer::State TestServer::state() const {
    return process_->state;
}

int TestServer::exitStatus() const {
    return -1;
}

std::string TestServer::logTail(std::size_t /*bytes*/) const {
    return {};
}

}  // namespace lagrange::viewer::builder
