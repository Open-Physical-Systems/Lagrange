// The ROS 2 domain of a test that talks over DDS, added to a test target by
// lagrange_test_ros_domain() (LagrangeCompileOptions.cmake).
//
// Before main() this sets ROS_DOMAIN_ID to the test's own default only when the environment
// does not already name one, so a plain `colcon test` keeps each test on its usual domain while
// a CI job or agent that exports its own ROS_DOMAIN_ID keeps its tests apart from everyone
// else's on the same machine.
#include <cstdlib>
#include <string>

#ifndef LAGRANGE_TEST_ROS_DOMAIN_DEFAULT
#error "lagrange_test_ros_domain() defines LAGRANGE_TEST_ROS_DOMAIN_DEFAULT"
#endif

namespace {

// getenv and setenv are not thread safe (concurrency-mt-unsafe); this runs before main(), when
// the test process has no other threads.
const bool kRosDomainSet = [] {
    const char* domain = std::getenv("ROS_DOMAIN_ID");  // NOLINT(concurrency-mt-unsafe)
    if (domain == nullptr || std::string(domain).empty()) {
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        ::setenv("ROS_DOMAIN_ID", LAGRANGE_TEST_ROS_DOMAIN_DEFAULT, 1);
    }
    return true;
}();

}  // namespace
