// v0.2 acceptance criterion 4: disabling any domain that has a module, in favour of its
// substitute, keeps the simulation running. Every combination of the domains with modules is
// wired (so each substitute provides what the other modules consume) and run for 2 s.
#include "lagrange/server/Server.hpp"

#include <filesystem>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

TEST(DomainToggles, EveryCombinationOfSubstitutesRuns) {
    const std::vector<std::string> domains{"rigid_body", "sensors"};
    const std::filesystem::path dir =
        std::filesystem::path(::testing::TempDir()) / "lagrange_toggles";
    std::filesystem::create_directories(dir);
    for (unsigned mask = 0; mask < (1U << domains.size()); ++mask) {
        ServerOptions options;
        options.endpoint =
            proto::Endpoint::unixSocket((dir / ("s" + std::to_string(mask) + ".sock")).string());
        options.robotPath = LAGRANGE_TEST_AMR_URDF;
        options.worldPath = LAGRANGE_TEST_WAREHOUSE_URDF;
        options.realTimeFactor = 0.0;
        std::string label;
        for (std::size_t d = 0; d < domains.size(); ++d) {
            if ((mask & (1U << d)) != 0) {
                options.substituted.insert(domains[d]);
                label += domains[d] + " off ";
            }
        }
        SCOPED_TRACE(label.empty() ? "all on" : label);
        Server server(options);
        const Status started = server.start();
        ASSERT_TRUE(started.ok()) << started.message;
        while (server.simTime().ns < 2'000'000'000) {
            server.runOnce(0);
            ASSERT_TRUE(server.running()) << "the run stopped at t = " << server.simTime().ns;
        }
    }
}

}  // namespace
}  // namespace lagrange::server
