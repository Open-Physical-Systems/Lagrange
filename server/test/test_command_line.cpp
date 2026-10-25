// lagrange_server's command line: the defaults that depend on what else is given.
#include "CommandLine.hpp"

#include <array>
#include <string>

#include <gtest/gtest.h>

namespace lagrange::server {
namespace {

CommandLine parse(std::vector<std::string> args) {
    args.insert(args.begin(), "lagrange_server");
    std::vector<char*> argv;
    for (std::string& a : args) {
        argv.push_back(a.data());
    }
    CommandLine line = defaultCommandLine();
    const auto error = parseCommandLine(static_cast<int>(argv.size()), argv.data(), 1, false, line);
    EXPECT_FALSE(error) << *error;
    return line;
}

TEST(CommandLine, AScenarioWithoutAWorldDrivesOnItsOwnRoads) {
    // Without --world the default is the sample warehouse, except with a scenario: its
    // OpenDRIVE network is then the world (the warehouse has no road lanes for its entities).
    EXPECT_FALSE(parse({}).options.worldPath.empty());
    EXPECT_TRUE(
        parse({"--robot", "car.usda", "--scenario", "cut_in.xosc"}).options.worldPath.empty());
    EXPECT_EQ(parse({"--scenario", "cut_in.xosc", "--world", "city.usda"}).options.worldPath,
              "city.usda");
}

}  // namespace
}  // namespace lagrange::server
