// lagrange_server's command line: the defaults that depend on what else is given.
#include "CommandLine.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

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

/// The error for these arguments, or nothing when they were taken.
std::optional<std::string> refusal(std::vector<std::string> args) {
    args.insert(args.begin(), "lagrange_server");
    std::vector<char*> argv;
    for (std::string& a : args) {
        argv.push_back(a.data());
    }
    CommandLine line = defaultCommandLine();
    return parseCommandLine(static_cast<int>(argv.size()), argv.data(), 1, false, line);
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

// QA 2026-11-12: numbers that are not finite, do not fit, or trail text were taken, and some
// (--ambient-temperature inf, --tether ...,inf) made MuJoCo reset at t = 0.
TEST(CommandLine, RefusesNumbersThatAreNotFiniteOrDoNotFit) {
    for (const std::vector<std::string>& args : std::vector<std::vector<std::string>>{
             {"--rtf", "inf"},
             {"--ambient-temperature", "inf"},
             {"--air-density", "inf"},
             {"--geo-origin", "nan,8,400"},
             {"--geo-origin", "47,nan,400"},
             {"--geo-origin", "47,8,inf"},
             {"--humans", "nan"},
             {"--humans", "inf"},
             {"--humans", "1e12"},
             {"--humans", "3,radius=inf"},
             {"--traffic", "vehicles=99999999999"},
             {"--joint-loop-hz", "nan"},
             {"--joint-loop-hz", "inf"},
             {"--joint-loop-hz", "5abc"},
             {"--emi-motor-leads", "0.1junk"},
             {"--emi-motor-leads", "inf"},
             {"--emi-motor-leads", "0.1,1,2"},
             {"--tether", "0,0,3,base_link,inf"},
             {"--weather", "temperature=nan,pressure=101325,relative_humidity=0.5"},
             {"--spawn", "1,2,3,4,5"},
             {"--spawn", "1,2,"},
         }) {
        EXPECT_TRUE(refusal(args)) << args[0] << " " << args[1];
    }
    // What they guard still parses.
    EXPECT_EQ(parse({"--spawn", "1,2,3,0.5"}).options.spawn.position[2], 3.0);
    EXPECT_EQ(parse({"--joint-loop-hz", "12.5"}).options.jointLoopHz, 12.5);
    EXPECT_EQ(parse({"--humans", "3,seed=4294967296,radius=7"}).options.humanSeed, 4294967296U);
    EXPECT_EQ(*parse({"--emi-motor-leads", "0.01,0.1,0,0.05"}).options.emiMotorLeads, 0.01);
    EXPECT_EQ(parse({"--tether", "0,0,3,base_link,12"}).options.tethers.at(0).length, 12.0);
    EXPECT_EQ(parse({"--traffic", "vehicles=3,seed=99999999999"}).options.trafficSeed,
              99999999999U);
}

}  // namespace
}  // namespace lagrange::server
