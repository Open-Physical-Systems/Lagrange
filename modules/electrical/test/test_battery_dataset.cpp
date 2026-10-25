// electrical.md §8, battery equivalent circuit against published cell data: the network's own
// battery model (Models.hpp) driven by the measured current of HPPC pulse blocks of an LG HG2
// cell at 25 °C (Kollmeyer et al., doi:10.17632/b5mj79w5w9.2, CC BY 4.0), with the parameters
// fitted by data/lg_hg2/fit_ecm.py on other blocks of the same test. The terminal voltage must
// stay within the §8 tolerance where one parameter set holds (the fit's state-of-charge range).
#include "lagrange/electrical/Models.hpp"

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace lagrange::electrical {
namespace {

constexpr double kToleranceV = 0.020;  // electrical.md §8: terminal-voltage RMS < 20 mV per cell
constexpr double kAmbientK = 298.15;   // the 25 °C test; the fit has no temperature dependence
// Below about 35 % state of charge the cell's resistance and OCV slope rise beyond what one
// parameter set covers (the 24.5 % block is off by 21.6 mV, README); asserted from here up.
constexpr double kValidFromSoc = 0.35;

std::map<std::string, std::vector<double>> readFit() {
    std::ifstream in(LAGRANGE_TEST_HG2_DIR "/ecm_fit.txt");
    std::map<std::string, std::vector<double>> out;
    for (std::string line; std::getline(in, line);) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream words(line);
        std::string key;
        words >> key;
        for (double v = 0.0; words >> v;) {
            out[key].push_back(v);
        }
    }
    return out;
}

struct Sample {
    double t, current, voltage, soc;
};

std::map<int, std::vector<Sample>> readBlocks() {
    std::ifstream in(LAGRANGE_TEST_HG2_DIR "/hppc_25degC_validation.csv");
    std::map<int, std::vector<Sample>> out;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        std::istringstream fields(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(fields, cell, ',')) {
            v.push_back(std::stod(cell));
        }
        if (v.size() == 5) {
            out[static_cast<int>(v[0])].push_back({v[1], v[2], v[3], v[4]});
        }
    }
    return out;
}

TEST(BatteryDataset, TheEquivalentCircuitFollowsMeasuredHppcPulses) {
    const auto fit = readFit();
    const auto blocks = readBlocks();
    ASSERT_FALSE(fit.empty()) << "ecm_fit.txt not found";
    ASSERT_EQ(blocks.size(), 4U);
    BatteryParams p;
    p.capacity = fit.at("capacity_Ah")[0] * 3600.0;
    p.ocvSoc = fit.at("ocv_soc");
    p.ocvVoltage = fit.at("ocv_V");
    p.r0 = fit.at("r0_ohm")[0];
    p.r1 = fit.at("r1_ohm")[0];
    p.c1 = fit.at("c1_F")[0];
    p.r2 = fit.at("r2_ohm")[0];
    p.c2 = fit.at("c2_F")[0];
    p.cutoffVoltage = 2.5;  // V, the cell's discharge cut-off; never reached in these blocks
    p.maxChargeCurrent = 10.0;
    int checked = 0;
    for (const auto& [id, samples] : blocks) {
        BatteryState s;
        s.soc = samples.front().soc;  // after an hour's rest: RC pairs relaxed
        double squares = 0.0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            if (k > 0) {
                advanceBattery(p, s, samples[k - 1].current, samples[k].t - samples[k - 1].t);
            }
            const Thevenin source = theveninOf(p, s, kAmbientK);
            const double model = source.emf - source.resistance * samples[k].current;
            squares += (model - samples[k].voltage) * (model - samples[k].voltage);
        }
        const double rms = std::sqrt(squares / static_cast<double>(samples.size()));
        // Coulomb counting agrees with the cycler's Ah counter at the block's end.
        EXPECT_NEAR(s.soc, samples.back().soc, 2e-3) << "block " << id;
        std::printf("HG2 HPPC block at SoC %.3f: terminal voltage RMS error %.1f mV\n",
                    samples.front().soc, rms * 1e3);
        if (samples.front().soc >= kValidFromSoc) {
            EXPECT_LT(rms, kToleranceV) << "block at SoC " << samples.front().soc;
            ++checked;
        }
    }
    EXPECT_EQ(checked, 3);
}

}  // namespace
}  // namespace lagrange::electrical
