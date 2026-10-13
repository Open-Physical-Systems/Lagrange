#include "lagrange/viewer/ProfileSession.hpp"

#include <gtest/gtest.h>

namespace lagrange::viewer {
namespace {

std::shared_ptr<const ProfileInfo> profile(std::uint64_t serial = 1) {
    auto info = std::make_shared<ProfileInfo>();
    info->serial = serial;
    info->yaml =
        "schema_version: 1\n"
        "stage: a.usda\n"
        "\n"
        "domains:\n"
        "  rigid_body: {enabled: true, backend: mujoco, dt: 0.001}\n"
        "  aerodynamics: {enabled: true, fidelity: drag_plus_field, dt: 0.001}\n"
        "  radiation: {enabled: false, substitute: zero}\n";
    DomainInfo rigid;
    rigid.name = "rigid_body";
    rigid.enabled = true;
    rigid.backends = {"mujoco", "jolt"};
    rigid.dtNs = 1'000'000;
    rigid.substitutes = {{"fixed", "ideal", "held in place", false},
                         {"replay", "replay", "recorded poses", true}};
    DomainInfo aero;
    aero.name = "aerodynamics";
    aero.enabled = true;
    aero.fidelities = {"drag", "drag_plus_field"};
    aero.dtNs = 1'000'000;
    aero.substitutes = {{"zero", "ideal", "no drag", false}};
    DomainInfo radiation;
    radiation.name = "radiation";
    radiation.dtNs = 1'000'000'000;
    radiation.substitutes = {{"zero", "ideal", "no dose", false}};
    DomainInfo comms;
    comms.name = "comms";
    comms.dtNs = 10'000'000;
    comms.substitutes = {{"ideal", "ideal", "perfect link", false}};
    DomainInfo rotor;
    rotor.name = "rotor_aero";
    rotor.available = false;
    info->domains = {rigid, aero, radiation, comms, rotor};
    return info;
}

std::string line(const ProfileSession& session, std::string_view start) {
    for (const std::string& text : session.draft().lines()) {
        if (text.starts_with(start)) {
            return text;
        }
    }
    return {};
}

TEST(ProfileSession, HasNothingBeforeTheServersProfile) {
    ProfileSession session;
    EXPECT_EQ(session.validity(), ProfileSession::Validity::None);
    EXPECT_TRUE(session.domains().empty());
    EXPECT_FALSE(session.setEnabled("thermal", false));
    EXPECT_FALSE(session.canApply());
}

TEST(ProfileSession, DescribesEveryDomainTheServerListed) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_EQ(session.domains().size(), 5U);
    const DomainView* rigid = session.find("rigid_body");
    ASSERT_NE(rigid, nullptr);
    EXPECT_TRUE(rigid->enabled && rigid->inProfile && !rigid->pending);
    EXPECT_EQ(rigid->mode, "mujoco");
    EXPECT_EQ(rigid->dtNs, 1'000'000);
    const DomainView* radiation = session.find("radiation");
    EXPECT_FALSE(radiation->enabled);
    EXPECT_EQ(radiation->mode, "zero");
    EXPECT_FALSE(session.find("comms")->inProfile);
}

// The acceptance check of the Domains screen: switching aerodynamics off writes a line that
// names a substitute. mock_server/test/test_profile.py checks that this text is a valid profile.
TEST(ProfileSession, TurningADomainOffNamesItsSubstitute) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setEnabled("aerodynamics", false));
    EXPECT_EQ(line(session, "  aerodynamics"),
              "  aerodynamics: {enabled: false, substitute: zero}");
    const DomainView* aero = session.find("aerodynamics");
    EXPECT_FALSE(aero->enabled);
    EXPECT_TRUE(aero->pending);
    EXPECT_EQ(aero->mode, "zero");
    EXPECT_EQ(session.draft().changedLineCount(), 1U);
}

TEST(ProfileSession, TurningItBackOnRestoresTheAppliedSettings) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setEnabled("aerodynamics", false));
    ASSERT_TRUE(session.setEnabled("aerodynamics", true));
    EXPECT_FALSE(session.draft().dirty());
    EXPECT_FALSE(session.find("aerodynamics")->pending);
}

TEST(ProfileSession, TurningOnADisabledDomainUsesTheServersDefaults) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setEnabled("radiation", true));
    EXPECT_EQ(line(session, "  radiation"), "  radiation: {enabled: true, dt: 1}");
    ASSERT_TRUE(session.setEnabled("comms", true));
    EXPECT_EQ(line(session, "  comms"), "  comms: {enabled: true, dt: 0.01}");
    EXPECT_TRUE(session.find("comms")->inProfile);
}

TEST(ProfileSession, AnUnavailableDomainCannotBeEnabled) {
    ProfileSession session;
    session.sync(profile());
    EXPECT_FALSE(session.setEnabled("rotor_aero", true));
    EXPECT_FALSE(session.draft().dirty());
}

TEST(ProfileSession, ModeIsTheBackendOrFidelityWhenEnabled) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setMode("rigid_body", "jolt"));
    EXPECT_EQ(line(session, "  rigid_body"),
              "  rigid_body: {enabled: true, backend: jolt, dt: 0.001}");
    ASSERT_TRUE(session.setMode("aerodynamics", "drag"));
    EXPECT_EQ(session.find("aerodynamics")->mode, "drag");
}

TEST(ProfileSession, ModeIsTheSubstituteWhenDisabledAndReplayKeepsItsSource) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setEnabled("rigid_body", false));
    EXPECT_EQ(session.find("rigid_body")->mode, "fixed");
    ASSERT_TRUE(session.setMode("rigid_body", "replay"));
    EXPECT_TRUE(session.find("rigid_body")->needsSource);
    ASSERT_TRUE(session.setSource("rigid_body", "logs/real run.mcap"));
    EXPECT_EQ(line(session, "  rigid_body"),
              "  rigid_body: {enabled: false, substitute: replay, source: \"logs/real run.mcap\"}");
    EXPECT_EQ(session.find("rigid_body")->source, "logs/real run.mcap");
    ASSERT_TRUE(session.setMode("rigid_body", "fixed"));
    EXPECT_EQ(line(session, "  rigid_body"), "  rigid_body: {enabled: false, substitute: fixed}");
    EXPECT_FALSE(session.setMode("rigid_body", "ground_truth"));  // not offered for this domain
}

TEST(ProfileSession, StepIsWrittenAsExactSeconds) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setStep("rigid_body", 500'000));
    EXPECT_EQ(line(session, "  rigid_body"),
              "  rigid_body: {enabled: true, backend: mujoco, dt: 0.0005}");
    EXPECT_EQ(session.find("rigid_body")->dtNs, 500'000);
    EXPECT_FALSE(session.setStep("rigid_body", 0));
}

TEST(ProfileSession, RevertAndANewServerProfileDropEdits) {
    ProfileSession session;
    session.sync(profile());
    ASSERT_TRUE(session.setEnabled("aerodynamics", false));
    session.revert();
    EXPECT_FALSE(session.draft().dirty());
    EXPECT_TRUE(session.find("aerodynamics")->enabled);

    ASSERT_TRUE(session.setEnabled("aerodynamics", false));
    session.sync(profile(1));  // the same profile again: edits stay
    EXPECT_TRUE(session.draft().dirty());
    session.sync(profile(2));
    EXPECT_FALSE(session.draft().dirty());
}

TEST(ProfileSession, IsCheckingUntilTheServerAnswers) {
    ProfileSession session;
    session.sync(profile());
    EXPECT_EQ(session.validity(), ProfileSession::Validity::Checking);
    EXPECT_TRUE(session.issues().empty());
    EXPECT_FALSE(session.domainHasError("thermal"));
    EXPECT_FALSE(session.canApply());
}

}  // namespace
}  // namespace lagrange::viewer
