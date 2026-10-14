// Validation of MuJoCo against closed-form mechanics: the cases of
// docs/design/domains/rigid_body.md section 7 that do not need the backend module itself.
// Each test builds a small MJCF scene, steps it at 1 ms, and compares with the analytic result.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

namespace {

constexpr double kGravity = 9.81;  // m/s^2, set in every scene below
constexpr double kStep = 0.001;    // s, the rigid-body step of docs/design/domains/rigid_body.md §4
constexpr int kTenSeconds = 10'000;  // steps

struct ModelDeleter {
    void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct DataDeleter {
    void operator()(mjData* d) const { mj_deleteData(d); }
};

/// A compiled scene and its state.
struct Scene {
    std::unique_ptr<mjModel, ModelDeleter> model;
    std::unique_ptr<mjData, DataDeleter> data;

    [[nodiscard]] mjModel* m() const { return model.get(); }
    [[nodiscard]] mjData* d() const { return data.get(); }
    void step(int count) const {
        for (int i = 0; i < count; ++i) {
            mj_step(m(), d());
        }
    }
    /// Potential plus kinetic energy of the current state, J.
    [[nodiscard]] double energy() const {
        mj_forward(m(), d());
        return d()->energy[0] + d()->energy[1];
    }
};

Scene load(const std::string& xml) {
    char error[1024] = "";
    mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
    EXPECT_NE(spec, nullptr) << error;
    if (spec == nullptr) {
        return {};
    }
    mjModel* model = mj_compile(spec, nullptr);
    EXPECT_NE(model, nullptr) << mjs_getError(spec);
    mj_deleteSpec(spec);
    if (model == nullptr) {
        return {};
    }
    Scene scene{std::unique_ptr<mjModel, ModelDeleter>(model),
                std::unique_ptr<mjData, DataDeleter>(mj_makeData(model))};
    return scene;
}

/// A number for RecordProperty, in scientific notation so small errors stay visible.
std::string sci(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.3e", value);
    return text;
}

std::string option(const std::string& integrator, const std::string& extra = "") {
    return "<option timestep=\"0.001\" gravity=\"0 0 -9.81\" integrator=\"" + integrator + "\" " +
           extra + "><flag energy=\"enable\"/></option>";
}

// --- free fall ----------------------------------------------------------------------------

std::string freeFall(const std::string& integrator) {
    return "<mujoco>" + option(integrator) +
           R"(<worldbody><body><freejoint/>
                <geom type="sphere" size="0.1" mass="1" contype="0" conaffinity="0"/>
              </body></worldbody></mujoco>)";
}

TEST(FreeFall, Rk4IsExactForConstantAcceleration) {
    const Scene scene = load(freeFall("RK4"));
    ASSERT_TRUE(scene.model);
    scene.step(kTenSeconds);
    const double t = kTenSeconds * kStep;
    EXPECT_NEAR(scene.d()->qpos[2], -0.5 * kGravity * t * t, 1e-6);
    EXPECT_NEAR(scene.d()->qvel[2], -kGravity * t, 1e-9);
}

TEST(FreeFall, SemiImplicitEulerLagsByHalfAStep) {
    // Semi-implicit Euler (v first, then x with the new v) gives x_n = -g dt^2 n (n + 1) / 2:
    // the closed form plus -g t dt / 2, 4.9 cm after 10 s at 1 ms. This pins down which
    // integrator MuJoCo's default is and how large its position error is.
    const Scene scene = load(freeFall("Euler"));
    ASSERT_TRUE(scene.model);
    scene.step(kTenSeconds);
    const double n = kTenSeconds;
    EXPECT_NEAR(scene.d()->qpos[2], -kGravity * kStep * kStep * n * (n + 1.0) / 2.0, 1e-6);
}

// --- pendulum -----------------------------------------------------------------------------

/// Complete elliptic integral of the first kind K(k), by the arithmetic-geometric mean.
double ellipticK(double k) {
    double a = 1.0;
    double b = std::sqrt(1.0 - k * k);
    for (int i = 0; i < 30 && std::abs(a - b) > 1e-16 * a; ++i) {
        const double next = 0.5 * (a + b);
        b = std::sqrt(a * b);
        a = next;
    }
    return std::numbers::pi / (2.0 * a);
}

// A point mass of 1 kg on a massless rod of 1 m. The tiny rotational inertia (1e-9 kg m^2) is
// required by MuJoCo and changes the period by about 5e-10.
std::string pendulum(const std::string& integrator) {
    return "<mujoco>" + option(integrator, "") +
           R"(<worldbody><body>
                <joint type="hinge" axis="0 1 0"/>
                <inertial pos="0 0 -1" mass="1" diaginertia="1e-9 1e-9 1e-9"/>
              </body></worldbody></mujoco>)";
}

struct PendulumRun {
    double maxEnergyDrift{0.0};  // |E - E0| / (m g L)
    double period{0.0};          // s, mean over the run
};

PendulumRun runPendulum(const std::string& integrator, double amplitude) {
    const Scene scene = load(pendulum(integrator));
    PendulumRun run;
    if (!scene.model) {
        return run;
    }
    scene.d()->qpos[0] = amplitude;
    const double initial = scene.energy();
    std::vector<double> crossings;  // times the angle passes zero going down
    double previous = scene.d()->qpos[0];
    for (int i = 1; i <= kTenSeconds; ++i) {
        scene.step(1);
        const double angle = scene.d()->qpos[0];
        if (previous > 0.0 && angle <= 0.0) {
            crossings.push_back((i - 1 + previous / (previous - angle)) * kStep);
        }
        previous = angle;
        if (i % 10 == 0) {
            const double drift = std::abs(scene.energy() - initial) / (1.0 * kGravity * 1.0);
            run.maxEnergyDrift = std::max(run.maxEnergyDrift, drift);
        }
    }
    if (crossings.size() >= 2) {
        run.period =
            (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
    }
    return run;
}

TEST(Pendulum, EnergyDriftAndLargeAnglePeriod) {
    constexpr double kAmplitude = 1.0;  // rad, far outside the small-angle approximation
    const double exactPeriod =
        4.0 * std::sqrt(1.0 / kGravity) * ellipticK(std::sin(kAmplitude / 2.0));
    const PendulumRun rk4 = runPendulum("RK4", kAmplitude);
    // rigid_body.md §7: energy drift below 0.1 % over 10 s at 1 ms.
    EXPECT_LT(rk4.maxEnergyDrift, 1e-3);
    EXPECT_NEAR(rk4.period / exactPeriod, 1.0, 1e-5);
    RecordProperty("rk4_energy_drift", sci(rk4.maxEnergyDrift));
    RecordProperty("rk4_period_error", sci(rk4.period / exactPeriod - 1.0));
}

TEST(Pendulum, DefaultIntegratorsReported) {
    // Recorded for the spike report: the backend picks its integrator from these numbers.
    for (const char* integrator : {"Euler", "implicit", "implicitfast"}) {
        const PendulumRun run = runPendulum(integrator, 1.0);
        RecordProperty(std::string(integrator) + "_energy_drift", sci(run.maxEnergyDrift));
        std::printf("%-13s energy drift %.3e of m g L over 10 s\n", integrator, run.maxEnergyDrift);
        EXPECT_LT(run.maxEnergyDrift, 0.05) << integrator;
    }
}

// --- double pendulum ----------------------------------------------------------------------

TEST(DoublePendulum, Rk4EnergyDrift) {
    const Scene scene = load("<mujoco>" + option("RK4") + R"(<worldbody>
          <body><joint type="hinge" axis="0 1 0"/>
            <inertial pos="0 0 -1" mass="1" diaginertia="1e-9 1e-9 1e-9"/>
            <body pos="0 0 -1"><joint type="hinge" axis="0 1 0"/>
              <inertial pos="0 0 -1" mass="1" diaginertia="1e-9 1e-9 1e-9"/>
            </body>
          </body></worldbody></mujoco>)");
    ASSERT_TRUE(scene.model);
    scene.d()->qpos[0] = 1.0;
    scene.d()->qpos[1] = 0.5;
    const double initial = scene.energy();
    const double scale = 2.0 * kGravity * 2.0;  // m g L of the whole chain
    double drift = 0.0;
    for (int i = 0; i < kTenSeconds / 10; ++i) {
        scene.step(10);
        drift = std::max(drift, std::abs(scene.energy() - initial) / scale);
    }
    EXPECT_LT(drift, 1e-3);
    RecordProperty("rk4_energy_drift", sci(drift));
}

// --- sliding block ------------------------------------------------------------------------

/// Whether a box on a plane with Coulomb friction mu slides when the slope is `degrees`.
/// The plane stays horizontal and gravity is tilted instead, which is the same mechanics.
bool slides(double degrees, double mu, const std::string& cone) {
    const double angle = degrees * std::numbers::pi / 180.0;
    const std::string gravity = std::to_string(kGravity * std::sin(angle)) + " 0 " +
                                std::to_string(-kGravity * std::cos(angle));
    const std::string friction = std::to_string(mu) + " 0.005 0.0001";
    const Scene scene =
        load("<mujoco><option timestep=\"0.001\" cone=\"" + cone + "\" gravity=\"" + gravity +
             "\"/><worldbody><geom type=\"plane\" size=\"5 5 0.1\" friction=\"" + friction +
             "\"/><body pos=\"0 0 0.1\"><freejoint/><geom type=\"box\" size=\"0.1 0.1 0.1\" "
             "mass=\"1\" friction=\"" +
             friction + "\"/></body></worldbody></mujoco>");
    if (!scene.model) {
        return false;
    }
    scene.step(2000);
    return std::abs(scene.d()->qpos[0]) > 0.01;  // more than 1 cm in 2 s
}

double stickSlipAngle(double mu, const std::string& cone) {
    for (double degrees = 15.0; degrees < 45.0; degrees += 0.1) {
        if (slides(degrees, mu, cone)) {
            return degrees;
        }
    }
    return 90.0;
}

TEST(Incline, EllipticConeSlipsAtTheCoulombAngle) {
    constexpr double kMu = 0.5;
    const double coulomb = std::atan(kMu) * 180.0 / std::numbers::pi;  // 26.565 degrees
    const double measured = stickSlipAngle(kMu, "elliptic");
    std::printf("elliptic  cone: slips at %.1f deg, Coulomb %.3f deg\n", measured, coulomb);
    RecordProperty("elliptic_slip_angle", std::to_string(measured));
    // rigid_body.md §7: within 1 degree. The backend therefore uses cone="elliptic".
    EXPECT_NEAR(measured, coulomb, 1.0);
}

TEST(Incline, PyramidalConeSlipsEarly) {
    // MuJoCo's default pyramidal cone approximates the friction cone and lets the block slide
    // about 1.5 degrees before the Coulomb angle, outside the tolerance of rigid_body.md §7.
    // Kept as a test so a MuJoCo upgrade that changes this is noticed.
    constexpr double kMu = 0.5;
    const double coulomb = std::atan(kMu) * 180.0 / std::numbers::pi;
    const double measured = stickSlipAngle(kMu, "pyramidal");
    std::printf("pyramidal cone: slips at %.1f deg, Coulomb %.3f deg\n", measured, coulomb);
    RecordProperty("pyramidal_slip_angle", std::to_string(measured));
    EXPECT_LT(measured, coulomb - 1.0);
}

// --- determinism --------------------------------------------------------------------------

TEST(Determinism, TwoRunsAreBitIdentical) {
    const std::string xml = "<mujoco>" + option("Euler") + R"(<worldbody>
          <geom type="plane" size="5 5 0.1"/>
          <body pos="0 0 0.5"><freejoint/><geom type="box" size="0.1 0.2 0.05" mass="1"/></body>
          <body pos="0.05 0 0.8"><freejoint/><geom type="capsule" size="0.05 0.2" mass="2"/></body>
        </worldbody></mujoco>)";
    std::vector<std::vector<double>> finals;
    for (int run = 0; run < 2; ++run) {
        const Scene scene = load(xml);
        ASSERT_TRUE(scene.model);
        scene.d()->qvel[3] = 2.0;  // spin the box so contacts are rich
        scene.step(5000);
        std::vector<double> state(scene.d()->qpos, scene.d()->qpos + scene.m()->nq);
        state.insert(state.end(), scene.d()->qvel, scene.d()->qvel + scene.m()->nv);
        finals.push_back(state);
    }
    ASSERT_EQ(finals[0].size(), finals[1].size());
    EXPECT_EQ(std::memcmp(finals[0].data(), finals[1].data(), finals[0].size() * sizeof(double)),
              0);
}

// --- joint-space inertia ------------------------------------------------------------------

TEST(Inertia, TwoLinkArmMatchesTheAnalyticMatrix) {
    // Planar arm about z. Link i: mass m_i, centre of mass c_i from its joint, inertia I_i
    // about its centre of mass; link 2's joint is l1 from link 1's.
    constexpr double m1 = 2.0, c1 = 0.3, i1 = 0.05, l1 = 0.6;
    constexpr double m2 = 1.5, c2 = 0.25, i2 = 0.03, q2 = 0.7;
    const Scene scene = load(R"(<mujoco><option gravity="0 0 0"/><worldbody>
          <body><joint type="hinge" axis="0 0 1"/>
            <inertial pos="0.3 0 0" mass="2" diaginertia="0.03 0.03 0.05"/>
            <body pos="0.6 0 0"><joint type="hinge" axis="0 0 1"/>
              <inertial pos="0.25 0 0" mass="1.5" diaginertia="0.02 0.02 0.03"/>
            </body>
          </body></worldbody></mujoco>)");
    ASSERT_TRUE(scene.model);
    scene.d()->qpos[1] = q2;
    mj_forward(scene.m(), scene.d());
    double full[4] = {};
    mj_fullM(scene.m(), scene.d(), full);

    const double m11 =
        i1 + i2 + m1 * c1 * c1 + m2 * (l1 * l1 + c2 * c2 + 2.0 * l1 * c2 * std::cos(q2));
    const double m12 = i2 + m2 * (c2 * c2 + l1 * c2 * std::cos(q2));
    const double m22 = i2 + m2 * c2 * c2;
    // rigid_body.md §7: 1e-9 relative.
    EXPECT_NEAR(full[0] / m11, 1.0, 1e-9);
    EXPECT_NEAR(full[1] / m12, 1.0, 1e-9);
    EXPECT_NEAR(full[2] / m12, 1.0, 1e-9);
    EXPECT_NEAR(full[3] / m22, 1.0, 1e-9);
}

}  // namespace
