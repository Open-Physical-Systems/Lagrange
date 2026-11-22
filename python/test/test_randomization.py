"""Domain randomisation (v1.6.0 plan A1): each parameter changes the physics as a closed form
says, draws repeat per seed, and an episode never keeps the last one's values."""

from __future__ import annotations

import math

import lagrange
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from lagrange import Simulation
from lagrange.envs import LagrangeEnv, ParallelEnvs
from lagrange.randomization import Randomizer, check

G = 9.80665  # m/s², standard gravity, the run's default (World::gravityLocal)

BOX = """<robot name="box">
  <link name="box">
    <inertial><origin xyz="0 0 0"/><mass value="2.0"/>
      <inertia ixx="0.0133" iyy="0.0133" izz="0.0133" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
</robot>
"""

# A cart on a horizontal slide: its joint's damping and friction act alone along the axis.
SLIDER = """<robot name="slider">
  <link name="world"/>
  <link name="rail"><inertial><mass value="1"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="rail"/>
    <origin xyz="0 0 1"/></joint>
  <link name="cart"><inertial><mass value="2.0"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="slide" type="prismatic"><parent link="rail"/><child link="cart"/>
    <axis xyz="1 0 0"/><limit lower="-100" upper="100" effort="100" velocity="100"/>
    <dynamics damping="1.0" friction="0.0"/></joint>
</robot>
"""


@pytest.fixture(scope="module")
def box(tmp_path_factory: pytest.TempPathFactory) -> str:
    path = tmp_path_factory.mktemp("robots") / "box.urdf"
    path.write_text(BOX)
    return str(path)


@pytest.fixture(scope="module")
def slider(tmp_path_factory: pytest.TempPathFactory) -> str:
    path = tmp_path_factory.mktemp("robots") / "slider.urdf"
    path.write_text(SLIDER)
    return str(path)


def arm() -> str:
    return get_package_share_directory("lagrange_assets") + "/robots/arm6/arm6.urdf"


def quad() -> Simulation:
    return Simulation(
        lagrange.sample_quadcopter(),
        world=None,
        profile=lagrange.sample_profile("quad_x500_hover"),
    )


def mean_of(
    sim: Simulation, signal: str, until: float, samples: int = 50, step: float = 0.01
) -> np.ndarray:
    """The mean of a signal over `samples` reads, `step` s apart, ending at `until`."""
    values = []
    for i in range(samples):
        sim.step_until(until - (samples - 1 - i) * step)
        values.append(sim.read(signal))
    return np.mean(np.asarray(values), axis=0)


# --- The keys and the API -------------------------------------------------------------------


def test_unknown_keys_and_bad_ranges_are_refused_by_name() -> None:
    with pytest.raises(ValueError, match="no key 'masss'"):
        check({"masss": (0.8, 1.2)})
    with pytest.raises(ValueError, match="low <= high"):
        check({"mass": (1.2, 0.8)})
    with pytest.raises(ValueError, match="no entry colour"):
        check({"mass": {"scale": (0.8, 1.2), "colour": "red"}})
    with pytest.raises(ValueError, match="scale or a value"):
        check({"friction": {"bodies": "*"}})
    with pytest.raises(ValueError, match="a dict"):
        check({"push": (1, 2)})
    with pytest.raises(ValueError, match="force"):
        check({"push": {"count": 2}})
    with pytest.raises(ValueError, match="no key"):
        LagrangeEnv(arm(), randomize={"mass_scale": (0.8, 1.2)})


def test_a_key_with_nothing_to_act_on_is_refused(box: str) -> None:
    sim = Simulation(box, world=None)
    with pytest.raises(ValueError, match="nothing in this run"):
        Randomizer({"motor_resistance": (0.9, 1.1)}, sim.parameters())
    with pytest.raises(ValueError, match="nothing in this run"):
        Randomizer({"mass": {"bodies": "wheel*", "scale": (0.9, 1.1)}}, sim.parameters())


def test_parameters_list_the_files_values_and_unknown_ones_are_refused(box: str) -> None:
    sim = Simulation(box, world=None)
    parameters = sim.parameters()
    assert parameters["body/box/mass"] == pytest.approx(2.0)
    assert parameters["body/box/inertia_xx"] == pytest.approx(0.0133)
    assert parameters["start/x"] == 0.0
    state = sim.save_state()
    with pytest.raises(ValueError, match="not a parameter"):
        sim.reset_to(state, {"colour": 1.0})
    with pytest.raises(ValueError, match="no such body"):
        sim.reset_to(state, {"body/wheel/mass": 1.0})
    with pytest.raises(ValueError, match="positive"):
        sim.reset_to(state, {"body/box/mass": -1.0})
    # A refused reset leaves the files' values in place.
    assert sim.parameters()["body/box/mass"] == pytest.approx(2.0)


# --- Each parameter against a closed form ---------------------------------------------------


def test_a_heavier_body_falls_the_same(box: str) -> None:
    """In vacuum z(t) = z0 - g t² / 2 whatever the mass."""
    sim = Simulation(box, world=None)
    state = sim.save_state()
    heights = []
    for mass in (2.0, 8.0):
        sim.reset_to(state, {"body/box/mass": mass, "start/z": 5.0})
        z0 = sim.read("body/box/pose")[2]
        sim.step_until(0.5)
        heights.append(z0 - sim.read("body/box/pose")[2])
    assert heights[0] == pytest.approx(G * 0.5**2 / 2.0, rel=0.01)
    assert heights[1] == pytest.approx(heights[0], rel=1e-6)


def test_a_heavier_quadcopter_needs_more_thrust() -> None:
    """At hover the rotors' thrust carries the weight: sum T = M g."""
    sim = quad()
    state = sim.save_state()
    masses = {b: sim.parameters()[f"body/{b}/mass"] for b in sim.bodies()}
    thrusts = []
    for scale in (1.0, 1.3):
        sim.reset_to(state, {"body/base_link/mass": masses["base_link"] * scale})
        total = sum(masses.values()) + masses["base_link"] * (scale - 1.0)
        thrust = sum(mean_of(sim, f"rotor/rotor_{i}_joint/thrust", 6.0)[0] for i in range(1, 5))
        assert thrust == pytest.approx(total * G, rel=0.03)
        thrusts.append(thrust)
    assert thrusts[1] > thrusts[0] * 1.2


def test_higher_friction_slides_less(box: str) -> None:
    """A box pushed by F for T on the ground: a = F/m - mu g while pushed, then -mu g, so it
    slides a T² / 2 + (a T)² / (2 mu g)."""
    sim = Simulation(box, world=None, pushes=True)
    state = sim.save_state()
    m, force, push = 2.0, 30.0, 0.5
    slid = []
    for mu in (0.3, 0.6):
        sim.reset_to(state, {"body/box/friction": mu, "world/friction": mu})
        sim.set_pushes([("box", 0.2, push, force, 0.0, 0.0, 0.0, 0.0, 0.0)])
        x0 = sim.read("body/box/pose")[0]
        sim.step_until(3.0)
        x = sim.read("body/box/pose")[0] - x0
        a = force / m - mu * G
        expected = a * push**2 / 2.0 + (a * push) ** 2 / (2.0 * mu * G)
        assert x == pytest.approx(expected, rel=0.08)
        slid.append(x)
    assert slid[1] < slid[0] / 2.0


def test_a_push_gives_its_impulse_and_a_torque_turns_by_the_inertia(box: str) -> None:
    """Without gravity: dv = F T / m, and dw = tau T / I about a principal axis."""
    sim = Simulation(box, world=None, pushes=True)
    state = sim.save_state()
    for inertia in (0.0133, 0.0266):
        sim.reset_to(
            state,
            {"site/gravity": 0.0, "start/z": 2.0, "body/box/inertia_zz": inertia},
        )
        sim.set_pushes([("", 0.1, 0.2, 4.0, 0.0, 0.0, 0.0, 0.0, 0.05)])
        sim.step_until(0.5)
        twist = sim.read("body/box/twist")
        assert twist[0] == pytest.approx(4.0 * 0.2 / 2.0, rel=0.01)
        assert twist[5] == pytest.approx(0.05 * 0.2 / inertia, rel=0.01)


def test_a_centre_of_mass_past_the_edge_tips_the_box(box: str) -> None:
    """A box stands while its centre of mass is over its 0.2 m base, and tips past the edge."""
    sim = Simulation(box, world=None)
    state = sim.save_state()
    for offset, tips in ((0.05, False), (0.15, True)):
        sim.reset_to(state, {"body/box/com_x": offset})
        sim.step_until(2.0)
        w = sim.read("body/box/pose")[3]
        tilt = 2.0 * math.degrees(math.acos(min(1.0, abs(w))))
        assert (tilt > 30.0) == tips, (offset, tilt)


def test_joint_damping_and_friction_act_as_written(slider: str) -> None:
    """A pushed cart on a damped slide: v(t) = F/c (1 - exp(-c t / m)); with dry friction f,
    a = (F - f) / m, and none at all when f > F."""
    sim = Simulation(slider, world=None, pushes=["cart"])
    state = sim.save_state()
    m, force, t = 2.0, 4.0, 1.0
    for damping in (1.0, 4.0):
        sim.reset_to(state, {"joint/slide/damping": damping, "site/gravity": 0.0})
        sim.set_pushes([("cart", 0.0, 10.0, force, 0.0, 0.0, 0.0, 0.0, 0.0)])
        sim.step_until(t)
        expected = force / damping * (1.0 - math.exp(-damping * t / m))
        assert sim.read("joint/slide/velocity")[0] == pytest.approx(expected, rel=0.02)
    for friction, expected in ((1.0, (force - 1.0) / m * t), (6.0, 0.0)):
        sim.reset_to(
            state,
            {"joint/slide/damping": 0.0, "joint/slide/friction": friction, "site/gravity": 0.0},
        )
        sim.set_pushes([("cart", 0.0, 10.0, force, 0.0, 0.0, 0.0, 0.0, 0.0)])
        sim.step_until(t)
        assert sim.read("joint/slide/velocity")[0] == pytest.approx(expected, rel=0.03, abs=0.01)


def test_the_start_pose_moves_and_turns_the_robot(box: str) -> None:
    sim = Simulation(box, world=None)
    state = sim.save_state()
    sim.reset_to(state, {"start/x": 3.0, "start/y": -2.0, "start/yaw": math.pi / 2})
    pose = sim.read("body/box/pose")
    assert pose[0:2] == pytest.approx([3.0, -2.0], abs=1e-9)
    assert pose[3] == pytest.approx(math.cos(math.pi / 4), abs=1e-9)
    assert pose[6] == pytest.approx(math.sin(math.pi / 4), abs=1e-9)
    sim.step_until(0.5)
    with pytest.raises(ValueError, match="t = 0"):
        sim.reset_to(sim.save_state(), {"start/x": 1.0})


def test_motor_torque_constant_and_resistance() -> None:
    """A motor's torque is Kt i_q, and its copper loss grows with R at the same current."""
    sim = quad()
    state = sim.save_state()
    parameters = sim.parameters()
    motor = "/quad_x500/base_link/rotor_1_motor"
    kt = parameters[f"{motor}.elec:torqueConstant"]
    resistance = parameters[f"{motor}.elec:windingResistance"]
    ratios = []
    for scale in (1.0, 1.25):
        sim.reset_to(
            state,
            {
                f"{motor}.elec:torqueConstant": kt * scale,
                f"{motor}.elec:windingResistance": resistance * 2 * scale,
            },
        )
        sim.step_until(4.0)
        torque = sim.read("joint/rotor_1_joint/torque")[0]
        current = sim.read("motor/rotor_1_motor/current_q")[0]
        loss = sim.read("motor/rotor_1_motor/loss_power")[0]
        ratios.append((torque / current, loss / current**2))
    assert ratios[1][0] / ratios[0][0] == pytest.approx(1.25, rel=0.02)
    assert ratios[1][1] / ratios[0][1] == pytest.approx(1.25, rel=0.05)


def test_a_motor_current_limit_holds_the_current() -> None:
    sim = quad()
    state = sim.save_state()
    motor = "/quad_x500/base_link/rotor_1_motor.elec:maxCurrent"
    sim.reset_to(state, {motor: 2.0})
    peak = 0.0
    for k in range(1, 101):
        sim.step_until(k * 0.02)
        peak = max(peak, abs(sim.read("motor/rotor_1_motor/current_q")[0]))
    assert peak <= 2.0 * 1.01


def test_battery_capacity_and_resistance() -> None:
    """The state of charge falls by the charge drawn over the capacity; the terminal voltage
    falls by R0 I more when R0 is larger."""
    sim = quad()
    state = sim.save_state()
    parameters = sim.parameters()
    battery = "/quad_x500/power/battery"
    capacity = parameters[f"{battery}.elec:battery:capacity"]
    r0 = parameters[f"{battery}.elec:battery:r0"]
    drops = []
    for scale in (1.0, 2.0):
        sim.reset_to(state, {f"{battery}.elec:battery:capacity": capacity * scale})
        charge, dt = 0.0, 0.005
        for k in range(1, 601):
            sim.step_until(k * dt)
            charge += sim.read("battery/battery/current")[0] * dt
        drop = 1.0 - sim.read("battery/battery/soc")[0]
        assert drop == pytest.approx(charge / (capacity * scale), rel=0.03)
        drops.append(drop)
    assert drops[1] == pytest.approx(drops[0] / 2.0, rel=0.03)
    # R0 is per cell; the sample pack is 4S1P (quad_x500.usda elec:battery:seriesCells = 4).
    # Twice R0 is about as far as it goes: five times trips the cut-off voltage at take-off.
    cells = 4
    volts = []
    for scale in (1.0, 2.0):
        sim.reset_to(state, {f"{battery}.elec:battery:r0": r0 * scale})
        sim.step_until(3.0)
        volts.append(
            (
                sim.read("battery/battery/terminal_voltage")[0],
                sim.read("battery/battery/current")[0],
            )
        )
    (v1, i1), (v2, i2) = volts
    assert v1 - v2 == pytest.approx(cells * r0 * (2.0 * i2 - i1), rel=0.05)


def test_rotor_coefficients_scale_thrust_and_torque() -> None:
    """T = C_T rho n² D⁴ and Q = C_P rho n² D⁵ / (2 pi): at one speed both scale with their
    coefficient, so T / w² and Q / w² do."""
    sim = quad()
    state = sim.save_state()
    parameters = sim.parameters()
    rotor = "/rotor_1_joint.aero:rotor:"
    ratios = []
    for scale in (1.0, 1.2):
        sim.reset_to(
            state,
            {
                f"{rotor}thrustCoeff": parameters[f"{rotor}thrustCoeff"] * scale,
                f"{rotor}powerCoeff": parameters[f"{rotor}powerCoeff"] * scale,
            },
        )
        sim.step_until(4.0)
        w = sim.read("joint/rotor_1_joint/velocity")[0]
        ratios.append(
            (
                sim.read("rotor/rotor_1_joint/thrust")[0] / w**2,
                abs(sim.read("rotor/rotor_1_joint/aero_torque")[0]) / w**2,
            )
        )
    assert ratios[1][0] / ratios[0][0] == pytest.approx(1.2, rel=0.01)
    assert ratios[1][1] / ratios[0][1] == pytest.approx(1.2, rel=0.01)


def amr() -> Simulation:
    return Simulation(lagrange.sample_robot(), world=None, joint_control=True)  # held still


def accel_samples(sim: Simulation, signal: str, until: float, count: int) -> np.ndarray:
    out = []
    for k in range(count):
        sim.step_until(until + k * 0.01)  # the IMU samples at 100 Hz or faster
        out.append(sim.read(signal))
    return np.asarray(out)


def test_sensor_noise_scale_and_bias() -> None:
    """An IMU at rest reads g plus noise of its sigma, plus its bias of the mean drawn (of either
    sign per axis)."""
    sim = amr()
    state = sim.save_state()
    sigma = sim.parameters()["sensor/imu/accel_noise"]
    spreads = []
    for scale in (1.0, 5.0):
        sim.reset_to(state, {"sensor/imu/accel_noise": sigma * scale})
        samples = accel_samples(sim, "sensor/imu/specific_force", 1.0, 400)
        spreads.append(np.std(samples[:, 0]))
    assert spreads[0] == pytest.approx(sigma, rel=0.2)
    assert spreads[1] / spreads[0] == pytest.approx(5.0, rel=0.2)
    sim.reset_to(state, {"sensor/imu/accel_bias_mean": 0.5})
    samples = accel_samples(sim, "sensor/imu/specific_force", 1.0, 200)
    assert abs(np.mean(samples[:, 0])) == pytest.approx(0.5, abs=0.02)
    assert abs(np.mean(samples[:, 1])) == pytest.approx(0.5, abs=0.02)


def test_sensor_latency_delays_its_samples() -> None:
    sim = amr()
    state = sim.save_state()
    sim.reset_to(state, {"sensor/imu/latency": 0.05})
    sim.step_until(1.0)
    stamp = sim.read("sensor/imu/delayed/stamp")[0]
    assert sim.time - stamp == pytest.approx(0.05, abs=0.011)
    sim.reset_to(state, {})  # back to none: the delay line is gone with it
    assert "sensor/imu/delayed/stamp" not in {s["name"] for s in sim.signals()}


# --- Seeds and episodes ---------------------------------------------------------------------

SPEC = {
    "mass": {"bodies": "*", "scale": (0.7, 1.3)},
    "friction": (0.4, 1.2),
    "joint_damping": {"joints": "*_joint", "value": (0.0, 2.0)},
    "gravity": (9.0, 10.5),
}


def rollout(env: LagrangeEnv, seed: int | None) -> tuple[np.ndarray, dict]:
    _, info = env.reset(seed=seed)
    out = []
    for _ in range(5):
        observation, *_ = env.step(np.full(len(env.joint_names), 0.3))
        out.append(observation)
    return np.stack(out), info


def test_the_same_seed_draws_the_same_and_another_does_not() -> None:
    env = LagrangeEnv(arm(), mode="effort", action_scale=20.0, randomize=SPEC)
    first, info = rollout(env, 7)
    again, info_again = rollout(env, 7)
    np.testing.assert_array_equal(first, again)
    assert info["randomization"] == info_again["randomization"]
    other, info_other = rollout(env, 8)
    assert info["randomization"] != info_other["randomization"]
    assert not np.array_equal(first, other)
    assert 9.0 <= info["site"]["gravity"] <= 10.5
    masses = {k: v for k, v in info["randomization"].items() if k.endswith("/mass")}
    assert len(masses) == len(env.sim.bodies())


def test_no_leakage_between_episodes() -> None:
    """An episode with the files' values after randomised ones is the plain run, bit for bit."""
    plain = LagrangeEnv(arm(), mode="effort", action_scale=20.0)
    reference, _ = rollout(plain, 1)
    env = LagrangeEnv(arm(), mode="effort", action_scale=20.0, randomize=SPEC)
    rollout(env, 3)
    rollout(env, None)
    _, info = env.reset(seed=1, options={"randomize": False})
    assert info["randomization"] == {}
    out = []
    for _ in range(5):
        observation, *_ = env.step(np.full(len(env.joint_names), 0.3))
        out.append(observation)
    np.testing.assert_array_equal(np.stack(out), reference)
    assert env.sim.parameters() == plain.sim.parameters()


def test_pushes_are_drawn_in_the_episode() -> None:
    env = LagrangeEnv(
        arm(),
        randomize={"push": {"force": (5.0, 10.0), "count": (2, 3), "duration": 0.05}},
        episode_seconds=1.0,
    )
    _, info = env.reset(seed=2)
    pushes = info["randomization"]["push"]
    assert 2 <= len(pushes) <= 3
    for push in pushes:
        assert 0.0 <= push["start"] <= 1.0 - 0.05
        assert 5.0 <= np.linalg.norm(push["force"]) <= 10.0
        assert push["force"][2] == 0.0


def test_parallel_environments_draw_per_seed() -> None:
    with ParallelEnvs(arm(), 2, randomize={"mass": (0.5, 1.5)}) as envs:
        envs.reset(seed=4)
        first = [info["randomization"] for info in envs.infos]
        envs.reset(seed=4)
        assert [info["randomization"] for info in envs.infos] == first
        assert first[0] != first[1]
    with pytest.raises(ValueError, match="no key"):
        ParallelEnvs(arm(), 2, randomize={"mas": (0.5, 1.5)})
