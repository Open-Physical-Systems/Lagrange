"""The Gymnasium interface and parallel environments."""

from __future__ import annotations

import multiprocessing
import os
from pathlib import Path
from typing import Any

import lagrange
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from lagrange.envs import Box, LagrangeEnv, ParallelEnvs, VectorEnv, WorkerError, profile_path


def arm() -> str:
    return get_package_share_directory("lagrange_assets") + "/robots/arm6/arm6.urdf"


def make_arm() -> LagrangeEnv:
    return LagrangeEnv(arm(), mode="position", action_scale=0.5, episode_seconds=0.2)


def rollout(env: LagrangeEnv, seed: int) -> np.ndarray:
    env.reset(seed=seed)
    out = []
    for _ in range(5):
        observation, *_ = env.step(env.action_space.sample())
        out.append(observation)
    return np.stack(out)


def test_one_step_has_the_gymnasium_shape() -> None:
    env = make_arm()
    observation, _info = env.reset(seed=1)
    assert env.observation_space.shape == observation.shape
    assert observation.shape == (2 * len(env.joint_names),)
    result = env.step(np.zeros(len(env.joint_names)))
    assert len(result) == 5
    assert isinstance(result[1], float)


def test_the_same_seed_repeats_and_another_does_not() -> None:
    env = make_arm()
    first = rollout(env, 3)
    np.testing.assert_array_equal(first, rollout(env, 3))
    assert not np.array_equal(first, rollout(env, 4))


def test_an_episode_ends_by_time() -> None:
    env = make_arm()
    env.reset(seed=1)
    ended = [env.step(np.zeros(len(env.joint_names)))[3] for _ in range(10)]
    assert ended == [False] * 9 + [True]


def test_randomize_draws_the_site_at_each_reset() -> None:
    env = LagrangeEnv(arm(), randomize={"gravity": (1.0, 2.0)})
    _, one = env.reset(seed=1)
    _, two = env.reset()
    assert 1.0 <= one["site"]["gravity"] <= 2.0
    assert one["site"]["gravity"] != two["site"]["gravity"]


def test_a_bad_mode_is_refused() -> None:
    with pytest.raises(ValueError, match="mode"):
        LagrangeEnv(arm(), mode="torque")


def test_parallel_environments_step_together() -> None:
    envs = ParallelEnvs(make_arm, 2)
    try:
        observations = envs.reset(seed=1)
        assert observations.shape[0] == 2
        actions = np.zeros((2, observations.shape[1] // 2))
        ended = [envs.step(actions)[2] for _ in range(10)]
        assert ended[-1].all()
    finally:
        envs.close()


# --- Parallel environments against one process ---------------------------------------------

STEPS = 25  # two and a half episodes of 10 steps: two autoresets


def actions_for(count: int, joints: int) -> np.ndarray:
    """The same random actions for every run, (STEPS, count, joints), some beyond -1..1, in
    float32 as action_space.sample() gives them."""
    rng = np.random.default_rng(7)
    return rng.uniform(-1.5, 1.5, (STEPS, count, joints)).astype(np.float32)


def alone(seed: int, actions: np.ndarray) -> dict[str, list[Any]]:
    """One LagrangeEnv in this process, restarted when an episode ends, as the workers do."""
    env = make_arm()
    observation, _ = env.reset(seed=seed)
    out: dict[str, list[Any]] = {"first": [observation], "obs": [], "reward": [], "ended": []}
    out["final"] = []
    for action in actions:
        observation, reward, end, cut, _ = env.step(action)
        if end or cut:
            out["final"].append(observation)
            observation, _ = env.reset()
        out["obs"].append(observation)
        out["reward"].append(reward)
        out["ended"].append(end or cut)
    return out


@pytest.fixture(scope="module")
def reference() -> tuple[np.ndarray, list[dict[str, list[Any]]]]:
    joints = len(make_arm().joint_names)
    actions = actions_for(3, joints)
    return actions, [alone(10 + i, actions[:, i]) for i in range(3)]


@pytest.mark.parametrize("envs_per_worker", [1, 2])
def test_parallel_trajectories_are_those_of_one_process(
    reference: tuple[np.ndarray, list[dict[str, list[Any]]]], envs_per_worker: int
) -> None:
    actions, expected = reference
    with ParallelEnvs(make_arm, 3, envs_per_worker=envs_per_worker) as envs:
        first = envs.reset(seed=10)
        for i in range(3):
            np.testing.assert_array_equal(first[i], expected[i]["first"][0])
        finals: list[list[np.ndarray]] = [[], [], []]
        for t in range(STEPS):
            observations, rewards, ended = envs.step(actions[t])
            for i in range(3):
                np.testing.assert_array_equal(observations[i], expected[i]["obs"][t])
                assert rewards[i] == expected[i]["reward"][t]
                assert ended[i] == expected[i]["ended"][t]
                if ended[i]:
                    assert envs.truncated[i] and not envs.terminated[i]
                    finals[i].append(envs.final_observations[i].copy())
        for i in range(3):
            assert len(finals[i]) == 2
            np.testing.assert_array_equal(np.stack(finals[i]), np.stack(expected[i]["final"]))


def test_send_and_recv_step_some_while_others_run(
    reference: tuple[np.ndarray, list[dict[str, list[Any]]]],
) -> None:
    """Each environment its own pace: still its own trajectory."""
    actions, expected = reference
    with ParallelEnvs(make_arm, 3) as envs:
        envs.reset(seed=10)
        steps = [0, 0, 0]
        envs.send(actions[0], [0, 1, 2])
        while min(steps) < STEPS:
            ids, observations, rewards, _terminated, truncated = envs.recv()
            assert len(ids) >= 1
            again = []
            for k, i in enumerate(ids.tolist()):
                t = steps[i]
                np.testing.assert_array_equal(observations[k], expected[i]["obs"][t])
                assert rewards[k] == expected[i]["reward"][t]
                assert truncated[k] == expected[i]["ended"][t]
                steps[i] += 1
                if steps[i] < STEPS:
                    again.append(i)
            if again:
                envs.send(np.stack([actions[steps[i], i] for i in again]), again)
        assert steps == [STEPS] * 3


def test_reset_takes_one_seed_each_or_a_list() -> None:
    with ParallelEnvs(make_arm, 2) as envs:
        joints = envs.action_space.shape[0]
        envs.reset(seed=[5, 5])
        observations, _, _ = envs.step(np.full((2, joints), 0.3))
        np.testing.assert_array_equal(observations[0], observations[1])
        with pytest.raises(ValueError, match="seeds"):
            envs.reset(seed=[1, 2, 3])
        # A reset in the middle of an episode starts it again, at the same first state.
        first = envs.reset(seed=1)
        envs.step(np.full((2, joints), 0.3))
        np.testing.assert_array_equal(envs.reset(seed=1), first)


def test_the_vector_interface_resets_in_the_same_step() -> None:
    envs = VectorEnv(make_arm, 2)
    try:
        observations, infos = envs.reset(seed=3)
        assert observations.shape == (2, *envs.single_observation_space.shape)
        assert envs.action_space.shape == (2, *envs.single_action_space.shape)
        assert infos["_site"].all()
        zero = np.zeros(envs.action_space.shape)
        for step in range(10):
            observations, rewards, terminations, truncations, infos = envs.step(zero)
            assert rewards.shape == (2,) and terminations.shape == (2,)
            if step < 9:
                assert not truncations.any() and "final_obs" not in infos
        assert truncations.all() and not terminations.any()
        assert infos["_final_obs"].all()
        assert infos["final_obs"][0].shape == envs.single_observation_space.shape
        # The next episode's first observation: the reset state, not the ended one.
        np.testing.assert_array_equal(observations, envs.reset(seed=3)[0])
        mode = envs.metadata["autoreset_mode"]
        assert getattr(mode, "value", mode) == "SameStep"
    finally:
        envs.close()
    assert envs.closed


# --- Failures and shutdown, on environments without a simulation ----------------------------


class Counter:
    """An environment whose observation counts its steps; it fails as told at a step."""

    def __init__(self, fail_at: int = -1, how: str = "raise") -> None:
        self.observation_space = Box(-np.inf, np.inf, (2,), np.float64)
        self.action_space = Box(-1.0, 1.0, (1,), np.float32)
        self.fail_at, self.how, self.steps = fail_at, how, 0

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        self.steps = 0
        return np.array([0.0, float(seed or 0)]), {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        self.steps += 1
        if self.steps == self.fail_at:
            if self.how == "exit":
                os._exit(3)
            raise ValueError("this environment fails here")
        return np.array([float(self.steps), float(action[0])]), 1.0, False, self.steps >= 4, {}


def make_counter() -> Counter:
    return Counter()


def make_raising() -> Counter:
    return Counter(fail_at=3)


def make_dying() -> Counter:
    return Counter(fail_at=3, how="exit")


def make_nothing() -> Counter:
    raise RuntimeError("cannot build this environment")


def gone(envs: ParallelEnvs) -> bool:
    """No worker process left: none running, none unreaped."""
    live = {p.pid for p in multiprocessing.active_children()}
    return all(p.exitcode is not None and p.pid not in live for p in envs.processes)


def test_close_leaves_no_process_and_no_shared_memory() -> None:
    envs = ParallelEnvs(make_counter, 5, envs_per_worker=2)
    assert len(envs.processes) == 3
    name = envs._pool._memory.name  # type: ignore[union-attr]
    envs.reset()
    envs.step(np.zeros((5, 1)))
    envs.close()
    envs.close()  # twice is harmless
    assert gone(envs)
    assert all(p.exitcode == 0 for p in envs.processes)  # each left by itself
    assert not Path("/dev/shm", name.lstrip("/")).exists()
    with pytest.raises(WorkerError, match="closed"):
        envs.step(np.zeros((5, 1)))


def test_close_while_stepping_stops_every_worker() -> None:
    envs = ParallelEnvs(make_counter, 3)
    envs.reset()
    envs.step_async(np.zeros((3, 1)))
    envs.close()
    assert gone(envs)


def test_an_environment_that_raises_closes_all_with_its_traceback() -> None:
    envs = ParallelEnvs(make_raising, 3)
    envs.reset()
    envs.step(np.zeros((3, 1)))
    envs.step(np.zeros((3, 1)))
    with pytest.raises(WorkerError, match="this environment fails here"):
        envs.step(np.zeros((3, 1)))
    assert envs.closed
    assert gone(envs)


def test_a_worker_that_dies_is_reported() -> None:
    envs = ParallelEnvs(make_dying, 2)
    envs.reset()
    envs.step(np.zeros((2, 1)))
    envs.step(np.zeros((2, 1)))
    with pytest.raises(WorkerError, match="died"):
        envs.step(np.zeros((2, 1)))
    assert gone(envs)


def test_an_environment_that_cannot_be_made_is_reported() -> None:
    with pytest.raises(WorkerError, match="cannot build this environment"):
        ParallelEnvs(make_nothing, 2)


@pytest.mark.parametrize("spin", [0.0, 0.001])
def test_the_counter_steps_and_restarts_in_the_worker(spin: float) -> None:
    with ParallelEnvs(make_counter, 2, spin_seconds=spin) as envs:
        first = envs.reset(seed=[4, 6])
        np.testing.assert_array_equal(first, [[0.0, 4.0], [0.0, 6.0]])
        for step in range(1, 5):
            observations, rewards, ended = envs.step(np.array([[0.5], [-0.5]]))
            assert rewards.tolist() == [1.0, 1.0]
            assert ended.tolist() == [step == 4] * 2
        np.testing.assert_array_equal(envs.final_observations, [[4.0, 0.5], [4.0, -0.5]])
        np.testing.assert_array_equal(observations, [[0.0, 0.0], [0.0, 0.0]])


def test_read_many_and_command_joints_are_read_and_command_joint_in_one_call() -> None:
    env = make_arm()
    sim = env.sim
    names = env._observed
    np.testing.assert_array_equal(
        sim.read_many(names), np.concatenate([sim.read(name) for name in names])
    )
    assert sim.command_joints(env.joint_names, "position", [0.1] * len(env.joint_names))
    assert not sim.command_joints(["no_such_joint"], "position", [0.1])
    with pytest.raises(KeyError):
        sim.read_many(["no/such/signal"])
    with pytest.raises(ValueError, match="one value per joint"):
        sim.command_joints(env.joint_names, "position", [0.1])


# --- Profile, overlay and world: what LagrangeEnv passes to Simulation --------------------------

POWER = ["bus/dc_48v/voltage", "battery/battery/soc", "battery/battery/current"]


def make_powered_quadruped() -> LagrangeEnv:
    return LagrangeEnv(
        lagrange.sample_quadruped(),
        profile="quadruped_trot_powered",
        overlay=lagrange.sample_quadruped_overlay(),
        mode="position",
        action_scale=0.5,
        signals=POWER,
        episode_seconds=0.2,
    )


def test_an_overlay_gives_the_quadruped_its_power_system() -> None:
    env = make_powered_quadruped()
    assert env.profile == lagrange.sample_profile("quadruped_trot_powered")
    assert env.overlay == lagrange.sample_quadruped_overlay()
    assert len(env.joint_names) == 12
    names = {signal["name"] for signal in env.sim.signals()}
    assert set(POWER) <= names
    assert {
        f"motor/{joint[: -len('_joint')]}_motor/current_q" for joint in env.joint_names
    } <= names
    observation, _ = env.reset(seed=0)
    assert observation.shape == (2 * 12 + len(POWER),)
    for _ in range(5):
        observation, *_ = env.step(env.action_space.sample())
    volts, soc, amps = observation[-3:]
    assert 44.8 < volts < 60.0  # between the battery's cutoff and the bus's overvoltage trip
    assert 0.0 < soc <= 1.0
    assert amps > 0.0  # the computer's 45 W at least
    # Without the overlay the robot has no power system to observe.
    bare = LagrangeEnv(lagrange.sample_quadruped(), profile="quadruped_trot")
    assert bare.overlay is None
    assert not set(POWER) & {signal["name"] for signal in bare.sim.signals()}
    with pytest.raises(KeyError):
        LagrangeEnv(lagrange.sample_quadruped(), signals=POWER)


def test_the_environment_steps_as_the_simulation_it_describes() -> None:
    """LagrangeEnv(profile=, overlay=) is Simulation(profile=, overlay=) with joint commands."""
    env = make_powered_quadruped()
    sim = lagrange.Simulation(
        lagrange.sample_quadruped(),
        profile=lagrange.sample_profile("quadruped_trot_powered"),
        overlay=lagrange.sample_quadruped_overlay(),
        joint_control=True,
    )
    env.reset(seed=0)
    rng = np.random.default_rng(3)
    for _ in range(5):
        action = rng.uniform(-1.0, 1.0, 12)
        observation, *_ = env.step(action)
        sim.command_joints(env.joint_names, "position", (action * 0.5).tolist())
        sim.step_until(sim.time + env.step_seconds)
        np.testing.assert_array_equal(observation, sim.read_many(env._observed))
    assert env.sim.state_hash() == sim.state_hash()


def test_a_profile_is_a_path_or_an_example_name(tmp_path: Path) -> None:
    assert profile_path(None) is None
    assert profile_path("quadruped_trot") == lagrange.sample_profile("quadruped_trot")
    own = tmp_path / "mine.yaml"
    own.write_text(Path(lagrange.sample_profile("quadruped_trot")).read_text())
    assert profile_path(own) == str(own)
    with pytest.raises(FileNotFoundError, match="no_such_profile"):
        profile_path("no_such_profile")
    with pytest.raises(FileNotFoundError):
        LagrangeEnv(arm(), profile=tmp_path / "missing.yaml")


def test_the_profile_turns_the_electrical_domain_off() -> None:
    """The same robot and overlay with electrical substituted by the ideal source: no bus."""
    env = LagrangeEnv(
        lagrange.sample_quadruped(),
        profile="quadruped_trot",
        overlay=lagrange.sample_quadruped_overlay(),
    )
    names = {signal["name"] for signal in env.sim.signals()}
    assert "bus/dc_48v/voltage" not in names
    env.reset(seed=0)
    env.step(np.zeros(12))


def test_parallel_environments_take_a_robot_file_with_its_options() -> None:
    """ParallelEnvs(robot, profile=, overlay=, env_options=) steps as LagrangeEnv does alone."""
    options: dict[str, Any] = {
        "mode": "position",
        "action_scale": 0.5,
        "signals": POWER,
        "episode_seconds": 0.2,
    }
    env = make_powered_quadruped()
    actions = actions_for(2, 12)[:6]
    expected = []
    for i in range(2):
        env.reset(seed=20 + i)
        expected.append([env.step(actions[t, i])[0] for t in range(len(actions))])
    with ParallelEnvs(
        lagrange.sample_quadruped(),
        2,
        profile="quadruped_trot_powered",
        overlay=lagrange.sample_quadruped_overlay(),
        env_options=options,
    ) as envs:
        assert envs.observation_space.shape == (2 * 12 + len(POWER),)
        envs.reset(seed=20)
        for t in range(len(actions)):
            observations, _, _ = envs.step(actions[t])
            for i in range(2):
                np.testing.assert_array_equal(observations[i], expected[i][t])
    vector = VectorEnv(
        lagrange.sample_quadruped(),
        1,
        world=lagrange.sample_world(),
        profile="quadruped_trot",
        env_options={"episode_seconds": 0.2},
    )
    try:
        assert vector.single_action_space.shape == (12,)
        vector.reset(seed=0)
        vector.step(np.zeros((1, 12)))
    finally:
        vector.close()


def test_options_for_a_robot_file_are_refused_with_make() -> None:
    with pytest.raises(ValueError, match="robot file"):
        ParallelEnvs(make_arm, 2, profile="quadruped_trot")
    with pytest.raises(ValueError, match="robot file"):
        VectorEnv(make_arm, 2, env_options={"mode": "position"})
    # A misspelt profile fails here, before any worker starts.
    with pytest.raises(FileNotFoundError, match="quadruped_trto"):
        ParallelEnvs(lagrange.sample_quadruped(), 2, profile="quadruped_trto")


def test_a_world_goes_to_the_simulation() -> None:
    bare = LagrangeEnv(lagrange.sample_quadruped())
    with_world = LagrangeEnv(lagrange.sample_quadruped(), lagrange.sample_world())
    assert with_world.sim.colliders() > bare.sim.colliders()
