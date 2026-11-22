"""The task library (lagrange.tasks): spaces, reward signs, ends, the baselines, and Gymnasium."""

from __future__ import annotations

import functools
import math

import numpy as np
import pytest
from lagrange.envs import ParallelEnvs, VectorEnv
from lagrange.tasks import TASKS, AmrGoToGoal, NumpyPolicy, QuadHover, QuadWaypoint, make


@functools.cache
def hover() -> QuadHover:
    return QuadHover()


@functools.cache
def waypoint() -> QuadWaypoint:
    return QuadWaypoint()


@functools.cache
def amr() -> AmrGoToGoal:
    return AmrGoToGoal(lidar=True)


def episode(env, act) -> tuple[float, int, bool, bool, dict]:
    """Runs one episode with act(env) -> step result; its return, steps and how it ended."""
    total, steps = 0.0, 0
    while True:
        _observation, reward, terminated, truncated, info = act(env)
        total += reward
        steps += 1
        if terminated or truncated:
            return total, steps, terminated, truncated, info


@pytest.mark.parametrize(
    ("task", "observed", "acted"),
    [(hover, 18, 4), (waypoint, 21, 4), (amr, 25, 2)],
)
def test_spaces_match_what_steps_give(task, observed: int, acted: int) -> None:
    env = task()
    observation, _info = env.reset(seed=1)
    assert env.observation_space.shape == (observed,) == observation.shape
    assert env.action_space.shape == (acted,)
    assert np.all(env.action_space.low == -1.0) and np.all(env.action_space.high == 1.0)
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert observation.shape == (observed,)
    assert np.all(np.isfinite(observation))
    assert isinstance(reward, float)
    assert isinstance(terminated, bool) and isinstance(truncated, bool)
    assert "success" in info and "distance" in info
    with pytest.raises(ValueError, match="actions expected"):
        env.step(np.zeros(acted + 1))


def test_the_same_seed_draws_the_same_task_and_start() -> None:
    env = hover()
    first, _ = env.reset(seed=7)
    target = env.target.copy()
    again, _ = env.reset(seed=7)
    np.testing.assert_array_equal(first, again)
    np.testing.assert_array_equal(target, env.target)
    env.reset(seed=8)
    assert not np.array_equal(target, env.target)


def test_hover_rewards_the_target_and_punishes_a_crash() -> None:
    env = hover()
    # The built-in controller flies to the target: positive rewards, near 1 there, success.
    total, steps, terminated, truncated, info = episode(env, QuadHover.step_builtin)
    assert truncated and not terminated
    assert steps == round(env.episode_seconds / env.step_seconds)
    assert info["success"] and info["distance"] < 0.2
    assert total > 0.6 * steps
    # Thrust at its lowest: it falls, and the crash ends the episode with the penalty.
    env.reset(seed=2)
    total, _steps, terminated, _truncated, info = episode(
        env, lambda e: e.step(np.array([-1.0, 0.0, 0.0, 0.0]))
    )
    assert terminated and not info["success"]
    assert total < 0.0
    # Each step's reward falls as the error grows (the tanh term).
    env.reset(seed=3, options={"target": env.home})
    near = env.step(np.zeros(4))[1]
    env.reset(seed=3, options={"target": env.home + np.array([0.0, 0.0, 1.5])})
    far = env.step(np.zeros(4))[1]
    assert near > far


def test_waypoints_are_reached_in_turn_with_bonuses() -> None:
    env = waypoint()
    env.reset(seed=4)
    assert len(env.points) == 4
    steps = np.linalg.norm(np.diff(np.vstack([env.home, env.points]), axis=0), axis=1)
    assert np.all(steps <= 2.5 + 1e-9)
    total, _steps, terminated, _truncated, info = episode(env, QuadWaypoint.step_builtin)
    assert terminated and info["success"] and info["reached"] == 4
    assert total > 4 * 5.0  # the four bonuses, and progress
    # Random flying does not get there.
    env.reset(seed=4)
    _total, _steps, _terminated, _truncated, info = episode(
        env, lambda e: e.step(e.action_space.sample())
    )
    assert not info["success"]


def test_amr_reaches_its_goal_and_a_wall_ends_it() -> None:
    env = amr()
    observation, _ = env.reset(seed=5)
    assert 1.0 <= observation[2] <= env.max_distance  # the goal's distance
    assert np.all((observation[9:] >= 0.0) & (observation[9:] <= 1.0))  # the scan
    total, _steps, terminated, _truncated, info = episode(env, AmrGoToGoal.step_builtin)
    assert terminated and info["success"] and total > 10.0
    # Straight towards the rack on its left (y = 3.0 m): progress first, then the collision.
    env.reset(seed=5, options={"goal": np.array([0.0, 4.0, 0.0])})
    turned = 0
    while env._pose()[1] < math.pi / 2 - 0.05 and turned < 50:
        env.step(np.array([0.0, 0.5]))
        turned += 1
    total, _steps, terminated, _truncated, info = episode(env, lambda e: e.step(np.array([1, 0])))
    assert terminated and info.get("collision") and not info["success"]
    assert total < 0.0


def test_tasks_step_in_parallel_workers() -> None:
    with ParallelEnvs(functools.partial(make, "QuadHover"), 2) as envs:
        observations = envs.reset(seed=1)
        assert observations.shape == (2, 18)
        observations, rewards, ended = envs.step(np.zeros((2, 4)))
        assert observations.shape == (2, 18) and rewards.shape == (2,) and not ended.any()
    envs = VectorEnv(functools.partial(make, "AmrGoToGoal"), 2)
    try:
        observations, _infos = envs.reset(seed=1)
        assert observations.shape == (2, 9)
        _o, rewards, terminations, truncations, _i = envs.step(np.zeros((2, 2)))
        assert rewards.shape == terminations.shape == truncations.shape == (2,)
    finally:
        envs.close()


def test_make_knows_every_task_and_no_other() -> None:
    assert set(TASKS) == {"QuadHover", "QuadWaypoint", "AmrGoToGoal"}
    with pytest.raises(ValueError, match="no task"):
        make("Juggle")


@pytest.mark.parametrize("task", [hover, waypoint, amr])
def test_gymnasium_check_env(task) -> None:
    pytest.importorskip("gymnasium")
    from gymnasium.utils.env_checker import check_env

    check_env(task(), skip_render_check=True)


def test_command_flight_is_only_for_a_flown_multicopter(tmp_path) -> None:
    with pytest.raises(ValueError, match="no built-in flight controller"):
        amr().sim.command_flight("hold")
    env = hover()
    with pytest.raises(ValueError, match="four values"):
        env.sim.command_flight("rates", [9.81])
    with pytest.raises(ValueError, match="flight mode"):
        env.sim.command_flight("sideways", [0.0, 0.0, 0.0, 0.0])


def test_a_policy_runs_from_its_arrays_and_a_flight_records(tmp_path) -> None:
    # A one-layer "policy" of plain arrays, as ppo.py exports: thrust up a little, no rates.
    path = tmp_path / "policy.npz"
    np.savez(
        path,
        layers=np.array(1),
        w0=np.zeros((4, 18), dtype=np.float32),
        b0=np.array([0.2, 0.0, 0.0, 0.0], dtype=np.float32),
        obs_mean=np.zeros(18),
        obs_var=np.ones(18),
        clip_obs=np.array(10.0),
        epsilon=np.array(1e-8),
    )
    policy = NumpyPolicy(path)
    np.testing.assert_allclose(policy(np.ones(18)), [0.2, 0.0, 0.0, 0.0], atol=1e-6)
    env = QuadHover(kick_seconds=0.0)
    observation, _ = env.reset(seed=1, options={"target": env.home})
    recording = tmp_path / "flight.mcap"
    env.sim.record(str(recording))
    for _ in range(10):
        observation, *_ = env.step(policy(observation))
    env.sim.stop_recording()
    assert observation[2] < 0.0  # 1.1 g of thrust: above the target now
    assert recording.stat().st_size > 0
